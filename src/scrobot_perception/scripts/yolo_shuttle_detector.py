#!/usr/bin/env python3
"""YOLO + RGB-aligned depth shuttle detector for SCROBOT."""

from __future__ import annotations

import math
import os
import time
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

try:
    from ultralytics import YOLO
except ImportError as exc:
    raise ImportError(
        "Ultralytics is required by yolo_shuttle_detector. "
        "Install it in the ROS Python environment with: pip install ultralytics"
    ) from exc


class YoloShuttleDetector(Node):
    """Detect shuttlecocks in RGB and recover camera-relative 3D position.

    RGB and aligned-depth frames are synchronized. YOLO supplies the 2D bbox.
    A robust foreground-biased depth sample is taken from the inner bbox ROI,
    then deprojected with the color-camera intrinsics. The resulting 3D
    detections are published in the color optical frame.
    """

    def __init__(self) -> None:
        super().__init__("yolo_shuttle_detector")

        self.declare_parameter("model_path", "")
        self.declare_parameter(
            "device",
            "0",
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("imgsz", 960)
        self.declare_parameter("confidence_threshold", 0.10)
        self.declare_parameter("iou_threshold", 0.70)
        self.declare_parameter("max_detection_rate", 15.0)

        self.declare_parameter(
            "color_topic",
            "/camera/camera/color/image_raw",
        )
        self.declare_parameter(
            "aligned_depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/camera/camera/color/camera_info",
        )
        self.declare_parameter(
            "output_3d_topic",
            "/perception/shuttle_detections_3d",
        )
        self.declare_parameter(
            "output_2d_topic",
            "/perception/shuttle_detections_2d",
        )
        self.declare_parameter(
            "debug_image_topic",
            "/perception/shuttle_debug/image",
        )

        self.declare_parameter("class_id", "shuttle")
        self.declare_parameter("yolo_class_index", 0)

        # Camera-depth validity only. Mission collection eligibility is
        # evaluated later in base_link by shuttle_collection_filter.
        self.declare_parameter("min_depth_range", 0.20)
        self.declare_parameter("max_depth_range", 3.00)
        self.declare_parameter("depth_roi_scale", 0.40)
        self.declare_parameter("depth_percentile", 25.0)
        self.declare_parameter("min_valid_depth_pixels", 4)

        self.declare_parameter("bbox_size_x", 0.08)
        self.declare_parameter("bbox_size_y", 0.08)
        self.declare_parameter("bbox_size_z", 0.10)

        self.declare_parameter("sync_queue_size", 5)
        self.declare_parameter("sync_slop", 0.08)
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("stats_period", 5.0)

        self.bridge = CvBridge()

        model_path_param = str(self.get_parameter("model_path").value).strip()
        model_path_text = model_path_param or os.environ.get(
            "SCROBOT_YOLO_MODEL", ""
        ).strip()
        if not model_path_text:
            raise RuntimeError(
                "No YOLO model configured. Set ROS parameter 'model_path' or "
                "environment variable SCROBOT_YOLO_MODEL."
            )

        self.model_path = Path(model_path_text).expanduser().resolve()
        if not self.model_path.is_file():
            raise FileNotFoundError(
                f"YOLO model not found: {self.model_path}"
            )

        self.device = str(self.get_parameter("device").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.conf_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.max_detection_rate = float(
            self.get_parameter("max_detection_rate").value
        )

        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(
            self.get_parameter("aligned_depth_topic").value
        )
        self.camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )
        self.output_3d_topic = str(
            self.get_parameter("output_3d_topic").value
        )
        self.output_2d_topic = str(
            self.get_parameter("output_2d_topic").value
        )
        self.debug_image_topic = str(
            self.get_parameter("debug_image_topic").value
        )

        self.class_id = str(self.get_parameter("class_id").value)
        self.yolo_class_index = int(
            self.get_parameter("yolo_class_index").value
        )

        self.min_depth_range = float(
            self.get_parameter("min_depth_range").value
        )
        self.max_depth_range = float(
            self.get_parameter("max_depth_range").value
        )
        self.depth_roi_scale = float(
            self.get_parameter("depth_roi_scale").value
        )
        self.depth_percentile = float(
            self.get_parameter("depth_percentile").value
        )
        self.min_valid_depth_pixels = int(
            self.get_parameter("min_valid_depth_pixels").value
        )

        self.bbox_size = (
            float(self.get_parameter("bbox_size_x").value),
            float(self.get_parameter("bbox_size_y").value),
            float(self.get_parameter("bbox_size_z").value),
        )

        sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        sync_slop = float(self.get_parameter("sync_slop").value)
        self.publish_debug_image = bool(
            self.get_parameter("publish_debug_image").value
        )
        self.stats_period = float(self.get_parameter("stats_period").value)

        if self.imgsz <= 0:
            raise ValueError("imgsz must be > 0")
        if not 0.0 <= self.conf_threshold <= 1.0:
            raise ValueError("confidence_threshold must be in [0, 1]")
        if not 0.0 < self.iou_threshold <= 1.0:
            raise ValueError("iou_threshold must be in (0, 1]")
        if self.max_detection_rate <= 0.0:
            raise ValueError("max_detection_rate must be > 0")
        if (
            self.min_depth_range <= 0.0
            or self.max_depth_range <= self.min_depth_range
        ):
            raise ValueError(
                "invalid min_depth_range / max_depth_range"
            )
        if not 0.0 < self.depth_roi_scale <= 1.0:
            raise ValueError("depth_roi_scale must be in (0, 1]")
        if not 0.0 <= self.depth_percentile <= 100.0:
            raise ValueError("depth_percentile must be in [0, 100]")
        if self.min_valid_depth_pixels <= 0:
            raise ValueError("min_valid_depth_pixels must be > 0")

        self._min_period = 1.0 / self.max_detection_rate
        self._last_inference_wall = 0.0
        self._camera_info: CameraInfo | None = None

        self._frames_processed = 0
        self._detections_2d = 0
        self._detections_3d = 0
        self._depth_rejected = 0
        self._inference_ms_sum = 0.0
        self._last_stats_wall = time.perf_counter()
        self._last_camera_info_warning_wall = 0.0

        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(str(self.model_path))

        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )

        self.color_sub = message_filters.Subscriber(
            self,
            Image,
            self.color_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=sync_queue_size,
            slop=sync_slop,
        )
        self.sync.registerCallback(self._image_pair_callback)

        self.publisher_3d = self.create_publisher(
            Detection3DArray,
            self.output_3d_topic,
            qos_profile_sensor_data,
        )
        self.publisher_2d = self.create_publisher(
            Detection2DArray,
            self.output_2d_topic,
            qos_profile_sensor_data,
        )
        self.debug_publisher = None
        if self.publish_debug_image:
            self.debug_publisher = self.create_publisher(
                Image,
                self.debug_image_topic,
                qos_profile_sensor_data,
            )

        self.get_logger().info(
            "YOLO shuttle detector ready: "
            f"imgsz={self.imgsz}, conf={self.conf_threshold:.2f}, "
            f"device={self.device}, rate<={self.max_detection_rate:.1f} Hz, "
            f"camera_depth_valid={self.min_depth_range:.2f}-"
            f"{self.max_depth_range:.2f} m"
        )
        self.get_logger().info(
            f"RGB={self.color_topic}, aligned_depth={self.depth_topic}, "
            f"CameraInfo={self.camera_info_topic}"
        )
        self.get_logger().info(
            f"3D output={self.output_3d_topic}"
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if (
            msg.width <= 0
            or msg.height <= 0
            or len(msg.k) < 9
            or msg.k[0] <= 0.0
            or msg.k[4] <= 0.0
        ):
            return
        self._camera_info = msg

    def _image_pair_callback(
        self,
        color_msg: Image,
        depth_msg: Image,
    ) -> None:
        now_wall = time.perf_counter()
        if now_wall - self._last_inference_wall < self._min_period:
            return
        self._last_inference_wall = now_wall

        if self._camera_info is None:
            if now_wall - self._last_camera_info_warning_wall >= 2.0:
                self._last_camera_info_warning_wall = now_wall
                self.get_logger().warn("Waiting for valid color CameraInfo...")
            return

        try:
            color = self.bridge.imgmsg_to_cv2(
                color_msg,
                desired_encoding="bgr8",
            )
            depth_m = self._depth_to_meters(depth_msg)
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        t0 = time.perf_counter()
        results = self.model.predict(
            source=color,
            imgsz=self.imgsz,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            classes=[self.yolo_class_index],
            device=self.device,
            verbose=False,
        )
        inference_ms = (time.perf_counter() - t0) * 1000.0

        output_2d = Detection2DArray()
        output_2d.header = color_msg.header

        output_3d = Detection3DArray()
        output_3d.header.stamp = color_msg.header.stamp
        output_3d.header.frame_id = (
            self._camera_info.header.frame_id.strip()
            or color_msg.header.frame_id.strip()
            or "camera_color_optical_frame"
        )

        debug_image = color.copy() if self.publish_debug_image else None

        boxes = results[0].boxes if results else None
        if boxes is not None:
            xyxy = boxes.xyxy.detach().cpu().numpy()
            confs = boxes.conf.detach().cpu().numpy()

            for index, (coords, score) in enumerate(zip(xyxy, confs)):
                x0, y0, x1, y1 = [float(v) for v in coords]
                det2d = self._make_detection_2d(
                    color_msg,
                    x0,
                    y0,
                    x1,
                    y1,
                    float(score),
                    index,
                )
                output_2d.detections.append(det2d)
                self._detections_2d += 1

                depth = self._sample_depth(
                    depth_m,
                    x0,
                    y0,
                    x1,
                    y1,
                )
                if depth is None:
                    self._depth_rejected += 1
                    self._draw_debug(
                        debug_image,
                        x0,
                        y0,
                        x1,
                        y1,
                        float(score),
                        None,
                    )
                    continue

                u = 0.5 * (x0 + x1)
                v = 0.5 * (y0 + y1)
                point = self._deproject(u, v, depth)
                det3d = self._make_detection_3d(
                    color_msg,
                    output_3d.header.frame_id,
                    point,
                    float(score),
                    index,
                )
                output_3d.detections.append(det3d)
                self._detections_3d += 1

                self._draw_debug(
                    debug_image,
                    x0,
                    y0,
                    x1,
                    y1,
                    float(score),
                    depth,
                )

        self.publisher_2d.publish(output_2d)
        self.publisher_3d.publish(output_3d)

        if debug_image is not None and self.debug_publisher is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(
                debug_image,
                encoding="bgr8",
            )
            debug_msg.header = color_msg.header
            self.debug_publisher.publish(debug_msg)

        self._frames_processed += 1
        self._inference_ms_sum += inference_ms
        self._maybe_log_stats(now_wall)

    def _depth_to_meters(self, msg: Image) -> np.ndarray:
        if msg.encoding in ("16UC1", "mono16"):
            raw = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )
            return raw.astype(np.float32) * 0.001

        if msg.encoding == "32FC1":
            raw = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )
            return raw.astype(np.float32)

        raise ValueError(
            f"Unsupported aligned-depth encoding: {msg.encoding}"
        )

    def _sample_depth(
        self,
        depth_m: np.ndarray,
        x0: float,
        y0: float,
        x1: float,
        y1: float,
    ) -> float | None:
        height, width = depth_m.shape[:2]

        cx = 0.5 * (x0 + x1)
        cy = 0.5 * (y0 + y1)
        half_w = max(1.0, 0.5 * (x1 - x0) * self.depth_roi_scale)
        half_h = max(1.0, 0.5 * (y1 - y0) * self.depth_roi_scale)

        ix0 = max(0, int(math.floor(cx - half_w)))
        iy0 = max(0, int(math.floor(cy - half_h)))
        ix1 = min(width, int(math.ceil(cx + half_w)) + 1)
        iy1 = min(height, int(math.ceil(cy + half_h)) + 1)

        if ix1 <= ix0 or iy1 <= iy0:
            return None

        roi = depth_m[iy0:iy1, ix0:ix1]
        valid = roi[
            np.isfinite(roi)
            & (roi >= self.min_depth_range)
            & (roi <= self.max_depth_range)
        ]

        if valid.size < self.min_valid_depth_pixels:
            return None

        return float(np.percentile(valid, self.depth_percentile))

    def _deproject(
        self,
        u: float,
        v: float,
        depth: float,
    ) -> tuple[float, float, float]:
        info = self._camera_info
        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])

        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        return x, y, z

    def _make_detection_2d(
        self,
        color_msg: Image,
        x0: float,
        y0: float,
        x1: float,
        y1: float,
        score: float,
        index: int,
    ) -> Detection2D:
        detection = Detection2D()
        detection.header = color_msg.header
        detection.id = str(index)
        detection.bbox.center.position.x = 0.5 * (x0 + x1)
        detection.bbox.center.position.y = 0.5 * (y0 + y1)
        detection.bbox.center.theta = 0.0
        detection.bbox.size_x = max(0.0, x1 - x0)
        detection.bbox.size_y = max(0.0, y1 - y0)

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = self.class_id
        hypothesis.hypothesis.score = score
        detection.results.append(hypothesis)
        return detection

    def _make_detection_3d(
        self,
        color_msg: Image,
        frame_id: str,
        point: tuple[float, float, float],
        score: float,
        index: int,
    ) -> Detection3D:
        detection = Detection3D()
        detection.header.stamp = color_msg.header.stamp
        detection.header.frame_id = frame_id
        detection.id = str(index)

        x, y, z = point
        detection.bbox.center.position.x = x
        detection.bbox.center.position.y = y
        detection.bbox.center.position.z = z
        detection.bbox.center.orientation.w = 1.0
        detection.bbox.size.x = self.bbox_size[0]
        detection.bbox.size.y = self.bbox_size[1]
        detection.bbox.size.z = self.bbox_size[2]

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = self.class_id
        hypothesis.hypothesis.score = score
        hypothesis.pose.pose.position.x = x
        hypothesis.pose.pose.position.y = y
        hypothesis.pose.pose.position.z = z
        hypothesis.pose.pose.orientation.w = 1.0
        detection.results.append(hypothesis)
        return detection

    @staticmethod
    def _draw_debug(
        image: np.ndarray | None,
        x0: float,
        y0: float,
        x1: float,
        y1: float,
        score: float,
        depth: float | None,
    ) -> None:
        if image is None:
            return

        p0 = (int(round(x0)), int(round(y0)))
        p1 = (int(round(x1)), int(round(y1)))
        cv2.rectangle(image, p0, p1, (0, 255, 0), 2)

        if depth is None:
            text = f"shuttle {score:.2f} depth:N/A"
        else:
            text = f"shuttle {score:.2f} {depth:.2f}m"

        cv2.putText(
            image,
            text,
            (p0[0], max(18, p0[1] - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    def _maybe_log_stats(self, now_wall: float) -> None:
        elapsed = now_wall - self._last_stats_wall
        if elapsed < self.stats_period:
            return

        frames = max(1, self._frames_processed)
        avg_inference_ms = self._inference_ms_sum / frames
        self.get_logger().info(
            "YOLO stats: "
            f"frames={self._frames_processed}, "
            f"2d={self._detections_2d}, "
            f"3d={self._detections_3d}, "
            f"depth_rejected={self._depth_rejected}, "
            f"avg_inference={avg_inference_ms:.1f} ms"
        )

        self._frames_processed = 0
        self._detections_2d = 0
        self._detections_3d = 0
        self._depth_rejected = 0
        self._inference_ms_sum = 0.0
        self._last_stats_wall = now_wall


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloShuttleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
