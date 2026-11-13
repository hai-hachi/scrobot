#!/usr/bin/env python3
"""Terminal monitor for the SCROBOT YOLO perception debug launch."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray, Detection3DArray


class YoloPerceptionMonitor(Node):
    def __init__(self):
        super().__init__("yolo_perception_monitor")

        self.declare_parameter("report_rate", 1.0)
        self.declare_parameter("stale_timeout", 2.0)

        self.report_rate = float(self.get_parameter("report_rate").value)
        self.stale_timeout = float(self.get_parameter("stale_timeout").value)

        self.last_rgb = None
        self.last_depth = None
        self.last_info = None
        self.last_2d = None
        self.last_3d = None

        self.rgb_count = 0
        self.depth_count = 0
        self.det2d_count = 0
        self.det3d_count = 0

        self.last_report_wall = time.perf_counter()

        self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self._rgb_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self._depth_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self._info_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Detection2DArray,
            "/perception/shuttle_detections_2d",
            self._det2d_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Detection3DArray,
            "/perception/shuttle_detections_3d",
            self._det3d_cb,
            qos_profile_sensor_data,
        )

        self.timer = self.create_timer(1.0 / self.report_rate, self._report)
        self.get_logger().info(
            "YOLO perception monitor started. "
            "Watching RGB, aligned depth, CameraInfo, 2D and 3D detections."
        )

    @staticmethod
    def _now():
        return time.perf_counter()

    def _rgb_cb(self, _msg):
        self.last_rgb = self._now()
        self.rgb_count += 1

    def _depth_cb(self, _msg):
        self.last_depth = self._now()
        self.depth_count += 1

    def _info_cb(self, _msg):
        self.last_info = self._now()

    def _det2d_cb(self, msg):
        self.last_2d = self._now()
        self.det2d_count = len(msg.detections)

    def _det3d_cb(self, msg):
        self.last_3d = self._now()
        self.det3d_count = len(msg.detections)
        self._last_3d_msg = msg

    def _state(self, stamp):
        if stamp is None:
            return "WAIT"
        age = self._now() - stamp
        return "OK" if age <= self.stale_timeout else f"STALE({age:.1f}s)"

    def _best_detection_text(self):
        msg = getattr(self, "_last_3d_msg", None)
        if msg is None or not msg.detections:
            return "none"

        best = max(
            msg.detections,
            key=lambda d: (
                float(d.results[0].hypothesis.score)
                if d.results else 0.0
            ),
        )
        if best.results:
            result = best.results[0]
            p = result.pose.pose.position
            score = float(result.hypothesis.score)
        else:
            p = best.bbox.center.position
            score = 0.0

        range_3d = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
        return (
            f"score={score:.2f} "
            f"xyz=({p.x:+.3f},{p.y:+.3f},{p.z:+.3f})m "
            f"range={range_3d:.3f}m"
        )

    def _report(self):
        now = self._now()
        dt = max(1e-6, now - self.last_report_wall)

        rgb_hz = self.rgb_count / dt
        depth_hz = self.depth_count / dt

        node_names = set(self.get_node_names())
        detector_state = (
            "OK" if "yolo_shuttle_detector" in node_names else "MISSING"
        )

        self.get_logger().info(
            "[YOLO CHECK] "
            f"NODE={detector_state} | "
            f"RGB={self._state(self.last_rgb)} {rgb_hz:.1f}Hz | "
            f"ALIGNED_DEPTH={self._state(self.last_depth)} {depth_hz:.1f}Hz | "
            f"INFO={self._state(self.last_info)} | "
            f"2D={self._state(self.last_2d)} n={self.det2d_count} | "
            f"3D={self._state(self.last_3d)} n={self.det3d_count} | "
            f"best={self._best_detection_text()}"
        )

        self.rgb_count = 0
        self.depth_count = 0
        self.last_report_wall = now


def main(args=None):
    rclpy.init(args=args)
    node = YoloPerceptionMonitor()
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
