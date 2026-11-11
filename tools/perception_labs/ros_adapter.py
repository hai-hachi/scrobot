"""Read-only ROS2 telemetry for RGB-D, AprilTag, YOLO and Gazebo truth.

ROS subscriptions are best-effort for sensor streams. Preview encoding is
rate-limited and never modifies actual sensor messages.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from vision_msgs.msg import Detection2DArray, Detection3DArray
try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:
    AprilTagDetectionArray = None

from core import StreamMeter, timing_summary
from tag_geometry import known_tags, expected_camera_to_tag, translation_error


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def rgb_image(msg):
    if msg.encoding not in ("rgb8", "bgr8", "mono8"):
        return None
    channels = 1 if msg.encoding == "mono8" else 3
    if msg.step < msg.width * channels:
        return None
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if data.size < msg.step * msg.height:
        return None
    image = data[:msg.step * msg.height].reshape(msg.height, msg.step)[:, :msg.width * channels]
    if channels == 1:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    else:
        image = image.reshape(msg.height, msg.width, channels)
        if msg.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image


class PerceptionDriver(Node):
    def __init__(self):
        super().__init__(
            "perception_lab_monitor",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
            automatically_declare_parameters_from_overrides=True,
        )
        self.guard = threading.RLock()
        self.streams = {name: StreamMeter() for name in (
            "rgb", "depth", "aligned_depth", "camera_info", "depth_info", "apriltag",
            "yolo2d", "yolo3d", "truth", "shuttle_truth",
        )}
        self.samples = deque(maxlen=3000)
        self.last_preview = None
        self.last_preview_stamp = 0.0
        self.last_preview_wall = 0.0
        self.last_overlay = None
        self.last_overlay_wall = 0.0
        self.last_overlay_stamp = 0.0
        self.preview_kind = "RGB"
        self.tag_ids = []
        self.tag_margins = []
        self.tag_hamming = []
        self.last_bboxes = []
        self.last_confidence = []
        self.last_truth = None
        self.truth_pose = None
        self.last_shuttles = []
        self.tag_errors = deque(maxlen=300)
        self.last_tag_stamp_evaluated = 0
        self.latest_tag_stamp = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self,spin_thread=False)
        try:
            source=(Path(get_package_share_directory("scrobot_localization"))
                    / "config/court_landmarks.yaml")
            self.known_tags = known_tags(yaml.safe_load(source.read_text()))
        except (OSError,ValueError,KeyError) as exc:
            self.known_tags={}
            self.error=f"Court landmark calibration unavailable: {exc}"
        self.create_timer(.1,self.evaluate_tags)
        self.info = {}
        self.error = None
        sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        def sub(msgtype, topic, cb):
            self.create_subscription(msgtype, topic, cb, sensor)
        sub(Image, "/camera/camera/color/image_raw", self.on_rgb)
        sub(Image, "/camera/camera/depth/image_raw", self.on_depth)
        sub(Image, "/camera/camera/aligned_depth_to_color/image_raw",
            self.on_aligned_depth)
        sub(CameraInfo, "/camera/camera/color/camera_info", self.on_color_info)
        sub(CameraInfo, "/camera/camera/depth/camera_info", self.on_depth_info)
        sub(Image, "/debug/apriltag_overlay/image", self.on_tag_overlay)
        sub(Image, "/perception/shuttle_debug/image", self.on_yolo_overlay)
        sub(Detection2DArray, "/perception/shuttle_detections_2d", self.on_yolo2d)
        sub(Detection3DArray, "/perception/shuttle_detections_3d", self.on_yolo3d)
        sub(Odometry, "/evaluation/ground_truth_odom", self.on_truth)
        sub(PoseArray, "/evaluation/shuttle_ground_truth", self.on_shuttle_truth)
        if AprilTagDetectionArray is not None:
            sub(AprilTagDetectionArray, "/apriltag/detections", self.on_tag)
        else:
            self.error = "apriltag_msgs missing: tag count unavailable; install apriltag_ros"
        self.last_clock = 0.0

    def _record(self, name, msg, width=None, height=None):
        t = stamp_s(msg.header)
        now = self.get_clock().now().nanoseconds / 1e9
        meter = self.streams[name]
        meter.add(t, msg.header.frame_id, width, height, sim_now=now,
                  bytes_len=len(msg.data) if hasattr(msg, "data") else 0)
        return t

    def on_rgb(self, msg):
        with self.guard:
            t = self._record("rgb", msg, msg.width, msg.height)
            if time.monotonic() - self.last_preview_wall < 0.18:
                return
            self._jpeg(msg, t, overlay=False)
            self.samples.append({"t":t,"source":"rgb","width":msg.width,"height":msg.height})

    def _jpeg(self, msg, stamp, overlay=False):
        image = rgb_image(msg)
        if image is None:
            return
        if image.shape[1] > 960:
            h = round(image.shape[0] * 960 / image.shape[1])
            image = cv2.resize(image, (960,h), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 78])
        if ok:
            if overlay:
                self.last_overlay = bytes(buf)
                self.last_overlay_wall = time.monotonic()
                self.last_overlay_stamp = stamp
            else:
                self.last_preview = bytes(buf)
                self.last_preview_wall = time.monotonic()
                self.last_preview_stamp = stamp

    def on_depth(self, msg):
        with self.guard:
            self._record("depth", msg, msg.width, msg.height)

    def on_aligned_depth(self, msg):
        with self.guard:
            self._record("aligned_depth", msg, msg.width, msg.height)

    def on_color_info(self, msg):
        with self.guard:
            self._record("camera_info", msg, msg.width, msg.height)
            self.info["rgb"] = {"width": msg.width,"height":msg.height,
                                "fx":round(float(msg.k[0]),3),
                                "fy":round(float(msg.k[4]),3),
                                "cx":round(float(msg.k[2]),3),
                                "cy":round(float(msg.k[5]),3),
                                "frame":msg.header.frame_id}

    def on_depth_info(self, msg):
        with self.guard:
            self._record("depth_info", msg, msg.width, msg.height)
            self.info["depth"] = {"width":msg.width,"height":msg.height,
                                  "fx":round(float(msg.k[0]),3),
                                  "fy":round(float(msg.k[4]),3)}

    def on_tag_overlay(self, msg):
        with self.guard:
            if time.monotonic() - self.last_overlay_wall >= .18:
                self._jpeg(msg, stamp_s(msg.header), overlay=True)
                self.preview_kind = "AprilTag overlay"

    def on_yolo_overlay(self, msg):
        with self.guard:
            if time.monotonic() - self.last_overlay_wall >= .18:
                self._jpeg(msg, stamp_s(msg.header), overlay=True)
                self.preview_kind = "YOLO boxes"

    def on_tag(self, msg):
        with self.guard:
            t=self._record("apriltag", msg)
            ids, margins, hamming = [], [], []
            for det in msg.detections:
                ids.append(int(det.id))
                if hasattr(det, "decision_margin"):
                    margins.append(float(det.decision_margin))
                if hasattr(det, "hamming"):
                    hamming.append(int(det.hamming))
            self.tag_ids=ids
            self.latest_tag_stamp=t
            self.tag_margins=margins
            self.tag_hamming=hamming
            self.samples.append({"t":t,"source":"apriltag",
                                 "ids":ids,"margins":margins,"hamming":hamming})

    def evaluate_tags(self):
        # TF often arrives just after the detector message. Evaluate after
        # a short delay and only when the robot is stationary to avoid
        # mismatching a fresh TF with an old Gazebo base pose.
        with self.guard:
            if (not self.tag_ids or not self.truth_pose or
                    self.latest_tag_stamp<=self.last_tag_stamp_evaluated):
                return
            if self.last_truth is None:
                return
            if (abs(self.last_truth.get("v",0))>.02 or
                    abs(self.last_truth.get("omega",0))>.03):
                return
            try:
                static=self.tf_buffer.lookup_transform(
                    "base_footprint","camera_color_optical_frame",Time())
                cb=static.transform
                camera=((cb.translation.x,cb.translation.y,cb.translation.z),
                        (cb.rotation.x,cb.rotation.y,cb.rotation.z,cb.rotation.w))
                usable=0
                for tid in self.tag_ids:
                    if tid not in self.known_tags:continue
                    detected=self.tf_buffer.lookup_transform(
                        "camera_color_optical_frame",f"observed_tag_{tid}",Time())
                    p=detected.transform.translation
                    observed=(float(p.x),float(p.y),float(p.z))
                    expected=expected_camera_to_tag(
                        self.truth_pose,camera,self.known_tags[tid])
                    err=translation_error(observed,expected)
                    record={"id":tid,"t":self.latest_tag_stamp,**err}
                    self.tag_errors.append(record)
                    self.samples.append({"source":"apriltag_error",**record})
                    usable+=1
                if usable:
                    self.last_tag_stamp_evaluated=self.latest_tag_stamp
            except Exception:
                # No TF yet is not a zero-error detection. Try later;
                # dashboard explicitly reports insufficient pose samples.
                pass

    def on_yolo2d(self, msg):
        with self.guard:
            t=self._record("yolo2d",msg)
            detections=[]
            for det in msg.detections:
                center=getattr(det.bbox.center,"position",det.bbox.center)
                conf=(float(det.results[0].hypothesis.score)
                      if det.results else None)
                detections.append({
                    "cx":round(float(center.x),2),
                    "cy":round(float(center.y),2),
                    "w":round(float(det.bbox.size_x),2),
                    "h":round(float(det.bbox.size_y),2),
                    "confidence":conf,
                })
            self.last_bboxes=detections
            self.last_confidence=[d["confidence"] for d in detections]
            self.samples.append({"t":t,"source":"yolo2d","detections":detections})

    def on_yolo3d(self,msg):
        with self.guard:
            self._record("yolo3d",msg)

    def on_truth(self,msg):
        with self.guard:
            self._record("truth",msg)
            p=msg.pose.pose.position
            q=msg.pose.pose.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),
                           1-2*(q.y*q.y+q.z*q.z))
            self.truth_pose=(
                (float(p.x),float(p.y),float(p.z)),
                (float(q.x),float(q.y),float(q.z),float(q.w))
            )
            self.last_truth={"x":round(float(p.x),4),
                             "y":round(float(p.y),4),
                             "z":round(float(p.z),4),
                             "yaw_deg":round(math.degrees(yaw),2),
                             "v":round(float(msg.twist.twist.linear.x),3),
                             "omega":round(float(msg.twist.twist.angular.z),3),
                             "t":stamp_s(msg.header)}

    def on_shuttle_truth(self,msg):
        with self.guard:
            self._record("shuttle_truth",msg)
            self.last_shuttles=[{"x":round(float(p.position.x),3),
                                 "y":round(float(p.position.y),3),
                                 "z":round(float(p.position.z),3)}
                                for p in msg.poses[:100]]

    def status(self,settings):
        with self.guard:
            summary=timing_summary(self.streams,settings)
            errors=list(self.tag_errors)
            per_tag={}
            for tid in sorted({r["id"] for r in errors}):
                vals=[r for r in errors if r["id"]==tid]
                per_tag[str(tid)]={
                    "n":len(vals),
                    "translation_rmse_m":round(math.sqrt(
                        sum(v["translation_error_m"]**2 for v in vals)/len(vals)),4),
                    "range_rmse_m":round(math.sqrt(
                        sum(v["range_error_m"]**2 for v in vals)/len(vals)),4),
                    "average_range_m":round(
                        sum(v["true_range_m"] for v in vals)/len(vals),3),
                }
            summary.update({
                "apriltag_pose":per_tag,
                "apriltag_pose_samples":len(errors),
                "apriltag_pose_note":"Only stationary camera-to-tag XYZ errors using known court model and Gazebo/TF. No unverified rotation comparison.",
                "preview_kind":self.preview_kind,
                "preview_stamp":round(self.last_overlay_stamp if settings["mode"]!="camera"
                                     else self.last_preview_stamp,4),
                "tag_ids":list(self.tag_ids),
                "decision_margins":list(self.tag_margins),
                "hamming":list(self.tag_hamming),
                "bboxes":list(self.last_bboxes),
                "ground_truth":self.last_truth,
                "shuttles":list(self.last_shuttles),
                "camera_info":dict(self.info),
                "monitor_error":self.error,
                "sim_s":round(self.get_clock().now().nanoseconds/1e9,3),
            })
            return summary

    def frame(self,mode):
        with self.guard:
            # Never display an old overlay for a new pose or mode.
            if (mode in ("apriltag","yolo") and self.last_overlay is not None
                    and time.monotonic()-self.last_overlay_wall < .8):
                return self.last_overlay
            return self.last_preview

    def counts(self):
        with self.guard:
            return {k:v.count for k,v in self.streams.items()}

    def save_records(self):
        with self.guard:
            return list(self.samples)

    def reset_observation(self):
        with self.guard:
            self.samples.clear()
            self.tag_errors.clear()
            self.tag_ids=[];self.last_bboxes=[]
            self.latest_tag_stamp=0.0
            self.last_tag_stamp_evaluated=0.0
            self.last_overlay=None;self.last_overlay_wall=0.0


class RosBridge:
    def __init__(self):
        rclpy.init()
        self.node=PerceptionDriver()
        self.thread=threading.Thread(target=rclpy.spin,args=(self.node,),
                                     name="PerceptionLabROS",daemon=True)
        self.thread.start()

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.thread.join(timeout=2)
