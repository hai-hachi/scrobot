#!/usr/bin/env python3
"""AprilTags Lab: RGB Gaussian-noise input, true ROS detection/TF metrics, JPEG overlay.

Never invents simulated detections or poses. The production apriltag_ros
node receives the actual noisy RGB image and camera_info. No physical
camera driver, localization EKF, or existing Perception Labs is launched.
"""
from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from tag_math import comparison, expected_translation


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def summarize(values):
    if not values:
        return None
    return round(math.sqrt(sum(float(x)**2 for x in values)/len(values)), 5)


class TagMonitor(Node):
    def __init__(self, folder):
        super().__init__("apriltag_lab_monitor")
        self.folder = Path(folder)
        self.control_file = self.folder / "tag_control.json"
        self.status_file = self.folder / "tag_status.json"
        self.preview_file = self.folder / "tag_preview.jpg"
        self.control = None
        self.epoch = -1
        self.last_preview = 0.0
        self.camera_info = None
        self.rng = np.random.default_rng(42)
        self.rgb = deque(maxlen=5)
        self.detected = {}
        self.latest_tfs = {}
        self._reset()
        self.pub = self.create_publisher(Image, "/camera_lab/tag_color",
                                         qos_profile_sensor_data)
        self.create_subscription(Image, "/camera_lab/color", self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera_lab/color/camera_info",
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(AprilTagDetectionArray,
                                 "/camera_lab/detections", self.on_detections,
                                 qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 10)
        self.create_timer(0.12, self.read_control)
        self.create_timer(0.40, self.write_status)
        self.read_control()

    def _reset(self):
        self.reset_wall = time.monotonic()
        self.rgb_count = 0
        self.tag_frames = 0
        self.hits = 0
        self.pose_count = 0
        self.errors = []
        self.margins = []
        self.det_times = deque(maxlen=150)
        self.latest_observed = None
        self.last_stamp = 0.0
        self.rgb.clear()
        self.detected.clear()
        self.latest_tfs.clear()

    def read_control(self):
        try:
            data = json.loads(self.control_file.read_text())
            epoch = data.get("epoch")
            if type(epoch) is not int:
                return
            if epoch != self.epoch:
                self.control = data
                self.epoch = epoch
                self._reset()
            else:
                self.control = data
        except (OSError, ValueError, KeyError):
            return

    def on_info(self, msg):
        if len(msg.k) == 9 and msg.k[0] > 0 and msg.k[4] > 0:
            self.camera_info = msg

    def on_image(self, msg):
        cfg = self.control
        if cfg is None:
            return
        sigma = float(cfg["tag"]["noise_sigma"])
        if sigma > 0 and msg.encoding.lower() in ("rgb8", "bgr8"):
            # RGB camera bytes are not decoded/encoded for the detector,
            # only one noise copy; preview JPEG is separately throttled.
            a = np.frombuffer(msg.data, np.uint8)
            if a.size < msg.step * msg.height or msg.step < msg.width * 3:
                return
            raw = a[:msg.step*msg.height].reshape(msg.height,msg.step)
            arr = raw[:,:msg.width*3].reshape(
                msg.height,msg.width,3).copy()
            noise = self.rng.normal(0.0,sigma,arr.shape)
            arr = np.clip(arr.astype(np.float32)+noise,0,255).astype(np.uint8)
            out = Image()
            out.header = msg.header
            out.height = msg.height
            out.width = msg.width
            out.encoding = msg.encoding
            out.is_bigendian = msg.is_bigendian
            out.step = msg.width * 3
            out.data = arr.tobytes()
        else:
            out = msg
        self.pub.publish(out)
        if time.monotonic() - self.reset_wall > .35:
            self.rgb_count += 1
        self.rgb.append((stamp(out.header), out))
        # Preview after empty detections: still show the current camera POV.
        if time.monotonic() - self.last_preview > .75:
            self.preview(out, [])

    def on_detections(self, msg):
        if self.control is None:
            return
        now = time.monotonic()
        if now - self.reset_wall < .35:
            return
        self.tag_frames += 1
        self.det_times.append(now)
        target = self.control["tag"]["tag_id"]
        ds = []
        found = False
        for d in msg.detections:
            if d.id not in (0,1,2,3):
                continue
            item = {"id":int(d.id),"margin":float(d.decision_margin),
                    "hamming":int(d.hamming),
                    "corners":[(float(p.x),float(p.y)) for p in d.corners]}
            ds.append(item)
            if d.id == target:
                found = True
                self.margins.append(float(d.decision_margin))
        if found:
            self.hits += 1
        self.detected[round(stamp(msg.header),4)] = ds
        if len(self.detected)>8:
            self.detected.pop(next(iter(self.detected)))
        nearest = min(self.rgb, key=lambda x:abs(x[0]-stamp(msg.header)),
                      default=None)
        if nearest and abs(nearest[0]-stamp(msg.header))<0.03:
            self.preview(nearest[1], ds)
        self.last_stamp = stamp(msg.header)

    def on_tf(self, msg):
        cfg = self.control
        if cfg is None or time.monotonic()-self.reset_wall<.35:
            return
        tag_id = cfg["tag"]["tag_id"]
        for tf in msg.transforms:
            if tf.child_frame_id not in {
                f"observed_tag_{i}" for i in range(4)
            }:
                continue
            t = tf.transform.translation
            q = tf.transform.rotation
            key = int(tf.child_frame_id.split("_")[-1])
            sample = {
                "x":float(t.x), "y":float(t.y), "z":float(t.z),
                "quaternion":(float(q.x),float(q.y),float(q.z),float(q.w)),
                "stamp":stamp(tf.header),
            }
            self.latest_tfs[key] = sample
            if key != tag_id:
                continue
            # Match only this camera optical frame. Comparing a different
            # TF parent would silently invalidate every position error.
            if tf.header.frame_id.lstrip("/") != "camera_color_optical_frame":
                continue
            err = comparison(cfg["tag"],(sample["x"],sample["y"],sample["z"]),
                             sample["quaternion"])
            if err is not None:
                self.errors.append(err)
                self.pose_count += 1
                self.latest_observed = sample
        if len(self.errors)>5000:
            self.errors=self.errors[-5000:]

    def preview(self, msg, detections):
        now = time.monotonic()
        if now - self.last_preview < 0.35:
            return
        if msg.encoding.lower() not in ("rgb8", "bgr8"):
            return
        a=np.frombuffer(msg.data,np.uint8)
        if a.size<msg.step*msg.height or msg.step<msg.width*3:
            return
        image=a[:msg.step*msg.height].reshape(
            msg.height,msg.step)[:,:msg.width*3].reshape(
                msg.height,msg.width,3).copy()
        if msg.encoding.lower()=="rgb8":
            image=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
        scale=min(1.0,960.0/max(1,msg.width))
        if scale<1.0:
            image=cv2.resize(image,None,fx=scale,fy=scale,
                             interpolation=cv2.INTER_AREA)
        for d in detections:
            points=np.array(d["corners"],np.float32)*scale
            poly=np.round(points).astype(np.int32).reshape(-1,1,2)
            cv2.polylines(image,[poly],True,(0,235,255),2,cv2.LINE_AA)
            mx,my=np.mean(points,axis=0)
            cv2.putText(image,f'Tag {d["id"]}, margin {d["margin"]:.1f}',
                        (max(5,int(mx)+8),max(18,int(my)-12)),
                        cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,230),2)
            pose=self.latest_tfs.get(d["id"])
            if pose and abs(pose["stamp"]-stamp(msg.header)) < .09 and self.camera_info:
                self.draw_axes(image,pose,scale)
        sigma=self.control["tag"]["noise_sigma"] if self.control else 0
        cv2.putText(image,f'AprilTag 16h5 | RGB noise sigma={sigma:.1f}/255',
                    (10,25),cv2.FONT_HERSHEY_SIMPLEX,.62,(220,255,220),2)
        ok,jpg=cv2.imencode(".jpg",image,[cv2.IMWRITE_JPEG_QUALITY,75])
        if ok:
            tmp=self.preview_file.with_suffix(".tmp")
            tmp.write_bytes(jpg.tobytes())
            tmp.replace(self.preview_file)
            self.last_preview=now

    def draw_axes(self,image,pose,scale):
        # Project TF coordinate axes (not a simulated or fabricated overlay).
        from tag_math import rotate_quaternion
        v=np.array([pose["x"],pose["y"],pose["z"]],float)
        if v[2]<.01:
            return
        K=self.camera_info.k
        def project(point):
            if point[2]<=.01:
                return None
            return (int(round((K[0]*point[0]/point[2]+K[2])*scale)),
                    int(round((K[4]*point[1]/point[2]+K[5])*scale)))
        base=project(v)
        if base is None:
            return
        length=min(.10,float(self.control["tag"]["size"]))
        for axis,color in (((1,0,0),(0,0,255)),
                           ((0,1,0),(0,255,0)),
                           ((0,0,1),(255,0,0))):
            target=v+np.array(rotate_quaternion(pose["quaternion"],axis))*length
            p=project(target)
            if p is not None:
                cv2.arrowedLine(image,base,p,color,2,cv2.LINE_AA,tipLength=.2)

    def write_status(self):
        now=time.monotonic()
        window=[t for t in self.det_times if now-t<=3]
        det_fps=((len(window)-1)/(window[-1]-window[0])
                 if len(window)>1 and window[-1]>window[0] else None)
        summary = {"updated_wall":time.time(),"epoch":self.epoch,
                   "tag_id":self.control["tag"]["tag_id"] if self.control else None,
                   "rgb_frames":self.rgb_count,"detector_frames":self.tag_frames,
                   "detected_frames":self.hits,"pose_frames":self.pose_count,
                   "detection_pct":round(100*self.hits/self.tag_frames,2)
                       if self.tag_frames else None,
                   "detector_fps":round(det_fps,2) if det_fps else None,
                   "position_rmse_m":summarize(
                       [e["position_error_m"] for e in self.errors]),
                   "range_rmse_m":summarize(
                       [e["range_error_m"] for e in self.errors]),
                   "bearing_rmse_deg":summarize(
                       [e["bearing_error_deg"] for e in self.errors]),
                   "normal_axis_rmse_deg":summarize(
                       [e.get("normal_axis_error_deg",0) for e in self.errors]),
                   "decision_margin_mean":round(sum(self.margins)/len(self.margins),2)
                        if self.margins else None,
                   "true_translation":list(expected_translation(self.control["tag"]))
                        if self.control else None,
                   "observed_translation":self.latest_observed,
                   "camera_info_valid":self.camera_info is not None,
                   "noise_sigma":self.control["tag"]["noise_sigma"]
                       if self.control else 0,
                   "preview_age_s":round(now-self.last_preview,2)
                        if self.last_preview else None,
                  }
        tmp=self.status_file.with_suffix(".tmp")
        tmp.write_text(json.dumps(summary,allow_nan=False))
        tmp.replace(self.status_file)


def main():
    if len(sys.argv)!=2:
        raise SystemExit("Usage: tag_monitor.py result_folder")
    rclpy.init()
    node=TagMonitor(sys.argv[1])
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
