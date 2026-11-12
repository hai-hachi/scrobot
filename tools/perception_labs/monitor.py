#!/usr/bin/env python3
"""Separate ROS monitor for Perception Labs.

A crash in camera JPEG conversion, AprilTag messages, or inference never
prevents the web dashboard from opening or hides the Gazebo server log.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from core import Meter, bottleneck
from tag_quality import TagQuality

def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9

def atomic(path, data):
    target=Path(path)
    tmp=target.with_name(target.name+".tmp")
    tmp.write_bytes(data)
    tmp.replace(target)

class Observer(Node):
    def __init__(self, output, cfg):
        super().__init__("perception_lab_observer")
        self.out=Path(output)
        self.cfg=cfg
        self.meters={name:Meter() for name in (
            "rgb","depth","rgb_info","depth_info","tag","yolo","yolo3d")}
        self.tags=[]
        self.margins=[]
        self.tag_quality=TagQuality()
        self.bboxes=[]
        self.last_image_wall=0.0
        self.last_image_type="raw_rgb"
        self.prev_stamp=-1.0
        self.note=None
        self.create_subscription(
            Image, "/camera/camera/color/image_raw",
            self.rgb,qos_profile_sensor_data)
        self.create_subscription(
            Image,"/camera/camera/depth/image_raw",
            self.depth,qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo,"/camera/camera/color/camera_info",
            self.rgb_info,qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo,"/camera/camera/depth/camera_info",
            self.depth_info,qos_profile_sensor_data)
        try:
            from apriltag_msgs.msg import AprilTagDetectionArray
            self.create_subscription(AprilTagDetectionArray,
                "/apriltag/detections",self.tag,qos_profile_sensor_data)
        except ImportError:
            self.note="apriltag_msgs unavailable; install apriltag_ros"
        try:
            from vision_msgs.msg import Detection2DArray,Detection3DArray
            self.create_subscription(Detection2DArray,
                "/perception/shuttle_detections_2d",
                self.yolo,qos_profile_sensor_data)
            self.create_subscription(Detection3DArray,
                "/perception/shuttle_detections_3d",
                self.yolo3d,qos_profile_sensor_data)
        except ImportError:
            self.note="vision_msgs unavailable"
        self.create_subscription(
            Image,"/perception/shuttle_debug/image",self.yolo_preview,
            qos_profile_sensor_data)
        self.create_timer(.5,self.write_status)
        self.write_status()

    def meter(self,name,msg):
        self.meters[name].add(stamp(msg.header),
            getattr(msg,"width",None),getattr(msg,"height",None))

    def preview(self,msg,label):
        now=time.monotonic()
        if now-self.last_image_wall < .4:
            return
        try:
            import numpy as np
            import cv2
            if msg.encoding.lower() in ("rgb8","bgr8"):
                colors=3
                frame=np.ndarray((msg.height,msg.width,colors),
                                 dtype=np.uint8,buffer=msg.data,
                                 strides=(msg.step,colors,1))
                if msg.encoding.lower()=="rgb8":
                    frame=cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
            elif msg.encoding.lower()=="mono8":
                frame=np.ndarray((msg.height,msg.width),dtype=np.uint8,
                                 buffer=msg.data,strides=(msg.step,1))
            else:
                self.note=f"Unsupported preview image encoding: {msg.encoding}"
                return
            if frame.shape[1]>960:
                frame=cv2.resize(frame,(960,round(960*frame.shape[0]/frame.shape[1])),
                                 interpolation=cv2.INTER_AREA)
            ok,jpg=cv2.imencode(".jpg",frame,[cv2.IMWRITE_JPEG_QUALITY,75])
            if not ok:
                return
            atomic(self.out/"camera.jpg",jpg.tobytes())
            self.last_image_wall=now
            self.last_image_type=label
        except (ValueError,ImportError,RuntimeError) as exc:
            self.note=f"Preview conversion: {exc}"

    def rgb(self,msg):
        self.meter("rgb",msg)
        if self.cfg["mode"]!="yolo" or time.monotonic()-self.last_image_wall>.9:
            self.preview(msg,"RGB")
    def depth(self,msg):
        self.meter("depth",msg)
    def rgb_info(self,msg):
        self.meter("rgb_info",msg)
    def depth_info(self,msg):
        self.meter("depth_info",msg)
    def tag(self,msg):
        self.meter("tag",msg)
        self.tags=[int(d.id) for d in msg.detections]
        self.margins=[round(float(d.decision_margin),2)
                      for d in msg.detections if hasattr(d,"decision_margin")]
        samples=[]
        for d in msg.detections:
            item={"id":int(d.id),
                  "margin":getattr(d,"decision_margin",None),
                  "hamming":getattr(d,"hamming",None)}
            try:
                # apriltag_msgs pose is PoseWithCovarianceStamped in ROS 2.
                pose=d.pose.pose.pose
                pos,q=pose.position,pose.orientation
                item["position"]=(pos.x,pos.y,pos.z)
                item["quaternion"]=(q.x,q.y,q.z,q.w)
            except (AttributeError,TypeError,ValueError):
                # Some detector builds omit pose; ID/margin remain valid.
                pass
            samples.append(item)
        self.tag_quality.add(samples)
    def yolo(self,msg):
        self.meter("yolo",msg)
        detections=[]
        for det in msg.detections:
            center=getattr(det.bbox.center,"position",det.bbox.center)
            score=float(det.results[0].hypothesis.score) if det.results else None
            detections.append({
                "x":round(float(center.x),2),"y":round(float(center.y),2),
                "w":round(float(det.bbox.size_x),2),
                "h":round(float(det.bbox.size_y),2),
                "confidence":score})
        self.bboxes=detections
    def yolo3d(self,msg):
        self.meter("yolo3d",msg)
    def yolo_preview(self,msg):
        self.preview(msg,"YOLO bounding boxes")

    def write_status(self):
        data={
            "time_wall":time.time(),
            "streams":{key:m.status() for key,m in self.meters.items()},
            "tags":self.tags,
            "decision_margins":self.margins,
            "tag_quality":self.tag_quality.status(),
            "tag_frames":self.tag_quality.message_count,
            "bboxes":self.bboxes,
            "image_type":self.last_image_type,
            "preview_age_s":round(time.monotonic()-self.last_image_wall,2)
                  if self.last_image_wall else None,
            "message":self.note,
        }
        data["bottleneck"]=bottleneck(data["streams"],self.cfg["color_fps"],
                                      self.cfg["mode"])
        atomic(self.out/"monitor.json",json.dumps(data,allow_nan=False).encode())

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--out",required=True)
    p.add_argument("--config",required=True)
    args=p.parse_args()
    cfg=json.loads(Path(args.config).read_text())
    Path(args.out).mkdir(parents=True,exist_ok=True)
    rclpy.init()
    node=Observer(args.out,cfg)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
