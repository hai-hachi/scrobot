#!/usr/bin/env python3
"""Real YOLO bounding boxes on Gazebo ROS RGB; never blocks the raw FPS meter.

Runs model inference in its own ROS process. Preview is limited to 960px,
processed frames are intentionally bounded by max_rate; its throughput is
not the simulated RGB sensor's FPS.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO


class LiveDetection(Node):
    def __init__(self, model, output, confidence, imgsz, max_rate):
        super().__init__("yolo_lab_detector_" + Path(output).name.replace("-", "_"))
        self.model_path = Path(model).resolve()
        if not self.model_path.is_file():
            raise FileNotFoundError(self.model_path)
        self.model = YOLO(str(self.model_path))
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.confidence = float(confidence)
        self.imgsz = int(imgsz)
        self.period = 1.0 / max_rate
        self.last_infer = 0.0
        self.total = 0
        self.processed = 0
        self.prev_processed = 0.0
        self.last_boxes = []
        self.create_subscription(
            Image, "/camera_lab/color", self.frame, qos_profile_sensor_data)

    def frame(self, msg):
        self.total += 1
        now = time.monotonic()
        if now - self.last_infer < self.period:
            return
        self.last_infer = now
        if msg.encoding not in ("rgb8", "bgr8") or msg.step < msg.width * 3:
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < msg.step * msg.height:
            return
        pixels = raw[:msg.step * msg.height].reshape(
            msg.height, msg.step)[:, :msg.width * 3].reshape(
            msg.height, msg.width, 3)
        img = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else pixels
        tic = time.perf_counter()
        detected = self.model.predict(
            source=img, imgsz=self.imgsz, conf=self.confidence,
            classes=[0], verbose=False)[0]
        inference_ms = (time.perf_counter() - tic) * 1000.0
        width = min(int(msg.width), 960)
        ratio = width / float(msg.width)
        height = max(1, int(msg.height * ratio))
        view = cv2.resize(img, (width, height))
        boxes = []
        if detected.boxes is not None and len(detected.boxes):
            coords = detected.boxes.xyxy.detach().cpu().numpy()
            scores = detected.boxes.conf.detach().cpu().numpy()
            for bbox, score in zip(coords,scores):
                x0,y0,x1,y1 = map(float,bbox)
                cv2.rectangle(view,(int(x0*ratio),int(y0*ratio)),
                              (int(x1*ratio),int(y1*ratio)),(40,235,60),2)
                cv2.putText(view,f"Shuttle {score:.2f}",
                            (int(x0*ratio),max(14,int(y0*ratio)-4)),
                            cv2.FONT_HERSHEY_SIMPLEX,.44,(40,235,60),1)
                boxes.append({"x0":round(x0,1),"y0":round(y0,1),
                              "x1":round(x1,1),"y1":round(y1,1),
                              "confidence":round(float(score),4)})
        ok, encoded = cv2.imencode(".jpg",view,[cv2.IMWRITE_JPEG_QUALITY,80])
        if ok:
            tmp=self.output/"live.jpg.tmp"
            tmp.write_bytes(encoded.tobytes())
            tmp.replace(self.output/"live.jpg")
        self.processed += 1
        fps = (1.0/(now-self.prev_processed)) if self.prev_processed>0 and now>self.prev_processed else None
        self.prev_processed = now
        result={
            "updated_wall":time.time(),"model":str(self.model_path),
            "received_frames":self.total,"processed_frames":self.processed,
            "input_resolution":[int(msg.width),int(msg.height)],
            "inference_ms":round(inference_ms,2),
            "processed_fps":round(fps,2) if fps else None,
            "detections":len(boxes),"boxes":boxes,
        }
        temp=self.output/"live_status.tmp"
        temp.write_text(json.dumps(result)+"\n")
        temp.replace(self.output/"live_status.json")


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--model",required=True)
    p.add_argument("--output",required=True)
    p.add_argument("--confidence",type=float,default=0.1)
    p.add_argument("--imgsz",type=int,default=640)
    p.add_argument("--max-rate",type=float,default=15.0)
    args=p.parse_args()
    if not 0<args.confidence<=1 or args.max_rate<=0 or args.imgsz<320:
        p.error("Invalid confidence / max-rate / image size")
    rclpy.init()
    node=LiveDetection(args.model,args.output,args.confidence,args.imgsz,args.max_rate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # normal stop/restart of the A/B comparison
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=="__main__":
    main()
