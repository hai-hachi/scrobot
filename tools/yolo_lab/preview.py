#!/usr/bin/env python3
"""Low-rate preview only: does not republish full-resolution images to ROS.

The raw input frame-rate monitor runs separately in camera_lab/monitor.py.
No artificial Gaussian noise is applied to image callbacks here.
"""
from __future__ import annotations

import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Preview(Node):
    def __init__(self, output):
        super().__init__("yolo_lab_preview")
        self.output = Path(output)
        self.last = 0.0
        self.count = 0
        self.saved = 0
        self.create_subscription(
            Image, "/camera_lab/color", self.frame, qos_profile_sensor_data)

    def frame(self, msg):
        self.count += 1
        now = time.monotonic()
        if now - self.last < 0.5:
            return
        if msg.encoding not in ("rgb8", "bgr8"):
            return
        if msg.step < msg.width * 3 or msg.height < 1:
            return
        self.last = now
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < msg.step * msg.height:
            return
        img = raw[:msg.step * msg.height].reshape(
            msg.height, msg.step)[:, :msg.width * 3].reshape(
            msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        target_width = min(msg.width, 960)
        if msg.width > target_width:
            scale = target_width / msg.width
            img = cv2.resize(
                img, (target_width, max(1, int(round(msg.height * scale)))),
                interpolation=cv2.INTER_AREA)
        ok, encoded = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 78])
        if not ok:
            return
        self.output.parent.mkdir(parents=True, exist_ok=True)
        temp = self.output.with_suffix(".jpg.tmp")
        temp.write_bytes(encoded.tobytes())
        temp.replace(self.output)
        self.saved += 1
        meta = self.output.with_suffix(".json")
        meta_tmp = meta.with_suffix(".tmp")
        meta_tmp.write_text(json.dumps({
            "updated_wall": time.time(),
            "input_resolution": [int(msg.width), int(msg.height)],
            "generated_previews": self.saved,
            "received_frames": self.count,
            "note": "JPEG preview is deliberately throttled; see raw meter for actual FPS",
        }) + "\n")
        meta_tmp.replace(meta)


def main():
    if len(sys.argv) != 2:
        raise SystemExit("Usage: preview.py /path/to/preview.jpg")
    rclpy.init()
    node = None
    try:
        node = Preview(sys.argv[1])
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT/SIGTERM is expected during Gazebo lab restarts.
        pass
    finally:
        if node is not None:
            node.destroy_node()
        # ROS 2's signal handler may have already shut down the context.
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
