#!/usr/bin/env python3
"""Raw ROS image transport meter: no decoding, JPEG, GUI, or YOLO."""
from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image


class CameraMeter(Node):
    def __init__(self, output, depth_enabled):
        super().__init__("camera_lab_meter")
        self.output = Path(output)
        self.records = {"color": deque(maxlen=500),
                        "depth": deque(maxlen=500)}
        self.count = {"color": 0, "depth": 0}
        self.bad_stamps = {"color": 0, "depth": 0}
        self.dimensions = {"color": None, "depth": None}
        self.start = time.monotonic()
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(
            Image, "/camera_lab/color",
            lambda m: self.receive("color", m), qos)
        if depth_enabled:
            self.create_subscription(
                Image, "/camera_lab/depth",
                lambda m: self.receive("depth", m), qos)
        self.create_timer(0.5, self.write)

    def receive(self, name, msg):
        arrival = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        data_size = len(msg.data)
        self.count[name] += 1
        rec = self.records[name]
        if rec and stamp <= rec[-1][1]:
            self.bad_stamps[name] += 1
        rec.append((arrival, stamp, data_size))
        self.dimensions[name] = [int(msg.width), int(msg.height)]

    def write(self):
        streams = {}
        now = time.monotonic()
        for name, records in self.records.items():
            recent = [x for x in records if now - x[0] <= 3.0]
            n = len(recent)
            if n >= 2:
                wall_span = recent[-1][0] - recent[0][0]
                sim_span = recent[-1][1] - recent[0][1]
                wall_fps = (n - 1) / wall_span if wall_span > 0 else None
                sim_fps = (n - 1) / sim_span if sim_span > 0 else None
                mb_s = sum(x[2] for x in recent[1:]) / wall_span / 1e6 if wall_span > 0 else None
            else:
                wall_fps = sim_fps = mb_s = None
            streams[name] = {
                "frames": self.count[name],
                "resolution": self.dimensions[name],
                "wall_fps": round(wall_fps, 2) if wall_fps is not None else None,
                "sim_fps": round(sim_fps, 2) if sim_fps is not None else None,
                "mb_s": round(mb_s, 3) if mb_s is not None else None,
                "last_age_s": round(now - records[-1][0], 2) if records else None,
                "nonmonotonic_stamps": self.bad_stamps[name],
            }
        payload = {
            "updated_wall": time.time(),
            "monitor_uptime_s": round(now - self.start, 1),
            "streams": streams,
        }
        tmp = self.output.with_suffix(".tmp")
        tmp.write_text(json.dumps(payload))
        tmp.replace(self.output)


def main():
    if len(sys.argv) != 3 or sys.argv[2] not in ("true", "false"):
        raise SystemExit("Usage: monitor.py output.json true|false")
    rclpy.init()
    meter = CameraMeter(sys.argv[1], sys.argv[2] == "true")
    try:
        rclpy.spin(meter)
    finally:
        meter.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
