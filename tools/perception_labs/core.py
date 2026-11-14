"""Perception Labs v3: isolated camera, tags and YOLO benchmark. No ROS imports."""
from __future__ import annotations
from collections import deque
import math
import statistics
import time

COLOR = {"640x480": (640, 480), "848x480": (848, 480), "1280x720": (1280, 720), "1920x1080": (1920, 1080)}
DEPTH = {"424x240": (424, 240), "640x480": (640, 480),
         "848x480": (848, 480), "1280x720": (1280, 720)}
FPS = (5, 10, 15, 30, 60)
MODES = ("world_smoke", "rgb_smoke", "camera", "apriltag", "yolo")
DEFAULTS = {
    "mode": "world_smoke", "color": "640x480", "depth": "640x480",
    "color_fps": 15, "depth_fps": 15, "depth_enabled": False,
    "robot_x": 2.0, "robot_y": 0.0, "robot_yaw": 0.0,
    "shuttle_x": 3.0, "shuttle_y": 0.0, "shuttle_count": 5,
    "model": "", "device": "0", "dataset_images": 1200,
    "dataset_shuttles": 40, "seed": 42, "positive_fraction": 0.85,
    "epochs": 15, "batch": 8, "imgsz": 640,
}

def validate(raw):
    if not isinstance(raw, dict) or set(raw) - set(DEFAULTS):
        raise ValueError("Unknown perception settings")
    cfg = {**DEFAULTS, **raw}
    if cfg["mode"] not in MODES:
        raise ValueError("Unknown mode")
    if cfg["color"] not in COLOR or cfg["depth"] not in DEPTH:
        raise ValueError("Unsupported camera resolution")
    for name in ("color_fps", "depth_fps"):
        if type(cfg[name]) is not int or cfg[name] not in FPS:
            raise ValueError(f"{name}: use {FPS}")
    if type(cfg["depth_enabled"]) is not bool:
        raise ValueError("depth_enabled must be boolean")
    for name, lower, upper in (
        ("robot_x", -6.3, 6.3), ("robot_y", -2.85, 2.85),
        ("robot_yaw", -180, 180), ("shuttle_x", -6.3, 6.3),
        ("shuttle_y", -2.85, 2.85), ("positive_fraction", 0, 1),
    ):
        v = cfg[name]
        if type(v) not in (float, int) or not math.isfinite(v) or not lower <= v <= upper:
            raise ValueError(f"{name} must be in [{lower}, {upper}]")
        cfg[name] = float(v)
    for name, lower, upper in (
        ("shuttle_count", 1, 20), ("dataset_images", 100, 50000),
        ("dataset_shuttles", 1, 100), ("seed", 0, 99999999),
        ("epochs", 1, 300), ("batch", 1, 64), ("imgsz", 320, 1280),
    ):
        v = cfg[name]
        if type(v) is not int or not lower <= v <= upper:
            raise ValueError(f"{name} must be in [{lower}, {upper}]")
    if cfg["imgsz"] % 32:
        raise ValueError("YOLO image size must be divisible by 32")
    if not isinstance(cfg["model"], str) or len(cfg["model"]) > 500:
        raise ValueError("Invalid model path")
    if not isinstance(cfg["device"], str) or not cfg["device"] or any(
        c not in "0123456789cpu,-" for c in cfg["device"]
    ):
        raise ValueError("Invalid inference device")
    if cfg["mode"] in ("world_smoke", "rgb_smoke"):
        cfg.update(color="640x480", color_fps=15, depth_enabled=False)
    if cfg["mode"] == "yolo":
        # Exactly co-located depth and RGB with identical intrinsics avoids
        # false "aligned depth" claims. This lab intentionally uses that
        # simplified aligned setup, not the physical D435i stereo baseline.
        cfg["depth"] = cfg["color"]
        cfg["depth_fps"] = cfg["color_fps"]
        cfg["depth_enabled"] = True
    return cfg

def frame_rate(stamps):
    intervals = [b-a for a,b in zip(stamps,list(stamps)[1:]) if b > a]
    if len(intervals) < 3:
        return None, None
    return round(1 / statistics.fmean(intervals), 2), round(
        statistics.pstdev(intervals) * 1000, 2)

class Meter:
    def __init__(self):
        self.stamps = deque(maxlen=180)
        self.wall = deque(maxlen=180)
        self.n = 0
        self.invalid = 0
        self.size = None
        self.payload = deque(maxlen=180)
        self.payload_sum = 0
        self.total_bytes = 0

    def add(self, stamp, width=None, height=None, bytes_count=0):
        now = time.monotonic()
        if not math.isfinite(stamp) or stamp < 0:
            self.invalid += 1
            return
        self.n += 1
        if bytes_count:
            size = max(0, int(bytes_count))
            if len(self.payload) == self.payload.maxlen:
                self.payload_sum -= self.payload[0][1]
            self.payload.append((now, size))
            self.payload_sum += size
            self.total_bytes += size
        if self.stamps and stamp <= self.stamps[-1]:
            self.invalid += 1
        else:
            self.stamps.append(stamp)
        self.wall.append(now)
        if width and height:
            self.size = [int(width), int(height)]

    def status(self):
        sim,jitter = frame_rate(self.stamps)
        wall,_ = frame_rate(self.wall)
        # Bytes over the actual receive-time interval, not requested FPS.
        # This is delivered ROS image DATA throughput, not Gazebo native
        # network wire rate or DDS framing overhead.
        rate_window = (self.payload[-1][0]-self.payload[0][0]
                       if len(self.payload) > 1 else 0)
        payload_rate = (sum(x[1] for x in list(self.payload)[1:]) /
                        rate_window / 1e6) if rate_window > 0 else None
        return {
            "samples": self.n, "sim_fps": sim, "wall_fps": wall,
            "payload_mb_s": round(payload_rate, 3) if payload_rate is not None else None,
            "payload_total_mb": round(self.total_bytes / 1e6, 3),
            "payload_mean_mb": round(self.payload_sum / len(self.payload) / 1e6, 3)
                if self.payload else None,
            "jitter_ms": jitter, "effective_rtf": round(wall/sim,3) if sim and wall else None,
            "age_s": round(time.monotonic()-self.wall[-1],2) if self.wall else None,
            "nonmonotonic": self.invalid, "resolution": self.size,
        }

def bottleneck(streams, requested, mode):
    if mode == "world_smoke":
        return "Physics-only Gazebo smoke test; no rendering or cameras required."
    rgb = streams.get("rgb", {})
    depth = streams.get("depth", {})
    det = streams.get("tag" if mode == "apriltag" else "yolo", {})
    if not rgb.get("samples"):
        return "Waiting for camera frames: inspect Gazebo and image bridge"
    if rgb.get("age_s") is not None and rgb["age_s"] > 3:
        return "RGB stream stalled"
    sim,wall = rgb.get("sim_fps"),rgb.get("wall_fps")
    if sim and sim < requested * 0.8:
        return "Camera/rendering/ROS-bridge stage below requested sim FPS"
    if wall and wall < requested * 0.8:
        return "Gazebo slower than real time (compare effective RTF)"
    if mode in ("apriltag","yolo") and det.get("samples",0) < 3:
        return "Detector has not produced enough outputs"
    if mode not in ("world_smoke", "rgb_smoke") and depth.get("samples",0) and depth.get("sim_fps"):
        if depth["sim_fps"] < requested * 0.8:
            return "Depth stream below target sim FPS"
    return "No clear bottleneck at the selected profile"
