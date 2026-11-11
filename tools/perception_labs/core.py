"""Pure validation, telemetry statistics and pose math for Perception Labs."""
from __future__ import annotations

from collections import deque
import bisect
import math
import statistics
import time

COLOR_PROFILES = {
    "640x360": (640, 360),
    "640x480": (640, 480),
    "848x480": (848, 480),
    "1280x720": (1280, 720),
}
DEPTH_PROFILES = {
    "424x240": (424, 240),
    "640x480": (640, 480),
    "848x480": (848, 480),
    "1280x720": (1280, 720),
}
FPS_OPTIONS = (5, 15, 30, 60)
MODES = ("camera", "apriltag", "yolo")
DEFAULTS = {
    "mode": "camera", "color_mode": "1280x720", "depth_mode": "848x480",
    "color_fps": 15, "depth_fps": 15, "enable_depth": True,
    "robot_x": 2.0, "robot_y": 0.0, "robot_yaw_deg": 0.0,
    "shuttle_x": 3.0, "shuttle_y": 0.0, "shuttle_count": 5,
    "model_path": "", "device": "0", "max_detection_rate": 15,
    "dataset_images": 1200, "dataset_shuttles": 40,
    "dataset_positive_fraction": 0.85, "dataset_seed": 42,
    "train_epochs": 15, "train_batch": 8, "train_imgsz": 640,
    "train_lr": 0.0001,
}
PHYSICAL_BOUNDS = {
    "robot_x": (-6.25, 6.25), "robot_y": (-2.75, 2.75),
    "robot_yaw_deg": (-180, 180),
    "shuttle_x": (-6.25, 6.25), "shuttle_y": (-2.75, 2.75),
}
INT_BOUNDS = {
    "shuttle_count": (1, 50), "max_detection_rate": (1, 60),
    "dataset_images": (100, 100000), "dataset_shuttles": (1, 100),
    "dataset_seed": (0, 10000000), "train_epochs": (1, 300),
    "train_batch": (1, 64), "train_imgsz": (320, 1280),
}


def validate(raw):
    if not isinstance(raw, dict) or set(raw) - set(DEFAULTS):
        raise ValueError("Invalid Perception Labs settings or unknown keys")
    result = dict(DEFAULTS)
    result.update(raw)
    if result["mode"] not in MODES:
        raise ValueError("Unknown perception experiment")
    if result["color_mode"] not in COLOR_PROFILES:
        raise ValueError("Unsupported RGB resolution")
    if result["depth_mode"] not in DEPTH_PROFILES:
        raise ValueError("Unsupported depth resolution")
    for name in ("color_fps", "depth_fps"):
        if type(result[name]) is not int or result[name] not in FPS_OPTIONS:
            raise ValueError(f"{name} must be one of {FPS_OPTIONS}")
    if type(result["enable_depth"]) is not bool:
        raise ValueError("enable_depth must be boolean")
    for name, (low, high) in PHYSICAL_BOUNDS.items():
        if isinstance(result[name], bool):
            raise ValueError(name + " must be numeric")
        v = float(result[name])
        if not math.isfinite(v) or not low <= v <= high:
            raise ValueError(f"{name} outside allowed court pose [{low}, {high}]")
        result[name] = v
    for name, (low, high) in INT_BOUNDS.items():
        if type(result[name]) is not int or not low <= result[name] <= high:
            raise ValueError(f"{name} must be an integer within [{low}, {high}]")
    for name, low, high in (
        ("dataset_positive_fraction", 0.0, 1.0),
        ("train_lr", 0.000001, 0.01),
    ):
        if isinstance(result[name], bool):
            raise ValueError(name + " must be numeric")
        v = float(result[name])
        if not math.isfinite(v) or not low <= v <= high:
            raise ValueError(name + " invalid")
        result[name] = v
    if result["train_imgsz"] % 32:
        raise ValueError("train_imgsz must be divisible by 32")
    for key in ("model_path", "device"):
        if not isinstance(result[key], str) or len(result[key]) > 512:
            raise ValueError(key + " invalid")
    if not result["device"] or not all(x.isalnum() or x in ":-,_" for x in result["device"]):
        raise ValueError("Invalid device identifier")
    return result


def camera_args(settings):
    s = validate(settings)
    cw, ch = COLOR_PROFILES[s["color_mode"]]
    dw, dh = DEPTH_PROFILES[s["depth_mode"]]
    return {
        "color_width": cw, "color_height": ch, "color_fps": s["color_fps"],
        "depth_width": dw, "depth_height": dh, "depth_fps": s["depth_fps"],
    }


def pose_request(entity, x, y, yaw_deg, z=0.003):
    """Generate typed gz.msgs.Pose protobuf text. Validated names and floats."""
    if not entity or not all(ch.isalnum() or ch in "_-" for ch in entity):
        raise ValueError("Invalid entity name")
    x, y, yaw_deg, z = map(float, (x, y, yaw_deg, z))
    if not all(map(math.isfinite, (x, y, yaw_deg, z))):
        raise ValueError("Nonfinite pose")
    if not -7 <= x <= 7 or not -3.5 <= y <= 3.5 or not -180 <= yaw_deg <= 180:
        raise ValueError("Pose outside permitted evaluation area")
    angle = math.radians(yaw_deg) / 2
    return (f'name: "{entity}", position: {{x: {x:.6f}, y: {y:.6f}, z: {z:.6f}}}, '
            f'orientation: {{w: {math.cos(angle):.8f}, z: {math.sin(angle):.8f}}}')


class StreamMeter:
    """Separate sim-time header rate from wall arrival rate and freshness."""
    def __init__(self, maxlen=240):
        self.stamps = deque(maxlen=maxlen)
        self.arrivals = deque(maxlen=maxlen)
        self.count = 0
        self.frame = ""
        self.width = None
        self.height = None
        self.bytes = 0
        self.duplicates = 0
        self.out_of_order = 0
        self.delay_ms = deque(maxlen=maxlen)

    def add(self, stamp, frame="", width=None, height=None, bytes_len=0,
            sim_now=None):
        if not math.isfinite(stamp) or stamp < 0:
            return
        now = time.monotonic()
        self.count += 1
        self.frame = str(frame)
        self.bytes = int(bytes_len)
        if width is not None: self.width = int(width)
        if height is not None: self.height = int(height)
        if self.stamps:
            if stamp == self.stamps[-1]:
                self.duplicates += 1
            elif stamp < self.stamps[-1]:
                self.out_of_order += 1
        if not self.stamps or stamp > self.stamps[-1]:
            self.stamps.append(stamp)
        self.arrivals.append(now)
        if sim_now is not None and math.isfinite(sim_now) and sim_now >= stamp:
            self.delay_ms.append((sim_now - stamp) * 1000)

    def snapshot(self):
        def rate(times):
            if len(times) < 3:return None, None
            diffs = [b-a for a,b in zip(times,list(times)[1:]) if b>a]
            if len(diffs) < 2:return None, None
            return (round(len(diffs)/sum(diffs),2),
                    round(statistics.pstdev(diffs)*1000,3))
        sim_rate, jitter = rate(self.stamps)
        wall_rate, _ = rate(self.arrivals)
        age = time.monotonic() - self.arrivals[-1] if self.arrivals else None
        return {
            "count": self.count, "sim_fps": sim_rate, "wall_fps": wall_rate,
            "jitter_ms": jitter, "age_wall_s": round(age,2) if age is not None else None,
            "frame": self.frame, "width": self.width, "height": self.height,
            "bytes": self.bytes, "duplicates": self.duplicates,
            "out_of_order": self.out_of_order,
            "transport_sim_delay_ms_p95": round(sorted(self.delay_ms)[int(.95*(len(self.delay_ms)-1))],2)
            if self.delay_ms else None,
        }


def timing_summary(streams, settings):
    expected = {
        "rgb": settings["color_fps"],
        "depth": settings["depth_fps"] if settings["enable_depth"] else 0,
        "apriltag": settings["color_fps"] if settings["mode"] == "apriltag" else 0,
        "yolo2d": min(settings["color_fps"], settings["max_detection_rate"])
        if settings["mode"] == "yolo" else 0,
    }
    metrics = {key: obj.snapshot() for key, obj in streams.items()}
    # Determining bottlenecks from measured rates requires steady-state
    # samples; lacking data must never be interpreted as zero throughput.
    def ratio(key):
        m = metrics.get(key, {})
        return (m["wall_fps"] / expected[key]) if expected.get(key) and m.get("wall_fps") is not None else None
    if ratio("rgb") is not None and ratio("rgb") < .8:
        bottleneck = "camera rendering / ROS bridge (RGB delivery)"
    elif settings["enable_depth"] and ratio("depth") is not None and ratio("depth") < .8:
        bottleneck = "depth camera rendering / bridge"
    elif settings["mode"] == "apriltag" and ratio("apriltag") is not None and ratio("apriltag") < .8:
        bottleneck = "AprilTag detection"
    elif settings["mode"] == "yolo" and ratio("yolo2d") is not None and ratio("yolo2d") < .8:
        bottleneck = "YOLO inference / RGB-depth synchronization (not isolated)"
    else:
        bottleneck = "no confirmed bottleneck; review RTF and matched stages"
    return {"streams":metrics,"bottleneck_hint":bottleneck,
            "note":"Delivered FPS vs requested FPS; sim-time delay is not wall-clock end-to-end latency."}
