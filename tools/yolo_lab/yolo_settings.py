"""Pure, validated per-lab configuration (camera settings belong to Camera Lab)."""
from __future__ import annotations

import math

DEFAULT = {
    "shuttle_count": 12,
    "seed": 42,
    "robot_x": 2.0,
    "robot_y": 0.0,
    "robot_yaw": 0.0,
    "dataset_images": 1200,
    "positive_fraction": 0.85,
    "epochs": 15,
    "batch": 8,
    "imgsz": 640,
    "confidence": 0.10,
}

def validate(raw):
    if not isinstance(raw, dict) or set(raw) != set(DEFAULT):
        raise ValueError("Invalid YOLO Lab YAML settings fields")
    result = dict(raw)
    for key, lo, hi in (
        ("shuttle_count", 1, 60),
        ("seed", 0, 2147483647),
        ("dataset_images", 50, 50000),
        ("epochs", 1, 300),
        ("batch", 1, 64),
        ("imgsz", 320, 1920),
    ):
        value = result[key]
        if type(value) is not int or not lo <= value <= hi:
            raise ValueError(f"{key} must be an integer in [{lo}, {hi}]")
    if result["imgsz"] % 32:
        raise ValueError("imgsz must be a multiple of 32")
    for key, lo, hi in (
        ("robot_x", -15.0, 15.0),
        ("robot_y", -12.0, 12.0),
        ("robot_yaw", -360.0, 360.0),
        ("positive_fraction", 0.0, 1.0),
        ("confidence", 0.001, 1.0),
    ):
        value = result[key]
        if type(value) not in (float, int) or not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError(f"{key} must be a finite number in [{lo}, {hi}]")
        result[key] = float(value)
    return result
