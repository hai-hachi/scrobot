"""Validated YOLO Lab choices; shuttle geometry belongs to shuttle_spawn.yaml."""
from __future__ import annotations

import math

from distribution import geometry

# These geometry keys were previously copied into YOLO Lab YAML. Accept but
# ignore them so old saved labs cannot silently override production geometry.
LEGACY_GEOMETRY_FIELDS = frozenset({
    "single_x", "single_y", "cluster_sigma_x", "cluster_sigma_y",
    "cluster_left_y", "cluster_right_y", "cluster_line_x",
})

DEFAULT = {
    "shuttle_count": 12,
    "spawn_mode": "random",
    "random_density": "uniform",
    "random_fraction": 0.10,
    "cluster_weights": geometry()["cluster"]["weights"],
    "settle_seconds": 3.0,
    "seed": 42,
    "robot_x": 2.0,
    "robot_y": 0.0,
    "robot_yaw": 0.0,
    "dataset_images": 1200,
    "positive_fraction": 0.85,
    "epochs": 15,
    "batch": 8,
    "imgsz": 640,
    "validation_imgsz": 640,
    "validation_batch": 8,
    "live_imgsz": 640,
    "confidence": 0.10,
}


def validate(raw):
    if not isinstance(raw, dict) or set(raw) - (
            set(DEFAULT) | LEGACY_GEOMETRY_FIELDS):
        raise ValueError("Invalid YOLO Lab YAML settings fields")
    result = {**DEFAULT, **{k: v for k, v in raw.items() if k in DEFAULT}}
    if result["spawn_mode"] not in ("single", "random", "cluster", "mixed"):
        raise ValueError("Invalid spawn mode")
    if result["random_density"] not in ("uniform", "center", "net"):
        raise ValueError("Invalid random density")
    weights = result["cluster_weights"]
    if not isinstance(weights, list) or len(weights) != 7 or any(
            not isinstance(row, list) or len(row) != 2 for row in weights):
        raise ValueError("cluster_weights must be a 7x2 list")
    parsed = []
    for row in weights:
        vals = []
        for item in row:
            if type(item) not in (int, float) or not math.isfinite(item) or item < 0:
                raise ValueError("Cluster weights must be finite and nonnegative")
            vals.append(float(item))
        parsed.append(vals)
    if not any(v > 0 for row in parsed for v in row):
        raise ValueError("At least one cluster weight must be positive")
    result["cluster_weights"] = parsed
    for key, low, high in (
        ("shuttle_count", 1, 60),
        ("seed", 0, 2147483647),
        ("dataset_images", 50, 50000),
        ("epochs", 1, 300),
        ("batch", 1, 64),
        ("imgsz", 320, 1920),
        ("validation_imgsz", 320, 1920),
        ("validation_batch", 1, 64),
        ("live_imgsz", 320, 1920),
    ):
        value = result[key]
        if type(value) is not int or not low <= value <= high:
            raise ValueError(f"{key} must be an integer in [{low}, {high}]")
    for name in ("imgsz", "validation_imgsz", "live_imgsz"):
        if result[name] % 32:
            raise ValueError(f"{name} must be a multiple of 32")
    for key, low, high in (
        ("random_fraction", 0, 1),
        ("settle_seconds", 0.5, 15.0),
        ("robot_x", -15.0, 15.0),
        ("robot_y", -12.0, 12.0),
        ("robot_yaw", -360.0, 360.0),
        ("positive_fraction", 0.0, 1.0),
        ("confidence", 0.001, 1.0),
    ):
        value = result[key]
        if type(value) not in (float, int) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"{key} must be a finite number in [{low}, {high}]")
        result[key] = float(value)
    return result
