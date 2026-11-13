"""Pure, validated per-lab configuration (camera settings belong to Camera Lab)."""
from __future__ import annotations

import math

DEFAULT = {
    "shuttle_count": 12,
    "spawn_mode": "random",
    "random_density": "uniform",
    "random_fraction": 0.10,
    "single_x": 0.0,
    "single_y": 0.0,
    "cluster_sigma_x": 0.10,
    "cluster_sigma_y": 0.90,
    "cluster_left_y": 1.525,
    "cluster_right_y": -1.525,
    "cluster_line_x": [6.70, 5.94, 1.98, 0.0, -1.98, -5.94, -6.70],
    "cluster_weights": [[1.0,0.0],[0.0,1.0],[0.2,0.8],[1.0,0.3],
                        [0.7,0.3],[0.0,1.0],[1.0,0.0]],
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
    "confidence": 0.10,
}

def validate(raw):
    if not isinstance(raw, dict) or set(raw) - set(DEFAULT):
        raise ValueError("Invalid YOLO Lab YAML settings fields")
    result = {**DEFAULT, **raw}
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
    line_x = result["cluster_line_x"]
    if not isinstance(line_x, list) or len(line_x) != 7 or any(
            type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 7
            for v in line_x):
        raise ValueError("Invalid cluster line coordinates")
    result["cluster_line_x"] = [float(v) for v in line_x]
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
        ("random_fraction", 0, 1),
        ("single_x", -6.7, 6.7),
        ("single_y", -3.05, 3.05),
        ("cluster_sigma_x", 0, 4),
        ("cluster_sigma_y", 0, 4),
        ("cluster_left_y", 0, 3.05),
        ("cluster_right_y", -3.05, 0),
        ("settle_seconds", 0.5, 15.0),
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
