"""Validated, lab-only mesh inspection settings and saved lighting inputs."""
from __future__ import annotations

import copy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.shared import lab_configs
from tools.lighting_lab.scene import checked_lights, checked_colors, checked_venue

DEFAULTS = {
    "model": "both",
    "robot_visual_mode": "full",
    "shuttle_count": 1,
    "shuttle_seed": 20261115,
    "lighting_enabled": True,
    "shadows_enabled": True,
    "depth_camera": False,
    "benchmark_seconds": 15,
}


def validate(raw):
    """No silent coercion of browser strings or imported YAML."""
    if not isinstance(raw, dict) or set(raw) != set(DEFAULTS):
        raise ValueError("Mesh Lab settings must contain exactly: " + ", ".join(DEFAULTS))
    data = copy.deepcopy(raw)
    if data["model"] not in ("robot", "shuttle", "both"):
        raise ValueError("model must be robot, shuttle or both")
    if data["robot_visual_mode"] not in ("full", "simple", "none"):
        raise ValueError("robot_visual_mode must be full, simple or none")
    for name in ("lighting_enabled", "shadows_enabled", "depth_camera"):
        if type(data[name]) is not bool:
            raise ValueError(name + " must be boolean")
    for name, low, high in (
        ("shuttle_count", 1, 50),
        ("shuttle_seed", 0, 2147483647),
        ("benchmark_seconds", 5, 60),
    ):
        if type(data[name]) is not int or not low <= data[name] <= high:
            raise ValueError(f"{name} must be an integer between {low} and {high}")
    return data


def read_settings():
    saved = lab_configs.read("mesh")
    return validate(saved if saved is not None else DEFAULTS)


def save_settings(data):
    return lab_configs.save("mesh", data, validator=validate)


def read_lighting():
    """Consume the Lighting Lab's saved YAML, not its production publication."""
    raw = lab_configs.lighting_for_consumer()
    return {
        "lights": checked_lights(raw["lights"]),
        "colors": checked_colors(raw["colors"]),
        "venue": checked_venue(raw["venue"]),
    }
