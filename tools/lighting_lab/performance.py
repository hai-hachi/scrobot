"""Pure helpers for low-overhead Lighting Lab updates and preview settings.

Fast/accurate modes affect ONLY dashboard JPEG processing, not camera
sensors, Gazebo rendering resolution, or calibration output.
"""
from __future__ import annotations

import copy

PREVIEW_PROFILES = {
    "fast": {"min_period": 0.38, "max_size": (640, 360), "jpeg_quality": 60,
             "poll_ms": 600},
    "accurate": {"min_period": 0.18, "max_size": (860, 490), "jpeg_quality": 82,
                 "poll_ms": 380},
}


def changed_fields(previous, current):
    """Report values changed since the last successfully applied operation."""
    if previous is None:
        return set(current)
    return {key for key in set(previous) | set(current)
            if previous.get(key) != current.get(key)}


def dirty_material_groups(previous, current):
    """Only send visual_config for materials whose colors changed."""
    return [key for key in ("floor", "court", "walls", "poles")
            if previous is None or previous.get(key) != current.get(key)]


def preview_world_settings(settings, shadow_free):
    """Create a temporary SDF settings copy without changing saved calibration."""
    result = copy.deepcopy(settings)
    if shadow_free:
        result["lights"]["shadows"] = False
    return result
