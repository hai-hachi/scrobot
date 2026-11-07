#!/usr/bin/env python3
"""Read/write the Lighting Lab's own YAML defaults, never the production world.

The dashboard saves only light, material and venue calibration. Camera poses
and shuttle arrangements remain experiment-specific JSON preset data.
"""
from __future__ import annotations

import os
from pathlib import Path
import tempfile

from scene import checked_colors, checked_lights, checked_venue

LAB_SECTIONS = ("lights", "colors", "venue")


def _yaml():
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError(
            "Lighting Lab YAML support requires: sudo apt install python3-yaml"
        ) from exc
    return yaml


def validated_calibration(raw):
    if not isinstance(raw, dict):
        raise ValueError("Lighting Lab config must be a YAML/JSON mapping")
    unknown = set(raw) - set(LAB_SECTIONS)
    if unknown:
        raise ValueError("Unknown Lighting Lab config sections: " + ", ".join(sorted(unknown)))
    for name in LAB_SECTIONS:
        if name not in raw or not isinstance(raw[name], dict):
            raise ValueError(f"Missing or invalid config section: {name}")
    return {
        "lights": checked_lights(raw["lights"]),
        "colors": checked_colors(raw["colors"]),
        "venue": checked_venue(raw["venue"]),
    }


def read_lab_config(path: Path):
    """Load saved calibration defaults, or use code defaults if absent."""
    if not path.exists():
        return {}
    raw = _yaml().safe_load(path.read_text(encoding="utf-8"))
    if raw is None:
        raise ValueError(f"Lighting Lab YAML is empty: {path}")
    return validated_calibration(raw)


def write_lab_config(path: Path, raw):
    """Validate and atomically update the fixed Lighting Lab config YAML."""
    clean = validated_calibration(raw)
    output = (
        "# SCROBOT lighting-lab calibration defaults (not production ROS params).\n"
        "# Updated only via the dashboard's Update lab config YAML button.\n"
        "# Changes are loaded when the Lighting Lab is next started.\n"
        + _yaml().safe_dump(clean, sort_keys=False, allow_unicode=True)
    )
    tmp_name = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent,
            prefix=".lighting_lab_", suffix=".yaml.tmp", delete=False,
        ) as tmp:
            tmp_name = tmp.name
            tmp.write(output)
            tmp.flush()
            os.fsync(tmp.fileno())
        os.replace(tmp_name, path)
        tmp_name = None
    finally:
        if tmp_name is not None:
            Path(tmp_name).unlink(missing_ok=True)
    return path
