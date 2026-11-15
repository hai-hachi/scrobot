"""Lab-only Gazebo speed cap, independent of physical calibration.

Every lab stores its requested maximum real-time factor here, but never
scales sensor / controller simulated-Hz parameters. At RTF=0.25 a 100 sim-Hz
source legitimately arrives at about 25 wall-Hz.
"""
from __future__ import annotations

import fcntl
import math
import os
from pathlib import Path
import re
import tempfile
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[2]
FILE = ROOT / "tools/configs/labs/simulation_speed.yaml"
LABS = ("lighting", "mobility", "odometry", "camera", "apriltags",
        "yolo", "mesh", "nav2")
DEFAULT = 0.25


def validate(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("RTF must be a finite number")
    result = float(value)
    if not 0.10 <= result <= 1.00:
        raise ValueError("RTF cap must be between 0.10 and 1.00")
    return round(result, 3)


def _lab(lab):
    if lab not in LABS:
        raise ValueError("Unknown lab for simulation speed: " + str(lab))
    return lab


def read_all():
    if not FILE.exists():
        return {}
    data = yaml.safe_load(FILE.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema_version") != 1 or data.get("lab") != "simulation_speed":
        raise ValueError("Invalid simulation_speed.yaml schema")
    values = data.get("settings")
    if not isinstance(values, dict) or any(name not in LABS for name in values):
        raise ValueError("Unexpected simulation speed keys")
    return {name: validate(value) for name, value in values.items()}


def get(lab):
    return read_all().get(_lab(lab), DEFAULT)


def save(lab, value):
    name = _lab(lab)
    speed = validate(value)
    FILE.parent.mkdir(parents=True, exist_ok=True)
    # A separate lock survives atomic replacements and handles several labs.
    lock_path = FILE.with_suffix(".lock")
    with lock_path.open("a+") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        values = read_all()
        values[name] = speed
        payload = yaml.safe_dump(
            {"schema_version": 1, "lab": "simulation_speed", "settings": values},
            sort_keys=False)
        temp = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w", dir=FILE.parent, prefix=".simulation_speed_",
                suffix=".tmp", encoding="utf-8", delete=False
            ) as stream:
                temp = Path(stream.name)
                stream.write("# Lab-only Gazebo RTF caps; never published as calibration.\n")
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp, FILE)
            temp = None
        finally:
            if temp is not None:
                temp.unlink(missing_ok=True)
        fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
    return speed


def cap_world(source, destination, max_rtf):
    """Copy and cap exactly one physics RTF element; preserve source world.

    Neither max_step_size nor any sensor's update_rate is changed. Return
    a path for passing into the lab's next Gazebo process.
    """
    speed = validate(max_rtf)
    src, dst = Path(source), Path(destination)
    original = src.read_text(encoding="utf-8")
    pattern = r"(<real_time_factor>)[^<]*(</real_time_factor>)"
    if len(re.findall(pattern, original)) != 1:
        raise ValueError(f"Expected exactly one <real_time_factor> in {src}")
    patched = re.sub(
        pattern, lambda match: match.group(1) + str(speed) + match.group(2),
        original, count=1)
    ET.fromstring(patched)
    dst.parent.mkdir(parents=True, exist_ok=True)
    temp = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", dir=dst.parent, prefix=".rtf_world_",
            suffix=".sdf.tmp", encoding="utf-8", delete=False
        ) as stream:
            temp = Path(stream.name)
            stream.write(patched)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, dst)
        temp = None
    finally:
        if temp is not None:
            temp.unlink(missing_ok=True)
    return str(dst)


def response(lab, running=False):
    return {"max_rtf": get(lab), "running": bool(running),
            "restart_required": bool(running),
            "note": "RTF is a Gazebo speed cap, not guaranteed achieved speed; "
                    "restart this lab simulator to apply. All topic targets "
                    "remain in simulation Hz; wall Hz is approximately sim Hz x measured RTF."}
