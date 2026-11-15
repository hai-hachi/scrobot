"""Persistent Nav2 Lab scene (separate from planner and upstream calibration YAML).

Coordinates refer to the badminton court's world/map frame. A saved scene is
loaded at simulation startup; unsaved changes never update the source labs.
"""
from copy import deepcopy
import math
import os
from pathlib import Path
import tempfile
import yaml

ROOT = Path(__file__).resolve().parents[3]
SCENE_PATH = ROOT / "tools/configs/labs/nav2_scene.yaml"
DEFAULT = {
    "robot": {"x": -3.0, "y": 0.0, "yaw_deg": 0.0},
    "obstacles": [],
}
MAX_OBSTACLES = 15


def number(value, field, lo, hi):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f"{field} must be a finite number")
    if not lo <= value <= hi:
        raise ValueError(f"{field} must be in [{lo}, {hi}]")
    return float(value)


def validate_scene(value):
    if not isinstance(value, dict) or set(value) != {"robot", "obstacles"}:
        raise ValueError("Scene must contain robot and obstacles only")
    p = value["robot"]
    if not isinstance(p, dict) or set(p) != {"x", "y", "yaw_deg"}:
        raise ValueError("Robot pose requires x, y and yaw_deg")
    robot = {
        "x": number(p["x"], "robot.x", -9.0, 9.0),
        "y": number(p["y"], "robot.y", -5.0, 5.0),
        "yaw_deg": number(p["yaw_deg"], "robot.yaw_deg", -180, 180),
    }
    obs = value["obstacles"]
    if not isinstance(obs, list) or len(obs) > MAX_OBSTACLES:
        raise ValueError(f"Scene must have 0..{MAX_OBSTACLES} obstacles")
    entries = []
    for i, o in enumerate(obs):
        if not isinstance(o, dict) or o.get("kind") not in ("box", "human"):
            raise ValueError(f"Obstacle {i}: kind must be box or human")
        kind = o["kind"]
        if kind == "box":
            if set(o) != {"kind", "x", "y", "size"}:
                raise ValueError(f"Box {i} must specify kind, x, y, size")
            entry = {
                "kind": "box",
                "x": number(o["x"], "box.x", -8.0, 8.0),
                "y": number(o["y"], "box.y", -3.6, 3.6),
                "size": number(o["size"], "box.size", .20, 1.50),
            }
        else:
            if set(o) != {"kind", "point_a", "point_b", "speed_mps"}:
                raise ValueError(f"Human {i} requires point_a, point_b, speed_mps")
            def point(src, label):
                if not isinstance(src, dict) or set(src) != {"x", "y"}:
                    raise ValueError(f"{label} requires x and y")
                return {"x":number(src["x"], label+".x", -8.0, 8.0),
                        "y":number(src["y"], label+".y", -3.6, 3.6)}
            a, b = point(o["point_a"], "point_a"), point(o["point_b"], "point_b")
            if math.hypot(b["x"]-a["x"], b["y"]-a["y"]) < .20:
                raise ValueError("Human route points must be at least 0.2 m apart")
            entry = {"kind":"human", "point_a":a, "point_b":b,
                     "speed_mps":number(o["speed_mps"], "human.speed_mps", .1, 2.0)}
        entries.append(entry)
    for i,o in enumerate(entries):
        # Avoid starting inside or directly overlapping the robot.
        if o["kind"] == "box":
            coords = [(o["x"],o["y"])]
            margin = .60 + o["size"]/2
        else:
            coords = [(o["point_a"]["x"],o["point_a"]["y"]),
                      (o["point_b"]["x"],o["point_b"]["y"])]
            margin = .85
        if any(math.hypot(x-robot["x"],y-robot["y"]) < margin
               for x,y in coords):
            raise ValueError(f"Obstacle {i} overlaps the initial robot position")
    return {"robot":robot, "obstacles":entries}


def load_scene(path=SCENE_PATH, default=None):
    location=Path(path)
    if not location.exists():
        return validate_scene(deepcopy(DEFAULT if default is None else default))
    data=yaml.safe_load(location.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema_version") != 1 or data.get("lab") != "nav2_scene":
        raise ValueError("Invalid Nav2 saved scene YAML")
    return validate_scene(data.get("settings"))


def save_scene(scene,path=SCENE_PATH):
    values=validate_scene(scene)
    target=Path(path)
    target.parent.mkdir(parents=True,exist_ok=True)
    doc={"schema_version":1,"lab":"nav2_scene","settings":values}
    text=yaml.safe_dump(doc,sort_keys=False)
    with tempfile.NamedTemporaryFile("w",dir=target.parent,
        prefix=".nav2_scene_",suffix=".tmp",delete=False) as handle:
        temp=Path(handle.name)
        handle.write("# Saved Nav2 scene; separate from planner and upstream calibration.\n")
        handle.write(text)
        handle.flush()
        os.fsync(handle.fileno())
    try:
        os.replace(temp,target)
    finally:
        temp.unlink(missing_ok=True)
    return values
