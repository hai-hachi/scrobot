"""Deterministic derived Gazebo world for geometry and shadow tests.

Never mutates the original court, shuttle SDF, robot Xacro or Lighting Lab YAML.
The model://shuttle resource uses the already-installed scrobot_simulation model.
"""
from __future__ import annotations

import copy
import random
import xml.etree.ElementTree as ET
from pathlib import Path

from tools.lighting_lab.scene import (
    WORLD, centered_positions, configure_model, configure_venue, make_light,
)
from tools.mesh_lab.settings import validate


def shuttle_poses(count, seed):
    """One easy-to-inspect shuttle or a reproducible spread without overlap."""
    if count == 1:
        return [(0.4, 0.0, 0.045, 0.0)]
    rng = random.Random(seed)
    candidates = [(x, y) for x in (-5.5, -4.0, -2.5, -1.0, 0.5, 2.0, 3.5, 5.0)
                  for y in (-2.2, -1.25, -0.3, 0.65, 1.6, 2.55)]
    rng.shuffle(candidates)
    # 48 grid sites; the last two are extra sites farther from center.
    candidates.extend([(-6.3, 0.0), (6.3, 0.0)])
    return [(x, y, 0.045, 0.0) for x, y in candidates[:count]]


def build_world(source: Path, lighting: dict, raw: dict) -> str:
    settings = validate(raw)
    root = ET.parse(source).getroot()
    world = root.find("world")
    if world is None or world.get("name") != WORLD:
        raise ValueError("Unexpected source world name")
    for old in list(world.findall("light")):
        world.remove(old)
    lights = copy.deepcopy(lighting["lights"])
    if not settings["shadows_enabled"]:
        lights["shadows"] = False
        lights["shadow_count"] = 0
    if settings["lighting_enabled"]:
        for name, x, y, z in centered_positions(lights):
            world.append(make_light(name, x, y, z, lights))
    configure_model(world, lighting["colors"])
    configure_venue(world, lighting["venue"])
    # Inspection and benchmarks must not remove shuttles through auto-pickup.
    for plugin in world.findall("plugin"):
        if plugin.get("filename") == "libshuttle_manager_system.so":
            enabled = plugin.find("enable_collection")
            if enabled is not None:
                enabled.text = "false"
    if settings["model"] in ("shuttle", "both"):
        for i, (x, y, z, yaw) in enumerate(
                shuttle_poses(settings["shuttle_count"], settings["shuttle_seed"]), 1):
            include = ET.SubElement(world, "include")
            ET.SubElement(include, "uri").text = "model://shuttle"
            ET.SubElement(include, "name").text = f"mesh_shuttle_{i:02d}"
            ET.SubElement(include, "pose").text = f"{x} {y} {z} 0 0 {yaw}"
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode")
