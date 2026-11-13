"""Use the saved Lighting Lab YAML for all Camera/AprilTags scene rendering.

The Lighting Lab's own helpers remain the authority for light placement,
shadow selection, materials and venue. Do not derive light parameters from
a previously published production SDF: unsaved values never become shared.
"""
from __future__ import annotations

import copy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tools/lighting_lab"))
from lab_configs import lighting_for_consumer, path_for  # noqa: E402
from config import validated_calibration  # noqa: E402
from scene import (centered_positions, make_light, configure_model,
                   configure_venue, hsv_rgb, material_for)  # noqa: E402


def calibration():
    return validated_calibration(lighting_for_consumer())


def apply_lighting(world, data=None):
    """Replace the default world light(s) with the saved calibrated array."""
    data = calibration() if data is None else validated_calibration(data)
    for light in list(world.findall("light")):
        world.remove(light)
    for name, x, y, z in centered_positions(data["lights"]):
        world.append(make_light(name, x, y, z, data["lights"]))
    return data


def apply_simple_world(world, data=None):
    """Keep Camera Lab's minimal test objects but use saved light/venue/color."""
    settings = apply_lighting(world, data)
    floor = world.find('./model[@name="floor"]')
    if floor is None:
        raise ValueError("Missing Camera Lab floor")
    length = 15.4 + 2 * settings["venue"]["clearance_x"]
    width = 7.2 + 2 * settings["venue"]["clearance_y"]
    for part in floor.findall(".//box/size"):
        part.text = f"{length:.4f} {width:.4f} 0.1"
    for visual in floor.findall(".//visual"):
        previous = visual.find("material")
        if previous is not None:
            visual.remove(previous)
        visual.append(material_for(hsv_rgb(settings["colors"]["floor"]), "floor"))
    return settings


def apply_court_world(world, data=None):
    """Match Lighting Lab's entire court: material groups, walls, venue, lights."""
    settings = apply_lighting(world, data)
    configure_venue(world, settings["venue"])
    configure_model(world, settings["colors"])
    return settings
