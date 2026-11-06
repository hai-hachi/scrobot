#!/usr/bin/env python3
"""Pure-Python, offline-testable lighting lab scene configuration.

The source badminton_court.sdf is NEVER modified. Only a generated world in
/tmp (or the caller's chosen output directory) is changed.
"""
from __future__ import annotations

import colorsys
import copy
import math
import xml.etree.ElementTree as ET
from pathlib import Path


WORLD = "badminton_court"
MAX_LIGHTS = 32

DEFAULTS = {
    "lights": {
        "rows": 2, "cols": 3, "span_x": 9.0, "span_y": 9.2,
        "height": 7.0, "kind": "spot", "intensity": 1.0,
        "range": 18.0, "constant": 0.45, "linear": 0.015,
        "quadratic": 0.001, "inner": 0.70, "outer": 1.20,
        "falloff": 0.8, "shadows": True,
        "hsv": [55, 3, 95], "specular": 0.20,
    },
    "colors": {
        "floor": [210, 12, 8], "walls": [240, 7, 15],
        "court": [147, 84, 35], "poles": [0, 0, 15],
        "pole_diameter": 0.05,
    },
    "robot": {"x": 2.0, "y": 3.05, "z": 0.003, "yaw": 180.0},
    "free": {"x": 8.0, "y": -8.0, "z": 6.0, "yaw": 135.0, "pitch": 24.0},
    "shuttles": {"count": 20, "seed": 20261003, "density": "uniform"},
}


def default_settings():
    return copy.deepcopy(DEFAULTS)


def hsv_rgb(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("HSV must contain [hue 0..360, saturation 0..100, value 0..100]")
    h, s, v = map(float, value)
    if not all(math.isfinite(x) for x in (h, s, v)) or not (
        0 <= h <= 360 and 0 <= s <= 100 and 0 <= v <= 100
    ):
        raise ValueError("HSV outside allowed range")
    return colorsys.hsv_to_rgb((h % 360) / 360, s / 100, v / 100)


def rgb_text(rgb, alpha=True):
    return " ".join(f"{x:.5f}" for x in (*rgb, 1.0) if alpha) if alpha else " ".join(
        f"{x:.5f}" for x in rgb
    )


def bounded(value, lo, hi, name):
    try:
        value = float(value)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"{name} must be a number") from exc
    if not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError(f"{name} must be between {lo} and {hi}")
    return value


def integer(value, lo, hi, name):
    v = bounded(value, lo, hi, name)
    if int(v) != v:
        raise ValueError(f"{name} must be an integer")
    return int(v)


def checked_lights(raw):
    out = dict(DEFAULTS["lights"])
    out.update(raw)
    out["rows"] = integer(out["rows"], 1, 8, "rows")
    out["cols"] = integer(out["cols"], 1, 12, "cols")
    if out["rows"] * out["cols"] > MAX_LIGHTS:
        raise ValueError(f"Maximum {MAX_LIGHTS} lights (protect Gazebo performance)")
    for key, lo, hi in (
        ("span_x", 0, 20), ("span_y", 0, 16), ("height", 2, 13),
        ("intensity", 0, 8), ("range", 2, 40), ("constant", 0.01, 2),
        ("linear", 0, 1), ("quadratic", 0, 0.15),
        ("inner", 0.05, 2.5), ("outer", 0.10, 3.0),
        ("falloff", 0, 2), ("specular", 0, 1),
    ):
        out[key] = bounded(out[key], lo, hi, key)
    if out["inner"] > out["outer"]:
        raise ValueError("Spot inner angle cannot exceed outer angle")
    if out["kind"] not in ("spot", "point"):
        raise ValueError("Light kind must be spot or point")
    if not isinstance(out["shadows"], bool):
        raise ValueError("shadows must be boolean")
    hsv_rgb(out["hsv"])
    return out


def checked_colors(raw):
    out = dict(DEFAULTS["colors"])
    out.update(raw)
    for name in ("floor", "walls", "court", "poles"):
        hsv_rgb(out[name])
    out["pole_diameter"] = bounded(out["pole_diameter"], 0.015, 0.15, "pole_diameter")
    return out


def centered_positions(light):
    rows, cols = light["rows"], light["cols"]
    for r in range(rows):
        y = (r / (rows - 1) - .5) * light["span_y"] if rows > 1 else 0.0
        for c in range(cols):
            x = (c / (cols - 1) - .5) * light["span_x"] if cols > 1 else 0.0
            yield f"lab_light_{r:02d}_{c:02d}", x, y, light["height"]


def light_direction(x, y, z):
    # Aim towards the nearest point on the middle service corridor.
    dx, dy, dz = 0.0, -y, -z
    magnitude = math.sqrt(dx * dx + dy * dy + dz * dz)
    return dx / magnitude, dy / magnitude, dz / magnitude


def make_light(name, x, y, z, settings):
    root = ET.Element("light", type=settings["kind"], name=name)
    ET.SubElement(root, "pose").text = f"{x:.4f} {y:.4f} {z:.4f} 0 0 0"
    rgb = hsv_rgb(settings["hsv"])
    ET.SubElement(root, "diffuse").text = rgb_text(rgb)
    ET.SubElement(root, "specular").text = rgb_text(
        tuple(min(1, v * settings["specular"]) for v in rgb)
    )
    ET.SubElement(root, "intensity").text = str(settings["intensity"])
    ET.SubElement(root, "cast_shadows").text = str(settings["shadows"]).lower()
    ET.SubElement(root, "direction").text = rgb_text(light_direction(x, y, z), False)
    attenuation = ET.SubElement(root, "attenuation")
    for key, tag in (("range", "range"), ("constant", "constant"),
                     ("linear", "linear"), ("quadratic", "quadratic")):
        ET.SubElement(attenuation, tag).text = str(settings[key])
    if settings["kind"] == "spot":
        spot = ET.SubElement(root, "spot")
        for key, tag in (("inner", "inner_angle"), ("outer", "outer_angle"),
                         ("falloff", "falloff")):
            ET.SubElement(spot, tag).text = str(settings[key])
    return root


def material_for(rgb):
    material = ET.Element("material")
    ET.SubElement(material, "ambient").text = rgb_text(tuple(0.8 * v for v in rgb))
    ET.SubElement(material, "diffuse").text = rgb_text(rgb)
    ET.SubElement(material, "specular").text = "0.02 0.02 0.02 1"
    return material


VISUAL_GROUPS = {
    "floor": [("court_floor", "venue_floor_visual")],
    "court": [("court_floor", "badminton_mat_visual")],
    "walls": [("simulation_safety_boundary", f"wall_{axis}_{sign}_visual")
              for axis in ("x", "y") for sign in ("positive", "negative")],
    "poles": [("net_assembly", "left_post_visual"),
              ("net_assembly", "right_post_visual")],
}


def find_visual(world, model_name, visual_name):
    for model in world.findall("model"):
        if model.get("name") == model_name:
            for visual in model.findall(".//visual"):
                if visual.get("name") == visual_name:
                    return visual
    raise ValueError(f"Missing world visual {model_name}/{visual_name}")


def configure_model(world, colors):
    for group, members in VISUAL_GROUPS.items():
        rgb = hsv_rgb(colors[group])
        for model, name in members:
            visual = find_visual(world, model, name)
            old = visual.find("material")
            if old is not None:
                visual.remove(old)
            visual.append(material_for(rgb))
    # Update visual AND collision dimensions for both poles.
    for model in world.findall("model"):
        if model.get("name") == "net_assembly":
            for child in model.findall(".//visual") + model.findall(".//collision"):
                if "post_" not in child.get("name", ""):
                    continue
                radius = child.find("./geometry/cylinder/radius")
                if radius is not None:
                    radius.text = f"{colors['pole_diameter'] / 2:.6f}"
            return
    raise ValueError("Missing net_assembly")


def camera_entity(name, topic, pose, resolution=(800, 450), hz=8.0, hfov=1.6):
    model = ET.Element("model", name=name)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = " ".join(f"{v:.6f}" for v in pose)
    link = ET.SubElement(model, "link", name="camera_mount")
    sensor = ET.SubElement(link, "sensor", name="rgb", type="camera")
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = str(hz)
    ET.SubElement(sensor, "topic").text = topic
    camera = ET.SubElement(sensor, "camera")
    ET.SubElement(camera, "horizontal_fov").text = str(hfov)
    image = ET.SubElement(camera, "image")
    ET.SubElement(image, "width").text = str(resolution[0])
    ET.SubElement(image, "height").text = str(resolution[1])
    ET.SubElement(image, "format").text = "R8G8B8"
    clip = ET.SubElement(camera, "clip")
    ET.SubElement(clip, "near").text = "0.1"
    ET.SubElement(clip, "far").text = "40"
    return model


def build_world(source, settings):
    """Return a complete derivative world; preserve the real world source file."""
    lights = checked_lights(settings.get("lights", {}))
    colors = checked_colors(settings.get("colors", {}))
    root = ET.parse(source).getroot()
    world = root.find("world")
    if world is None or world.get("name") != WORLD:
        raise ValueError("Expected badminton_court Gazebo world")
    for original in list(world.findall("light")):
        world.remove(original)
    for name, x, y, z in centered_positions(lights):
        world.append(make_light(name, x, y, z, lights))
    configure_model(world, colors)
    # Gazebo cameras look along local +X; positive pitch looks downward.
    world.append(camera_entity("lab_overhead_camera", "/lighting_lab/overhead",
                               (10.5, 0, 9.5, 0, .736, math.pi),
                               resolution=(960, 540), hz=8.0, hfov=1.56))
    free = settings.get("free", DEFAULTS["free"])
    pose = (bounded(free["x"], -18, 18, "free.x"),
            bounded(free["y"], -14, 14, "free.y"),
            bounded(free["z"], 1, 16, "free.z"),
            0, math.radians(bounded(free["pitch"], -60, 88, "free.pitch")),
            math.radians(bounded(free["yaw"], -360, 360, "free.yaw")))
    world.append(camera_entity("lab_free_camera", "/lighting_lab/free", pose,
                               resolution=(800, 450), hz=8.0, hfov=1.3))
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode")


def net_model_xml(source, diameter, pole_hsv):
    root = ET.parse(source).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError("Missing world")
    clean = ET.Element("sdf", version="1.10")
    for model in world.findall("model"):
        if model.get("name") == "net_assembly":
            clean.append(copy.deepcopy(model))
            break
    if not list(clean):
        raise ValueError("Missing net assembly")
    tmpworld = ET.Element("world", name=WORLD)
    tmpworld.append(list(clean)[0])
    configure_net = {"poles": pole_hsv, "pole_diameter": diameter}
    for child in tmpworld.findall(".//visual") + tmpworld.findall(".//collision"):
        if "post_" in child.get("name", ""):
            radius = child.find("./geometry/cylinder/radius")
            if radius is not None:
                radius.text = f"{diameter/2:.6f}"
        if child.get("name") in ("left_post_visual", "right_post_visual"):
            old = child.find("material")
            if old is not None:
                child.remove(old)
            child.append(material_for(hsv_rgb(configure_net["poles"])))
    clean.remove(list(clean)[0])
    clean.append(tmpworld.find("model"))
    return ET.tostring(clean, encoding="unicode")
