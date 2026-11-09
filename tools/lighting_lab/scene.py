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
INSPECTION_PROFILES = {
    "lean": {"overhead": ((640, 360), 4.0), "free": ((640, 360), 4.0)},
    "full": {"overhead": ((960, 540), 8.0), "free": ((800, 450), 8.0)},
}

DEFAULTS = {
    "lights": {
        "rows": 2, "cols": 4, "reach_x": 4.5, "reach_y": 4.6,
        "height": 7.0, "kind": "spot",
        "hsv": [55, 3, 95], "intensity": 1.0,
        "range": 18.0, "constant": 0.45, "linear": 0.015,
        "quadratic": 0.001, "inner": 0.70, "outer": 1.20,
        "falloff": 0.8, "shadows": True, "shadow_count": 2,
        "specular": 0.20,
    },
    "colors": {
        "floor": [210, 12, 8], "walls": [240, 7, 15],
        "court": [147, 84, 35], "poles": [0, 0, 15],
    },
    # Clearances are distances from each EDGE of the 15.4x7.2 m court mat
    # to the venue floor edge; the safety walls follow the venue floor.
    "venue": {"clearance_x": 2.3, "clearance_y": 2.4},
    "robot": {"x": 2.0, "y": 3.05, "z": 0.003, "yaw": 180.0},
    "free": {"x": 8.0, "y": -8.0, "z": 6.0, "yaw": 135.0, "pitch": 24.0},
    "shuttles": {"count": 20, "seed": 20261003, "density": "uniform"},
    # Lab-only rendering budget: these keys are never saved to lighting YAML.
    "inspection_profile": "lean",
    "robot_visual_mode": "simple",
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


def kelvin_rgb(temperature):
    """Approximate sRGB blackbody colour (for appearance, NOT photometric lux)."""
    t = bounded(temperature, 1800, 12000, "temperature") / 100.0
    red = 255.0 if t <= 66.0 else 329.698727446 * (t - 60.0) ** -0.1332047592
    green = (
        99.4708025861 * math.log(t) - 161.1195681661
        if t <= 66.0 else
        288.1221695283 * (t - 60.0) ** -0.0755148492
    )
    blue = (0.0 if t <= 19.0 else
            138.5177312231 * math.log(t - 10.0) - 305.0447927307
            if t <= 66.0 else 255.0)
    return tuple(max(0.0, min(255.0, v)) / 255.0 for v in (red, green, blue))


def rgb_to_hsv(rgb):
    """Convert normalized RGB into 0..360/0..100/0..100 HSV sliders."""
    h, s, v = colorsys.rgb_to_hsv(*rgb)
    return [round(h * 360, 3), round(s * 100, 3), round(v * 100, 3)]


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
    # Accept historic full-span and Kelvin/brightness calibration presets.
    raw = dict(raw)
    for old, new in (("span_x", "reach_x"), ("span_y", "reach_y")):
        if old in raw and new not in raw:
            raw[new] = float(raw[old]) / 2.0
        raw.pop(old, None)
    # HSV is authoritative whenever both old and new colour fields exist.
    if "temperature" in raw and "hsv" not in raw:
        raw["hsv"] = rgb_to_hsv(kelvin_rgb(raw["temperature"]))
    raw.pop("temperature", None)
    if "brightness" in raw and "intensity" not in raw:
        raw["intensity"] = float(raw["brightness"]) / 100.0
    raw.pop("brightness", None)
    out = copy.deepcopy(DEFAULTS["lights"])
    out.update(raw)
    out["rows"] = integer(out["rows"], 1, 8, "rows")
    out["cols"] = integer(out["cols"], 1, 12, "cols")
    # shadow_count is an upper bound: shrinking the grid never invalidates
    # a saved preset, and shadows=false still explicitly turns all off.
    out["shadow_count"] = integer(out["shadow_count"], 0, MAX_LIGHTS, "shadow_count")
    if out["rows"] * out["cols"] > MAX_LIGHTS:
        raise ValueError(f"Maximum {MAX_LIGHTS} lights (protect Gazebo performance)")
    for key, lo, hi in (
        ("reach_x", 0, 10), ("reach_y", 0, 8), ("height", 2, 13),
        ("intensity", 0, 4),
        ("range", 2, 40), ("constant", 0.01, 2),
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
    out["hsv"] = [float(value) for value in out["hsv"]]
    return out


def checked_colors(raw):
    out = dict(DEFAULTS["colors"])
    out.update(raw)
    # Preserve old HSV material presets, but remove the retired pole geometry
    # setting rather than carrying it into newly saved calibration results.
    out.pop("pole_diameter", None)
    for name in ("floor", "walls", "court", "poles"):
        hsv_rgb(out[name])
    return out


def centered_positions(light):
    rows, cols = light["rows"], light["cols"]
    for r in range(rows):
        y = (2 * r / (rows - 1) - 1) * light["reach_y"] if rows > 1 else 0.0
        for c in range(cols):
            x = (2 * c / (cols - 1) - 1) * light["reach_x"] if cols > 1 else 0.0
            yield f"lab_light_{r:02d}_{c:02d}", x, y, light["height"]


def shadow_light_names(light):
    """Choose spatially separated, center-biased shadow lights deterministically.

    In a 2x4 array with count=2, choose one near each court side, diagonally
    separated. The other six still illuminate normally but render no shadows.
    """
    if not light["shadows"] or light["shadow_count"] <= 0:
        return set()
    spots = list(centered_positions(light))
    count = min(light["shadow_count"], len(spots))
    if count == len(spots):
        return {name for name, *_ in spots}
    rx = max(float(light["reach_x"]), 1.0)
    ry = max(float(light["reach_y"]), 1.0)
    center = lambda p: (p[1] / rx) ** 2 + (p[2] / ry) ** 2
    selected = [min(spots, key=lambda p: (center(p), p[2], p[1]))]
    while len(selected) < count:
        remaining = [p for p in spots if p not in selected]
        def score(p):
            spacing = min(
                ((p[1] - other[1]) / rx) ** 2 +
                ((p[2] - other[2]) / ry) ** 2
                for other in selected
            )
            return spacing - 0.35 * center(p)
        selected.append(max(remaining, key=lambda p: (score(p), -center(p), p[2], p[1])))
    return {p[0] for p in selected}


def light_direction(x, y, z):
    # Ceiling grid lights point straight down. The previous inward Y aim
    # made the illuminated footprint bunch near court center even when
    # light fixtures were physically at +/-reach_y.
    # In SDF, direction is in the light's local frame (pose RPY is zero).
    return 0.0, 0.0, -1.0


def make_light(name, x, y, z, settings):
    root = ET.Element("light", type=settings["kind"], name=name)
    ET.SubElement(root, "pose").text = f"{x:.4f} {y:.4f} {z:.4f} 0 0 0"
    rgb = hsv_rgb(settings["hsv"])
    ET.SubElement(root, "diffuse").text = rgb_text(rgb)
    ET.SubElement(root, "specular").text = rgb_text(
        tuple(min(1, v * settings["specular"]) for v in rgb)
    )
    ET.SubElement(root, "intensity").text = str(settings["intensity"])
    ET.SubElement(root, "cast_shadows").text = str(name in shadow_light_names(settings)).lower()
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


def material_for(rgb, group="court"):
    # Court / walls / venue are matte, already plain-colored primitives.
    # Avoid reflective effects and extra texture maps, keep poles' original
    # minor highlight so there is no misleading change to their appearance.
    material = ET.Element("material")
    ET.SubElement(material, "ambient").text = rgb_text(tuple(0.8 * v for v in rgb))
    ET.SubElement(material, "diffuse").text = rgb_text(rgb)
    ET.SubElement(material, "specular").text = (
        "0.02 0.02 0.02 1" if group == "poles" else "0 0 0 1"
    )
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
            visual.append(material_for(rgb, group))


MAT_LENGTH = 15.4
MAT_WIDTH = 7.2
WALL_THICKNESS = 0.10


def checked_venue(raw):
    out = dict(DEFAULTS["venue"])
    out.update(raw)
    out["clearance_x"] = bounded(out["clearance_x"], 0.25, 8.0, "clearance_x")
    out["clearance_y"] = bounded(out["clearance_y"], 0.25, 8.0, "clearance_y")
    return out


def configure_venue(world, venue):
    """Resize actual floor COLLISION and visuals and reposition boundary walls.

    Clearances are per-side distances from the mat edge to the venue edge.
    No changes are made to official court markings, tag mounts or net poles.
    """
    x_length = MAT_LENGTH + 2.0 * venue["clearance_x"]
    y_width = MAT_WIDTH + 2.0 * venue["clearance_y"]
    floor_model = next((m for m in world.findall("model")
                        if m.get("name") == "court_floor"), None)
    boundary = next((m for m in world.findall("model")
                     if m.get("name") == "simulation_safety_boundary"), None)
    if floor_model is None or boundary is None:
        raise ValueError("Missing venue floor or safety boundary model")
    for name, zsize in (("floor_collision", 0.02),
                        ("venue_floor_visual", 0.018)):
        elem = next((e for e in floor_model.iter()
                     if e.get("name") == name), None)
        if elem is None:
            raise ValueError(f"Missing {name}")
        size = elem.find("./geometry/box/size")
        if size is None:
            raise ValueError(f"Missing geometry of {name}")
        size.text = f"{x_length:.4f} {y_width:.4f} {zsize:.4f}"
    for axis in ("x", "y"):
        for side in ("positive", "negative"):
            sign = 1 if side == "positive" else -1
            name_base = f"wall_{axis}_{side}"
            if axis == "x":
                pose = f"{sign * (x_length / 2.0 - WALL_THICKNESS / 2.0):.4f} 0 0.20 0 0 0"
                dims = f"{WALL_THICKNESS:.4f} {y_width:.4f} 0.40"
            else:
                pose = f"0 {sign * (y_width / 2.0 - WALL_THICKNESS / 2.0):.4f} 0.20 0 0 0"
                dims = f"{x_length - 2 * WALL_THICKNESS:.4f} {WALL_THICKNESS:.4f} 0.40"
            for suffix in ("collision", "visual"):
                element = next((e for e in boundary.iter()
                                if e.get("name") == name_base + "_" + suffix), None)
                if element is None:
                    raise ValueError(f"Missing {name_base}_{suffix}")
                element.find("pose").text = pose
                element.find("./geometry/box/size").text = dims




def camera_entity(name, topic, pose, resolution=(800, 450), hz=8.0, hfov=1.6):
    model = ET.Element("model", name=name)
    ET.SubElement(model, "static").text = "true"
    pose_element = ET.SubElement(model, "pose")
    # Preserve an exact Gazebo GUI orbit quaternion when restoring a preset.
    # Existing six-component Euler poses remain the default for manual views.
    if len(pose) == 7:
        pose_element.set("rotation_format", "quat_xyzw")
    elif len(pose) != 6:
        raise ValueError("Camera pose must be XYZ+RPY or XYZ+quaternion")
    pose_element.text = " ".join(f"{v:.9f}" for v in pose)
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
    venue = checked_venue(settings.get("venue", {}))
    profile = settings.get("inspection_profile", "lean")
    if profile not in INSPECTION_PROFILES:
        raise ValueError("Invalid inspection camera profile: " + str(profile))
    inspection = INSPECTION_PROFILES[profile]
    root = ET.parse(source).getroot()
    world = root.find("world")
    if world is None or world.get("name") != WORLD:
        raise ValueError("Expected badminton_court Gazebo world")
    for original in list(world.findall("light")):
        world.remove(original)
    # Calibration needs shuttles to remain visible at all robot poses;
    # disable automatic simulated pickup in this derivative world only.
    for plugin in world.findall("plugin"):
        if plugin.get("filename") == "libshuttle_manager_system.so":
            collection = plugin.find("enable_collection")
            if collection is not None:
                collection.text = "false"
    for name, x, y, z in centered_positions(lights):
        world.append(make_light(name, x, y, z, lights))
    configure_model(world, colors)
    configure_venue(world, venue)
    # Gazebo cameras look along local +X; positive pitch looks downward.
    world.append(camera_entity("lab_overhead_camera", "/lighting_lab/overhead",
                               (10.5, 0, 9.5, 0, .736, math.pi),
                               resolution=inspection["overhead"][0],
                               hz=inspection["overhead"][1], hfov=1.56))
    free = settings.get("free", DEFAULTS["free"])
    if "orientation" in free:
        # GUI orbit poses can be farther away or rolled relative to the court.
        # Persist the complete rotation instead of reducing it to yaw/pitch.
        orientation = free["orientation"]
        if not isinstance(orientation, (list, tuple)) or len(orientation) != 4:
            raise ValueError("free.orientation must be [qx, qy, qz, qw]")
        q = tuple(bounded(value, -1.01, 1.01, "free.orientation") for value in orientation)
        norm = math.sqrt(sum(value * value for value in q))
        if norm < 1e-6:
            raise ValueError("free.orientation quaternion must be nonzero")
        pose = (
            bounded(free["x"], -100, 100, "free.x"),
            bounded(free["y"], -100, 100, "free.y"),
            bounded(free["z"], -100, 100, "free.z"),
            *(value / norm for value in q),
        )
    else:
        pose = (bounded(free["x"], -18, 18, "free.x"),
                bounded(free["y"], -14, 14, "free.y"),
                bounded(free["z"], 1, 16, "free.z"),
                0, math.radians(bounded(free["pitch"], -60, 88, "free.pitch")),
                math.radians(bounded(free["yaw"], -360, 360, "free.yaw")))
    world.append(camera_entity("lab_free_camera", "/lighting_lab/free", pose,
                               resolution=inspection["free"][0],
                               hz=inspection["free"][1], hfov=1.3))
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode")

