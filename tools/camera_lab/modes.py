"""RealSense D435i USB 3 RGB/depth modes and the minimal Gazebo camera world.

Source: Intel/RealSense D400 Series datasheet, table 4-2 (USB 3.1 Gen 1).
RGB is simulated as RGB8 (3 bytes/pixel) and depth as 32FC1 (4 bytes/pixel),
not the physical USB YUY2 and Z16 formats.
"""
from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

COLOR = {
    "1920x1080": (6, 15, 30),
    "1280x720": (6, 15, 30),
    "960x540": (6, 15, 30, 60),
    "848x480": (6, 15, 30, 60),
    "640x480": (6, 15, 30, 60),
    "640x360": (6, 15, 30, 60),
    "424x240": (6, 15, 30, 60),
    "320x240": (6, 30, 60),
    "320x180": (6, 30, 60),
}
DEPTH = {
    "1280x720": (6, 15, 30),
    "848x480": (6, 15, 30, 60, 90),
    "640x480": (6, 15, 30, 60, 90),
    "640x360": (6, 15, 30, 60, 90),
    "480x270": (6, 15, 30, 60, 90),
    "424x240": (6, 15, 30, 60, 90),
}
DEFAULT = {
    "color": "1280x720", "color_fps": 15,
    "depth": "848x480", "depth_fps": 15,
    "depth_enabled": True,
}


def validate(data):
    if not isinstance(data, dict) or set(data) != set(DEFAULT):
        raise ValueError("Invalid camera configuration fields")
    if data["color"] not in COLOR or data["depth"] not in DEPTH:
        raise ValueError("Unsupported RealSense D435i resolution")
    if (type(data["color_fps"]) is not int or
            data["color_fps"] not in COLOR[data["color"]]):
        raise ValueError("RGB FPS is unsupported for the selected resolution")
    if (type(data["depth_fps"]) is not int or
            data["depth_fps"] not in DEPTH[data["depth"]]):
        raise ValueError("Depth FPS is unsupported for the selected resolution")
    if type(data["depth_enabled"]) is not bool:
        raise ValueError("depth_enabled must be a boolean")
    return dict(data)


def expected_mb_s(settings):
    """Uncompressed ROS image payload estimate, not USB or DDS bandwidth."""
    cfg = validate(settings)
    cw, ch = (int(x) for x in cfg["color"].split("x"))
    dw, dh = (int(x) for x in cfg["depth"].split("x"))
    rgb = cw * ch * 3 * cfg["color_fps"] / 1e6
    depth = dw * dh * 4 * cfg["depth_fps"] / 1e6 if cfg["depth_enabled"] else 0
    return {"rgb": round(rgb, 3), "depth": round(depth, 3),
            "total": round(rgb + depth, 3)}


def world(settings, destination):
    """Two independent Gazebo sensors and two primitives: floor + cube."""
    cfg = validate(settings)
    root = ET.Element("sdf", version="1.10")
    scene = ET.SubElement(root, "world", name="camera_lab")

    def node(parent, tag, value=None, **attributes):
        elem = ET.SubElement(parent, tag, attributes)
        if value is not None:
            elem.text = str(value)
        return elem

    physics = node(scene, "physics", name="camera_lab_physics", type="ignored")
    node(physics, "max_step_size", "0.005")
    node(physics, "real_time_factor", "1")
    for filename, name in (
        ("gz-sim-physics-system", "Physics"),
        ("gz-sim-user-commands-system", "UserCommands"),
        ("gz-sim-scene-broadcaster-system", "SceneBroadcaster"),
    ):
        node(scene, "plugin", filename=filename,
             name="gz::sim::systems::" + name)
    sensor_plugin = node(scene, "plugin", filename="gz-sim-sensors-system",
                         name="gz::sim::systems::Sensors")
    node(sensor_plugin, "render_engine", "ogre2")
    sc = node(scene, "scene")
    node(sc, "ambient", "0.7 0.7 0.7 1")
    node(sc, "background", "0.15 0.18 0.23 1")
    sun = node(scene, "light", name="sun", type="directional")
    node(sun, "pose", "0 0 8 0 0 0")
    node(sun, "diffuse", "0.9 0.9 0.9 1")
    node(sun, "direction", "-0.5 0.1 -1")
    for name, pose, size, color in (
        ("floor", "0 0 -0.05 0 0 0", "16 10 0.1", "0.14 0.42 0.35 1"),
        ("target", "2 0 0.5 0 0 0", "0.7 0.7 1.0", "0.95 0.9 0.8 1"),
    ):
        model = node(scene, "model", name=name)
        node(model, "static", "true")
        node(model, "pose", pose)
        link = node(model, "link", name="body")
        visual = node(link, "visual", name="visual")
        box = node(node(visual, "geometry"), "box")
        node(box, "size", size)
        material = node(visual, "material")
        node(material, "ambient", color)
        node(material, "diffuse", color)
    camera = node(scene, "model", name="camera_rig")
    node(camera, "static", "true")
    node(camera, "pose", "0 0 0.5 0 0 0")
    link = node(camera, "link", name="camera_link")

    def sensor(name, sensor_type, resolution, fps, hfov, image_format):
        sensor_node = node(link, "sensor", name=name, type=sensor_type)
        node(sensor_node, "always_on", "true")
        node(sensor_node, "update_rate", fps)
        node(sensor_node, "visualize", "false")
        node(sensor_node, "topic", "/camera_lab/" + name)
        node(sensor_node, "gz_frame_id", name + "_optical_frame")
        cam = node(sensor_node, "camera", name=name + "_camera")
        node(cam, "horizontal_fov", hfov)
        node(cam, "camera_info_topic", "/camera_lab/" + name + "/camera_info")
        image = node(cam, "image")
        width, height = resolution.split("x")
        node(image, "width", width)
        node(image, "height", height)
        node(image, "format", image_format)
        clip = node(cam, "clip")
        node(clip, "near", "0.1")
        node(clip, "far", "20")
        node(cam, "optical_frame_id", name + "_optical_frame")

    sensor("color", "camera", cfg["color"], cfg["color_fps"],
           "1.229041640167034", "R8G8B8")
    if cfg["depth_enabled"]:
        sensor("depth", "depth_camera", cfg["depth"], cfg["depth_fps"],
               "1.5795850624055896", "R_FLOAT32")
    dest = Path(destination)
    dest.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    dest.write_bytes(ET.tostring(root, encoding="utf-8", xml_declaration=True))
    return str(dest)
