"""Build an isolated, minimal Gazebo world. Never modify production files.

The court mode copies the exact published lighting and static court visuals.
No full robot, controllers, Nav2, moving sensors or shuttle collection plugin:
the poseable 'perception_robot' is a camera rig with a visual body proxy.
"""
from __future__ import annotations
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from core import COLOR, DEPTH, validate

WORLD = "perception_lab"
COLOR_HFOV = "1.229041640167034"
DEPTH_HFOV = "1.5795850624055896"
COURT_MODELS = ("court_floor", "court_markings", "net_assembly",
                "simulation_safety_boundary")

def sub(parent, name, value=None, attrs=None):
    node = ET.SubElement(parent, name, attrs or {})
    if value is not None:
        node.text = str(value)
    return node

def plugins(world):
    for suffix, name in (
        ("physics", "Physics"), ("user-commands", "UserCommands"),
        ("scene-broadcaster", "SceneBroadcaster"),
    ):
        sub(world, "plugin", attrs={
            "filename": f"gz-sim-{suffix}-system",
            "name": f"gz::sim::systems::{name}"})
    sensor = sub(world, "plugin", attrs={
        "filename": "gz-sim-sensors-system",
        "name": "gz::sim::systems::Sensors"})
    sub(sensor, "render_engine", "ogre2")

def add_camera(link, name, kind, resolution, fps, hfov, frame):
    sensor = sub(link, "sensor", attrs={"name": name, "type": kind})
    # Same pose for RGB and depth in YOLO mode: pixel coordinates align.
    # The rig model pose represents base_footprint.
    sub(sensor, "pose", "0.19 0 0.287 0 0.2617993878 0")
    sub(sensor, "always_on", "true")
    sub(sensor, "update_rate", fps)
    sub(sensor, "visualize", "false")
    path = "color" if kind == "camera" else "depth"
    sub(sensor, "topic", f"/lab/camera/{path}")
    sub(sensor, "gz_frame_id", frame)
    cam = sub(sensor, "camera", attrs={"name": name+"_camera"})
    sub(cam, "horizontal_fov", hfov)
    sub(cam, "camera_info_topic", f"/lab/camera/{path}/camera_info")
    image = sub(cam, "image")
    sub(image, "width", resolution[0])
    sub(image, "height", resolution[1])
    sub(image, "format", "R8G8B8" if kind == "camera" else "R_FLOAT32")
    clip = sub(cam, "clip")
    sub(clip, "near", "0.05" if kind == "camera" else "0.20")
    sub(clip, "far", "20.0" if kind == "camera" else "10.0")
    sub(cam, "optical_frame_id", frame)

def robot(world, cfg):
    model = sub(world, "model", attrs={"name":"perception_robot"})
    sub(model, "static", "true")
    import math
    sub(model, "pose", f'{cfg["robot_x"]} {cfg["robot_y"]} 0 0 0 {math.radians(cfg["robot_yaw"]):.9f}')
    link = sub(model, "link", attrs={"name":"camera_rig_link"})
    visual = sub(link, "visual", attrs={"name":"robot_proxy"})
    sub(visual, "pose", "0 0 0.15 0 0 0")
    box = sub(sub(visual,"geometry"),"box")
    sub(box,"size","0.55 0.43 0.30")
    mat = sub(visual,"material")
    sub(mat,"ambient","0.16 0.23 0.28 1")
    sub(mat,"diffuse","0.20 0.29 0.35 1")
    add_camera(link,"lab_color","camera",COLOR[cfg["color"]],
               cfg["color_fps"],COLOR_HFOV,"camera_color_optical_frame")
    if cfg["depth_enabled"]:
        add_camera(link,"lab_depth","depth_camera",DEPTH[cfg["depth"]],
                   cfg["depth_fps"], COLOR_HFOV if cfg["mode"]=="yolo"
                   else DEPTH_HFOV,
                   "camera_color_optical_frame" if cfg["mode"]=="yolo"
                   else "camera_depth_optical_frame")

def build(settings, production, destination):
    cfg=validate(settings)
    root = ET.Element("sdf",{"version":"1.10"})
    world = sub(root,"world", attrs={"name":WORLD})
    physics = sub(world,"physics",attrs={"name":"lab_physics","type":"ignored"})
    sub(physics,"max_step_size","0.01")
    sub(physics,"real_time_factor","1.0")
    plugins(world)
    source_hash = None
    if cfg["mode"]=="smoke":
        scene = sub(world,"scene")
        sub(scene,"ambient","0.6 0.6 0.6 1")
        sub(scene,"background","0.07 0.09 0.11 1")
        floor = sub(world,"model",attrs={"name":"smoke_floor"})
        sub(floor,"static","true")
        link=sub(floor,"link",attrs={"name":"floor_link"})
        box=sub(sub(sub(link,"visual",attrs={"name":"floor"}),"geometry"),"box")
        sub(box,"size","16 8 0.01")
    else:
        raw=Path(production).read_bytes()
        source_hash=hashlib.sha256(raw).hexdigest()
        original=ET.fromstring(raw).find("world")
        if original is None or original.get("name")!="badminton_court":
            raise ValueError("Expected published badminton_court.sdf")
        for child in original:
            if child.tag in ("scene","light"):
                world.append(copy.deepcopy(child))
            elif child.tag=="model" and child.get("name") in COURT_MODELS:
                world.append(copy.deepcopy(child))
        present={m.get("name") for m in world.findall("model")}
        if present != set(COURT_MODELS) or not world.findall("light"):
            raise ValueError("Published world is missing a court model or lighting")
        inc=sub(world,"include")
        sub(inc,"uri","model://court_apriltags")
    robot(world,cfg)
    destination=Path(destination)
    destination.parent.mkdir(parents=True,exist_ok=True)
    ET.indent(root,space="  ")
    payload=ET.tostring(root,encoding="utf-8",xml_declaration=True)
    temporary=destination.with_suffix(".tmp")
    temporary.write_bytes(payload)
    temporary.replace(destination)
    return {
        "world":str(destination), "world_sha256":hashlib.sha256(payload).hexdigest(),
        "lighting_source":str(production) if source_hash else None,
        "lighting_sha256":source_hash, "mode":cfg["mode"],
        "rgb":COLOR[cfg["color"]], "depth":DEPTH[cfg["depth"]]
                if cfg["depth_enabled"] else None,
    }

def build_dataset(production, template, destination):
    """Keep the existing triggered YOLO camera/capture code, replace old lighting."""
    original=ET.parse(production).getroot().find("world")
    doc=ET.parse(template)
    world=doc.getroot().find("world")
    if world is None or world.get("name")!="yolo_dataset":
        raise ValueError("Unexpected dataset world")
    if original is None:
        raise ValueError("Missing published court")
    for child in list(world):
        if child.tag=="scene" or child.tag=="light" or (
            child.tag=="model" and child.get("name") in
            ("dataset_court","net_assembly",*COURT_MODELS)
        ):
            world.remove(child)
    for child in original:
        if child.tag in ("scene","light") or (
            child.tag=="model" and child.get("name") in COURT_MODELS
        ):
            world.append(copy.deepcopy(child))
    if len(world.findall("light"))<1:
        raise ValueError("Cannot generate an unlit dataset")
    includes=[x.findtext("uri") for x in world.findall("include")]
    if "model://yolo_camera_rig" not in includes:
        raise ValueError("Missing existing triggered YOLO camera")
    ET.indent(doc,space="  ")
    destination=Path(destination)
    destination.parent.mkdir(parents=True,exist_ok=True)
    doc.write(destination,encoding="utf-8",xml_declaration=True)
    return {"production_sha256":hashlib.sha256(Path(production).read_bytes()).hexdigest(),
            "generated_sha256":hashlib.sha256(destination.read_bytes()).hexdigest(),
            "lights":len(world.findall("light"))}
