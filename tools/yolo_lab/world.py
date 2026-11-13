"""Standalone YOLO Lab inspection scene, using existing Camera & Lighting Lab code.

This is NOT yet the automated dataset capture pipeline. It contains real
production shuttle visual meshes and deterministic positions that can be
inspected before integrating the stale-render-safe label generator.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import random
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/camera_lab"))
from modes import validate as validate_camera, world as camera_world
from lighting_source import apply_court_world, calibration as load_lighting
from tag_math import CAMERA_OFFSET, CAMERA_PITCH

from config import validate as validate_yolo

COURT = ROOT / "src/scrobot_simulation/worlds/badminton_court.sdf"
STATIC_MODELS = ("court_floor", "court_markings", "net_assembly",
                 "simulation_safety_boundary")

def build(camera, yolo, destination, lighting=None):
    """Build real court + calibrated lights + independent RGB/depth + shuttles."""
    camera = validate_camera(camera)
    yolo = validate_yolo(yolo)
    lighting = load_lighting() if lighting is None else lighting
    dest = Path(destination)
    dest.parent.mkdir(parents=True, exist_ok=True)
    camera_world(camera, dest, lighting=lighting)
    generated = ET.parse(dest)
    root = generated.getroot()
    world = root.find("world")
    source = ET.parse(COURT).getroot().find("world")
    if world is None or source is None:
        raise ValueError("Missing expected Gazebo world")
    world.set("name", "yolo_lab")
    for model in list(world.findall("model")):
        if model.get("name") in ("floor", "target"):
            world.remove(model)
    copied = set()
    for model in source.findall("model"):
        name = model.get("name")
        if name in STATIC_MODELS:
            world.append(copy.deepcopy(model))
            copied.add(name)
    if copied != set(STATIC_MODELS):
        raise ValueError("Production court is missing required static models")
    apply_court_world(world, lighting)

    rig = world.find("./model[@name='camera_rig']")
    if rig is None:
        raise ValueError("Camera rig not found")
    rig.find("pose").text = (
        f'{yolo["robot_x"]:.6f} {yolo["robot_y"]:.6f} 0 0 0 '
        f'{math.radians(yolo["robot_yaw"]):.9f}'
    )
    link = rig.find("link")
    visual = ET.SubElement(link, "visual", name="robot_proxy")
    ET.SubElement(visual, "pose").text = "-0.10 0 0.10 0 0 0"
    shape = ET.SubElement(ET.SubElement(visual, "geometry"), "box")
    ET.SubElement(shape, "size").text = "0.35 0.40 0.18"
    mat = ET.SubElement(visual, "material")
    ET.SubElement(mat, "diffuse").text = "0.30 0.35 0.41 1"
    for sensor in link.findall("sensor"):
        pose = sensor.find("pose")
        if pose is None:
            pose = ET.SubElement(sensor, "pose")
        pose.text = (
            f"{CAMERA_OFFSET[0]:.5f} {CAMERA_OFFSET[1]:.5f} "
            f"{CAMERA_OFFSET[2]:.5f} 0 {CAMERA_PITCH:.9f} 0"
        )
        frame = sensor.find("gz_frame_id")
        if frame is not None:
            frame.text = "camera_color_optical_frame" if sensor.get("name") == "color" else "camera_depth_optical_frame"

    rng = random.Random(yolo["seed"])
    placed = []
    for i in range(yolo["shuttle_count"]):
        x = rng.uniform(-6.3, 6.3)
        y = rng.uniform(-2.8, 2.8)
        yaw = rng.uniform(-math.pi, math.pi)
        z = 0.05
        model = ET.SubElement(world, "model", name=f"yolo_shuttle_{i:03d}")
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = f"{x:.8f} {y:.8f} {z:.8f} 0 0 {yaw:.8f}"
        shuttle_link = ET.SubElement(model, "link", name="shuttle_link")
        shuttle_visual = ET.SubElement(shuttle_link, "visual", name="shuttle_visual")
        geometry = ET.SubElement(shuttle_visual, "geometry")
        mesh = ET.SubElement(geometry, "mesh")
        ET.SubElement(mesh, "uri").text = "model://shuttle/meshes/shuttle.STL"
        material = ET.SubElement(shuttle_visual, "material")
        ET.SubElement(material, "ambient").text = "0.78 0.78 0.78 1"
        ET.SubElement(material, "diffuse").text = "0.92 0.92 0.92 1"
        placed.append({"name": model.get("name"), "x": round(x, 7),
                       "y": round(y, 7), "z": z, "yaw_rad": round(yaw, 7)})
    ET.indent(root, space="  ")
    generated.write(dest, encoding="utf-8", xml_declaration=True)
    manifest = {
        "schema_version": 1, "world": "yolo_lab",
        "camera": camera, "yolo": yolo,
        "lighting_source": str(ROOT / "src/scrobot_simulation/config/lighting_lab.yaml"),
        "camera_source": str(ROOT / "src/scrobot_simulation/config/camera_lab.yaml"),
        "shuttles": placed,
        "dataset_labels_created": False,
        "note": "Visual inspection scene, NOT a synchronized labelled dataset",
    }
    (dest.parent / "scene_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return str(dest), manifest
