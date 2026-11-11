"""Reuse current Lighting Lab published court visuals in headless YOLO capture.

Don't modify production SDF or the original dataset world. The original
triggered camera and scene plugins remain, but lights/court/walls/net are
replaced with exact static models from today's published badminton court.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def sync_dataset_world(production_sdf, dataset_template, output_sdf):
    production_sdf, dataset_template, output_sdf = map(
        Path, (production_sdf, dataset_template, output_sdf))
    source = production_sdf.read_bytes()
    prod = ET.fromstring(source)
    template = ET.parse(dataset_template)
    world = template.getroot().find("world")
    original = prod.find("world")
    if world is None or world.get("name") != "yolo_dataset":
        raise ValueError("Expected headless YOLO dataset world")
    if original is None or original.get("name") != "badminton_court":
        raise ValueError("Expected current production badminton court")

    # Reuse fixed static geometry/materials and current lighting. Intentionally
    # do NOT reuse production Physics, collection manager or moving shuttles:
    # the headless rig must remain deterministic, quick and non-destructive.
    for name in ("dataset_court", "net_assembly", "court_floor",
                 "court_markings", "simulation_safety_boundary"):
        for model in list(world.findall("model")):
            if model.get("name") == name:
                world.remove(model)
    for name in ("court_floor", "court_markings", "net_assembly",
                 "simulation_safety_boundary"):
        model = original.find(f"./model[@name='{name}']")
        if model is None:
            raise ValueError(f"Missing production court model: {name}")
        if model.find("plugin") is not None:
            raise ValueError(f"Refusing to copy plugin from {name}")
        world.append(copy.deepcopy(model))
    for light in list(world.findall("light")):
        world.remove(light)
    lights = original.findall("light")
    if not lights:
        raise ValueError("Production court has no lights")
    for light in lights:
        world.append(copy.deepcopy(light))
    scene = original.find("scene")
    old_scene = world.find("scene")
    if scene is None or old_scene is None:
        raise ValueError("Missing SDF scene")
    world.remove(old_scene)
    world.insert(0, copy.deepcopy(scene))
    # Keep one fixed tag model, and the triggered yolo_camera_rig.
    includes = [m.findtext("uri", "") for m in world.findall("include")]
    if includes.count("model://yolo_camera_rig") != 1:
        raise ValueError("Lost triggered RGB camera rig")
    if includes.count("model://court_apriltags") != 1:
        raise ValueError("Expected exactly one AprilTag model")
    output_sdf.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(template, "  ")
    temp = output_sdf.with_suffix(".tmp")
    template.write(temp, encoding="unicode", xml_declaration=True)
    temp.replace(output_sdf)
    return {
        "lighting_source": str(production_sdf),
        "lighting_source_sha256": hashlib.sha256(source).hexdigest(),
        "headless_world": str(output_sdf),
        "headless_world_sha256": hashlib.sha256(output_sdf.read_bytes()).hexdigest(),
        "light_count": len(lights),
        "source_models": ["court_floor", "court_markings", "net_assembly",
                          "simulation_safety_boundary"],
        "camera": "existing event-triggered yolo_camera_rig RGB 1280x720",
        "note": "Exact published production lighting and floor materials, no production simulation edits",
    }


def write_dataset_provenance(metadata, dataset_root, settings):
    root = Path(dataset_root)
    root.mkdir(parents=True, exist_ok=True)
    info = dict(metadata)
    info["generation_settings"] = {
        "target_images": settings["dataset_images"],
        "shuttle_count": settings["dataset_shuttles"],
        "positive_pose_fraction": settings["dataset_positive_fraction"],
        "random_seed": settings["dataset_seed"],
    }
    p = root / "lighting_provenance.json"
    p.write_text(json.dumps(info, indent=2) + "\n")
    return p
