"""Generate a triggered capture world with the saved Lighting and Camera settings.

Reuse YOLO inspection world for court appearance, then replace continuous
robot camera and example shuttles with the historical triggered RGB rig.
The collector alone spawns and labels randomized shuttles.
"""
from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

from world import build, ROOT
from yolo_settings import validate as validate_yolo
from modes import validate as validate_camera


RIG_SDF = Path(__file__).parent / "models/yolo_camera_rig/model.sdf"


def build_dataset(camera, yolo, path, lighting=None):
    camera = validate_camera(camera)
    yolo = validate_yolo(yolo)
    target = Path(path)
    build(camera, yolo, target, lighting=lighting)
    parsed = ET.parse(target)
    scene = parsed.getroot().find("world")
    scene.set("name", "yolo_dataset")
    for model in list(scene.findall("model")):
        if model.get("name", "").startswith("yolo_shuttle_") or model.get("name") == "camera_rig":
            scene.remove(model)
    source = ET.parse(RIG_SDF).getroot().find("model")
    if source is None:
        raise ValueError("Triggered dataset camera rig is invalid")
    source.set("name", "yolo_camera_rig")
    source.find("pose") if source.find("pose") is not None else ET.SubElement(source, "pose")
    source.find("pose").text = "2 0 0.28683059 0 0.2617993877991494 0"
    sensor = source.find("./link/sensor")
    if sensor is None:
        raise ValueError("Triggered dataset RGB sensor missing")
    sensor.find("update_rate").text = str(camera["color_fps"])
    image = sensor.find("camera/image")
    width,height = camera["color"].split("x")
    image.find("width").text = width
    image.find("height").text = height
    scene.append(source)
    ET.indent(parsed.getroot(), space="  ")
    parsed.write(target, encoding="utf-8", xml_declaration=True)
    return str(target)
