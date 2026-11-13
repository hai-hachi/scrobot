"""AprilTags Lab court and detector setup, independent of old perception tools."""
from __future__ import annotations

import copy
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml
from modes import world
from tag_math import CAMERA_OFFSET, CAMERA_PITCH, mount_radius

ROOT = Path(__file__).resolve().parents[2]
LANDMARKS = ROOT / "src/scrobot_localization/config/court_landmarks.yaml"
COURT = ROOT / "src/scrobot_simulation/worlds/badminton_court.sdf"
GENERATOR = ROOT / "src/scrobot_simulation/scripts/generate_court_apriltags.py"
COURT_MODELS = ("court_floor", "court_markings", "net_assembly")


def detector_yaml(settings, destination):
    """Same physical tag family/IDs, with lab-specific size; not production YAML."""
    size = float(settings["size"])
    doc = {"apriltag": {"ros__parameters": {
        "use_sim_time": True,
        "image_transport": "raw",
        "qos_profile": "sensor_data",
        "family": "16h5",
        "size": size,
        "max_hamming": 0,
        "profile": False,
        "detector": {"threads": 2, "decimate": 1.0, "blur": 0.0,
                     "refine": True, "sharpening": 0.25, "debug": False},
        "pose_estimation_method": "pnp",
        "tag": {"ids": [0, 1, 2, 3],
                "frames": [f"observed_tag_{i}" for i in range(4)],
                "sizes": [size] * 4},
    }}}
    dest = Path(destination)
    dest.write_text(yaml.safe_dump(doc, sort_keys=False))
    return str(dest)


def generate_tags(tag_config, output):
    """Use project's canonical tag texture/mesh generator, not old lab code."""
    values = yaml.safe_load(LANDMARKS.read_text())
    params = values["tag_global_localizer"]["ros__parameters"]
    params["tag_edge_size"] = float(tag_config["size"])
    params["tag_height"] = float(tag_config["height"])
    params["tag_mount_radius"] = mount_radius(float(tag_config["size"]))
    config = Path(output) / "tag_landmarks.yaml"
    config.parent.mkdir(parents=True, exist_ok=True)
    config.write_text(yaml.safe_dump(values, sort_keys=False))
    target = Path(output) / "tag_models" / "court_apriltags"
    subprocess.run(
        [sys.executable, str(GENERATOR), "--config", str(config),
         "--output", str(target)],
        check=True, timeout=45,
    )
    if not (target / "model.sdf").is_file():
        raise RuntimeError("AprilTag generator did not produce model.sdf")
    return target, params


def tag_world(cam_settings, tag_config, destination, tag_model):
    """Camera-only world plus authentic court geometry, poles and 16h5 tags."""
    path = Path(world(cam_settings, destination))
    root = ET.parse(path)
    generated = root.getroot().find("world")
    source = ET.parse(COURT).getroot().find("world")
    if generated is None or source is None:
        raise RuntimeError("Bad generated camera or published court SDF")
    for element in list(generated.findall("model")):
        if element.get("name") in ("floor", "target"):
            generated.remove(element)
    found = set()
    for model in source.findall("model"):
        if model.get("name") in COURT_MODELS:
            generated.append(copy.deepcopy(model))
            found.add(model.get("name"))
    if found != set(COURT_MODELS):
        raise RuntimeError("Published court does not contain required static models")
    tags = ET.parse(Path(tag_model) / "model.sdf").getroot().find("model")
    if tags is None:
        raise RuntimeError("Generated tag model is missing")
    generated.append(copy.deepcopy(tags))
    rig = generated.find("./model[@name='camera_rig']")
    rig.find("pose").text = (
        f'{tag_config["robot_x"]} {tag_config["robot_y"]} 0 0 0 '
        f'{__import__("math").radians(tag_config["robot_yaw"]):.9f}'
    )
    link = rig.find("link")
    marker = ET.SubElement(link, "visual", name="robot_proxy")
    geom = ET.SubElement(marker, "geometry")
    box = ET.SubElement(geom, "box")
    ET.SubElement(box, "size").text = "0.35 0.40 0.18"
    ET.SubElement(marker, "pose").text = "-0.10 0 0.10 0 0 0"
    material = ET.SubElement(marker, "material")
    ET.SubElement(material, "diffuse").text = "0.30 0.35 0.41 1"
    cam_offset = (
        f"{CAMERA_OFFSET[0]:.5f} {CAMERA_OFFSET[1]:.5f} "
        f"{CAMERA_OFFSET[2]:.5f} 0 {CAMERA_PITCH:.9f} 0"
    )
    for sensor in link.findall("sensor"):
        pose = sensor.find("pose")
        if pose is None:
            pose = ET.SubElement(sensor, "pose")
        pose.text = cam_offset
        # gazebo sensor frame name must correspond to optical topic convention.
        sensor.find("gz_frame_id").text = "camera_color_optical_frame"
    ET.indent(root, space="  ")
    root.write(path, encoding="utf-8", xml_declaration=True)
    return str(path)


def camera_info_bridge_yaml(destination):
    doc = [{
        "ros_topic_name": "/camera_lab/color/camera_info",
        "gz_topic_name": "/camera_lab/color/camera_info",
        "ros_type_name": "sensor_msgs/msg/CameraInfo",
        "gz_type_name": "gz.msgs.CameraInfo",
        "direction": "GZ_TO_ROS",
        "qos_profile": "SENSOR_DATA",
    }]
    path = Path(destination)
    path.write_text(yaml.safe_dump(doc, sort_keys=False))
    return str(path)
