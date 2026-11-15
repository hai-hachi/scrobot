"""Validated, non-publishing Performance Lab settings and world speed cap."""
from __future__ import annotations

from copy import deepcopy
import hashlib
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml

ROOT=Path(__file__).resolve().parents[2]
WORLD=ROOT/"src/scrobot_simulation/worlds/badminton_court.sdf"
RESULTS=ROOT/"evaluation_results/performance_lab"
DEFAULTS={
    "mode":"synchronized",
    "profile":"full_idle",
    "max_rtf":1.0,
    "duration_s":60,
    "warmup_s":15,
    "ros_domain_id":85,
    "shuttle_count":50,
    "shuttle_seed":20261003,
    "yolo_device":"0",
    "model_path":"",
}
EXPECTED_TOPICS={
    "/camera/camera/color/image_raw":30.0,
    "/camera/camera/depth/image_raw":30.0,
    "/camera/camera/depth/points":30.0,
    "/camera/camera/imu":100.0,
    "/imu/mag":100.0,
    "/diff_drive_controller/odom":100.0,
    "/odometry/filtered":100.0,
    "/joint_states":100.0,
}


def validate(value):
    if not isinstance(value,dict) or set(value)!=set(DEFAULTS):
        raise ValueError("Performance Lab settings have missing or unknown fields")
    s=deepcopy(value)
    if s["mode"] not in ("synchronized","real_time"):
        raise ValueError("mode must be synchronized or real_time")
    if s["profile"] not in ("full_idle","full_mission"):
        raise ValueError("profile must be full_idle or full_mission")
    for key,lo,hi in (
        ("max_rtf",.1,1.0),("duration_s",10,600),
        ("warmup_s",0,120),("ros_domain_id",1,232),
        ("shuttle_count",0,200),("shuttle_seed",0,2147483647)
    ):
        val=s[key]
        if isinstance(val,bool) or not isinstance(val,(int,float)) or not math.isfinite(val):
            raise ValueError(key+" must be finite numeric")
        if not lo<=val<=hi:raise ValueError(key+" is out of range")
        if key in ("duration_s","warmup_s","ros_domain_id","shuttle_count","shuttle_seed"):
            if type(val) is not int:raise ValueError(key+" must be integer")
    if not isinstance(s["yolo_device"],str) or not re.fullmatch(r"[a-zA-Z0-9:,._-]{1,32}",s["yolo_device"]):
        raise ValueError("Invalid YOLO device")
    if not isinstance(s["model_path"],str) or len(s["model_path"])>1024 or "\x00" in s["model_path"]:
        raise ValueError("Invalid model path")
    s["max_rtf"]=float(s["max_rtf"])
    return s


def read_settings():
    from tools.shared.lab_configs import read
    return validate(read("performance") or DEFAULTS)


def save_settings(settings):
    from tools.shared.lab_configs import save
    return str(save("performance",settings,validator=validate))


def prepare_world(source,dest,max_rtf):
    """Copy the full calibrated world, changing ONLY the simulated speed cap.

    Gazebo may run slower than the cap. No camera, physics step, light,
    collision, robot or shuttle parameters are altered.
    """
    cap=float(max_rtf)
    if not math.isfinite(cap) or not .1<=cap<=1.:
        raise ValueError("Invalid real-time factor cap")
    content=Path(source).read_text(encoding="utf-8")
    pattern=r"(<real_time_factor>)[^<]*(</real_time_factor>)"
    if len(re.findall(pattern,content))!=1:
        raise ValueError("Expected one real_time_factor in production SDF")
    replaced=re.sub(pattern,lambda m:m.group(1)+str(cap)+m.group(2),
                    content,count=1)
    ET.fromstring(replaced)
    dest=Path(dest)
    dest.parent.mkdir(parents=True,exist_ok=True)
    dest.write_text(replaced,encoding="utf-8")
    return {"path":str(dest),
            "original_sha256":hashlib.sha256(content.encode()).hexdigest(),
            "generated_sha256":hashlib.sha256(replaced.encode()).hexdigest(),
            "max_rtf":cap}


def calibration_snapshot():
    """Record the actual production inputs AND saved lab state, not silent publish."""
    files={
        "production_world":WORLD,
        "production_camera":ROOT/"src/scrobot_simulation/config/camera_calibration.yaml",
        "production_mobility":ROOT/"src/scrobot_simulation/config/mobility_calibration.yaml",
        "production_odometry":ROOT/"src/scrobot_simulation/config/odometry_calibration.yaml",
    }
    from tools.shared.lab_configs import FILES
    files.update({"saved_"+name:path for name,path in FILES.items()
                  if name not in ("performance",)})
    result={}
    for name,path in files.items():
        try:
            data=path.read_bytes()
            result[name]={"path":str(path),"sha256":hashlib.sha256(data).hexdigest(),
                          "yaml":yaml.safe_load(data) if path.suffix==".yaml" else None}
        except (OSError,ValueError,yaml.YAMLError) as exc:
            result[name]={"path":str(path),"error":str(exc)}
    # Published calibration is what the production launch actually consumes.
    result["authority_note"]=(
        "Full production launch consumes published src/ configs and production world. "
        "Saved lab YAMLs are snapshotted for provenance, not silently published."
    )
    return result
