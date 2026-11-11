"""Publish only RGB/depth camera configuration to NORMAL Gazebo.

Checks current schema, saves exact backup, atomically replaces YAML.
Never edits Lighting Lab, physical RealSense ROS config or perception models.
"""
from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import os
import tempfile
import yaml

from core import validate, camera_args

REPO = Path(__file__).resolve().parents[2]
SIM_CONFIG = REPO / "src/scrobot_simulation/config/perception_calibration.yaml"
BACKUPS = Path.home() / "scrobot_ws/evaluation_results/perception_labs/published_backups"


def document(settings):
    s = validate(settings)
    c = camera_args(s)
    return {
        "schema_version": 1,
        "color": {"width":c["color_width"],"height":c["color_height"],"fps":c["color_fps"]},
        "depth": {"width":c["depth_width"],"height":c["depth_height"],"fps":c["depth_fps"]},
    }


def read_config(path=SIM_CONFIG):
    d = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(d,dict) or set(d) != {"schema_version","color","depth"} or d["schema_version"] != 1:
        raise ValueError("Unexpected simulation camera calibration schema")
    clean = {
        "color_mode":f'{d["color"]["width"]}x{d["color"]["height"]}',
        "depth_mode":f'{d["depth"]["width"]}x{d["depth"]["height"]}',
        "color_fps":d["color"]["fps"],"depth_fps":d["depth"]["fps"],
    }
    if document(clean) != d:
        raise ValueError("Invalid published camera fields")
    return d


def status(settings,path=SIM_CONFIG):
    try:
        d = read_config(path)
        return {"matches":d==document(settings),"path":str(path),"error":None}
    except (ValueError,OSError,yaml.YAMLError) as exc:
        return {"matches":False,"path":str(path),"error":str(exc)}


def publish(settings,path=SIM_CONFIG,backups=BACKUPS):
    path,backups=Path(path),Path(backups)
    prior = path.read_text(encoding="utf-8")
    read_config(path)
    new_doc = document(settings)
    if read_config(path) == new_doc:
        return {"changed":False,"path":str(path),"backup":None}
    backups.mkdir(parents=True,exist_ok=True)
    backup=backups/(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")+".yaml")
    backup.write_text(prior,encoding="utf-8")
    tmp=None
    try:
        with tempfile.NamedTemporaryFile(mode="w",encoding="utf-8",dir=path.parent,
                                         prefix=".perception_",suffix=".tmp",delete=False) as f:
            tmp=Path(f.name)
            f.write("# Simulation-only RGB/depth benchmark calibration.\n")
            yaml.safe_dump(new_doc,f,sort_keys=False)
            f.flush()
            os.fsync(f.fileno())
        tmp.chmod(path.stat().st_mode)
        os.replace(tmp,path)
        tmp=None
    finally:
        if tmp is not None:tmp.unlink(missing_ok=True)
    if read_config(path)!=new_doc:
        raise RuntimeError("Published camera file failed verification")
    return {"changed":True,"path":str(path),"backup":str(backup)}
