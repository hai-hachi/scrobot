#!/usr/bin/env python3
"""Isolated SCROBOT Odometry Lab web control and CSV export."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime,timezone
import io
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from urllib.parse import urlsplit
import webbrowser

import yaml

from core import DEFAULTS, RATE_NAMES, validate

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lab_configs import path_for as lab_yaml_path, read as read_lab_yaml, save as save_lab_yaml

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
OUT=Path.home()/"scrobot_ws/evaluation_results/odometry_lab"
SETTINGS=OUT/"settings.json"
PUBLISHED=REPO/"src/scrobot_simulation/config/odometry_calibration.yaml"
BACKUPS=OUT/"published_backups"

sys.path.insert(0,str(REPO/"src/scrobot_simulation/launch"))
from odometry_rate_config import (
    load_rates, prepare_configurations, validated_rates,
)


def saved_settings():
    saved_yaml = read_lab_yaml("odometry")
    if saved_yaml is not None:
        return validate(saved_yaml)
    if not SETTINGS.is_file():
        return dict(DEFAULTS)
    return validate(json.loads(SETTINGS.read_text(encoding="utf-8")))


def persist(settings):
    OUT.mkdir(parents=True,exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w",encoding="utf-8",dir=OUT,prefix=".odometry_settings_",delete=False
    ) as fd:
        path=Path(fd.name)
        json.dump(settings,fd,indent=2)
        fd.write("\n")
    try:
        os.replace(path,SETTINGS)
    finally:
        path.unlink(missing_ok=True)


def rates_of(settings):
    return validated_rates({key:settings[key] for key in RATE_NAMES})


def publish_settings(settings):
    """Back up and replace only the SIMULATION YAML, never hardware configs."""
    before=load_rates(PUBLISHED)
    after=rates_of(settings)
    if before==after:
        return {"changed":False,"path":str(PUBLISHED),"backup":None}
    BACKUPS.mkdir(parents=True,exist_ok=True)
    stamp=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    backup=BACKUPS/(stamp+".yaml")
    old=PUBLISHED.read_text(encoding="utf-8")
    with backup.open("x",encoding="utf-8") as f:
        f.write(old)
    doc={
        "schema_version":1,
        "rates":after,
    }
    new="# Simulation-only Odometry Lab published calibration.\n"+yaml.safe_dump(doc,sort_keys=False)
    with tempfile.NamedTemporaryFile(
        mode="w",encoding="utf-8",dir=PUBLISHED.parent,
        prefix=".odometry_calibration_",delete=False
    ) as fd:
        tmp=Path(fd.name)
        fd.write(new)
        fd.flush()
        os.fsync(fd.fileno())
    try:
        os.replace(tmp,PUBLISHED)
    finally:
        tmp.unlink(missing_ok=True)
    if load_rates(PUBLISHED)!=after:
        raise RuntimeError("Publication failed verification")
    return {"changed":True,"path":str(PUBLISHED),"backup":str(backup)}


def csv_text(rows):
    if not rows:
        return ""
    keys=sorted(set().union(*(r.keys() for r in rows)))
    buf=io.StringIO()
    writer=csv.DictWriter(buf,fieldnames=keys)
    writer.writeheader()
    writer.writerows(rows)
    return buf.getvalue()


class Lab:
    def __init__(self,attach=False):
        from ament_index_python.packages import get_package_share_directory
        from ros_adapter import RosBridge
        self.share=get_package_share_directory
        self.lock=threading.RLock()
        self.settings=saved_settings()
        OUT.mkdir(parents=True,exist_ok=True)
        sim_share=Path(self.share("scrobot_simulation"))
        mobility=yaml.safe_load(
            (sim_share/"config/mobility_calibration.yaml").read_text(encoding="utf-8")
        )
        self.motor_model=mobility["motor"]["model"]
        if self.motor_model not in ("baseline","first_order"):
            raise RuntimeError("Invalid published Mobility Lab motor model")
        self.command_topic=(
            "/cmd_vel_motor_target" if self.motor_model=="first_order"
            else "/diff_drive_controller/cmd_vel"
        )
        self.bridge=RosBridge(self.settings,self.command_topic)
        self.attach=attach
        self.proc=None
        self.log=None
        self.restarting=False
        self.error=None
        self.active_rates=None
        if not self.attach:
            self.restart()
        else:
            self.active_rates=rates_of(self.settings)

    def stop_sim(self):
        proc=self.proc
        self.proc=None
        if proc is not None and proc.poll() is None:
            for sig,timeout in ((signal.SIGINT,4),(signal.SIGTERM,2)):
                if proc.poll() is not None:
                    break
                try:
                    os.killpg(proc.pid,sig)
                    proc.wait(timeout=timeout)
                except (ProcessLookupError,subprocess.TimeoutExpired):
                    pass
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid,signal.SIGKILL)
                except ProcessLookupError:
                    pass
        if self.log is not None:
            self.log.close()
            self.log=None

    def restart(self):
        if self.attach:
            raise RuntimeError("Cannot restart an externally managed simulation")
        with self.lock:
            if self.restarting:
                raise RuntimeError("Restart in progress")
            self.restarting=True
            config=dict(self.settings)
        try:
            self.bridge.node.reset_for_restart()
            self.stop_sim()
            if not shutil.which("ros2"):
                raise RuntimeError("ROS 2 unavailable: source Jazzy and workspace install")
            controllers=(
                Path(self.share("scrobot_control"))/"config/controllers.yaml"
            )
            ekf=Path(self.share("scrobot_localization"))/"config/ekf.yaml"
            generated=prepare_configurations(
                rates_of(config),controllers,ekf
            )
            self.log=(OUT/"gazebo.log").open("ab")
            cmd=[
                "ros2","launch","scrobot_simulation","odometry_lab.launch.py",
                "use_magnetometer:="+str(config["use_magnetometer"]).lower(),
                "imu_rate_hz:="+str(config["imu_hz"]),
                "magnetometer_rate_hz:="+str(config["magnetometer_hz"]),
                "ground_truth_rate_hz:="+str(config["ground_truth_hz"]),
                "controller_params:="+generated["controller_params"],
                "ekf_params:="+generated["ekf_params"],
            ]
            self.proc=subprocess.Popen(
                cmd,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True
            )
            time.sleep(0.1)
            if self.proc.poll() is not None:
                raise RuntimeError("Gazebo launch exited. Inspect "+str(OUT/"gazebo.log"))
            self.active_rates=rates_of(config)
            with self.bridge.node.lock:
                self.bridge.node.settings=dict(config)
            self.error=None
            return {"started":True}
        except Exception as exc:
            self.error=str(exc)
            raise
        finally:
            with self.lock:
                self.restarting=False

    def change_settings(self,raw):
        with self.lock:
            if self.bridge.node.state in ("pre","running"):
                raise RuntimeError("Stop experiment before changing parameters")
            self.settings=validate({**self.settings,**raw})
            persist(self.settings)
            return dict(self.settings)

    def save_lab_yaml(self):
        with self.lock:
            if self.bridge.node.state in ("pre", "running"):
                raise RuntimeError("Stop experiment before saving Odometry Lab YAML")
            path = save_lab_yaml("odometry", self.settings, validator=validate)
            return {"saved": str(path), "settings": dict(self.settings)}

    def run(self):
        with self.lock:
            if self.restarting:
                raise RuntimeError("Gazebo restarting")
            if self.active_rates!=rates_of(self.settings):
                raise RuntimeError("Rates changed: restart Gazebo to apply")
            if not self.attach and (self.proc is None or self.proc.poll() is not None):
                raise RuntimeError("Gazebo not running")
            if self.bridge.node.settings["use_magnetometer"]!=self.settings["use_magnetometer"]:
                raise RuntimeError("Magnetometer mode changed: restart Gazebo")
            return self.bridge.node.start(self.settings)

    def status(self):
        with self.lock:
            status=self.bridge.node.status()
            status.update({
                "settings":dict(self.settings),
                "rates_changed":self.active_rates!=rates_of(self.settings) or
                     self.bridge.node.settings["use_magnetometer"]!=self.settings["use_magnetometer"],
                "gazebo_alive":self.attach or
                    (self.proc is not None and self.proc.poll() is None),
                "restarting":self.restarting,
                "published_match":(
                    read_lab_yaml("odometry") is not None and
                    load_rates(PUBLISHED)==rates_of(validate(read_lab_yaml("odometry")))
                ),
                "lab_yaml": {
                    "path": str(lab_yaml_path("odometry")),
                    "saved": read_lab_yaml("odometry") is not None,
                    "matches_live": read_lab_yaml("odometry") == self.settings,
                },
                "mobility_lab_yaml": {
                    "path": str(lab_yaml_path("mobility")),
                    "saved": read_lab_yaml("mobility") is not None,
                    "note": "Live odometry simulator still uses PUBLISHED Mobility calibration until explicitly transferred and simulation restarted.",
                },
                "server_error":self.error,
                "motor_model":self.motor_model,
                "command_topic":self.command_topic,
            })
            return status

    def publish(self):
        with self.lock:
            if self.bridge.node.state in ("pre","running"):
                raise RuntimeError("Stop experiment before publishing")
            saved = read_lab_yaml("odometry")
            if saved is None:
                raise RuntimeError("Save Odometry Lab YAML before publishing")
            return publish_settings(validate(saved))

    def save(self):
        result=self.bridge.node.result()
        if not result:
            raise RuntimeError("No complete experiment to save")
        folder=OUT/"recordings"/(
            datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
            +"_"+result["settings"]["profile"]
        )
        folder.mkdir(parents=True,exist_ok=False)
        (folder/"result.json").write_text(json.dumps(result,indent=2)+"\n")
        for stream,rows in result["data"].items():
            if rows:
                (folder/(stream+".csv")).write_text(csv_text(rows))
        return {"saved":str(folder),
                "metrics":result["metrics"]}

    def close(self):
        self.bridge.node.abort()
        self.stop_sim()
        self.bridge.close()


class Handler(BaseHTTPRequestHandler):
    server_version="ScrobotOdometryLab/1.0"
    def send_data(self,status,obj,content_type="application/json"):
        data=obj if isinstance(obj,bytes) else (
            obj.encode("utf-8") if isinstance(obj,str) else
            json.dumps(obj,allow_nan=False).encode("utf-8")
        )
        self.send_response(status)
        self.send_header("Content-Type",content_type)
        self.send_header("Cache-Control","no-store")
        self.send_header("X-Content-Type-Options","nosniff")
        self.send_header("Content-Length",str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def body(self):
        n=int(self.headers.get("Content-Length","0"))
        if not 0<n<=100000:
            raise ValueError("Invalid request size")
        return json.loads(self.rfile.read(n).decode("utf-8"))

    def do_GET(self):
        path=urlsplit(self.path).path
        try:
            if path=="/":
                return self.send_data(200,(HERE/"index.html").read_bytes(),
                                      "text/html; charset=utf-8")
            if path=="/api/status":
                return self.send_data(200,self.server.lab.status())
            if path=="/api/result":
                return self.send_data(200,self.server.lab.bridge.node.result() or {})
            return self.send_data(404,{"error":"Unknown route"})
        except (ValueError,OSError,RuntimeError) as exc:
            return self.send_data(400,{"error":str(exc)})

    def do_POST(self):
        path=urlsplit(self.path).path
        try:
            body=self.body()
            lab=self.server.lab
            if path=="/api/settings":
                result=lab.change_settings(body)
            elif path=="/api/restart":
                result=lab.restart()
            elif path=="/api/run":
                result=lab.run()
            elif path=="/api/stop":
                lab.bridge.node.abort()
                result={"stopped":True}
            elif path=="/api/save":
                result=lab.save()
            elif path=="/api/config/save":
                if body != {}:
                    raise ValueError("Save Lab YAML uses current validated settings")
                result=lab.save_lab_yaml()
            elif path=="/api/publish":
                if body.get("confirm") is not True:
                    raise ValueError("Publishing requires explicit confirmation")
                result=lab.publish()
            else:
                return self.send_data(404,{"error":"Unknown route"})
            return self.send_data(200,result)
        except (ValueError,TypeError,OSError,RuntimeError,KeyError) as exc:
            return self.send_data(400,{"error":str(exc)})

    def log_message(self,*args):
        pass


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port",type=int,default=8767)
    parser.add_argument("--ros-domain-id",type=int,default=78)
    parser.add_argument("--no-browser",action="store_true")
    parser.add_argument("--attach",action="store_true")
    args=parser.parse_args()
    if not 0<=args.ros_domain_id<=232:
        parser.error("--ros-domain-id outside 0..232")
    # Distinct from Mobility Lab domain 77 and physical robot domain.
    os.environ["ROS_DOMAIN_ID"]=str(args.ros_domain_id)
    os.environ.setdefault("GZ_PARTITION",f"scrobot_odom_{os.getuid()}_{args.ros_domain_id}")
    lab=Lab(attach=args.attach)
    httpd=ThreadingHTTPServer(("127.0.0.1",args.port),Handler)
    httpd.lab=lab
    url=f"http://127.0.0.1:{args.port}"
    print("Odometry Lab:",url)
    print("ROS_DOMAIN_ID=",os.environ["ROS_DOMAIN_ID"])
    print("Recordings:",OUT)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        httpd.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
        lab.close()


if __name__=="__main__":
    main()
