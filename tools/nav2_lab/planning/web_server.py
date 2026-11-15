#!/usr/bin/env python3
"""SCROBOT Nav2 path-planning laboratory: local-only experiment orchestration."""
import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
import uuid
import webbrowser
from copy import deepcopy
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import yaml
from core import PLANNERS, nav2_params, validate
from map_builder import write_court_map
from obstacles import Obstacles

try: import psutil
except ImportError: psutil=None

HERE=Path(__file__).resolve().parent
LAB=HERE.parent
REPO=HERE.parents[2]
CONFIG=LAB/"config"/"lab.yaml"
WEB=HERE/"web"
LAUNCH=HERE/"launch"/"planning.launch.py"
RVIZ=HERE/"config"/"planning.rviz"
WORLD=REPO/"src"/"scrobot_simulation"/"worlds"/"badminton_court.sdf"
LANDMARKS=REPO/"src"/"scrobot_localization"/"config"/"court_landmarks.yaml"
PRODUCTION_NAV2=REPO/"src"/"scrobot_navigation"/"config"/"nav2_params.yaml"


def status_file(path):
    try: return json.loads(path.read_text())
    except (ValueError,FileNotFoundError):return None


def log_tail(path,limit=3400):
    if not path or not path.exists():return ""
    with path.open("rb") as f:
        f.seek(max(0,path.stat().st_size-limit))
        return f.read().decode("utf-8","replace")


def running(proc):
    return proc is not None and proc.poll() is None


def interrupt(proc,sig=signal.SIGINT):
    if running(proc):
        try:os.killpg(proc.pid,sig)
        except ProcessLookupError:pass


def preflight(env):
    if not shutil.which("ros2") or not shutil.which("gz"):
        raise RuntimeError("Source ROS 2 Jazzy and Gazebo before starting Nav2 Lab")
    result=subprocess.run(["ros2","pkg","list"],env=env,capture_output=True,
                          text=True,timeout=15,check=False)
    if result.returncode:
        raise RuntimeError("Could not list ROS packages: "+result.stderr[-600:])
    installed=set(result.stdout.splitlines())
    required={"nav2_map_server","nav2_planner","nav2_navfn_planner",
              "nav2_smac_planner","nav2_theta_star_planner",
              "nav2_lifecycle_manager","ros_gz_sim","scrobot_simulation",
              "scrobot_control","scrobot_localization","scrobot_perception"}
    missing=sorted(required-installed)
    if missing:
        raise RuntimeError("Missing ROS packages: "+", ".join(missing))
    graph=subprocess.run(["ros2","topic","list"],env=env,capture_output=True,
                         text=True,timeout=10,check=False)
    if graph.returncode:
        raise RuntimeError("ROS topic discovery failed")
    collision=set(graph.stdout.splitlines()) & {
        "/clock","/joint_states","/diff_drive_controller/odom","/map"}
    if collision:
        raise RuntimeError("Lab ROS_DOMAIN_ID already has active robot/map: "+str(sorted(collision)))


class Service:
    def __init__(self,config=CONFIG):
        self.config_path=Path(config)
        self.lock=threading.RLock()
        self.stack=None
        self.rviz=None
        self.trial=None
        self.active_dir=None
        self.env=None
        self.obstacles=None
        self.run_dir=None
        self.stack_log=None
        self.run_log=None
        self.loaded_config=None

    def config(self):
        return validate(yaml.safe_load(self.config_path.read_text()))

    def env_for(self,cfg):
        env=os.environ.copy()
        domain=int(cfg["simulation"]["ros_domain_id"])
        env["ROS_DOMAIN_ID"]=str(domain)
        env["GZ_PARTITION"]=f"scrobot_nav2_lab_{domain}"
        return env

    def save(self,payload):
        if not isinstance(payload,dict):
            raise ValueError("Expected configuration object")
        with self.lock:
            cfg=deepcopy(self.config())
            values=payload.get("planning",{})
            if not isinstance(values,dict):
                raise ValueError("planning must be an object")
            for key in ("planner","timeout_s","depth_obstacles","resolution_m"):
                if key in values:cfg["planning"][key]=values[key]
            for section in ("start","goal"):
                if section in values:
                    if not isinstance(values[section],dict):
                        raise ValueError(section+" must be an object")
                    for coord in ("x","y","yaw_deg"):
                        if coord in values[section]:
                            cfg["planning"][section][coord]=values[section][coord]
            validate(cfg)
            tmp=self.config_path.with_suffix(".tmp")
            tmp.write_text(yaml.safe_dump(cfg,sort_keys=False))
            tmp.replace(self.config_path)
            return cfg

    def root(self):
        return Path(self.config()["experiment"]["run_root"]).expanduser()

    def stack_start(self):
        with self.lock:
            if running(self.stack):raise RuntimeError("Lab simulation is already running")
            if running(self.trial):raise RuntimeError("Stop planner trial before restarting")
            cfg=self.config()
            self.env=self.env_for(cfg)
            preflight(self.env)
            root=self.root()
            root.mkdir(parents=True,exist_ok=True)
            self.active_dir=root/"_planning_active"
            self.active_dir.mkdir(parents=True,exist_ok=True)
            self.loaded_config=deepcopy(cfg)
            map_path,geometry=write_court_map(
                WORLD,LANDMARKS,self.active_dir/"static_map",
                cfg["planning"]["resolution_m"])
            generated=nav2_params(yaml.safe_load(PRODUCTION_NAV2.read_text()),
                                  cfg,map_path,geometry)
            params=self.active_dir/"nav2_planning.yaml"
            params.write_text(yaml.safe_dump(generated,sort_keys=False))
            (self.active_dir/"config_snapshot.yaml").write_text(
                yaml.safe_dump(cfg,sort_keys=False))
            cache=self.active_dir/"paths"
            cache.mkdir(exist_ok=True)
            for f in cache.glob("*.json"):f.unlink()
            self.stack_log=self.active_dir/"stack.log"
            args=["ros2","launch",str(LAUNCH),
                  "nav2_params_file:="+str(params),
                  "path_cache:="+str(cache)]
            with self.stack_log.open("w") as log:
                self.stack=subprocess.Popen(args,env=self.env,cwd=REPO,
                  stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            self.obstacles=Obstacles(self.active_dir/"fixtures",self.env)
            self.run_dir=None
            return self.status()

    def trial_start(self):
        with self.lock:
            if not running(self.stack):raise RuntimeError("Start simulation first")
            if running(self.trial):raise RuntimeError("Planner benchmark already running")
            cfg=self.config()
            if (cfg["planning"]["depth_obstacles"]!=self.loaded_config["planning"]["depth_obstacles"]
                or cfg["planning"]["resolution_m"]!=self.loaded_config["planning"]["resolution_m"]):
                raise RuntimeError("Costmap settings changed. Restart simulation to apply them.")
            root=self.root()
            self.run_dir=root/(datetime.now().strftime("%Y%m%d_%H%M%S")+
                               "_planning_"+uuid.uuid4().hex[:5])
            self.run_dir.mkdir()
            (self.run_dir/"config_snapshot.yaml").write_text(yaml.safe_dump(cfg,sort_keys=False))
            shutil.copyfile(self.active_dir/"nav2_planning.yaml",
                            self.run_dir/"nav2_params_snapshot.yaml")
            self.run_log=self.run_dir/"trial.log"
            pid=self.planner_pid()
            cmd=[sys.executable,str(HERE/"runner.py"),
                 "--config",str(self.run_dir/"config_snapshot.yaml"),
                 "--output",str(self.run_dir),
                 "--cache",str(self.active_dir/"paths")]
            if pid:cmd+=["--planner-pid",str(pid)]
            with self.run_log.open("w") as log:
                self.trial=subprocess.Popen(cmd,env=self.env,cwd=HERE,
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            return self.status()

    def planner_pid(self):
        if not psutil or not running(self.stack):return None
        try:
            root=psutil.Process(self.stack.pid)
            for p in root.children(recursive=True):
                command=" ".join(p.cmdline())
                if ("planner_server" in command and
                    ("nav2_planner" in command or "/planner_server" in command)):
                    return p.pid
        except (psutil.NoSuchProcess,psutil.AccessDenied):
            pass
        return None

    def abort_trial(self):
        with self.lock:
            interrupt(self.trial)
            return self.status()

    def add_obstacle(self,body):
        with self.lock:
            if not running(self.stack) or self.obstacles is None:
                raise RuntimeError("Start simulation before adding obstacles")
            if running(self.trial):
                raise RuntimeError("Do not change obstacles during a planner benchmark")
            return self.obstacles.spawn(body.get("kind"),body.get("x"),body.get("y"),
                                        body.get("moving",False))

    def remove_obstacle(self,body):
        with self.lock:
            if not self.obstacles or not running(self.stack):
                raise RuntimeError("No simulation running")
            if running(self.trial):
                raise RuntimeError("Finish benchmark before modifying obstacles")
            return self.obstacles.delete(body.get("name"))

    def clear_obstacles(self):
        with self.lock:
            if not self.obstacles or not running(self.stack):
                raise RuntimeError("No simulation running")
            if running(self.trial):
                raise RuntimeError("Finish benchmark before modifying obstacles")
            return self.obstacles.clear()

    def open_rviz(self):
        with self.lock:
            if not running(self.stack):raise RuntimeError("Start simulation first")
            if running(self.rviz):return self.status()
            with (self.active_dir/"rviz.log").open("w") as log:
                self.rviz=subprocess.Popen(["rviz2","-d",str(RVIZ),
                    "--ros-args","-p","use_sim_time:=true"],
                    env=self.env,cwd=REPO,stdout=log,stderr=subprocess.STDOUT,
                    start_new_session=True)
            return self.status()

    def close_rviz(self):
        with self.lock:
            interrupt(self.rviz)
            return self.status()

    def stop(self):
        with self.lock:
            interrupt(self.trial)
            if running(self.trial):
                try:self.trial.wait(timeout=5)
                except subprocess.TimeoutExpired:interrupt(self.trial,signal.SIGTERM)
            interrupt(self.rviz)
            if self.obstacles:
                self.obstacles.shutdown()
                if running(self.stack):
                    try:self.obstacles.clear()
                    except Exception:pass
                self.obstacles=None
            interrupt(self.stack)
            self.loaded_config=None
            return self.status()

    def status(self):
        with self.lock:
            summary=status_file(self.run_dir/"summary.json") if self.run_dir else None
            outputs=[]
            if summary:
                outputs=[{"planner":name,**item}
                         for name,item in summary.get("results",{}).items()]
            return {
                "stack_running":running(self.stack),
                "stack_exit_code":self.stack.poll() if self.stack else None,
                "rviz_running":running(self.rviz),
                "trial_running":running(self.trial),
                "trial_exit_code":self.trial.poll() if self.trial else None,
                "run_name":self.run_dir.name if self.run_dir else None,
                "summary":summary,"results":outputs,
                "obstacles":self.obstacles.list() if self.obstacles else [],
                "moving_obstacle_error":self.obstacles.last_move_error if self.obstacles else "",
                "stack_log":log_tail(self.stack_log),
                "trial_log":log_tail(self.run_log),
                "domain":self.config()["simulation"]["ros_domain_id"],
                "loaded_depth":self.loaded_config["planning"]["depth_obstacles"]
                    if running(self.stack) and self.loaded_config else None,
            }

    def history(self):
        root=self.root()
        if not root.exists():return []
        output=[]
        for path in sorted(root.glob("*_planning_*/summary.json"),reverse=True)[:25]:
            info=status_file(path)
            if info:output.append({"name":path.parent.name,"summary":info})
        return output


class Handler(BaseHTTPRequestHandler):
    service=None
    def json_reply(self,data,code=200):
        body=json.dumps(data).encode()
        self.send_response(code)
        self.send_header("Content-Type","application/json; charset=utf-8")
        self.send_header("Cache-Control","no-store")
        self.send_header("X-Content-Type-Options","nosniff")
        self.send_header("Content-Length",str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path=urlparse(self.path).path
        try:
            if path=="/api/config":return self.json_reply(self.service.config())
            if path=="/api/status":return self.json_reply(self.service.status())
            if path=="/api/history":return self.json_reply(self.service.history())
            filename={"/":"index.html","/index.html":"index.html",
                      "/app.js":"app.js","/style.css":"style.css"}.get(path)
            if not filename:return self.json_reply({"error":"Not found"},404)
            body=(WEB/filename).read_bytes()
            mime={"html":"text/html","js":"application/javascript",
                  "css":"text/css"}[filename.split(".")[-1]]
            self.send_response(200)
            self.send_header("Content-Type",mime+"; charset=utf-8")
            self.send_header("Cache-Control","no-store")
            self.send_header("Content-Length",str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except Exception as exc:self.json_reply({"error":str(exc)},500)

    def do_POST(self):
        path=urlparse(self.path).path
        mapping={
            "/api/save":lambda x:{"config":self.service.save(x)},
            "/api/start":lambda x:self.service.stack_start(),
            "/api/stop":lambda x:self.service.stop(),
            "/api/plan":lambda x:self.service.trial_start(),
            "/api/cancel":lambda x:self.service.abort_trial(),
            "/api/rviz/open":lambda x:self.service.open_rviz(),
            "/api/rviz/close":lambda x:self.service.close_rviz(),
            "/api/obstacles/add":lambda x:{"obstacles":self.service.add_obstacle(x)},
            "/api/obstacles/remove":lambda x:{"obstacles":self.service.remove_obstacle(x)},
            "/api/obstacles/clear":lambda x:{"obstacles":self.service.clear_obstacles()},
        }
        if path not in mapping:return self.json_reply({"error":"Not found"},404)
        origin=self.headers.get("Origin","")
        if origin and urlparse(origin).hostname not in ("localhost","127.0.0.1"):
            return self.json_reply({"error":"Cross-origin request blocked"},403)
        if self.headers.get("Content-Type","").split(";")[0].strip()!="application/json":
            return self.json_reply({"error":"JSON body required"},415)
        try:
            length=int(self.headers.get("Content-Length","0"))
            if length<0 or length>16000:
                return self.json_reply({"error":"Invalid request size"},413)
            body=json.loads(self.rfile.read(length) or "{}")
            if not isinstance(body,dict):raise ValueError("Expected JSON object")
            return self.json_reply(mapping[path](body))
        except (ValueError,RuntimeError,subprocess.TimeoutExpired) as exc:
            return self.json_reply({"error":str(exc)},400)
        except Exception as exc:return self.json_reply({"error":str(exc)},500)

    def log_message(self,*_):
        return


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--port",type=int,default=8772)
    p.add_argument("--no-browser",action="store_true")
    a=p.parse_args()
    Handler.service=Service()
    server=ThreadingHTTPServer(("127.0.0.1",a.port),Handler)
    print(f"SCROBOT Nav2 Path Planning Lab: http://127.0.0.1:{a.port}",flush=True)
    if not a.no_browser:webbrowser.open(f"http://127.0.0.1:{a.port}")
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        Handler.service.stop()
        server.server_close()


if __name__=="__main__":main()
