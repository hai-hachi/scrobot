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
from planning.core import PLANNERS, nav2_params, validate
from planning.map_builder import write_court_map
from planning.obstacles import Obstacles
from planning.scene import load_scene, save_scene, validate_scene, SCENE_PATH
from planning.calibration import (load_inputs, build_world, generate_rate_profiles,
                         inputs_fingerprint, saved_summary)

try: import psutil
except ImportError: psutil=None

HERE=Path(__file__).resolve().parent
LAB=HERE.parent
REPO=HERE.parents[2]
sys.path.insert(0,str(REPO/"tools"))
from shared.lab_ui import GazeboGUI
from shared.lab_configs import (path_for as lab_yaml_path, read as read_lab_yaml,
                                save as save_lab_yaml)
CONFIG=lab_yaml_path("nav2")
TEMPLATE=LAB/"config"/"lab.yaml"
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
        self.gui=None
        self.trial=None
        self.active_dir=None
        self.env=None
        self.obstacles=None
        self.run_dir=None
        self.stack_log=None
        self.run_log=None
        self.loaded_config=None
        self.loaded_inputs=None
        previous=self.config()["planning"]["start"]
        self.scene=load_scene(default={"robot":previous,"obstacles":[]})
        self.active_robot=None

    def scene_status(self):
        return {"scene":deepcopy(self.scene),
                "path":str(SCENE_PATH),
                "saved":SCENE_PATH.exists(),
                "restart_required":(
                    running(self.stack) and
                    self.active_robot is not None and
                    self.scene["robot"]!=self.active_robot)}

    def save_scene(self,body):
        with self.lock:
            proposal=deepcopy(self.scene)
            if "robot" in body:
                proposal["robot"]=body["robot"]
            self.scene=save_scene(validate_scene(proposal))
            return self.scene_status()

    def load_scene(self):
        with self.lock:
            if running(self.stack):
                raise RuntimeError("Stop simulation before loading saved scene; live Gazebo entities would differ.")
            if not SCENE_PATH.exists():
                raise FileNotFoundError("No saved Nav2 scene YAML yet. Click Save Scene YAML first.")
            self.scene=load_scene(default=self.scene)
            return self.scene_status()

    def place_robot(self,body):
        with self.lock:
            proposed=deepcopy(self.scene)
            proposed["robot"]=body.get("robot")
            proposed=validate_scene(proposed)
            self.scene=proposed
            if running(self.stack):
                # Reinitialize odometry, TF and costmaps consistently.
                self.stop()
                return self.stack_start()
            return self.scene_status()

    def config(self):
        # Nav2 owns an independently saved YAML under tools/configs/labs/.
        # Until first Save, use its bundled template rather than production.
        if self.config_path == CONFIG:
            data=read_lab_yaml("nav2")
            if data is None:
                data=yaml.safe_load(TEMPLATE.read_text())
        else:
            data=yaml.safe_load(self.config_path.read_text())
        return validate(data)

    def calibration_status(self):
        current=load_inputs()
        response=saved_summary(current)
        response["restart_required"]=(
            running(self.stack) and self.loaded_inputs is not None and
            inputs_fingerprint(current)!=inputs_fingerprint(self.loaded_inputs)
        )
        response["calibration_snapshot_file"]=(
            str(self.active_dir/"calibration_snapshot.yaml")
            if running(self.stack) and self.active_dir else None
        )
        return response

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
            cfg["planning"]["start"]=deepcopy(self.scene["robot"])
            cfg["simulation"]["start_x"]=self.scene["robot"]["x"]
            cfg["simulation"]["start_y"]=self.scene["robot"]["y"]
            cfg["simulation"]["start_yaw"]=__import__("math").radians(
                self.scene["robot"]["yaw_deg"])
            validate(cfg)
            if self.config_path == CONFIG:
                save_lab_yaml("nav2",cfg,validator=validate)
            else:
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
            loaded_scene=validate_scene(self.scene)
            cfg["planning"]["start"]=deepcopy(loaded_scene["robot"])
            cfg["simulation"]["start_x"]=loaded_scene["robot"]["x"]
            cfg["simulation"]["start_y"]=loaded_scene["robot"]["y"]
            cfg["simulation"]["start_yaw"]=__import__("math").radians(
                loaded_scene["robot"]["yaw_deg"])
            calibration=load_inputs()
            self.env=self.env_for(cfg)
            preflight(self.env)
            root=self.root()
            root.mkdir(parents=True,exist_ok=True)
            self.active_dir=root/"_planning_active"
            self.active_dir.mkdir(parents=True,exist_ok=True)
            (self.active_dir/"map_status.json").unlink(missing_ok=True)
            # Build world + ROS overrides from validated saved owner YAMLs.
            # These are frozen for the entire Gazebo session.
            scene=build_world(calibration,self.active_dir,loaded_scene)
            self.active_robot=deepcopy(loaded_scene["robot"])
            profiles=generate_rate_profiles(calibration,self.active_dir)
            self.loaded_config=deepcopy(cfg)
            self.loaded_inputs=calibration
            (self.active_dir/"calibration_snapshot.yaml").write_text(
                yaml.safe_dump(calibration,sort_keys=False))
            resource=scene["tag_resource_dir"]
            old_resource=self.env.get("GZ_SIM_RESOURCE_PATH","")
            self.env["GZ_SIM_RESOURCE_PATH"]=(
                resource+(os.pathsep+old_resource if old_resource else "")
            )
            map_path,geometry=write_court_map(
                scene["world_path"],scene["landmarks_path"],
                self.active_dir/"static_map",cfg["planning"]["resolution_m"])
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
            self.gui = GazeboGUI(self.active_dir/"gazebo_gui.log", env=self.env, cwd=REPO)
            camera=calibration["camera"]
            mobility=calibration["mobility"]["physics"]
            rates=calibration["odometry"]
            cw,ch=map(int,camera["color"].split("x"))
            dw,dh=map(int,camera["depth"].split("x"))
            robot=loaded_scene["robot"]
            launch_values={
                "robot_x":robot["x"], "robot_y":robot["y"],
                "robot_yaw_rad":__import__("math").radians(robot["yaw_deg"]),
                "map_status_file":str(self.active_dir/"map_status.json"),
                "world":scene["world_path"],
                "controller_params":profiles["controller_params"],
                "ekf_params":profiles["ekf_params"],
                "color_width":cw,"color_height":ch,"color_fps":camera["color_fps"],
                "depth_width":dw,"depth_height":dh,"depth_fps":camera["depth_fps"],
                **mobility,
                "imu_rate_hz":rates["imu_hz"],
                "magnetometer_rate_hz":rates["magnetometer_hz"],
                "ground_truth_rate_hz":rates["ground_truth_hz"],
                "use_magnetometer":str(calibration["use_magnetometer"]).lower(),
            }
            args=["ros2","launch",str(LAUNCH),
                  "nav2_params_file:="+str(params),
                  "path_cache:="+str(cache)]
            args.extend(f"{key}:={value}" for key,value in launch_values.items())
            with self.stack_log.open("w") as log:
                self.stack=subprocess.Popen(args,env=self.env,cwd=REPO,
                  stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            self.obstacles=Obstacles(self.active_dir/"fixtures",self.env,
                                     initial_specs=loaded_scene["obstacles"],
                                     clock_path=self.active_dir/"map_status.json")
            self.run_dir=None
            return self.status()

    def trial_start(self):
        with self.lock:
            if not running(self.stack):raise RuntimeError("Start simulation first")
            if running(self.trial):raise RuntimeError("Planner benchmark already running")
            cfg=self.config()
            if self.scene["robot"] != self.active_robot:
                raise RuntimeError("Robot pose changed: restart simulation before planning.")
            cfg["planning"]["start"]=deepcopy(self.active_robot)
            cfg["simulation"]["start_x"]=self.active_robot["x"]
            cfg["simulation"]["start_y"]=self.active_robot["y"]
            cfg["simulation"]["start_yaw"]=__import__("math").radians(
                self.active_robot["yaw_deg"])
            if inputs_fingerprint(load_inputs()) != inputs_fingerprint(self.loaded_inputs):
                raise RuntimeError(
                    "Another lab's saved calibration changed since Nav2 launch. "
                    "Restart this lab to apply it before benchmarking.")
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
            shutil.copyfile(self.active_dir/"calibration_snapshot.yaml",
                            self.run_dir/"calibration_snapshot.yaml")
            (self.run_dir/"scene_snapshot.yaml").write_text(
                yaml.safe_dump(validate_scene(self.scene),sort_keys=False))
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
            if running(self.trial):
                raise RuntimeError("Do not change obstacles during a benchmark")
            spec=body.get("obstacle")
            candidate=deepcopy(self.scene)
            candidate["obstacles"].append(spec)
            candidate=validate_scene(candidate)
            selected=candidate["obstacles"][-1]
            if running(self.stack):
                if not self.obstacles:
                    raise RuntimeError("Obstacle manager unavailable")
                self.obstacles.spawn(selected)
            self.scene=candidate
            return self.scene_status()

    def remove_obstacle(self,body):
        with self.lock:
            if running(self.trial):
                raise RuntimeError("Finish benchmark before changing obstacles")
            index=body.get("index")
            if type(index) is not int or not 0<=index<len(self.scene["obstacles"]):
                raise ValueError("Select a valid obstacle index")
            if running(self.stack):
                name=self.obstacles.list()[index]["name"]
                self.obstacles.delete(name)
            self.scene["obstacles"].pop(index)
            return self.scene_status()

    def clear_obstacles(self):
        with self.lock:
            if running(self.trial):
                raise RuntimeError("Finish benchmark before changing obstacles")
            if running(self.stack) and self.obstacles:
                self.obstacles.clear()
            self.scene["obstacles"]=[]
            return self.scene_status()

    def open_gui(self):
        with self.lock:
            if not running(self.stack):
                raise RuntimeError("Start simulation before opening Gazebo GUI")
            if self.gui is None:
                raise RuntimeError("Gazebo GUI helper not initialized")
            self.gui.open()
            return self.status()

    def close_gui(self):
        with self.lock:
            if self.gui:
                self.gui.close()
            return self.status()

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
            if self.gui:
                self.gui.close()
            if self.obstacles:
                self.obstacles.shutdown()
                if running(self.stack):
                    try:self.obstacles.clear()
                    except Exception:pass
                self.obstacles=None
            interrupt(self.stack)
            if running(self.stack):
                try:
                    self.stack.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    interrupt(self.stack,signal.SIGTERM)
                    try:self.stack.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        interrupt(self.stack,signal.SIGKILL)
                        self.stack.wait(timeout=3)
            self.loaded_config=None
            self.loaded_inputs=None
            self.active_robot=None
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
                "gazebo_gui":self.gui.status() if self.gui else {"running":False},
                "gui_log":log_tail(self.gui.log_path) if self.gui else "",
                "rviz_log":log_tail(self.active_dir/"rviz.log") if self.active_dir else "",
                "trial_running":running(self.trial),
                "trial_exit_code":self.trial.poll() if self.trial else None,
                "run_name":self.run_dir.name if self.run_dir else None,
                "summary":summary,"results":outputs,
                "obstacles":self.obstacles.list() if self.obstacles else [],
                "scene":deepcopy(self.scene),
                "scene_yaml":str(SCENE_PATH),
                "scene_saved":SCENE_PATH.exists(),
                "active_robot":deepcopy(self.active_robot),
                "scene_restart_required":(
                    running(self.stack) and self.active_robot is not None and
                    self.active_robot!=self.scene["robot"]),
                "map_status":status_file(self.active_dir/"map_status.json")
                    if running(self.stack) and self.active_dir else None,
                "moving_obstacle_error":self.obstacles.last_move_error if self.obstacles else "",
                "stack_log":log_tail(self.stack_log),
                "trial_log":log_tail(self.run_log),
                "domain":self.config()["simulation"]["ros_domain_id"],
                "calibration":saved_summary(self.loaded_inputs)
                    if self.loaded_inputs is not None else None,
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
            if path=="/api/scene":return self.json_reply(self.service.scene_status())
            if path=="/api/calibration":return self.json_reply(
                self.service.calibration_status())
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
            "/api/scene/save":lambda x:self.service.save_scene(x),
            "/api/scene/load":lambda x:self.service.load_scene(),
            "/api/scene/place-robot":lambda x:self.service.place_robot(x),
            "/api/start":lambda x:self.service.stack_start(),
            "/api/stop":lambda x:self.service.stop(),
            "/api/plan":lambda x:self.service.trial_start(),
            "/api/cancel":lambda x:self.service.abort_trial(),
            "/api/gazebo-gui/open":lambda x:self.service.open_gui(),
            "/api/gazebo-gui/close":lambda x:self.service.close_gui(),
            "/api/rviz/open":lambda x:self.service.open_rviz(),
            "/api/rviz/close":lambda x:self.service.close_rviz(),
            "/api/obstacles/add":lambda x:self.service.add_obstacle(x),
            "/api/obstacles/remove":lambda x:self.service.remove_obstacle(x),
            "/api/obstacles/clear":lambda x:self.service.clear_obstacles(),
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
    if not a.no_browser:webbrowser.open_new_tab(f"http://127.0.0.1:{a.port}")
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        Handler.service.stop()
        server.server_close()


if __name__=="__main__":main()
