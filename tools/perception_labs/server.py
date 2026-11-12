#!/usr/bin/env python3
"""Perception Labs v3: verify physics first, then RGB, tags and YOLO independently.

Run from <scrobot_ws>/tools/perception_labs/start.sh. No real robot, wheel
controllers, tag global-localizer, mission or other labs are started.
"""
from __future__ import annotations

from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import argparse
import json
import math
import os
import random
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import urlparse
import webbrowser

from core import DEFAULTS, validate
from world import build, build_dataset, WORLD
from resources import ResourceMeter
from camera_config import publish_benchmark, status as camera_publish_status

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUTPUT = ROOT / "evaluation_results/perception_labs"
PRODUCTION = ROOT / "src/scrobot_simulation/worlds/badminton_court.sdf"
TEMPLATE = ROOT / "src/scrobot_debug/worlds/yolo_dataset.sdf"
SIM_MODELS = ROOT / "src/scrobot_simulation/models"
YOLO = Path(os.environ.get("SCROBOT_YOLO_REPO",
            str(Path.home()/"Desktop/yoloshuttle"))).expanduser()
SETTINGS = OUTPUT/"settings.json"

def now_id():
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")

def tail(path, length=4500):
    if not path or not Path(path).is_file():
        return ""
    with Path(path).open("rb") as f:
        f.seek(0, 2)
        f.seek(max(0, f.tell()-length))
        return f.read().decode("utf-8","replace")

def terminate(proc):
    if not proc or proc.poll() is not None:
        return
    for sig, seconds in ((signal.SIGINT,2),(signal.SIGTERM,1)):
        try:
            os.killpg(proc.pid,sig)
            proc.wait(timeout=seconds)
            return
        except (ProcessLookupError,subprocess.TimeoutExpired):
            pass
    try:
        os.killpg(proc.pid,signal.SIGKILL)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=1)
    except subprocess.TimeoutExpired:
        pass

class Lab:
    def __init__(self):
        OUTPUT.mkdir(parents=True,exist_ok=True)
        self.lock=threading.RLock()
        self.settings=dict(DEFAULTS)
        if SETTINGS.exists():
            try:
                self.settings=validate(json.loads(SETTINGS.read_text()))
            except (ValueError,OSError,TypeError,json.JSONDecodeError):
                # Never let an incompatible old UI settings file break startup.
                self.settings=dict(DEFAULTS)
        self.processes={}
        self.handles={}
        self.phase="idle"
        self.error=None
        self.session=0
        self.provenance=None
        self.placed={}
        self.job=None
        self.job_kind=None
        self.job_log=None
        self.job_file=None
        self.dataset=OUTPUT/"latest_dataset.txt"
        self.active_mode=None
        self.gui_requested=False
        self.resource_meter=ResourceMeter()
        self.saved_camera_benchmark=None

    def environment(self,domain=79):
        env=os.environ.copy()
        env["ROS_DOMAIN_ID"]=str(domain)
        env["ROS_AUTOMATIC_DISCOVERY_RANGE"]="LOCALHOST"
        env["GZ_PARTITION"]=f"scrobot_perception_v3_{os.getuid()}_{domain}"
        env["GZ_IP"]="127.0.0.1"
        env.pop("ROS_LOCALHOST_ONLY",None)
        paths=[str(SIM_MODELS),
               str(ROOT/"install/scrobot_simulation/share/scrobot_simulation/models"),
               str(ROOT/"install/scrobot_debug/share/scrobot_debug/models")]
        existing=env.get("GZ_SIM_RESOURCE_PATH","")
        env["GZ_SIM_RESOURCE_PATH"]=os.pathsep.join([*paths,existing])
        return env

    def spawn(self,key,argv,domain=79):
        log=OUTPUT/(key+".log")
        handle=log.open("wb")
        try:
            proc=subprocess.Popen(
                argv,env=self.environment(domain),stdout=handle,
                stderr=subprocess.STDOUT,start_new_session=True,cwd=str(ROOT))
        except Exception:
            handle.close()
            raise
        self.processes[key]=proc
        self.handles[key]=handle
        return proc

    def stop_locked(self):
        # Stop detector/bridge before Gazebo. Only processes owned by this lab.
        for key in reversed(list(self.processes)):
            terminate(self.processes.pop(key))
            self.handles.pop(key).close()
        self.active_mode=None
        self.placed.clear()
        for path in (OUTPUT/"monitor.json",OUTPUT/"camera.jpg",
                     OUTPUT/"tag_pose_reset.txt"):
            path.unlink(missing_ok=True)

    def stop(self):
        with self.lock:
            self.session+=1
            self.stop_locked()
            self.phase="stopped"
            return {"stopped":True}

    def model_path(self):
        value=self.settings.get("model","").strip()
        if value:
            p=Path(value).expanduser()
            return p if p.is_absolute() else YOLO/p
        pointer=YOLO/"artifacts/latest_model.txt"
        if pointer.is_file():
            p=Path(pointer.read_text().strip()).expanduser()
            if p.is_file():
                return p
        return YOLO/"best.pt"

    def start(self,raw):
        cfg=validate(raw)
        if cfg["mode"]=="yolo":
            model=Path(cfg["model"]).expanduser() if cfg["model"] else self.model_path()
            if cfg["model"] and not model.is_absolute():
                model=YOLO/model
            if not model.is_file():
                raise ValueError("YOLO weights missing: "+str(model)+
                    ". Camera and AprilTag can start without YOLO.")
        with self.lock:
            if self.job and self.job.poll() is None:
                raise ValueError("Stop the dataset/training job before starting Gazebo")
            self.session+=1
            token=self.session
            self.stop_locked()
            self.settings=cfg
            SETTINGS.write_text(json.dumps(cfg,indent=2)+"\n")
            self.phase="generating_world"
            self.error=None
            self.gui_requested=False
            self.saved_camera_benchmark=None
            thread=threading.Thread(target=self._start, args=(token,cfg),
                                    daemon=True,name="PerceptionGazeboStarter")
            thread.start()
        return {"started":True,"mode":cfg["mode"],"stage":"generating_world"}

    def still_current(self,token):
        with self.lock:
            return token==self.session

    def phase_set(self,token,value):
        with self.lock:
            if self.session==token:
                self.phase=value

    def _start(self,token,cfg):
        try:
            doc=build(cfg,PRODUCTION,OUTPUT/"world.sdf")
            with self.lock:
                if not self.still_current(token):
                    return
                self.provenance=doc
                self.active_mode=cfg["mode"]
                self.spawn("gazebo",["gz","sim","-r","-s","-v","3",doc["world"]])
                self.phase="waiting_for_world"
            deadline=time.monotonic()+40
            found=False
            while time.monotonic()<deadline and self.still_current(token):
                with self.lock:
                    p=self.processes.get("gazebo")
                    if p is None or p.poll() is not None:
                        raise RuntimeError("Gazebo server exited. See gazebo.log.")
                try:
                    query=subprocess.run(["gz","service","-l"],env=self.environment(),
                        stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                        text=True,timeout=2)
                    found="/world/perception_lab/create" in query.stdout
                except (OSError,subprocess.TimeoutExpired):
                    found=False
                if found:
                    break
                time.sleep(.6)
            if not self.still_current(token):
                return
            if not found:
                raise RuntimeError("Gazebo world did not advertise create service within 40 s. "
                                   "Check gazebo.log and GPU/render configuration.")
            self.phase_set(token,"world_ready")
            if cfg["mode"]=="world_smoke":
                # Verify Gazebo physics services without loading Ogre2 or ROS.
                self.phase_set(token,"world_verified")
                return
            with self.lock:
                self._write_bridge_config(cfg)
                self.spawn("bridge",[
                    "ros2","run","ros_gz_bridge","parameter_bridge",
                    "--ros-args","-p",f"config_file:={OUTPUT/'bridge.yaml'}"])
                topics=["/lab/camera/color"]
                remap=["-r","/lab/camera/color:=/camera/camera/color/image_raw"]
                if cfg["depth_enabled"]:
                    topics.append("/lab/camera/depth")
                    remap+=["-r","/lab/camera/depth:=/camera/camera/depth/image_raw"]
                self.spawn("images",[
                    "ros2","run","ros_gz_image","image_bridge",*topics,
                    "--ros-args","-p","qos:=sensor_data",*remap])
                self.spawn("monitor",[sys.executable,str(HERE/"monitor.py"),
                    "--out",str(OUTPUT),"--config",str(SETTINGS)])
                if cfg["mode"]=="apriltag":
                    self.spawn("detector",[
                        "ros2","run","apriltag_ros","apriltag_node",
                        "--ros-args","-r","__node:=apriltag",
                        "--params-file",str(ROOT/"src/scrobot_perception/config/perception.yaml"),
                        "-p","use_sim_time:=true",
                        "-r","image_rect:=/camera/camera/color/image_raw",
                        "-r","camera_info:=/camera/camera/color/camera_info",
                        "-r","detections:=/apriltag/detections"])
                elif cfg["mode"]=="yolo":
                    self.spawn("detector",[
                        "ros2","run","scrobot_perception","yolo_shuttle_detector",
                        "--ros-args","--params-file",
                        str(ROOT/"src/scrobot_perception/config/yolo_shuttle_detector.yaml"),
                        "-p","use_sim_time:=true",
                        "-p","model_path:="+str(self.model_path()),
                        "-p","device:="+cfg["device"],
                        "-p","publish_debug_image:=true",
                        "-p","aligned_depth_topic:=/camera/camera/depth/image_raw"])
                self.phase="waiting_for_rgb"
        except Exception as exc:
            with self.lock:
                if self.session==token:
                    self.phase="failed"
                    self.error=str(exc)

    def _write_bridge_config(self,cfg):
        chunks=[
            ('/clock','/clock','rosgraph_msgs/msg/Clock','gz.msgs.Clock','CLOCK'),
            ('/camera/camera/color/camera_info','/lab/camera/color/camera_info',
             'sensor_msgs/msg/CameraInfo','gz.msgs.CameraInfo','SENSOR_DATA'),
        ]
        if cfg["depth_enabled"]:
            chunks.append((
                '/camera/camera/depth/camera_info','/lab/camera/depth/camera_info',
                'sensor_msgs/msg/CameraInfo','gz.msgs.CameraInfo','SENSOR_DATA'))
        lines=[]
        for ros,gz,ros_type,gz_type,qos in chunks:
            lines += [
                "- ros_topic_name: "+ros,
                "  gz_topic_name: "+gz,
                "  ros_type_name: "+ros_type,
                "  gz_type_name: "+gz_type,
                "  direction: GZ_TO_ROS",
                "  qos_profile: "+qos,
            ]
        (OUTPUT/"bridge.yaml").write_text("\n".join(lines)+"\n")

    def gui(self):
        with self.lock:
            server=self.processes.get("gazebo")
            if not server or server.poll() is not None:
                raise ValueError("Start the Gazebo server first")
            if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
                raise ValueError("No graphical display available (DISPLAY/WAYLAND_DISPLAY). "
                                 "Server-mode camera testing can still run.")
            gui=self.processes.get("gui")
            if gui and gui.poll() is None:
                return {"running":True,"already":True}
            self.spawn("gui",["gz","sim","-g","-v","3"])
            self.gui_requested=True
        return {"running":True}

    def gz_request(self,operation,request_type,request):
        with self.lock:
            p=self.processes.get("gazebo")
            if not p or p.poll() is not None:
                raise ValueError("Gazebo server not running")
        cmd=["gz","service","-s",f"/world/{WORLD}/{operation}",
             "--reqtype",request_type,"--reptype","gz.msgs.Boolean",
             "--timeout","5000","--req",request]
        try:
            r=subprocess.run(cmd,env=self.environment(),text=True,
                             stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                             timeout=8)
        except subprocess.TimeoutExpired as exc:
            raise ValueError("Gazebo service timed out: "+operation) from exc
        if r.returncode or not re.search(r"\btrue\b",r.stdout,re.I):
            raise ValueError("Gazebo "+operation+" failed: "+r.stdout[-900:])
        return {"success":True}

    def move_robot(self,d):
        x,y,yaw=(float(d[k]) for k in ("x","y","yaw"))
        validate({**self.settings,"robot_x":x,"robot_y":y,"robot_yaw":yaw})
        angle=math.radians(yaw)/2
        text=(f'name: "perception_robot", position: {{x:{x:.5f}, y:{y:.5f}, z:0}}, '
              f'orientation: {{w:{math.cos(angle):.8f}, z:{math.sin(angle):.8f}}}')
        self.gz_request("set_pose","gz.msgs.Pose",text)
        # Reset AprilTag repeatability history when the camera is moved.
        (OUTPUT/"tag_pose_reset.txt").write_text(str(time.time_ns()))
        return {"position":[x,y],"yaw_deg":yaw,
                "note":"Camera frames after this pose replace earlier observations; wait for fresh frames."}

    def spawn_shuttle(self,d):
        x,y=(float(d[k]) for k in ("x","y"))
        validate({**self.settings,"shuttle_x":x,"shuttle_y":y})
        name=d.get("name") or f"lab_shuttle_{len(self.placed)+1:03d}"
        if not re.fullmatch(r"[a-zA-Z0-9_-]{1,64}",name):
            raise ValueError("Invalid shuttle name")
        if name in self.placed:
            text=f'name: "{name}", position: {{x:{x:.5f}, y:{y:.5f}, z:0.05}}'
            self.gz_request("set_pose","gz.msgs.Pose",text)
        else:
            sdf=SIM_MODELS/"shuttle/model.sdf"
            if not sdf.is_file():
                raise ValueError("Shuttle model missing: "+str(sdf))
            text=(f'name: "{name}", sdf_filename: "{sdf}", '
                  f'pose: {{position: {{x:{x:.5f}, y:{y:.5f}, z:0.05}}}}')
            self.gz_request("create","gz.msgs.EntityFactory",text)
        self.placed[name]=[x,y]
        return {"name":name,"initial_xy":[x,y]}

    def random_shuttles(self,n,seed):
        n=int(n);seed=int(seed)
        if not 1<=n<=20 or not 0<=seed<100000000:
            raise ValueError("Invalid random shuttle count/seed")
        placed=[]
        rng=random.Random(seed)
        for i in range(n):
            d={"name":f"lab_shuttle_{i+1:03d}",
               "x":rng.uniform(-6.1,6.1),"y":rng.uniform(-2.8,2.8)}
            placed.append(self.spawn_shuttle(d))
        return {"count":len(placed),"seed":seed}

    def clear_shuttles(self):
        removed=[]
        for name in list(self.placed):
            try:
                self.gz_request("remove","gz.msgs.Entity",
                                f'name:"{name}", type: MODEL')
                removed.append(name)
                del self.placed[name]
            except ValueError:
                pass
        return {"removed":removed,"remaining":list(self.placed)}

    def latest_dataset(self):
        if self.dataset.is_file():
            path=Path(self.dataset.read_text().strip())
            if path.is_absolute() and path.parent==YOLO/"dataset":
                return path
        return YOLO/"dataset/gazebo_scrobot_current"

    def start_job(self,kind):
        if kind not in ("dataset","check","train","validate"):
            raise ValueError("Unknown job")
        if not YOLO.is_dir():
            raise ValueError("Clone hai-hachi/yoloshuttle or set SCROBOT_YOLO_REPO")
        with self.lock:
            if self.job and self.job.poll() is None:
                raise ValueError("A dataset/training job is already running")
            self.session+=1
            self.stop_locked()
            self.phase="offline_job"
            d=self.latest_dataset()
            python=YOLO/".venv/bin/python"
            py=str(python) if python.is_file() else sys.executable
            stamp=now_id()
            if kind=="dataset":
                d=YOLO/"dataset"/f"gazebo_scrobot_current_{stamp}"
                scene=OUTPUT/"dataset_world.sdf"
                meta=build_dataset(PRODUCTION,TEMPLATE,scene)
                d.mkdir(parents=True,exist_ok=True)
                (d/"lighting_provenance.json").write_text(
                    json.dumps({**meta,"settings":self.settings},indent=2)+"\n")
                self.dataset.write_text(str(d)+"\n")
                cmd=["ros2","launch","scrobot_debug","yolo_dataset_capture.launch.py",
                     f"world:={scene}",f"output_dir:={d}",
                     f"target_images:={self.settings['dataset_images']}",
                     f"shuttle_count:={self.settings['dataset_shuttles']}",
                     f"positive_pose_fraction:={self.settings['positive_fraction']}",
                     f"random_seed:={self.settings['seed']}",
                     f"session_name:=perception_{stamp}"]
            else:
                data=d/"data.yaml"
                if not data.is_file():
                    raise ValueError("Generate a dataset first; missing "+str(data))
                if kind=="check":
                    cmd=[py,str(YOLO/"scripts/validate_generated_dataset.py"),
                         "--dataset",str(d),"--samples","15"]
                elif kind=="train":
                    summary=d/"dataset_summary.json"
                    if not summary.exists() or json.loads(summary.read_text()).get("status")!="complete":
                        raise ValueError("Only a completed, validated generated dataset may be trained")
                    model=self.model_path()
                    if not model.is_file():
                        raise ValueError("Starting weights missing: "+str(model))
                    cmd=[py,str(YOLO/"scripts/train_finetune.py"),
                         "--model",str(model),"--data",str(data),
                         "--epochs",str(self.settings["epochs"]),
                         "--batch",str(self.settings["batch"]),
                         "--imgsz",str(self.settings["imgsz"]),
                         "--device",self.settings["device"],
                         "--lr0","0.0001","--name",f"perception_{stamp}"]
                else:
                    cmd=[py,str(YOLO/"scripts/validate_model.py"),
                         "latest","--data",str(data),"--split","test",
                         "--device",self.settings["device"],
                         "--imgsz",str(self.settings["imgsz"])]
            self.job_kind=kind
            self.job_log=OUTPUT/f"job_{kind}.log"
            self.job_file=self.job_log.open("wb")
            try:
                self.job=subprocess.Popen(cmd,cwd=YOLO,env=self.environment(80),
                    stdout=self.job_file,stderr=subprocess.STDOUT,start_new_session=True)
            except Exception:
                self.job_file.close()
                self.job_file=None
                raise
            return {"job":kind,"dataset":str(d)}

    def stop_job(self):
        with self.lock:
            terminate(self.job)
            self.job=None
            if self.job_file:
                self.job_file.close()
                self.job_file=None
            self.job_kind=None
            return {"stopped":True}

    def job_status(self):
        return {
            "kind":self.job_kind,
            "running":self.job is not None and self.job.poll() is None,
            "exit_code":self.job.poll() if self.job else None,
            "log":tail(self.job_log,4500),
        }

    def save_benchmark(self):
        with self.lock:
            if self.phase not in ("streaming", "world_verified"):
                raise ValueError("A healthy Gazebo test is required before saving")
            record = {
                "created_utc": datetime.now(timezone.utc).isoformat(),
                "mode": self.active_mode, "settings": dict(self.settings),
                "scene": self.provenance, "phase": self.phase, "monitor": {},
                "resource_usage": self.resource_meter.snapshot(self.processes),
            }
            status=OUTPUT/"monitor.json"
            if status.is_file():
                record["monitor"] = json.loads(status.read_text())
            folder=OUTPUT/"benchmarks"
            folder.mkdir(parents=True,exist_ok=True)
            filename=folder/(now_id()+"_"+self.active_mode+".json")
            suffix=0
            while filename.exists():
                suffix+=1
                filename=folder/(now_id()+"_"+self.active_mode+"_"+str(suffix)+".json")
            filename.write_text(json.dumps(record,indent=2)+chr(10))
            if self.active_mode=="camera":
                self.saved_camera_benchmark=filename
            return {"saved":str(filename),"mode":self.active_mode}

    def publish_camera(self):
        with self.lock:
            if self.phase!="streaming" or self.active_mode!="camera":
                raise ValueError("Start Camera Lab and verify streaming first")
            if self.saved_camera_benchmark is None:
                raise ValueError("Save a Camera Lab benchmark before publishing")
            record=json.loads(self.saved_camera_benchmark.read_text())
            if record["settings"]!=self.settings:
                raise ValueError("Settings changed since the saved benchmark")
            return publish_benchmark(record)

    def status(self):
        with self.lock:
            processes={name:{
                "pid":p.pid,"alive":p.poll() is None,"return_code":p.poll()}
                for name,p in self.processes.items()}
            monitor={}
            path=OUTPUT/"monitor.json"
            if path.is_file():
                try:
                    monitor=json.loads(path.read_text())
                    monitor["fresh"]=time.time()-monitor["time_wall"]<4
                except (ValueError,OSError,KeyError):
                    monitor={}
            if self.phase in ("waiting_for_rgb","streaming") and monitor.get("fresh"):
                rgb=monitor.get("streams",{}).get("rgb",{})
                if rgb.get("samples",0)>0 and (rgb.get("age_s") or 99)<3:
                    self.phase="streaming"
            if "gazebo" in processes and not processes["gazebo"]["alive"]:
                self.phase="failed"
                self.error=self.error or "Gazebo server exited; open gazebo.log"
            logs={k:tail(OUTPUT/(k+".log"),3000) for k in
                  ("gazebo","bridge","images","monitor","detector","gui")}
            return {
                "phase":self.phase,"error":self.error,"mode":self.active_mode,
                "resources":self.resource_meter.snapshot(self.processes),
                "camera_publication":camera_publish_status(self.settings),
                "camera_benchmark_saved":str(self.saved_camera_benchmark)
                    if self.saved_camera_benchmark else None,
                "settings":self.settings,"processes":processes,
                "monitor":monitor,"logs":logs,"scene":self.provenance,
                "placed":self.placed,"job":self.job_status(),
                "model_path":str(self.model_path()),
                "model_found":self.model_path().is_file(),
                "yoloshuttle":str(YOLO),"yoloshuttle_found":YOLO.is_dir(),
                "dataset":str(self.latest_dataset()),
                "display_available":bool(os.environ.get("DISPLAY") or
                    os.environ.get("WAYLAND_DISPLAY")),
            }

    def close(self):
        self.stop()
        self.stop_job()

class Handler(BaseHTTPRequestHandler):
    @property
    def lab(self):
        return self.server.lab

    def reply(self,status,value,media="application/json"):
        data=value if isinstance(value,bytes) else (
            value.encode() if isinstance(value,str) else json.dumps(value).encode())
        self.send_response(status)
        self.send_header("Content-Type",media)
        self.send_header("Content-Length",str(len(data)))
        self.send_header("Cache-Control","no-store")
        self.send_header("X-Content-Type-Options","nosniff")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError,ConnectionResetError):
            pass

    def body(self):
        length=int(self.headers.get("Content-Length","0"))
        if not 2<=length<=120000:
            raise ValueError("Invalid JSON request length")
        obj=json.loads(self.rfile.read(length))
        if not isinstance(obj,dict):
            raise ValueError("JSON object required")
        return obj

    def do_GET(self):
        path=urlparse(self.path).path
        try:
            if path=="/":
                return self.reply(200,(HERE/"index.html").read_bytes(),
                                  "text/html; charset=utf-8")
            if path=="/api/status":
                return self.reply(200,self.lab.status())
            if path=="/api/image":
                file=OUTPUT/"camera.jpg"
                if file.is_file() and time.time()-file.stat().st_mtime<5:
                    return self.reply(200,file.read_bytes(),"image/jpeg")
                return self.reply(503,{"error":"No fresh ROS camera image"})
            return self.reply(404,{"error":"Unknown endpoint"})
        except (OSError,ValueError) as exc:
            return self.reply(400,{"error":str(exc)})

    def do_POST(self):
        path=urlparse(self.path).path
        try:
            d=self.body()
            if path=="/api/start":out=self.lab.start(d)
            elif path=="/api/stop":out=self.lab.stop()
            elif path=="/api/gui":out=self.lab.gui()
            elif path=="/api/pose":out=self.lab.move_robot(d)
            elif path=="/api/shuttle":out=self.lab.spawn_shuttle(d)
            elif path=="/api/random":out=self.lab.random_shuttles(d.get("count"),d.get("seed"))
            elif path=="/api/clear":out=self.lab.clear_shuttles()
            elif path=="/api/job":out=self.lab.start_job(d.get("kind"))
            elif path=="/api/job/stop":out=self.lab.stop_job()
            elif path=="/api/benchmark":out=self.lab.save_benchmark()
            elif path=="/api/camera/publish":
                if d.get("confirm") is not True:
                    raise ValueError("Explicit publication confirmation required")
                out=self.lab.publish_camera()
            else:return self.reply(404,{"error":"Unknown endpoint"})
            return self.reply(200,out)
        except (ValueError,RuntimeError,OSError,TypeError,KeyError,
                subprocess.TimeoutExpired,json.JSONDecodeError) as exc:
            return self.reply(400,{"error":str(exc)})

    def log_message(self,format,*args):
        if "200" not in str(args[1] if len(args)>1 else ""):
            super().log_message(format,*args)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--port",type=int,default=8768)
    parser.add_argument("--no-browser",action="store_true")
    args=parser.parse_args()
    os.environ["ROS_DOMAIN_ID"]="79"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"]="LOCALHOST"
    os.environ["GZ_PARTITION"]=f"scrobot_perception_v3_{os.getuid()}_79"
    os.environ["GZ_IP"]="127.0.0.1"
    os.environ.pop("ROS_LOCALHOST_ONLY",None)
    lab=Lab()
    web=ThreadingHTTPServer(("127.0.0.1",args.port),Handler)
    web.lab=lab
    url=f"http://127.0.0.1:{args.port}"
    print("Perception Labs v3:",url,flush=True)
    print("Start with MINIMAL SMOKE TEST in the dashboard; Gazebo will not auto-start.",flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        web.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        web.server_close()
        lab.close()

if __name__=="__main__":
    main()
