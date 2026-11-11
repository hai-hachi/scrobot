#!/usr/bin/env python3
"""SCROBOT Perception Labs: camera, AprilTag and Shuttle YOLO.

Sourced ROS 2 Jazzy/SCROBOT workspace is required. Runs LOCALHOST only.
Perception simulation uses ROS_DOMAIN_ID=79; headless YOLO dataset capture
uses 80 in a separate Gazebo partition. No autonomy or real robot commands.
"""
from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
import webbrowser

from core import DEFAULTS, camera_args, pose_request, validate
from publish import publish, status as publication_status
from scene_sync import sync_dataset_world, write_dataset_provenance

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
OUTPUT=Path.home()/"scrobot_ws/evaluation_results/perception_labs"
SETTINGS=OUTPUT/"settings.json"
SOURCE_WORLD=REPO/"src/scrobot_simulation/worlds/badminton_court.sdf"
DATASET_TEMPLATE=REPO/"src/scrobot_debug/worlds/yolo_dataset.sdf"
YOLO_REPO=Path(os.environ.get("SCROBOT_YOLO_REPO",
                              str(Path.home()/"Desktop/yoloshuttle"))).expanduser()
DATASET_BASE=YOLO_REPO/"dataset"
DATASET_POINTER=OUTPUT/"latest_dataset.txt"
WORLD=OUTPUT/"worlds/yolo_dataset_lighting_current.sdf"


def latest_dataset():
    # Each lighting condition gets its own immutable dataset folder.
    if DATASET_POINTER.exists():
        path=Path(DATASET_POINTER.read_text(encoding="utf-8").strip())
        if path.is_absolute() and path.parent == DATASET_BASE:
            return path
    return DATASET_BASE/"gazebo_scrobot_lighting_current"


def tag():
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def saved_settings():
    if not SETTINGS.is_file():
        return dict(DEFAULTS)
    return validate(json.loads(SETTINGS.read_text(encoding="utf-8")))


def write_settings(s):
    OUTPUT.mkdir(parents=True,exist_ok=True)
    temp=SETTINGS.with_suffix(".tmp")
    temp.write_text(json.dumps(s,indent=2)+"\n",encoding="utf-8")
    temp.replace(SETTINGS)


def stop_group(proc, timeout=5):
    if proc is None or proc.poll() is not None:return
    for sig,wait in ((signal.SIGINT,timeout),(signal.SIGTERM,2)):
        try:os.killpg(proc.pid,sig)
        except ProcessLookupError:break
        try:proc.wait(timeout=wait);return
        except subprocess.TimeoutExpired:pass
    try:os.killpg(proc.pid,signal.SIGKILL)
    except ProcessLookupError:pass
    try:proc.wait(timeout=2)
    except subprocess.TimeoutExpired:pass


def _tail(path,limit=4000):
    if not path or not Path(path).exists():return ""
    with Path(path).open("rb") as f:
        f.seek(0,2);pos=f.tell();f.seek(max(0,pos-limit))
        return f.read().decode("utf-8","replace")


class Lab:
    def __init__(self):
        from ros_adapter import RosBridge
        OUTPUT.mkdir(parents=True,exist_ok=True)
        self.lock=threading.RLock()
        self.settings=saved_settings()
        self.applied=None
        self.bridge=RosBridge()
        self.process=None
        self.sim_log=None
        self.restarting=False
        self.error=None
        self.job=None
        self.job_kind=None
        self.job_log=None
        self.job_handle=None
        self.spawned_shuttles=[]
        self.last_record=None
        self.hardware_cache={}
        self.hardware_checked=0.0
        self.restart()

    def env(self,domain=79):
        env=os.environ.copy()
        env["ROS_DOMAIN_ID"]=str(domain)
        env["GZ_PARTITION"]=f"scrobot_perception_{os.getuid()}_{domain}"
        return env

    def stop_sim(self):
        stop_group(self.process)
        self.process=None
        if self.sim_log:
            self.sim_log.close()
            self.sim_log=None
        self.bridge.node.reset_observation()
        self.applied=None
        self.spawned_shuttles=[]

    def restart(self):
        with self.lock:
            if self.restarting:raise RuntimeError("Gazebo restart is in progress")
            if self.job is not None and self.job.poll() is None:
                raise RuntimeError("Stop dataset/training/validation before starting Gazebo")
            self.restarting=True
            s=dict(self.settings)
            try:
                self.stop_sim()
                if s["mode"]=="yolo":
                    model=self.model_path()
                    if not model.is_file():
                        raise RuntimeError(
                            f"No Shuttle YOLO model found: {model}. Clone yoloshuttle "
                            "or choose a valid existing .pt model in settings."
                        )
                args=camera_args(s)
                cmd=["ros2","launch","scrobot_simulation","perception_lab.launch.py",
                     f'mode:={s["mode"]}',
                     f'robot_x:={s["robot_x"]}',
                     f'robot_y:={s["robot_y"]}',
                     f'robot_yaw:={math.radians(s["robot_yaw_deg"])}',
                     f'enable_depth_camera:={str(s["enable_depth"]).lower()}',
                     f'enable_depth_registration:={str(s["mode"]=="yolo" or s["enable_registration"]).lower()}',
                     f'model_path:={str(self.model_path()) if s["mode"]=="yolo" else ""}',
                     f'device:={s["device"]}',
                     f'max_detection_rate:={s["max_detection_rate"]}']
                cmd += [f"{key}:={value}" for key,value in args.items()]
                self.sim_log=(OUTPUT/"gazebo.log").open("ab")
                self.process=subprocess.Popen(
                    cmd,env=self.env(),stdout=self.sim_log,
                    stderr=subprocess.STDOUT,start_new_session=True
                )
                time.sleep(.12)
                if self.process.poll() is not None:
                    raise RuntimeError("Gazebo exited during startup; inspect gazebo.log")
                self.applied=s
                self.error=None
            except Exception as exc:
                self.error=str(exc)
                raise
            finally:
                self.restarting=False

    def model_path(self):
        requested=self.settings["model_path"].strip()
        if requested:
            candidate=Path(requested).expanduser()
            return candidate if candidate.is_absolute() else YOLO_REPO/candidate
        pointer=YOLO_REPO/"artifacts/latest_model.txt"
        if pointer.is_file():
            candidate=Path(pointer.read_text(encoding="utf-8").strip()).expanduser()
            if candidate.is_file():return candidate
        return YOLO_REPO/"best.pt"

    def save_settings(self,raw):
        with self.lock:
            if self.restarting:raise RuntimeError("Restart in progress")
            s=validate(raw)
            self.settings=s
            write_settings(s)
            return s

    def publish(self):
        with self.lock:
            if not SETTINGS.is_file():raise RuntimeError("Save the lab settings first")
            if saved_settings()!=self.settings:
                raise RuntimeError("Save the current lab settings before publishing")
            return publish(self.settings)

    def gz(self,service,reqtype,reptype,request):
        cmd=["gz","service","-s",f"/world/badminton_court/{service}",
             "--reqtype",reqtype,"--reptype",reptype,
             "--timeout","3500","--req",request]
        x=subprocess.run(cmd,env=self.env(),stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT,text=True,timeout=6)
        if x.returncode!=0 or "true" not in x.stdout.lower():
            raise RuntimeError("Gazebo "+service+" failed: "+x.stdout[-700:])
        return {"success":True,"response":x.stdout.strip()[:300]}

    def pose(self,data):
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError("Restart Gazebo first")
        x=float(data.get("x"));y=float(data.get("y"));yaw=float(data.get("yaw_deg"))
        # Only reposition within approved court and prevent invalid coordinates.
        subset={**self.settings,"robot_x":x,"robot_y":y,"robot_yaw_deg":yaw}
        validate(subset)
        response=self.gz("set_pose","gz.msgs.Pose","gz.msgs.Boolean",
                         pose_request("scrobot",x,y,yaw))
        self.bridge.node.reset_observation()
        return response

    def shuttle(self,data):
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError("Restart Gazebo first")
        x=float(data["x"]);y=float(data["y"])
        validate({**self.settings,"shuttle_x":x,"shuttle_y":y})
        if self.spawned_shuttles:
            name=self.spawned_shuttles[0]
            self.gz("set_pose","gz.msgs.Pose","gz.msgs.Boolean",
                    pose_request(name,x,y,0,z=.05))
            return {"name":name,"moved":True}
        name="perception_lab_shuttle_001"
        # spawn_shuttles is the existing production shuttle model, not a fake
        # bounding box or visual proxy.
        result=subprocess.run(
            ["ros2","run","scrobot_simulation","spawn_shuttles",
             "--mode","single","--name",name,"--x",str(x),"--y",str(y)],
            env=self.env(),stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
            text=True,timeout=30,
        )
        if result.returncode!=0:
            raise RuntimeError("Shuttle spawn failed: "+result.stdout[-700:])
        self.spawned_shuttles.append(name)
        return {"name":name,"moved":False}

    def random_shuttles(self):
        n=self.settings["shuttle_count"]
        seed=self.settings["dataset_seed"]
        batch=f"perception{int(time.time())}"
        result=subprocess.run(
            ["ros2","run","scrobot_simulation","spawn_shuttles",
             "--mode","random","--count",str(n),"--seed",str(seed),
             "--batch",batch],
            env=self.env(),stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
            text=True,timeout=90,
        )
        if result.returncode!=0:
            raise RuntimeError("Random spawn failed: "+result.stdout[-700:])
        self.spawned_shuttles.extend(f"random{batch}-{i:02d}" for i in range(1,n+1))
        return {"count":n,"batch":batch}

    def clear_shuttles(self):
        names=list(self.spawned_shuttles)
        failures=[]
        for name in names:
            try:
                self.gz("remove","gz.msgs.Entity","gz.msgs.Boolean",
                        f'name: "{name}", type: MODEL')
            except (RuntimeError,subprocess.TimeoutExpired) as exc:
                failures.append(f"{name}: {exc}")
        self.spawned_shuttles=[x for x in names if any(f.startswith(x+":") for f in failures)]
        return {"removed":len(names)-len(self.spawned_shuttles),"errors":failures}

    def observe(self,seconds=3):
        seconds=float(seconds)
        if not 1<=seconds<=10:raise ValueError("Sample window must be 1–10 seconds")
        before=self.bridge.node.counts()
        start=time.monotonic()
        time.sleep(seconds)
        after=self.bridge.node.counts()
        records=[x for x in self.bridge.node.save_records()
                 if x["t"]>=self.bridge.node.get_clock().now().nanoseconds/1e9-(seconds+1)]
        tags=[x for x in records if x.get("source")=="apriltag"]
        positive=sum(bool(x.get("ids")) for x in tags)
        rgb=max(0,after["rgb"]-before["rgb"])
        summary={
            "duration_wall_s":round(time.monotonic()-start,2),
            "rgb_frames":rgb,
            "tag_messages":after["apriltag"]-before["apriltag"],
            "tag_positive_messages":positive,
            "tag_positive_fraction_of_rgb":round(min(positive/rgb,1.0),3) if rgb else None,
            "yolo2d_messages":after["yolo2d"]-before["yolo2d"],
            "yolo3d_messages":after["yolo3d"]-before["yolo3d"],
            "note":"AprilTag positive messages / RGB frames is indicative only; not ground-truth visibility-conditioned recall.",
        }
        self.last_record=summary
        return summary

    def record(self):
        t=tag()
        folder=OUTPUT/"recordings"
        folder.mkdir(parents=True,exist_ok=True)
        data={"settings":self.settings,
              "telemetry":self.bridge.node.status(self.settings),
              "sample":self.last_record,
              "records":self.bridge.node.save_records(),
              "scene_source":str(SOURCE_WORLD),
              "yoloshuttle_repo":str(YOLO_REPO)}
        output=folder/f"perception_{t}.json"
        output.write_text(json.dumps(data,indent=2,allow_nan=False)+"\n")
        return {"path":str(output),"records":len(data["records"])}

    def start_job(self,kind):
        with self.lock:
            if self.job and self.job.poll() is None:
                raise RuntimeError("A dataset/training/validation task is already running")
            if kind not in ("dataset","check_dataset","train","validate"):
                raise ValueError("Unknown YOLO task")
            if not YOLO_REPO.is_dir():
                raise RuntimeError(f"Clone hai-hachi/yoloshuttle at {YOLO_REPO} or set SCROBOT_YOLO_REPO")
            self.stop_sim()  # Free the GPU; never measure camera FPS while training.
            stamp=tag()
            python=YOLO_REPO/".venv/bin/python"
            py=str(python) if python.is_file() else sys.executable
            dataset=latest_dataset()
            if kind=="dataset":
                dataset=DATASET_BASE/f"gazebo_scrobot_lighting_{stamp}"
                provenance=sync_dataset_world(SOURCE_WORLD,DATASET_TEMPLATE,WORLD)
                write_dataset_provenance(provenance,dataset,self.settings)
                # Store pointer before capture. A failed/incomplete capture
                # will be rejected by subsequent dataset checks/training.
                DATASET_POINTER.write_text(str(dataset)+"\n",encoding="utf-8")
                cmd=["ros2","launch","scrobot_debug","yolo_dataset_capture.launch.py",
                     f"world:={WORLD}",f"output_dir:={dataset}",
                     f"session_name:=lighting_{stamp}",
                     f'target_images:={self.settings["dataset_images"]}',
                     f'shuttle_count:={self.settings["dataset_shuttles"]}',
                     f'positive_pose_fraction:={self.settings["dataset_positive_fraction"]}',
                     f'random_seed:={self.settings["dataset_seed"]}']
                env=self.env(80)
            else:
                dataset_yaml=dataset/"data.yaml"
                if not dataset_yaml.is_file():
                    raise RuntimeError(f"Dataset missing: {dataset_yaml}. Generate it first.")
                if kind=="check_dataset":
                    cmd=[py,str(YOLO_REPO/"scripts/validate_generated_dataset.py"),
                         "--dataset",str(dataset),"--samples","15"]
                elif kind=="train":
                    source=self.model_path()
                    if not source.is_file():raise RuntimeError(f"Missing initial YOLO weights: {source}")
                    cmd=[py,str(YOLO_REPO/"scripts/train_finetune.py"),
                         "--model",str(source),"--data",str(dataset_yaml),
                         "--device",self.settings["device"],
                         "--epochs",str(self.settings["train_epochs"]),
                         "--batch",str(self.settings["train_batch"]),
                         "--imgsz",str(self.settings["train_imgsz"]),
                         "--lr0",str(self.settings["train_lr"]),
                         "--patience","8","--optimizer","AdamW",
                         "--name",f"lighting_{stamp}"]
                else:
                    cmd=[py,str(YOLO_REPO/"scripts/validate_model.py"),
                         "latest","--data",str(dataset_yaml),
                         "--split","test","--device",self.settings["device"],
                         "--imgsz",str(self.settings["train_imgsz"])]
                env=self.env(80)
            self.job_kind=kind
            folder=OUTPUT/"jobs"
            folder.mkdir(parents=True,exist_ok=True)
            self.job_log=folder/f"{kind}_{stamp}.log"
            self.job_handle=self.job_log.open("ab")
            self.job=subprocess.Popen(cmd,cwd=YOLO_REPO,env=env,
                                      stdout=self.job_handle,
                                      stderr=subprocess.STDOUT,start_new_session=True)
            return {"kind":kind,"pid":self.job.pid,"log":str(self.job_log)}

    def stop_job(self):
        with self.lock:
            stop_group(self.job)
            if self.job_handle:
                self.job_handle.close()
                self.job_handle=None
            return {"stopped":True}

    def job_status(self):
        proc=self.job
        return {
            "kind":self.job_kind,"running":proc is not None and proc.poll() is None,
            "returncode":None if proc is None else proc.poll(),
            "log_path":str(self.job_log) if self.job_log else None,
            "log_tail":_tail(self.job_log,2000),
        }

    def system_status(self):
        if time.monotonic()-self.hardware_checked<2:
            return self.hardware_cache
        out={"cpu_pct":None,"memory_mb":None,"gpu_util_pct":None,
             "gpu_mem_mb":None}
        try:
            import psutil
            out["cpu_pct"]=psutil.cpu_percent(interval=None)
            out["memory_mb"]=round(psutil.virtual_memory().used/1048576)
        except ImportError:pass
        if shutil.which("nvidia-smi"):
            try:
                t=subprocess.run(
                    ["nvidia-smi","--query-gpu=utilization.gpu,memory.used",
                     "--format=csv,noheader,nounits"],
                    text=True,capture_output=True,timeout=1)
                if t.returncode==0:
                    g=t.stdout.splitlines()[0].split(",")
                    out["gpu_util_pct"]=float(g[0].strip())
                    out["gpu_mem_mb"]=float(g[1].strip())
            except (OSError,ValueError,IndexError,subprocess.TimeoutExpired):pass
        self.hardware_cache=out
        self.hardware_checked=time.monotonic()
        return out

    def status(self):
        with self.lock:
            data=self.bridge.node.status(self.settings)
            data.update({
                "settings":dict(self.settings),
                "applied":self.applied,
                "restart_needed":self.applied is None or self.applied!=self.settings,
                "sim_alive":self.process is not None and self.process.poll() is None,
                "restarting":self.restarting,
                "server_error":self.error,
                "job":self.job_status(),
                "last_sample":self.last_record,
                "publication":publication_status(self.settings),
                "yoloshuttle_found":YOLO_REPO.is_dir(),
                "yoloshuttle_path":str(YOLO_REPO),
                "model_path":str(self.model_path()),
                "model_found":self.model_path().is_file(),
                "dataset_path":str(latest_dataset()),
                "system":self.system_status(),
            })
            return data

    def close(self):
        self.stop_sim()
        stop_group(self.job)
        if self.job_handle:self.job_handle.close()
        self.bridge.close()


class Handler(BaseHTTPRequestHandler):
    server_version="SCROBOTPerceptionLabs/1.0"

    @property
    def lab(self):return self.server.lab

    def respond(self,code,payload,typ="application/json"):
        b=payload if isinstance(payload,bytes) else (
            payload.encode("utf-8") if isinstance(payload,str) else
            json.dumps(payload,allow_nan=False).encode("utf-8"))
        self.send_response(code)
        self.send_header("Content-Type",typ)
        self.send_header("Content-Length",str(len(b)))
        self.send_header("Cache-Control","no-store")
        self.send_header("X-Content-Type-Options","nosniff")
        self.end_headers()
        try:self.wfile.write(b)
        except (BrokenPipeError,ConnectionResetError):pass

    def read_json(self):
        length=int(self.headers.get("Content-Length","0"))
        if not 1<=length<=150000:
            raise ValueError("Invalid JSON request length")
        data=json.loads(self.rfile.read(length).decode("utf-8"))
        if not isinstance(data,dict):
            raise ValueError("JSON object required")
        return data

    def do_GET(self):
        path=urlsplit(self.path).path
        try:
            if path=="/":
                return self.respond(200,(HERE/"index.html").read_bytes(),
                                    "text/html; charset=utf-8")
            if path=="/api/status":
                return self.respond(200,self.lab.status())
            if path=="/api/frame":
                data=self.lab.bridge.node.frame(self.lab.settings["mode"])
                if not data:
                    return self.respond(503,{"error":"Camera has not published a frame"})
                return self.respond(200,data,"image/jpeg")
            if path=="/api/job":
                return self.respond(200,self.lab.job_status())
            return self.respond(404,{"error":"Unknown endpoint"})
        except (RuntimeError,OSError,ValueError) as exc:
            return self.respond(400,{"error":str(exc)})

    def do_POST(self):
        path=urlsplit(self.path).path
        try:
            body=self.read_json()
            if path=="/api/settings":out=self.lab.save_settings(body)
            elif path=="/api/restart":self.lab.restart();out={"restarted":True}
            elif path=="/api/pose":out=self.lab.pose(body)
            elif path=="/api/shuttle":out=self.lab.shuttle(body)
            elif path=="/api/shuttles/random":out=self.lab.random_shuttles()
            elif path=="/api/shuttles/clear":out=self.lab.clear_shuttles()
            elif path=="/api/observe":out=self.lab.observe(body.get("seconds",3))
            elif path=="/api/record":out=self.lab.record()
            elif path=="/api/publish":
                if body.get("confirm") is not True:
                    raise ValueError("Publishing requires explicit confirmation")
                out=self.lab.publish()
            elif path=="/api/job/start":out=self.lab.start_job(body.get("kind"))
            elif path=="/api/job/stop":out=self.lab.stop_job()
            else:return self.respond(404,{"error":"Unknown endpoint"})
            return self.respond(200,out)
        except (RuntimeError,ValueError,KeyError,TypeError,OSError,
                subprocess.TimeoutExpired,json.JSONDecodeError) as exc:
            return self.respond(400,{"error":str(exc)})

    def log_message(self,fmt,*args):
        if not args or "200" not in str(args[1] if len(args)>1 else ""):
            super().log_message(fmt,*args)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port",type=int,default=8768)
    parser.add_argument("--no-browser",action="store_true")
    opts=parser.parse_args()
    os.environ["ROS_DOMAIN_ID"]="79"
    os.environ["GZ_PARTITION"]=f"scrobot_perception_{os.getuid()}_79"
    lab=Lab()
    http=ThreadingHTTPServer(("127.0.0.1",opts.port),Handler)
    http.lab=lab
    url=f"http://127.0.0.1:{opts.port}"
    print("SCROBOT Perception Labs",url,"yoloshuttle:",YOLO_REPO,flush=True)
    if not opts.no_browser:webbrowser.open(url)
    try:http.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:pass
    finally:
        http.server_close()
        lab.close()


if __name__=="__main__":
    main()
