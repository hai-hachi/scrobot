"""Tracker Lab lifecycle: reproducible physics runs and safety-scoped files."""
import json
import math
import os
from copy import deepcopy
from datetime import datetime
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import uuid

ROOT=Path(__file__).resolve().parents[3]
TOOLS=ROOT/"tools"
sys.path.insert(0,str(TOOLS))
from shared.lab_configs import read as read_lab,save as save_lab,path_for
from shared.lab_ui import GazeboGUI,log_tail
from shared.sim_speed import get as speed_get,save as speed_save
from planning.process_metrics import ProcessSampler
from tracking.core import defaults,validate,reference,initial_pose
from tracking.world import write_world
from tracking.configuration import generate

try:
    import psutil
except ImportError:
    psutil=None

HERE=Path(__file__).resolve().parent
LAUNCH=HERE/"launch/tracking.launch.py"
RVIZ=HERE/"config/tracking.rviz"
RUNS=ROOT/"evaluation_results/nav2_lab/tracking"
RUN_PATTERN=re.compile(r"[0-9]{8}_[0-9]{6}_tracking_[0-9a-f]{5}\Z")


def running(proc):
    return proc is not None and proc.poll() is None


def interrupt(proc,sig=signal.SIGINT):
    if running(proc):
        try:os.killpg(proc.pid,sig)
        except ProcessLookupError:pass


def read_json(file):
    try:return json.loads(Path(file).read_text())
    except (OSError,ValueError,TypeError):return None


def preflight(env):
    if not shutil.which("ros2") or not shutil.which("gz"):
        raise RuntimeError("Source ROS 2 Jazzy, Gazebo Harmonic and the workspace")
    for pkg in ("scrobot_simulation","scrobot_tracker_controller",
                "nav2_controller","ros_gz_sim","ros_gz_bridge","controller_manager"):
        result=subprocess.run(["ros2","pkg","prefix",pkg],env=env,
            stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=10)
        if result.returncode:
            raise RuntimeError("Missing ROS package "+pkg+
                               " — build and source the workspace")
    # Jazzy upstream controller_server schedules with WallRate.
    # Do not silently run a 25 Hz wall loop at 0.25 RTF.
    sys.path.insert(0,str(TOOLS/"performance_lab"))
    from nav2_timing import installed_sim_clock_overlay
    if not installed_sim_clock_overlay()["active"]:
        raise RuntimeError(
          "Nav2 ROS-clock controller overlay is not active. "
          "Run: python3 tools/performance_lab/prepare_nav2_sim_clock.py --build; "
          "source install/setup.bash. Unpatched Jazzy WallRate would invalidate 0.25 RTF tracking.")
    topics=subprocess.run(["ros2","topic","list"],env=env,capture_output=True,
                           text=True,timeout=10)
    if topics.returncode:
        raise RuntimeError("ROS graph unavailable")
    forbidden={"/clock","/evaluation/ground_truth_odom",
               "/controller_server","/diff_drive_controller/odom"}
    if forbidden.intersection(topics.stdout.splitlines()):
        raise RuntimeError("Tracking ROS domain already has an active simulator or controller")


class TrackingService:
    def __init__(self,run_root=RUNS):
        self.root=Path(run_root)
        self.lock=threading.RLock()
        self.stack=None
        self.trial=None
        self.rviz=None
        self.gui=None
        self.env=None
        self.active_dir=None
        self.run_dir=None
        self.stack_log=None
        self.run_log=None
        self.settings_at_launch=None
        self.applied_rtf=None
        self.sampler=ProcessSampler(psutil,interval_s=1.5)

    def config(self):
        saved=read_lab("tracking")
        return deepcopy(validate(saved if saved is not None else defaults()))

    def save(self,data):
        checked=deepcopy(validate(data))
        save_lab("tracking",checked,validator=validate)
        return checked

    def sim_speed(self):
        return {"max_rtf":speed_get("nav2"),
                "applied_max_rtf":self.applied_rtf if running(self.stack) else None,
                "restart_required":running(self.stack)}

    def set_speed(self,value):
        with self.lock:
            speed_save("nav2",value)
            return self.sim_speed()

    def preview(self,values):
        checked=validate(values)
        return {"path":checked["path"],
                "points":[[round(x,4),round(y,4)] for x,y in reference(checked["path"])],
                "pose":initial_pose(checked),
                "controller":checked["controller"]}

    def _safe_folder(self,name):
        if not isinstance(name,str) or not RUN_PATTERN.fullmatch(name):
            raise ValueError("Invalid experiment ID")
        folder=self.root/name
        base=self.root.resolve()
        if folder.is_symlink() or folder.resolve().parent!=base:
            raise ValueError("Unsafe experiment folder")
        if not folder.is_dir() or not (folder/"summary.json").is_file():
            raise FileNotFoundError("Experiment not found")
        return folder

    def _history_rows(self):
        if not self.root.exists():return []
        rows=[]
        for folder in sorted(self.root.iterdir(),reverse=True):
            if not folder.is_dir() or folder.is_symlink() or not RUN_PATTERN.fullmatch(folder.name):
                continue
            summary=read_json(folder/"summary.json")
            if summary:
                rows.append({"name":folder.name,"summary":summary})
        return rows

    def history(self):
        return self._history_rows()[:50]

    def delete(self,body):
        with self.lock:
            if running(self.trial):
                raise RuntimeError("Stop tracking before deleting experiments")
            if body.get("confirm") is not True:
                raise ValueError("Confirm experiment deletion")
            folder=self._safe_folder(body.get("name"))
            shutil.rmtree(folder)
            if self.run_dir==folder:
                self.run_dir=None;self.run_log=None
            return {"deleted":folder.name}

    def clear_history(self,body):
        with self.lock:
            if running(self.trial):
                raise RuntimeError("Stop tracking before deleting experiments")
            if body.get("confirm") is not True:
                raise ValueError("Confirm clearing tracking history")
            names=[entry["name"] for entry in self._history_rows()]
            for name in names:shutil.rmtree(self._safe_folder(name))
            if self.run_dir is not None and self.run_dir.name in names:
                self.run_dir=None;self.run_log=None
            return {"deleted":len(names)}

    def _stack_start(self,checked):
        if running(self.stack):
            raise RuntimeError("Tracker simulator is already running")
        self.env=os.environ.copy()
        self.env["ROS_DOMAIN_ID"]="78"
        self.env["GZ_PARTITION"]="scrobot_tracker_lab_78"
        preflight(self.env)
        self.root.mkdir(parents=True,exist_ok=True)
        self.active_dir=self.root/"_tracking_active"
        self.active_dir.mkdir(parents=True,exist_ok=True)
        self.applied_rtf=speed_get("nav2")
        world=write_world(self.active_dir/"tracking_flat_world.sdf",self.applied_rtf)
        profiles=generate(checked,self.active_dir)
        physics=profiles["inputs"]["mobility"]["physics"]
        motor=profiles["inputs"]["mobility"]["motor"]
        odom=profiles["inputs"]["odometry"]
        pose=initial_pose(checked)
        (self.active_dir/"reference.json").write_text(
            json.dumps({"points":reference(checked["path"])}))
        (self.active_dir/"config_snapshot.json").write_text(
            json.dumps({**checked,"resolved_pose":pose,
                        "max_rtf":self.applied_rtf},indent=2))
        (self.active_dir/"health.json").unlink(missing_ok=True)
        self.settings_at_launch=deepcopy(checked)
        self.gui=GazeboGUI(self.active_dir/"gazebo_gui.log",
                          env=self.env,cwd=ROOT)
        args=["ros2","launch",str(LAUNCH),
              "world:="+str(world),
              "path_json:="+str(self.active_dir/"reference.json"),
              "monitor_file:="+str(self.active_dir/"health.json"),
              "nav2_params:="+profiles["nav2_params"],
              "controller_params:="+profiles["controller_params"],
              "robot_x:="+str(pose["x"]),"robot_y:="+str(pose["y"]),
              "robot_yaw:="+str(math.radians(pose["yaw_deg"])),
              "ground_truth_rate_hz:="+str(odom["ground_truth_hz"]),
              "enable_motor_response:="+("true" if motor["model"]=="first_order" else "false")]
        args.extend(f"{name}:={value}" for name,value in physics.items())
        args.extend(f"{name}:={motor[name]}" for name in (
            "tau_left","tau_right","gain_left","gain_right"))
        self.stack_log=self.active_dir/"stack.log"
        with self.stack_log.open("w") as logfile:
            self.stack=subprocess.Popen(args,env=self.env,cwd=ROOT,
                stdout=logfile,stderr=subprocess.STDOUT,start_new_session=True)
        return self.status()

    def start(self,values):
        with self.lock:
            if running(self.trial):
                raise RuntimeError("Stop current trial before restarting")
            config=deepcopy(validate(values))
            if running(self.stack):
                raise RuntimeError("Simulator is running; use Restart/Reset")
            return self._stack_start(config)

    def reset(self,values):
        with self.lock:
            if running(self.trial):
                raise RuntimeError("Stop current trial before resetting")
            config=deepcopy(validate(values))
            self.stop()
            return self._stack_start(config)

    def run(self,values):
        with self.lock:
            if running(self.trial):raise RuntimeError("Tracking trial already running")
            checked=deepcopy(validate(values))
            # Rebuild the world for EVERY comparison after a trial, guaranteeing
            # identical initial conditions (no teleport physics/odom residue).
            if (not running(self.stack) or self.trial is not None or
                    checked!=self.settings_at_launch):
                self.stop()
                self._stack_start(checked)
            self.root.mkdir(parents=True,exist_ok=True)
            self.run_dir=self.root/(datetime.now().strftime("%Y%m%d_%H%M%S")+
                              "_tracking_"+uuid.uuid4().hex[:5])
            self.run_dir.mkdir()
            session=json.loads((self.active_dir/"config_snapshot.json").read_text())
            path_file=self.run_dir/"config_snapshot.json"
            path_file.write_text(json.dumps(session,indent=2))
            shutil.copyfile(self.active_dir/"reference.json",
                            self.run_dir/"reference.json")
            shutil.copyfile(self.active_dir/"calibration_snapshot.yaml",
                            self.run_dir/"calibration_snapshot.yaml")
            shutil.copyfile(self.active_dir/"tracking_controller.yaml",
                            self.run_dir/"nav2_params_snapshot.yaml")
            self.run_log=self.run_dir/"trial.log"
            command=[sys.executable,str(HERE/"runner.py"),
                     "--config",str(path_file),"--output",str(self.run_dir)]
            with self.run_log.open("w") as logfile:
                self.trial=subprocess.Popen(command,env=self.env,cwd=HERE.parent,
                    stdout=logfile,stderr=subprocess.STDOUT,start_new_session=True)
            return self.status()

    def abort(self):
        with self.lock:
            interrupt(self.trial)
            return self.status()

    def open_gui(self):
        with self.lock:
            if not running(self.stack):
                raise RuntimeError("Start tracking simulation first")
            self.gui.open()
            return self.status()

    def close_gui(self):
        with self.lock:
            if self.gui:self.gui.close()
            return self.status()

    def open_rviz(self):
        with self.lock:
            if not running(self.stack):
                raise RuntimeError("Start tracking simulation first")
            if running(self.rviz):return self.status()
            dest=self.active_dir/"rviz.log"
            with dest.open("w") as logfile:
                self.rviz=subprocess.Popen(
                    ["rviz2","-d",str(RVIZ),"--ros-args","-p","use_sim_time:=true"],
                    cwd=ROOT,env=self.env,stdout=logfile,
                    stderr=subprocess.STDOUT,start_new_session=True)
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
                except subprocess.TimeoutExpired:
                    interrupt(self.trial,signal.SIGTERM)
            interrupt(self.rviz)
            if self.gui:self.gui.close()
            interrupt(self.stack)
            if running(self.stack):
                try:self.stack.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    interrupt(self.stack,signal.SIGTERM)
                    try:self.stack.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        interrupt(self.stack,signal.SIGKILL)
            self.sampler.reset()
            self.stack=None
            self.trial=None
            self.rviz=None
            self.applied_rtf=None
            return self.status()

    def status(self):
        with self.lock:
            summary=read_json(self.run_dir/"summary.json") if self.run_dir else None
            live=read_json(self.run_dir/"live.json") if self.run_dir else None
            return {
                "stack_running":running(self.stack),
                "trial_running":running(self.trial),
                "stack_exit_code":self.stack.poll() if self.stack else None,
                "trial_exit_code":self.trial.poll() if self.trial else None,
                "rviz_running":running(self.rviz),
                "gazebo_gui":self.gui.status() if self.gui else {"running":False},
                "run_name":self.run_dir.name if self.run_dir else None,
                "summary":summary,"live":live,
                "health":read_json(self.active_dir/"health.json")
                    if running(self.stack) and self.active_dir else None,
                "applied_max_rtf":self.applied_rtf,
                "calibration_source":(
                    read_json(self.active_dir/"config_snapshot.json")
                    if running(self.stack) and self.active_dir else None),
                "resources":self.sampler.sample(
                    self.stack.pid if running(self.stack) else None,
                    self.rviz.pid if running(self.rviz) else None),
                "stack_log":log_tail(self.stack_log),
                "trial_log":log_tail(self.run_log),
                "rviz_log":log_tail(self.active_dir/"rviz.log")
                    if self.active_dir else "",
                "gazebo_log":log_tail(self.gui.log_path) if self.gui else "",
            }
