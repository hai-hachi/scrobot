#!/usr/bin/env python3
"""SCROBOT full-stack Performance Lab: timing, sync audit and run comparisons.

Runs the genuine scrobot_bringup sim.launch.py. It never publishes calibration
or changes the production world: only a copied world has its RTF cap adjusted.
No automatic throttle of controller callbacks or artificial clock rewriting.
"""
from __future__ import annotations

import argparse
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from urllib.parse import urlparse,unquote
import uuid
import webbrowser

if __package__ in (None,""):
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))

from tools.performance_lab.settings import (
    ROOT,WORLD,RESULTS,validate,read_settings,save_settings,prepare_world,
    calibration_snapshot,
)
from tools.shared.lab_ui import GazeboGUI,log_tail
from tools.nav2_lab.planning.process_metrics import ProcessSampler
from tools.performance_lab.thread_metrics import ThreadSampler
from tools.performance_lab.nav2_timing import (
    ControllerWarningMeter,installed_controller_frequency,
    nav2_scheduler_expectations,installed_sim_clock_overlay,
)

try:import psutil
except ImportError:psutil=None

HERE=Path(__file__).resolve().parent
WEB=HERE/"web"
ACTIVE=RESULTS/"_active"
RUN_NAME=re.compile(r"performance_[0-9]{8}T[0-9]{6}_[0-9a-f]{6}\.json\Z")


def running(proc):
    return bool(proc and proc.poll() is None)


def kill_group(proc):
    if not running(proc):return
    for sig,delay in ((signal.SIGINT,5),(signal.SIGTERM,4),(signal.SIGKILL,2)):
        if not running(proc):break
        try:
            os.killpg(proc.pid,sig)
            proc.wait(timeout=delay)
        except (ProcessLookupError,subprocess.TimeoutExpired):
            pass


def json_file(path):
    try:return json.loads(Path(path).read_text())
    except (OSError,ValueError,TypeError):return None


def local_environment(settings):
    env=os.environ.copy()
    env["ROS_DOMAIN_ID"]=str(settings["ros_domain_id"])
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"]="LOCALHOST"
    env.pop("ROS_LOCALHOST_ONLY",None)
    env["GZ_PARTITION"]=f"scrobot_performance_lab_{os.getuid()}_{settings['ros_domain_id']}"
    env["GZ_IP"]="127.0.0.1"
    # Honor an explicit GPU/Mesa backend setting from the user's environment.
    return env


def gpu_metrics():
    """NVIDIA device-wide stats, explicitly NOT Gazebo per-process utilization."""
    try:
        out=subprocess.run(
            ["nvidia-smi",
             "--query-gpu=index,name,utilization.gpu,memory.used,memory.total,temperature.gpu",
             "--format=csv,noheader,nounits"],
            capture_output=True,text=True,timeout=2,check=False)
        if out.returncode:return None
        rows=[]
        for line in out.stdout.splitlines():
            parts=[v.strip() for v in line.split(",")]
            if len(parts)<6:continue
            try:
                rows.append({"index":int(parts[0]),"name":parts[1],
                             "util_percent":float(parts[2]),
                             "memory_used_mib":float(parts[3]),
                             "memory_total_mib":float(parts[4]),
                             "temperature_c":float(parts[5])})
            except ValueError:continue
        return {"devices":rows,"scope":"whole GPU, not only Gazebo",
                "source":"nvidia-smi"} if rows else None
    except (OSError,subprocess.TimeoutExpired):
        return None


def summary_of_samples(samples,settings):
    """Sustained sensor cadence, separate Nav2 lifecycle/map and timing evidence.

    A latched full map is evidence of a published costmap, NOT a 3-Hz stream.
    Warning counters come from Jazzy's WALL scheduler, so they are reported
    separately and must not be counted as simulated-time deadline misses.
    """
    usable=[x for x in samples if x.get("monitor",{}).get("rtf") is not None]
    def average(items):
        values=[float(v) for v in items if v is not None]
        return round(sum(values)/len(values),3) if values else None
    def delta_count(items,key):
        values=[x.get(key) for x in items if isinstance(x.get(key),int)]
        if len(values)<2 or values[-1]<values[0]:return None
        return values[-1]-values[0]

    rtfs=[x["monitor"]["rtf"] for x in usable]
    required=[
        "/camera/camera/color/image_raw",
        "/camera/camera/depth/image_raw",
        "/camera/camera/depth/points",
        "/camera/camera/depth/scan_raw",
        "/camera/camera/depth/scan",
        "/camera/camera/imu",
        "/imu/mag",
        "/diff_drive_controller/odom",
        "/odometry/filtered",
        "/joint_states",
    ]
    checks={}
    for topic in required:
        rows=[x["monitor"].get("topics",{}).get(topic,{}) for x in usable]
        sim=[r.get("sim_hz") for r in rows if r.get("sim_hz") is not None]
        wall=[r.get("wall_hz") for r in rows if r.get("wall_hz") is not None]
        expected=next((r.get("expected_sim_hz") for r in rows
                       if r.get("expected_sim_hz")),None)
        sim_mean=average(sim)
        wall_mean=average(wall)
        enough=len(sim)>=max(2,int(len(usable)*.6))
        good=bool(enough and expected and sim_mean is not None
                  and .9*expected<=sim_mean<=1.15*expected)
        check={
            "expected_sim_hz":expected,
            "avg_sim_hz":sim_mean,"avg_wall_hz":wall_mean,
            "coverage":len(sim),"sim_rate_pass":good,
            "header_interval_p95_ms":average(
                [r.get("header_interval_p95_ms") for r in rows]),
            "header_abs_jitter_p95_ms":average(
                [r.get("header_abs_jitter_p95_ms") for r in rows]),
            # Unique changes in monotonic counters across the sampled
            # measurement interval; do not sum overlapping five-second windows.
            "observed_large_header_gaps":delta_count(
                rows,"header_large_gap_total"),
            "observed_backward_stamps":delta_count(
                rows,"header_backward_total"),
            "observed_repeated_stamps":delta_count(
                rows,"header_duplicate_total"),
            "avg_source_unique_hz":average(
                [r.get("source_unique_hz") for r in rows]),
        }
        if settings["mode"]=="real_time":
            check["wall_rate_pass"]=bool(
                enough and expected and wall_mean is not None
                and wall_mean>=.9*expected)
        checks[topic]=check
    cadence=bool(usable) and all(x["sim_rate_pass"] for x in checks.values())

    nav2_samples=[x["monitor"].get("nav2",{}) for x in usable]
    lifecycle={}
    for name in ("planner_server","controller_server","global_costmap"):
        states=[x.get("lifecycle",{}).get(name,{}) for x in nav2_samples]
        seen=[v.get("state") for v in states]
        lifecycle[name]={
            "active_observed":"active" in seen,
            "last_state":next((v for v in reversed(seen) if v), "unavailable"),
            "services":list(dict.fromkeys(
                v.get("service") for v in states if v.get("service"))),
        }
    costmaps=[x.get("costmap",{}) for x in nav2_samples]
    full_observed=any(x.get("full_received",False) for x in costmaps)
    update_observed=any(x.get("update_received",False) for x in costmaps)
    nav2_required=settings["profile"]=="full_mission"
    nav2_pass=(
        lifecycle["planner_server"]["active_observed"]
        and lifecycle["controller_server"]["active_observed"]
        and full_observed
    ) if nav2_required else None
    nav2_coverage={
        "required":nav2_required,
        "verified":nav2_pass,
        "lifecycle":lifecycle,
        "full_map_observed":full_observed,
        "incremental_updates_observed":update_observed,
        "full_map_messages":max(
            (x.get("full_messages",0) for x in costmaps),default=0),
        "incremental_update_messages":max(
            (x.get("update_messages",0) for x in costmaps),default=0),
        "note":(
            "Active planner + controller lifecycle AND at least one full "
            "costmap are required to verify Nav2 coverage during mission. "
            "Global costmap lifecycle is reported when its service exists. "
            "Incremental updates are optional in static scenes; no fixed "
            "costmap visualization Hz is required."
        ),
    }
    warning_rows=[x.get("nav2_warnings",{}) for x in usable]
    warnings_new=delta_count(warning_rows,"warning_total")
    def command_quality(topic,target):
        rows=[x["monitor"].get("topics",{}).get(topic,{}) for x in usable]
        active=[r for r in rows if r.get("sim_hz") is not None]
        mean=average([r["sim_hz"] for r in active])
        header_p95=average([r.get("header_interval_p95_ms") for r in active])
        enough=len(active)>=max(3,int(len(usable)*.20))
        # Header p95 rejects a seemingly correct mean with frequent long gaps.
        good=bool(enough and mean is not None
                  and .90*target<=mean<=1.15*target
                  and header_p95 is not None
                  and header_p95<=1500.0/target)
        return {
            "expected_sim_hz":target,"avg_sim_hz":mean,
            "header_interval_p95_ms":header_p95,
            "active_windows":len(active),"verified":good,
            "note":"Topic output during active control, not an internal callback trace.",
        }

    cmd_quality=command_quality("/cmd_vel_nav",25.0)
    smoother_quality=command_quality("/cmd_vel_smoothed",25.0)
    odom_quality=checks.get("/diff_drive_controller/odom",{})
    odom_stable=bool(
        odom_quality.get("header_interval_p95_ms") is not None
        and odom_quality["header_interval_p95_ms"]<=15.0
        and odom_quality.get("observed_repeated_stamps")==0
        and (odom_quality.get("avg_source_unique_hz") or 0)>=95.0)
    overlay_active=installed_sim_clock_overlay()["active"]
    control_pass=bool(
        overlay_active and cmd_quality["verified"]
        and smoother_quality["verified"])
    timing={
        "nav2_wall_scheduler_warnings_in_window":warnings_new,
        "controller_scheduler_warnings_in_window":warnings_new,
        "scheduler_time_basis":"ros_sim" if overlay_active else "wall",
        "nav2_cmd_vel_avg_sim_hz":cmd_quality["avg_sim_hz"],
        "nav2_cmd_vel_available":cmd_quality["active_windows"]>0,
        "nav2_sim_clock_overlay_active":overlay_active,
        "nav2_controller":cmd_quality,
        "velocity_smoother":smoother_quality,
        "controller_and_smoother_cadence_pass":control_pass,
        "wheel_odom_header_jitter_pass":odom_stable,
        "wheel_odom_source_unique_hz":odom_quality.get("avg_source_unique_hz"),
        "wheel_odom_repeated_stamps":odom_quality.get("observed_repeated_stamps"),
        "joint_state_repeated_stamps":checks.get("/joint_states",{}).get(
            "observed_repeated_stamps"),
        "wheel_odom_publishers":next(
            (x["monitor"].get("publisher_graph",{}).get(
                "/diff_drive_controller/odom") for x in reversed(usable)
             if x["monitor"].get("publisher_graph",{}).get(
                "/diff_drive_controller/odom")),None),
        "joint_state_publishers":next(
            (x["monitor"].get("publisher_graph",{}).get(
                "/joint_states") for x in reversed(usable)
             if x["monitor"].get("publisher_graph",{}).get(
                "/joint_states")),None),
        "note":(
            "Controller/smoother require installed ROS-clock overlays AND "
            "25 simulated Hz output with reasonable header jitter. "
            "This still does not instrument internal Nav2 loop executions. "
            "Wheel odometry also requires distinct source timestamps, >95 Hz "
            "unique-stamp cadence, and <=15 ms p95 source intervals. "
            "Publisher counts distinguish duplicate sources from scheduler issues."
        ),
    }

    rtf_mean=average(rtfs)
    real_time=bool(rtf_mean is not None and rtf_mean>=.98 and cadence
                   and all(x.get("wall_rate_pass",False)
                           for x in checks.values()))
    if not cadence:overall="sensor_cadence_not_verified"
    elif nav2_required and not nav2_pass:overall="nav2_coverage_not_verified"
    elif not control_pass:overall="controller_or_smoother_timing_not_verified"
    elif not odom_stable:overall="wheel_odom_timestamp_jitter_not_verified"
    else:overall="observed_cadence_verified_not_deterministic_lockstep"
    return {
        "avg_rtf":rtf_mean,"minimum_rtf":min(rtfs) if rtfs else None,
        "sample_count":len(samples),"cadence_pass":cadence,
        "nav2_coverage":nav2_coverage,"nav2_coverage_pass":nav2_pass,
        "timing_quality":timing,
        "overall_evidence":overall,
        "coverage_note":(
            "Parked baseline: Nav2 may not be lifecycle-active."
            if not nav2_required else
            "Full mission: verify planner/controller active and one latched "
            "global costmap. Changed-cell updates are optional, not 3 Hz targets."
        ),
        "real_time_rate_pass":(
            real_time if settings["mode"]=="real_time" else None),
        "checks":checks,
        "note":(
            "Topic cadence alone is not closed-loop synchronization. "
            "This benchmark does not measure actual PID/ControllerServer "
            "callback executions, end-to-end perception age, or physical "
            "hardware deadlines. Deadline warning timebase follows the installed "
            "Nav2 controller scheduler (ROS clock when the overlay is active)."
        ),
    }

class Runtime:
    def __init__(self):
        self.lock=threading.RLock()
        self.settings=read_settings()
        self.env=local_environment(self.settings)
        self.stack=None
        self.observer=None
        self.rviz=None
        self.gui=GazeboGUI(ACTIVE/"gazebo_gui.log",env=self.env,cwd=ROOT)
        self.sampler=ProcessSampler(psutil,interval_s=1.5)
        self.thread_sampler=ThreadSampler(psutil,interval_s=2.0)
        self.nav2_warning_meter=ControllerWarningMeter()
        self.nav2_controller_hz=installed_controller_frequency()
        self.nav2_overlay=installed_sim_clock_overlay()
        self.monitor_file=ACTIVE/"monitor.json"
        self.log_file=ACTIVE/"stack.log"
        self.observer_log=ACTIVE/"observer.log"
        self.rviz_log=ACTIVE/"rviz.log"
        self.run_settings=None
        self.world_info=None
        self.inputs=None
        self.bench=None
        self.bench_thread=None
        self.bench_stop=threading.Event()
        self.audit={"status":"not_run"}
        self.audit_thread=None
        self.gpu=None
        self.gpu_at=0.
        self.last_result=None

    def _output_environment(self):
        self.env=local_environment(self.settings)
        self.gui.env=self.env

    def update_settings(self,body):
        with self.lock:
            value=validate(body.get("settings"))
            self.settings=value
            if not running(self.stack):
                self._output_environment()
            return {"settings":self.settings,
                    "restart_required":running(self.stack)}

    def save(self,body):
        self.update_settings(body)
        return {"file":save_settings(self.settings),"settings":self.settings,
                "restart_required":running(self.stack)}

    def start(self):
        with self.lock:
            if running(self.stack):
                raise RuntimeError("Stop the Performance Lab before restarting")
            if self.bench is not None:
                raise RuntimeError("Finish the benchmark first")
            if not shutil.which("ros2"):
                raise RuntimeError("ros2 not in PATH; source ROS Jazzy and workspace")
            if not shutil.which("gz"):
                raise RuntimeError("gz not in PATH; source Gazebo Harmonic")
            if not WORLD.is_file():raise RuntimeError("Production court world is missing")
            self.nav2_overlay=installed_sim_clock_overlay()
            if (self.settings["mode"]=="synchronized"
                    and not self.nav2_overlay["active"]):
                raise RuntimeError(
                    "Synchronized mode requires the ROS-time Nav2 controller "
                    "AND velocity smoother overlay. Run: "
                    "python3 tools/performance_lab/prepare_nav2_sim_clock.py --build; "
                    "source install/setup.bash; restart this lab server. "
                    "The installed upstream Jazzy packages still use wall clocks."
                )
            # Prevent late stale report from appearing as a new successful run.
            ACTIVE.mkdir(parents=True,exist_ok=True)
            self.monitor_file.unlink(missing_ok=True)
            self.world_info=prepare_world(
                WORLD,ACTIVE/"performance_world.sdf",self.settings["max_rtf"])
            self.inputs=calibration_snapshot()
            (ACTIVE/"inputs_snapshot.json").write_text(
                json.dumps(self.inputs,indent=2))
            self.run_settings=dict(self.settings)
            self._output_environment()
            # Prevent mixing topic samples with another /clock in this domain.
            try:
                graph=subprocess.run(
                    ["ros2","topic","list"],env=self.env,
                    capture_output=True,text=True,timeout=5,check=False)
                if (graph.returncode==0
                    and "/clock" in graph.stdout.splitlines()):
                    raise RuntimeError(
                        "ROS_DOMAIN_ID "+str(self.settings["ros_domain_id"])+
                        " already has /clock. Stop that simulator or change domains.")
            except (OSError,subprocess.TimeoutExpired):
                pass
            self.sampler.reset()
            self.thread_sampler.reset()
            self.nav2_warning_meter.reset()
            self.nav2_controller_hz=installed_controller_frequency()
            self.audit={"status":"not_run"}
            self.gpu_at=0.
            self.gpu=None
            self.last_result=None
            # This is the REAL production launch; only headless, speed cap
            # and automatic mission start are different launch arguments.
            cmd=[
                "ros2","launch","scrobot_bringup","sim.launch.py",
                "use_sim_time:=true","headless:=true",
                "world:="+self.world_info["path"],
                # Launch Nav2/collection even when the parked idle
                # mission manager is not commanded to move.
                "mission_enabled:=true",
                "mission_autostart:="+(
                    "true" if self.settings["profile"]=="full_mission" else "false"),
                "shuttle_count:="+str(self.settings["shuttle_count"]),
                "shuttle_seed:="+str(self.settings["shuttle_seed"]),
                "device:="+self.settings["yolo_device"],
            ]
            if self.settings["model_path"]:
                cmd.append("model_path:="+self.settings["model_path"])
            try:
                with self.log_file.open("wb") as log:
                    self.stack=subprocess.Popen(cmd,env=self.env,cwd=ROOT,
                        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                with self.observer_log.open("wb") as log:
                    self.observer=subprocess.Popen([
                        sys.executable,str(HERE/"monitor.py"),
                        "--output",str(self.monitor_file),
                        "--mode",self.settings["mode"]],
                        env=self.env,cwd=ROOT,stdout=log,
                        stderr=subprocess.STDOUT,start_new_session=True)
            except Exception:
                kill_group(self.observer)
                kill_group(self.stack)
                self.stack=self.observer=None
                raise
            return {"launched":True,"pid":self.stack.pid,
                    "mode":self.settings["mode"],
                    "profile":self.settings["profile"],
                    "note":"Wait for ROS nodes and camera topics to initialize."}

    def stop(self):
        with self.lock:
            self.bench_stop.set()
            self.gui.close()
            kill_group(self.rviz)
            kill_group(self.observer)
            kill_group(self.stack)
            self.stack=self.observer=self.rviz=None
            self.sampler.reset()
            self.thread_sampler.reset()
            return {"stopped":True}

    def observer_restart(self):
        """Repair clock monitoring without resetting the real Gazebo world."""
        with self.lock:
            if not running(self.stack):
                raise RuntimeError("Start the full simulation first")
            if self.bench is not None:
                raise RuntimeError("Finish or cancel the benchmark before restarting the observer")
            kill_group(self.observer)
            self.observer=None
            self.monitor_file.unlink(missing_ok=True)
            with self.observer_log.open("wb") as log:
                self.observer=subprocess.Popen(
                    [sys.executable,str(HERE/"monitor.py"),
                     "--output",str(self.monitor_file),
                     "--mode",(self.run_settings or self.settings)["mode"]],
                    env=self.env,cwd=ROOT,stdout=log,
                    stderr=subprocess.STDOUT,start_new_session=True)
            return {"restarted":True,"observer_pid":self.observer.pid,
                    "note":"Metrics will appear once /clock and topic samples arrive"}

    def gui_open(self):
        with self.lock:
            if not running(self.stack):raise RuntimeError("Start simulation first")
            return {"message":self.gui.open()}

    def gui_close(self):
        with self.lock:return {"message":self.gui.close()}

    def rviz_open(self):
        with self.lock:
            if not running(self.stack):raise RuntimeError("Start simulation first")
            if running(self.rviz):return {"running":True}
            with self.rviz_log.open("wb") as log:
                self.rviz=subprocess.Popen(
                    ["rviz2","-d",str(HERE/"config/performance.rviz"),
                     "--ros-args","-p","use_sim_time:=true"],
                    stdout=log,stderr=subprocess.STDOUT,env=self.env,
                    cwd=ROOT,start_new_session=True)
            return {"running":True}

    def rviz_close(self):
        with self.lock:
            kill_group(self.rviz)
            self.rviz=None
            return {"running":False}

    def _gpu(self):
        now=time.monotonic()
        if now-self.gpu_at>=4.:
            self.gpu=gpu_metrics()
            self.gpu_at=now
        return self.gpu

    def status(self):
        with self.lock:
            data=json_file(self.monitor_file) if running(self.stack) else None
            if data and time.time()-data.get("observer_wall_time",0)>4:
                data=None  # stale monitor or aborted process
            return {
                "stack_running":running(self.stack),
                "stack_exit_code":self.stack.poll() if self.stack else None,
                "observer_running":running(self.observer),
                "observer_exit_code":self.observer.poll() if self.observer else None,
                "rviz_running":running(self.rviz),
                "gazebo_gui":self.gui.status(),
                "settings":self.settings,
                "launch_settings":self.run_settings,
                "world_info":self.world_info,
                "monitor":data,
                "nav2_timing":{
                    "overlay":self.nav2_overlay,
                    "scheduler":nav2_scheduler_expectations(
                        data.get("rtf") if data else None,
                        self.nav2_controller_hz,
                        self.nav2_overlay["active"]),
                    "warnings":self.nav2_warning_meter.sample(self.log_file)
                        if running(self.stack) else
                        self.nav2_warning_meter.snapshot(),
                    "cmd_vel_nav":(data or {}).get("topics",{}).get("/cmd_vel_nav"),
                },
                "resources":self.sampler.sample(
                    self.stack.pid if running(self.stack) else None,
                    self.rviz.pid if running(self.rviz) else None),
                "gpu":self._gpu() if running(self.stack) else None,
                "threads":self.thread_sampler.sample(
                    self.stack.pid if running(self.stack) else None),
                "audit":self.audit,
                "benchmark":self._bench_status(),
                "last_result":self.last_result,
                "logs":{
                    "stack":log_tail(self.log_file,9000),
                    "observer":log_tail(self.observer_log,4000),
                    "gazebo":log_tail(self.gui.log_path,3000),
                    "rviz":log_tail(self.rviz_log,3000),
                },
                "calibration":self.inputs,
                "warnings":(
                    (["Clock observer stopped unexpectedly (exit " +
                      str(self.observer.poll() if self.observer else "not launched") +
                      "). See ROS clock observer log and use Restart observer."]
                     if running(self.stack) and not running(self.observer)
                     else [])
                    + (["No YOLO model configured: set model_path or "
                        "SCROBOT_YOLO_MODEL to run complete perception."]
                       if running(self.stack) and not (
                           (self.run_settings or {}).get("model_path") or
                           self.env.get("SCROBOT_YOLO_MODEL"))
                       else [])
                    + (["Idle profile: Nav2 and collection nodes are launched, "
                        "but Nav2 lifecycle/costmaps remain inactive until "
                        "mission initialization; /global_costmap is not expected."]
                       if running(self.stack) and
                          (self.run_settings or {}).get("profile")=="full_idle"
                       else [])
                ),
            }

    def _bench_status(self):
        if not self.bench:return None
        b=self.bench
        return {"phase":b["phase"],"elapsed_s":round(
                    time.monotonic()-b["start_wall"],1),
                "warmup_s":b["warmup_s"],"duration_s":b["duration_s"],
                "sample_count":len(b["samples"])}

    def benchmark_start(self):
        with self.lock:
            if not running(self.stack) or not running(self.observer):
                raise RuntimeError("Start full simulation and observer first")
            if self.bench:raise RuntimeError("A benchmark is already running")
            snapshot=json_file(self.monitor_file)
            if not snapshot or time.time()-snapshot.get("observer_wall_time",0)>4:
                raise RuntimeError("Wait for a fresh /clock monitor snapshot")
            self.bench_stop.clear()
            cfg=dict(self.run_settings)
            self.bench={
                "phase":"warmup",
                "warmup_s":cfg["warmup_s"],
                "duration_s":cfg["duration_s"],
                "start_wall":time.monotonic(),
                "samples":[],"settings":cfg,
                "world_info":self.world_info,
                "calibration":self.inputs,
            }
            self.bench_thread=threading.Thread(
                target=self._benchmark_worker,daemon=True)
            self.bench_thread.start()
            return {"started":True,"warmup_s":cfg["warmup_s"],
                    "duration_s":cfg["duration_s"]}

    def benchmark_cancel(self):
        with self.lock:
            if not self.bench:raise RuntimeError("No active benchmark")
            self.bench_stop.set()
            return {"cancel_requested":True}

    def _benchmark_worker(self):
        with self.lock:b=self.bench
        if b is None:return
        try:
            if self.bench_stop.wait(b["warmup_s"]):
                outcome="cancelled"
            else:
                outcome="complete"
                start=time.monotonic()
                with self.lock:
                    if self.bench is b:b["phase"]="measuring"
                while time.monotonic()-start<b["duration_s"]:
                    if self.bench_stop.wait(1.0):
                        outcome="cancelled"
                        break
                    if not running(self.stack):
                        outcome="simulation_stopped"
                        break
                    snapshot=json_file(self.monitor_file)
                    if snapshot and time.time()-snapshot.get("observer_wall_time",0)<=4:
                        with self.lock:
                            warning_sample=self.nav2_warning_meter.sample(
                                self.log_file)
                            b["samples"].append({
                                "wall":time.time(),"monitor":snapshot,
                                "nav2_warnings":warning_sample,
                                "cpu":self.sampler.sample(
                                    self.stack.pid if running(self.stack) else None),
                                "gpu":self._gpu(),
                            })
            result={
                "run_id":"performance_"+datetime.now(timezone.utc).strftime(
                    "%Y%m%dT%H%M%S")+"_"+uuid.uuid4().hex[:6],
                "created_at_utc":datetime.now(timezone.utc).isoformat(),
                "outcome":outcome,
                "profile":b["settings"]["profile"],
                "mode":b["settings"]["mode"],
                "settings":b["settings"],
                "world_info":b["world_info"],
                "calibration":b["calibration"],
                "summary":summary_of_samples(b["samples"],b["settings"]),
                "samples":b["samples"],
            }
            RESULTS.mkdir(parents=True,exist_ok=True)
            filename=RESULTS/(result["run_id"]+".json")
            tmp=filename.with_suffix(".tmp")
            tmp.write_text(json.dumps(result,indent=2)+"\n")
            tmp.replace(filename)
            result["file"]=str(filename)
            with self.lock:
                self.last_result={
                    "run_id":result["run_id"],"outcome":outcome,
                    "summary":result["summary"],"file":str(filename)}
        except Exception as exc:
            with self.lock:
                self.last_result={"outcome":"error","error":str(exc)}
        finally:
            with self.lock:
                if self.bench is b:self.bench=None

    def history(self):
        RESULTS.mkdir(parents=True,exist_ok=True)
        records=[]
        for file in sorted(RESULTS.glob("performance_*.json"),reverse=True):
            if not RUN_NAME.fullmatch(file.name) or file.is_symlink():continue
            record=json_file(file)
            if record:
                records.append({
                    "run_id":record.get("run_id"),"outcome":record.get("outcome"),
                    "created_at_utc":record.get("created_at_utc"),
                    "mode":record.get("mode"),"profile":record.get("profile"),
                    "summary":record.get("summary"),
                })
        return records[:50]

    def history_one(self,name):
        if not RUN_NAME.fullmatch(name):raise ValueError("Invalid run name")
        path=RESULTS/name
        if path.is_symlink() or not path.is_file():
            raise ValueError("Run does not exist")
        return json_file(path)

    def history_delete(self,body):
        with self.lock:
            if self.bench:raise RuntimeError("Stop benchmark before deletion")
            name=body.get("name")
            if not isinstance(name,str) or not RUN_NAME.fullmatch(name):
                raise ValueError("Invalid run name")
            file=RESULTS/name
            if file.is_symlink() or not file.is_file():raise ValueError("Unknown run")
            file.unlink()
            if self.last_result and self.last_result.get("file")==str(file):
                self.last_result=None
            return {"deleted":name}

    def audit_start(self):
        with self.lock:
            if not running(self.stack):raise RuntimeError("Start simulation first")
            if self.audit.get("status")=="running":
                return self.audit
            self.audit={"status":"running","nodes":[],"started_wall":time.time(),
                        "note":"Checking ROS use_sim_time parameters; wall timers in custom "
                               "code cannot be inferred from this check."}
            self.audit_thread=threading.Thread(
                target=self._audit_worker,daemon=True)
            self.audit_thread.start()
            return self.audit

    def _audit_worker(self):
        try:
            listed=subprocess.run(["ros2","node","list"],env=self.env,
                capture_output=True,text=True,timeout=12,check=False)
            if listed.returncode:raise RuntimeError(listed.stderr[-600:])
            nodes=sorted(set(line.strip() for line in listed.stdout.splitlines()
                             if line.strip().startswith("/")))
            output=[]
            for name in nodes[:48]:
                if not running(self.stack):break
                try:
                    response=subprocess.run(
                        ["ros2","param","get",name,"use_sim_time"],
                        env=self.env,capture_output=True,text=True,timeout=2,
                        check=False)
                    text=(response.stdout+" "+response.stderr).lower()
                    if response.returncode==0 and "true" in text:
                        state="sim_time"
                    elif response.returncode==0 and "false" in text:
                        state="wall_time"
                    else:
                        state="unknown"
                    output.append({"node":name,"status":state,
                                   "detail":text.strip()[-130:]})
                except (OSError,subprocess.TimeoutExpired) as exc:
                    output.append({"node":name,"status":"unknown",
                                   "detail":str(exc)[:130]})
                with self.lock:self.audit["nodes"]=list(output)
            with self.lock:
                self.audit.update({"status":"complete","nodes":output,
                    "total_found":len(nodes),
                    "truncated":len(nodes)>48,
                    "finished_wall":time.time()})
        except Exception as exc:
            with self.lock:self.audit.update({"status":"error","error":str(exc)})


class Handler(BaseHTTPRequestHandler):
    lab=None
    def reply(self,payload,status=200):
        blob=json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type","application/json; charset=utf-8")
        self.send_header("X-Content-Type-Options","nosniff")
        self.send_header("Cache-Control","no-store")
        self.send_header("Content-Length",str(len(blob)))
        self.end_headers()
        self.wfile.write(blob)

    def do_GET(self):
        path=urlparse(self.path).path
        try:
            if path=="/api/status":return self.reply(self.lab.status())
            if path=="/api/settings":return self.reply(self.lab.settings)
            if path=="/api/history":return self.reply(self.lab.history())
            if path.startswith("/api/history/"):
                name=unquote(path.rsplit("/",1)[-1])
                return self.reply(self.lab.history_one(name))
            file={"/":WEB/"index.html","/app.js":WEB/"app.js",
                  "/style.css":WEB/"style.css"}.get(path)
            if file is None:return self.reply({"error":"Not found"},404)
            blob=file.read_bytes()
            ext=file.suffix
            mime={".html":"text/html",".js":"application/javascript",
                  ".css":"text/css"}[ext]
            self.send_response(200)
            self.send_header("Content-Type",mime+"; charset=utf-8")
            self.send_header("Cache-Control","no-store")
            self.send_header("Content-Length",str(len(blob)))
            self.end_headers()
            self.wfile.write(blob)
        except (ValueError,RuntimeError) as exc:self.reply({"error":str(exc)},400)
        except Exception as exc:self.reply({"error":str(exc)},500)

    def do_POST(self):
        route=urlparse(self.path).path
        actions={
            "/api/settings":lambda body:self.lab.update_settings(body),
            "/api/save":self.lab.save,
            "/api/start":lambda body:self.lab.start(),
            "/api/stop":lambda body:self.lab.stop(),
            "/api/gazebo/open":lambda body:self.lab.gui_open(),
            "/api/gazebo/close":lambda body:self.lab.gui_close(),
            "/api/rviz/open":lambda body:self.lab.rviz_open(),
            "/api/rviz/close":lambda body:self.lab.rviz_close(),
            "/api/benchmark/start":lambda body:self.lab.benchmark_start(),
            "/api/benchmark/cancel":lambda body:self.lab.benchmark_cancel(),
            "/api/observer/restart":lambda body:self.lab.observer_restart(),
            "/api/history/delete":self.lab.history_delete,
            "/api/audit/start":lambda body:self.lab.audit_start(),
        }
        if route not in actions:return self.reply({"error":"Not found"},404)
        origin=self.headers.get("Origin","")
        if origin and urlparse(origin).hostname not in ("localhost","127.0.0.1"):
            return self.reply({"error":"Cross-origin blocked"},403)
        if self.headers.get("Content-Type","").split(";")[0].strip()!="application/json":
            return self.reply({"error":"JSON body required"},415)
        try:
            length=int(self.headers.get("Content-Length","0"))
            if length<0 or length>14000:raise ValueError("Invalid JSON body size")
            body=json.loads(self.rfile.read(length) or "{}")
            if not isinstance(body,dict):raise ValueError("Expected JSON object")
            self.reply(actions[route](body))
        except (ValueError,RuntimeError,FileNotFoundError) as exc:
            self.reply({"error":str(exc)},400)
        except Exception as exc:self.reply({"error":str(exc)},500)

    def log_message(self,*args):
        return


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--port",type=int,default=8774)
    p.add_argument("--no-browser",action="store_true")
    args=p.parse_args()
    Handler.lab=Runtime()
    server=ThreadingHTTPServer(("127.0.0.1",args.port),Handler)
    url=f"http://127.0.0.1:{args.port}"
    print("SCROBOT Performance Lab:",url,flush=True)
    if not args.no_browser:webbrowser.open_new_tab(url)
    try:server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:pass
    finally:
        Handler.lab.stop()
        server.server_close()


if __name__=="__main__":main()
