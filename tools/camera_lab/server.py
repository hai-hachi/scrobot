#!/usr/bin/env python3
"""Standalone Camera Lab: Gazebo + RGB/depth ROS image bridge + CPU meters.

Nothing from the previous Perception Labs is imported or launched.
"""
from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse
from datetime import datetime, timezone
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
import webbrowser

from modes import COLOR, DEPTH, DEFAULT, validate, expected_mb_s, world
from tag_math import (TAG_DEFAULT, validate as validate_tag, mount_radius,
                      head_on_pose, sweep_values, tag_pose)
from tags_scene import (generate_tags, tag_world, detector_yaml,
                        camera_info_bridge_yaml)

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RESULTS = ROOT / "evaluation_results/camera_lab"
PUBLISHED = ROOT / "src/scrobot_simulation/config/camera_calibration.yaml"


def proc_tree(pid):
    pending, seen = [int(pid)], set()
    while pending:
        p = pending.pop()
        if p in seen:
            continue
        seen.add(p)
        try:
            pending += [int(x) for x in Path(
                f"/proc/{p}/task/{p}/children").read_text().split()]
        except (OSError, ValueError):
            continue
    return seen


def usage(pid):
    cpu, mem = 0.0, 0
    for child in proc_tree(pid):
        try:
            raw = Path(f"/proc/{child}/stat").read_text().rsplit(")", 1)[1].split()
            cpu += (int(raw[11]) + int(raw[12])) / os.sysconf("SC_CLK_TCK")
            mem += int(raw[21]) * os.sysconf("SC_PAGE_SIZE")
        except (OSError, ValueError, IndexError):
            continue
    return cpu, round(mem / 1e6, 1)


def stop_process(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=2)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=2)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=1)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass


class Lab:
    def __init__(self):
        RESULTS.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.settings = dict(DEFAULT)
        self.procs = {}
        self.logs = {}
        self.previous_cpu = {}
        self.previous_time = None
        self.phase = "idle"
        self.error = None
        self.mode = "camera"
        self.tag = dict(TAG_DEFAULT)
        self.tag_epoch = 0
        self.sweep_id = 0
        self.sweep_cancel = threading.Event()
        self.sweep = {"state": "idle", "kind": None, "rows": []}

    def environment(self):
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = "82"
        env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        env["GZ_PARTITION"] = f"scrobot_camera_lab_{os.getuid()}_82"
        env["GZ_IP"] = "127.0.0.1"
        # Generated plate textures, built by the canonical SCROBOT tag model
        # generator, resolve as model://court_apriltags/... in the lab only.
        extra = str(RESULTS / "tag_models")
        env["GZ_SIM_RESOURCE_PATH"] = (extra + os.pathsep +
            env.get("GZ_SIM_RESOURCE_PATH", ""))
        env.pop("ROS_LOCALHOST_ONLY", None)
        return env

    def launch(self, name, args):
        handle = (RESULTS / (name + ".log")).open("wb")
        try:
            p = subprocess.Popen(args, cwd=ROOT, env=self.environment(),
                                 stdout=handle, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        except Exception:
            handle.close()
            raise
        self.procs[name] = p
        self.logs[name] = handle

    def stop(self):
        with self.lock:
            self.sweep_cancel.set()
            self.sweep_id += 1
            for name in reversed(list(self.procs)):
                stop_process(self.procs.pop(name))
                self.logs.pop(name).close()
            (RESULTS / "meter.json").unlink(missing_ok=True)
            (RESULTS / "tag_status.json").unlink(missing_ok=True)
            (RESULTS / "tag_preview.jpg").unlink(missing_ok=True)
            self.previous_cpu.clear()
            self.previous_time = None
            self.phase = "stopped"
            self.sweep = {"state": "idle", "kind": None, "rows": []}
            return {"stopped": True}

    def start(self, data):
        cfg = validate(data)
        with self.lock:
            self.stop()
            self.settings = cfg
            self.mode = "camera"
            self.error = None
            try:
                path = world(cfg, RESULTS / "world.sdf")
                # Gazebo is headless; no GUI, controllers, tags or YOLO.
                self.launch("gazebo", ["gz", "sim", "-r", "-s", "-v", "3", path])
                topics = ["/camera_lab/color"]
                if cfg["depth_enabled"]:
                    topics.append("/camera_lab/depth")
                self.launch("image_bridge", [
                    "ros2", "run", "ros_gz_image", "image_bridge", *topics,
                    "--ros-args", "-p", "qos:=sensor_data"
                ])
                self.launch("monitor", [
                    sys.executable, str(HERE / "monitor.py"),
                    str(RESULTS / "meter.json"),
                    "true" if cfg["depth_enabled"] else "false"
                ])
                self.phase = "starting"
            except (OSError, ValueError) as exc:
                self.error = str(exc)
                self.stop()
                self.phase = "failed"
                raise
            return {"started": True, "settings": dict(cfg)}

    def write_tag_control(self):
        """Epoch change resets observer statistics and applies live RGB noise."""
        self.tag_epoch += 1
        file = RESULTS / "tag_control.json"
        temp = file.with_suffix(".tmp")
        temp.write_text(json.dumps({
            "epoch": self.tag_epoch, "tag": self.tag, "camera": self.settings,
            "changed_wall": time.time(),
        }))
        temp.replace(file)

    def start_tags(self, data):
        if not isinstance(data, dict) or set(data) != {"camera", "tag"}:
            raise ValueError("Specify camera and tag configurations")
        cfg = validate(data["camera"])
        tag = validate_tag(data["tag"])
        with self.lock:
            self.stop()
            self.settings = cfg
            self.tag = tag
            self.mode = "apriltags"
            self.error = None
            try:
                # Generate current genuine 16h5 tag textures and meshes. Never
                # change production model files or court_landmarks.yaml.
                model, params = generate_tags(tag, RESULTS)
                path = tag_world(cfg, tag, RESULTS / "world.sdf", model)
                detector = detector_yaml(
                    tag, RESULTS / "tag_detector.yaml")
                bridge_cfg = camera_info_bridge_yaml(
                    RESULTS / "camera_info_bridge.yaml")
                self.write_tag_control()
                self.launch("gazebo", [
                    "gz", "sim", "-r", "-s", "-v", "3", path])
                topics = ["/camera_lab/color"]
                if cfg["depth_enabled"]:
                    topics.append("/camera_lab/depth")
                self.launch("image_bridge", [
                    "ros2", "run", "ros_gz_image", "image_bridge", *topics,
                    "--ros-args", "-p", "qos:=sensor_data"])
                self.launch("info_bridge", [
                    "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                    "--ros-args", "-p", f"config_file:={bridge_cfg}"])
                self.launch("monitor", [
                    sys.executable, str(HERE / "monitor.py"),
                    str(RESULTS / "meter.json"),
                    "true" if cfg["depth_enabled"] else "false"])
                self.launch("tag_monitor", [
                    sys.executable, str(HERE / "tag_monitor.py"), str(RESULTS)])
                self.launch("detector", [
                    "ros2", "run", "apriltag_ros", "apriltag_node",
                    "--ros-args", "--params-file", str(detector),
                    "-r", "__node:=apriltag",
                    "-r", "image_rect:=/camera_lab/tag/image_rect",
                    "-r", "camera_info:=/camera_lab/tag/camera_info",
                    "-r", "detections:=/camera_lab/detections"])
                self.phase = "starting"
            except (OSError, ValueError, subprocess.CalledProcessError,
                    subprocess.TimeoutExpired) as exc:
                self.error = str(exc)
                self.stop()
                self.phase = "failed"
                raise
            return {"started": True, "mode": "apriltags",
                    "tag": self.tag, "tag_mount_radius": mount_radius(tag["size"]),
                    "world": path}

    def gz_set_robot_pose(self, tag):
        p = self.procs.get("gazebo")
        if self.mode != "apriltags" or p is None or p.poll() is not None:
            raise ValueError("Start the AprilTags court simulation first")
        yaw = math.radians(tag["robot_yaw"])
        request = (
            'name: "camera_rig" '
            f'position: {{x: {tag["robot_x"]:.7f} y: {tag["robot_y"]:.7f} z: 0.0}} '
            f'orientation: {{x: 0 y: 0 z: {math.sin(yaw/2):.9f} '
            f'w: {math.cos(yaw/2):.9f}}}'
        )
        result = subprocess.run(
            ["gz", "service", "-s", "/world/camera_lab/set_pose",
             "--reqtype", "gz.msgs.Pose",
             "--reptype", "gz.msgs.Boolean", "--timeout", "5000",
             "--req", request],
            env=self.environment(), text=True, capture_output=True, timeout=8,
        )
        reply = result.stdout.lower()
        if result.returncode or "data: true" not in reply:
            raise RuntimeError("Gazebo did not accept camera rig pose: " +
                               (result.stdout + result.stderr)[-400:])

    def place_robot(self, data):
        if not isinstance(data, dict) or set(data) != {
            "robot_x", "robot_y", "robot_yaw", "tag_id"
        }:
            raise ValueError("Pose requires robot_x, robot_y, robot_yaw, tag_id")
        with self.lock:
            new = validate_tag({**self.tag, **data})
            self.gz_set_robot_pose(new)
            self.tag = new
            self.write_tag_control()
            return {"tag": dict(self.tag), "epoch": self.tag_epoch}

    def set_noise(self, data):
        if not isinstance(data, dict) or set(data) != {"noise_sigma"}:
            raise ValueError("Use noise_sigma in pixel levels")
        with self.lock:
            if self.mode != "apriltags":
                raise ValueError("Start AprilTags Lab before adjusting noise")
            if self.sweep.get("state") == "running":
                raise ValueError("Do not change RGB noise during an accuracy sweep")
            self.tag = validate_tag({**self.tag, **data})
            self.write_tag_control()
            return {"tag": dict(self.tag), "epoch": self.tag_epoch}

    def begin_sweep(self, data):
        if not isinstance(data, dict) or set(data) != {
            "kind", "tag_id", "distance_min", "distance_max",
            "distance_step", "fixed_distance", "angle_min",
            "angle_max", "angle_step", "dwell_s",
        }:
            raise ValueError("Incomplete distance or heading sweep settings")
        kind = data["kind"]
        if kind not in ("distance", "heading"):
            raise ValueError("Sweep kind must be distance or heading")
        tag_id = data["tag_id"]
        if type(tag_id) is not int or tag_id not in (0,1,2,3):
            raise ValueError("Select tag ID 0, 1, 2 or 3")
        if kind == "distance":
            xs = sweep_values(data["distance_min"], data["distance_max"],
                              data["distance_step"])
            if not .4 <= xs[0] < xs[-1] <= 8:
                raise ValueError("Distance sweep must stay within 0.4..8.0 m")
        else:
            xs = sweep_values(data["angle_min"],data["angle_max"],
                              data["angle_step"])
            if not -90 <= xs[0] < xs[-1] <= 90:
                raise ValueError("Angle offset sweep must stay within ±90°")
        fixed = data["fixed_distance"]
        dwell = data["dwell_s"]
        if not isinstance(fixed,(int,float)) or not .4 <= fixed <= 8:
            raise ValueError("Fixed heading-sweep distance must be 0.4..8 m")
        if not isinstance(dwell,(int,float)) or not 1.0 <= dwell <= 6.0:
            raise ValueError("Sampling time must be 1..6 seconds per pose")
        with self.lock:
            if self.mode != "apriltags" or self.phase != "streaming":
                raise ValueError("Wait for live AprilTag camera streaming first")
            if self.sweep.get("state") == "running":
                raise ValueError("A sweep is already running")
            if not self.tag_status().get("camera_info_valid"):
                raise ValueError("Wait for CameraInfo before sweeping; detector needs intrinsics")
            for value in xs:
                head_on_pose(tag_id,
                             value if kind=="distance" else fixed,
                             self.tag,
                             0 if kind=="distance" else value)
            self.sweep_cancel=threading.Event()
            self.sweep_id += 1
            session=self.sweep_id
            self.sweep={
                "state":"running","kind":kind,"tag_id":tag_id,
                "rows":[],"total":len(xs),"current":0,
                "error":None,"file":None,
                "unit":"m" if kind=="distance" else "deg",
            }
            worker=threading.Thread(
                target=self._run_sweep,
                args=(session,self.sweep_cancel,kind,tag_id,xs,float(fixed),
                      float(dwell)),daemon=True)
            worker.start()
            return {"started":True,"points":len(xs),"kind":kind}

    def cancel_sweep(self):
        with self.lock:
            self.sweep_cancel.set()
            if self.sweep.get("state") == "running":
                self.sweep["state"]="cancelled"
            return {"cancelled":True}

    def _run_sweep(self, session, cancelled, kind, tag_id, values, fixed, dwell):
        try:
            for i, value in enumerate(values):
                if cancelled.is_set():
                    return
                pose=head_on_pose(tag_id,
                                  value if kind=="distance" else fixed,
                                  self.tag,
                                  0 if kind=="distance" else value)
                self.place_robot({**pose, "tag_id":tag_id})
                with self.lock:
                    if session != self.sweep_id:
                        return
                    epoch=self.tag_epoch
                    self.sweep["current"]=i+1
                # Fresh measurements only after the Gazebo pose update and
                # observer control epoch. This also flushes stale image queues.
                if cancelled.wait(.45):
                    return
                if cancelled.wait(dwell):
                    return
                with self.lock:
                    if session != self.sweep_id:
                        return
                    measured=self.tag_status()
                    if measured.get("epoch") != epoch:
                        measured={}
                    enough_frames=measured.get("detector_frames",0)>=5
                    enough_poses=measured.get("pose_frames",0)>=3
                    row={
                        "quality":"ok" if enough_frames else "insufficient_detector_frames",
                        "value":value,
                        "pose":pose,
                        "tag_id":tag_id,
                        "detector_frames":measured.get("detector_frames",0),
                        "detected_frames":measured.get("detected_frames",0),
                        "pose_frames":measured.get("pose_frames",0),
                        "detection_pct":measured.get("detection_pct")
                            if enough_frames else None,
                        "position_rmse_m":measured.get("position_rmse_m")
                            if enough_poses else None,
                        "range_rmse_m":measured.get("range_rmse_m")
                            if enough_poses else None,
                        "bearing_rmse_deg":measured.get("bearing_rmse_deg")
                            if enough_poses else None,
                        "normal_axis_rmse_deg":
                            measured.get("normal_axis_rmse_deg")
                            if enough_poses else None,
                        "detector_fps":measured.get("detector_fps"),
                        "image_to_detection_p95_ms":
                            measured.get("image_to_detection_p95_ms"),
                    }
                    self.sweep["rows"].append(row)
            with self.lock:
                if session != self.sweep_id or cancelled.is_set():
                    return
                report={
                    "created_utc":datetime.now(timezone.utc).isoformat(),
                    "kind":kind,"tag_id":tag_id,"camera":dict(self.settings),
                    "tag":dict(self.tag),"dwell_s":dwell,
                    "rows":self.sweep["rows"],
                    "accuracy_definition":
                        "Position RMSE is observed camera-optical TF tag translation "
                        "vs known Gazebo static geometry. Undetected samples are missing, "
                        "not zero-error. Normal axis ignores sign; no AprilTag TF "
                        "pose is invented for missed detections.",
                }
                folder=RESULTS/"tag_sweeps"
                folder.mkdir(parents=True,exist_ok=True)
                filename=folder/(
                    f'apriltag_{kind}_'+datetime.now(timezone.utc).strftime(
                        "%Y%m%dT%H%M%S%fZ")+'.json')
                filename.write_text(json.dumps(report,indent=2)+"\n")
                self.sweep["file"]=str(filename)
                self.sweep["state"]="complete"
        except (OSError,ValueError,RuntimeError,subprocess.TimeoutExpired) as exc:
            with self.lock:
                if session==self.sweep_id:
                    self.sweep["state"]="failed"
                    self.sweep["error"]=str(exc)

    def tag_status(self):
        path = RESULTS / "tag_status.json"
        try:
            data = json.loads(path.read_text())
            if time.time()-data.get("updated_wall",0)>3:
                return {}
            return data
        except (OSError, ValueError, TypeError):
            return {}

    def monitor(self):
        path = RESULTS / "meter.json"
        try:
            data = json.loads(path.read_text())
            if time.time() - data["updated_wall"] > 3:
                return {}
            return data
        except (OSError, ValueError, KeyError):
            return {}

    def resources(self):
        now = time.monotonic()
        dt = now - self.previous_time if self.previous_time else 0
        current, out = {}, {}
        for name, proc in self.procs.items():
            if proc.poll() is not None:
                continue
            total, mb = usage(proc.pid)
            current[name] = total
            prior = self.previous_cpu.get(name)
            cpu = max(0, (total - prior) / dt * 100) if prior is not None and dt > .2 else None
            out[name] = {
                "cpu_pct": round(cpu, 1) if cpu is not None else None,
                "memory_mb": mb, "pid": proc.pid
            }
        self.previous_cpu = current
        self.previous_time = now
        return out

    def status(self):
        with self.lock:
            obs = self.monitor()
            running = {name: proc.poll() is None for name, proc in self.procs.items()}
            if running and not running.get("gazebo"):
                self.phase = "failed"
                self.error = self.error or "Gazebo exited. Check gazebo.log."
            elif self.phase in ("starting", "streaming") and obs:
                rgb = obs.get("streams", {}).get("color") or {}
                if rgb.get("last_age_s") is not None and rgb["last_age_s"] < 2:
                    self.phase = "streaming"
            if self.mode=="apriltags":
                dead=[name for name in ("image_bridge","info_bridge",
                                       "tag_monitor","detector")
                      if name in running and not running[name]]
                if dead:
                    self.phase="failed"
                    self.error="AprilTag pipeline process exited: "+", ".join(dead)+". Inspect logs below."
            logs = {}
            for name in ("gazebo", "image_bridge", "info_bridge",
                         "monitor", "tag_monitor", "detector"):
                path = RESULTS / (name + ".log")
                if path.is_file():
                    with path.open("rb") as f:
                        f.seek(max(0, path.stat().st_size - 1300))
                        logs[name] = f.read().decode("utf-8", "replace")
            return {
                "phase": self.phase,
                "error": self.error,
                "mode": self.mode,
                "tag": dict(self.tag),
                "tag_mount_radius": mount_radius(self.tag["size"]),
                "tags": {str(i): tag_pose(i, self.tag) for i in range(4)},
                "tag_observation": self.tag_status() if self.mode=="apriltags" else {},
                "sweep": dict(self.sweep),
                "settings": self.settings,
                "expected": expected_mb_s(self.settings),
                "monitor": obs,
                "resources": self.resources(),
                "processes": running,
                "logs": logs,
            }

    def check_measurement(self):
        if self.phase != "streaming":
            raise ValueError("Start Camera Lab and receive image messages first")
        streams = self.monitor().get("streams") or {}
        for name, res, fps in (
            ("color", self.settings["color"], self.settings["color_fps"]),
            ("depth", self.settings["depth"], self.settings["depth_fps"]),
        ):
            if name == "depth" and not self.settings["depth_enabled"]:
                continue
            obs = streams.get(name) or {}
            if obs.get("frames", 0) < 30:
                raise ValueError("Receive at least 30 " + name + " frames before saving")
            if obs.get("resolution") != [int(x) for x in res.split("x")]:
                raise ValueError(name + " measured resolution does not match settings")
        return streams

    def save(self):
        with self.lock:
            self.check_measurement()
            report = self.status()
            path = RESULTS / ("benchmark_" +
                datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".json")
            path.write_text(json.dumps(report, indent=2) + "\n")
            return {"file": str(path)}

    def publish(self):
        """Deliberate YAML publication, without changing hardware configuration."""
        with self.lock:
            if self.mode != "camera":
                raise ValueError("Publish camera settings from Camera Lab, not AprilTags")
            streams = self.check_measurement()
            for name, fps in (("color", self.settings["color_fps"]),
                              ("depth", self.settings["depth_fps"])):
                if name == "depth" and not self.settings["depth_enabled"]:
                    continue
                obs = streams[name]
                if not obs.get("wall_fps") or obs["wall_fps"] < fps * .8:
                    raise ValueError(name + " wall FPS is less than 80% of target; choose a lighter profile")
            if not PUBLISHED.is_file():
                raise FileNotFoundError("Missing main simulation camera YAML")
            cfg = self.settings
            color_width, color_height = map(int, cfg["color"].split("x"))
            depth_width, depth_height = map(int, cfg["depth"].split("x"))
            new = (
                "# Camera Lab calibrated Gazebo camera; not the physical D435i.\n"
                "schema_version: 1\n"
                f"color:\n  width: {color_width}\n  height: {color_height}\n  fps: {cfg['color_fps']}\n"
                f"depth:\n  width: {depth_width}\n  height: {depth_height}\n"
                f"  fps: {cfg['depth_fps']}\n"
                f"  enabled: {str(cfg['depth_enabled']).lower()}\n"
            )
            old = PUBLISHED.read_text()
            if old == new:
                return {"changed": False, "path": str(PUBLISHED)}
            backup = RESULTS / "backups"
            backup.mkdir(parents=True, exist_ok=True)
            saved = backup / (
                datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".yaml")
            saved.write_text(old)
            temp = PUBLISHED.with_suffix(".tmp")
            temp.write_text(new)
            os.replace(temp, PUBLISHED)
            return {"changed": True, "path": str(PUBLISHED), "backup": str(saved)}


class Handler(BaseHTTPRequestHandler):
    def respond(self, status, payload):
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        route = urlparse(self.path).path
        if route == "/":
            payload = (HERE / "index.html").read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
        elif route == "/api/status":
            self.respond(200, self.server.lab.status())
        elif route == "/api/modes":
            self.respond(200, {"color": COLOR, "depth": DEPTH,
                               "default": DEFAULT, "tag_default": TAG_DEFAULT})
        elif route == "/api/tag/image":
            file = RESULTS / "tag_preview.jpg"
            if not file.is_file() or time.time()-file.stat().st_mtime>4:
                return self.respond(503, {"error": "No fresh AprilTag RGB overlay"})
            payload = file.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError,ConnectionResetError):
                pass
        elif route == "/api/tag/sweep/result":
            with self.server.lab.lock:
                file = self.server.lab.sweep.get("file")
                if not file or not Path(file).is_file():
                    return self.respond(404, {"error": "No completed sweep"})
                self.respond(200, json.loads(Path(file).read_text()))
        else:
            self.respond(404, {"error": "Unknown endpoint"})

    def do_POST(self):
        route = urlparse(self.path).path
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 1 <= size <= 4096:
                raise ValueError("Invalid request size")
            data = json.loads(self.rfile.read(size))
            lab = self.server.lab
            if route == "/api/start":
                result = lab.start(data)
            elif route == "/api/tag/start":
                result = lab.start_tags(data)
            elif route == "/api/tag/pose":
                result = lab.place_robot(data)
            elif route == "/api/tag/noise":
                result = lab.set_noise(data)
            elif route == "/api/tag/sweep":
                result = lab.begin_sweep(data)
            elif route == "/api/tag/cancel":
                result = lab.cancel_sweep()
            elif route == "/api/stop":
                result = lab.stop()
            elif route == "/api/save":
                result = lab.save()
            elif route == "/api/publish":
                if data != {"confirm": True}:
                    raise ValueError("Confirm publishing camera YAML first")
                result = lab.publish()
            else:
                return self.respond(404, {"error": "Unknown action"})
            self.respond(200, result)
        except (OSError, ValueError, TypeError, KeyError, RuntimeError,
                subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
            self.respond(400, {"error": str(exc)})

    def log_message(self, *args):
        pass


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8769)
    parser.add_argument(
        "--open-browser", action="store_true",
        help="Optionally open the dashboard in the system browser; off by default")
    # Retain backwards compatibility with older lab start commands.
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    if not shutil.which("gz") or not shutil.which("ros2"):
        raise SystemExit("Source ROS Jazzy + install/setup.bash and install Gazebo Harmonic first")
    lab = Lab()
    web = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    web.lab = lab
    url = f"http://127.0.0.1:{args.port}"
    print("SCROBOT Camera Lab: " + url, flush=True)
    # Avoid browser-side Crashpad / Opera GX warnings on routine lab startup.
    # Browser launch is optional: the HTTP server works independently.
    if args.open_browser and not args.no_browser:
        try:
            webbrowser.open(url)
        except Exception as exc:
            print(f"Browser could not open automatically: {exc}\nOpen {url} manually.",
                  file=sys.stderr)
    try:
        web.serve_forever(poll_interval=.3)
    except KeyboardInterrupt:
        pass
    finally:
        lab.stop()
        web.server_close()


if __name__ == "__main__":
    main()
