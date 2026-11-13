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
            self.respond(200, {"color": COLOR, "depth": DEPTH, "default": DEFAULT})
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
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self.respond(400, {"error": str(exc)})

    def log_message(self, *args):
        pass


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8769)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    if not shutil.which("gz") or not shutil.which("ros2"):
        raise SystemExit("Source ROS Jazzy + install/setup.bash and install Gazebo Harmonic first")
    lab = Lab()
    web = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    web.lab = lab
    url = f"http://127.0.0.1:{args.port}"
    print("SCROBOT Camera Lab: " + url, flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        web.serve_forever(poll_interval=.3)
    except KeyboardInterrupt:
        pass
    finally:
        lab.stop()
        web.server_close()


if __name__ == "__main__":
    main()
