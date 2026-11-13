#!/usr/bin/env python3
"""Independent SCROBOT YOLO Lab: shared camera settings, calibrated court, shuttles.

Initial stage implements actual Gazebo inspection, raw RGB/depth monitoring,
camera/Yolo YAML persistence and GUI. Dataset capture, automated labels,
training and evaluation are deliberately NOT wired until their workflow is
reviewed; the dashboard identifies this boundary.
"""
from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import urlsplit, parse_qs

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tools/camera_lab"))
from lab_configs import read as read_lab, save as save_lab, path_for
from modes import (COLOR, DEPTH, DEFAULT as CAMERA_DEFAULT,
                   validate as validate_camera, expected_mb_s)
from lighting_source import calibration as lighting_calibration
from yolo_settings import DEFAULT as YOLO_DEFAULT, validate as validate_yolo
from world import build
from workflow import Workflow
from log_text import tail as clean_tail

OUTPUT = ROOT / "evaluation_results/yolo_lab"
CAMERA_METER = ROOT / "tools/camera_lab/monitor.py"


def terminate(proc):
    if proc is None or proc.poll() is not None:
        return
    for sig, seconds in ((signal.SIGINT, 2), (signal.SIGTERM, 1)):
        try:
            os.killpg(proc.pid, sig)
            proc.wait(timeout=seconds)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            continue
    try:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait(timeout=1)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass


def tail(path, max_bytes=3500):
    if not Path(path).is_file():
        return ""
    with Path(path).open("rb") as stream:
        stream.seek(0, 2)
        stream.seek(max(0, stream.tell() - max_bytes))
        return stream.read().decode("utf-8", errors="replace")


class Lab:
    def __init__(self):
        OUTPUT.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.camera = validate_camera(read_lab("camera") or CAMERA_DEFAULT)
        self.yolo = validate_yolo(read_lab("yolo") or YOLO_DEFAULT)
        self.phase = "idle"
        self.error = None
        self.processes = {}
        self.handles = {}
        self.manifest = None
        self.started_with = None
        self.workflow = Workflow(self)

    def environment(self):
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = "83"
        env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        env["GZ_PARTITION"] = f"scrobot_yolo_lab_{os.getuid()}_83"
        env["GZ_IP"] = "127.0.0.1"
        env.pop("ROS_LOCALHOST_ONLY", None)
        additional = [
            str(ROOT / "src/scrobot_simulation/models"),
            str(HERE / "models"),
            str(ROOT / "src/scrobot_debug/models"),  # compatibility with old debug tools
            str(ROOT / "install/scrobot_simulation/share/scrobot_simulation/models"),
        ]
        existing = env.get("GZ_SIM_RESOURCE_PATH")
        if existing:
            additional.append(existing)
        env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(additional)
        return env

    def launch(self, name, cmd):
        if name in self.processes:
            raise RuntimeError("Process already launched: " + name)
        handle = (OUTPUT / f"{name}.log").open("ab")
        try:
            proc = subprocess.Popen(
                cmd, stdout=handle, stderr=subprocess.STDOUT,
                cwd=str(ROOT), env=self.environment(), start_new_session=True)
        except Exception:
            handle.close()
            raise
        self.processes[name] = proc
        self.handles[name] = handle
        return proc

    def _terminate(self, name):
        proc = self.processes.pop(name, None)
        terminate(proc)
        handle = self.handles.pop(name, None)
        if handle:
            handle.close()

    def stop(self):
        with self.lock:
            for name in reversed(list(self.processes)):
                self._terminate(name)
            for name in ("meter.json", "preview.jpg", "preview.json"):
                (OUTPUT / name).unlink(missing_ok=True)
            self.phase = "stopped"
            self.error = None
            return {"stopped": True}

    def save_camera(self, body):
        with self.lock:
            config = validate_camera(body)
            path = save_lab("camera", config, validator=validate_camera)
            self.camera = config
            return {"saved": str(path), "camera": config}

    def save_yolo(self, body):
        with self.lock:
            config = validate_yolo(body)
            path = save_lab("yolo", config, validator=validate_yolo)
            self.yolo = config
            return {"saved": str(path), "yolo": config}

    def start(self):
        """Use two SAVED YAMLs, never transient UI controls."""
        with self.lock:
            selected_camera = read_lab("camera")
            selected_yolo = read_lab("yolo")
            if selected_camera is None:
                raise ValueError(
                    "First choose settings in the Camera tab and click Save Camera Lab YAML")
            if selected_yolo is None:
                raise ValueError(
                    "First click Save YOLO Lab YAML to select shuttle positions/settings")
            camera = validate_camera(selected_camera)
            yolo = validate_yolo(selected_yolo)
            self.stop()
            self.camera = camera
            self.yolo = yolo
            self.phase = "generating"
            try:
                lighting = lighting_calibration()
                path, manifest = build(
                    camera, yolo, OUTPUT / "world.sdf", lighting=lighting)
                self.manifest = manifest
                self.started_with = {
                    "camera": camera, "yolo": yolo,
                    "lighting": lighting,
                }
                self.launch("gazebo", [
                    "gz", "sim", "-r", "-s", "-v", "3", path])
                topics = ["/camera_lab/color"]
                if camera["depth_enabled"]:
                    topics.append("/camera_lab/depth")
                self.launch("image_bridge", [
                    "ros2", "run", "ros_gz_image", "image_bridge", *topics,
                    "--ros-args", "-p", "qos:=sensor_data"])
                self.launch("meter", [
                    sys.executable, str(CAMERA_METER),
                    str(OUTPUT / "meter.json"),
                    "true" if camera["depth_enabled"] else "false"])
                self.launch("preview", [
                    sys.executable, str(HERE / "preview.py"),
                    str(OUTPUT / "preview.jpg")])
                self.phase = "starting"
            except Exception as exc:
                self.stop()
                self.phase = "failed"
                self.error = str(exc)
                raise
            return {
                "started": True, "world": path,
                "camera": camera, "yolo": yolo,
                "source_files": {
                    "camera": str(path_for("camera")),
                    "yolo": str(path_for("yolo")),
                    "lighting": str(path_for("lighting")),
                },
                "shuttles": len(manifest["shuttles"]),
            }

    def open_gui(self):
        with self.lock:
            gazebo = self.processes.get("gazebo")
            if not gazebo or gazebo.poll() is not None:
                raise ValueError("Start YOLO Lab scene before opening Gazebo UI")
            client = self.processes.get("gui")
            if client is not None and client.poll() is None:
                return {"open": True, "already_open": True}
            if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
                raise ValueError("No graphical desktop: DISPLAY/WAYLAND_DISPLAY unavailable")
            self._terminate("gui")
            self.launch("gui", ["gz", "sim", "-g", "-v", "3"])
            return {"open": True, "already_open": False}

    def close_gui(self):
        with self.lock:
            self._terminate("gui")
            return {"open": False}

    @staticmethod
    def recent_json(path):
        try:
            doc = json.loads(Path(path).read_text())
            if time.time() - doc.get("updated_wall", 0) < 4:
                return doc
        except (OSError, ValueError, TypeError):
            pass
        return {}

    def status(self):
        with self.lock:
            camera_saved = read_lab("camera")
            yolo_saved = read_lab("yolo")
            lighting = lighting_calibration()
            raw = self.recent_json(OUTPUT / "meter.json")
            preview = self.recent_json(OUTPUT / "preview.json")
            gazebo = self.processes.get("gazebo")
            if gazebo is not None and gazebo.poll() is not None:
                self.phase = "failed"
                self.error = self.error or "Gazebo server exited; see gazebo.log"
            elif (self.phase in ("starting", "streaming")
                  and (raw.get("streams") or {}).get("color", {}).get("frames", 0) > 0):
                self.phase = "streaming"
            elif self.phase == "streaming" and not raw:
                self.phase = "starting"
            procs = {name: {
                "pid": proc.pid, "alive": proc.poll() is None,
                "exit_code": proc.poll(),
            } for name, proc in self.processes.items()}
            return {
                "phase": self.phase, "error": self.error,
                "camera": self.camera, "yolo": self.yolo,
                "saved_camera": camera_saved or None,
                "saved_yolo": yolo_saved or None,
                "saved_paths": {
                    "camera": str(path_for("camera")),
                    "yolo": str(path_for("yolo")),
                    "lighting": str(path_for("lighting")),
                },
                "lighting": {
                    "lights": lighting["lights"]["rows"] * lighting["lights"]["cols"],
                    "shadow_count": lighting["lights"]["shadow_count"],
                    "venue": lighting["venue"],
                },
                "scene_matches_saved": self.started_with is not None and
                    self.started_with == {
                        "camera": validate_camera(camera_saved) if camera_saved else None,
                        "yolo": validate_yolo(yolo_saved) if yolo_saved else None,
                        "lighting": lighting,
                    },
                "manifest": self.manifest,
                "meter": raw, "preview": preview,
                "expected_mb_s": expected_mb_s(self.camera),
                "processes": procs,
                "logs": {n: clean_tail(OUTPUT / f"{n}.log") for n in
                         ("gazebo", "image_bridge", "meter", "preview", "gui")},
                "display_available": bool(os.environ.get("DISPLAY")
                                          or os.environ.get("WAYLAND_DISPLAY")),
                "workflow": self.workflow.progress(),
            }


class Handler(BaseHTTPRequestHandler):
    lab: Lab

    def respond(self, code, body, content_type="application/json"):
        payload = (json.dumps(body).encode("utf-8")
                   if content_type == "application/json" else body)
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def request_body(self):
        size = int(self.headers.get("Content-Length", "0"))
        if size < 1 or size > 16384:
            raise ValueError("Invalid JSON payload size")
        data = json.loads(self.rfile.read(size))
        if not isinstance(data, dict):
            raise ValueError("Expected a JSON object")
        return data

    def do_GET(self):
        route = urlsplit(self.path).path
        try:
            if route == "/":
                return self.respond(
                    200, (HERE / "index.html").read_bytes(),
                    "text/html; charset=utf-8")
            if route == "/api/options":
                return self.respond(200, {
                    "color": COLOR, "depth": DEPTH,
                    "camera_default": CAMERA_DEFAULT,
                    "yolo_default": YOLO_DEFAULT,
                    "status": self.lab.status()})
            if route == "/api/status":
                return self.respond(200, self.lab.status())
            if route == "/api/preview":
                path = OUTPUT / "preview.jpg"
                if path.is_file() and self.lab.recent_json(OUTPUT / "preview.json"):
                    return self.respond(200, path.read_bytes(), "image/jpeg")
                return self.respond(404, {"error": "Waiting for fresh RGB image"})
            if route == "/api/live-frame":
                path = OUTPUT / "live/live.jpg"
                if path.is_file():
                    return self.respond(200, path.read_bytes(), "image/jpeg")
                return self.respond(404, {"error": "Live YOLO image not available"})
            if route == "/api/label-preview":
                name = parse_qs(urlsplit(self.path).query).get("name", [""])[0]
                info = self.lab.workflow.progress()
                if not isinstance(name, str) or name not in info["previews"]:
                    raise ValueError("Select a generated label preview from the inspector")
                dataset = self.lab.workflow.current_dataset()
                source = dataset / "preview" / name
                if not source.is_file():
                    raise ValueError("Requested label preview file does not exist")
                return self.respond(200, source.read_bytes(), "image/jpeg")
            return self.respond(404, {"error": "Unknown route"})
        except (ValueError, OSError, RuntimeError) as exc:
            return self.respond(400, {"error": str(exc)})

    def do_POST(self):
        route = urlsplit(self.path).path
        try:
            body = self.request_body()
            if route == "/api/config/camera/save":
                result = self.lab.save_camera(body)
            elif route == "/api/config/yolo/save":
                result = self.lab.save_yolo(body)
            elif route == "/api/scene/start":
                if body:
                    raise ValueError("Start uses saved lab YAML, not request values")
                result = self.lab.start()
            elif route == "/api/scene/stop":
                if body:
                    raise ValueError("Stop expects empty body")
                result = self.lab.stop()
            elif route == "/api/gui/open":
                if body:
                    raise ValueError("Open GUI expects empty body")
                result = self.lab.open_gui()
            elif route == "/api/gui/close":
                if body:
                    raise ValueError("Close GUI expects empty body")
                result = self.lab.close_gui()
            elif route == "/api/dataset/start":
                if body:
                    raise ValueError("Dataset capture uses saved Camera and YOLO YAML")
                result = self.lab.workflow.start_dataset()
            elif route == "/api/dataset/cancel":
                if body:
                    raise ValueError("Cancel expects an empty request")
                result = self.lab.workflow.cancel_dataset()
            elif route == "/api/dataset/select":
                if set(body) != {"dataset"}:
                    raise ValueError("Select a dataset name from the list")
                result = self.lab.workflow.choose_dataset(body["dataset"])
            elif route == "/api/inspect/start":
                if set(body) != {"samples"}:
                    raise ValueError("Specify label-preview sample count")
                result = self.lab.workflow.start_inspect(body["samples"])
            elif route == "/api/train/start":
                if set(body) != {"model", "device"}:
                    raise ValueError("Specify trained model and device")
                result = self.lab.workflow.start_train(body["model"],body["device"])
            elif route == "/api/validate/start":
                if set(body) != {"model", "split", "device"}:
                    raise ValueError("Specify model, split and device")
                result = self.lab.workflow.start_validate(
                    body["model"],body["split"],body["device"])
            elif route == "/api/live/start":
                if set(body) != {"model"}:
                    raise ValueError("Select a model from the picker")
                result = self.lab.workflow.start_live(body["model"])
            elif route == "/api/model/import":
                if set(body) != {"model"}:
                    raise ValueError("Select a pinned original model")
                result = self.lab.workflow.import_model(body["model"])
            elif route == "/api/task/stop":
                if set(body) != {"name"}:
                    raise ValueError("Choose the stage to stop")
                result = self.lab.workflow.stop_task(body["name"])
            else:
                return self.respond(404, {"error": "Unknown route"})
            return self.respond(200, result)
        except (ValueError, OSError, RuntimeError, TypeError, KeyError) as exc:
            return self.respond(400, {"error": str(exc)})

    def log_message(self, *_args):
        return


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8770)
    args = parser.parse_args()
    lab = Lab()
    httpd = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    httpd.RequestHandlerClass.lab = lab
    print(f"SCROBOT YOLO Lab: http://127.0.0.1:{args.port}", flush=True)
    print("ROS_DOMAIN_ID=83 (isolated); Camera, Scene, Dataset, Labels, Train, Validate and Live tabs", flush=True)
    print("Training requires Ultralytics; a checkpoint must be imported explicitly.", flush=True)
    try:
        httpd.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
        lab.stop()


if __name__ == "__main__":
    main()
