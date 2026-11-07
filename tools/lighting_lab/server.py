#!/usr/bin/env python3
"""SCROBOT Gazebo Harmonic lighting lab -- local three-camera dashboard.

Starts ONLY the standard simulation (no Nav2/mission/evaluator), generates a
temporary derivative SDF, and uses Gazebo Transport for every runtime action.
The original badminton_court.sdf is never edited.
"""
from __future__ import annotations

import argparse
from collections import deque
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse
import webbrowser

from scene import (
    WORLD, DEFAULTS, VISUAL_GROUPS, build_world, centered_positions,
    checked_colors, checked_lights, checked_venue, default_settings,
    hsv_rgb, kelvin_rgb, light_direction, bounded, integer,
)


SOURCE = Path(__file__).resolve().parents[2] / (
    "src/scrobot_simulation/worlds/badminton_court.sdf"
)
HTML = Path(__file__).with_name("index.html")
PRESSTS = Path.home() / "scrobot_ws/evaluation_results/lighting_lab/presets"
CAMERAS = {
    "robot": "/camera/color",
    "overhead": "/lighting_lab/overhead",
    "free": "/lighting_lab/free",
}


def quaternion_from_yaw_pitch(yaw_deg, pitch_deg):
    yaw = math.radians(yaw_deg) / 2
    pitch = math.radians(pitch_deg) / 2
    # ZYX rotation, roll = 0
    return (
        -math.sin(yaw) * math.sin(pitch),
        math.cos(yaw) * math.sin(pitch),
        math.sin(yaw) * math.cos(pitch),
        math.cos(yaw) * math.cos(pitch),
    )


class GazeboAdapter:
    def __init__(self, settings):
        try:
            from gz.transport13 import Node
            from gz.msgs10.image_pb2 import Image as GzImage
            from gz.msgs10.world_stats_pb2 import WorldStatistics
            from gz.msgs10.boolean_pb2 import Boolean
            from gz.msgs10.entity_pb2 import Entity
            from gz.msgs10.entity_factory_pb2 import EntityFactory
            from gz.msgs10.light_pb2 import Light
            from gz.msgs10.pose_pb2 import Pose
            from gz.msgs10.visual_pb2 import Visual
            from PIL import Image
        except ImportError as exc:
            raise RuntimeError(
                "Lighting Lab requires Python 3 bindings: "
                "sudo apt install python3-gz-transport13 python3-gz-msgs10 python3-pil"
            ) from exc

        self.GzImage, self.WorldStatistics = GzImage, WorldStatistics
        self.Boolean, self.Entity, self.EntityFactory = Boolean, Entity, EntityFactory
        self.Light, self.Pose, self.Visual, self.PILImage = Light, Pose, Visual, Image
        self.node = Node()
        self.lock = threading.RLock()
        self._gpu_cached_at = 0.0
        self._gpu_info = None
        self.settings = settings
        self.light_names = {
            name for name, *_ in centered_positions(checked_lights(settings["lights"]))
        }
        self.frames = {}
        self.frame_times = {key: deque(maxlen=20) for key in CAMERAS}
        self.sensor_times = {key: deque(maxlen=80) for key in CAMERAS}
        self.exposures = {}
        self.sim_stats = {}
        self.errors = deque(maxlen=12)
        self.last_action = "Calibration world starting"
        self.gui_pose = None
        self.gui_pose_at = 0.0
        self.gui_follow = False
        self.gui_last_sent = None
        self.gui_stop = threading.Event()
        self.gui_pose_subscription_ok = self.node.subscribe(
            Pose, "/gui/camera/pose", self.on_gui_pose
        )
        self.gui_thread = threading.Thread(
            target=self._gui_follow_loop, name="GazeboOrbitSync", daemon=True
        )
        self.gui_thread.start()
        for key, topic in CAMERAS.items():
            ok = self.node.subscribe(GzImage, topic,
                                     lambda msg, key=key: self.on_image(key, msg))
            if not ok:
                self.errors.append(f"Could not subscribe to {topic}")
        self.node.subscribe(WorldStatistics, f"/world/{WORLD}/stats", self.on_stats)

    def on_image(self, key, message):
        # Avoid expensive conversion of the full 1280x720 robot image.
        now = time.monotonic()
        with self.lock:
            self.sensor_times[key].append(now)
            last = self.frame_times[key][-1] if self.frame_times[key] else 0
        # Browser previews are deliberately throttled to ~5 FPS for CPU
        # efficiency; sensor FPS below is measured before this decimation.
        if now - last < 0.18:
            return
        width, height = int(message.width), int(message.height)
        data = message.data
        if width < 1 or height < 1 or width > 4096 or height > 2160:
            return
        try:
            if len(data) == width * height * 3:
                mode = "RGB"
            elif len(data) == width * height * 4:
                mode = "RGBA"
            elif len(data) == width * height:
                mode = "L"
            else:
                return
            picture = self.PILImage.frombytes(mode, (width, height), data)
            if picture.mode != "RGB":
                picture = picture.convert("RGB")
            picture.thumbnail((860, 490))
            # Estimate camera clipping, not physical illuminance/lux.
            gray = picture.convert("L").resize((80, 45))
            samples = gray.tobytes()
            clipped_bright = sum(v >= 248 for v in samples) / len(samples)
            clipped_dark = sum(v <= 12 for v in samples) / len(samples)
            avg_brightness = sum(samples) / len(samples) / 255
            stream = io.BytesIO()
            picture.save(stream, format="JPEG", quality=72, optimize=False)
            with self.lock:
                self.frames[key] = stream.getvalue()
                self.frame_times[key].append(now)
                self.exposures[key] = {
                    "mean": round(avg_brightness * 100, 1),
                    "clipped_bright": round(clipped_bright * 100, 1),
                    "clipped_dark": round(clipped_dark * 100, 1),
                }
        except (OSError, ValueError) as exc:
            with self.lock:
                self.errors.append(f"{key} frame error: {exc}")

    def on_gui_pose(self, message):
        # CameraTracking publishes the true GUI orbit camera pose. Only store
        # it in this callback; never make a blocking transport request from a
        # Gazebo callback thread.
        with self.lock:
            self.gui_pose = self.Pose()
            self.gui_pose.CopyFrom(message)
            self.gui_pose_at = time.monotonic()

    def _gui_follow_loop(self):
        while not self.gui_stop.wait(0.45):
            with self.lock:
                if not self.gui_follow or self.gui_pose is None:
                    continue
                if time.monotonic() - self.gui_pose_at > 2.5:
                    continue
                raw = self.Pose()
                raw.CopyFrom(self.gui_pose)
                key = (
                    round(raw.position.x, 3), round(raw.position.y, 3),
                    round(raw.position.z, 3),
                    round(raw.orientation.x, 4), round(raw.orientation.y, 4),
                    round(raw.orientation.z, 4), round(raw.orientation.w, 4),
                )
                if key == self.gui_last_sent:
                    continue
            try:
                self.apply_gui_pose(raw)
                with self.lock:
                    self.gui_last_sent = key
            except (RuntimeError, OSError, ValueError) as exc:
                with self.lock:
                    self.errors.append(f"Orbit follow: {exc}")

    def apply_gui_pose(self, raw):
        msg = self.Pose()
        msg.CopyFrom(raw)
        msg.name = "lab_free_camera"
        self.call("set_pose", msg, self.Pose, timeout=2500)
        # Keep JSON preset free pose aligned to the GUI orbit-camera position.
        q = msg.orientation
        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z),
        )
        pitch_sin = max(-1., min(1., 2 * (q.w * q.y - q.z * q.x)))
        pitch = math.asin(pitch_sin)
        with self.lock:
            self.settings["free"] = {
                "x": round(msg.position.x, 3),
                "y": round(msg.position.y, 3),
                "z": round(msg.position.z, 3),
                "yaw": round(math.degrees(yaw), 2),
                "pitch": round(math.degrees(pitch), 2),
            }

    def sync_gui_camera(self, follow, copy_now):
        if not isinstance(follow, bool) or not isinstance(copy_now, bool):
            raise ValueError("follow/copy_now must be booleans")
        with self.lock:
            raw = self.Pose()
            if self.gui_pose is None or time.monotonic() - self.gui_pose_at > 3.0:
                raise RuntimeError(
                    "No fresh /gui/camera/pose; enable Camera Tracking in Gazebo GUI"
                )
            raw.CopyFrom(self.gui_pose)
        if copy_now:
            self.apply_gui_pose(raw)
        with self.lock:
            self.gui_follow = follow
            self.gui_last_sent = None
        self.last_action = (
            "Following Gazebo orbit view" if follow
            else "Copied Gazebo orbit view" if copy_now else "Orbit sync disabled"
        )
        return self.last_action

    def on_stats(self, message):
        with self.lock:
            self.sim_stats = {
                "rtf": round(float(message.real_time_factor), 3),
                "sim_seconds": round(
                    float(message.sim_time.sec) + message.sim_time.nsec * 1e-9, 1
                ),
                "paused": bool(message.paused),
                "at": time.monotonic(),
            }

    def call(self, suffix, req, cls, timeout=4000):
        result, reply = self.node.request(
            f"/world/{WORLD}/{suffix}", req, cls, self.Boolean, timeout
        )
        if not result or not reply.data:
            raise RuntimeError(f"Gazebo /world/{WORLD}/{suffix} rejected or timed out")
        return True

    def light_message(self, name, x, y, z, cfg):
        m = self.Light()
        m.name = name
        m.type = self.Light.SPOT if cfg["kind"] == "spot" else self.Light.POINT
        m.intensity = cfg["brightness"] / 100.0
        m.range = cfg["range"]
        m.attenuation_constant = cfg["constant"]
        m.attenuation_linear = cfg["linear"]
        m.attenuation_quadratic = cfg["quadratic"]
        m.cast_shadows = cfg["shadows"]
        m.spot_inner_angle = cfg["inner"]
        m.spot_outer_angle = cfg["outer"]
        m.spot_falloff = cfg["falloff"]
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        m.direction.x, m.direction.y, m.direction.z = light_direction(x, y, z)
        rgb = kelvin_rgb(cfg["temperature"])
        for field, multiplier in (("diffuse", 1.0), ("specular", cfg["specular"])):
            color = getattr(m, field)
            color.r, color.g, color.b, color.a = (
                min(1., rgb[0] * multiplier),
                min(1., rgb[1] * multiplier),
                min(1., rgb[2] * multiplier), 1.
            )
        return m

    def apply_lights(self, raw):
        cfg = checked_lights(raw)
        positions = list(centered_positions(cfg))
        desired = {name for name, *_ in positions}
        # Do not remove existing lights until the world has responded to an
        # ordinary command. If the simulation is frozen, raise without mutation.
        for name, x, y, z in positions:
            msg = self.light_message(name, x, y, z, cfg)
            if name in self.light_names:
                self.call("light_config", msg, self.Light)
            else:
                req = self.EntityFactory()
                req.light.CopyFrom(msg)
                self.call("create", req, self.EntityFactory)
                self.light_names.add(name)
        for name in sorted(self.light_names - desired):
            req = self.Entity()
            req.name = name
            req.type = self.Entity.LIGHT
            self.call("remove", req, self.Entity)
            self.light_names.remove(name)
        self.settings["lights"] = cfg
        self.last_action = f"Applied {len(desired)} {cfg['kind']} lights"
        return self.last_action

    def apply_materials(self, raw):
        cfg = checked_colors(raw)
        for group, members in VISUAL_GROUPS.items():
            rgb = hsv_rgb(cfg[group])
            for model_name, visual_name in members:
                parent = {
                    "court_floor": "floor_link",
                    "simulation_safety_boundary": "boundary_link",
                    "net_assembly": "net_link",
                }[model_name]
                msg = self.Visual()
                msg.name = visual_name
                msg.parent_name = parent
                for item, multiplier in (("ambient", .8), ("diffuse", 1.0)):
                    c = getattr(msg.material, item)
                    c.r, c.g, c.b, c.a = (
                        rgb[0] * multiplier, rgb[1] * multiplier,
                        rgb[2] * multiplier, 1.0
                    )
                # With partial material fields VisualCmd updates, specular
                # can otherwise be reset to a default.
                msg.material.specular.r = 0.02
                msg.material.specular.g = 0.02
                msg.material.specular.b = 0.02
                msg.material.specular.a = 1.0
                self.call("visual_config", msg, self.Visual)
        self.settings["colors"] = cfg
        self.last_action = "Applied court, venue, wall and pole materials"
        return self.last_action

    def set_pose(self, entity, values):
        req = self.Pose()
        req.name = entity
        req.position.x = values["x"]
        req.position.y = values["y"]
        req.position.z = values["z"]
        x, y, z, w = quaternion_from_yaw_pitch(
            values["yaw"], values.get("pitch", 0.0)
        )
        req.orientation.x, req.orientation.y = x, y
        req.orientation.z, req.orientation.w = z, w
        self.call("set_pose", req, self.Pose)

    def move_camera(self, raw, key):
        if key == "free":
            with self.lock:
                self.gui_follow = False  # Explicit manual move takes priority.
        if key == "robot":
            position = {
                "x": bounded(raw["x"], -7, 7, "robot.x"),
                "y": bounded(raw["y"], -3.5, 3.5, "robot.y"),
                "z": 0.003,
                "yaw": bounded(raw["yaw"], -360, 360, "robot.yaw"),
            }
            self.set_pose("scrobot", position)
        else:
            position = {
                "x": bounded(raw["x"], -18, 18, "free.x"),
                "y": bounded(raw["y"], -14, 14, "free.y"),
                "z": bounded(raw["z"], 1, 16, "free.z"),
                "yaw": bounded(raw["yaw"], -360, 360, "free.yaw"),
                "pitch": bounded(raw["pitch"], -60, 88, "free.pitch"),
            }
            self.set_pose("lab_free_camera", position)
        self.settings[key] = position
        self.last_action = f"Moved {key} camera"
        return self.last_action

    def get_frame(self, key):
        with self.lock:
            return self.frames.get(key)

    def gpu_metrics(self):
        # Diagnostic only. Not a per-process GPU load nor a lighting meter.
        now = time.monotonic()
        if now - self._gpu_cached_at < 5.0:
            return self._gpu_info
        self._gpu_cached_at = now
        if shutil.which("nvidia-smi") is None:
            return None
        try:
            proc = subprocess.run(
                ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,memory.total",
                 "--format=csv,noheader,nounits"],
                capture_output=True, text=True, check=False, timeout=1.5
            )
            if proc.returncode == 0:
                fields = proc.stdout.splitlines()[0].split(",")
                self._gpu_info = {
                    "utilization_percent": float(fields[0].strip()),
                    "memory_used_mib": float(fields[1].strip()),
                    "memory_total_mib": float(fields[2].strip()),
                }
        except (OSError, ValueError, IndexError, subprocess.TimeoutExpired):
            pass
        return self._gpu_info

    def status(self):
        now = time.monotonic()
        gpu = self.gpu_metrics()
        with self.lock:
            stats = dict(self.sim_stats)
            stats["stats_age_s"] = (
                round(now - stats.pop("at"), 1) if "at" in stats else None
            )
            frame_info = {}
            for key, timestamps in self.frame_times.items():
                raw = [x for x in self.sensor_times[key] if now - x <= 3.0]
                frames = [x for x in timestamps if now - x <= 3.0]
                sensor_fps = (
                    (len(raw) - 1) / (raw[-1] - raw[0])
                    if len(raw) >= 2 and raw[-1] > raw[0] else 0.0
                )
                preview_fps = (
                    (len(frames) - 1) / (frames[-1] - frames[0])
                    if len(frames) >= 2 and frames[-1] > frames[0] else 0.0
                )
                frame_info[key] = {
                    "fps": round(sensor_fps, 1),
                    "preview_fps": round(preview_fps, 1),
                    "age_s": round(now - timestamps[-1], 1) if timestamps else None,
                    "exposure": self.exposures.get(key),
                }
            return {
                "sim": stats, "feeds": frame_info,
                "lights": len(self.light_names), "settings": self.settings,
                "load_1m": round(os.getloadavg()[0], 2),
                "cpu_cores": os.cpu_count(),
                "gpu": gpu,
                "last_action": self.last_action,
                "errors": list(self.errors)[-5:],
                "gui_camera": self._gui_status(now),
            }

    def _gui_status(self, now):
        fresh = self.gui_pose is not None and now - self.gui_pose_at < 3.0
        result = {
            "available": fresh,
            "topic_subscribed": bool(self.gui_pose_subscription_ok),
            "following": self.gui_follow,
            "age_s": round(now - self.gui_pose_at, 1) if self.gui_pose is not None else None,
        }
        if fresh:
            result["position"] = {
                "x": round(self.gui_pose.position.x, 3),
                "y": round(self.gui_pose.position.y, 3),
                "z": round(self.gui_pose.position.z, 3),
            }
        return result

    def stop(self):
        self.gui_stop.set()
        self.gui_thread.join(timeout=3.0)


class Lab:
    def __init__(self, settings, launch, workdir, source):
        self.settings = settings
        self.adapter = GazeboAdapter(settings)
        self.lock = threading.RLock()
        self.proc = None
        self.workdir = workdir
        self.source = source
        self.spawned = []
        self.spawn_counter = 0
        self.log = workdir / "gazebo_launch.log"
        if launch:
            worldfile = workdir / "lighting_lab_world.sdf"
            worldfile.write_text(build_world(source, settings))
            robot = settings["robot"]
            command = [
                "ros2", "launch", "scrobot_simulation", "simulation.launch.py",
                f"world:={worldfile}",
                f"x:={robot['x']}", f"y:={robot['y']}",
                f"z:={robot['z']}",
                f"yaw:={math.radians(robot['yaw'])}",
                "enable_magnetometer:=false",
                "gz_verbosity:=2",
            ]
            self.loghandle = self.log.open("w")
            self.proc = subprocess.Popen(
                command, stdout=self.loghandle, stderr=subprocess.STDOUT,
                start_new_session=True
            )
            print(f"Gazebo PID {self.proc.pid}; log {self.log}", flush=True)

    def spawn_shuttles(self, raw):
        count = integer(raw.get("count", 20), 1, 100, "shuttle count")
        seed = integer(raw.get("seed", 20261003), 0, 2147483647, "seed")
        density = raw.get("density", "uniform")
        if density not in ("uniform", "center", "net"):
            raise ValueError("density must be uniform, center or net")
        # Remove previous batches before spawning, so a new randomization
        # does not silently accumulate hundreds of shuttles in the court.
        for name in self.spawned:
            req = self.adapter.Entity()
            req.name = name
            req.type = self.adapter.Entity.MODEL
            self.adapter.call("remove", req, self.adapter.Entity)
        self.spawned.clear()
        self.spawn_counter += 1
        batch = f"lab{int(time.time())}{self.spawn_counter}"
        command = [
            "ros2", "run", "scrobot_simulation", "spawn_shuttles",
            "--mode", "random", "--count", str(count),
            "--seed", str(seed), "--density", density,
            "--world", WORLD, "--batch", batch,
            "--parallel-workers", "4",
        ]
        completed = subprocess.run(
            command, capture_output=True, text=True,
            timeout=180, check=False
        )
        # Track all requested entity names even on partial failure; this
        # lets the next request clean up surviving shuttles where possible.
        self.spawned = [f"random{batch}-{i:02d}" for i in range(1, count + 1)]
        if completed.returncode:
            self.adapter.errors.append(completed.stderr[-500:])
            raise RuntimeError("Shuttle spawn failed: " + completed.stderr[-350:])
        self.settings["shuttles"] = {
            "count": count, "seed": seed, "density": density
        }
        self.adapter.last_action = f"Spawned {count} random shuttles (seed {seed})"
        return self.adapter.last_action

    def save_preset(self, raw):
        name = str(raw.get("name", "lighting_calibration"))
        if not re.fullmatch(r"[A-Za-z0-9_-]{1,60}", name):
            raise ValueError("Preset name: 1..60 letters, digits, hyphens, underscores")
        PRESSTS.mkdir(parents=True, exist_ok=True)
        output = PRESSTS / (name + ".json")
        snapshot = self.adapter.status()
        commit = subprocess.run(
            ["git", "-C", str(self.source.parents[3]), "rev-parse", "HEAD"],
            capture_output=True, text=True, check=False, timeout=3
        )
        data = {
            "saved_utc": datetime.now(timezone.utc).isoformat(),
            "branch_commit": commit.stdout.strip() if commit.returncode == 0 else "unknown",
            "source_world_sha256": hashlib.sha256(self.source.read_bytes()).hexdigest(),
            "settings": self.settings,
            "performance": {
                "sim": snapshot["sim"], "feeds": snapshot["feeds"],
                "gpu": snapshot["gpu"], "load_1m": snapshot["load_1m"],
                "cpu_cores": snapshot["cpu_cores"],
            },
        }
        output.write_text(json.dumps(data, indent=2) + "\n")
        self.adapter.last_action = f"Saved {output}"
        return str(output)

    def close(self):
        if self.proc is not None and self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, signal.SIGINT)
                self.proc.wait(timeout=10)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(self.proc.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        if hasattr(self, "loghandle"):
            self.loghandle.close()


class Handler(BaseHTTPRequestHandler):
    lab: Lab = None

    def reply(self, status, data, content_type="application/json"):
        body = data if isinstance(data, bytes) else json.dumps(data).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            self.reply(200, HTML.read_bytes(), "text/html; charset=utf-8")
        elif path == "/api/status":
            result = self.lab.adapter.status()
            result["gazebo_alive"] = (
                self.lab.proc.poll() is None if self.lab.proc is not None else None
            )
            self.reply(200, result)
        elif path.startswith("/frame/"):
            key = path.split("/")[-1]
            if key not in CAMERAS:
                self.reply(404, {"error": "Unknown camera"})
                return
            jpg = self.lab.adapter.get_frame(key)
            if jpg is None:
                self.reply(503, {"error": f"Waiting for {CAMERAS[key]}"})
            else:
                self.reply(200, jpg, "image/jpeg")
        else:
            self.reply(404, {"error": "Not found"})

    def do_POST(self):
        path = urlparse(self.path).path
        if path not in ("/api/lights", "/api/materials", "/api/robot",
                        "/api/free", "/api/shuttles", "/api/preset"):
            self.reply(404, {"error": "Unknown action"})
            return
        size = int(self.headers.get("Content-Length", "0"))
        if size < 1 or size > 16384:
            self.reply(400, {"error": "Invalid request size"})
            return
        try:
            payload = json.loads(self.rfile.read(size))
            if not isinstance(payload, dict):
                raise ValueError("Expected a JSON object")
            with self.lab.lock:
                if path == "/api/lights":
                    text = self.lab.adapter.apply_lights(payload)
                elif path == "/api/materials":
                    text = self.lab.adapter.apply_materials(payload)
                elif path in ("/api/robot", "/api/free"):
                    text = self.lab.adapter.move_camera(payload, path[5:])
                elif path == "/api/shuttles":
                    text = self.lab.spawn_shuttles(payload)
                else:
                    text = self.lab.save_preset(payload)
            self.reply(200, {"ok": True, "message": text})
        except (KeyError, ValueError, TypeError, RuntimeError,
                subprocess.TimeoutExpired) as exc:
            self.lab.adapter.errors.append(str(exc))
            self.reply(400, {"ok": False, "error": str(exc)})
        except Exception as exc:
            self.lab.adapter.errors.append(f"Internal error: {exc}")
            self.reply(500, {"ok": False, "error": "Internal error; see Python console"})

    def log_message(self, fmt, *args):
        # Avoid an access-log line for every camera JPEG frame.
        if not self.path.startswith(("/frame/", "/api/status")):
            super().log_message(fmt, *args)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--attach", action="store_true",
                        help="Attach to an already running lighting-lab world")
    parser.add_argument("--preset", type=Path, default=None)
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error("port must be 1024..65535")
    settings = default_settings()
    if args.preset:
        settings.update(json.loads(args.preset.read_text())["settings"])
    if not SOURCE.is_file():
        parser.error("Missing source world: " + str(SOURCE))
    if shutil.which("ros2") is None and not args.attach:
        parser.error("ros2 not found; source /opt/ros/jazzy/setup.bash and workspace")
    with tempfile.TemporaryDirectory(prefix="scrobot_lighting_lab_") as tmp:
        lab = Lab(settings, not args.attach, Path(tmp), SOURCE)
        Handler.lab = lab
        server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
        url = f"http://127.0.0.1:{args.port}"
        print("SCROBOT lighting lab:", url, flush=True)
        print("Full Gazebo log:", lab.log, flush=True)
        if not args.no_browser:
            webbrowser.open(url)
        try:
            server.serve_forever(poll_interval=0.4)
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()
            lab.close()


if __name__ == "__main__":
    main()
