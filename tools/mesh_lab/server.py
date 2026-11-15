#!/usr/bin/env python3
"""SCROBOT Mesh Lab: native Gazebo inspection + optional RViz + benchmarks."""
from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
from urllib.parse import urlparse
import webbrowser
import sys

# Allow direct invocation: python3 tools/mesh_lab/server.py
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.shared.lab_ui import GazeboGUI, log_tail as lab_log_tail
from tools.shared.sim_speed import get as read_speed, save as save_speed, cap_world, response as speed_response
from tools.mesh_lab.audit import audit
from tools.mesh_lab.scene import build_world
from tools.mesh_lab.settings import DEFAULTS, ROOT, read_lighting, read_settings, save_settings, validate

WORLD_FILE = ROOT / "src/scrobot_simulation/worlds/badminton_court.sdf"
HTML_FILE = Path(__file__).with_name("index.html")
RESULT_DIR = ROOT / "evaluation_results/mesh_lab"
WORLD_NAME = "badminton_court"


def local_environment():
    env = os.environ.copy()
    env["GZ_IP"] = "127.0.0.1"
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    env.pop("ROS_LOCALHOST_ONLY", None)
    return env


def process_tree(pid, sig):
    try:
        os.killpg(pid, sig)
    except ProcessLookupError:
        pass


def proc_stats(pid):
    """Linux /proc direct process sample; missing metrics are null, not zero."""
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
        tail = stat[stat.rfind(")") + 2:].split()
        # After the executable name: fields 3 onwards; utime 14, stime 15.
        ticks = int(tail[11]) + int(tail[12])
        rss = int(tail[21]) * os.sysconf("SC_PAGE_SIZE") / 1048576
        return {"ticks": ticks, "rss_mib": round(rss, 2)}
    except (OSError, IndexError, ValueError):
        return None


def pid_and_descendants(root_pid):
    """Include gazebo child processes, ros2 launch and controllers."""
    nodes = {root_pid}
    for _ in range(8):
        new = set(nodes)
        for pid in list(nodes):
            try:
                children = Path(f"/proc/{pid}/task/{pid}/children").read_text().split()
                new.update(int(p) for p in children)
            except (OSError, ValueError):
                continue
        if new == nodes:
            break
        nodes = new
    return nodes


class Runtime:
    def __init__(self, root: Path, launch=True):
        self.lock = threading.RLock()
        self.settings = read_settings()
        self.lighting = read_lighting()
        self.workdir = root
        self.proc = None
        self.rviz = None
        self.gui = GazeboGUI(root / "gazebo_gui.log", env=local_environment())
        self.logs = {}
        self.stats = deque(maxlen=2400)
        self.last_stats_at = None
        self.last_sim_seconds = None
        self.last_wall_at = None
        self.sim_rtf = None
        self.benchmark = None
        self.result = None
        self.prev_cpu = None
        self.gpu_cache = None
        self.gpu_cached_at = 0
        self.errors = deque(maxlen=10)
        self.gz_node = None
        try:
            from gz.transport13 import Node
            from gz.msgs10.world_stats_pb2 import WorldStatistics
            self.gz_node = Node()
            self.gz_node.subscribe(
                WorldStatistics, f"/world/{WORLD_NAME}/stats", self.on_world_stats)
        except ImportError:
            self.errors.append("Gazebo Python transport unavailable; RTF will show as unavailable")
        if launch:
            self.start()

    def on_world_stats(self, msg):
        now = time.monotonic()
        sim = float(msg.sim_time.sec) + float(msg.sim_time.nsec) / 1e9
        with self.lock:
            if self.last_sim_seconds is not None and self.last_wall_at is not None:
                dt = now - self.last_wall_at
                if dt > .05 and sim >= self.last_sim_seconds:
                    self.sim_rtf = (sim - self.last_sim_seconds) / dt
            self.last_sim_seconds = sim
            self.last_wall_at = now
            self.last_stats_at = now

    def command(self):
        s = self.settings
        if s["model"] == "shuttle":
            # No robot, controllers, depth sensors or bridge process needed.
            return ["ros2", "launch", "scrobot_simulation", "gazebo.launch.py",
                    f"world:={self.workdir / 'mesh_lab_world.sdf'}",
            "headless:=true", "headless:=true", "gz_verbosity:=2"]
        return [
            "ros2", "launch", "scrobot_simulation", "simulation.launch.py",
            f"world:={self.workdir / 'mesh_lab_world.sdf'}",
            "x:=2.0", "y:=3.05", "z:=0.003", "yaw:=3.141592653589793",
            "robot_visual_mode:=" + s["robot_visual_mode"],
            "enable_magnetometer:=false",
            "enable_depth_camera:=" + str(s["depth_camera"]).lower(),
            "gz_verbosity:=2",
        ]

    def start(self):
        if self.proc and self.proc.poll() is None:
            return
        worldfile = self.workdir / "mesh_lab_world.sdf"
        worldfile.write_text(build_world(WORLD_FILE, self.lighting, self.settings))
        cap_world(worldfile, worldfile, read_speed("mesh"))
        log = (self.workdir / "gazebo.log").open("a")
        self.logs["gazebo"] = log
        self.proc = subprocess.Popen(self.command(), stdout=log,
                                     stderr=subprocess.STDOUT,
                                     start_new_session=True, env=local_environment())
        self.reset_measurements()

    def stop(self):
        self.gui.close()
        for name in ("rviz", "proc"):
            proc = getattr(self, name)
            if proc and proc.poll() is None:
                process_tree(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process_tree(proc.pid, signal.SIGTERM)
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process_tree(proc.pid, signal.SIGKILL)
                        proc.wait(timeout=3)
            setattr(self, name, None)
        for fh in self.logs.values():
            fh.close()
        self.logs.clear()

    def restart(self, settings):
        updated = validate(settings)
        # Check generated world before shutting down the working Gazebo process.
        generated = build_world(WORLD_FILE, self.lighting, updated)
        self.stop()
        self.settings = updated
        (self.workdir / "mesh_lab_world.sdf").write_text(generated)
        self.start()

    def open_rviz(self):
        if self.settings["model"] == "shuttle":
            raise ValueError("RViz robot-model inspection requires robot or both mode")
        if self.rviz and self.rviz.poll() is None:
            return "RViz already running"
        log = (self.workdir / "rviz.log").open("a")
        self.logs["rviz"] = log
        # Use an isolated RViz config with no navigation displays.
        config = Path(__file__).with_name("mesh.rviz")
        self.rviz = subprocess.Popen(
            ["rviz2", "-d", str(config), "--ros-args", "-p", "use_sim_time:=true"],
            stdout=log, stderr=subprocess.STDOUT,
            env=local_environment(), start_new_session=True,
        )
        return "RViz launched; robot description/TF use existing simulation publishers"

    def close_rviz(self):
        if self.rviz and self.rviz.poll() is None:
            process_tree(self.rviz.pid, signal.SIGINT)
            try:
                self.rviz.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process_tree(self.rviz.pid, signal.SIGTERM)
        self.rviz = None
        log = self.logs.pop("rviz", None)
        if log:
            log.close()
        return "RViz closed"

    def reset_measurements(self):
        self.stats.clear()
        self.prev_cpu = None
        self.last_sim_seconds = None
        self.last_wall_at = None
        self.sim_rtf = None
        self.benchmark = None
        self.result = None

    def gpu(self):
        now = time.monotonic()
        if now - self.gpu_cached_at < 2:
            return self.gpu_cache
        self.gpu_cached_at = now
        self.gpu_cache = None
        try:
            p = subprocess.run(
                ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used",
                 "--format=csv,noheader,nounits"],
                capture_output=True, text=True, timeout=2, check=False)
            if p.returncode == 0 and p.stdout.strip():
                util, memory = [float(x.strip()) for x in p.stdout.splitlines()[0].split(",")[:2]]
                self.gpu_cache = {"util_percent": util, "memory_mib": memory,
                                  "scope": "whole GPU, not Gazebo-only"}
        except (OSError, ValueError, subprocess.TimeoutExpired):
            pass
        return self.gpu_cache

    def sample(self):
        now = time.monotonic()
        if self.proc is None or self.proc.poll() is not None:
            return {"at": now, "rtf": None, "cpu_percent": None,
                    "rss_mib": None, "gpu": None}
        pids = pid_and_descendants(self.proc.pid)
        metrics = [proc_stats(pid) for pid in pids]
        metrics = [m for m in metrics if m]
        ticks = sum(m["ticks"] for m in metrics)
        rss = sum(m["rss_mib"] for m in metrics)
        cpu = None
        if self.prev_cpu:
            previous_at, previous_ticks = self.prev_cpu
            dt = now - previous_at
            if dt > .1:
                cpu = round(max(0, (ticks - previous_ticks) /
                                    (os.sysconf("SC_CLK_TCK") * dt) * 100), 1)
        self.prev_cpu = now, ticks
        gpu = self.gpu()
        rtf = round(self.sim_rtf, 3) if self.last_stats_at and now-self.last_stats_at < 4 and self.sim_rtf is not None else None
        item = {"at": now, "rtf": rtf, "cpu_percent": cpu, "rss_mib": round(rss, 1),
                "gpu": gpu}
        self.stats.append(item)
        return item

    def begin_benchmark(self):
        if self.benchmark is not None:
            raise ValueError("Benchmark already running")
        if self.proc is None or self.proc.poll() is not None:
            raise ValueError("Gazebo is not running")
        if self.rviz and self.rviz.poll() is None:
            raise ValueError("Close RViz before benchmarking")
        if self.gui.running:
            raise ValueError("Close Gazebo GUI before benchmarking")
        duration = self.settings["benchmark_seconds"]
        self.benchmark = {"start": time.monotonic(), "duration": duration,
                          "samples": [], "settings": dict(self.settings),
                          "lighting": dict(self.lighting)}
        self.result = None
        return f"Sampling {duration} seconds; wait for the benchmark result"

    @staticmethod
    def average(samples, key):
        values = [v[key] for v in samples if v[key] is not None]
        return round(sum(values)/len(values), 3) if values else None

    def finish_benchmark(self):
        job = self.benchmark
        if not job:
            return
        samples = job["samples"]
        result = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "settings": job["settings"], "saved_lighting": job["lighting"],
            "duration_seconds": job["duration"], "sample_count": len(samples),
            "averages": {
                "rtf": self.average(samples, "rtf"),
                "cpu_percent": self.average(samples, "cpu_percent"),
                "rss_mib": self.average(samples, "rss_mib"),
            },
            "gpu_scope": "whole GPU (if NVIDIA SMI available)",
            "measurement_note": "CPU and RSS include ROS launch descendant processes, not the separate Gazebo GUI. "
                                "GPU utilization is device-wide. RTF is computed from Gazebo world stats. "
                                "GUI FPS is not measured.",
            "samples": samples,
        }
        RESULT_DIR.mkdir(parents=True, exist_ok=True)
        file = RESULT_DIR / ("mesh_benchmark_" +
                             datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".json")
        file.write_text(json.dumps(result, indent=2) + "\n")
        result["file"] = str(file)
        self.result = result
        self.benchmark = None

    def status(self):
        with self.lock:
            item = self.sample()
            if self.benchmark is not None:
                self.benchmark["samples"].append(item)
                if time.monotonic() - self.benchmark["start"] >= self.benchmark["duration"]:
                    self.finish_benchmark()
            return {
                "gazebo_running": self.proc is not None and self.proc.poll() is None,
                "rviz_running": self.rviz is not None and self.rviz.poll() is None,
                "gazebo_gui": self.gui.status(),
                "logs": {"gazebo": lab_log_tail(self.workdir / "gazebo.log"),
                         "gazebo_gui": lab_log_tail(self.gui.log_path),
                         "rviz": lab_log_tail(self.workdir / "rviz.log")},
                "settings": self.settings, "sample": item,
                "benchmark": {
                    "elapsed": round(time.monotonic()-self.benchmark["start"], 1),
                    "duration": self.benchmark["duration"],
                    "samples": len(self.benchmark["samples"]),
                } if self.benchmark else None,
                "result": self.result, "errors": list(self.errors),
            }


class Handler(BaseHTTPRequestHandler):
    lab = None

    def send_json(self, status, data):
        output = json.dumps(data).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(output)))
        self.end_headers()
        self.wfile.write(output)

    def do_GET(self):
        path = urlparse(self.path).path
        try:
            if path == "/":
                data = HTML_FILE.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            elif path == "/api/sim-speed":
                self.send_json(200,speed_response("mesh", bool(self.lab.proc and self.lab.proc.poll() is None)))
            elif path == "/api/status":
                self.send_json(200, self.lab.status())
            elif path == "/api/audit":
                self.send_json(200, audit(self.lab.settings["robot_visual_mode"]))
            else:
                self.send_json(404, {"error": "Unknown endpoint"})
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:
            self.send_json(500, {"error": str(exc)})

    def do_POST(self):
        path = urlparse(self.path).path
        if path not in ("/api/apply", "/api/save", "/api/rviz/open",
                        "/api/rviz/close", "/api/benchmark",
                        "/api/gazebo-gui/open", "/api/gazebo-gui/close",
                        "/api/sim-speed"):
            self.send_json(404, {"error": "Unknown endpoint"})
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if size < 0 or size > 8192:
                raise ValueError("Invalid request size")
            payload = json.loads(self.rfile.read(size)) if size else {}
            if not isinstance(payload, dict):
                raise ValueError("Expected an object")
            with self.lab.lock:
                if path == "/api/sim-speed":
                    if set(payload) != {"max_rtf"}:raise ValueError("Set only max_rtf")
                    speed=save_speed("mesh",payload["max_rtf"])
                    msg=f"Saved {speed:.2f}x RTF cap; restart Gazebo to apply"
                elif path == "/api/apply":
                    self.lab.restart(payload)
                    msg = "Restarted temporary inspection world; saved YAML unchanged"
                elif path == "/api/save":
                    p = save_settings(payload)
                    msg = f"Saved Mesh Lab preferences to {p}; production unchanged"
                elif path == "/api/gazebo-gui/open":
                    if payload:
                        raise ValueError("GUI open expects empty object")
                    if not self.lab.proc or self.lab.proc.poll() is not None:
                        raise RuntimeError("Start Gazebo before opening the GUI")
                    msg = self.lab.gui.open()
                elif path == "/api/gazebo-gui/close":
                    if payload:
                        raise ValueError("GUI close expects empty object")
                    msg = self.lab.gui.close()
                elif path == "/api/rviz/open":
                    msg = self.lab.open_rviz()
                elif path == "/api/rviz/close":
                    msg = self.lab.close_rviz()
                else:
                    msg = self.lab.begin_benchmark()
            self.send_json(200, {"ok": True, "message": msg})
        except (ValueError, OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
            self.send_json(400, {"ok": False, "error": str(exc)})

    def log_message(self, fmt, *args):
        if not self.path.startswith(("/api/status", "/api/audit")):
            super().log_message(fmt, *args)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8771)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--attach", action="store_true",
                        help="Dashboard only: connect to an existing Gazebo world")
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error("Port must be between 1024 and 65535")
    if not WORLD_FILE.is_file():
        parser.error(f"Source world not found: {WORLD_FILE}")
    if not args.attach and not __import__("shutil").which("ros2"):
        parser.error("ROS 2 not available; source Jazzy and scrobot_ws/install/setup.bash")
    with tempfile.TemporaryDirectory(prefix="scrobot_mesh_lab_") as folder:
        # Import here to make the rest of the module unit-testable off workstation.
        lab = Runtime(Path(folder), launch=not args.attach)
        Handler.lab = lab
        server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
        address = f"http://127.0.0.1:{args.port}"
        print("SCROBOT Mesh Lab:", address, flush=True)
        if not args.no_browser:
            webbrowser.open_new_tab(address)
        try:
            server.serve_forever(poll_interval=.5)
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()
            lab.stop()


if __name__ == "__main__":
    main()
