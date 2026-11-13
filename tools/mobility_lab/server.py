#!/usr/bin/env python3
"""SCROBOT Mobility Lab: isolated floor-loaded Gazebo + web calibration GUI.

Usage, from a sourced ROS 2 Jazzy / SCROBOT workspace:
    python3 ~/scrobot_ws/tools/mobility_lab/server.py

This is localhost only, makes no production URDF edits, and saves data under
~/scrobot_ws/evaluation_results/mobility_lab/.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
import webbrowser

from core import DEFAULTS, reference_csv, validate_settings
from reference import (
    bundled_validation, compare_validation, phase_aligned_reference, status_label,
)
from publish import publish_saved_calibration, published_status

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lab_configs import path_for as lab_yaml_path, read as read_lab_yaml, save as save_lab_yaml

HERE = Path(__file__).resolve().parent
OUTPUT = Path.home() / "scrobot_ws/evaluation_results/mobility_lab"
SETTINGS_PATH = OUTPUT / "settings.json"
PHYSICS = ("drive_contact_mu", "caster_contact_mu",
           "wheel_joint_damping", "wheel_collision_width",
           "drive_slip1", "drive_slip2")
CSV_FIELDS = (
    "t", "input_left_rpm", "input_right_rpm",
    "sent_left_rpm", "sent_right_rpm",
    "actual_left_rpm", "actual_right_rpm",
    "encoder_v_m_s", "encoder_yaw_rad_s",
    "truth_v_m_s", "truth_yaw_rad_s",
    "truth_x_m", "truth_y_m", "truth_yaw_rad",
)


def time_label():
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def read_settings():
    saved_yaml = read_lab_yaml("mobility")
    if saved_yaml is not None:
        return validate_settings(saved_yaml)
    if not SETTINGS_PATH.exists():
        return dict(DEFAULTS)
    try:
        return validate_settings(json.loads(SETTINGS_PATH.read_text()))
    except (ValueError, OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Invalid {SETTINGS_PATH}: {exc}") from exc


def write_settings(settings):
    OUTPUT.mkdir(parents=True, exist_ok=True)
    tmp = SETTINGS_PATH.with_suffix(".tmp")
    tmp.write_text(json.dumps(settings, indent=2, sort_keys=True) + "\n")
    tmp.replace(SETTINGS_PATH)


def make_csv(rows):
    dest = io.StringIO()
    w = csv.DictWriter(dest, fieldnames=CSV_FIELDS, extrasaction="ignore")
    w.writeheader()
    w.writerows(rows)
    return dest.getvalue()


class Lab:
    def __init__(self, attach=False):
        from ros_adapter import RosBridge  # Import only after ROS env is sourced.
        self.lock = threading.RLock()
        OUTPUT.mkdir(parents=True, exist_ok=True)
        self.settings = read_settings()
        self.bridge = RosBridge(self.settings)
        self.attach = bool(attach)
        self.proc = None
        self.log = None
        self.reference_step = []
        self.reference_stop = []
        self.reference_source = None
        self.reference_manifest = {}
        self.applied_physics = None
        self.restarting = False
        self.error = None
        self.load_reference()
        if not attach:
            self.restart()
        else:
            self.applied_physics = {k: self.settings[k] for k in PHYSICS}

    def load_reference(self):
        # Bundled files are two separate 90 RPM floor-loaded validations.
        # Uploading a custom CSV overrides the built-in reference, never
        # modifies either original source log in the repository.
        self.reference_step, self.reference_stop, self.reference_manifest = (
            bundled_validation()
        )
        self.reference_source = "builtin"
        p = OUTPUT / "real_reference/matlab_reference.csv"
        if p.is_file():
            try:
                self.reference_step = reference_csv(p.read_text())
                self.reference_stop = []
                self.reference_source = "uploaded"
            except (ValueError, OSError) as exc:
                self.error = f"Ignoring malformed reference CSV: {exc}"

    def stop_sim(self):
        if self.proc is None:
            return
        p = self.proc
        self.proc = None
        if p.poll() is None:
            try:
                os.killpg(p.pid, signal.SIGINT)
                p.wait(timeout=5.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(p.pid, signal.SIGTERM)
                    p.wait(timeout=2.0)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(p.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
        if self.log:
            self.log.close()
            self.log = None

    def restart(self):
        if self.attach:
            raise RuntimeError("Cannot restart externally managed Gazebo")
        with self.lock:
            if self.restarting:
                raise RuntimeError("Already restarting")
            self.restarting = True
            cfg = dict(self.settings)
        try:
            self.bridge.node.reset_for_restart()
            self.stop_sim()
            if not shutil_which("ros2"):
                raise RuntimeError("Cannot find 'ros2'; source ROS 2 Jazzy and the SCROBOT install.")
            self.log = (OUTPUT / "gazebo.log").open("ab")
            cmd = ["ros2", "launch", "scrobot_simulation", "mobility_lab.launch.py"]
            cmd += [f"{key}:={cfg[key]}" for key in PHYSICS]
            self.proc = subprocess.Popen(
                cmd, stdout=self.log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            time.sleep(0.1)
            if self.proc.poll() is not None:
                raise RuntimeError("Gazebo process exited at startup; see evaluation_results/mobility_lab/gazebo.log")
            self.applied_physics = {k: cfg[k] for k in PHYSICS}
            self.error = None
        except Exception as exc:
            self.error = str(exc)
            raise
        finally:
            with self.lock:
                self.restarting = False

    def physics_dirty(self):
        return (
            self.applied_physics is None
            or any(self.applied_physics[k] != self.settings[k] for k in PHYSICS)
        )

    def change_settings(self, change):
        with self.lock:
            if self.bridge.node.stage in ("pre", "hold", "post"):
                raise RuntimeError("Stop the test before changing settings")
            self.settings = validate_settings({**self.settings, **change})
            write_settings(self.settings)
            return self.settings

    def save_lab_yaml(self):
        """One deliberate save action; interactive settings remain JSON drafts."""
        with self.lock:
            return {"saved": str(save_lab_yaml(
                "mobility", self.settings, validator=validate_settings))}

    def publish(self):
        # Publish only explicitly saved YAML, not the JSON working draft.
        # No Gazebo/ROS state is modified until the normal sim restarts.
        with self.lock:
            if self.bridge.node.stage in ("pre", "hold", "post"):
                raise RuntimeError("Stop the wheel experiment before publishing")
            current_saved = read_lab_yaml("mobility")
            if current_saved is None:
                raise RuntimeError("Click Save Mobility Lab YAML before publishing")
            clean = validate_settings(current_saved)
            return publish_saved_calibration(lab_yaml_path("mobility"))

    def run(self):
        with self.lock:
            if self.restarting:
                raise RuntimeError("Gazebo is restarting")
            if self.physics_dirty():
                raise RuntimeError("Physics changed. Click Restart Gazebo before testing.")
            if not self.attach and (self.proc is None or self.proc.poll() is not None):
                raise RuntimeError("Gazebo is not running. Click Restart Gazebo.")
            return self.bridge.node.start(self.settings)

    def status(self):
        with self.lock:
            data = self.bridge.node.status(rows=800)
            result = self.bridge.node.result()
            settings = result["settings"] if result else self.settings
            if result:
                rmse, reference = compare_validation(
                    result["samples"], settings,
                    self.reference_step, self.reference_stop
                )
            else:
                rmse = None
                reference = phase_aligned_reference(
                    settings, self.reference_step, self.reference_stop
                )
            data.update({
                "physics_dirty": self.physics_dirty(),
                "gazebo_alive": self.attach or (self.proc is not None and self.proc.poll() is None),
                "gazebo_managed": not self.attach,
                "restarting": self.restarting,
                "settings": self.settings,
                "result_settings": settings,
                "reference": reference[::max(1, len(reference)//450)],
                "reference_count": len(self.reference_step),
                "reference_source": self.reference_source,
                "reference_label": status_label(settings, self.reference_source),
                "real_source_count": 903 if self.reference_source == "builtin" else None,
                "reference_rmse": rmse,
                "publication": published_status(
                    read_lab_yaml("mobility") or self.settings),
                "lab_yaml": {
                    "path": str(lab_yaml_path("mobility")),
                    "saved": read_lab_yaml("mobility") is not None,
                    "matches_live": read_lab_yaml("mobility") == self.settings,
                },
                "server_error": self.error,
            })
            return data

    def upload_reference(self, csv_text):
        rows = reference_csv(csv_text)
        with self.lock:
            root = OUTPUT / "real_reference"
            root.mkdir(parents=True, exist_ok=True)
            (root / "matlab_reference.csv").write_text(csv_text)
            self.reference_step = rows
            self.reference_stop = []
            self.reference_source = "uploaded"
        return {"rows": len(rows), "path": str(root / "matlab_reference.csv")}

    def reset_reference(self):
        with self.lock:
            p = OUTPUT / "real_reference/matlab_reference.csv"
            if p.exists():
                p.unlink()
            self.reference_step, self.reference_stop, self.reference_manifest = (
                bundled_validation()
            )
            self.reference_source = "builtin"
        return {"reference": "WL/WR floor-loaded September 22 90 RPM",
                "step_samples": len(self.reference_step),
                "stop_samples": len(self.reference_stop)}

    def save_result(self):
        result = self.bridge.node.result()
        if not result:
            raise RuntimeError("No completed experiment to save")
        folder = OUTPUT / "recordings" / (
            time_label() + "_" + result["settings"]["profile"]
        )
        suffix = 1
        original = folder
        while folder.exists():
            folder = Path(str(original) + "_" + str(suffix))
            suffix += 1
        folder.mkdir(parents=True)
        (folder / "samples.csv").write_text(make_csv(result["samples"]))
        result["reference_rmse"], _ = compare_validation(
            result["samples"], result["settings"],
            self.reference_step, self.reference_stop
        )
        result["reference_source"] = self.reference_source
        if self.reference_source == "builtin":
            result["reference_manifest"] = self.reference_manifest
        (folder / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        return {"saved": str(folder), "sample_count": len(result["samples"])}

    def close(self):
        self.bridge.node.abort()
        self.stop_sim()
        self.bridge.close()


def shutil_which(command):
    import shutil
    return shutil.which(command)


class Handler(BaseHTTPRequestHandler):
    server_version = "ScrobotMobilityLab/1.0"

    @property
    def lab(self):
        return self.server.lab

    def respond(self, code, payload, content_type="application/json"):
        data = payload if isinstance(payload, bytes) else (
            payload.encode("utf8") if isinstance(payload, str)
            else json.dumps(payload, allow_nan=False).encode("utf8")
        )
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def request_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length < 1 or length > 4_100_000:
            raise ValueError("Invalid request size")
        return json.loads(self.rfile.read(length).decode("utf8"))

    def do_GET(self):
        route = urlsplit(self.path).path
        try:
            if route == "/":
                return self.respond(200, (HERE / "index.html").read_bytes(), "text/html; charset=utf-8")
            if route == "/api/status":
                return self.respond(200, self.lab.status())
            if route == "/api/result":
                return self.respond(200, self.lab.bridge.node.result() or {})
            if route == "/api/last.csv":
                result = self.lab.bridge.node.result()
                if not result:
                    raise RuntimeError("No completed experiment")
                return self.respond(200, make_csv(result["samples"]), "text/csv; charset=utf-8")
            return self.respond(404, {"error": "Unknown endpoint"})
        except (RuntimeError, ValueError, OSError) as exc:
            return self.respond(400, {"error": str(exc)})

    def do_POST(self):
        route = urlsplit(self.path).path
        try:
            body = self.request_json()
            if route == "/api/settings":
                if not isinstance(body, dict):
                    raise ValueError("Settings body must be a JSON object")
                result = self.lab.change_settings(body)
            elif route == "/api/run":
                result = self.lab.run()
            elif route == "/api/abort":
                self.lab.bridge.node.abort()
                result = {"stage": "aborted"}
            elif route == "/api/restart":
                self.lab.restart()
                result = {"started": True}
            elif route == "/api/reference":
                result = self.lab.upload_reference(body.get("csv", ""))
            elif route == "/api/reference/reset":
                result = self.lab.reset_reference()
            elif route == "/api/config/save":
                if body != {}:
                    raise ValueError("Save Lab YAML uses current validated settings")
                result = self.lab.save_lab_yaml()
            elif route == "/api/publish":
                if body.get("confirm") is not True:
                    raise ValueError("Publishing requires explicit confirmation")
                result = self.lab.publish()
            elif route == "/api/save":
                result = self.lab.save_result()
            else:
                return self.respond(404, {"error": "Unknown endpoint"})
            return self.respond(200, result)
        except (RuntimeError, ValueError, OSError, KeyError, TypeError) as exc:
            return self.respond(400, {"error": str(exc)})

    def log_message(self, fmt, *args):
        if args and not str(args[1] if len(args) > 1 else "").startswith("200"):
            super().log_message(fmt, *args)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--attach", action="store_true",
                        help="Connect to already-running isolated Mobility Lab Gazebo")
    parser.add_argument("--ros-domain-id", type=int, default=77,
                        help="Isolated ROS_DOMAIN_ID (default 77; avoid the real robot's domain)")
    args = parser.parse_args()
    if not 0 <= args.ros_domain_id <= 232:
        parser.error("--ros-domain-id must be in [0,232]")
    # Both the browser's ROS2 node and its launched Gazebo/bridge processes
    # must inherit these BEFORE rclpy.init(). Never default to production ROS
    # domain 0 when the test directly commands drive wheel velocity.
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    os.environ.setdefault("GZ_PARTITION",
                          f"scrobot_mobility_{os.getuid()}_{args.ros_domain_id}")
    lab = Lab(attach=args.attach)
    httpd = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    httpd.lab = lab
    url = f"http://127.0.0.1:{args.port}"
    print(f"Mobility Lab: {url}")
    print(f"ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}  GZ_PARTITION={os.environ['GZ_PARTITION']}")
    print(f"Recording: {OUTPUT}")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        httpd.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
        lab.close()


if __name__ == "__main__":
    main()
