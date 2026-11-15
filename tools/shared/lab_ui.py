"""Shared, opt-in Gazebo GUI client and bounded local log tails for developer labs.

Does not start Gazebo's server. Closing the GUI leaves the headless simulation
running. The owner is responsible for closing this helper at lab shutdown.
"""
from __future__ import annotations

import os
from pathlib import Path
import shutil
import signal
import subprocess


def log_tail(path, max_bytes=6000):
    if not path:
        return ""
    path = Path(path)
    try:
        with path.open("rb") as stream:
            stream.seek(0, 2)
            stream.seek(max(0, stream.tell() - max_bytes))
            return stream.read().decode("utf-8", errors="replace")
    except OSError:
        return ""


class GazeboGUI:
    def __init__(self, log_path, env=None, cwd=None):
        self.log_path = Path(log_path)
        self.env = env
        self.cwd = cwd
        self.proc = None
        self.log_handle = None

    @property
    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def status(self):
        return {"running": self.running,
                "exit_code": self.proc.poll() if self.proc is not None else None,
                "display_available": bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))}

    def open(self):
        if self.running:
            return "Gazebo GUI already open"
        if not self.status()["display_available"]:
            raise RuntimeError("No graphical desktop (DISPLAY/WAYLAND_DISPLAY is unset)")
        if not shutil.which("gz"):
            raise RuntimeError("'gz' executable not found; source Gazebo Harmonic")
        self.close()
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        handle = self.log_path.open("ab")
        try:
            proc = subprocess.Popen(
                ["gz", "sim", "-g", "-v", "3"],
                stdout=handle, stderr=subprocess.STDOUT, start_new_session=True,
                cwd=self.cwd, env=self.env if self.env is not None else os.environ.copy(),
            )
        except Exception:
            handle.close()
            raise
        self.proc, self.log_handle = proc, handle
        return "Gazebo GUI client opened (simulation server unchanged)"

    def close(self):
        if self.running:
            for sig, timeout in ((signal.SIGINT, 3), (signal.SIGTERM, 2)):
                if not self.running:
                    break
                try:
                    os.killpg(self.proc.pid, sig)
                    self.proc.wait(timeout=timeout)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    continue
            if self.running:
                try:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                    self.proc.wait(timeout=2)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    pass
        if self.log_handle:
            self.log_handle.close()
            self.log_handle = None
        self.proc = None
        return "Gazebo GUI closed (headless simulation unchanged)"
