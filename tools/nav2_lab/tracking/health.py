"""Fail-fast detection of fatal Tracker Nav2 controller startup errors.

Gazebo can remain alive when the controller_server subprocess crashes;
checking only the top-level ros2 launch process is insufficient.
"""
from pathlib import Path
import re


def controller_failure(log_path):
    if not log_path:
        return None
    path = Path(log_path)
    try:
        with path.open("rb") as source:
            source.seek(0, 2)
            size = source.tell()
            source.seek(max(0, size - 524288))
            text = source.read().decode("utf-8", "replace")
    except (OSError, ValueError):
        return None
    crashed = re.search(r"\[ERROR\] \[controller_server-\d+\]: process has died", text)
    if not crashed:
        return None
    reason = re.findall(r"what\(\):\s*([^\r\n]+)", text[:crashed.start()])
    if reason:
        detail = reason[-1].strip()
    else:
        detail = "ROS controller_server process terminated"
    return ("Nav2 controller_server crashed: " + detail +
            ". Stop and restart the Tracker simulation after correcting the "
            "configuration; Gazebo wheel controllers alone cannot follow a path.")
