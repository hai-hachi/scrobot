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
    # A Nav2 lifecycle configure failure may leave controller_server alive
    # but forever UNCONFIGURED. The manager stops trying after this message,
    # which previously made the Tracker hang at Initializing for 160 seconds.
    aborted = re.search(
        r"Failed to bring up all requested nodes\. Aborting bringup\.", text)
    crashed = re.search(
        r"\[ERROR\] \[controller_server-\d+\]: process has died", text)
    if not aborted and not crashed:
        return None
    # Jazzy reports plugin parameter exceptions as "Original error:" in the
    # controller_server lifecycle callback; subprocess aborts use "what():".
    # Keep the actual reason, rather than a generic timeout or stale log tail.
    match_end = max(m.start() for m in (aborted, crashed) if m)
    reason = re.findall(
        r"(?:Original error:|what\(\):)\s*([^\r\n]+)",
        text[:match_end])
    detail = reason[-1].strip() if reason else (
        "Nav2 lifecycle manager could not configure controller_server"
        if aborted else "ROS controller_server process terminated")
    failure = ("failed lifecycle configuration" if aborted and not crashed
               else "crashed")
    return ("Nav2 controller_server " + failure + ": " + detail +
            ". Stop and restart the Tracker simulation after correcting the "
            "configuration; Gazebo wheel controllers alone cannot follow a path.")


def ready_for_tracking(sim_clock_s, ground_truth_samples, wheel_odom_samples,
                       action_server_available, controller_lifecycle_active):
    """Require actual Nav2 lifecycle ACTIVE, not merely an advertised action.

    Jazzy creates the FollowPath endpoint before the lifecycle manager
    transitions controller_server to ACTIVE. A goal sent during that gap
    is rejected even when server_is_ready() already returned True.
    """
    return (sim_clock_s > .1 and ground_truth_samples >= 4 and
            wheel_odom_samples >= 4 and bool(action_server_available) and
            bool(controller_lifecycle_active))
