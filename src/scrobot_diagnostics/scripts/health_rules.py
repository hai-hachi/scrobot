"""Pure, ROS-independent SCROBOT health classification rules.

This module is intentionally importable without rclpy so that fault rules can
be unit-tested without Gazebo or ROS. Monitoring is read-only: no recovery
command is ever produced by these functions.
"""

from dataclasses import dataclass

OK = 0
WARN = 1
ERROR = 2


@dataclass(frozen=True)
class Finding:
    component: str
    level: int
    code: str
    message: str
    detail: str = ""


STATE_LIMITS_S = {
    "INITIAL_TAG_APPROACH": 90.0,
    "INITIAL_RELOCALIZATION": 35.0,
    "STARTING_NAV2": 35.0,
    "JOIN_SWEEP": 80.0,
    "TURN_TO_TAG": 45.0,
    "RELOCALIZING": 45.0,
    "RESTORE_SWEEP_HEADING": 50.0,
    "LOCAL_COLLECT": 140.0,
    "RETURN_TO_SWEEP": 80.0,
    "RETURN_TO_IDLE": 80.0,
}

MOVING_STATES = {
    "JOIN_SWEEP", "SWEEPING", "RETURN_TO_SWEEP", "RETURN_TO_IDLE",
}

GLOBAL_TF_STATES = {
    "JOIN_SWEEP", "SWEEPING", "TURN_TO_TAG", "RELOCALIZING",
    "RESTORE_SWEEP_HEADING", "LOCAL_COLLECT", "RETURN_TO_SWEEP",
    "RETURN_TO_IDLE",
}

# Underlying blockers should take precedence over their secondary symptoms.
PRIORITY = (
    "CLOCK_STALLED",
    "ROBOT_DESCRIPTION_MISSING",
    "MISSION_ERROR",
    "LOCAL_TF_MISSING",
    "GLOBAL_TF_MISSING",
    "ODOMETRY_STALE",
    "DEPTH_SCAN_STALE",
    "COLLISION_STOP",
    "FINAL_COMMAND_SUPPRESSED",
    "AUTO_COMMAND_SUPPRESSED",
    "MOTION_STALLED",
    "MISSION_NO_COMMAND",
    "MISSION_STATE_TIMEOUT",
    "RGB_STALE",
    "DEPTH_STALE",
    "YOLO_STALE",
    "MISSION_NOT_STARTED",
)

ACTIONS = {
    "CLOCK_STALLED": "Check Gazebo pause state and the /clock bridge.",
    "ROBOT_DESCRIPTION_MISSING": "Check xacro and robot_state_publisher; inspect /robot_description QoS.",
    "MISSION_ERROR": "Open /mission/failure_reason and the latest /debug/incident_event.",
    "LOCAL_TF_MISSING": "Check EKF, /odometry/filtered and odom -> base_footprint TF.",
    "GLOBAL_TF_MISSING": "Check AprilTag relocalization and map -> odom TF.",
    "ODOMETRY_STALE": "Check wheel odometry, IMU input, and the EKF.",
    "DEPTH_SCAN_STALE": "Check depth PointCloud2, scan conversion and self-filter.",
    "COLLISION_STOP": "Inspect /collision_monitor_state, stop polygon and depth scan.",
    "FINAL_COMMAND_SUPPRESSED": "Compare /cmd_vel_smoothed with /diff_drive_controller/cmd_vel and collision monitor.",
    "AUTO_COMMAND_SUPPRESSED": "Check control-mode mux and AUTO/MANUAL selection.",
    "MOTION_STALLED": "Final velocity is nonzero but odometry is stationary; check controller and Gazebo physics.",
    "MISSION_NO_COMMAND": "Mission expects travel but command pipeline is idle; inspect Nav2 action/lifecycle.",
    "MISSION_STATE_TIMEOUT": "Inspect the pending ROS action and corresponding server logs.",
    "RGB_STALE": "Check RGB CameraInfo, image bridge and YOLO freshness.",
    "DEPTH_STALE": "Check depth CameraInfo and the depth sensor/bridge.",
    "YOLO_STALE": "Check YOLO model, image/depth synchronization and detector process.",
    "MISSION_NOT_STARTED": "Check whether the mission launch and node are running.",
}


def _stream(obs, component, key, deadline, code):
    age = obs.get(key)
    if not obs.get("ready", False):
        return Finding(component, OK, "STARTING", "Startup grace period.")
    if age is None:
        return Finding(component, WARN, code, f"No {component.lower()} messages received.")
    if age > deadline:
        return Finding(component, WARN, code, f"{component} silent for {age:.1f} s.")
    return Finding(component, OK, "OK", f"Updated {age:.1f} s ago.")


def evaluate(obs):
    """Return a stable status for each monitored subsystem.

    Keys ending in _age are monotonic receive ages in seconds (None if never
    seen). *_for fields represent *continuous* durations, not the age of the
    most recent message. The caller is responsible for computing those timers.
    """
    ready = bool(obs.get("ready", False))
    state = obs.get("mission_state") or "UNKNOWN"
    mode = obs.get("control_mode") or "UNKNOWN"

    findings = []

    clock = _stream(obs, "Clock", "clock_age", 2.0, "CLOCK_STALLED")
    if clock.level == WARN:
        clock = Finding(clock.component, ERROR, clock.code, clock.message)
    findings.append(clock)

    if not ready:
        findings.append(Finding("Robot description", OK, "STARTING", "Startup grace period."))
    elif obs.get("robot_description_seen", False):
        findings.append(Finding("Robot description", OK, "OK", "/robot_description received."))
    else:
        findings.append(Finding(
            "Robot description", ERROR, "ROBOT_DESCRIPTION_MISSING",
            "No /robot_description (transient-local) received."
        ))

    findings.append(_stream(obs, "RGB", "rgb_age", 3.0, "RGB_STALE"))
    findings.append(_stream(obs, "Depth", "depth_age", 3.0, "DEPTH_STALE"))
    if obs.get("expect_scan", True):
        findings.append(_stream(obs, "Depth scan", "scan_age", 3.0, "DEPTH_SCAN_STALE"))
    else:
        findings.append(Finding(
            "Depth scan", OK, "NOT_REQUIRED", "Scan pipeline is optional in this launch."
        ))

    if obs.get("expect_localization", True):
        findings.append(_stream(obs, "Odometry", "odom_age", 2.0, "ODOMETRY_STALE"))
    else:
        findings.append(Finding(
            "Odometry", OK, "NOT_REQUIRED", "Localization is optional in this launch."
        ))

    if not obs.get("expect_localization", True):
        findings.append(Finding("TF", OK, "NOT_REQUIRED", "EKF/TF is optional in this launch."))
    elif not ready:
        findings.append(Finding("TF", OK, "STARTING", "Startup grace period."))
    elif obs.get("tf_local") is False:
        findings.append(Finding(
            "TF", ERROR, "LOCAL_TF_MISSING", "Missing odom -> base_footprint transform."
        ))
    elif (
        state in GLOBAL_TF_STATES
        and obs.get("tf_global") is False
    ):
        findings.append(Finding(
            "TF", ERROR, "GLOBAL_TF_MISSING", "Missing map -> odom transform."
        ))
    else:
        findings.append(Finding("TF", OK, "OK", "Required TF chains available."))

    if not ready:
        mission = Finding("Mission", OK, "STARTING", "Startup grace period.")
    elif state == "ERROR":
        reason = obs.get("failure_reason") or "See mission logs."
        mission = Finding("Mission", ERROR, "MISSION_ERROR", reason)
    elif state == "UNKNOWN" and obs.get("expect_mission", False):
        mission = Finding(
            "Mission", WARN, "MISSION_NOT_STARTED", "No /mission/state received."
        )
    elif state in STATE_LIMITS_S and (obs.get("state_age") or 0) > STATE_LIMITS_S[state]:
        mission = Finding(
            "Mission", WARN, "MISSION_STATE_TIMEOUT",
            f"{state} has lasted {obs['state_age']:.1f} s; inspect pending action."
        )
    else:
        mission = Finding("Mission", OK, "OK", f"Mission state: {state}.")
    findings.append(mission)

    if not obs.get("expect_perception", False):
        findings.append(Finding(
            "YOLO", OK, "NOT_REQUIRED", "Perception is optional in this launch."
        ))
    else:
        findings.append(_stream(obs, "YOLO", "yolo_age", 3.0, "YOLO_STALE"))

    collision = obs.get("collision_action")
    polygon = obs.get("collision_polygon") or "unnamed"
    if not ready:
        safety = Finding("Collision monitor", OK, "STARTING", "Startup grace period.")
    elif collision == "STOP":
        safety = Finding(
            "Collision monitor", WARN, "COLLISION_STOP",
            f"Nav2 collision monitor STOP: {polygon}."
        )
    elif collision == "SLOWDOWN":
        safety = Finding(
            "Collision monitor", WARN, "COLLISION_SLOWDOWN",
            f"Robot deliberately slowed by polygon {polygon}."
        )
    else:
        safety = Finding(
            "Collision monitor", OK, "OK",
            f"Last reported action: {collision or 'not observed'}."
        )
    findings.append(safety)

    if not ready:
        gate = Finding("Command gate", OK, "STARTING", "Startup grace period.")
    elif (obs.get("blocked_for") or 0) >= 1.0:
        reason = (
            f"Collision monitor STOP ({polygon}) blocks the final command."
            if collision == "STOP" else
            "Smoothed velocity is nonzero but final drive velocity is zero."
        )
        gate = Finding(
            "Command gate", WARN,
            "COLLISION_STOP" if collision == "STOP" else "FINAL_COMMAND_SUPPRESSED",
            reason,
        )
    elif mode != "MANUAL" and (obs.get("auto_blocked_for") or 0) >= 1.0:
        gate = Finding(
            "Command gate", WARN, "AUTO_COMMAND_SUPPRESSED",
            "AUTO velocity is nonzero but selected velocity is zero."
        )
    else:
        gate = Finding(
            "Command gate", OK, "OK",
            "No unexplained downstream velocity suppression."
        )
    findings.append(gate)

    if not ready:
        progress = Finding("Drive progress", OK, "STARTING", "Startup grace period.")
    elif (obs.get("stall_for") or 0) >= 4.0:
        progress = Finding(
            "Drive progress", WARN, "MOTION_STALLED",
            f"Drive commanded for {obs['stall_for']:.1f} s, odometry shows no movement."
        )
    elif (
        state in MOVING_STATES
        and mode != "MANUAL"
        and (obs.get("no_command_for") or 0) >= 15.0
    ):
        progress = Finding(
            "Drive progress", WARN, "MISSION_NO_COMMAND",
            f"{state}: no travel command for {obs['no_command_for']:.1f} s."
        )
    else:
        progress = Finding("Drive progress", OK, "OK", "No movement watchdog alarm.")
    findings.append(progress)

    return findings


def primary_finding(findings):
    """Pick the best explanation, avoiding secondary consequences where possible."""
    problems = [f for f in findings if f.level > OK]
    if not problems:
        return Finding("System", OK, "OK", "All monitored signals healthy.")

    def rank(f):
        try:
            priority = PRIORITY.index(f.code)
        except ValueError:
            priority = len(PRIORITY)
        return (-f.level, priority)

    return sorted(problems, key=rank)[0]
