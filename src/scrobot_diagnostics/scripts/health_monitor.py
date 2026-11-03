#!/usr/bin/env python3
"""Read-only SCROBOT health monitor and incident snapshot trigger.

The node never publishes motion commands, cancels actions, or changes lifecycle
states. All freshness and watchdog timers use a steady clock, so Gazebo /clock
pauses are themselves diagnosable.
"""

from __future__ import annotations

import json
import time
from datetime import datetime, timezone

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseArray, TwistStamped
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from rclpy.time import Time
from rosbag2_interfaces.srv import Snapshot
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray

from health_rules import ACTIONS, MOVING_STATES, evaluate, primary_finding


SEVERITY_TEXT = {0: "OK", 1: "WARN", 2: "ERROR"}
AUTO_SNAPSHOT_CODES = {
    "MISSION_ERROR",
    "MOTION_STALLED",
    "FINAL_COMMAND_SUPPRESSED",
    "COLLISION_STOP",
    "CLOCK_STALLED",
    "ROBOT_DESCRIPTION_MISSING",
}

COMMAND_TOPICS = {
    "nav": "/cmd_vel_nav",
    "auto": "/cmd_vel_auto",
    "selected": "/cmd_vel_selected",
    "smoothed": "/cmd_vel_smoothed",
    "drive": "/diff_drive_controller/cmd_vel",
}


def is_active(command, linear_min=0.05, angular_min=0.08):
    return command is not None and (
        abs(command[0]) > linear_min or abs(command[1]) > angular_min
    )


def continuous_duration(previous, condition, now):
    """Compute continuous condition duration from monotonic timestamps."""
    if not condition:
        return None, 0.0
    start = previous if previous is not None else now
    return start, now - start


class HealthMonitor(Node):
    def __init__(self):
        super().__init__("scrobot_health_monitor")

        self.declare_parameter("expect_clock", True)
        self.declare_parameter("expect_mission", True)
        self.declare_parameter("expect_perception", True)
        self.declare_parameter("expect_localization", True)
        self.declare_parameter("expect_scan", True)
        self.declare_parameter("startup_grace_sec", 25.0)
        self.declare_parameter("report_rate", 1.0)
        self.declare_parameter("auto_snapshot", False)
        self.declare_parameter("snapshot_cooldown_sec", 90.0)

        self.expect_clock = bool(self.get_parameter("expect_clock").value)
        self.expect_mission = bool(self.get_parameter("expect_mission").value)
        self.expect_perception = bool(self.get_parameter("expect_perception").value)
        self.expect_localization = bool(self.get_parameter("expect_localization").value)
        self.expect_scan = bool(self.get_parameter("expect_scan").value)
        self.startup_grace = max(
            0.0, float(self.get_parameter("startup_grace_sec").value)
        )
        self.report_rate = max(0.2, float(self.get_parameter("report_rate").value))
        self.auto_snapshot = bool(self.get_parameter("auto_snapshot").value)
        self.snapshot_cooldown = max(
            0.0, float(self.get_parameter("snapshot_cooldown_sec").value)
        )

        self.started = time.monotonic()
        self.seen = {}
        self.commands = {}
        self.robot_description_seen = False
        self.mission_state = "UNKNOWN"
        self.phase = "UNKNOWN"
        self.control_mode = "UNKNOWN"
        self.failure_reason = ""
        self.mission_state_since = self.started
        self.odom_velocity = None
        self.collision_action = None
        self.collision_polygon = ""
        self.shuttles_remaining = None

        self.blocked_since = None
        self.auto_blocked_since = None
        self.stall_since = None
        self.no_command_since = None
        self.last_primary_code = None
        self.snapshot_armed_for = None
        self.snapshot_last_at = -float("inf")
        self.snapshot_inflight = False

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        event_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", reliable_qos
        )
        self.summary_pub = self.create_publisher(
            String, "/debug/health_summary", state_qos
        )
        self.event_pub = self.create_publisher(
            DiagnosticStatus, "/debug/incident_events", event_qos
        )

        self.create_subscription(
            ClockMessage, "/clock", lambda _m: self._touch("clock"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String, "/robot_description", self._robot_description_cb, state_qos
        )
        # CameraInfo is tiny compared with raw RGB (~2.8 MB/frame).
        # Sample sensor metadata here; YOLO freshness checks the RGB pipeline.
        self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info",
            lambda _m: self._touch("rgb"), qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, "/camera/camera/depth/camera_info",
            lambda _m: self._touch("depth"), qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan, "/camera/camera/depth/scan",
            lambda _m: self._touch("scan"), qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, "/odometry/filtered",
            self._odom_cb, qos_profile_sensor_data,
        )
        self.create_subscription(
            Detection3DArray, "/perception/shuttle_detections_3d",
            lambda _m: self._touch("yolo"), qos_profile_sensor_data,
        )
        self.create_subscription(
            String, "/mission/state", self._mission_cb, state_qos
        )
        self.create_subscription(
            String, "/mission/failure_reason", self._failure_cb, state_qos
        )
        self.create_subscription(
            String, "/mission/local_collect_phase", self._phase_cb, reliable_qos
        )
        self.create_subscription(
            String, "/control/mode", self._control_mode_cb, state_qos
        )
        self.create_subscription(
            CollisionMonitorState, "/collision_monitor_state",
            self._collision_cb, reliable_qos,
        )
        self.create_subscription(
            PoseArray, "/evaluation/shuttle_ground_truth",
            self._shuttles_cb, qos_profile_sensor_data,
        )

        for key, topic in COMMAND_TOPICS.items():
            self.create_subscription(
                TwistStamped, topic,
                lambda msg, channel=key: self._command_cb(channel, msg),
                qos_profile_sensor_data,
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.snapshot_client = self.create_client(
            Snapshot, "/rosbag2_recorder/snapshot"
        ) if self.auto_snapshot else None

        # Steady timer is critical. A ROS-time timer would stop reporting
        # precisely when Gazebo /clock freezes.
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / self.report_rate, self._tick, clock=steady_clock)
        self.get_logger().info(
            "SCROBOT health monitor ready: /diagnostics, /debug/health_summary, "
            "/debug/incident_events; all watchdogs use monotonic time."
        )

    def _touch(self, key):
        self.seen[key] = time.monotonic()

    def _age(self, key, now):
        stamp = self.seen.get(key)
        return None if stamp is None else max(0.0, now - stamp)

    def _robot_description_cb(self, msg):
        if msg.data.strip():
            self.robot_description_seen = True

    def _odom_cb(self, msg):
        self._touch("odom")
        twist = msg.twist.twist
        self.odom_velocity = (float(twist.linear.x), float(twist.angular.z))

    def _shuttles_cb(self, msg):
        self.shuttles_remaining = len(msg.poses)

    def _command_cb(self, key, msg):
        self.commands[key] = (
            time.monotonic(),
            float(msg.twist.linear.x),
            float(msg.twist.angular.z),
        )

    def _recent_command(self, key, now, max_age=1.0):
        datum = self.commands.get(key)
        if datum is None or now - datum[0] > max_age:
            return None
        return (datum[1], datum[2])

    def _mission_cb(self, msg):
        new_state = msg.data.strip() or "UNKNOWN"
        if new_state == self.mission_state:
            return
        old = self.mission_state
        self.mission_state = new_state
        self.mission_state_since = time.monotonic()
        self._event(
            "MISSION_TRANSITION", "Mission",
            f"{old} -> {new_state}",
            level=DiagnosticStatus.ERROR if new_state == "ERROR" else
            DiagnosticStatus.WARN if new_state == "PAUSED" else
            DiagnosticStatus.OK,
            values={"from_state": old, "to_state": new_state},
        )

    def _failure_cb(self, msg):
        reason = msg.data.strip()
        if reason and reason != self.failure_reason:
            self._event(
                "MISSION_FAILURE", "Mission", reason,
                level=DiagnosticStatus.ERROR,
                values={"mission_state": self.mission_state},
            )
        self.failure_reason = reason

    def _phase_cb(self, msg):
        phase = msg.data.strip() or "UNKNOWN"
        if phase == self.phase:
            return
        previous = self.phase
        self.phase = phase
        self._event(
            "COLLECT_PHASE", "Local collect", f"{previous} -> {phase}",
            values={"from_phase": previous, "to_phase": phase},
        )

    def _control_mode_cb(self, msg):
        mode = msg.data.strip() or "UNKNOWN"
        if mode == self.control_mode:
            return
        previous = self.control_mode
        self.control_mode = mode
        self._event(
            "CONTROL_MODE", "Control", f"{previous} -> {mode}",
            level=DiagnosticStatus.WARN if mode == "MANUAL" else DiagnosticStatus.OK,
        )

    def _collision_cb(self, msg):
        name = {
            CollisionMonitorState.DO_NOTHING: "DO_NOTHING",
            CollisionMonitorState.STOP: "STOP",
            CollisionMonitorState.SLOWDOWN: "SLOWDOWN",
            CollisionMonitorState.APPROACH: "APPROACH",
            CollisionMonitorState.LIMIT: "LIMIT",
        }.get(msg.action_type, f"UNKNOWN({msg.action_type})")
        polygon = msg.polygon_name
        if (name, polygon) != (self.collision_action, self.collision_polygon):
            self.collision_action = name
            self.collision_polygon = polygon
            self._event(
                "SAFETY_ACTION", "Collision monitor", f"{name}: {polygon or '-'}",
                level=DiagnosticStatus.WARN if name in (
                    "STOP", "SLOWDOWN", "LIMIT"
                ) else DiagnosticStatus.OK,
                values={"polygon": polygon, "action": name},
            )

    def _event(self, code, component, message, level=DiagnosticStatus.OK, values=None):
        event = DiagnosticStatus()
        event.level = int(level)
        event.name = f"SCROBOT/{component}"
        event.hardware_id = "scrobot"
        event.message = message
        data = {
            "code": code,
            "utc": datetime.now(timezone.utc).isoformat(),
        }
        data.update(values or {})
        event.values = [KeyValue(key=k, value=str(v)) for k, v in data.items()]
        self.event_pub.publish(event)
        if level == DiagnosticStatus.ERROR:
            self.get_logger().error(f"[{code}] {message}")
        elif level == DiagnosticStatus.WARN:
            self.get_logger().warn(f"[{code}] {message}")
        else:
            self.get_logger().info(f"[{code}] {message}")

    def _tf_ok(self, target_frame, source_frame):
        try:
            return bool(self.tf_buffer.can_transform(target_frame, source_frame, Time()))
        except Exception:
            return False

    def _observation(self, now):
        selected = self._recent_command("selected", now)
        automatic = self._recent_command("auto", now)
        smoothed = self._recent_command("smoothed", now)
        final_drive = self._recent_command("drive", now)

        self.blocked_since, blocked_for = continuous_duration(
            self.blocked_since,
            is_active(smoothed) and not is_active(final_drive),
            now,
        )
        self.auto_blocked_since, auto_blocked_for = continuous_duration(
            self.auto_blocked_since,
            self.control_mode != "MANUAL"
            and is_active(automatic) and not is_active(selected),
            now,
        )

        velocity = self.odom_velocity if self._age("odom", now) is not None and (
            self._age("odom", now) <= 2.0
        ) else None
        stationary = velocity is not None and (
            abs(velocity[0]) < 0.025 and abs(velocity[1]) < 0.04
        )
        self.stall_since, stall_for = continuous_duration(
            self.stall_since, is_active(final_drive) and stationary, now,
        )

        active_mission = (
            self.mission_state in MOVING_STATES
            and self.control_mode != "MANUAL"
        )
        has_command = any(is_active(x) for x in (
            selected, automatic, smoothed, final_drive,
        ))
        self.no_command_since, no_command_for = continuous_duration(
            self.no_command_since,
            active_mission and not has_command,
            now,
        )

        return {
            "ready": now - self.started >= self.startup_grace,
            "expect_clock": self.expect_clock,
            "expect_mission": self.expect_mission,
            "expect_perception": self.expect_perception,
            "expect_localization": self.expect_localization,
            "expect_scan": self.expect_scan,
            "robot_description_seen": self.robot_description_seen,
            "clock_age": self._age("clock", now),
            "rgb_age": self._age("rgb", now),
            "depth_age": self._age("depth", now),
            "scan_age": self._age("scan", now),
            "odom_age": self._age("odom", now),
            "yolo_age": self._age("yolo", now),
            "tf_local": self._tf_ok("odom", "base_footprint")
            if self.expect_localization else None,
            "tf_global": self._tf_ok("map", "odom")
            if self.expect_localization else None,
            "mission_state": self.mission_state,
            "state_age": max(0.0, now - self.mission_state_since),
            "control_mode": self.control_mode,
            "failure_reason": self.failure_reason,
            "collision_action": self.collision_action,
            "collision_polygon": self.collision_polygon,
            "blocked_for": blocked_for,
            "auto_blocked_for": auto_blocked_for,
            "stall_for": stall_for,
            "no_command_for": no_command_for,
            "smoothed": smoothed,
            "final_drive": final_drive,
            "odom_velocity": velocity,
        }

    def _snapshot_result(self, future):
        self.snapshot_inflight = False
        try:
            success = bool(future.result().success)
        except Exception as exc:
            self._event(
                "SNAPSHOT_FAILED", "Recorder", f"Snapshot error: {exc}",
                level=DiagnosticStatus.WARN,
            )
            return
        self._event(
            "SNAPSHOT_SAVED" if success else "SNAPSHOT_FAILED",
            "Recorder",
            "Incident cache written to MCAP bag." if success
            else "rosbag2 recorder rejected the snapshot.",
            level=DiagnosticStatus.OK if success else DiagnosticStatus.WARN,
        )

    def _maybe_snapshot(self, primary, now):
        if primary.code == "OK":
            self.snapshot_armed_for = None
            return
        if not self.auto_snapshot or primary.code not in AUTO_SNAPSHOT_CODES:
            return
        if self.snapshot_armed_for == primary.code or self.snapshot_inflight:
            return
        if now - self.snapshot_last_at < self.snapshot_cooldown:
            return
        if self.snapshot_client is None or not self.snapshot_client.service_is_ready():
            return

        future = self.snapshot_client.call_async(Snapshot.Request())
        self.snapshot_armed_for = primary.code
        self.snapshot_last_at = now
        self.snapshot_inflight = True
        future.add_done_callback(self._snapshot_result)
        self._event(
            "SNAPSHOT_REQUESTED", "Recorder",
            f"Saving incident for {primary.code}.",
            level=DiagnosticStatus.WARN,
        )

    def _tick(self):
        now = time.monotonic()
        observation = self._observation(now)
        findings = evaluate(observation)
        primary = primary_finding(findings)

        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        overall = DiagnosticStatus()
        overall.name = "SCROBOT/Overall"
        overall.hardware_id = "scrobot"
        overall.level = primary.level
        overall.message = f"{primary.code}: {primary.message}"
        overall.values = [
            KeyValue(key="code", value=primary.code),
            KeyValue(key="next_step", value=ACTIONS.get(primary.code, "")),
            KeyValue(key="mission", value=self.mission_state),
            KeyValue(key="phase", value=self.phase),
        ]
        message.status.append(overall)

        for finding in findings:
            status = DiagnosticStatus()
            status.level = finding.level
            status.name = f"SCROBOT/{finding.component}"
            status.hardware_id = "scrobot"
            status.message = finding.message
            status.values = [
                KeyValue(key="code", value=finding.code),
                KeyValue(key="detail", value=finding.detail),
            ]
            message.status.append(status)
        self.diagnostics_pub.publish(message)

        summary = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "severity": SEVERITY_TEXT[primary.level],
            "code": primary.code,
            "reason": primary.message,
            "next_step": ACTIONS.get(primary.code, ""),
            "mission_state": self.mission_state,
            "local_collect_phase": self.phase,
            "control_mode": self.control_mode,
            "shuttles_remaining_gt": self.shuttles_remaining,
            "smoothed_command": observation["smoothed"],
            "final_drive_command": observation["final_drive"],
            "odometry_velocity": observation["odom_velocity"],
            "alerts": [
                {"component": f.component, "level": SEVERITY_TEXT[f.level],
                 "code": f.code, "message": f.message}
                for f in findings if f.level > 0
            ],
        }
        summary_msg = String()
        summary_msg.data = json.dumps(summary, separators=(",", ":"))
        self.summary_pub.publish(summary_msg)

        if primary.code != self.last_primary_code:
            if self.last_primary_code is not None:
                self._event(
                    "HEALTH_TRANSITION", "Health",
                    f"{self.last_primary_code} -> {primary.code}: {primary.message}",
                    level=primary.level,
                    values={
                        "from_code": self.last_primary_code,
                        "to_code": primary.code,
                    },
                )
            self.last_primary_code = primary.code

        self._maybe_snapshot(primary, now)


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
