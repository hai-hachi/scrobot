"""ROS2 experiment driver for floor-loaded Gazebo mobility calibration."""
from __future__ import annotations

import copy
import math
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from core import (
    DEFAULTS, WheelFilter, cap_wheels, profile_targets, response_metrics,
    rpm_to_twist, validate_settings, RADIUS, SEPARATION,
)


class MobilityDriver(Node):
    """The ONLY lab publisher to diff_drive_controller/cmd_vel.

    Deliberately excludes Nav2, velocity smoother and command arbitration:
    each test is a genuine step at the controller's external command port.
    """

    def __init__(self, settings=None):
        super().__init__(
            "mobility_lab_driver",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
            automatically_declare_parameters_from_overrides=True,
        )
        self.guard = threading.RLock()
        self.settings = validate_settings(settings or DEFAULTS)
        self.pub = self.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 30)
        # simulation_bridge publishes best-effort SENSOR_DATA; a reliable
        # subscription here would silently receive no ground truth.
        truth_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Odometry, "/evaluation/ground_truth_odom", self.on_truth, truth_qos
        )
        self.create_subscription(
            TwistStamped, "/diff_drive_controller/cmd_vel_out", self.on_limited, 20
        )
        self.timer = self.create_timer(0.01, self.tick)
        self.filter = WheelFilter()
        self.stage = "idle"
        self.step_time = 0.0
        self.last_tick = None
        self.latest_joints = None
        self.latest_truth = None
        self.latest_pose = None
        self.latest_limited = None
        self.last_joint_wall = 0.0
        self.last_truth_wall = 0.0
        self.samples = []
        self.snapshot = None
        self.target = (0.0, 0.0)
        self.sent = (0.0, 0.0)
        self.run_settings = None
        self.started_wall = None
        self.error = None

    def sim_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_joints(self, msg):
        try:
            left_idx = msg.name.index("left_wheel_joint")
            right_idx = msg.name.index("right_wheel_joint")
            left = float(msg.velocity[left_idx]) * 60 / (2 * math.pi)
            right = float(msg.velocity[right_idx]) * 60 / (2 * math.pi)
        except (ValueError, IndexError, TypeError):
            return
        with self.guard:
            self.latest_joints = (left, right)
            self.last_joint_wall = time.monotonic()
            if self.stage not in ("pre", "hold", "post"):
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            now = stamp if stamp > 0 else self.sim_time()
            t = now - self.step_time
            gt = self.latest_truth
            v_enc, w_enc = rpm_to_twist(left, right)
            self.samples.append({
                "t": round(t, 6),
                "input_left_rpm": round(self.target[0], 5),
                "input_right_rpm": round(self.target[1], 5),
                "sent_left_rpm": round(self.sent[0], 5),
                "sent_right_rpm": round(self.sent[1], 5),
                "actual_left_rpm": round(left, 5),
                "actual_right_rpm": round(right, 5),
                "encoder_v_m_s": round(v_enc, 6),
                "encoder_yaw_rad_s": round(w_enc, 6),
                "truth_v_m_s": round(gt[0], 6) if gt else None,
                "truth_yaw_rad_s": round(gt[1], 6) if gt else None,
                "truth_x_m": round(self.latest_pose[0], 5) if self.latest_pose else None,
                "truth_y_m": round(self.latest_pose[1], 5) if self.latest_pose else None,
                "truth_yaw_rad": round(self.latest_pose[2], 6) if self.latest_pose else None,
            })
            if len(self.samples) > 2000:
                self.samples = self.samples[-2000:]

    def on_truth(self, msg):
        # The Gazebo plugin supplies robot ground truth independently of
        # diff_drive_controller encoder-derived odometry.
        with self.guard:
            self.latest_truth = (
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.angular.z),
            )
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            self.latest_pose = (p.x, p.y, yaw)
            self.last_truth_wall = time.monotonic()

    def on_limited(self, msg):
        with self.guard:
            self.latest_limited = (float(msg.twist.linear.x), float(msg.twist.angular.z))

    def send(self, rpm):
        v, yaw = rpm_to_twist(*rpm)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = v
        msg.twist.angular.z = yaw
        self.pub.publish(msg)

    def tick(self):
        with self.guard:
            now = self.sim_time()
            dt = 0.01 if self.last_tick is None else max(0.0, min(0.1, now - self.last_tick))
            if self.last_tick is not None and now < self.last_tick - 0.01:
                # Gazebo restarted/reset its /clock: never continue stale trial.
                self.abort_locked("Gazebo clock reset during trial")
            self.last_tick = now

            if self.stage in ("pre", "hold", "post"):
                elapsed = now - self.step_time
                hold = self.run_settings["hold_s"]
                post = self.run_settings["post_s"]
                if elapsed < 0:
                    self.stage = "pre"
                    self.target = (0.0, 0.0)
                elif elapsed < hold:
                    self.stage = "hold"
                    self.target = profile_targets(
                        self.run_settings["profile"], self.run_settings["target_rpm"]
                    )
                elif elapsed < hold + post:
                    self.stage = "post"
                    self.target = (0.0, 0.0)
                else:
                    self.finish_locked()
            else:
                self.target = (0.0, 0.0)

            if self.stage in ("pre", "hold", "post"):
                self.sent = self.filter.update(self.target, dt, self.run_settings)
            else:
                self.filter.reset()
                self.sent = (0.0, 0.0)

            # Idle zeros are deliberately sent too; the drivetrain stops if
            # the browser disconnects or a test aborts.
            self.send(self.sent)

    def start(self, settings):
        checked = validate_settings(settings)
        with self.guard:
            if self.stage in ("pre", "hold", "post"):
                raise RuntimeError("An experiment is already running")
            if not self.last_joint_wall or time.monotonic() - self.last_joint_wall > 1.5:
                raise RuntimeError("No fresh /joint_states. Wait for controllers to activate.")
            if self.latest_joints and max(abs(x) for x in self.latest_joints) > 3.0:
                raise RuntimeError("Wheels must be at rest (<3 RPM) before a new experiment.")
            self.filter.reset()
            self.run_settings = checked
            self.settings = checked
            self.target = (0.0, 0.0)
            self.sent = (0.0, 0.0)
            self.samples = []
            self.snapshot = None
            self.error = None
            self.stage = "pre"
            self.step_time = self.sim_time() + checked["pre_s"]
            self.started_wall = time.monotonic()
            return {"stage": self.stage, "step_time": self.step_time}

    def finish_locked(self):
        cfg = dict(self.run_settings)
        left, right = profile_targets(cfg["profile"], cfg["target_rpm"])
        measured = self.samples
        metrics = {
            "left": response_metrics(measured, "left", left, cfg["hold_s"]),
            "right": response_metrics(measured, "right", right, cfg["hold_s"]),
        }
        # Slip / contact metrics are indicative only: wheel-to-body mismatch
        # requires ground truth and depends on slip, effective radius and
        # separation, not solely on a tire friction coefficient.
        gt = [x for x in measured if x["truth_v_m_s"] is not None
              and x["truth_yaw_rad_s"] is not None and 0 <= x["t"] <= cfg["hold_s"]]
        if gt:
            metrics["body_velocity_rmse_m_s"] = round(
                math.sqrt(sum((x["encoder_v_m_s"] - x["truth_v_m_s"])**2 for x in gt) / len(gt)), 5
            )
            metrics["yaw_rate_rmse_rad_s"] = round(
                math.sqrt(sum((x["encoder_yaw_rad_s"] - x["truth_yaw_rad_s"])**2 for x in gt) / len(gt)), 5
            )
        self.snapshot = {
            "settings": cfg,
            "metrics": metrics,
            "samples": copy.deepcopy(measured),
            "source": "Gazebo loaded robot, no smoother, diff_drive_controller input",
            "finished_sim_s": self.sim_time(),
        }
        self.stage = "complete"

    def abort_locked(self, reason="Operator canceled"):
        self.error = reason
        self.stage = "aborted"
        self.target = self.sent = (0.0, 0.0)
        self.filter.reset()
        self.send((0.0, 0.0))

    def abort(self):
        with self.guard:
            self.abort_locked()

    def reset_for_restart(self):
        with self.guard:
            self.abort_locked("Gazebo process restarting")
            self.stage = "idle"
            self.latest_joints = None
            self.latest_truth = None
            self.latest_pose = None
            self.latest_limited = None
            self.last_joint_wall = self.last_truth_wall = 0
            self.last_tick = None
            self.samples = []
            self.snapshot = None

    def status(self, rows=600):
        with self.guard:
            age = time.monotonic() - self.last_joint_wall if self.last_joint_wall else None
            gt_age = time.monotonic() - self.last_truth_wall if self.last_truth_wall else None
            return {
                "stage": self.stage,
                "sim_time": round(self.sim_time(), 4),
                "joint_age_wall_s": round(age, 2) if age is not None else None,
                "truth_age_wall_s": round(gt_age, 2) if gt_age is not None else None,
                "joints_rpm": list(self.latest_joints) if self.latest_joints else None,
                "truth_velocity": list(self.latest_truth) if self.latest_truth else None,
                "pose": list(self.latest_pose) if self.latest_pose else None,
                "requested_rpm": list(self.target),
                "sent_rpm": list(self.sent),
                "limited_twist": list(self.latest_limited) if self.latest_limited else None,
                "settings": dict(self.settings),
                "metrics": self.snapshot["metrics"] if self.snapshot else None,
                "samples": list(self.samples[-rows:]),
                "error": self.error,
                "controller_ready": age is not None and age < 1.5,
            }

    def result(self):
        with self.guard:
            return copy.deepcopy(self.snapshot)


class RosBridge:
    def __init__(self, settings):
        rclpy.init()
        self.node = MobilityDriver(settings)
        self.thread = threading.Thread(
            target=rclpy.spin, args=(self.node,), name="MobilityLabROS", daemon=True
        )
        self.thread.start()

    def close(self):
        self.node.abort()
        self.node.destroy_node()
        rclpy.shutdown()
        self.thread.join(timeout=2)
