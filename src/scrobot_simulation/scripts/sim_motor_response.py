#!/usr/bin/env python3
"""Simulation-only first-order closed-loop wheel RPM response surrogate.

Takes the output of Nav2 Collision Monitor, before the differential-drive
controller. Source wheel rates are converted from Twist, filtered
independently, then converted back to a TwistStamped. The default Gazebo
velocity interface still enforces joint velocity; this intentionally
approximates the measured closed-loop STM32 wheel RPM dynamics, NOT torque.
"""
from __future__ import annotations

import math

RADIUS = 0.050
SEPARATION = 0.420
MAX_RPM = 95.0


def twist_to_wheel_rpm(linear, angular):
    factor = 60.0 / (2.0 * math.pi * RADIUS)
    return ((linear - angular * SEPARATION / 2.0) * factor,
            (linear + angular * SEPARATION / 2.0) * factor)


def wheel_rpm_to_twist(left, right):
    factor = 2.0 * math.pi * RADIUS / 60.0
    return ((left + right) * factor / 2.0,
            (right - left) * factor / SEPARATION)


def limit_wheels(left, right, limit=MAX_RPM):
    highest = max(abs(left), abs(right))
    scale = min(1.0, limit / highest) if highest else 1.0
    return left * scale, right * scale


def filter_update(state, reference, dt, tau, gain):
    if not math.isfinite(dt) or dt < 0 or not 0.001 <= tau <= 0.5:
        raise ValueError("Invalid first-order simulation state")
    alpha = -math.expm1(-min(dt, 0.1) / tau)
    return state + alpha * (gain * reference - state)


def bound_wheel_rpm(previous,filtered,dt,max_rpm,max_accel_rpmps):
    """Final individual wheel acceleration and RPM clamp after motor filtering."""
    if not all(map(math.isfinite,(
            previous,filtered,dt,max_rpm,max_accel_rpmps))):
        raise ValueError("Non-finite simulated motor state")
    if dt<0 or max_rpm<=0 or max_accel_rpmps<0:
        raise ValueError("Invalid simulated motor limits")
    if max_accel_rpmps>0:
        delta=max_accel_rpmps*dt
        filtered=max(previous-delta,min(previous+delta,filtered))
    return max(-max_rpm,min(max_rpm,filtered))


def main():
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.node import Node

    class SimMotorResponse(Node):
        def __init__(self):
            super().__init__("sim_motor_response")
            self.declare_parameter("tau_left", 0.05)
            self.declare_parameter("tau_right", 0.05)
            self.declare_parameter("gain_left", 1.0)
            self.declare_parameter("gain_right", 1.0)
            self.declare_parameter("command_timeout", 0.4)
            # Optional tracker-only motor envelope. Disabled by default so
            # this shared surrogate preserves other lab behavior.
            self.declare_parameter("max_wheel_rpm", MAX_RPM)
            self.declare_parameter("max_wheel_accel_rpmps", 0.0)
            self.tau = [
                float(self.get_parameter("tau_left").value),
                float(self.get_parameter("tau_right").value),
            ]
            self.gain = [
                float(self.get_parameter("gain_left").value),
                float(self.get_parameter("gain_right").value),
            ]
            self.timeout = float(self.get_parameter("command_timeout").value)
            self.max_wheel_rpm = float(
                self.get_parameter("max_wheel_rpm").value)
            self.max_wheel_accel_rpmps = float(
                self.get_parameter("max_wheel_accel_rpmps").value)
            if any(not 0.005 <= t <= 0.5 for t in self.tau):
                raise ValueError("Motor time constants outside [0.005, 0.5] s")
            if any(not 0.8 <= k <= 1.1 for k in self.gain):
                raise ValueError("Motor gains outside [0.8, 1.1]")
            if not 0.1 <= self.timeout <= 2.0:
                raise ValueError("Invalid command_timeout")
            if not math.isfinite(self.max_wheel_rpm) or self.max_wheel_rpm<=0:
                raise ValueError("Invalid max_wheel_rpm")
            if (not math.isfinite(self.max_wheel_accel_rpmps) or
                    self.max_wheel_accel_rpmps<0):
                raise ValueError("Invalid max_wheel_accel_rpmps")
            self.current = [0.0, 0.0]
            self.reference = [0.0, 0.0]
            self.last_command_ns = None
            self.last_tick_ns = None
            self.pub = self.create_publisher(
                TwistStamped, "/diff_drive_controller/cmd_vel", 10
            )
            self.create_subscription(
                TwistStamped, "/cmd_vel_motor_target", self.on_target, 10
            )
            self.create_timer(0.01, self.tick)
            self.get_logger().info(
                f"Simulation-only motor response active: tau={self.tau} "
                f"K={self.gain}, max_rpm={self.max_wheel_rpm}, "
                f"max_rpm_per_s={self.max_wheel_accel_rpmps}; "
                f"input=/cmd_vel_motor_target "
                f"output=/diff_drive_controller/cmd_vel"
            )

        def on_target(self, msg):
            v = float(msg.twist.linear.x)
            yaw = float(msg.twist.angular.z)
            if not all(map(math.isfinite, (v, yaw))):
                self.get_logger().warning("Discarding nonfinite motor target")
                return
            self.reference = list(limit_wheels(
                *twist_to_wheel_rpm(v, yaw),limit=self.max_wheel_rpm))
            self.last_command_ns = self.get_clock().now().nanoseconds

        def tick(self):
            now_ns = self.get_clock().now().nanoseconds
            if self.last_tick_ns is None or now_ns < self.last_tick_ns:
                self.current = [0.0, 0.0]
                self.reference = [0.0, 0.0]
                self.last_command_ns = None
                dt = 0.0
            else:
                dt = max(0.0, min((now_ns - self.last_tick_ns) * 1e-9, 0.1))
            self.last_tick_ns = now_ns

            if (self.last_command_ns is None or
                    now_ns - self.last_command_ns > self.timeout * 1e9):
                self.reference = [0.0, 0.0]
            for i in (0, 1):
                filtered = filter_update(
                    self.current[i], self.reference[i], dt,
                    self.tau[i], self.gain[i])
                if self.max_wheel_accel_rpmps>0:
                    self.current[i] = bound_wheel_rpm(
                        self.current[i],filtered,dt,
                        self.max_wheel_rpm,self.max_wheel_accel_rpmps)
                else:
                    # Legacy non-Tracker lab behavior: the old filter
                    # proportionally scales both wheel rates together.
                    self.current[i] = filtered
            if self.max_wheel_accel_rpmps==0:
                self.current=list(limit_wheels(
                    *self.current,limit=self.max_wheel_rpm))
            linear, angular = wheel_rpm_to_twist(*self.current)
            out = TwistStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = "base_link"
            out.twist.linear.x = linear
            out.twist.angular.z = angular
            self.pub.publish(out)

    rclpy.init()
    node = SimMotorResponse()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
