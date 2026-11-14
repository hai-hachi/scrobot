#!/usr/bin/env python3
"""Enforce each physical drive wheel's RPM limit on normal AUTO/MANUAL commands.

Place between velocity_smoother and collision_monitor. This is an operational
constraint, not a replacement for the STM32 firmware safety/watchdog limits.
"""
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def constrain_twist(v, w, wheel_radius=0.05, wheel_separation=0.42,
                    max_wheel_rpm=100.0):
    """Uniformly scale (v, w) to satisfy both differential-drive wheel bounds."""
    if not all(math.isfinite(x) for x in (v, w, wheel_radius,
                                         wheel_separation, max_wheel_rpm)):
        return 0.0, 0.0
    if wheel_radius <= 0 or wheel_separation <= 0 or max_wheel_rpm <= 0:
        return 0.0, 0.0
    half_track = wheel_separation / 2.0
    max_wheel_mps = (2.0 * math.pi * wheel_radius * max_wheel_rpm) / 60.0
    requested = max(abs(v - half_track * w), abs(v + half_track * w))
    if requested > max_wheel_mps:
        scale = max_wheel_mps / requested
        return v * scale, w * scale
    return v, w


class WheelSpeedLimiter(Node):
    def __init__(self):
        super().__init__('wheel_speed_limiter')
        self.declare_parameter('input_topic', '/cmd_vel_smoothed')
        self.declare_parameter('output_topic', '/cmd_vel_wheel_limited')
        self.declare_parameter('wheel_radius_m', 0.05)
        self.declare_parameter('wheel_separation_m', 0.42)
        self.declare_parameter('max_wheel_rpm', 100.0)

        self.radius = float(self.get_parameter('wheel_radius_m').value)
        self.track = float(self.get_parameter('wheel_separation_m').value)
        self.limit = float(self.get_parameter('max_wheel_rpm').value)
        if not all(math.isfinite(x) and x > 0 for x in
                   (self.radius, self.track, self.limit)):
            raise ValueError('wheel radius, separation, and max RPM must be positive')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(
            TwistStamped, str(self.get_parameter('output_topic').value), qos)
        self.sub = self.create_subscription(
            TwistStamped, str(self.get_parameter('input_topic').value),
            self.on_command, qos)

    def on_command(self, msg):
        v, w = constrain_twist(msg.twist.linear.x, msg.twist.angular.z,
                               self.radius, self.track, self.limit)
        limited = TwistStamped()
        limited.header = msg.header
        limited.twist.linear.x = v
        limited.twist.angular.z = w
        # Differential drive cannot execute lateral, vertical, roll, or pitch.
        # Those components intentionally remain zero.
        self.pub.publish(limited)


def main(args=None):
    rclpy.init(args=args)
    node = WheelSpeedLimiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
