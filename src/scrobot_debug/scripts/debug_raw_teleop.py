#!/usr/bin/env python3

import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


HELP = """
------------------------------------------------
SC Robot DEBUG RAW teleop
------------------------------------------------

        W
    A   S   D

W/S   : forward / backward
A/D   : rotate left / right
SPACE : stop
1-9   : set linear and angular speed to 0.1-0.9
Q     : stop and quit

WARNING:
This debug-only tool publishes directly to the diff-drive controller.
It bypasses AUTO/MANUAL arbitration, velocity smoothing, and collision monitor.
Use only in controlled simulation / bench tests.
------------------------------------------------
"""


class DebugRawTeleop(Node):
    def __init__(self):
        super().__init__('debug_raw_teleop')

        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('angular_speed', 0.50)
        self.declare_parameter(
            'cmd_vel_topic',
            '/diff_drive_controller/cmd_vel',
        )

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)

        self.publisher = self.create_publisher(
            TwistStamped,
            str(self.get_parameter('cmd_vel_topic').value),
            10,
        )

    def publish_command(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.publisher.publish(msg)

    def stop(self):
        self.publish_command(0.0, 0.0)


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = DebugRawTeleop()

    print(HELP)

    try:
        while rclpy.ok():
            key = get_key(settings)
            lower = key.lower()

            if key == ' ':
                node.stop()
                continue

            if key in '123456789':
                speed = int(key) * 0.1
                node.linear_speed = speed
                node.angular_speed = speed
                print(f' Speed set to {speed:.1f}')
                node.stop()
                continue

            if lower == 'q':
                node.stop()
                break

            linear = 0.0
            angular = 0.0
            if lower == 'w':
                linear = node.linear_speed
            elif lower == 's':
                linear = -node.linear_speed
            elif lower == 'a':
                angular = node.angular_speed
            elif lower == 'd':
                angular = -node.angular_speed
            else:
                node.stop()
                continue

            node.publish_command(linear, angular)

    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
