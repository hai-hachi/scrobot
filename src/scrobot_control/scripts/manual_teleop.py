#!/usr/bin/env python3

import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


HELP = """
------------------------------------------------
SC Robot manual teleop
------------------------------------------------

        W
    A   S   D

M     : enter MANUAL and pause autonomous mission
R     : return to AUTO and resume autonomous mission
W/S   : forward / backward
A/D   : rotate left / right
SPACE : stop
1-9   : set linear and angular speed to 0.1-0.9
Q     : quit teleop (robot stays in MANUAL if MANUAL is active)

Manual driving is accepted only after M succeeds.
Normal MANUAL commands still pass through velocity smoothing and collision monitoring.
Use R explicitly when you want autonomous operation to resume.
------------------------------------------------
"""


class ManualTeleop(Node):
    def __init__(self):
        super().__init__('manual_teleop')

        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('angular_speed', 0.50)
        self.declare_parameter('manual_input_topic', '/cmd_vel_manual_input')
        self.declare_parameter('mode_service', '/control/set_manual_mode')
        self.declare_parameter('manual_mode_topic', '/control/manual_mode')

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.manual_active = False

        control_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(
            TwistStamped,
            str(self.get_parameter('manual_input_topic').value),
            control_qos,
        )
        self.mode_client = self.create_client(
            SetBool,
            str(self.get_parameter('mode_service').value),
        )

        mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('manual_mode_topic').value),
            self._mode_cb,
            mode_qos,
        )

    def _mode_cb(self, msg):
        self.manual_active = bool(msg.data)

    def publish_command(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.publisher.publish(msg)

    def stop(self):
        self.publish_command(0.0, 0.0)

    def set_manual_mode(self, enabled):
        if not self.mode_client.wait_for_service(timeout_sec=2.0):
            print(' Manual-mode service is unavailable. Start scrobot_control first.')
            return False

        request = SetBool.Request()
        request.data = bool(enabled)
        future = self.mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        response = future.result()
        if response is None or not response.success:
            print(' Failed to change control mode.')
            return False

        self.manual_active = bool(enabled)
        print(f' {response.message}')
        return True


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = ManualTeleop()

    print(HELP)

    try:
        while rclpy.ok():
            key = get_key(settings)
            lower = key.lower()

            if lower == 'm':
                node.stop()
                node.set_manual_mode(True)
                continue

            if lower == 'r':
                node.stop()
                node.set_manual_mode(False)
                continue

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
                print(
                    ' Teleop closed. Control mode was not changed; '
                    'press R before Q if autonomous operation should resume.'
                )
                break

            if not node.manual_active:
                if lower in ('w', 'a', 's', 'd'):
                    print(' Press M to enter MANUAL before driving.')
                continue

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

    except Exception as error:
        print(error)

    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
