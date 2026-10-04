#!/usr/bin/env python3

import select
import sys
import termios
import time
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
W/S   : hold to drive forward / backward
A/D   : hold to rotate left / right
SPACE : stop immediately
1-9   : set linear and angular speed to 0.1-0.9
Q     : quit teleop (robot stays in MANUAL if MANUAL is active)

Manual driving is accepted only after M succeeds.
Motion commands are republished continuously while a movement key is held.
Releasing the key stops after a short key-repeat timeout.
Normal MANUAL commands still pass through velocity smoothing and collision monitoring.
Use R explicitly when you want autonomous operation to resume.
------------------------------------------------
"""


class ManualTeleop(Node):
    def __init__(self):
        super().__init__('manual_teleop')

        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('angular_speed', 0.50)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('key_hold_timeout', 0.60)
        self.declare_parameter('manual_input_topic', '/cmd_vel_manual_input')
        self.declare_parameter('mode_service', '/control/set_manual_mode')
        self.declare_parameter('manual_mode_topic', '/control/manual_mode')

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.publish_rate = max(
            1.0, float(self.get_parameter('publish_rate').value)
        )
        self.key_hold_timeout = max(
            0.05, float(self.get_parameter('key_hold_timeout').value)
        )

        self.manual_active = False
        self.command_active = False
        self.command_linear = 0.0
        self.command_angular = 0.0
        self.last_motion_key_time = None

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
        if not self.manual_active:
            self.clear_motion()

    def publish_command(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.publisher.publish(msg)

    def clear_motion(self):
        self.command_active = False
        self.command_linear = 0.0
        self.command_angular = 0.0
        self.last_motion_key_time = None

    def stop(self):
        self.clear_motion()
        self.publish_command(0.0, 0.0)

    def set_motion(self, linear_x, angular_z):
        self.command_linear = float(linear_x)
        self.command_angular = float(angular_z)
        self.command_active = True
        self.last_motion_key_time = time.monotonic()

    def publish_motion_if_active(self):
        if not self.manual_active or not self.command_active:
            return

        if self.last_motion_key_time is None:
            self.stop()
            return

        age = time.monotonic() - self.last_motion_key_time
        if age > self.key_hold_timeout:
            self.stop()
            return

        self.publish_command(
            self.command_linear,
            self.command_angular,
        )

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
        if not self.manual_active:
            self.clear_motion()

        print(f' {response.message}')
        return True


def read_key(timeout_sec):
    ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if not ready:
        return None
    return sys.stdin.read(1)


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = ManualTeleop()

    print(HELP)

    loop_period = 1.0 / node.publish_rate
    tty.setraw(sys.stdin.fileno())

    try:
        while rclpy.ok():
            # Process the transient-local manual-mode state subscription.
            rclpy.spin_once(node, timeout_sec=0.0)

            key = read_key(loop_period)
            if key is not None:
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
                    print(f'\r Speed set to {speed:.1f}      ', end='', flush=True)
                    node.stop()
                    continue

                if lower == 'q':
                    node.stop()
                    print(
                        '\r Teleop closed. Control mode was not changed; '
                        'press R before Q if autonomous operation should resume.'
                    )
                    break

                if lower in ('w', 'a', 's', 'd'):
                    if not node.manual_active:
                        print(
                            '\r Press M to enter MANUAL before driving.      ',
                            end='',
                            flush=True,
                        )
                        continue

                    if lower == 'w':
                        node.set_motion(node.linear_speed, 0.0)
                    elif lower == 's':
                        node.set_motion(-node.linear_speed, 0.0)
                    elif lower == 'a':
                        node.set_motion(0.0, node.angular_speed)
                    elif lower == 'd':
                        node.set_motion(0.0, -node.angular_speed)
                else:
                    node.stop()

            node.publish_motion_if_active()

    except Exception as error:
        print(f'\n{error}')

    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
