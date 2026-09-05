#!/usr/bin/env python3

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped


HELP = """
---------------------------
WASD manual teleop
---------------------------

        W
    A   S   D

W/S : forward/backward
A/D : rotate left/right
SPACE: stop
Q    : quit

---------------------------
"""


class WasdTeleop(Node):

    def __init__(self):

        super().__init__('wasd_teleop')

        self.declare_parameter(
            'linear_speed',
            0.2
        )

        self.declare_parameter(
            'angular_speed',
            0.2
        )

        self.linear_speed = float(
            self.get_parameter(
                'linear_speed'
            ).value
        )

        self.angular_speed = float(
            self.get_parameter(
                'angular_speed'
            ).value
        )

        self.publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel_manual',
            10
        )

    def publish_command(
        self,
        linear_x,
        angular_z
    ):

        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = 'base_link'

        msg.twist.linear.x = (
            float(linear_x)
        )

        msg.twist.angular.z = (
            float(angular_z)
        )

        self.publisher.publish(msg)

    def stop(self):

        self.publish_command(
            0.0,
            0.0
        )


def get_key(settings):

    tty.setraw(
        sys.stdin.fileno()
    )

    key = sys.stdin.read(1)

    termios.tcsetattr(
        sys.stdin,
        termios.TCSADRAIN,
        settings
    )

    return key


def main(args=None):

    settings = termios.tcgetattr(
        sys.stdin
    )

    rclpy.init(args=args)

    node = WasdTeleop()

    print(HELP)

    try:

        while rclpy.ok():

            key = get_key(settings)

            linear = 0.0
            angular = 0.0

            if key.lower() == 'w':

                linear = (
                    node.linear_speed
                )

            elif key.lower() == 's':

                linear = (
                    -node.linear_speed
                )

            elif key.lower() == 'a':

                angular = (
                    node.angular_speed
                )

            elif key.lower() == 'd':

                angular = (
                    -node.angular_speed
                )

            elif key == ' ':

                node.stop()

                continue

            elif key.lower() == 'q':

                node.stop()

                break

            else:

                node.stop()

                continue

            node.publish_command(
                linear,
                angular
            )

    except Exception as error:

        print(error)

    finally:

        node.stop()

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            settings
        )

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()