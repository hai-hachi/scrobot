#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


class ManualModeManager(Node):
    """Own the AUTO/MANUAL control mode and gate manual velocity commands.

    The final command mux already gives /cmd_vel_manual higher priority than
    autonomous commands. This node makes that override persistent and explicit:
    while MANUAL is active it continuously publishes the latest manual command,
    or zero if the operator is not commanding motion. Autonomous commands can
    therefore never leak through between keyboard key presses.

    /control/manual_mode is transient-local so mission nodes that start later
    immediately learn the current mode and can pause/resume their actions.
    """

    def __init__(self):
        super().__init__('manual_mode_manager')

        self.declare_parameter('manual_input_topic', '/cmd_vel_manual_input')
        self.declare_parameter('manual_output_topic', '/cmd_vel_manual')
        self.declare_parameter('mode_topic', '/control/mode')
        self.declare_parameter('manual_mode_topic', '/control/manual_mode')
        self.declare_parameter('service_name', '/control/set_manual_mode')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('command_timeout', 0.20)
        self.declare_parameter('default_manual_mode', False)

        self.manual_input_topic = str(
            self.get_parameter('manual_input_topic').value
        )
        self.manual_output_topic = str(
            self.get_parameter('manual_output_topic').value
        )
        self.mode_topic = str(self.get_parameter('mode_topic').value)
        self.manual_mode_topic = str(
            self.get_parameter('manual_mode_topic').value
        )
        self.service_name = str(self.get_parameter('service_name').value)
        self.publish_rate = max(
            1.0, float(self.get_parameter('publish_rate').value)
        )
        self.command_timeout = max(
            0.0, float(self.get_parameter('command_timeout').value)
        )
        self.manual_mode = bool(
            self.get_parameter('default_manual_mode').value
        )

        command_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.manual_pub = self.create_publisher(
            TwistStamped, self.manual_output_topic, command_qos
        )
        self.mode_pub = self.create_publisher(
            String, self.mode_topic, state_qos
        )
        self.manual_mode_pub = self.create_publisher(
            Bool, self.manual_mode_topic, state_qos
        )
        self.create_subscription(
            TwistStamped,
            self.manual_input_topic,
            self._manual_input_cb,
            command_qos,
        )
        self.create_service(
            SetBool,
            self.service_name,
            self._set_manual_mode_cb,
        )

        self.last_manual_command = TwistStamped()
        self.last_manual_command.header.frame_id = 'base_link'
        self.last_input_monotonic = None

        self.create_timer(1.0 / self.publish_rate, self._tick)
        self._publish_mode_state()

        self.get_logger().info(
            'Manual mode manager ready: '
            f'{self.service_name}, input={self.manual_input_topic}, '
            f'output={self.manual_output_topic}, '
            f'default={"MANUAL" if self.manual_mode else "AUTO"}.'
        )

    def _manual_input_cb(self, msg):
        self.last_manual_command = msg
        self.last_input_monotonic = time.monotonic()

    def _set_manual_mode_cb(self, request, response):
        requested = bool(request.data)
        changed = requested != self.manual_mode
        self.manual_mode = requested

        # Always force a zero transition command. Entering MANUAL immediately
        # suppresses autonomous motion; leaving MANUAL never carries a stale
        # operator command into the mux timeout window.
        self._publish_manual(0.0, 0.0)
        self.last_input_monotonic = None
        self.last_manual_command = TwistStamped()
        self.last_manual_command.header.frame_id = 'base_link'

        self._publish_mode_state()

        response.success = True
        response.message = (
            f'Control mode {"changed to" if changed else "already"} '
            f'{"MANUAL" if self.manual_mode else "AUTO"}.'
        )
        self.get_logger().info(response.message)
        return response

    def _publish_mode_state(self):
        mode_msg = String()
        mode_msg.data = 'MANUAL' if self.manual_mode else 'AUTO'
        self.mode_pub.publish(mode_msg)

        manual_msg = Bool()
        manual_msg.data = self.manual_mode
        self.manual_mode_pub.publish(manual_msg)

    def _publish_manual(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.manual_pub.publish(msg)

    def _tick(self):
        if not self.manual_mode:
            return

        now = self.get_clock().now()
        if self.last_input_monotonic is None:
            self._publish_manual(0.0, 0.0)
            return

        # Operator deadman timing deliberately uses wall-clock monotonic time.
        # It must expire even if Gazebo /clock is paused.
        age = time.monotonic() - self.last_input_monotonic
        if age > self.command_timeout:
            self._publish_manual(0.0, 0.0)
            return

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist = self.last_manual_command.twist
        self.manual_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ManualModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
