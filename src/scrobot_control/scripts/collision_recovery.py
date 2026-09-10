#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav2_msgs.msg import CollisionMonitorState
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class CollisionRecovery(Node):
    """Back the robot away briefly when Collision Monitor triggers front STOP."""

    def __init__(self):
        super().__init__('collision_recovery')

        self.declare_parameter('state_topic', '/collision_monitor_state')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_collision_recovery')
        self.declare_parameter('stop_polygon_name', 'stop_zone')
        self.declare_parameter('reverse_speed', -0.15)
        self.declare_parameter('reverse_duration', 0.80)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cooldown', 0.75)

        self.state_topic = str(self.get_parameter('state_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.stop_polygon_name = str(
            self.get_parameter('stop_polygon_name').value
        )
        self.reverse_speed = float(self.get_parameter('reverse_speed').value)
        self.reverse_duration = float(
            self.get_parameter('reverse_duration').value
        )
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.cooldown = float(self.get_parameter('cooldown').value)

        if self.reverse_speed >= 0.0:
            raise ValueError('reverse_speed must be negative.')
        if self.reverse_duration <= 0.0:
            raise ValueError('reverse_duration must be > 0.')
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be > 0.')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.cmd_vel_topic,
            qos,
        )
        self.state_sub = self.create_subscription(
            CollisionMonitorState,
            self.state_topic,
            self.state_callback,
            qos,
        )

        self.recovery_active = False
        self.recovery_end = 0.0
        self.last_recovery_end = -1e9
        self.stop_latched = False

        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.update,
        )

        self.get_logger().info(
            'Collision recovery ready: '
            f'{self.stop_polygon_name} STOP -> '
            f'{self.reverse_speed:.2f} m/s for '
            f'{self.reverse_duration:.2f} s.'
        )

    def state_callback(self, msg):
        is_front_stop = (
            msg.action_type == CollisionMonitorState.STOP
            and msg.polygon_name == self.stop_polygon_name
        )

        if not is_front_stop:
            self.stop_latched = False
            return

        if self.stop_latched or self.recovery_active:
            return

        now = time.monotonic()
        if now - self.last_recovery_end < self.cooldown:
            return

        self.stop_latched = True
        self.recovery_active = True
        self.recovery_end = now + self.reverse_duration

        self.get_logger().warn(
            f'{self.stop_polygon_name} triggered: backing out.'
        )

    def publish_cmd(self, linear_x):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = float(linear_x)
        self.cmd_pub.publish(msg)

    def update(self):
        if not self.recovery_active:
            return

        now = time.monotonic()
        if now < self.recovery_end:
            self.publish_cmd(self.reverse_speed)
            return

        self.publish_cmd(0.0)
        self.recovery_active = False
        self.last_recovery_end = now
        self.get_logger().info('Collision back-out complete.')


def main(args=None):
    rclpy.init(args=args)
    node = CollisionRecovery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
