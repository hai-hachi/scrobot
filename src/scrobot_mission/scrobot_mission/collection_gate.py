#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class CollectionGate(Node):
    """Enable physical pickup only during COLLECT_ROUTE."""

    def __init__(self):
        super().__init__('collection_gate')

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        control_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.enabled = False
        self.publisher = self.create_publisher(
            Bool,
            '/mission/collection_enabled',
            control_qos,
        )
        self.create_subscription(
            String,
            '/mission/state',
            self.state_callback,
            state_qos,
        )

        # Re-publish continuously so the ROS<->Gazebo bridge/plugin can restart
        # without leaving the simulator in an unknown pickup state.
        self.timer = self.create_timer(0.20, self.publish_state)
        self.publish_state()

    def state_callback(self, msg):
        new_enabled = msg.data == 'COLLECT_ROUTE'
        if new_enabled != self.enabled:
            self.enabled = new_enabled
            self.get_logger().info(
                f'Physical collection enabled={self.enabled} for mission state {msg.data}.'
            )
        self.publish_state()

    def publish_state(self):
        msg = Bool()
        msg.data = bool(self.enabled)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CollectionGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
