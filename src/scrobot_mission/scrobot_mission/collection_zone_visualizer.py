#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class CollectionZoneVisualizer(Node):
    def __init__(self):
        super().__init__('collection_zone_visualizer')
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('topic', '/mission/collection_zone')
        self.declare_parameter('pickup_offset_x', 0.165)
        self.declare_parameter('pickup_half_length', 0.060)
        self.declare_parameter('pickup_half_width', 0.150)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.topic = str(self.get_parameter('topic').value)
        cx = float(self.get_parameter('pickup_offset_x').value)
        hx = float(self.get_parameter('pickup_half_length').value)
        hy = float(self.get_parameter('pickup_half_width').value)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(PolygonStamped, self.topic, qos)

        self.msg = PolygonStamped()
        self.msg.header.frame_id = self.frame_id
        for x, y in [
            (cx - hx, +hy),
            (cx + hx, +hy),
            (cx + hx, -hy),
            (cx - hx, -hy),
        ]:
            p = Point32()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.01
            self.msg.polygon.points.append(p)

        self.timer = self.create_timer(0.5, self.publish)
        self.publish()
        self.get_logger().info(
            f'Collection zone RViz polygon: center_x={cx:.3f}, '
            f'x_tol=+-{hx:.3f}, y_tol=+-{hy:.3f} m.'
        )

    def publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = CollectionZoneVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
