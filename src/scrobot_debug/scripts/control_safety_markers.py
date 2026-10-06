#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class ControlSafetyMarkers(Node):
    """Publish local safety overlays for the control-stack RViz check."""

    def __init__(self):
        super().__init__('control_safety_markers')

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub = self.create_publisher(
            MarkerArray,
            '/debug/control_safety_markers',
            qos,
        )
        self.timer = self.create_timer(0.5, self.publish_markers)

        self.stop_zone = [
            (0.40, 0.225),
            (0.40, -0.225),
            (0.30, -0.225),
            (0.30, 0.225),
        ]
        self.slowdown_zone = [
            (0.50, 0.325),
            (0.50, -0.325),
            (0.30, -0.325),
            (0.30, 0.325),
        ]
        self.base_footprint = [
            (0.325, 0.225),
            (0.325, -0.225),
            (-0.420, -0.225),
            (-0.420, 0.225),
        ]

    @staticmethod
    def point(x, y, z):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def make_fill(self, marker_id, name, points, z, rgba):
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = name
        marker.id = marker_id
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]

        a, b, c, d = points
        for x, y in (a, b, c, a, c, d):
            marker.points.append(self.point(x, y, z))
        return marker

    def make_outline(self, marker_id, name, points, z, rgba, width=0.025):
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = name
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(width)
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]

        for x, y in points + [points[0]]:
            marker.points.append(self.point(x, y, z))
        return marker

    def publish_markers(self):
        msg = MarkerArray()
        msg.markers.extend([
            self.make_fill(1, 'slowdown_zone_fill', self.slowdown_zone, 0.012, (1.0, 0.75, 0.0, 0.18)),
            self.make_outline(2, 'slowdown_zone_outline', self.slowdown_zone, 0.018, (1.0, 0.85, 0.0, 1.0)),
            self.make_fill(3, 'stop_zone_fill', self.stop_zone, 0.025, (1.0, 0.0, 0.0, 0.28)),
            self.make_outline(4, 'stop_zone_outline', self.stop_zone, 0.032, (1.0, 0.0, 0.0, 1.0)),
            self.make_outline(5, 'base_footprint_outline', self.base_footprint, 0.045, (0.2, 0.8, 1.0, 1.0), 0.018),
        ])
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControlSafetyMarkers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
