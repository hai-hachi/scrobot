#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class DepthScanSelfFilter(Node):
    """Remove LaserScan rays that fall inside the robot body footprint."""

    def __init__(self):
        super().__init__('depth_scan_self_filter')

        self.declare_parameter('input_topic', '/camera/camera/depth/scan_raw')
        self.declare_parameter('output_topic', '/camera/camera/depth/scan')

        # Same tight body mask used for the false return at x=0.316, y=-0.218.
        self.declare_parameter('self_min_x', -0.45)
        self.declare_parameter('self_max_x', 0.34)
        self.declare_parameter('self_min_y', -0.24)
        self.declare_parameter('self_max_y', 0.24)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.bounds = (
            float(self.get_parameter('self_min_x').value),
            float(self.get_parameter('self_max_x').value),
            float(self.get_parameter('self_min_y').value),
            float(self.get_parameter('self_max_y').value),
        )

        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE

        self.pub = self.create_publisher(LaserScan, output_topic, sensor_qos)
        self.create_subscription(LaserScan, input_topic, self.scan_cb, sensor_qos)
        self.last_stats_sec = 0.0

        self.get_logger().info(
            'Filtering %s -> %s using body box x=[%.3f, %.3f], y=[%.3f, %.3f]'
            % (input_topic, output_topic, *self.bounds)
        )

    def inside_self(self, x, y):
        min_x, max_x, min_y, max_y = self.bounds
        return min_x <= x <= max_x and min_y <= y <= max_y

    def scan_cb(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities)

        removed = 0
        angle = msg.angle_min
        replacement = math.inf if math.isinf(msg.range_max) else msg.range_max + 1.0

        for i, range_value in enumerate(out.ranges):
            if math.isfinite(range_value) and msg.range_min <= range_value <= msg.range_max:
                x = range_value * math.cos(angle)
                y = range_value * math.sin(angle)
                if self.inside_self(x, y):
                    out.ranges[i] = replacement
                    removed += 1
            angle += msg.angle_increment

        self.pub.publish(out)

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_stats_sec > 2.0:
            self.get_logger().info(
                f'depth_scan_self_filter removed {removed}/{len(out.ranges)} scan rays inside robot body'
            )
            self.last_stats_sec = now


def main(args=None):
    rclpy.init(args=args)
    node = DepthScanSelfFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
