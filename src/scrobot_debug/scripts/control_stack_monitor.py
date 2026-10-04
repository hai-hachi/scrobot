#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class ControlStackMonitor(Node):
    """Visualize and diagnose the control-stack safety path.

    This is debug-only. It does not alter commands or safety behavior.
    """

    def __init__(self):
        super().__init__('control_stack_monitor')

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

        self.latest_scan = None
        self.smoothed_cmd = None
        self.drive_cmd = None
        self.last_status = ''

        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE

        command_qos = QoSProfile(depth=10)
        command_qos.reliability = ReliabilityPolicy.RELIABLE
        command_qos.durability = DurabilityPolicy.VOLATILE

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            LaserScan,
            '/camera/camera/depth/scan',
            self.scan_cb,
            sensor_qos,
        )
        self.create_subscription(
            TwistStamped,
            '/cmd_vel_smoothed',
            self.smoothed_cb,
            command_qos,
        )
        self.create_subscription(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            self.drive_cb,
            command_qos,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/debug/control_stack_viz',
            marker_qos,
        )
        self.status_pub = self.create_publisher(
            String,
            '/debug/control_stack_status',
            command_qos,
        )

        self.create_timer(0.2, self.publish_markers)
        self.create_timer(1.0, self.publish_status)

    def scan_cb(self, msg):
        self.latest_scan = msg

    def smoothed_cb(self, msg):
        self.smoothed_cmd = msg

    def drive_cb(self, msg):
        self.drive_cmd = msg

    @staticmethod
    def point(x, y, z):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    @staticmethod
    def inside_rect(x, y, rect):
        xs = [p[0] for p in rect]
        ys = [p[1] for p in rect]
        return min(xs) <= x <= max(xs) and min(ys) <= y <= max(ys)

    @staticmethod
    def command_pair(msg):
        if msg is None:
            return None
        return (float(msg.twist.linear.x), float(msg.twist.angular.z))

    @staticmethod
    def command_nonzero(cmd):
        if cmd is None:
            return False
        return abs(cmd[0]) > 1e-3 or abs(cmd[1]) > 1e-3

    def scan_points(self):
        if self.latest_scan is None:
            return []

        msg = self.latest_scan
        points = []
        angle = msg.angle_min
        for range_value in msg.ranges:
            if math.isfinite(range_value) and msg.range_min <= range_value <= msg.range_max:
                x = range_value * math.cos(angle)
                y = range_value * math.sin(angle)
                points.append((x, y, range_value, angle))
            angle += msg.angle_increment
        return points

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

    def make_scan_marker(self, marker_id, points):
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'scan_points'
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.055
        marker.scale.y = 0.055
        marker.scale.z = 0.055
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        for x, y, _, _ in points:
            marker.points.append(self.point(x, y, 0.08))
        return marker

    def publish_markers(self):
        scan_points = self.scan_points()

        msg = MarkerArray()
        msg.markers.extend([
            self.make_fill(1, 'slowdown_zone_fill', self.slowdown_zone, 0.012, (1.0, 0.75, 0.0, 0.18)),
            self.make_outline(2, 'slowdown_zone_outline', self.slowdown_zone, 0.018, (1.0, 0.85, 0.0, 1.0)),
            self.make_fill(3, 'stop_zone_fill', self.stop_zone, 0.025, (1.0, 0.0, 0.0, 0.28)),
            self.make_outline(4, 'stop_zone_outline', self.stop_zone, 0.032, (1.0, 0.0, 0.0, 1.0)),
            self.make_outline(5, 'base_footprint_outline', self.base_footprint, 0.045, (0.2, 0.8, 1.0, 1.0), 0.018),
            self.make_scan_marker(6, scan_points),
        ])
        self.marker_pub.publish(msg)

    def publish_status(self):
        scan_points = self.scan_points()
        stop_hits = sum(1 for x, y, _, _ in scan_points if self.inside_rect(x, y, self.stop_zone))
        slow_hits = sum(1 for x, y, _, _ in scan_points if self.inside_rect(x, y, self.slowdown_zone))

        min_point = None
        if scan_points:
            min_point = min(scan_points, key=lambda p: p[2])

        smoothed = self.command_pair(self.smoothed_cmd)
        drive = self.command_pair(self.drive_cmd)

        state = 'WAITING'
        if self.command_nonzero(smoothed) and not self.command_nonzero(drive):
            state = 'SMOOTHED_NONZERO_FINAL_ZERO'
        elif self.command_nonzero(drive):
            state = 'DRIVE_COMMAND_ACTIVE'
        elif scan_points:
            state = 'SCAN_ACTIVE_NO_DRIVE_COMMAND'

        min_text = 'none'
        if min_point is not None:
            min_text = (
                f'r={min_point[2]:.3f} x={min_point[0]:.3f} '
                f'y={min_point[1]:.3f} angle={min_point[3]:.3f}'
            )

        smoothed_text = 'none' if smoothed is None else f'vx={smoothed[0]:.3f} wz={smoothed[1]:.3f}'
        drive_text = 'none' if drive is None else f'vx={drive[0]:.3f} wz={drive[1]:.3f}'

        text = (
            f'{state} | scan_points={len(scan_points)} stop_hits={stop_hits} '
            f'slowdown_hits={slow_hits} min={min_text} | '
            f'smoothed={smoothed_text} | drive={drive_text}'
        )

        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

        if text != self.last_status:
            self.get_logger().info(text)
            self.last_status = text


def main(args=None):
    rclpy.init(args=args)
    node = ControlStackMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
