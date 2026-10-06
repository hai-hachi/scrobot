#!/usr/bin/env python3

import ast
import math

import rclpy
from geometry_msgs.msg import Point, PolygonStamped, TwistStamped
from rcl_interfaces.srv import GetParameters
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
        self.zone_source = 'debug_defaults'
        self.zone_topic_seen = False
        self.cm_param_future = None

        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE

        command_qos = QoSProfile(depth=10)
        command_qos.reliability = ReliabilityPolicy.RELIABLE
        command_qos.durability = DurabilityPolicy.VOLATILE

        marker_qos = QoSProfile(depth=5)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.VOLATILE

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
        self.create_subscription(
            PolygonStamped,
            '/collision_monitor/stop_zone',
            self.stop_zone_cb,
            command_qos,
        )
        self.create_subscription(
            PolygonStamped,
            '/collision_monitor/slowdown_zone',
            self.slowdown_zone_cb,
            command_qos,
        )

        self.cm_param_client = self.create_client(
            GetParameters,
            '/collision_monitor/get_parameters',
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
        self.create_timer(1.0, self.refresh_collision_monitor_params)

    def scan_cb(self, msg):
        self.latest_scan = msg

    def smoothed_cb(self, msg):
        self.smoothed_cmd = msg

    def drive_cb(self, msg):
        self.drive_cmd = msg

    def stop_zone_cb(self, msg):
        points = self.polygon_msg_points(msg)
        if points:
            self.stop_zone = points
            self.zone_topic_seen = True
            self.zone_source = 'collision_monitor_polygon_topics'

    def slowdown_zone_cb(self, msg):
        points = self.polygon_msg_points(msg)
        if points:
            self.slowdown_zone = points
            self.zone_topic_seen = True
            self.zone_source = 'collision_monitor_polygon_topics'

    @staticmethod
    def polygon_msg_points(msg):
        return [(float(p.x), float(p.y)) for p in msg.polygon.points]

    @staticmethod
    def parse_points(value):
        try:
            raw_points = ast.literal_eval(value)
        except (SyntaxError, ValueError):
            return []

        points = []
        for point in raw_points:
            if not isinstance(point, (list, tuple)) or len(point) < 2:
                return []
            points.append((float(point[0]), float(point[1])))

        return points if len(points) >= 3 else []

    def refresh_collision_monitor_params(self):
        if self.zone_topic_seen:
            return

        if self.cm_param_future is not None:
            if not self.cm_param_future.done():
                return

            try:
                response = self.cm_param_future.result()
            except Exception as exc:
                self.get_logger().warn(
                    f'Failed reading collision_monitor polygon params: {exc}'
                )
                self.cm_param_future = None
                return

            self.cm_param_future = None
            if response is None or len(response.values) != 2:
                return

            stop_points = self.parse_points(response.values[0].string_value)
            slowdown_points = self.parse_points(response.values[1].string_value)

            if stop_points:
                self.stop_zone = stop_points
            if slowdown_points:
                self.slowdown_zone = slowdown_points
            if stop_points or slowdown_points:
                self.zone_source = 'collision_monitor_params'
            return

        if not self.cm_param_client.service_is_ready():
            return

        request = GetParameters.Request()
        request.names = ['stop_zone.points', 'slowdown_zone.points']
        self.cm_param_future = self.cm_param_client.call_async(request)

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

    def make_delete_all(self):
        marker = Marker()
        marker.action = Marker.DELETEALL
        return marker

    def publish_markers(self):
        scan_points = self.scan_points()

        msg = MarkerArray()
        msg.markers.extend([
            self.make_delete_all(),
            self.make_outline(1, 'slowdown_zone_outline', self.slowdown_zone, 0.006, (1.0, 0.85, 0.0, 1.0), 0.008),
            self.make_outline(2, 'stop_zone_outline', self.stop_zone, 0.010, (1.0, 0.0, 0.0, 1.0), 0.010),
            self.make_outline(3, 'base_footprint_outline', self.base_footprint, 0.014, (0.2, 0.8, 1.0, 1.0), 0.008),
            self.make_scan_marker(4, scan_points),
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

        cm_topics = 'seen' if self.zone_topic_seen else 'silent'
        text = (
            f'{state} | zone_source={self.zone_source} '
            f'cm_polygon_topics={cm_topics} | '
            f'scan_points={len(scan_points)} stop_hits={stop_hits} '
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
