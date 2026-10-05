#!/usr/bin/env python3
"""RViz helper for the isolated shuttle pure-SMC test."""

import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker


class SmcShuttleVisualizer(Node):
    def __init__(self):
        super().__init__('smc_shuttle_visualizer')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('path_publish_rate', 5.0)
        self.declare_parameter('path_min_distance', 0.02)
        self.declare_parameter('path_min_yaw_deg', 1.0)
        self.declare_parameter('max_path_poses', 2500)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        path_publish_rate = max(
            1.0, float(self.get_parameter('path_publish_rate').value)
        )
        self.path_min_distance = max(
            0.0, float(self.get_parameter('path_min_distance').value)
        )
        self.path_min_yaw = math.radians(
            float(self.get_parameter('path_min_yaw_deg').value)
        )
        self.max_path_poses = max(
            100, int(self.get_parameter('max_path_poses').value)
        )

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.path_pub = self.create_publisher(
            Path,
            '/debug/smc_shuttle/trajectory',
            latched_qos,
        )
        self.marker_pub = self.create_publisher(
            Marker,
            '/debug/smc_shuttle/target_marker',
            latched_qos,
        )
        self.pre_pose_pub = self.create_publisher(
            PoseStamped,
            '/debug/smc_shuttle/pre_pose_view',
            latched_qos,
        )

        self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self._odom_cb,
            odom_qos,
        )
        self.create_subscription(
            PointStamped,
            '/debug/smc_shuttle/target',
            self._target_cb,
            latched_qos,
        )
        self.create_subscription(
            PoseStamped,
            '/debug/smc_shuttle/pre_pose',
            self._pre_pose_cb,
            latched_qos,
        )

        self.path = Path()
        self.path.header.frame_id = self.odom_frame
        self.last_path_pose = None
        self.last_target = None
        self.last_pre_pose = None

        self.create_timer(1.0 / path_publish_rate, self._publish_path)
        self.create_timer(0.2, self._republish_geometry)

        self.get_logger().info(
            'Shuttle SMC RViz helper ready: '
            '/odometry/filtered -> trajectory, frozen target + pre-pose latched.'
        )

    @staticmethod
    def _yaw_from_quaternion(q):
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return float(yaw)

    @staticmethod
    def _angle_error(a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def _odom_cb(self, msg):
        pose = msg.pose.pose
        yaw = self._yaw_from_quaternion(pose.orientation)

        if self.last_path_pose is not None:
            lx, ly, lyaw = self.last_path_pose
            distance = math.hypot(
                float(pose.position.x) - lx,
                float(pose.position.y) - ly,
            )
            yaw_change = abs(self._angle_error(yaw, lyaw))
            if (
                distance < self.path_min_distance
                and yaw_change < self.path_min_yaw
            ):
                return

        stamped = PoseStamped()
        stamped.header.stamp = msg.header.stamp
        stamped.header.frame_id = self.odom_frame
        stamped.pose = pose

        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(stamped)
        if len(self.path.poses) > self.max_path_poses:
            self.path.poses = self.path.poses[-self.max_path_poses:]

        self.last_path_pose = (
            float(pose.position.x),
            float(pose.position.y),
            yaw,
        )

    def _target_cb(self, msg):
        self.last_target = msg
        self._publish_target_marker()

    def _pre_pose_cb(self, msg):
        self.last_pre_pose = msg
        self.pre_pose_pub.publish(msg)

    def _publish_target_marker(self):
        if self.last_target is None:
            return

        marker = Marker()
        marker.header = self.last_target.header
        marker.ns = 'smc_shuttle'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = self.last_target.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.06
        marker.color.r = 1.0
        marker.color.g = 0.45
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def _republish_geometry(self):
        self._publish_target_marker()
        if self.last_pre_pose is not None:
            self.pre_pose_pub.publish(self.last_pre_pose)

    def _publish_path(self):
        if not self.path.poses:
            return
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = SmcShuttleVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
