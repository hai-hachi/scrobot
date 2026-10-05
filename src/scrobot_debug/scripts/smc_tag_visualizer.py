#!/usr/bin/env python3
"""Lightweight RViz data for the isolated AprilTag SMC test."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import (
    concatenate_matrices,
    euler_from_quaternion,
    inverse_matrix,
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_from_matrix,
    translation_matrix,
)
from visualization_msgs.msg import Marker


def xyz_rpy_to_matrix(xyz, rpy):
    quaternion = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    return concatenate_matrices(
        translation_matrix(xyz),
        quaternion_matrix(quaternion),
    )


def transform_to_matrix(transform):
    translation = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ]
    quaternion = [
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    ]
    return concatenate_matrices(
        translation_matrix(translation),
        quaternion_matrix(quaternion),
    )


class SmcTagVisualizer(Node):
    def __init__(self):
        super().__init__('smc_tag_visualizer')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('mount_frame_prefix', 'tag_mount_')
        self.declare_parameter('tag_id', 0)
        self.declare_parameter('target_distance', 0.90)
        self.declare_parameter('tag_size', 0.10)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('path_publish_rate', 5.0)
        self.declare_parameter('path_min_distance', 0.02)
        self.declare_parameter('path_min_yaw_deg', 1.0)
        self.declare_parameter('max_path_poses', 2500)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.mount_frame_prefix = str(
            self.get_parameter('mount_frame_prefix').value
        )
        self.tag_id = int(self.get_parameter('tag_id').value)
        self.target_distance = float(
            self.get_parameter('target_distance').value
        )
        self.tag_size = float(self.get_parameter('tag_size').value)
        publish_rate = max(
            1.0, float(self.get_parameter('publish_rate').value)
        )
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
            '/debug/smc_tag/trajectory',
            latched_qos,
        )
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/debug/smc_tag/desired_base_pose',
            latched_qos,
        )
        self.tag_pub = self.create_publisher(
            Marker,
            '/debug/smc_tag/tag_marker',
            latched_qos,
        )

        self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self._odom_cb,
            odom_qos,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path = Path()
        self.path.header.frame_id = self.odom_frame
        self.last_path_pose = None

        self.create_timer(1.0 / publish_rate, self._publish_target_geometry)
        self.create_timer(1.0 / path_publish_rate, self._publish_path)

        self.get_logger().info(
            'Tag controller RViz helper ready: '
            f'tag={self.tag_id}, base_link goal={self.target_distance:.2f} m, '
            'fixed frame=map.'
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
            last_x, last_y, last_yaw = self.last_path_pose
            distance = math.hypot(
                float(pose.position.x) - last_x,
                float(pose.position.y) - last_y,
            )
            yaw_change = abs(self._angle_error(yaw, last_yaw))
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

    def _publish_path(self):
        if not self.path.poses:
            return
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)

    def _publish_target_geometry(self):
        mount_frame = self.mount_frame_prefix + str(self.tag_id)

        try:
            tf_map_mount = self.tf_buffer.lookup_transform(
                self.map_frame,
                mount_frame,
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return

        T_map_mount = transform_to_matrix(tf_map_mount.transform)

        # Show the ideal base_link pose directly from the known court tag
        # geometry. The rigid camera offset is absorbed into the chosen 0.90 m
        # stand-off, so the SMC controlled-point offset is c = 0.
        T_mount_base_goal = xyz_rpy_to_matrix(
            [self.target_distance, 0.0, 0.0],
            [0.0, 0.0, math.pi],
        )
        T_map_base_goal = T_map_mount @ T_mount_base_goal

        now = self.get_clock().now().to_msg()

        tag_marker = Marker()
        tag_marker.header.frame_id = self.map_frame
        tag_marker.header.stamp = now
        tag_marker.ns = 'smc_tag'
        tag_marker.id = 0
        tag_marker.type = Marker.CUBE
        tag_marker.action = Marker.ADD
        tag_marker.pose.position.x = float(T_map_mount[0, 3])
        tag_marker.pose.position.y = float(T_map_mount[1, 3])
        tag_marker.pose.position.z = float(T_map_mount[2, 3])
        tag_q = quaternion_from_matrix(T_map_mount)
        tag_marker.pose.orientation.x = float(tag_q[0])
        tag_marker.pose.orientation.y = float(tag_q[1])
        tag_marker.pose.orientation.z = float(tag_q[2])
        tag_marker.pose.orientation.w = float(tag_q[3])
        tag_marker.scale.x = 0.015
        tag_marker.scale.y = self.tag_size
        tag_marker.scale.z = self.tag_size
        tag_marker.color.r = 1.0
        tag_marker.color.g = 1.0
        tag_marker.color.b = 1.0
        tag_marker.color.a = 1.0
        tag_marker.frame_locked = False
        self.tag_pub.publish(tag_marker)

        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = now
        goal.pose.position.x = float(T_map_base_goal[0, 3])
        goal.pose.position.y = float(T_map_base_goal[1, 3])
        goal.pose.position.z = 0.03
        goal_q = quaternion_from_matrix(T_map_base_goal)
        goal.pose.orientation.x = float(goal_q[0])
        goal.pose.orientation.y = float(goal_q[1])
        goal.pose.orientation.z = float(goal_q[2])
        goal.pose.orientation.w = float(goal_q[3])
        self.goal_pub.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    node = SmcTagVisualizer()
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
