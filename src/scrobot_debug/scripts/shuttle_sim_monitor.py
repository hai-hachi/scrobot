#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class ShuttleSimMonitor(Node):
    """Monitor isolated shuttle stability and collector-envelope intersection."""

    def __init__(self):
        super().__init__('shuttle_sim_monitor')

        self.declare_parameter('ground_truth_topic', '/evaluation/shuttle_ground_truth')
        self.declare_parameter('collected_topic', '/evaluation/shuttle_collected')
        self.declare_parameter('robot_odom_topic', '/evaluation/ground_truth_odom')

        self.declare_parameter('pickup_offset_x', 0.165)
        self.declare_parameter('pickup_half_length', 0.030)
        self.declare_parameter('pickup_half_width', 0.150)
        self.declare_parameter('shuttle_radius', 0.034)

        self.declare_parameter('movement_warning_threshold', 0.001)
        self.declare_parameter('report_rate', 2.0)

        self.pickup_offset_x = float(
            self.get_parameter('pickup_offset_x').value
        )
        self.pickup_half_length = float(
            self.get_parameter('pickup_half_length').value
        )
        self.pickup_half_width = float(
            self.get_parameter('pickup_half_width').value
        )
        self.shuttle_radius = float(
            self.get_parameter('shuttle_radius').value
        )
        self.movement_warning_threshold = float(
            self.get_parameter('movement_warning_threshold').value
        )
        report_rate = max(0.1, float(self.get_parameter('report_rate').value))

        self.robot_pose = None
        self.shuttles = []
        self.initial_single_pose = None
        self.last_count = None
        self.collection_events = 0

        self.create_subscription(
            PoseArray,
            str(self.get_parameter('ground_truth_topic').value),
            self._ground_truth_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter('collected_topic').value),
            self._collected_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('robot_odom_topic').value),
            self._odom_cb,
            qos_profile_sensor_data,
        )

        self.create_timer(1.0 / report_rate, self._report)

        self.get_logger().info(
            'Shuttle simulation monitor ready: '
            f'collector center x={self.pickup_offset_x:.3f} m, '
            f'half-size=({self.pickup_half_length:.3f}, '
            f'{self.pickup_half_width:.3f}) m, '
            f'shuttle radius={self.shuttle_radius:.3f} m.'
        )

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_pose = (
            float(p.x),
            float(p.y),
            yaw_from_quaternion(q),
        )

    def _ground_truth_cb(self, msg):
        self.shuttles = [
            (
                float(p.position.x),
                float(p.position.y),
                float(p.position.z),
            )
            for p in msg.poses
        ]

        count = len(self.shuttles)
        if count != self.last_count:
            self.get_logger().info(
                f'Shuttle ground-truth count: {count}'
            )
            self.last_count = count

        if count == 1 and self.initial_single_pose is None:
            self.initial_single_pose = self.shuttles[0]
            p = self.initial_single_pose
            self.get_logger().info(
                'Single-shuttle reference pose: '
                f'x={p[0]:.4f}, y={p[1]:.4f}, z={p[2]:.4f} m.'
            )

    def _collected_cb(self, msg):
        if not msg.poses:
            return

        self.collection_events += len(msg.poses)
        for pose in msg.poses:
            p = pose.position
            self.get_logger().info(
                'COLLECTED event: '
                f'x={p.x:.4f}, y={p.y:.4f}, z={p.z:.4f} m.'
            )

        self.get_logger().info(
            f'Total collection events observed: {self.collection_events}'
        )

    def _report(self):
        if self.robot_pose is None or not self.shuttles:
            return

        rx, ry, yaw = self.robot_pose
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        nearest = None
        for sx, sy, sz in self.shuttles:
            world_dx = sx - rx
            world_dy = sy - ry
            local_x = cos_yaw * world_dx + sin_yaw * world_dy
            local_y = -sin_yaw * world_dx + cos_yaw * world_dy
            distance = math.hypot(local_x, local_y)

            if nearest is None or distance < nearest[0]:
                nearest = (distance, local_x, local_y, sx, sy, sz)

        _, local_x, local_y, sx, sy, sz = nearest

        dx = max(
            abs(local_x - self.pickup_offset_x)
            - self.pickup_half_length,
            0.0,
        )
        dy = max(
            abs(local_y) - self.pickup_half_width,
            0.0,
        )
        separation = math.hypot(dx, dy)
        clearance = separation - self.shuttle_radius
        expected_pickup = clearance <= 0.0

        movement_text = ''
        if len(self.shuttles) == 1 and self.initial_single_pose is not None:
            ix, iy, iz = self.initial_single_pose
            drift = math.sqrt(
                (sx - ix) ** 2
                + (sy - iy) ** 2
                + (sz - iz) ** 2
            )
            movement_text = f', shuttle_drift={drift:.4f} m'
            if drift > self.movement_warning_threshold:
                movement_text += ' [MOVEMENT WARNING]'

        self.get_logger().info(
            'Nearest shuttle: '
            f'local=({local_x:+.4f}, {local_y:+.4f}) m, '
            f'pickup_clearance={clearance:+.4f} m, '
            f'expected_pickup={"YES" if expected_pickup else "NO"}'
            f'{movement_text}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ShuttleSimMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
