#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def rotate_local_z(q, distance):
    """Rotate (0, 0, distance) by quaternion q."""
    # Third column of the quaternion rotation matrix.
    x = 2.0 * (q.x * q.z + q.w * q.y) * distance
    y = 2.0 * (q.y * q.z - q.w * q.x) * distance
    z = (1.0 - 2.0 * (q.x * q.x + q.y * q.y)) * distance
    return x, y, z


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class CollectionTestMonitor(Node):
    """Report the exact shuttle-center geometry used by collection tests."""

    def __init__(self):
        super().__init__('collection_test_monitor')

        self.declare_parameter('center_offset_z', 0.045)
        self.declare_parameter('pickup_offset_x', 0.165)
        self.declare_parameter('pickup_half_length', 0.030)
        self.declare_parameter('pickup_half_width', 0.150)
        self.declare_parameter('report_rate', 2.0)
        self.declare_parameter('event_topic', '/debug/collection_test')

        self.center_offset_z = float(
            self.get_parameter('center_offset_z').value
        )
        self.pickup_offset_x = float(
            self.get_parameter('pickup_offset_x').value
        )
        self.pickup_half_length = float(
            self.get_parameter('pickup_half_length').value
        )
        self.pickup_half_width = float(
            self.get_parameter('pickup_half_width').value
        )
        self.report_rate = max(
            0.2, float(self.get_parameter('report_rate').value)
        )
        self.event_topic = str(
            self.get_parameter('event_topic').value
        )

        self.publisher = self.create_publisher(
            String, self.event_topic, 50
        )

        self.create_subscription(
            Odometry,
            '/evaluation/ground_truth_odom',
            self._robot_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseArray,
            '/evaluation/shuttle_ground_truth',
            self._shuttle_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseArray,
            '/evaluation/shuttle_collected',
            self._collected_cb,
            qos_profile_sensor_data,
        )

        self.robot_pose = None
        self.shuttles = []
        self.last_count = None
        self.total_collected = 0

        self.create_timer(1.0 / self.report_rate, self._report)

        self._emit(
            'READY '
            'center=origin+local_Z*0.045m '
            'pickup=0.300x0.060m '
            'collector_center=(+0.165,0.000)m'
        )

    def _emit(self, text):
        msg = String()
        msg.data = str(text)
        self.publisher.publish(msg)

    def _robot_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_pose = (
            float(p.x),
            float(p.y),
            yaw_from_quaternion(q),
        )

    def _center_world(self, pose):
        offset = rotate_local_z(
            pose.orientation, self.center_offset_z
        )
        return (
            float(pose.position.x) + offset[0],
            float(pose.position.y) + offset[1],
            float(pose.position.z) + offset[2],
        )

    def _center_robot(self, center):
        if self.robot_pose is None:
            return None

        rx, ry, ryaw = self.robot_pose
        dx = center[0] - rx
        dy = center[1] - ry
        c = math.cos(ryaw)
        s = math.sin(ryaw)

        return (
            c * dx + s * dy,
            -s * dx + c * dy,
            center[2],
        )

    def _inside(self, local):
        return (
            abs(local[0] - self.pickup_offset_x)
            <= self.pickup_half_length
            and abs(local[1]) <= self.pickup_half_width
        )

    def _margins(self, local):
        x_margin = self.pickup_half_length - abs(
            local[0] - self.pickup_offset_x
        )
        y_margin = self.pickup_half_width - abs(local[1])
        return x_margin, y_margin

    def _shuttle_cb(self, msg):
        self.shuttles = list(msg.poses)
        count = len(self.shuttles)
        if count != self.last_count:
            self._emit(f'GT_COUNT shuttles={count}')
            self.last_count = count

    def _collected_cb(self, msg):
        for pose in msg.poses:
            center = self._center_world(pose)
            local = self._center_robot(center)
            self.total_collected += 1

            if local is None:
                self._emit(
                    f'COLLECTED total={self.total_collected} '
                    'robot_pose=UNKNOWN'
                )
                continue

            self._emit(
                f'COLLECTED total={self.total_collected} '
                f'center_robot=({local[0]:+.4f},'
                f'{local[1]:+.4f},{local[2]:+.4f})m'
            )

    def _report(self):
        if self.robot_pose is None:
            return

        if not self.shuttles:
            return

        candidates = []
        for pose in self.shuttles:
            center = self._center_world(pose)
            local = self._center_robot(center)
            if local is None:
                continue

            distance_to_pickup_center = math.hypot(
                local[0] - self.pickup_offset_x,
                local[1],
            )
            candidates.append(
                (distance_to_pickup_center, pose, center, local)
            )

        if not candidates:
            return

        _, pose, center, local = min(
            candidates, key=lambda item: item[0]
        )
        x_margin, y_margin = self._margins(local)
        inside = self._inside(local)

        q = pose.orientation
        self._emit(
            f'GEOM center_robot=({local[0]:+.4f},'
            f'{local[1]:+.4f},{local[2]:+.4f})m '
            f'origin_world=({pose.position.x:+.4f},'
            f'{pose.position.y:+.4f},{pose.position.z:+.4f})m '
            f'q=({q.x:+.3f},{q.y:+.3f},{q.z:+.3f},{q.w:+.3f}) '
            f'x_margin={x_margin:+.4f}m '
            f'y_margin={y_margin:+.4f}m '
            f'expected_collect={"YES" if inside else "NO"}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CollectionTestMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
