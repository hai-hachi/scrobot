#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def rotate_vector(q, vector):
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)
    vx, vy, vz = vector
    return (
        (1.0 - 2.0 * (y * y + z * z)) * vx
        + 2.0 * (x * y - z * w) * vy
        + 2.0 * (x * z + y * w) * vz,
        2.0 * (x * y + z * w) * vx
        + (1.0 - 2.0 * (x * x + z * z)) * vy
        + 2.0 * (y * z - x * w) * vz,
        2.0 * (x * z - y * w) * vx
        + 2.0 * (y * z + x * w) * vy
        + (1.0 - 2.0 * (x * x + y * y)) * vz,
    )


def inverse_rotate_vector(q, vector):
    class Q:
        pass

    qi = Q()
    qi.x = -float(q.x)
    qi.y = -float(q.y)
    qi.z = -float(q.z)
    qi.w = float(q.w)
    return rotate_vector(qi, vector)


def rotate_local_z(q, distance):
    return rotate_vector(q, (0.0, 0.0, distance))


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
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('collector_frame', 'collector_link')
        self.declare_parameter('pickup_half_length', 0.030)
        self.declare_parameter('pickup_half_width', 0.150)
        self.declare_parameter('report_rate', 2.0)
        self.declare_parameter('event_topic', '/debug/collection_test')

        self.center_offset_z = float(
            self.get_parameter('center_offset_z').value
        )
        self.base_frame = str(
            self.get_parameter('base_frame').value
        )
        self.collector_frame = str(
            self.get_parameter('collector_frame').value
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
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
        self.max_count_seen = 0
        self.total_collected = 0
        self.removal_passes = 0
        self.pending_collection_wall = None
        self.removal_failure_reported_for = 0

        self.create_timer(1.0 / self.report_rate, self._report)

        self._emit(
            'READY '
            'center=origin+local_Z*0.045m '
            'pickup=0.300x0.060m '
            f'collector_frame={self.collector_frame}'
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
            float(p.z),
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

    def _center_base(self, center):
        if self.robot_pose is None:
            return None

        rx, ry, rz, ryaw = self.robot_pose
        dx = center[0] - rx
        dy = center[1] - ry
        c = math.cos(ryaw)
        s = math.sin(ryaw)

        return (
            c * dx + s * dy,
            -s * dx + c * dy,
            center[2] - rz,
        )

    def _base_to_collector(self, point):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.collector_frame,
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except Exception:
            return None

        t = tf.transform.translation
        delta = (
            point[0] - float(t.x),
            point[1] - float(t.y),
            point[2] - float(t.z),
        )
        return inverse_rotate_vector(tf.transform.rotation, delta)

    def _inside(self, local):
        return (
            abs(local[0]) <= self.pickup_half_length
            and abs(local[1]) <= self.pickup_half_width
        )

    def _margins(self, local):
        return (
            self.pickup_half_length - abs(local[0]),
            self.pickup_half_width - abs(local[1]),
        )

    def _check_removal_result(self):
        if self.last_count is None or self.total_collected <= 0:
            return

        expected_remaining = max(
            0, self.max_count_seen - self.total_collected
        )
        if self.last_count <= expected_remaining:
            if self.removal_passes < self.total_collected:
                self.removal_passes = self.total_collected
                self.pending_collection_wall = None
                self._emit(
                    'REMOVAL_PASS '
                    f'events={self.total_collected} '
                    f'gt_remaining={self.last_count} '
                    f'baseline={self.max_count_seen}'
                )
            return

        if (
            self.pending_collection_wall is not None
            and time.monotonic() - self.pending_collection_wall >= 1.0
            and self.removal_failure_reported_for < self.total_collected
        ):
            self.removal_failure_reported_for = self.total_collected
            self._emit(
                'REMOVAL_FAIL '
                f'events={self.total_collected} '
                f'gt_remaining={self.last_count} '
                f'expected_at_most={expected_remaining}'
            )

    def _shuttle_cb(self, msg):
        self.shuttles = list(msg.poses)
        count = len(self.shuttles)
        self.max_count_seen = max(self.max_count_seen, count)
        if count != self.last_count:
            self._emit(f'GT_COUNT shuttles={count}')
            self.last_count = count
        self._check_removal_result()

    def _collected_cb(self, msg):
        for pose in msg.poses:
            center = self._center_world(pose)
            base = self._center_base(center)
            local = None if base is None else self._base_to_collector(base)
            self.total_collected += 1
            self.pending_collection_wall = time.monotonic()

            if local is None:
                self._emit(
                    f'COLLECTED total={self.total_collected} '
                    'collector_tf=UNKNOWN'
                )
                continue

            self._emit(
                f'COLLECTED total={self.total_collected} '
                f'center_collector=({local[0]:+.4f},'
                f'{local[1]:+.4f},{local[2]:+.4f})m'
            )

        self._check_removal_result()

    def _report(self):
        self._check_removal_result()
        if self.robot_pose is None or not self.shuttles:
            return

        candidates = []
        for pose in self.shuttles:
            center = self._center_world(pose)
            base = self._center_base(center)
            if base is None:
                continue
            local = self._base_to_collector(base)
            if local is None:
                continue

            distance_to_pickup_center = math.hypot(
                local[0], local[1]
            )
            candidates.append(
                (distance_to_pickup_center, pose, base, local)
            )

        if not candidates:
            return

        _, pose, base, local = min(
            candidates, key=lambda item: item[0]
        )
        x_margin, y_margin = self._margins(local)
        inside = self._inside(local)

        q = pose.orientation
        self._emit(
            f'GEOM center_base=({base[0]:+.4f},'
            f'{base[1]:+.4f},{base[2]:+.4f})m '
            f'center_collector=({local[0]:+.4f},'
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
