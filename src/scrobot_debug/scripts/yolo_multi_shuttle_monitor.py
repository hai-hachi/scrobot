#!/usr/bin/env python3
"""Monitor multi-shuttle YOLO filtering, target locking, and reacquisition."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseArray
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray


def detection_position(detection):
    if detection.results:
        p = detection.results[0].pose.pose.position
        score = float(detection.results[0].hypothesis.score)
    else:
        p = detection.bbox.center.position
        score = 0.0
    return (float(p.x), float(p.y), float(p.z)), score


def quat_normalize(q):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    inv = 1.0 / norm
    return (x * inv, y * inv, z * inv, w * inv)


def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q, vector):
    qn = quat_normalize(q)
    vx, vy, vz = vector
    vq = (vx, vy, vz, 0.0)
    out = quat_multiply(quat_multiply(qn, vq), quat_conjugate(qn))
    return (out[0], out[1], out[2])


def vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def transform_to_tuple(transform):
    t = transform.translation
    q = transform.rotation
    return (
        (float(t.x), float(t.y), float(t.z)),
        quat_normalize((float(q.x), float(q.y), float(q.z), float(q.w))),
    )


class YoloMultiShuttleMonitor(Node):
    """Observe the production multi-shuttle selection chain.

    The launch uses four known world positions labelled A-D. The monitor never
    feeds those labels into perception or control; they are only used to make
    terminal debug output human-readable.
    """

    def __init__(self):
        super().__init__('yolo_multi_shuttle_monitor')

        self.declare_parameter(
            'raw_topic',
            '/perception/shuttle_detections_3d',
        )
        self.declare_parameter(
            'filtered_topic',
            '/perception/collectable_shuttle_detections_3d',
        )
        self.declare_parameter(
            'target_topic',
            '/debug/smc_shuttle/target',
        )
        self.declare_parameter(
            'phase_topic',
            '/mission/local_collect_phase',
        )
        self.declare_parameter(
            'collected_topic',
            '/evaluation/shuttle_collected',
        )
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('report_rate', 1.0)
        self.declare_parameter('association_radius', 0.25)

        self.raw_topic = str(self.get_parameter('raw_topic').value)
        self.filtered_topic = str(
            self.get_parameter('filtered_topic').value
        )
        self.target_topic = str(self.get_parameter('target_topic').value)
        self.phase_topic = str(self.get_parameter('phase_topic').value)
        self.collected_topic = str(
            self.get_parameter('collected_topic').value
        )
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.report_rate = max(
            0.2, float(self.get_parameter('report_rate').value)
        )
        self.association_radius = max(
            0.05, float(self.get_parameter('association_radius').value)
        )

        # Nominal world coordinates created by yolo_multi_collect_check.
        self.labels = {
            'A': (1.25, -0.50),
            'B': (1.45, -0.15),
            'C': (1.60, +0.20),
            'D': (1.75, +0.60),
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.raw_entries = []
        self.filtered_entries = []
        self.gt_count = 0
        self.phase = 'UNKNOWN'
        self.lock_sequence = []
        self.collection_sequence = []
        self.last_target_xy = None

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            Detection3DArray,
            self.raw_topic,
            self._raw_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Detection3DArray,
            self.filtered_topic,
            self._filtered_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointStamped,
            self.target_topic,
            self._target_cb,
            transient_qos,
        )
        self.create_subscription(
            String,
            self.phase_topic,
            self._phase_cb,
            10,
        )
        self.create_subscription(
            PoseArray,
            '/evaluation/shuttle_ground_truth',
            self._gt_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseArray,
            self.collected_topic,
            self._collected_cb,
            qos_profile_sensor_data,
        )

        self.create_timer(1.0 / self.report_rate, self._report)
        self.get_logger().info(
            'Multi-shuttle monitor ready. '
            'Expected initial gate: A/B/C eligible, D rejected.'
        )

    def _lookup(self, target, source):
        try:
            return self.tf_buffer.lookup_transform(
                target,
                source,
                Time(),
            )
        except TransformException:
            return None

    @staticmethod
    def _apply_transform(transform, point):
        t, q = transform_to_tuple(transform.transform)
        return vec_add(t, quat_rotate(q, point))

    def _to_frame(self, point, source_frame, target_frame):
        if source_frame == target_frame:
            return point
        tf = self._lookup(target_frame, source_frame)
        if tf is None:
            return None
        return self._apply_transform(tf, point)

    def _nearest_label_xy(self, x, y):
        label, distance = min(
            (
                (
                    name,
                    math.hypot(x - nominal[0], y - nominal[1]),
                )
                for name, nominal in self.labels.items()
            ),
            key=lambda item: item[1],
        )
        if distance > self.association_radius:
            return '?'
        return label

    def _entries(self, msg):
        source_frame = msg.header.frame_id.strip()
        if not source_frame and msg.detections:
            source_frame = msg.detections[0].header.frame_id.strip()
        if not source_frame:
            return []

        rows = []
        for index, detection in enumerate(msg.detections):
            point, score = detection_position(detection)
            point_base = self._to_frame(
                point,
                source_frame,
                self.base_frame,
            )
            point_odom = self._to_frame(
                point,
                source_frame,
                self.odom_frame,
            )
            if point_base is None:
                continue

            rng = math.hypot(point_base[0], point_base[1])
            bearing = math.degrees(
                math.atan2(point_base[1], point_base[0])
            )
            label = '?'
            if point_odom is not None:
                label = self._nearest_label_xy(
                    point_odom[0],
                    point_odom[1],
                )

            rows.append({
                'index': index,
                'label': label,
                'range': rng,
                'bearing': bearing,
                'score': score,
            })

        return rows

    def _raw_cb(self, msg):
        self.raw_entries = self._entries(msg)

    def _filtered_cb(self, msg):
        self.filtered_entries = self._entries(msg)

    def _phase_cb(self, msg):
        self.phase = str(msg.data)

    def _gt_cb(self, msg):
        self.gt_count = len(msg.poses)

    def _target_cb(self, msg):
        point = (
            float(msg.point.x),
            float(msg.point.y),
            float(msg.point.z),
        )
        source_frame = msg.header.frame_id.strip() or self.odom_frame
        point_odom = self._to_frame(
            point,
            source_frame,
            self.odom_frame,
        )
        if point_odom is None:
            return

        xy = (point_odom[0], point_odom[1])
        if self.last_target_xy is not None:
            if math.hypot(
                xy[0] - self.last_target_xy[0],
                xy[1] - self.last_target_xy[1],
            ) < 0.10:
                return

        self.last_target_xy = xy
        label = self._nearest_label_xy(xy[0], xy[1])
        self.lock_sequence.append(label)
        self.get_logger().info(
            '[TARGET LOCK] '
            f'index={len(self.lock_sequence)} label={label} '
            f'odom=({xy[0]:+.3f},{xy[1]:+.3f})m '
            f'sequence={"->".join(self.lock_sequence)}'
        )

    def _collected_cb(self, msg):
        for pose in msg.poses:
            x = float(pose.position.x)
            y = float(pose.position.y)
            label = self._nearest_label_xy(x, y)
            self.collection_sequence.append(label)
            self.get_logger().info(
                '[COLLECTED] '
                f'index={len(self.collection_sequence)} label={label} '
                f'world=({x:+.3f},{y:+.3f})m '
                f'sequence={"->".join(self.collection_sequence)}'
            )

    @staticmethod
    def _format_entries(entries):
        if not entries:
            return 'none'
        return ' '.join(
            (
                f'{row["index"]}:{row["label"]}'
                f'(r={row["range"]:.2f}m,'
                f'b={row["bearing"]:+.1f}deg,'
                f'c={row["score"]:.2f})'
            )
            for row in entries
        )

    def _initial_verdict(self):
        raw_labels = {row['label'] for row in self.raw_entries}
        filtered_labels = {row['label'] for row in self.filtered_entries}

        raw_ok = {'A', 'B', 'C', 'D'}.issubset(raw_labels)
        filter_ok = (
            {'A', 'B', 'C'}.issubset(filtered_labels)
            and 'D' not in filtered_labels
        )

        if raw_ok and filter_ok:
            return 'INITIAL_PASS'
        if not raw_ok:
            return 'WAIT_RAW_4'
        return 'FILTER_MISMATCH'

    def _report(self):
        locks = 'none' if not self.lock_sequence else '->'.join(
            self.lock_sequence
        )
        collected = (
            'none'
            if not self.collection_sequence
            else '->'.join(self.collection_sequence)
        )

        self.get_logger().info(
            '[MULTI] '
            f'phase={self.phase} gt={self.gt_count} '
            f'raw={len(self.raw_entries)} '
            f'filtered={len(self.filtered_entries)} '
            f'{self._initial_verdict()} | '
            f'locks={locks} collected={collected}'
        )
        self.get_logger().info(
            '[MULTI RAW] ' + self._format_entries(self.raw_entries)
        )
        self.get_logger().info(
            '[MULTI FILTERED] '
            + self._format_entries(self.filtered_entries)
        )


def main(args=None):
    rclpy.init(args=args)
    node = YoloMultiShuttleMonitor()
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
