#!/usr/bin/env python3
"""Validate the shuttle collection range gate with production YOLO detections."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray


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


def compose_pose(parent_t, parent_q, child_t, child_q):
    return (
        vec_add(parent_t, quat_rotate(parent_q, child_t)),
        quat_normalize(quat_multiply(parent_q, child_q)),
    )


def transform_point_inverse(frame_t, frame_q, point_world):
    delta = (
        point_world[0] - frame_t[0],
        point_world[1] - frame_t[1],
        point_world[2] - frame_t[2],
    )
    return quat_rotate(quat_conjugate(quat_normalize(frame_q)), delta)


def pose_to_tuple(pose):
    p = pose.position
    q = pose.orientation
    return (
        (float(p.x), float(p.y), float(p.z)),
        quat_normalize((float(q.x), float(q.y), float(q.z), float(q.w))),
    )


def transform_to_tuple(transform):
    t = transform.translation
    q = transform.rotation
    return (
        (float(t.x), float(t.y), float(t.z)),
        quat_normalize((float(q.x), float(q.y), float(q.z), float(q.w))),
    )


def detection_position(detection):
    if detection.results:
        p = detection.results[0].pose.pose.position
    else:
        p = detection.bbox.center.position
    return float(p.x), float(p.y), float(p.z)


class YoloRangeGateMonitor(Node):
    """Check whether production YOLO detections pass the mission range gate."""

    def __init__(self):
        super().__init__('yolo_range_gate_monitor')

        self.declare_parameter(
            'raw_topic',
            '/perception/shuttle_detections_3d',
        )
        self.declare_parameter(
            'filtered_topic',
            '/perception/collectable_shuttle_detections_3d',
        )
        self.declare_parameter(
            'shuttle_ground_truth_topic',
            '/evaluation/shuttle_ground_truth',
        )
        self.declare_parameter(
            'ground_truth_odom_topic',
            '/evaluation/ground_truth_odom',
        )
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('min_target_range', 0.50)
        self.declare_parameter('max_target_range', 1.80)
        self.declare_parameter('report_rate', 1.0)
        self.declare_parameter('stale_timeout', 2.0)

        self.raw_topic = str(self.get_parameter('raw_topic').value)
        self.filtered_topic = str(self.get_parameter('filtered_topic').value)
        self.shuttle_gt_topic = str(
            self.get_parameter('shuttle_ground_truth_topic').value
        )
        self.gt_odom_topic = str(
            self.get_parameter('ground_truth_odom_topic').value
        )
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.min_range = float(
            self.get_parameter('min_target_range').value
        )
        self.max_range = float(
            self.get_parameter('max_target_range').value
        )
        self.report_rate = max(
            0.2, float(self.get_parameter('report_rate').value)
        )
        self.stale_timeout = max(
            0.1, float(self.get_parameter('stale_timeout').value)
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot_pose_world = None
        self.robot_child_frame = ''
        self.shuttle_world = []

        self.raw_count = 0
        self.filtered_count = 0
        self.raw_range = None
        self.filtered_range = None
        self.raw_wall = None
        self.filtered_wall = None

        self.create_subscription(
            Odometry,
            self.gt_odom_topic,
            self._odom_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseArray,
            self.shuttle_gt_topic,
            self._shuttle_gt_cb,
            qos_profile_sensor_data,
        )
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

        self.create_timer(1.0 / self.report_rate, self._report)

        self.get_logger().info(
            'YOLO range-gate monitor ready: '
            f'{self.min_range:.2f} <= base_link planar range '
            f'<= {self.max_range:.2f} m.'
        )

    def _odom_cb(self, msg):
        self.robot_pose_world = pose_to_tuple(msg.pose.pose)
        self.robot_child_frame = (
            msg.child_frame_id.strip() or 'base_footprint'
        )

    def _shuttle_gt_cb(self, msg):
        self.shuttle_world = [
            (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            )
            for pose in msg.poses
        ]

    def _world_to_base_pose(self):
        if self.robot_pose_world is None:
            return None

        world_to_child_t, world_to_child_q = self.robot_pose_world
        child = self.robot_child_frame or 'base_footprint'

        if child == self.base_frame:
            return world_to_child_t, world_to_child_q

        try:
            stamped = self.tf_buffer.lookup_transform(
                child,
                self.base_frame,
                Time(),
            )
        except TransformException:
            return None

        child_to_base_t, child_to_base_q = transform_to_tuple(
            stamped.transform
        )
        return compose_pose(
            world_to_child_t,
            world_to_child_q,
            child_to_base_t,
            child_to_base_q,
        )

    def _gt_range(self):
        if not self.shuttle_world:
            return None

        world_to_base = self._world_to_base_pose()
        if world_to_base is None:
            return None

        p = transform_point_inverse(
            world_to_base[0],
            world_to_base[1],
            self.shuttle_world[0],
        )
        return math.hypot(p[0], p[1])

    def _detection_range(self, msg):
        if not msg.detections:
            return None

        source_frame = msg.header.frame_id.strip()
        if not source_frame:
            source_frame = msg.detections[0].header.frame_id.strip()
        if not source_frame:
            return None

        point = detection_position(msg.detections[0])

        if source_frame == self.base_frame:
            point_base = point
        else:
            try:
                stamped = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    source_frame,
                    Time(),
                )
            except TransformException:
                return None

            t, q = transform_to_tuple(stamped.transform)
            point_base = vec_add(t, quat_rotate(q, point))

        return math.hypot(point_base[0], point_base[1])

    def _raw_cb(self, msg):
        self.raw_wall = time.perf_counter()
        self.raw_count = len(msg.detections)
        self.raw_range = self._detection_range(msg)

    def _filtered_cb(self, msg):
        self.filtered_wall = time.perf_counter()
        self.filtered_count = len(msg.detections)
        self.filtered_range = self._detection_range(msg)

    def _fresh(self, stamp):
        return (
            stamp is not None
            and time.perf_counter() - stamp <= self.stale_timeout
        )

    def _inside(self, value):
        return (
            value is not None
            and self.min_range <= value <= self.max_range
        )

    @staticmethod
    def _decision_text(value):
        return 'ACCEPT' if value else 'REJECT'

    def _report(self):
        gt_range = self._gt_range()
        gt_expected = self._inside(gt_range)

        raw_fresh = self._fresh(self.raw_wall)
        filtered_fresh = self._fresh(self.filtered_wall)

        if not raw_fresh or self.raw_count == 0 or self.raw_range is None:
            verdict = 'INCONCLUSIVE_NO_RAW'
            measured_expected = None
        else:
            measured_expected = self._inside(self.raw_range)
            observed_accept = (
                filtered_fresh and self.filtered_count > 0
            )

            gate_internal_ok = observed_accept == measured_expected
            mission_boundary_ok = observed_accept == gt_expected

            if gate_internal_ok and mission_boundary_ok:
                verdict = 'PASS'
            elif gate_internal_ok:
                verdict = 'GATE_OK_BUT_GT_BOUNDARY_MISMATCH'
            else:
                verdict = 'FAIL_FILTER_DECISION'

        gt_text = (
            'WAIT' if gt_range is None else f'{gt_range:.4f}m'
        )
        raw_text = (
            'none' if self.raw_range is None else f'{self.raw_range:.4f}m'
        )
        filtered_text = (
            'none'
            if self.filtered_range is None
            else f'{self.filtered_range:.4f}m'
        )
        measured_text = (
            'UNKNOWN'
            if measured_expected is None
            else self._decision_text(measured_expected)
        )

        self.get_logger().info(
            '[RANGE GATE] '
            f'GT={gt_text} expected={self._decision_text(gt_expected)} | '
            f'RAW n={self.raw_count} range={raw_text} '
            f'measured_decision={measured_text} | '
            f'FILTERED n={self.filtered_count} range={filtered_text} | '
            f'{verdict}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = YoloRangeGateMonitor()
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
