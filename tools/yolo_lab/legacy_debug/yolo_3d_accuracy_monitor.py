#!/usr/bin/env python3
"""Compare YOLO+depth shuttle 3D estimates against Gazebo ground truth."""

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


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


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


def vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def compose_pose(parent_t, parent_q, child_t, child_q):
    out_t = vec_add(parent_t, quat_rotate(parent_q, child_t))
    out_q = quat_normalize(quat_multiply(parent_q, child_q))
    return out_t, out_q


def transform_point_inverse(frame_t, frame_q, point_world):
    return quat_rotate(
        quat_conjugate(quat_normalize(frame_q)),
        vec_sub(point_world, frame_t),
    )


def transform_to_tuple(transform):
    t = transform.translation
    q = transform.rotation
    return (
        (float(t.x), float(t.y), float(t.z)),
        quat_normalize((float(q.x), float(q.y), float(q.z), float(q.w))),
    )


def pose_to_tuple(pose):
    p = pose.position
    q = pose.orientation
    return (
        (float(p.x), float(p.y), float(p.z)),
        quat_normalize((float(q.x), float(q.y), float(q.z), float(q.w))),
    )


def detection_position(detection):
    if detection.results:
        p = detection.results[0].pose.pose.position
        score = float(detection.results[0].hypothesis.score)
    else:
        p = detection.bbox.center.position
        score = 0.0
    return (float(p.x), float(p.y), float(p.z)), score


class RunningStats:
    def __init__(self):
        self.n = 0
        self.sum = 0.0
        self.sum_sq = 0.0
        self.max_abs = 0.0

    def add(self, value):
        value = float(value)
        self.n += 1
        self.sum += value
        self.sum_sq += value * value
        self.max_abs = max(self.max_abs, abs(value))

    @property
    def mean(self):
        return self.sum / self.n if self.n else 0.0

    @property
    def rmse(self):
        return math.sqrt(self.sum_sq / self.n) if self.n else 0.0


class Yolo3dAccuracyMonitor(Node):
    """Evaluate shuttle 3D perception against simulation-only ground truth.

    Detection points are transformed from their reported camera frame into
    base_link using TF. Gazebo shuttle truth is transformed into the same base
    frame using /evaluation/ground_truth_odom plus the static transform from
    the ground-truth odometry child frame to base_link.

    This test is intentionally simulation-only. Ground truth never feeds the
    production detector or mission controller.
    """

    def __init__(self):
        super().__init__('yolo_3d_accuracy_monitor')

        self.declare_parameter(
            'detection_topic',
            '/perception/shuttle_detections_3d',
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
        self.declare_parameter('report_rate', 1.0)
        self.declare_parameter('stale_timeout', 2.0)

        self.detection_topic = str(
            self.get_parameter('detection_topic').value
        )
        self.shuttle_gt_topic = str(
            self.get_parameter('shuttle_ground_truth_topic').value
        )
        self.gt_odom_topic = str(
            self.get_parameter('ground_truth_odom_topic').value
        )
        self.base_frame = str(self.get_parameter('base_frame').value)
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
        self.last_detection_wall = None
        self.last_sample = None
        self.last_detection_count = 0

        self.stats = {
            'ex': RunningStats(),
            'ey': RunningStats(),
            'planar': RunningStats(),
            'range': RunningStats(),
            'bearing': RunningStats(),
            'longitudinal': RunningStats(),
            'lateral': RunningStats(),
        }

        self.create_subscription(
            Odometry,
            self.gt_odom_topic,
            self._ground_truth_odom_cb,
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
            self.detection_topic,
            self._detection_cb,
            qos_profile_sensor_data,
        )

        self.create_timer(1.0 / self.report_rate, self._report)

        self.get_logger().info(
            'YOLO 3D accuracy monitor ready: '
            f'detections={self.detection_topic}, '
            f'shuttle_gt={self.shuttle_gt_topic}, '
            f'robot_gt={self.gt_odom_topic}, '
            f'comparison_frame={self.base_frame}.'
        )

    def _ground_truth_odom_cb(self, msg):
        self.robot_pose_world = pose_to_tuple(msg.pose.pose)
        self.robot_child_frame = (
            msg.child_frame_id.strip() or 'base_footprint'
        )

    def _shuttle_gt_cb(self, msg):
        self.shuttle_world = [
            (
                float(p.position.x),
                float(p.position.y),
                float(p.position.z),
            )
            for p in msg.poses
        ]

    def _world_to_base_pose(self):
        if self.robot_pose_world is None:
            return None

        world_to_child_t, world_to_child_q = self.robot_pose_world
        child = self.robot_child_frame or 'base_footprint'

        if child == self.base_frame:
            return world_to_child_t, world_to_child_q

        try:
            tf_child_base = self.tf_buffer.lookup_transform(
                child,
                self.base_frame,
                Time(),
            )
        except TransformException:
            return None

        child_to_base_t, child_to_base_q = transform_to_tuple(
            tf_child_base.transform
        )
        return compose_pose(
            world_to_child_t,
            world_to_child_q,
            child_to_base_t,
            child_to_base_q,
        )

    def _ground_truth_points_base(self):
        world_to_base = self._world_to_base_pose()
        if world_to_base is None:
            return []

        world_to_base_t, world_to_base_q = world_to_base
        return [
            transform_point_inverse(
                world_to_base_t,
                world_to_base_q,
                point_world,
            )
            for point_world in self.shuttle_world
        ]

    def _detection_points_base(self, msg):
        source_frame = msg.header.frame_id.strip()
        if not source_frame and msg.detections:
            source_frame = msg.detections[0].header.frame_id.strip()
        if not source_frame:
            return []

        if source_frame == self.base_frame:
            transform = None
        else:
            try:
                stamped = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    source_frame,
                    Time(),
                )
                transform = transform_to_tuple(stamped.transform)
            except TransformException:
                return []

        output = []
        for detection in msg.detections:
            point_source, score = detection_position(detection)

            if transform is None:
                point_base = point_source
            else:
                base_to_source_t, base_to_source_q = transform
                point_base = vec_add(
                    base_to_source_t,
                    quat_rotate(base_to_source_q, point_source),
                )

            output.append((point_base, score))

        return output

    @staticmethod
    def _error_sample(gt, est, score):
        gx, gy, _gz = gt
        ex_est, ey_est, _ez_est = est

        dx = ex_est - gx
        dy = ey_est - gy

        gt_range = math.hypot(gx, gy)
        est_range = math.hypot(ex_est, ey_est)
        gt_bearing = math.atan2(gy, gx)
        est_bearing = math.atan2(ey_est, ex_est)

        if gt_range > 1e-9:
            ux = gx / gt_range
            uy = gy / gt_range
            longitudinal = dx * ux + dy * uy
            lateral = -dx * uy + dy * ux
        else:
            longitudinal = dx
            lateral = dy

        return {
            'gt': gt,
            'est': est,
            'score': score,
            'ex': dx,
            'ey': dy,
            'planar': math.hypot(dx, dy),
            'range': est_range - gt_range,
            'bearing': wrap_angle(est_bearing - gt_bearing),
            'longitudinal': longitudinal,
            'lateral': lateral,
            'gt_range': gt_range,
            'est_range': est_range,
            'gt_bearing': gt_bearing,
            'est_bearing': est_bearing,
        }

    def _detection_cb(self, msg):
        self.last_detection_wall = time.perf_counter()
        self.last_detection_count = len(msg.detections)

        gt_points = self._ground_truth_points_base()
        estimated = self._detection_points_base(msg)

        if not gt_points or not estimated:
            return

        # This launch intentionally uses one shuttle. If YOLO produces multiple
        # boxes, associate the estimate nearest the single Gazebo truth point.
        # This keeps the metric focused on 3D measurement error rather than
        # false-positive classification during this isolated geometry test.
        gt = gt_points[0]
        est, score = min(
            estimated,
            key=lambda item: math.sqrt(
                (item[0][0] - gt[0]) ** 2
                + (item[0][1] - gt[1]) ** 2
                + (item[0][2] - gt[2]) ** 2
            ),
        )

        sample = self._error_sample(gt, est, score)
        self.last_sample = sample

        for key in (
            'ex',
            'ey',
            'planar',
            'range',
            'bearing',
            'longitudinal',
            'lateral',
        ):
            self.stats[key].add(sample[key])

    def _state(self):
        if self.last_detection_wall is None:
            return 'WAIT'
        age = time.perf_counter() - self.last_detection_wall
        if age > self.stale_timeout:
            return f'STALE({age:.1f}s)'
        return 'OK'

    def _report(self):
        sample = self.last_sample
        if sample is None:
            self.get_logger().info(
                '[YOLO 3D ACC] '
                f'state={self._state()} detections={self.last_detection_count} '
                f'gt_shuttles={len(self.shuttle_world)} samples=0'
            )
            return

        gt = sample['gt']
        est = sample['est']
        bearing_deg = math.degrees(sample['bearing'])

        self.get_logger().info(
            '[YOLO 3D ACC] '
            f'state={self._state()} n={self.stats["planar"].n} '
            f'conf={sample["score"]:.3f} | '
            f'GT_base=({gt[0]:+.4f},{gt[1]:+.4f})m '
            f'r={sample["gt_range"]:.4f}m '
            f'b={math.degrees(sample["gt_bearing"]):+.2f}deg | '
            f'EST_base=({est[0]:+.4f},{est[1]:+.4f})m '
            f'r={sample["est_range"]:.4f}m '
            f'b={math.degrees(sample["est_bearing"]):+.2f}deg | '
            f'ex={sample["ex"]:+.4f}m '
            f'ey={sample["ey"]:+.4f}m '
            f'planar={sample["planar"]:.4f}m '
            f'drange={sample["range"]:+.4f}m '
            f'dbearing={bearing_deg:+.2f}deg '
            f'elong={sample["longitudinal"]:+.4f}m '
            f'elat={sample["lateral"]:+.4f}m'
        )

        self.get_logger().info(
            '[YOLO 3D RMSE] '
            f'n={self.stats["planar"].n} '
            f'ex={self.stats["ex"].rmse:.4f}m '
            f'ey={self.stats["ey"].rmse:.4f}m '
            f'planar={self.stats["planar"].rmse:.4f}m '
            f'range={self.stats["range"].rmse:.4f}m '
            f'bearing={math.degrees(self.stats["bearing"].rmse):.2f}deg '
            f'long={self.stats["longitudinal"].rmse:.4f}m '
            f'lat={self.stats["lateral"].rmse:.4f}m '
            f'max_planar={self.stats["planar"].max_abs:.4f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = Yolo3dAccuracyMonitor()
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
