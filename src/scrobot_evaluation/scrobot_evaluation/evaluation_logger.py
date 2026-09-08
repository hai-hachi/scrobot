#!/usr/bin/env python3

import csv
import math
import os
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import euler_from_quaternion


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return wrap_angle(euler_from_quaternion([q.x, q.y, q.z, q.w])[2])


def nearest_path_metrics(x, y, yaw, points):
    if len(points) < 2:
        return float('nan'), float('nan')

    best_distance = float('inf')
    best_heading_error = float('nan')

    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        vx = x2 - x1
        vy = y2 - y1
        length_sq = vx * vx + vy * vy

        if length_sq < 1e-12:
            continue

        t = ((x - x1) * vx + (y - y1) * vy) / length_sq
        t = max(0.0, min(1.0, t))
        px = x1 + t * vx
        py = y1 + t * vy
        distance = math.hypot(x - px, y - py)

        if distance < best_distance:
            best_distance = distance
            path_heading = math.atan2(vy, vx)
            best_heading_error = wrap_angle(yaw - path_heading)

    if not math.isfinite(best_distance):
        return float('nan'), float('nan')

    return best_distance, best_heading_error


class EvaluationLogger(Node):
    def __init__(self):
        super().__init__('evaluation_logger')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('ground_truth_topic', '/evaluation/ground_truth_tf')
        self.declare_parameter('ground_truth_child_contains', 'scrobot')
        self.declare_parameter('global_plan_topic', '/plan')
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('relocalization_event_topic', '/global_localization/tag_pose_committed')

        self.declare_parameter('ground_truth_x_offset', 0.0)
        self.declare_parameter('ground_truth_y_offset', 0.0)
        self.declare_parameter('ground_truth_yaw_offset', 0.0)

        self.declare_parameter('sample_rate', 20.0)
        self.declare_parameter('tf_timeout', 0.05)
        self.declare_parameter('output_root', '~/scrobot_evaluation_runs')
        self.declare_parameter('run_name', '')

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.ground_truth_topic = str(self.get_parameter('ground_truth_topic').value)
        self.ground_truth_child_contains = str(self.get_parameter('ground_truth_child_contains').value)
        self.global_plan_topic = str(self.get_parameter('global_plan_topic').value)
        self.mission_state_topic = str(self.get_parameter('mission_state_topic').value)
        self.relocalization_event_topic = str(self.get_parameter('relocalization_event_topic').value)

        self.gt_x_offset = float(self.get_parameter('ground_truth_x_offset').value)
        self.gt_y_offset = float(self.get_parameter('ground_truth_y_offset').value)
        self.gt_yaw_offset = float(self.get_parameter('ground_truth_yaw_offset').value)

        sample_rate = float(self.get_parameter('sample_rate').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        output_root = os.path.expanduser(str(self.get_parameter('output_root').value))
        run_name = str(self.get_parameter('run_name').value).strip()

        if not run_name:
            run_name = datetime.now().strftime('%Y%m%d_%H%M%S')

        self.output_dir = Path(output_root) / run_name
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_ground_truth = None
        self.latest_ground_truth_child = ''
        self.latest_plan = []
        self.plan_revision = 0
        self.mission_state = 'UNKNOWN'

        self.total_ground_truth_distance = 0.0
        self.total_estimated_distance = 0.0
        self.distance_since_relocalization = 0.0
        self.last_gt_xy = None
        self.last_est_xy = None
        self.relocalization_count = 0

        self.samples_file = open(self.output_dir / 'samples.csv', 'w', newline='')
        self.samples_writer = csv.writer(self.samples_file)
        self.samples_writer.writerow([
            'ros_time', 'mission_state', 'plan_revision',
            'gt_x', 'gt_y', 'gt_yaw',
            'est_x', 'est_y', 'est_yaw',
            'position_error', 'yaw_error',
            'gt_cross_track_error', 'gt_heading_error',
            'est_cross_track_error', 'est_heading_error',
            'gt_distance_total', 'est_distance_total', 'distance_since_relocalization'
        ])

        self.events_file = open(self.output_dir / 'relocalization_events.csv', 'w', newline='')
        self.events_writer = csv.writer(self.events_file)
        self.events_writer.writerow([
            'ros_time', 'event_index', 'mission_state',
            'committed_x', 'committed_y', 'committed_yaw',
            'distance_since_previous_relocalization'
        ])

        self.plans_file = open(self.output_dir / 'plans.csv', 'w', newline='')
        self.plans_writer = csv.writer(self.plans_file)
        self.plans_writer.writerow(['plan_revision', 'point_index', 'x', 'y'])

        self.gt_sub = self.create_subscription(TFMessage, self.ground_truth_topic, self.ground_truth_callback, 20)
        self.plan_sub = self.create_subscription(NavPath, self.global_plan_topic, self.plan_callback, 10)
        self.state_sub = self.create_subscription(String, self.mission_state_topic, self.state_callback, 10)
        self.event_sub = self.create_subscription(PoseStamped, self.relocalization_event_topic, self.relocalization_event_callback, 10)

        self.position_error_pub = self.create_publisher(Float64, '/evaluation/localization_position_error', 10)
        self.yaw_error_pub = self.create_publisher(Float64, '/evaluation/localization_yaw_error', 10)
        self.gt_cte_pub = self.create_publisher(Float64, '/evaluation/cross_track_error_ground_truth', 10)
        self.est_cte_pub = self.create_publisher(Float64, '/evaluation/cross_track_error_estimated', 10)
        self.gt_pose_pub = self.create_publisher(PoseStamped, '/evaluation/ground_truth_pose', 10)

        self.sample_timer = self.create_timer(1.0 / sample_rate, self.sample)
        self.get_logger().info(f'Evaluation logging to: {self.output_dir}')
        self.get_logger().info(f'Waiting for Gazebo ground truth on {self.ground_truth_topic}')

    def ground_truth_callback(self, msg):
        if not msg.transforms:
            return

        candidates = []
        needle = self.ground_truth_child_contains.lower()

        for transform in msg.transforms:
            child = transform.child_frame_id or ''
            if needle and needle not in child.lower():
                continue
            candidates.append(transform)

        if not candidates:
            return

        transform = min(candidates, key=lambda item: len(item.child_frame_id or ''))
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = yaw_from_quaternion(q)

        c = math.cos(self.gt_yaw_offset)
        s = math.sin(self.gt_yaw_offset)
        x = self.gt_x_offset + c * t.x - s * t.y
        y = self.gt_y_offset + s * t.x + c * t.y
        yaw = wrap_angle(yaw + self.gt_yaw_offset)

        self.latest_ground_truth = (float(x), float(y), float(yaw), q)

        if transform.child_frame_id != self.latest_ground_truth_child:
            self.latest_ground_truth_child = transform.child_frame_id
            self.get_logger().info(f'Using ground-truth entity: {self.latest_ground_truth_child}')

    def plan_callback(self, msg):
        points = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
        if len(points) < 2:
            return

        self.latest_plan = points
        self.plan_revision += 1

        for index, (x, y) in enumerate(points):
            self.plans_writer.writerow([self.plan_revision, index, x, y])
        self.plans_file.flush()

    def state_callback(self, msg):
        self.mission_state = msg.data

    def relocalization_event_callback(self, msg):
        self.relocalization_count += 1
        yaw = yaw_from_quaternion(msg.pose.orientation)

        distance_before_reset = self.distance_since_relocalization
        self.events_writer.writerow([
            self.now_seconds(), self.relocalization_count, self.mission_state,
            msg.pose.position.x, msg.pose.position.y, yaw,
            distance_before_reset
        ])
        self.events_file.flush()

        self.distance_since_relocalization = 0.0
        self.get_logger().info(
            f'Relocalization event #{self.relocalization_count}: '
            f'{distance_before_reset:.2f} m traveled since previous fix; counter reset.'
        )

    def now_seconds(self):
        now = self.get_clock().now().nanoseconds
        return now / 1e9

    def lookup_estimated_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(),
                timeout=Duration(seconds=self.tf_timeout)
            )
        except TransformException:
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        return float(t.x), float(t.y), yaw_from_quaternion(q)

    @staticmethod
    def update_distance(current_xy, previous_xy):
        if previous_xy is None:
            return 0.0
        return math.hypot(current_xy[0] - previous_xy[0], current_xy[1] - previous_xy[1])

    def sample(self):
        if self.latest_ground_truth is None:
            return

        gt_x, gt_y, gt_yaw, _ = self.latest_ground_truth
        estimated = self.lookup_estimated_pose()

        gt_step = self.update_distance((gt_x, gt_y), self.last_gt_xy)
        self.last_gt_xy = (gt_x, gt_y)

        if gt_step < 1.0:
            self.total_ground_truth_distance += gt_step
            self.distance_since_relocalization += gt_step

        est_x = est_y = est_yaw = float('nan')
        position_error = yaw_error = float('nan')
        est_cte = est_heading_error = float('nan')

        if estimated is not None:
            est_x, est_y, est_yaw = estimated
            est_step = self.update_distance((est_x, est_y), self.last_est_xy)
            self.last_est_xy = (est_x, est_y)
            if est_step < 1.0:
                self.total_estimated_distance += est_step

            position_error = math.hypot(est_x - gt_x, est_y - gt_y)
            yaw_error = abs(wrap_angle(est_yaw - gt_yaw))
            est_cte, est_heading_error = nearest_path_metrics(est_x, est_y, est_yaw, self.latest_plan)

            self.publish_float(self.position_error_pub, position_error)
            self.publish_float(self.yaw_error_pub, yaw_error)
            if math.isfinite(est_cte):
                self.publish_float(self.est_cte_pub, est_cte)

        gt_cte, gt_heading_error = nearest_path_metrics(gt_x, gt_y, gt_yaw, self.latest_plan)
        if math.isfinite(gt_cte):
            self.publish_float(self.gt_cte_pub, gt_cte)

        self.publish_ground_truth_pose(gt_x, gt_y, gt_yaw)

        self.samples_writer.writerow([
            self.now_seconds(), self.mission_state, self.plan_revision,
            gt_x, gt_y, gt_yaw,
            est_x, est_y, est_yaw,
            position_error, yaw_error,
            gt_cte, gt_heading_error,
            est_cte, est_heading_error,
            self.total_ground_truth_distance, self.total_estimated_distance,
            self.distance_since_relocalization
        ])
        self.samples_file.flush()

    @staticmethod
    def publish_float(publisher, value):
        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)

    def publish_ground_truth_pose(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.gt_pose_pub.publish(msg)

    def destroy_node(self):
        for handle in [self.samples_file, self.events_file, self.plans_file]:
            try:
                handle.flush()
                handle.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EvaluationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
