#!/usr/bin/env python3

import csv
import math
import os
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class CollectionSessionEvaluator(Node):
    """Record end-to-end performance of one shuttle-collection session.

    Gazebo ground truth is used only for evaluation metrics. It never feeds the
    mission controller. Recording starts when the mission first leaves IDLE and
    finishes automatically at COMPLETE / ERROR, or as PARTIAL on Ctrl-C.
    """

    def __init__(self):
        super().__init__('collection_session_evaluator')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('ground_truth_topic', '/evaluation/ground_truth_odom')
        self.declare_parameter('shuttle_ground_truth_topic', '/evaluation/shuttle_ground_truth')
        self.declare_parameter('shuttle_collected_topic', '/evaluation/shuttle_collected')
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('collection_phase_topic', '/mission/collection_phase')
        self.declare_parameter('collection_outcome_topic', '/mission/collection_outcome')
        self.declare_parameter('relocalization_status_topic', '/mission/relocalization_status')
        self.declare_parameter('sample_rate', 20.0)
        self.declare_parameter('path_publish_rate', 1.0)
        self.declare_parameter('tf_timeout', 0.02)
        self.declare_parameter(
            'output_root',
            '~/scrobot_ws/evaluation_results/collection_session',
        )
        self.declare_parameter('run_name', '')

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.gt_topic = str(self.get_parameter('ground_truth_topic').value)
        self.shuttle_gt_topic = str(
            self.get_parameter('shuttle_ground_truth_topic').value
        )
        self.shuttle_collected_topic = str(
            self.get_parameter('shuttle_collected_topic').value
        )
        self.state_topic = str(self.get_parameter('mission_state_topic').value)
        self.collection_phase_topic = str(
            self.get_parameter('collection_phase_topic').value
        )
        self.collection_outcome_topic = str(
            self.get_parameter('collection_outcome_topic').value
        )
        self.relocalization_status_topic = str(
            self.get_parameter('relocalization_status_topic').value
        )
        self.sample_rate = float(self.get_parameter('sample_rate').value)
        self.path_publish_rate = float(self.get_parameter('path_publish_rate').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)

        output_root = Path(
            os.path.expanduser(str(self.get_parameter('output_root').value))
        )
        run_name = str(self.get_parameter('run_name').value).strip()
        if not run_name:
            run_name = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_dir = output_root / run_name
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_gt = None
        self.remaining_shuttles = 0
        self.total_shuttles_seen = 0
        self.collected_count = 0

        self.state = 'UNKNOWN'
        self.collection_phase = 'IDLE'
        self.collection_outcome = 'UNKNOWN'
        self.relocalization_status = 'NORMAL'

        self.active = False
        self.finalized = False
        self.start_time = None
        self.end_time = None
        self.last_gt = None
        self.last_est = None
        self.gt_distance = 0.0
        self.est_distance = 0.0
        self.position_error_sq = []
        self.gt_path_xy = []
        self.est_path_xy = []

        self.state_enter_time = None
        self.state_durations = defaultdict(float)
        self.runtime_relocalizations = 0
        self.collection_passes = 0
        self.outcome_counts = defaultdict(int)

        self.trajectory_file = open(
            self.output_dir / 'collection_trajectory.csv', 'w', newline=''
        )
        self.trajectory_writer = csv.writer(self.trajectory_file)
        self.trajectory_writer.writerow([
            'ros_time', 'elapsed_s', 'mission_state', 'collection_phase',
            'relocalization_status',
            'gt_x', 'gt_y', 'est_x', 'est_y', 'position_error_m',
            'gt_distance_total_m', 'est_distance_total_m',
            'remaining_shuttles', 'collected_shuttles', 'total_shuttles_seen',
            'collection_rate_percent',
        ])

        self.state_file = open(self.output_dir / 'state_events.csv', 'w', newline='')
        self.state_writer = csv.writer(self.state_file)
        self.state_writer.writerow([
            'ros_time', 'elapsed_s', 'from_state', 'to_state',
        ])

        self.collection_file = open(
            self.output_dir / 'collection_events.csv', 'w', newline=''
        )
        self.collection_writer = csv.writer(self.collection_file)
        self.collection_writer.writerow([
            'pass_index', 'ros_time', 'elapsed_s', 'outcome',
            'collected_total', 'remaining_shuttles', 'total_shuttles_seen',
            'collection_rate_percent',
        ])

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(Odometry, self.gt_topic, self.gt_callback, sensor_qos)
        self.create_subscription(
            PoseArray, self.shuttle_gt_topic, self.shuttle_gt_callback, sensor_qos
        )
        self.create_subscription(
            PoseArray,
            self.shuttle_collected_topic,
            self.shuttle_collected_callback,
            sensor_qos,
        )
        self.create_subscription(String, self.state_topic, self.state_callback, state_qos)
        self.create_subscription(
            String,
            self.collection_phase_topic,
            self.collection_phase_callback,
            state_qos,
        )
        self.create_subscription(
            String,
            self.collection_outcome_topic,
            self.collection_outcome_callback,
            state_qos,
        )
        self.create_subscription(
            String,
            self.relocalization_status_topic,
            self.relocalization_status_callback,
            state_qos,
        )

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.gt_path_pub = self.create_publisher(
            NavPath, '/evaluation/ground_truth_path', path_qos
        )
        self.est_path_pub = self.create_publisher(
            NavPath, '/evaluation/estimated_path', path_qos
        )

        self.sample_timer = self.create_timer(
            1.0 / max(self.sample_rate, 1.0), self.sample
        )
        self.path_timer = self.create_timer(
            1.0 / max(self.path_publish_rate, 0.1), self.publish_paths
        )

        self.get_logger().info(
            f'Collection session recorder ready: {self.output_dir}'
        )

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def elapsed_s(self, now=None):
        if self.start_time is None:
            return 0.0
        if now is None:
            now = self.now_s()
        return max(0.0, now - self.start_time)

    def gt_callback(self, msg):
        p = msg.pose.pose.position
        self.latest_gt = (float(p.x), float(p.y))

    def shuttle_gt_callback(self, msg):
        self.remaining_shuttles = len(msg.poses)
        self.total_shuttles_seen = max(
            self.total_shuttles_seen,
            self.remaining_shuttles + self.collected_count,
        )

    def shuttle_collected_callback(self, msg):
        if not msg.poses:
            return
        self.collected_count += len(msg.poses)
        self.total_shuttles_seen = max(
            self.total_shuttles_seen,
            self.remaining_shuttles + self.collected_count,
        )

    def state_callback(self, msg):
        new_state = msg.data.strip() or 'UNKNOWN'
        now = self.now_s()
        previous = self.state

        if not self.active and not self.finalized and new_state not in ('UNKNOWN', 'IDLE'):
            self.active = True
            self.start_time = now
            self.state_enter_time = now
            self.get_logger().info(f'Evaluation started at mission state {new_state}.')

        if self.active and new_state != previous:
            if self.state_enter_time is not None and previous not in ('UNKNOWN', 'IDLE'):
                self.state_durations[previous] += max(0.0, now - self.state_enter_time)
            self.state_writer.writerow([
                now, self.elapsed_s(now), previous, new_state,
            ])
            self.state_file.flush()
            self.state_enter_time = now
            if new_state == 'RUNTIME_TAG_APPROACH':
                self.runtime_relocalizations += 1

        self.state = new_state

        if self.active and new_state in ('COMPLETE', 'ERROR'):
            self.end_time = now
            self.finalize(new_state)

    def collection_phase_callback(self, msg):
        new_phase = msg.data.strip() or 'UNKNOWN'
        previous = self.collection_phase
        self.collection_phase = new_phase

        if self.active and new_phase == 'DONE' and previous != 'DONE':
            self.collection_passes += 1
            outcome = self.collection_outcome
            self.outcome_counts[outcome] += 1
            rate = self.collection_rate_percent()
            now = self.now_s()
            self.collection_writer.writerow([
                self.collection_passes,
                now,
                self.elapsed_s(now),
                outcome,
                self.collected_count,
                self.remaining_shuttles,
                self.total_shuttles_seen,
                rate,
            ])
            self.collection_file.flush()

    def collection_outcome_callback(self, msg):
        self.collection_outcome = msg.data.strip() or 'UNKNOWN'

    def relocalization_status_callback(self, msg):
        self.relocalization_status = msg.data.strip() or 'NORMAL'

    def estimated_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException:
            return None
        t = tf.transform.translation
        return float(t.x), float(t.y)

    @staticmethod
    def step_distance(current, previous):
        if current is None or previous is None:
            return 0.0
        return math.hypot(current[0] - previous[0], current[1] - previous[1])

    def collection_rate_percent(self):
        if self.total_shuttles_seen <= 0:
            return 0.0
        return 100.0 * self.collected_count / float(self.total_shuttles_seen)

    def sample(self):
        if not self.active or self.finalized:
            return

        gt = self.latest_gt
        est = self.estimated_xy()
        if gt is None and est is None:
            return

        if gt is not None:
            step = self.step_distance(gt, self.last_gt)
            if 0.0 <= step < 1.0:
                self.gt_distance += step
            self.last_gt = gt
            self.gt_path_xy.append(gt)

        if est is not None:
            step = self.step_distance(est, self.last_est)
            if 0.0 <= step < 1.0:
                self.est_distance += step
            self.last_est = est
            self.est_path_xy.append(est)

        error = float('nan')
        if gt is not None and est is not None:
            error = math.hypot(est[0] - gt[0], est[1] - gt[1])
            self.position_error_sq.append(error * error)

        now = self.now_s()
        self.trajectory_writer.writerow([
            now,
            self.elapsed_s(now),
            self.state,
            self.collection_phase,
            self.relocalization_status,
            gt[0] if gt else '',
            gt[1] if gt else '',
            est[0] if est else '',
            est[1] if est else '',
            error,
            self.gt_distance,
            self.est_distance,
            self.remaining_shuttles,
            self.collected_count,
            self.total_shuttles_seen,
            self.collection_rate_percent(),
        ])
        self.trajectory_file.flush()

    def _make_nav_path(self, points):
        msg = NavPath()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def publish_paths(self):
        if self.gt_path_xy:
            self.gt_path_pub.publish(self._make_nav_path(self.gt_path_xy))
        if self.est_path_xy:
            self.est_path_pub.publish(self._make_nav_path(self.est_path_xy))

    def finalize(self, terminal_state='PARTIAL'):
        if self.finalized:
            return
        self.finalized = True
        self.active = False
        if self.end_time is None:
            self.end_time = self.now_s()

        if self.state_enter_time is not None and self.state not in ('UNKNOWN', 'IDLE'):
            self.state_durations[self.state] += max(
                0.0, self.end_time - self.state_enter_time
            )

        duration = (
            self.end_time - self.start_time
            if self.start_time is not None
            else float('nan')
        )
        rmse = (
            math.sqrt(sum(self.position_error_sq) / len(self.position_error_sq))
            if self.position_error_sq else float('nan')
        )
        path_error = self.est_distance - self.gt_distance
        rate = self.collection_rate_percent()
        distance_per_collected = (
            self.gt_distance / self.collected_count
            if self.collected_count > 0 else float('nan')
        )
        time_per_collected = (
            duration / self.collected_count
            if self.collected_count > 0 else float('nan')
        )
        runtime_relocalization_time = (
            self.state_durations.get('RUNTIME_TAG_APPROACH', 0.0)
            + self.state_durations.get('RUNTIME_RELOCALIZATION', 0.0)
        )

        for handle in (
            self.trajectory_file,
            self.state_file,
            self.collection_file,
        ):
            handle.flush()

        with open(self.output_dir / 'state_durations.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['state', 'duration_s', 'percent_of_session'])
            for state, state_duration in sorted(
                self.state_durations.items(), key=lambda item: item[1], reverse=True
            ):
                percent = (
                    100.0 * state_duration / duration
                    if duration and math.isfinite(duration) and duration > 0.0 else 0.0
                )
                writer.writerow([state, state_duration, percent])

        with open(self.output_dir / 'summary.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'terminal_state', 'duration_s',
                'total_shuttles_seen', 'collected_shuttles', 'remaining_shuttles',
                'collection_rate_percent', 'collection_passes',
                'outcome_collected', 'outcome_partial', 'outcome_missed',
                'ground_truth_path_m', 'estimated_path_m', 'path_length_error_m',
                'position_rmse_m', 'distance_per_collected_m',
                'time_per_collected_s', 'runtime_relocalizations',
                'runtime_relocalization_time_s',
            ])
            writer.writerow([
                terminal_state,
                duration,
                self.total_shuttles_seen,
                self.collected_count,
                self.remaining_shuttles,
                rate,
                self.collection_passes,
                self.outcome_counts.get('COLLECTED', 0),
                self.outcome_counts.get('PARTIAL', 0),
                self.outcome_counts.get('MISSED', 0),
                self.gt_distance,
                self.est_distance,
                path_error,
                rmse,
                distance_per_collected,
                time_per_collected,
                self.runtime_relocalizations,
                runtime_relocalization_time,
            ])

        self.publish_paths()
        self.get_logger().info(
            'Evaluation finished: '
            f'state={terminal_state}, collected={self.collected_count}/'
            f'{self.total_shuttles_seen} ({rate:.1f}%), '
            f'GT path={self.gt_distance:.2f} m, duration={duration:.1f} s, '
            f'RMSE={rmse:.3f} m, runtime relocalizations={self.runtime_relocalizations}.'
        )
        self.get_logger().info(
            f'Run data saved in: {self.output_dir}'
        )

    def destroy_node(self):
        if not self.finalized and (self.active or self.gt_path_xy or self.est_path_xy):
            self.finalize('PARTIAL')
        for handle in (
            self.trajectory_file,
            self.state_file,
            self.collection_file,
        ):
            if not handle.closed:
                handle.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CollectionSessionEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
