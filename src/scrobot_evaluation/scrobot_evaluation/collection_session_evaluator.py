#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class CollectionSessionEvaluator(Node):
    """Measure total robot path over one complete shuttle-collection mission."""

    def __init__(self):
        super().__init__('collection_session_evaluator')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('ground_truth_topic', '/evaluation/ground_truth_odom')
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('sample_rate', 20.0)
        self.declare_parameter('tf_timeout', 0.02)
        self.declare_parameter(
            'output_root',
            '~/scrobot_ws/evaluation_results/collection_session',
        )
        self.declare_parameter('run_name', '')

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.gt_topic = str(self.get_parameter('ground_truth_topic').value)
        self.state_topic = str(self.get_parameter('mission_state_topic').value)
        self.sample_rate = float(self.get_parameter('sample_rate').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)

        output_root = Path(os.path.expanduser(str(self.get_parameter('output_root').value)))
        run_name = str(self.get_parameter('run_name').value).strip()
        if not run_name:
            run_name = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_dir = output_root / run_name
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_gt = None
        self.state = 'UNKNOWN'
        self.active = False
        self.finalized = False
        self.start_time = None
        self.end_time = None
        self.last_gt = None
        self.last_est = None
        self.gt_distance = 0.0
        self.est_distance = 0.0
        self.position_error_sq = []
        self.gt_path = []
        self.est_path = []

        self.csv_file = open(self.output_dir / 'collection_trajectory.csv', 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'ros_time', 'mission_state',
            'gt_x', 'gt_y', 'est_x', 'est_y',
            'position_error_m',
            'gt_distance_total_m', 'est_distance_total_m',
        ])

        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(Odometry, self.gt_topic, self.gt_callback, sensor_qos)
        self.create_subscription(String, self.state_topic, self.state_callback, state_qos)
        self.timer = self.create_timer(1.0 / max(self.sample_rate, 1.0), self.sample)

        self.get_logger().info(f'Collection session evaluator output: {self.output_dir}')

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def gt_callback(self, msg):
        p = msg.pose.pose.position
        self.latest_gt = (float(p.x), float(p.y))

    def state_callback(self, msg):
        new_state = msg.data.strip() or 'UNKNOWN'
        self.state = new_state

        if not self.active and not self.finalized and new_state not in ('UNKNOWN', 'IDLE'):
            self.active = True
            self.start_time = self.now_s()
            self.get_logger().info(f'Evaluation started at mission state {new_state}.')

        if self.active and new_state in ('COMPLETE', 'ERROR'):
            self.end_time = self.now_s()
            self.finalize(new_state)

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

    def sample(self):
        if not self.active or self.finalized:
            return

        gt = self.latest_gt
        est = self.estimated_xy()
        if gt is None and est is None:
            return

        if gt is not None:
            step = self.step_distance(gt, self.last_gt)
            if step < 1.0:
                self.gt_distance += step
            self.last_gt = gt
            self.gt_path.append(gt)

        if est is not None:
            step = self.step_distance(est, self.last_est)
            if step < 1.0:
                self.est_distance += step
            self.last_est = est
            self.est_path.append(est)

        error = float('nan')
        if gt is not None and est is not None:
            error = math.hypot(est[0] - gt[0], est[1] - gt[1])
            self.position_error_sq.append(error * error)

        self.writer.writerow([
            self.now_s(), self.state,
            gt[0] if gt else '', gt[1] if gt else '',
            est[0] if est else '', est[1] if est else '',
            error,
            self.gt_distance, self.est_distance,
        ])
        self.csv_file.flush()

    def finalize(self, terminal_state='PARTIAL'):
        if self.finalized:
            return
        self.finalized = True
        self.active = False
        if self.end_time is None:
            self.end_time = self.now_s()

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

        self.csv_file.flush()
        with open(self.output_dir / 'summary.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                'terminal_state', 'duration_s', 'samples_with_position_error',
                'ground_truth_path_m', 'estimated_path_m',
                'path_length_error_m', 'position_rmse_m',
            ])
            w.writerow([
                terminal_state, duration, len(self.position_error_sq),
                self.gt_distance, self.est_distance, path_error, rmse,
            ])

        fig, ax = plt.subplots(figsize=(10, 6))
        if self.gt_path:
            gx, gy = zip(*self.gt_path)
            ax.plot(gx, gy, label='Gazebo ground truth', linewidth=2.0)
            ax.scatter([gx[0]], [gy[0]], marker='o', label='Start')
            ax.scatter([gx[-1]], [gy[-1]], marker='x', label='End')
        if self.est_path:
            ex, ey = zip(*self.est_path)
            ax.plot(ex, ey, label='Estimated map->base', linewidth=1.5)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_title(
            f'Collection session path | GT={self.gt_distance:.2f} m | '
            f'EST={self.est_distance:.2f} m | RMSE={rmse:.3f} m'
        )
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig(self.output_dir / 'collection_trajectory.svg', format='svg')
        plt.close(fig)

        self.get_logger().info(
            f'Evaluation finished: state={terminal_state}, GT path={self.gt_distance:.3f} m, '
            f'EST path={self.est_distance:.3f} m, position RMSE={rmse:.3f} m.'
        )

    def destroy_node(self):
        if not self.finalized and (self.active or self.gt_path or self.est_path):
            self.finalize('PARTIAL')
        self.csv_file.close()
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
