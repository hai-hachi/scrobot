#!/usr/bin/env python3

import csv
import math
import os
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
from tf_transformations import euler_from_quaternion


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return wrap_angle(
        euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    )


def nearest_path_distance(x, y, points):
    if len(points) < 2:
        return float('nan')

    best = float('inf')
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
        best = min(best, math.hypot(x - px, y - py))

    return best if math.isfinite(best) else float('nan')


class PatrolTrajectoryLogger(Node):
    """Log actual/estimated patrol trajectory plus checkpoint/spin events."""

    def __init__(self):
        super().__init__('patrol_trajectory_logger')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter(
            'ground_truth_topic',
            '/evaluation/ground_truth_odom',
        )
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('current_goal_topic', '/mission/current_goal')
        self.declare_parameter('patrol_points_topic', '/mission/patrol_points')
        self.declare_parameter('global_plan_topic', '/plan')
        self.declare_parameter('spin_target_angle', 2.0 * math.pi)
        self.declare_parameter('sample_rate', 20.0)
        self.declare_parameter('tf_timeout', 0.02)
        self.declare_parameter(
            'output_root',
            '~/scrobot_evaluation_runs/patrol',
        )
        self.declare_parameter('run_name', '')
        self.declare_parameter('flush_every_samples', 20)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.ground_truth_topic = str(
            self.get_parameter('ground_truth_topic').value
        )
        self.mission_state_topic = str(
            self.get_parameter('mission_state_topic').value
        )
        self.current_goal_topic = str(
            self.get_parameter('current_goal_topic').value
        )
        self.patrol_points_topic = str(
            self.get_parameter('patrol_points_topic').value
        )
        self.global_plan_topic = str(
            self.get_parameter('global_plan_topic').value
        )
        self.spin_target_angle = float(
            self.get_parameter('spin_target_angle').value
        )
        sample_rate = float(self.get_parameter('sample_rate').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.flush_every_samples = max(
            1,
            int(self.get_parameter('flush_every_samples').value),
        )

        output_root = os.path.expanduser(
            str(self.get_parameter('output_root').value)
        )
        run_name = str(self.get_parameter('run_name').value).strip()
        if not run_name:
            run_name = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_dir = Path(output_root) / run_name
        self.output_dir.mkdir(parents=True, exist_ok=True)

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        self.mission_state = 'UNKNOWN'
        self.previous_state = 'UNKNOWN'
        self.latest_goal = None
        self.patrol_points = []
        self.latest_plan = []
        self.plan_revision = 0

        self.latest_gt = None
        self.gt_received = False
        self.last_gt_xy = None
        self.last_est_xy = None
        self.gt_distance_total = 0.0
        self.est_distance_total = 0.0

        self.nav_start_time = None
        self.nav_start_gt_distance = 0.0
        self.current_goal_index = -1

        self.spin_active = False
        self.spin_start_time = None
        self.spin_start_gt = None
        self.spin_last_yaw = None
        self.spin_signed_rotation = 0.0
        self.spin_abs_rotation = 0.0
        self.spin_peak_progress = 0.0
        self.spin_checkpoint_index = -1

        self.samples_file = open(
            self.output_dir / 'trajectory.csv',
            'w',
            newline='',
        )
        self.samples_writer = csv.writer(self.samples_file)
        self.samples_writer.writerow([
            'ros_time', 'mission_state', 'goal_index', 'plan_revision',
            'gt_x', 'gt_y', 'gt_yaw',
            'est_x', 'est_y', 'est_yaw',
            'localization_position_error', 'localization_yaw_error',
            'gt_plan_cross_track_error', 'est_plan_cross_track_error',
            'gt_distance_total', 'est_distance_total',
        ])

        self.points_file = open(
            self.output_dir / 'patrol_points.csv',
            'w',
            newline='',
        )
        self.points_writer = csv.writer(self.points_file)
        self.points_writer.writerow(['index', 'x', 'y', 'yaw'])

        self.plans_file = open(
            self.output_dir / 'plans.csv',
            'w',
            newline='',
        )
        self.plans_writer = csv.writer(self.plans_file)
        self.plans_writer.writerow(['revision', 'index', 'x', 'y'])

        self.checkpoints_file = open(
            self.output_dir / 'checkpoints.csv',
            'w',
            newline='',
        )
        self.checkpoints_writer = csv.writer(self.checkpoints_file)
        self.checkpoints_writer.writerow([
            'index', 'ros_time', 'navigation_time_s',
            'navigation_path_length_m',
            'goal_x', 'goal_y', 'goal_yaw',
            'gt_x', 'gt_y', 'gt_yaw',
            'gt_position_error_m', 'gt_yaw_error_deg',
            'est_x', 'est_y', 'est_yaw',
            'est_position_error_m', 'est_yaw_error_deg',
        ])

        self.spins_file = open(
            self.output_dir / 'spins.csv',
            'w',
            newline='',
        )
        self.spins_writer = csv.writer(self.spins_file)
        self.spins_writer.writerow([
            'index', 'start_time', 'end_time', 'duration_s',
            'target_rotation_deg', 'net_rotation_deg',
            'absolute_rotation_deg', 'peak_progress_deg',
            'overshoot_deg', 'correction_after_peak_deg',
            'start_x', 'start_y', 'end_x', 'end_y',
            'position_drift_m',
        ])

        self.sample_count = 0

        self.create_subscription(
            Odometry,
            self.ground_truth_topic,
            self.ground_truth_callback,
            sensor_qos,
        )
        self.create_subscription(
            String,
            self.mission_state_topic,
            self.state_callback,
            state_qos,
        )
        self.create_subscription(
            PoseStamped,
            self.current_goal_topic,
            self.goal_callback,
            state_qos,
        )
        self.create_subscription(
            PoseArray,
            self.patrol_points_topic,
            self.patrol_points_callback,
            state_qos,
        )
        self.create_subscription(
            NavPath,
            self.global_plan_topic,
            self.plan_callback,
            sensor_qos,
        )

        self.sample_timer = self.create_timer(
            1.0 / sample_rate,
            self.sample,
        )

        self.get_logger().info(
            f'Patrol trajectory logging to: {self.output_dir}'
        )
        self.get_logger().info(
            f'Waiting for dedicated Gazebo ground truth odometry on '
            f'{self.ground_truth_topic}.'
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def ground_truth_callback(self, msg):
        # Same dedicated Gazebo OdometryPublisher path used by local_odom_logger.
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        pose = (float(p.x), float(p.y), yaw_from_quaternion(q))
        self.latest_gt = pose

        if not self.gt_received:
            self.gt_received = True
            self.get_logger().info(
                f'Receiving Gazebo ground truth from {self.ground_truth_topic}.'
            )

        if self.spin_active:
            self.update_spin_rotation(pose[2])

    def update_spin_rotation(self, yaw):
        if self.spin_last_yaw is None:
            self.spin_last_yaw = yaw
            return

        delta = wrap_angle(yaw - self.spin_last_yaw)
        self.spin_last_yaw = yaw
        self.spin_signed_rotation += delta
        self.spin_abs_rotation += abs(delta)

        direction = 1.0 if self.spin_target_angle >= 0.0 else -1.0
        progress = direction * self.spin_signed_rotation
        self.spin_peak_progress = max(self.spin_peak_progress, progress)

    def patrol_points_callback(self, msg):
        if self.patrol_points:
            return

        self.patrol_points = list(msg.poses)
        for index, pose in enumerate(self.patrol_points):
            self.points_writer.writerow([
                index,
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(pose.orientation),
            ])
        self.points_file.flush()
        self.get_logger().info(
            f'Received {len(self.patrol_points)} patrol points.'
        )

    def goal_callback(self, msg):
        self.latest_goal = msg
        self.current_goal_index = self.find_goal_index(msg)

    def find_goal_index(self, msg):
        if not self.patrol_points:
            return -1
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        distances = [
            math.hypot(gx - p.position.x, gy - p.position.y)
            for p in self.patrol_points
        ]
        return min(range(len(distances)), key=distances.__getitem__)

    def plan_callback(self, msg):
        points = [
            (float(p.pose.position.x), float(p.pose.position.y))
            for p in msg.poses
        ]
        if len(points) < 2:
            return

        self.latest_plan = points
        self.plan_revision += 1
        for index, (x, y) in enumerate(points):
            self.plans_writer.writerow([
                self.plan_revision,
                index,
                x,
                y,
            ])
        self.plans_file.flush()

    def state_callback(self, msg):
        new_state = msg.data
        old_state = self.mission_state
        self.previous_state = old_state
        self.mission_state = new_state
        now = self.now_seconds()

        if new_state == 'GO_TO_PATROL' and old_state != 'GO_TO_PATROL':
            self.nav_start_time = now
            self.nav_start_gt_distance = self.gt_distance_total

        if new_state == 'PATROL_SCAN' and old_state != 'PATROL_SCAN':
            self.record_checkpoint_arrival(now)
            self.start_spin_tracking(now)

        if old_state == 'PATROL_SCAN' and new_state != 'PATROL_SCAN':
            self.finish_spin_tracking(now)

    def lookup_estimated_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
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
        return math.hypot(
            current_xy[0] - previous_xy[0],
            current_xy[1] - previous_xy[1],
        )

    def update_total_distances(self, estimated):
        if self.latest_gt is not None:
            gt_xy = self.latest_gt[:2]
            step = self.update_distance(gt_xy, self.last_gt_xy)
            self.last_gt_xy = gt_xy
            if step < 1.0:
                self.gt_distance_total += step

        if estimated is not None:
            est_xy = estimated[:2]
            step = self.update_distance(est_xy, self.last_est_xy)
            self.last_est_xy = est_xy
            if step < 1.0:
                self.est_distance_total += step

    def record_checkpoint_arrival(self, now):
        if self.latest_goal is None:
            return

        goal = self.latest_goal.pose
        goal_yaw = yaw_from_quaternion(goal.orientation)
        gt = self.latest_gt
        est = self.lookup_estimated_pose()

        nav_time = (
            now - self.nav_start_time
            if self.nav_start_time is not None
            else float('nan')
        )
        nav_length = self.gt_distance_total - self.nav_start_gt_distance

        gt_values = [float('nan')] * 5
        if gt is not None:
            gt_values = [
                gt[0], gt[1], gt[2],
                math.hypot(gt[0] - goal.position.x, gt[1] - goal.position.y),
                math.degrees(abs(wrap_angle(gt[2] - goal_yaw))),
            ]

        est_values = [float('nan')] * 5
        if est is not None:
            est_values = [
                est[0], est[1], est[2],
                math.hypot(est[0] - goal.position.x, est[1] - goal.position.y),
                math.degrees(abs(wrap_angle(est[2] - goal_yaw))),
            ]

        self.checkpoints_writer.writerow([
            self.current_goal_index,
            now,
            nav_time,
            nav_length,
            goal.position.x,
            goal.position.y,
            goal_yaw,
            *gt_values,
            *est_values,
        ])
        self.checkpoints_file.flush()

    def start_spin_tracking(self, now):
        if self.latest_gt is None:
            return

        self.spin_active = True
        self.spin_start_time = now
        self.spin_start_gt = self.latest_gt
        self.spin_last_yaw = self.latest_gt[2]
        self.spin_signed_rotation = 0.0
        self.spin_abs_rotation = 0.0
        self.spin_peak_progress = 0.0
        self.spin_checkpoint_index = self.current_goal_index

    def finish_spin_tracking(self, now):
        if not self.spin_active or self.spin_start_gt is None:
            self.spin_active = False
            return

        end_gt = self.latest_gt if self.latest_gt is not None else self.spin_start_gt
        duration = now - self.spin_start_time

        direction = 1.0 if self.spin_target_angle >= 0.0 else -1.0
        target = abs(self.spin_target_angle)
        net_progress = direction * self.spin_signed_rotation
        peak = self.spin_peak_progress
        overshoot = max(0.0, peak - target)
        correction = max(0.0, peak - net_progress)
        position_drift = math.hypot(
            end_gt[0] - self.spin_start_gt[0],
            end_gt[1] - self.spin_start_gt[1],
        )

        self.spins_writer.writerow([
            self.spin_checkpoint_index,
            self.spin_start_time,
            now,
            duration,
            math.degrees(target),
            math.degrees(net_progress),
            math.degrees(self.spin_abs_rotation),
            math.degrees(peak),
            math.degrees(overshoot),
            math.degrees(correction),
            self.spin_start_gt[0],
            self.spin_start_gt[1],
            end_gt[0],
            end_gt[1],
            position_drift,
        ])
        self.spins_file.flush()

        self.get_logger().info(
            f'Spin P{self.spin_checkpoint_index}: '
            f'net={math.degrees(net_progress):.2f} deg, '
            f'peak={math.degrees(peak):.2f} deg, '
            f'overshoot={math.degrees(overshoot):.2f} deg, '
            f'correction={math.degrees(correction):.2f} deg.'
        )

        self.spin_active = False
        self.spin_start_time = None
        self.spin_start_gt = None
        self.spin_last_yaw = None

    def sample(self):
        estimated = self.lookup_estimated_pose()
        self.update_total_distances(estimated)

        gt = self.latest_gt
        if gt is None and estimated is None:
            return

        nan = float('nan')
        gt_x, gt_y, gt_yaw = gt if gt is not None else (nan, nan, nan)
        est_x, est_y, est_yaw = (
            estimated if estimated is not None else (nan, nan, nan)
        )

        loc_pos = nan
        loc_yaw = nan
        if gt is not None and estimated is not None:
            loc_pos = math.hypot(est_x - gt_x, est_y - gt_y)
            loc_yaw = abs(wrap_angle(est_yaw - gt_yaw))

        gt_cte = (
            nearest_path_distance(gt_x, gt_y, self.latest_plan)
            if gt is not None else nan
        )
        est_cte = (
            nearest_path_distance(est_x, est_y, self.latest_plan)
            if estimated is not None else nan
        )

        self.samples_writer.writerow([
            self.now_seconds(),
            self.mission_state,
            self.current_goal_index,
            self.plan_revision,
            gt_x, gt_y, gt_yaw,
            est_x, est_y, est_yaw,
            loc_pos, loc_yaw,
            gt_cte, est_cte,
            self.gt_distance_total,
            self.est_distance_total,
        ])
        self.sample_count += 1
        if self.sample_count % self.flush_every_samples == 0:
            self.samples_file.flush()

    def destroy_node(self):
        for handle in [
            self.samples_file,
            self.points_file,
            self.plans_file,
            self.checkpoints_file,
            self.spins_file,
        ]:
            try:
                handle.flush()
                handle.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolTrajectoryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
