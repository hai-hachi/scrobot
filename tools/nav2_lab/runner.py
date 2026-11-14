#!/usr/bin/env python3
"""One real Nav2 ComputePathToPose / FollowPath experiment.

Invoked as a subprocess by web_server.py. Logs planned path, actual odometry,
and final robot velocity commands for repeatable planner/controller comparisons.
"""
import argparse
import csv
import json
import math
import signal
import time
from pathlib import Path

import yaml
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from core import path_length, tracking_metrics, validate_config

STOP = False


def stop_signal(_signal, _frame):
    global STOP
    STOP = True


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def path_xy(path):
    return [(p.pose.position.x, p.pose.position.y) for p in path.poses]


def sample_position(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y),
                     1-2*(q.y*q.y+q.z*q.z))
    return (seconds(msg.header.stamp), p.x, p.y, yaw)


def finite_snapshot(points):
    return [list(x) for x in points if all(math.isfinite(v) for v in x)]


class Nav2Trial(Node):
    def __init__(self, config):
        super().__init__(
            'nav2_lab_trial',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
            automatically_declare_parameters_from_overrides=True)
        self.config = config
        self.odom = []
        self.truth = []
        self.commands = []
        self.tracking = False
        self.first_odom = False
        self.last_sample_time = -float('inf')
        self.latest_odom = None
        self.latest_truth = None
        self.rate_period = 1.0 / config['experiment']['sample_hz']
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/evaluation/ground_truth_odom',
                                 self.on_truth, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/diff_drive_controller/cmd_vel',
                                 self.on_command, qos_profile_sensor_data)
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.controller = ActionClient(self, FollowPath, '/follow_path')

    def on_odom(self, msg):
        p = sample_position(msg)
        self.latest_odom = p
        self.first_odom = True
        if self.tracking and p[0] >= self.last_sample_time + self.rate_period:
            self.odom.append(p)
            self.last_sample_time = p[0]

    def on_truth(self, msg):
        p = sample_position(msg)
        self.latest_truth = p
        if self.tracking and (not self.truth or
                              p[0] - self.truth[-1][0] >= self.rate_period):
            self.truth.append(p)

    def on_command(self, msg):
        if self.tracking:
            self.commands.append((seconds(msg.header.stamp),
                                  msg.twist.linear.x, msg.twist.angular.z))

    def pump_until(self, predicate, timeout_s, message):
        deadline = time.monotonic() + timeout_s
        while not STOP and time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self, timeout_sec=0.08)
        raise RuntimeError('Cancelled' if STOP else message)

    def action_available(self, client, label):
        deadline = time.monotonic() + 45.0
        while not STOP and time.monotonic() < deadline:
            if client.wait_for_server(timeout_sec=0.5):
                return
            rclpy.spin_once(self, timeout_sec=0)
        raise RuntimeError(f'{label} action server unavailable (check Nav2 lifecycle/plugins)')

    def goal_handle(self, client, goal, label):
        sent = client.send_goal_async(goal)
        self.pump_until(sent.done, 20, f'{label}: request timed out')
        handle = sent.result()
        if not handle or not handle.accepted:
            raise RuntimeError(f'{label}: goal rejected')
        return handle

    def await_result(self, handle, timeout_s, label):
        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout_s
        while not STOP and time.monotonic() < deadline:
            if result_future.done():
                response = result_future.result()
                if response.status != GoalStatus.STATUS_SUCCEEDED:
                    raise RuntimeError(f'{label} finished with action status {response.status}')
                result = response.result
                if getattr(result, 'error_code', 0) != 0:
                    raise RuntimeError(
                        f'{label} error {result.error_code}: {getattr(result, "error_msg", "")}')
                return result
            rclpy.spin_once(self, timeout_sec=0.08)
        # Never leave FollowPath running after a lab timeout/interrupt.
        cancelled = handle.cancel_goal_async()
        for _ in range(30):
            if cancelled.done():
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError('Cancelled' if STOP else f'{label} timed out')

    def compute_path(self):
        self.pump_until(lambda: self.first_odom, 45,
                        'No EKF /odometry/filtered received')
        self.action_available(self.planner, 'ComputePathToPose')
        goal = ComputePathToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.config['goal']['x'])
        pose.pose.position.y = float(self.config['goal']['y'])
        theta = math.radians(self.config['goal']['yaw_deg'])
        pose.pose.orientation.z = math.sin(theta/2)
        pose.pose.orientation.w = math.cos(theta/2)
        goal.goal = pose
        goal.planner_id = self.config['planner']
        goal.use_start = False
        start = time.monotonic()
        handle = self.goal_handle(self.planner, goal, 'planner')
        result = self.await_result(handle, min(50.0, self.config['experiment']['timeout_s']),
                                   'planner')
        duration = time.monotonic()-start
        if not result.path.poses:
            raise RuntimeError('Planner returned an empty path')
        # Planning wall-time includes action transport; Nav2 planning_time is
        # independently recorded when available.
        nav2_time = getattr(result, 'planning_time', None)
        self.planning_wall_s = duration
        self.planning_nav2_s = seconds(nav2_time) if nav2_time else None
        return result.path

    def follow_path(self, path):
        self.action_available(self.controller, 'FollowPath')
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.config['controller']
        goal.goal_checker_id = 'goal_checker'
        goal.progress_checker_id = 'progress_checker'
        handle = self.goal_handle(self.controller, goal, 'controller')
        self.tracking = True
        self.started = time.monotonic()
        if self.latest_odom:
            self.odom.append(self.latest_odom)
            self.last_sample_time = self.latest_odom[0]
        try:
            self.await_result(handle, self.config['experiment']['timeout_s'],
                              'controller')
            return time.monotonic() - self.started
        finally:
            self.tracking = False


def save_csv(filepath, headers, rows):
    with filepath.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(headers)
        w.writerows(rows)


def run(config, run_dir):
    validate_config(config)
    run_dir.mkdir(parents=True, exist_ok=True)
    summary = {
        'planner': config['planner'], 'controller': config['controller'],
        'mode': config['mode'], 'status': 'error',
        'goal': config['goal'], 'started_wall_time': time.time()
    }
    trial = None
    rclpy.init()
    try:
        trial = Nav2Trial(config)
        planned = trial.compute_path()
        points = path_xy(planned)
        summary.update({
            'planned_points': len(points),
            'path_length_m': path_length(points),
            'planning_wall_s': trial.planning_wall_s,
            'planning_nav2_s': trial.planning_nav2_s
        })
        (run_dir / 'path.json').write_text(json.dumps(
            {'frame_id': planned.header.frame_id, 'points': points}, indent=2))
        if config['mode'] == 'track':
            summary['tracking_duration_s'] = trial.follow_path(planned)
            tracked = [(x, y) for t, x, y, yaw in trial.odom]
            summary.update(tracking_metrics(points, tracked))
            summary['tracked_samples'] = len(tracked)
            if trial.commands:
                max_wheel_mps = 2*math.pi*.05*config['limits']['max_wheel_rpm']/60
                saturated = [
                    max(abs(v-.21*w), abs(v+.21*w)) >= .995*max_wheel_mps
                    for _,v,w in trial.commands]
                summary['wheel_limit_fraction'] = sum(saturated)/len(saturated)
                summary['command_total_variation'] = sum(
                    abs(b[1]-a[1])+0.21*abs(b[2]-a[2])
                    for a,b in zip(trial.commands, trial.commands[1:]))
        summary['status'] = 'success'
    except Exception as exc:
        summary['error'] = str(exc)
    finally:
        if trial:
            save_csv(run_dir / 'odometry.csv', ('stamp_s','x_m','y_m','yaw_rad'),
                     finite_snapshot(trial.odom))
            save_csv(run_dir / 'ground_truth.csv', ('stamp_s','x_world_m','y_world_m','yaw_rad'),
                     finite_snapshot(trial.truth))
            save_csv(run_dir / 'commands.csv', ('stamp_s','linear_mps','angular_rps'),
                     finite_snapshot(trial.commands))
            trial.destroy_node()
        rclpy.shutdown()
        summary['ended_wall_time'] = time.time()
        (run_dir / 'summary.json').write_text(json.dumps(summary, indent=2))
    return 0 if summary['status'] == 'success' else 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    config = yaml.safe_load(args.config.read_text())
    raise SystemExit(run(config, args.output))


if __name__ == '__main__':
    main()
