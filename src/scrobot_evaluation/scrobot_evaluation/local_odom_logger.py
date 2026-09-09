#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import euler_from_quaternion


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return wrap_angle(euler_from_quaternion([q.x, q.y, q.z, q.w])[2])


def stamp_to_seconds(stamp):
    if stamp is None:
        return float('nan')
    sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return sec if sec > 0.0 else float('nan')


class TimingStats:
    def __init__(self):
        self.count = 0
        self.last_recv = None
        self.interval_sum = 0.0
        self.interval_sq_sum = 0.0
        self.interval_min = float('inf')
        self.interval_max = 0.0
        self.age_count = 0
        self.age_sum = 0.0
        self.age_sq_sum = 0.0
        self.age_min = float('inf')
        self.age_max = 0.0

    def update(self, recv_time, msg_stamp):
        self.count += 1

        if self.last_recv is not None:
            dt = recv_time - self.last_recv
            if dt > 0.0:
                self.interval_sum += dt
                self.interval_sq_sum += dt * dt
                self.interval_min = min(self.interval_min, dt)
                self.interval_max = max(self.interval_max, dt)
        self.last_recv = recv_time

        if math.isfinite(msg_stamp):
            age = recv_time - msg_stamp
            self.age_count += 1
            self.age_sum += age
            self.age_sq_sum += age * age
            self.age_min = min(self.age_min, age)
            self.age_max = max(self.age_max, age)

    @staticmethod
    def _std(count, total, square_total):
        if count <= 0:
            return float('nan')
        mean = total / count
        variance = max(0.0, square_total / count - mean * mean)
        return math.sqrt(variance)

    def summary(self):
        interval_count = max(0, self.count - 1)
        mean_interval = (
            self.interval_sum / interval_count
            if interval_count > 0 else float('nan')
        )
        mean_hz = (
            1.0 / mean_interval
            if math.isfinite(mean_interval) and mean_interval > 0.0
            else float('nan')
        )
        mean_age = (
            self.age_sum / self.age_count
            if self.age_count > 0 else float('nan')
        )
        return {
            'count': self.count,
            'mean_hz': mean_hz,
            'mean_interval_s': mean_interval,
            'std_interval_s': self._std(
                interval_count,
                self.interval_sum,
                self.interval_sq_sum,
            ),
            'min_interval_s': (
                self.interval_min
                if interval_count > 0 else float('nan')
            ),
            'max_interval_s': (
                self.interval_max
                if interval_count > 0 else float('nan')
            ),
            'mean_age_s': mean_age,
            'std_age_s': self._std(
                self.age_count,
                self.age_sum,
                self.age_sq_sum,
            ),
            'min_age_s': (
                self.age_min
                if self.age_count > 0 else float('nan')
            ),
            'max_age_s': (
                self.age_max
                if self.age_count > 0 else float('nan')
            ),
        }


class LocalOdomLogger(Node):
    def __init__(self):
        super().__init__('local_odom_logger')

        self.declare_parameter('ground_truth_topic', '/evaluation/ground_truth_odom')
        self.declare_parameter('wheel_odom_topic', '/diff_drive_controller/odom')
        self.declare_parameter('ekf_odom_topic', '/odometry/filtered')
        self.declare_parameter('imu_raw_topic', '/imu/data_raw')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('requested_cmd_topic', '/cmd_vel_manual')
        self.declare_parameter('final_cmd_topic', '/diff_drive_controller/cmd_vel')
        self.declare_parameter('test_state_topic', '/evaluation/local_odom_test_state')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('left_wheel_joint', 'left_wheel_joint')
        self.declare_parameter('right_wheel_joint', 'right_wheel_joint')

        self.declare_parameter('sample_rate', 50.0)
        self.declare_parameter('tf_timeout', 0.02)
        self.declare_parameter('output_root', '~/scrobot_evaluation_runs/local_odom')
        self.declare_parameter('run_name', '')
        self.declare_parameter('flush_every_samples', 50)

        self.ground_truth_topic = str(
            self.get_parameter('ground_truth_topic').value
        )
        self.wheel_odom_topic = str(self.get_parameter('wheel_odom_topic').value)
        self.ekf_odom_topic = str(self.get_parameter('ekf_odom_topic').value)
        self.imu_raw_topic = str(self.get_parameter('imu_raw_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.joint_states_topic = str(
            self.get_parameter('joint_states_topic').value
        )
        self.requested_cmd_topic = str(
            self.get_parameter('requested_cmd_topic').value
        )
        self.final_cmd_topic = str(self.get_parameter('final_cmd_topic').value)
        self.test_state_topic = str(self.get_parameter('test_state_topic').value)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.left_wheel_joint = str(self.get_parameter('left_wheel_joint').value)
        self.right_wheel_joint = str(
            self.get_parameter('right_wheel_joint').value
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

        # Evaluation subscriptions are intentionally BEST_EFFORT + depth 1.
        # They must never back-pressure the localization/control graph.
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

        self.latest_gt = None
        self.latest_wheel = None
        self.latest_ekf = None
        self.latest_imu_raw = None
        self.latest_imu = None
        self.latest_joint = None
        self.latest_requested_cmd = None
        self.latest_final_cmd = None
        self.test_state = 'UNKNOWN'

        self.timing = {
            'ground_truth': TimingStats(),
            'wheel_odom': TimingStats(),
            'ekf_odom': TimingStats(),
            'imu_raw': TimingStats(),
            'imu_filtered': TimingStats(),
            'joint_states': TimingStats(),
            'requested_cmd': TimingStats(),
            'final_cmd': TimingStats(),
        }

        self.samples_file = open(
            self.output_dir / 'local_odom_samples.csv',
            'w',
            newline='',
        )
        self.samples_writer = csv.writer(self.samples_file)
        self.samples_writer.writerow([
            'ros_time', 'test_state',
            'gt_stamp', 'gt_x', 'gt_y', 'gt_yaw',
            'wheel_stamp', 'wheel_x', 'wheel_y', 'wheel_yaw',
            'wheel_vx', 'wheel_vy', 'wheel_wz',
            'ekf_stamp', 'ekf_x', 'ekf_y', 'ekf_yaw',
            'ekf_vx', 'ekf_vy', 'ekf_wz',
            'tf_x', 'tf_y', 'tf_yaw',
            'imu_raw_stamp', 'imu_raw_wx', 'imu_raw_wy', 'imu_raw_wz',
            'imu_raw_ax', 'imu_raw_ay', 'imu_raw_az',
            'imu_stamp', 'imu_yaw', 'imu_wx', 'imu_wy', 'imu_wz',
            'imu_ax', 'imu_ay', 'imu_az',
            'left_wheel_pos', 'left_wheel_vel',
            'right_wheel_pos', 'right_wheel_vel',
            'requested_vx', 'requested_wz',
            'final_vx', 'final_wz',
        ])
        self.sample_count = 0

        self.create_subscription(
            Odometry,
            self.ground_truth_topic,
            self.ground_truth_callback,
            sensor_qos,
        )
        self.create_subscription(
            Odometry,
            self.wheel_odom_topic,
            self.wheel_odom_callback,
            sensor_qos,
        )
        self.create_subscription(
            Odometry,
            self.ekf_odom_topic,
            self.ekf_odom_callback,
            sensor_qos,
        )
        self.create_subscription(
            Imu,
            self.imu_raw_topic,
            self.imu_raw_callback,
            sensor_qos,
        )
        self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            sensor_qos,
        )
        self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_state_callback,
            sensor_qos,
        )
        self.create_subscription(
            TwistStamped,
            self.requested_cmd_topic,
            self.requested_cmd_callback,
            sensor_qos,
        )
        self.create_subscription(
            TwistStamped,
            self.final_cmd_topic,
            self.final_cmd_callback,
            sensor_qos,
        )
        self.create_subscription(
            String,
            self.test_state_topic,
            self.test_state_callback,
            1,
        )

        self.sample_timer = self.create_timer(1.0 / sample_rate, self.sample)

        self.get_logger().info(f'Local odom logging to: {self.output_dir}')
        self.get_logger().info(
            'Recording wheel odom, EKF, IMU, wheel joints, odom TF, commands, '
            'and Gazebo ground truth.'
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def update_timing(self, key, msg_stamp):
        self.timing[key].update(self.now_seconds(), msg_stamp)

    def ground_truth_callback(self, msg):
        # Dedicated Gazebo OdometryPublisher output.
        # The plugin computes pose directly from the model world pose.
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        stamp = stamp_to_seconds(msg.header.stamp)
        self.latest_gt = (
            stamp,
            float(p.x),
            float(p.y),
            yaw_from_quaternion(q),
        )
        self.update_timing('ground_truth', stamp)

    @staticmethod
    def odom_tuple(msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist
        return (
            stamp_to_seconds(msg.header.stamp),
            float(p.x),
            float(p.y),
            yaw_from_quaternion(q),
            float(v.linear.x),
            float(v.linear.y),
            float(v.angular.z),
        )

    def wheel_odom_callback(self, msg):
        self.latest_wheel = self.odom_tuple(msg)
        self.update_timing('wheel_odom', self.latest_wheel[0])

    def ekf_odom_callback(self, msg):
        self.latest_ekf = self.odom_tuple(msg)
        self.update_timing('ekf_odom', self.latest_ekf[0])

    @staticmethod
    def imu_tuple(msg, include_yaw):
        yaw = yaw_from_quaternion(msg.orientation) if include_yaw else float('nan')
        return (
            stamp_to_seconds(msg.header.stamp),
            yaw,
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
            float(msg.linear_acceleration.z),
        )

    def imu_raw_callback(self, msg):
        self.latest_imu_raw = self.imu_tuple(msg, include_yaw=False)
        self.update_timing('imu_raw', self.latest_imu_raw[0])

    def imu_callback(self, msg):
        self.latest_imu = self.imu_tuple(msg, include_yaw=True)
        self.update_timing('imu_filtered', self.latest_imu[0])

    def joint_state_callback(self, msg):
        left_pos = left_vel = right_pos = right_vel = float('nan')
        index = {name: i for i, name in enumerate(msg.name)}

        if self.left_wheel_joint in index:
            i = index[self.left_wheel_joint]
            if i < len(msg.position):
                left_pos = float(msg.position[i])
            if i < len(msg.velocity):
                left_vel = float(msg.velocity[i])

        if self.right_wheel_joint in index:
            i = index[self.right_wheel_joint]
            if i < len(msg.position):
                right_pos = float(msg.position[i])
            if i < len(msg.velocity):
                right_vel = float(msg.velocity[i])

        stamp = stamp_to_seconds(msg.header.stamp)
        self.latest_joint = (stamp, left_pos, left_vel, right_pos, right_vel)
        self.update_timing('joint_states', stamp)

    @staticmethod
    def command_tuple(msg):
        return (
            stamp_to_seconds(msg.header.stamp),
            float(msg.twist.linear.x),
            float(msg.twist.angular.z),
        )

    def requested_cmd_callback(self, msg):
        self.latest_requested_cmd = self.command_tuple(msg)
        self.update_timing('requested_cmd', self.latest_requested_cmd[0])

    def final_cmd_callback(self, msg):
        self.latest_final_cmd = self.command_tuple(msg)
        self.update_timing('final_cmd', self.latest_final_cmd[0])

    def test_state_callback(self, msg):
        self.test_state = msg.data

    @staticmethod
    def values_or_nan(value, count):
        if value is None:
            return [float('nan')] * count
        return list(value)

    def lookup_odom_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException:
            return [float('nan')] * 3

        t = transform.transform.translation
        q = transform.transform.rotation
        return [float(t.x), float(t.y), yaw_from_quaternion(q)]

    def sample(self):
        gt = self.values_or_nan(self.latest_gt, 4)
        wheel = self.values_or_nan(self.latest_wheel, 7)
        ekf = self.values_or_nan(self.latest_ekf, 7)
        imu_raw = self.values_or_nan(self.latest_imu_raw, 8)
        imu = self.values_or_nan(self.latest_imu, 8)
        joint = self.values_or_nan(self.latest_joint, 5)
        req = self.values_or_nan(self.latest_requested_cmd, 3)
        final = self.values_or_nan(self.latest_final_cmd, 3)
        tf_pose = self.lookup_odom_tf()

        self.samples_writer.writerow([
            self.now_seconds(), self.test_state,
            gt[0], gt[1], gt[2], gt[3],
            wheel[0], wheel[1], wheel[2], wheel[3],
            wheel[4], wheel[5], wheel[6],
            ekf[0], ekf[1], ekf[2], ekf[3],
            ekf[4], ekf[5], ekf[6],
            tf_pose[0], tf_pose[1], tf_pose[2],
            imu_raw[0], imu_raw[2], imu_raw[3], imu_raw[4],
            imu_raw[5], imu_raw[6], imu_raw[7],
            imu[0], imu[1], imu[2], imu[3], imu[4],
            imu[5], imu[6], imu[7],
            joint[1], joint[2], joint[3], joint[4],
            req[1], req[2],
            final[1], final[2],
        ])

        self.sample_count += 1
        if self.sample_count % self.flush_every_samples == 0:
            self.samples_file.flush()

    def write_timing_summary(self):
        path = self.output_dir / 'timing_summary.csv'
        with open(path, 'w', newline='') as handle:
            fields = [
                'topic_key', 'count', 'mean_hz',
                'mean_interval_s', 'std_interval_s',
                'min_interval_s', 'max_interval_s',
                'mean_age_s', 'std_age_s', 'min_age_s', 'max_age_s',
            ]
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for key, stats in self.timing.items():
                row = {'topic_key': key}
                row.update(stats.summary())
                writer.writerow(row)

    def destroy_node(self):
        try:
            self.samples_file.flush()
            self.samples_file.close()
        except Exception:
            pass

        try:
            self.write_timing_summary()
        except Exception as exc:
            self.get_logger().error(f'Failed to write timing summary: {exc}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LocalOdomLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
