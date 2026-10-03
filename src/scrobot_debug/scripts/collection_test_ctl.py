#!/usr/bin/env python3

import argparse
import math
import os
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


WORLD = 'badminton_court'
MODEL_NAME = 'shuttle_collection_test'
CENTER_OFFSET_Z = 0.045


def run(cmd, label):
    result = subprocess.run(
        cmd,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f'{label} failed (return code {result.returncode}):\n'
            f'{result.stdout}'
        )
    if result.stdout.strip():
        print(result.stdout.strip(), flush=True)


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy_from_quaternion(q):
    x, y, z, w = q

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def rotate_vector(q, vector):
    x, y, z, w = q
    vx, vy, vz = vector

    # Rotation matrix from quaternion.
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


class RobotPoseReader(Node):
    def __init__(self):
        super().__init__('collection_test_pose_reader')
        self.pose = None
        self.create_subscription(
            Odometry,
            '/evaluation/ground_truth_odom',
            self._cb,
            qos_profile_sensor_data,
        )

    def _cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.pose = (float(p.x), float(p.y), yaw)


def current_robot_pose(timeout=3.0):
    rclpy.init(args=None)
    node = RobotPoseReader()
    deadline = time.monotonic() + timeout

    try:
        while rclpy.ok() and node.pose is None:
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    'Timed out waiting for /evaluation/ground_truth_odom. '
                    'Start collection_check.launch.py first.'
                )
            rclpy.spin_once(node, timeout_sec=0.1)

        return node.pose
    finally:
        node.destroy_node()
        rclpy.shutdown()


def delete_model(world, name, ignore_missing=False):
    cmd = [
        'gz', 'service',
        '-s', f'/world/{world}/remove/blocking',
        '--reqtype', 'gz.msgs.Entity',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '3000',
        '--req', f'name: "{name}", type: MODEL',
    ]

    try:
        run(cmd, f'delete {name}')
    except RuntimeError:
        if ignore_missing:
            print(
                f'[collection_test_ctl] {name}: already absent',
                flush=True,
            )
            return
        raise

    print(f'[collection_test_ctl] deleted {name}', flush=True)


def spawn_model(args):
    if args.delay > 0.0:
        print(
            f'[collection_test_ctl] spawning in {args.delay:.1f} s...',
            flush=True,
        )
        time.sleep(args.delay)

    robot_x, robot_y, robot_yaw = current_robot_pose()

    rr = math.radians(args.roll_deg)
    rp = math.radians(args.pitch_deg)
    ry = math.radians(args.yaw_deg)

    q_robot = quaternion_from_rpy(0.0, 0.0, robot_yaw)
    q_relative = quaternion_from_rpy(rr, rp, ry)
    q_world = quaternion_multiply(q_robot, q_relative)

    # Desired collection-center location in the robot frame.
    c = math.cos(robot_yaw)
    s = math.sin(robot_yaw)
    center_world_x = (
        robot_x + c * args.center_x - s * args.center_y
    )
    center_world_y = (
        robot_y + s * args.center_x + c * args.center_y
    )

    # The plugin evaluates origin + local +Z * 45 mm. Therefore place the
    # model origin so the requested collection center is exact even when the
    # shuttle orientation changes.
    center_offset_world = rotate_vector(
        q_world, (0.0, 0.0, CENTER_OFFSET_Z)
    )
    origin_x = center_world_x - center_offset_world[0]
    origin_y = center_world_y - center_offset_world[1]
    origin_z = args.origin_z

    roll, pitch, yaw = rpy_from_quaternion(q_world)

    simulation_share = get_package_share_directory(
        'scrobot_simulation'
    )
    model_file = os.path.join(
        simulation_share,
        'models',
        'shuttle',
        'model_physics_test.sdf',
    )

    if not os.path.isfile(model_file):
        raise FileNotFoundError(model_file)

    run(
        [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', args.world,
            '-name', args.name,
            '-file', model_file,
            '-x', f'{origin_x:.6f}',
            '-y', f'{origin_y:.6f}',
            '-z', f'{origin_z:.6f}',
            '-R', f'{roll:.6f}',
            '-P', f'{pitch:.6f}',
            '-Y', f'{yaw:.6f}',
        ],
        'spawn collection-test shuttle',
    )

    print(
        '[collection_test_ctl] spawned '
        f'{args.name}: requested_center_robot='
        f'({args.center_x:+.4f},{args.center_y:+.4f}) m, '
        f'rel_RPY=({args.roll_deg:+.1f},'
        f'{args.pitch_deg:+.1f},{args.yaw_deg:+.1f}) deg, '
        f'origin_world=({origin_x:+.4f},'
        f'{origin_y:+.4f},{origin_z:+.4f}) m',
        flush=True,
    )


def drive_straight(args):
    if args.delay > 0.0:
        print(
            f'[collection_test_ctl] straight drive starts in '
            f'{args.delay:.1f} s...',
            flush=True,
        )
        time.sleep(args.delay)

    rclpy.init(args=None)
    node = Node('collection_test_straight_drive')
    publisher = node.create_publisher(
        TwistStamped,
        '/diff_drive_controller/cmd_vel',
        10,
    )

    period = 1.0 / max(1.0, args.rate)
    deadline = time.monotonic() + max(0.0, args.duration)

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = float(args.speed)
            msg.twist.angular.z = 0.0
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)

        # Publish zero several times so the test always leaves the base stopped.
        for _ in range(5):
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(
        f'[collection_test_ctl] straight drive complete: '
        f'v={args.speed:.3f} m/s duration={args.duration:.3f} s',
        flush=True,
    )


def add_spawn_args(parser):
    parser.add_argument('--world', default=WORLD)
    parser.add_argument('--name', default=MODEL_NAME)
    parser.add_argument(
        '--center-x',
        type=float,
        default=0.165,
        help=(
            'Desired shuttle collection-center X in the CURRENT '
            'robot frame [m].'
        ),
    )
    parser.add_argument(
        '--center-y',
        type=float,
        default=0.0,
        help=(
            'Desired shuttle collection-center Y in the CURRENT '
            'robot frame [m].'
        ),
    )
    parser.add_argument('--roll-deg', type=float, default=0.0)
    parser.add_argument('--pitch-deg', type=float, default=90.0)
    parser.add_argument('--yaw-deg', type=float, default=0.0)
    parser.add_argument(
        '--origin-z',
        type=float,
        default=0.0367,
        help='World Z used for the shuttle model origin [m].',
    )
    parser.add_argument(
        '--delay',
        type=float,
        default=5.0,
        help='Wall-time countdown before spawning [s].',
    )


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Persistent collection-test shuttle controller. '
            'Gazebo and the robot stay running between tests.'
        )
    )
    sub = parser.add_subparsers(dest='command', required=True)

    spawn = sub.add_parser('spawn')
    add_spawn_args(spawn)

    delete = sub.add_parser('delete')
    delete.add_argument('--world', default=WORLD)
    delete.add_argument('--name', default=MODEL_NAME)
    delete.add_argument('--ignore-missing', action='store_true')

    respawn = sub.add_parser('respawn')
    add_spawn_args(respawn)
    respawn.add_argument('--delete-wait', type=float, default=0.5)

    drive = sub.add_parser(
        'drive',
        help='Drive the robot straight through the test shuttle.',
    )
    drive.add_argument('--speed', type=float, default=0.10)
    drive.add_argument('--duration', type=float, default=3.0)
    drive.add_argument('--rate', type=float, default=20.0)
    drive.add_argument('--delay', type=float, default=2.0)

    args = parser.parse_args()

    try:
        if args.command == 'spawn':
            spawn_model(args)
        elif args.command == 'delete':
            delete_model(
                args.world,
                args.name,
                ignore_missing=args.ignore_missing,
            )
        elif args.command == 'respawn':
            delete_model(
                args.world,
                args.name,
                ignore_missing=True,
            )
            time.sleep(max(0.0, args.delete_wait))
            spawn_model(args)
        elif args.command == 'drive':
            drive_straight(args)
    except Exception as exc:
        print(
            f'[collection_test_ctl] ERROR: {exc}',
            file=sys.stderr,
        )
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
