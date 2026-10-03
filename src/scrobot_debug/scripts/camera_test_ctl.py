#!/usr/bin/env python3

import argparse
import math
import os
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


WORLD = 'camera_test'
MODEL_NAME = 'shuttle_camera_target'
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


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rpy_from_quaternion(q):
    x, y, z, w = q
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    sinp = 2.0 * (w * y - z * x)
    pitch = (
        math.copysign(math.pi / 2.0, sinp)
        if abs(sinp) >= 1.0
        else math.asin(sinp)
    )
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, pitch, yaw


def rotate_vector(q, v):
    x, y, z, w = q
    vx, vy, vz = v
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
        super().__init__('camera_test_pose_reader')
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
        self.pose = (
            float(p.x),
            float(p.y),
            float(p.z),
            yaw,
        )


def current_robot_pose(timeout=3.0):
    rclpy.init(args=None)
    node = RobotPoseReader()
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and node.pose is None:
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    'Timed out waiting for /evaluation/ground_truth_odom. '
                    'Start camera_check.launch.py first.'
                )
            rclpy.spin_once(node, timeout_sec=0.1)
        return node.pose
    finally:
        node.destroy_node()
        rclpy.shutdown()


def delete_target(world, name, ignore_missing=False):
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
                f'[camera_test_ctl] {name}: already absent',
                flush=True,
            )
            return
        raise
    print(f'[camera_test_ctl] deleted {name}', flush=True)


def spawn_target(args):
    if args.delay > 0.0:
        print(
            f'[camera_test_ctl] spawning in {args.delay:.1f} s...',
            flush=True,
        )
        time.sleep(args.delay)

    robot_x, robot_y, robot_z, robot_yaw = current_robot_pose()

    relative_q = quaternion_from_rpy(
        math.radians(args.roll_deg),
        math.radians(args.pitch_deg),
        math.radians(args.yaw_deg),
    )
    robot_q = quaternion_from_rpy(0.0, 0.0, robot_yaw)
    world_q = quaternion_multiply(robot_q, relative_q)

    c = math.cos(robot_yaw)
    s = math.sin(robot_yaw)

    center_world = (
        robot_x + c * args.center_x - s * args.center_y,
        robot_y + s * args.center_x + c * args.center_y,
        robot_z + args.center_z,
    )

    offset_world = rotate_vector(
        world_q, (0.0, 0.0, CENTER_OFFSET_Z)
    )
    origin = (
        center_world[0] - offset_world[0],
        center_world[1] - offset_world[1],
        center_world[2] - offset_world[2],
    )

    roll, pitch, yaw = rpy_from_quaternion(world_q)

    share = get_package_share_directory('scrobot_simulation')
    model_file = os.path.join(
        share, 'models', 'shuttle', 'model_camera_test.sdf'
    )

    run(
        [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', args.world,
            '-name', args.name,
            '-file', model_file,
            '-x', f'{origin[0]:.6f}',
            '-y', f'{origin[1]:.6f}',
            '-z', f'{origin[2]:.6f}',
            '-R', f'{roll:.6f}',
            '-P', f'{pitch:.6f}',
            '-Y', f'{yaw:.6f}',
        ],
        'spawn camera target',
    )

    print(
        '[camera_test_ctl] spawned '
        f'{args.name}: center_robot='
        f'({args.center_x:+.3f},{args.center_y:+.3f},'
        f'{args.center_z:+.3f})m '
        f'rel_RPY=({args.roll_deg:+.1f},'
        f'{args.pitch_deg:+.1f},{args.yaw_deg:+.1f})deg',
        flush=True,
    )


def run_scan(args, axis):
    values = [float(v) for v in args.values]
    print(
        f'[camera_test_ctl] {axis} scan: {len(values)} positions, '
        f'hold={args.hold:.2f}s',
        flush=True,
    )

    for index, value in enumerate(values, start=1):
        delete_target(args.world, args.name, ignore_missing=True)
        time.sleep(max(0.0, args.delete_wait))

        if axis == 'horizontal':
            args.center_y = value
        elif axis == 'vertical':
            args.center_z = value
        elif axis in ('ground', 'depth'):
            args.center_x = value
        else:
            raise ValueError(axis)

        args.delay = 0.0
        print(
            f'[camera_test_ctl] scan {index}/{len(values)} '
            f'{axis}={value:+.3f} m',
            flush=True,
        )
        spawn_target(args)
        time.sleep(max(0.1, args.hold))

    print(
        f'[camera_test_ctl] {axis} scan complete; '
        'last target remains spawned.',
        flush=True,
    )


def add_scan_common(parser):
    parser.add_argument('--world', default=WORLD)
    parser.add_argument('--name', default=MODEL_NAME)
    parser.add_argument('--center-x', type=float, default=1.0)
    parser.add_argument('--center-y', type=float, default=0.0)
    parser.add_argument('--center-z', type=float, default=0.0367)
    parser.add_argument('--roll-deg', type=float, default=0.0)
    parser.add_argument('--pitch-deg', type=float, default=90.0)
    parser.add_argument('--yaw-deg', type=float, default=0.0)
    parser.add_argument('--hold', type=float, default=2.0)
    parser.add_argument('--delete-wait', type=float, default=0.2)


def add_spawn_args(parser):
    parser.add_argument('--world', default=WORLD)
    parser.add_argument('--name', default=MODEL_NAME)
    parser.add_argument('--center-x', type=float, default=1.0)
    parser.add_argument('--center-y', type=float, default=0.0)
    parser.add_argument(
        '--center-z',
        type=float,
        default=0.0367,
        help='Target collection-center height above base_footprint [m].',
    )
    parser.add_argument('--roll-deg', type=float, default=0.0)
    parser.add_argument('--pitch-deg', type=float, default=90.0)
    parser.add_argument('--yaw-deg', type=float, default=0.0)
    parser.add_argument('--delay', type=float, default=2.0)


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Move a non-colliding visual shuttle target through the '
            'persistent camera test world.'
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
    respawn.add_argument('--delete-wait', type=float, default=0.2)

    horizontal = sub.add_parser(
        'horizontal-scan',
        help='Sweep target laterally across the camera frame.',
    )
    add_scan_common(horizontal)
    horizontal.add_argument(
        '--values',
        nargs='+',
        type=float,
        default=[-1.0, -0.75, -0.50, -0.25, 0.0, 0.25, 0.50, 0.75, 1.0],
    )

    vertical = sub.add_parser(
        'vertical-scan',
        help='Sweep target height across the camera frame.',
    )
    add_scan_common(vertical)
    vertical.add_argument(
        '--values',
        nargs='+',
        type=float,
        default=[0.04, 0.10, 0.20, 0.30, 0.40, 0.50, 0.65, 0.80],
    )

    ground = sub.add_parser(
        'ground-scan',
        help='Sweep a floor-level shuttle target through near/far image range.',
    )
    add_scan_common(ground)
    ground.add_argument(
        '--values',
        nargs='+',
        type=float,
        default=[0.20, 0.30, 0.40, 0.50, 0.75, 1.0, 1.5, 2.0, 3.0, 5.0],
    )

    depth = sub.add_parser(
        'depth-scan',
        help=(
            'Sweep a floor-level shuttle through the RGB/depth overlap '
            'range while the monitor compares rendered depth with '
            'ground-truth optical Z.'
        ),
    )
    add_scan_common(depth)
    depth.add_argument(
        '--values',
        nargs='+',
        type=float,
        default=[0.46, 0.50, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 5.5, 5.9],
    )

    args = parser.parse_args()

    try:
        if args.command == 'spawn':
            spawn_target(args)
        elif args.command == 'delete':
            delete_target(
                args.world,
                args.name,
                ignore_missing=args.ignore_missing,
            )
        elif args.command == 'respawn':
            delete_target(
                args.world,
                args.name,
                ignore_missing=True,
            )
            time.sleep(max(0.0, args.delete_wait))
            spawn_target(args)
        elif args.command == 'horizontal-scan':
            run_scan(args, 'horizontal')
        elif args.command == 'vertical-scan':
            run_scan(args, 'vertical')
        elif args.command == 'ground-scan':
            run_scan(args, 'ground')
        elif args.command == 'depth-scan':
            run_scan(args, 'depth')
    except Exception as exc:
        print(f'[camera_test_ctl] ERROR: {exc}', file=sys.stderr)
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
