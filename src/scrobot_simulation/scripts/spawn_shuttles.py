#!/usr/bin/env python3

import argparse
import math
import os
import random
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def sample_xy(rng, density, court_length, court_width, margin):
    half_l = max(0.0, court_length * 0.5 - margin)
    half_w = max(0.0, court_width * 0.5 - margin)

    if density == 'uniform':
        return rng.uniform(-half_l, half_l), rng.uniform(-half_w, half_w)

    if density == 'center':
        sigma_x = max(0.05, court_length / 5.0)
        sigma_y = max(0.05, court_width / 5.0)
        x = clamp(rng.gauss(0.0, sigma_x), -half_l, half_l)
        y = clamp(rng.gauss(0.0, sigma_y), -half_w, half_w)
        return x, y

    if density == 'net':
        sigma_x = max(0.05, court_length * 0.08)
        x = clamp(rng.gauss(0.0, sigma_x), -half_l, half_l)
        y = rng.uniform(-half_w, half_w)
        return x, y

    raise ValueError(f'Unsupported density profile: {density}')


def spawn_entity(world, name, sdf_file, x, y, z, roll, pitch, yaw):
    cmd = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-world', world,
        '-name', name,
        '-file', sdf_file,
        '-x', f'{x:.6f}',
        '-y', f'{y:.6f}',
        '-z', f'{z:.6f}',
        '-R', f'{roll:.6f}',
        '-P', f'{pitch:.6f}',
        '-Y', f'{yaw:.6f}',
    ]

    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        raise RuntimeError(
            f'Failed to spawn {name}; ros_gz_sim create returned '
            f'{result.returncode}'
        )


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            'Spawn one or more dynamic badminton shuttles into an already '
            'running Gazebo world.'
        )
    )

    parser.add_argument('--mode', choices=['single', 'random'], default='single')
    parser.add_argument('--world', default='badminton_court')

    parser.add_argument('--name', default='')
    parser.add_argument('--prefix', default='shuttle')

    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--z', type=float, default=0.08)
    parser.add_argument('--roll', type=float, default=0.0)
    parser.add_argument('--pitch', type=float, default=math.pi / 2.0)
    parser.add_argument('--yaw', type=float, default=0.0)

    parser.add_argument('--count', type=int, default=20)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument(
        '--density',
        choices=['uniform', 'center', 'net'],
        default='uniform',
    )
    parser.add_argument('--court-length', type=float, default=13.40)
    parser.add_argument('--court-width', type=float, default=6.10)
    parser.add_argument('--margin', type=float, default=0.15)
    parser.add_argument('--spawn-height', type=float, default=0.08)
    parser.add_argument('--spawn-delay', type=float, default=0.05)

    return parser


def main():
    args = build_parser().parse_args()

    if args.count < 1:
        raise ValueError('--count must be at least 1')
    if args.court_length <= 0.0 or args.court_width <= 0.0:
        raise ValueError('Court dimensions must be positive')
    if args.margin < 0.0:
        raise ValueError('--margin must be non-negative')

    share = get_package_share_directory('scrobot_simulation')
    model_dir = os.path.join(share, 'models', 'shuttle')
    sdf_file = os.path.join(model_dir, 'model.sdf')
    visual_mesh = os.path.join(model_dir, 'meshes', 'shuttle.STL')
    collision_mesh = os.path.join(model_dir, 'meshes', 'shuttle_collision.STL')

    missing = [
        path for path in (sdf_file, visual_mesh, collision_mesh)
        if not os.path.exists(path)
    ]
    if missing:
        print('Missing shuttle model files:', file=sys.stderr)
        for path in missing:
            print(f'  {path}', file=sys.stderr)
        print(
            'Place shuttle.STL and shuttle_collision.STL in '
            'scrobot_simulation/models/shuttle/meshes and rebuild.',
            file=sys.stderr,
        )
        return 2

    run_id = int(time.time() * 1000)

    if args.mode == 'single':
        name = args.name or f'{args.prefix}_{run_id}'
        spawn_entity(
            args.world,
            name,
            sdf_file,
            args.x,
            args.y,
            args.z,
            args.roll,
            args.pitch,
            args.yaw,
        )
        print(
            f'Spawned {name} at '
            f'({args.x:.3f}, {args.y:.3f}, {args.z:.3f}) '
            f'RPY=({args.roll:.3f}, {args.pitch:.3f}, {args.yaw:.3f})'
        )
        return 0

    rng = random.Random(args.seed)
    print(
        f'Spawning {args.count} shuttles: density={args.density}, '
        f'seed={args.seed}, court={args.court_length:.2f}x{args.court_width:.2f} m'
    )

    for index in range(args.count):
        x, y = sample_xy(
            rng,
            args.density,
            args.court_length,
            args.court_width,
            args.margin,
        )
        yaw = rng.uniform(-math.pi, math.pi)
        name = f'{args.prefix}_{run_id}_{index:03d}'

        spawn_entity(
            args.world,
            name,
            sdf_file,
            x,
            y,
            args.spawn_height,
            0.0,
            math.pi / 2.0,
            yaw,
        )
        print(f'  {name}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')

        if args.spawn_delay > 0.0:
            time.sleep(args.spawn_delay)

    return 0


if __name__ == '__main__':
    sys.exit(main())
