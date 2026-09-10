#!/usr/bin/env python3

import argparse
import math
import os
import random
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

import yaml
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

    result = subprocess.run(
        cmd,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f'Failed to spawn {name}; ros_gz_sim create returned {result.returncode}:\n'
            f'{result.stdout}'
        )
    return name


def load_yaml(path):
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f'Config root must be a mapping: {path}')
    return data


def get_nested(config, keys, default=None):
    value = config
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def choose_seed(seed_arg, config_seed):
    if seed_arg is not None:
        text = str(seed_arg).strip()
        if text:
            return int(text)

    if config_seed is not None:
        return int(config_seed)

    return random.SystemRandom().randint(1, 999999)


def choose_batch(mode, batch_arg, seed):
    if batch_arg is not None:
        text = str(batch_arg).strip()
        if text:
            return text

    if mode == 'random':
        return str(seed)

    return str(int(time.time()) % 10000)


def build_parser():
    parser = argparse.ArgumentParser(
        description='Spawn dynamic badminton shuttles into an already running Gazebo world.'
    )

    parser.add_argument('--config', default='')
    parser.add_argument('--mode', choices=['single', 'random'], default='single')
    parser.add_argument('--world', default=None)
    parser.add_argument('--visual', choices=['detail', 'fast'], default='detail')

    parser.add_argument('--name', default='')
    parser.add_argument('--batch', default=None)

    parser.add_argument('--x', type=float, default=None)
    parser.add_argument('--y', type=float, default=None)
    parser.add_argument('--z', type=float, default=None)
    parser.add_argument('--roll', type=float, default=None)
    parser.add_argument('--pitch', type=float, default=None)
    parser.add_argument('--yaw', type=float, default=None)

    parser.add_argument('--count', type=int, default=None)
    parser.add_argument(
        '--seed',
        type=int,
        default=None,
        help='Integer for a repeatable random layout; omit it for a new random seed.',
    )
    parser.add_argument('--density', choices=['uniform', 'center', 'net'], default=None)
    parser.add_argument('--court-length', type=float, default=None)
    parser.add_argument('--court-width', type=float, default=None)
    parser.add_argument('--margin', type=float, default=None)
    parser.add_argument('--spawn-height', type=float, default=None)
    parser.add_argument('--parallel-workers', type=int, default=None)

    return parser


def main():
    args = build_parser().parse_args()

    share = get_package_share_directory('scrobot_simulation')
    default_config = os.path.join(share, 'config', 'shuttle_spawn.yaml')
    config_path = args.config or default_config
    config = load_yaml(config_path)

    model_dir = os.path.join(share, 'models', 'shuttle')
    sdf_file = os.path.join(
        model_dir,
        'model_fast.sdf' if args.visual == 'fast' else 'model.sdf',
    )
    visual_mesh = os.path.join(model_dir, 'meshes', 'shuttle.STL')
    fast_visual_mesh = os.path.join(model_dir, 'meshes', 'shuttle_collision.STL')

    required = [sdf_file]
    required.append(fast_visual_mesh if args.visual == 'fast' else visual_mesh)
    missing = [path for path in required if not os.path.exists(path)]
    if missing:
        print('Missing shuttle model files:', file=sys.stderr)
        for path in missing:
            print(f'  {path}', file=sys.stderr)
        return 2

    world = args.world or get_nested(config, ['world'], 'badminton_court')

    sx = args.x if args.x is not None else float(get_nested(config, ['single', 'x'], 0.0))
    sy = args.y if args.y is not None else float(get_nested(config, ['single', 'y'], 0.0))
    sz = args.z if args.z is not None else float(get_nested(config, ['single', 'z'], 0.08))
    sroll = args.roll if args.roll is not None else float(get_nested(config, ['single', 'roll'], 0.0))
    spitch = args.pitch if args.pitch is not None else float(get_nested(config, ['single', 'pitch'], math.pi / 2.0))
    syaw = args.yaw if args.yaw is not None else float(get_nested(config, ['single', 'yaw'], 0.0))

    count = args.count if args.count is not None else int(get_nested(config, ['random', 'count'], 20))
    density = args.density or str(get_nested(config, ['random', 'density'], 'uniform'))
    config_seed = get_nested(config, ['random', 'seed'], None)
    seed = choose_seed(args.seed, config_seed)

    court_length = args.court_length if args.court_length is not None else float(get_nested(config, ['court', 'length'], 13.40))
    court_width = args.court_width if args.court_width is not None else float(get_nested(config, ['court', 'width'], 6.10))
    margin = args.margin if args.margin is not None else float(get_nested(config, ['court', 'margin'], 0.15))
    spawn_height = args.spawn_height if args.spawn_height is not None else float(get_nested(config, ['spawn', 'height'], 0.08))
    parallel_workers = args.parallel_workers if args.parallel_workers is not None else int(get_nested(config, ['spawn', 'parallel_workers'], 8))

    if count < 1:
        raise ValueError('--count must be at least 1')
    if court_length <= 0.0 or court_width <= 0.0:
        raise ValueError('Court dimensions must be positive')
    if margin < 0.0:
        raise ValueError('--margin must be non-negative')
    if parallel_workers < 1:
        raise ValueError('--parallel-workers must be at least 1')

    batch = choose_batch(args.mode, args.batch, seed)

    if args.mode == 'single':
        name = args.name or f'single{batch}'
        spawn_entity(world, name, sdf_file, sx, sy, sz, sroll, spitch, syaw)
        print(
            f'Spawned {name} at ({sx:.3f}, {sy:.3f}, {sz:.3f}) '
            f'RPY=({sroll:.3f}, {spitch:.3f}, {syaw:.3f}), visual={args.visual}'
        )
        return 0

    rng = random.Random(seed)
    specs = []
    for index in range(1, count + 1):
        x, y = sample_xy(rng, density, court_length, court_width, margin)
        yaw = rng.uniform(-math.pi, math.pi)
        name = f'random{batch}-{index:02d}'
        specs.append((world, name, sdf_file, x, y, spawn_height, 0.0, math.pi / 2.0, yaw))

    print(
        f'Spawning {count} shuttles: density={density}, seed={seed}, batch={batch}, '
        f'visual={args.visual}, workers={min(parallel_workers, count)}, '
        f'court={court_length:.2f}x{court_width:.2f} m'
    )

    workers = min(parallel_workers, count)
    failures = []
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {executor.submit(spawn_entity, *spec): spec for spec in specs}
        for future in as_completed(futures):
            spec = futures[future]
            name = spec[1]
            try:
                future.result()
                print(f'  spawned {name}')
            except Exception as exc:
                failures.append((name, str(exc)))
                print(f'  FAILED {name}: {exc}', file=sys.stderr)

    if failures:
        print(f'{len(failures)} of {count} shuttle spawns failed.', file=sys.stderr)
        return 1

    print(f'Finished random batch {batch}; seed={seed}.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
