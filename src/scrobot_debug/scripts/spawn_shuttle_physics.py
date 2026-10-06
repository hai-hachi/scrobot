#!/usr/bin/env python3

import argparse
import math
import os
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

from ament_index_python.packages import get_package_share_directory


LARGE_END_RADIUS = 0.03661542731880105


def spawn_one(world, model_file, name, x, y, z, roll, pitch, yaw):
    cmd = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-world', world,
        '-name', name,
        '-file', model_file,
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
            f'Failed to spawn {name} (return code {result.returncode}):\n'
            f'{result.stdout}'
        )
    return name


def grid_points(count, spacing):
    cols = max(1, math.ceil(math.sqrt(count)))
    rows = max(1, math.ceil(count / cols))

    width = (cols - 1) * spacing
    height = (rows - 1) * spacing

    points = []
    for index in range(count):
        row = index // cols
        col = index % cols
        x = col * spacing - width / 2.0
        y = row * spacing - height / 2.0
        points.append((x, y))
    return points


def main():
    parser = argparse.ArgumentParser(
        description='Spawn the production dynamic shuttle model for physics testing.'
    )
    parser.add_argument('--world', default='shuttle_physics_test')
    parser.add_argument('--count', type=int, default=1)
    parser.add_argument('--spacing', type=float, default=0.15)
    parser.add_argument('--drop-height', type=float, default=0.0)
    parser.add_argument(
        '--orientation',
        choices=['sideways', 'upright'],
        default='sideways',
    )
    parser.add_argument('--workers', type=int, default=8)
    args = parser.parse_args()

    count = max(1, args.count)
    spacing = max(0.09, args.spacing)
    workers = max(1, min(args.workers, count))

    simulation_share = get_package_share_directory('scrobot_simulation')
    model_file = os.path.join(
        simulation_share,
        'models',
        'shuttle',
        'model.sdf',
    )
    if not os.path.isfile(model_file):
        raise FileNotFoundError(model_file)

    # With the shuttle axis along local +Z, +90 deg pitch places that axis
    # approximately along world +X. The large skirt radius then sets the
    # required center height for a side-resting shuttle.
    if args.orientation == 'sideways':
        roll = 0.0
        pitch = math.pi / 2.0
        base_z = LARGE_END_RADIUS
    else:
        roll = 0.0
        pitch = 0.0
        base_z = 0.0

    z = base_z + max(0.0, args.drop_height)
    points = grid_points(count, spacing)

    started = time.monotonic()
    futures = []
    with ThreadPoolExecutor(max_workers=workers) as executor:
        for index, (x, y) in enumerate(points):
            name = f'shuttle_physics_{index:03d}'
            # Rotate each octagon by a deterministic 45-deg sequence. This
            # avoids every body resting on exactly the same face in many-body
            # performance tests while remaining fully repeatable.
            yaw = (index % 8) * (math.pi / 4.0)
            futures.append(
                executor.submit(
                    spawn_one,
                    args.world,
                    model_file,
                    name,
                    x,
                    y,
                    z,
                    roll,
                    pitch,
                    yaw,
                )
            )

        completed = 0
        for future in as_completed(futures):
            future.result()
            completed += 1

    elapsed = time.monotonic() - started
    print(
        f'[shuttle_physics_spawner] spawned={completed} '
        f'orientation={args.orientation} drop_height={args.drop_height:.3f} m '
        f'spacing={spacing:.3f} m wall_time={elapsed:.3f} s',
        flush=True,
    )


if __name__ == '__main__':
    main()
