#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory


DEFAULT_WORLD = 'shuttle_physics_test'
DEFAULT_PREFIX = 'shuttle_physics_'


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
    output = result.stdout.strip()
    if output:
        print(output, flush=True)


def model_name(prefix, index):
    return f'{prefix}{index:03d}'


def delete_model(world, name, ignore_missing=False):
    cmd = [
        'gz', 'service',
        '-s', f'/world/{world}/remove',
        '--reqtype', 'gz.msgs.Entity',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '3000',
        '--req', f'name: "{name}", type: MODEL',
    ]
    try:
        run(cmd, f'delete {name}')
    except RuntimeError:
        if ignore_missing:
            print(f'[shuttle_physics_ctl] {name}: not removed / already absent', flush=True)
            return
        raise
    print(f'[shuttle_physics_ctl] deleted {name}', flush=True)


def spawn_models(args):
    if args.delay > 0.0:
        print(
            f'[shuttle_physics_ctl] spawning in {args.delay:.1f} s...',
            flush=True,
        )
        time.sleep(args.delay)

    cmd = [
        'ros2', 'run', 'scrobot_debug', 'spawn_shuttle_physics',
        '--world', args.world,
        '--count', str(args.count),
        '--spacing', str(args.spacing),
        '--drop-height', str(args.drop_height),
        '--orientation', args.orientation,
        '--workers', str(args.workers),
    ]
    run(cmd, 'spawn shuttle physics models')


def delete_models(args):
    for index in range(args.count):
        delete_model(
            args.world,
            model_name(args.prefix, index),
            ignore_missing=args.ignore_missing,
        )


def respawn_models(args):
    for index in range(args.count):
        delete_model(
            args.world,
            model_name(args.prefix, index),
            ignore_missing=True,
        )

    if args.delete_wait > 0.0:
        print(
            f'[shuttle_physics_ctl] waiting {args.delete_wait:.1f} s '
            'for Gazebo removal...',
            flush=True,
        )
        time.sleep(args.delete_wait)

    spawn_models(args)


def apply_impulse(args):
    entity_name = f'{args.model}::{args.link}'

    if args.delay > 0.0:
        print(
            f'[shuttle_physics_ctl] applying impulse in {args.delay:.1f} s...',
            flush=True,
        )
        time.sleep(args.delay)

    wrench_topic = f'/world/{args.world}/wrench/persistent'
    clear_topic = f'/world/{args.world}/wrench/clear'

    wrench_payload = (
        f'entity: {{name: "{entity_name}", type: LINK}}, '
        'wrench: {'
        f'force: {{x: {args.force_x}, y: {args.force_y}, z: {args.force_z}}}, '
        f'torque: {{x: {args.torque_x}, y: {args.torque_y}, z: {args.torque_z}}}'
        '}'
    )

    run(
        [
            'gz', 'topic',
            '-t', wrench_topic,
            '-m', 'gz.msgs.EntityWrench',
            '-p', wrench_payload,
        ],
        'apply shuttle wrench',
    )

    impulse = (
        args.force_x * args.duration,
        args.force_y * args.duration,
        args.force_z * args.duration,
    )
    print(
        '[shuttle_physics_ctl] wrench applied: '
        f'F=({args.force_x:.4f},{args.force_y:.4f},{args.force_z:.4f}) N '
        f'duration={args.duration:.3f} s '
        f'J=({impulse[0]:.6f},{impulse[1]:.6f},{impulse[2]:.6f}) N*s',
        flush=True,
    )

    time.sleep(args.duration)

    run(
        [
            'gz', 'topic',
            '-t', clear_topic,
            '-m', 'gz.msgs.Entity',
            '-p', f'name: "{entity_name}", type: LINK',
        ],
        'clear shuttle wrench',
    )
    print('[shuttle_physics_ctl] wrench cleared', flush=True)


def clear_wrench(args):
    entity_name = f'{args.model}::{args.link}'
    run(
        [
            'gz', 'topic',
            '-t', f'/world/{args.world}/wrench/clear',
            '-m', 'gz.msgs.Entity',
            '-p', f'name: "{entity_name}", type: LINK',
        ],
        'clear shuttle wrench',
    )
    print('[shuttle_physics_ctl] wrench cleared', flush=True)


def add_spawn_args(parser):
    parser.add_argument('--world', default=DEFAULT_WORLD)
    parser.add_argument('--count', type=int, default=1)
    parser.add_argument('--spacing', type=float, default=0.15)
    parser.add_argument('--drop-height', type=float, default=0.0)
    parser.add_argument(
        '--orientation',
        choices=['sideways', 'upright'],
        default='sideways',
    )
    parser.add_argument('--workers', type=int, default=8)
    parser.add_argument(
        '--delay',
        type=float,
        default=5.0,
        help='Wait before spawning so the user can observe the scene.',
    )


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Control the persistent shuttle-physics test world without '
            'restarting Gazebo.'
        )
    )
    sub = parser.add_subparsers(dest='command', required=True)

    spawn = sub.add_parser('spawn', help='Spawn shuttle physics models.')
    add_spawn_args(spawn)

    delete = sub.add_parser('delete', help='Delete shuttle physics models.')
    delete.add_argument('--world', default=DEFAULT_WORLD)
    delete.add_argument('--count', type=int, default=1)
    delete.add_argument('--prefix', default=DEFAULT_PREFIX)
    delete.add_argument('--ignore-missing', action='store_true')

    respawn = sub.add_parser(
        'respawn',
        help='Delete and recreate shuttle physics models.',
    )
    add_spawn_args(respawn)
    respawn.add_argument('--prefix', default=DEFAULT_PREFIX)
    respawn.add_argument(
        '--delete-wait',
        type=float,
        default=0.5,
        help='Wait after deletion before starting the spawn countdown.',
    )

    impulse = sub.add_parser(
        'impulse',
        help='Apply a finite-duration wrench to one shuttle.',
    )
    impulse.add_argument('--world', default=DEFAULT_WORLD)
    impulse.add_argument('--model', default='shuttle_physics_000')
    impulse.add_argument('--link', default='shuttle_link')
    impulse.add_argument('--delay', type=float, default=5.0)
    impulse.add_argument('--duration', type=float, default=0.05)
    impulse.add_argument('--force-x', type=float, default=0.0)
    impulse.add_argument('--force-y', type=float, default=0.03)
    impulse.add_argument('--force-z', type=float, default=0.0)
    impulse.add_argument('--torque-x', type=float, default=0.0)
    impulse.add_argument('--torque-y', type=float, default=0.0)
    impulse.add_argument('--torque-z', type=float, default=0.0)

    clear = sub.add_parser(
        'clear',
        help='Clear any persistent wrench on one shuttle.',
    )
    clear.add_argument('--world', default=DEFAULT_WORLD)
    clear.add_argument('--model', default='shuttle_physics_000')
    clear.add_argument('--link', default='shuttle_link')

    args = parser.parse_args()

    if hasattr(args, 'count'):
        args.count = max(1, int(args.count))
    if hasattr(args, 'workers'):
        args.workers = max(1, int(args.workers))
    if hasattr(args, 'delay'):
        args.delay = max(0.0, float(args.delay))

    try:
        if args.command == 'spawn':
            spawn_models(args)
        elif args.command == 'delete':
            delete_models(args)
        elif args.command == 'respawn':
            respawn_models(args)
        elif args.command == 'impulse':
            apply_impulse(args)
        elif args.command == 'clear':
            clear_wrench(args)
        else:
            parser.error(f'Unsupported command: {args.command}')
    except Exception as error:
        print(f'[shuttle_physics_ctl] ERROR: {error}', file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
