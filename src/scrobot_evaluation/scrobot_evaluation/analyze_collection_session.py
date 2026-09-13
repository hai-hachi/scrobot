#!/usr/bin/env python3

import argparse
import csv
import math
import os
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def fvalue(row, key, default=float('nan')):
    try:
        value = row.get(key, '')
        if value in ('', None):
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def resolve_run(args):
    root = Path(os.path.expanduser(args.root)).resolve()
    if args.run:
        candidate = Path(os.path.expanduser(args.run))
        if not candidate.is_absolute():
            candidate = root / candidate
        candidate = candidate.resolve()
        if not candidate.is_dir():
            raise FileNotFoundError(f'Run directory does not exist: {candidate}')
        return candidate

    runs = [p for p in root.iterdir() if p.is_dir()] if root.exists() else []
    if not runs:
        raise FileNotFoundError(f'No collection-session runs found in {root}')
    return max(runs, key=lambda p: p.stat().st_mtime)


def save_figure(fig, output_dir, stem):
    fig.tight_layout()
    fig.savefig(output_dir / f'{stem}.svg', format='svg')
    fig.savefig(output_dir / f'{stem}.png', dpi=180)
    plt.close(fig)


def plot_trajectory(rows, output_dir):
    gt = [(fvalue(r, 'gt_x'), fvalue(r, 'gt_y')) for r in rows]
    est = [(fvalue(r, 'est_x'), fvalue(r, 'est_y')) for r in rows]
    gt = [(x, y) for x, y in gt if math.isfinite(x) and math.isfinite(y)]
    est = [(x, y) for x, y in est if math.isfinite(x) and math.isfinite(y)]

    fig, ax = plt.subplots(figsize=(10, 6))
    if gt:
        gx, gy = zip(*gt)
        ax.plot(gx, gy, label='Gazebo ground truth', linewidth=2.0)
        ax.scatter([gx[0]], [gy[0]], marker='o', label='Start')
        ax.scatter([gx[-1]], [gy[-1]], marker='x', label='End')
    if est:
        ex, ey = zip(*est)
        ax.plot(ex, ey, label='Estimated map->base', linewidth=1.3)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('Collection session trajectory')
    ax.grid(True)
    ax.legend()
    save_figure(fig, output_dir, 'collection_trajectory')


def plot_collection_progress(rows, output_dir):
    t = [fvalue(r, 'elapsed_s') for r in rows]
    collected = [fvalue(r, 'collected_shuttles', 0.0) for r in rows]
    remaining = [fvalue(r, 'remaining_shuttles', 0.0) for r in rows]

    clean = [
        (a, b, c) for a, b, c in zip(t, collected, remaining)
        if math.isfinite(a)
    ]
    if not clean:
        return
    t, collected, remaining = zip(*clean)

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t, collected, label='Collected')
    ax.plot(t, remaining, label='Remaining')
    ax.set_xlabel('Elapsed time [s]')
    ax.set_ylabel('Shuttle count')
    ax.set_title('Collection progress')
    ax.grid(True)
    ax.legend()
    save_figure(fig, output_dir, 'collection_progress')


def plot_localization_error(rows, output_dir):
    points = []
    for row in rows:
        t = fvalue(row, 'elapsed_s')
        err = fvalue(row, 'position_error_m')
        if math.isfinite(t) and math.isfinite(err):
            points.append((t, err))
    if not points:
        return

    t, err = zip(*points)
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t, err)
    ax.set_xlabel('Elapsed time [s]')
    ax.set_ylabel('Position error [m]')
    ax.set_title('Localization error vs Gazebo ground truth')
    ax.grid(True)
    save_figure(fig, output_dir, 'localization_error')


def plot_state_time(rows, output_dir):
    if not rows:
        return
    states = [row['state'] for row in rows]
    durations = [fvalue(row, 'duration_s', 0.0) for row in rows]

    order = sorted(range(len(states)), key=lambda i: durations[i], reverse=True)
    states = [states[i] for i in order]
    durations = [durations[i] for i in order]

    fig, ax = plt.subplots(figsize=(10, max(5, 0.35 * len(states))))
    ax.barh(states, durations)
    ax.invert_yaxis()
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Mission state')
    ax.set_title('Time spent in mission states')
    ax.grid(True, axis='x')
    save_figure(fig, output_dir, 'state_time')


def print_summary(run_dir):
    summary_path = run_dir / 'summary.csv'
    if not summary_path.exists():
        print('summary.csv not found; plots were generated from available raw CSV files.')
        return
    rows = read_csv(summary_path)
    if not rows:
        return
    s = rows[0]
    print('\nCollection session summary')
    print('--------------------------')
    fields = [
        ('Terminal state', 'terminal_state'),
        ('Duration [s]', 'duration_s'),
        ('Collected', 'collected_shuttles'),
        ('Total shuttles seen', 'total_shuttles_seen'),
        ('Collection rate [%]', 'collection_rate_percent'),
        ('Collection passes', 'collection_passes'),
        ('GT path [m]', 'ground_truth_path_m'),
        ('Position RMSE [m]', 'position_rmse_m'),
        ('Distance / collected [m]', 'distance_per_collected_m'),
        ('Time / collected [s]', 'time_per_collected_s'),
        ('Runtime relocalizations', 'runtime_relocalizations'),
        ('Relocalization time [s]', 'runtime_relocalization_time_s'),
    ]
    for label, key in fields:
        print(f'{label:28s}: {s.get(key, "")}')


def main():
    parser = argparse.ArgumentParser(
        description='Plot one SC Robot collection-session evaluation run.'
    )
    parser.add_argument(
        '--root',
        default='~/scrobot_ws/evaluation_results/collection_session',
        help='Root directory containing collection-session run folders.',
    )
    parser.add_argument(
        '--run',
        default='',
        help='Run directory or run name. If omitted, analyze the latest run.',
    )
    args = parser.parse_args()

    run_dir = resolve_run(args)
    output_dir = run_dir / 'plots'
    output_dir.mkdir(parents=True, exist_ok=True)

    trajectory_path = run_dir / 'collection_trajectory.csv'
    if not trajectory_path.exists():
        raise FileNotFoundError(f'Missing {trajectory_path}')

    trajectory = read_csv(trajectory_path)
    states = read_csv(run_dir / 'state_durations.csv') if (run_dir / 'state_durations.csv').exists() else []

    plot_trajectory(trajectory, output_dir)
    plot_collection_progress(trajectory, output_dir)
    plot_localization_error(trajectory, output_dir)
    plot_state_time(states, output_dir)
    print_summary(run_dir)
    print(f'\nPlots saved in: {output_dir}')


if __name__ == '__main__':
    main()
