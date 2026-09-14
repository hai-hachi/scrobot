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
        ax.plot(ex, ey, label='Estimated map->base', linewidth=1.2)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('Collection-session trajectory')
    ax.grid(True)
    ax.legend()
    save_figure(fig, output_dir, 'collection_trajectory')


def plot_collection_progress(rows, output_dir):
    clean = []
    for row in rows:
        t = fvalue(row, 'elapsed_s')
        if not math.isfinite(t):
            continue
        clean.append((
            t,
            fvalue(row, 'collected_shuttles', 0.0),
            fvalue(row, 'remaining_eligible', 0.0),
            fvalue(row, 'remaining_near_poles', 0.0),
        ))
    if not clean:
        return

    t, collected, remaining_eligible, ignored = zip(*clean)
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t, collected, label='Collected')
    ax.plot(t, remaining_eligible, label='Remaining eligible')
    ax.plot(t, ignored, label='Ignored near poles')
    ax.set_xlabel('Elapsed time [s]')
    ax.set_ylabel('Shuttle count')
    ax.set_title('Collection progress')
    ax.grid(True)
    ax.legend()
    save_figure(fig, output_dir, 'collection_progress')


def plot_collection_rate(rows, output_dir):
    clean = []
    for row in rows:
        t = fvalue(row, 'elapsed_s')
        eligible_rate = fvalue(row, 'eligible_collection_rate_percent')
        overall_rate = fvalue(row, 'overall_collection_rate_percent')
        if math.isfinite(t):
            clean.append((t, eligible_rate, overall_rate))
    if not clean:
        return

    t, eligible_rate, overall_rate = zip(*clean)
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t, eligible_rate, label='Eligible collection rate')
    ax.plot(t, overall_rate, label='Overall rate incl. pole exclusions')
    ax.set_xlabel('Elapsed time [s]')
    ax.set_ylabel('Collection rate [%]')
    ax.set_ylim(0, 105)
    ax.set_title('Collection efficiency')
    ax.grid(True)
    ax.legend()
    save_figure(fig, output_dir, 'collection_rate')


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


def plot_duration_table(rows, output_dir, key, title, stem):
    if not rows:
        return
    labels = [row.get(key, '') for row in rows]
    durations = [fvalue(row, 'duration_s', 0.0) for row in rows]
    order = sorted(range(len(labels)), key=lambda i: durations[i], reverse=True)
    labels = [labels[i] for i in order]
    durations = [durations[i] for i in order]

    fig, ax = plt.subplots(figsize=(10, max(4, 0.38 * len(labels))))
    ax.barh(labels, durations)
    ax.invert_yaxis()
    ax.set_xlabel('Time [s]')
    ax.set_title(title)
    ax.grid(True, axis='x')
    save_figure(fig, output_dir, stem)


def plot_collection_passes(rows, output_dir):
    if not rows:
        return
    passes = []
    duration = []
    collected = []
    for row in rows:
        index = int(fvalue(row, 'pass_index', 0.0))
        if index <= 0:
            continue
        passes.append(str(index))
        duration.append(fvalue(row, 'duration_s', 0.0))
        collected.append(fvalue(row, 'collected_this_pass', 0.0))
    if not passes:
        return

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.bar(passes, duration)
    ax.set_xlabel('Local-collect pass')
    ax.set_ylabel('Duration [s]')
    ax.set_title('Local-collection pass duration')
    ax.grid(True, axis='y')
    for i, count in enumerate(collected):
        ax.text(i, duration[i], f'{int(count)} shuttle(s)', ha='center', va='bottom')
    save_figure(fig, output_dir, 'collection_pass_duration')


def print_summary(run_dir):
    summary_path = run_dir / 'summary.csv'
    if not summary_path.exists():
        print('summary.csv not found; plots were generated from available CSV files.')
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
        ('Total spawned/seen', 'total_shuttles_seen'),
        ('Ignored near poles', 'ignored_near_poles'),
        ('Eligible shuttles', 'eligible_shuttles'),
        ('Collected shuttles', 'collected_shuttles'),
        ('Remaining eligible', 'remaining_eligible'),
        ('Eligible collection rate [%]', 'eligible_collection_rate_percent'),
        ('Overall collection rate [%]', 'overall_collection_rate_percent'),
        ('Local-collect passes', 'collection_passes'),
        ('GT path [m]', 'ground_truth_path_m'),
        ('Estimated path [m]', 'estimated_path_m'),
        ('Position RMSE [m]', 'position_rmse_m'),
        ('Distance / collected [m]', 'distance_per_collected_m'),
        ('Time / collected [s]', 'time_per_collected_s'),
        ('Fixed relocalizations', 'fixed_relocalizations'),
        ('Relocalization time [s]', 'fixed_relocalization_time_s'),
        ('Sweep time [s]', 'sweeping_time_s'),
        ('Local collect time [s]', 'local_collect_time_s'),
        ('Return-to-sweep time [s]', 'return_to_sweep_time_s'),
    ]
    for label, key in fields:
        print(f'{label:31s}: {s.get(key, "")}')


def main():
    parser = argparse.ArgumentParser(
        description='Analyze one SC Robot sweep + local-collection evaluation run.'
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
    states_path = run_dir / 'state_durations.csv'
    phases_path = run_dir / 'local_collect_phase_durations.csv'
    passes_path = run_dir / 'collection_events.csv'

    states = read_csv(states_path) if states_path.exists() else []
    phases = read_csv(phases_path) if phases_path.exists() else []
    passes = read_csv(passes_path) if passes_path.exists() else []

    plot_trajectory(trajectory, output_dir)
    plot_collection_progress(trajectory, output_dir)
    plot_collection_rate(trajectory, output_dir)
    plot_localization_error(trajectory, output_dir)
    plot_duration_table(
        states, output_dir, 'state', 'Time spent in mission states', 'state_time'
    )
    plot_duration_table(
        phases,
        output_dir,
        'phase',
        'Time spent in local-collect phases',
        'local_collect_phase_time',
    )
    plot_collection_passes(passes, output_dir)

    print_summary(run_dir)
    print(f'\nPlots saved in: {output_dir}')


if __name__ == '__main__':
    main()
