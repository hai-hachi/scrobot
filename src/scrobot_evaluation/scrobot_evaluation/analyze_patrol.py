#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


LINE_WIDTH = 0.8
GRID_LINE_WIDTH = 0.4
POINT_SIZE = 18


def load_csv(path):
    with open(path, newline='') as handle:
        return list(csv.DictReader(handle))


def numeric(rows, key):
    values = []
    for row in rows:
        try:
            values.append(float(row[key]))
        except (KeyError, ValueError, TypeError):
            values.append(float('nan'))
    return np.asarray(values, dtype=float)


def finite(values):
    return values[np.isfinite(values)]


def mean(values):
    values = finite(values)
    return float(np.mean(values)) if values.size else float('nan')


def maximum(values):
    values = finite(values)
    return float(np.max(values)) if values.size else float('nan')


def rmse(values):
    values = finite(values)
    return float(np.sqrt(np.mean(values ** 2))) if values.size else float('nan')


def save_summary(run_dir, metrics):
    path = run_dir / 'summary.csv'
    with open(path, 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['metric', 'value'])
        for key, value in metrics.items():
            writer.writerow([key, value])
    return path


def save_svg(path):
    plt.tight_layout()
    plt.savefig(path, format='svg')
    plt.close()


def plot_trajectory(run_dir, samples, points):
    gt_x = numeric(samples, 'gt_x')
    gt_y = numeric(samples, 'gt_y')
    est_x = numeric(samples, 'est_x')
    est_y = numeric(samples, 'est_y')

    plt.figure()
    gt_mask = np.isfinite(gt_x) & np.isfinite(gt_y)
    est_mask = np.isfinite(est_x) & np.isfinite(est_y)

    if np.any(gt_mask):
        plt.plot(
            gt_x[gt_mask],
            gt_y[gt_mask],
            linewidth=LINE_WIDTH,
            label='Ground truth',
        )
    if np.any(est_mask):
        plt.plot(
            est_x[est_mask],
            est_y[est_mask],
            linewidth=LINE_WIDTH,
            label='Estimated',
        )

    if points:
        px = numeric(points, 'x')
        py = numeric(points, 'y')
        mask = np.isfinite(px) & np.isfinite(py)
        if np.any(mask):
            plt.scatter(
                px[mask],
                py[mask],
                s=POINT_SIZE,
                label='Patrol points',
            )
            for row in points:
                try:
                    plt.text(
                        float(row['x']),
                        float(row['y']),
                        f"P{row['index']}",
                        fontsize=8,
                    )
                except (ValueError, KeyError):
                    pass

    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Patrol trajectory')
    plt.axis('equal')
    plt.grid(True, linewidth=GRID_LINE_WIDTH)
    plt.legend()
    save_svg(run_dir / 'trajectory.svg')


def plot_checkpoint_errors(run_dir, checkpoints):
    if not checkpoints:
        return

    idx = numeric(checkpoints, 'index')
    pos = numeric(checkpoints, 'gt_position_error_m')
    yaw = numeric(checkpoints, 'gt_yaw_error_deg')

    mask = np.isfinite(idx) & np.isfinite(pos)
    if np.any(mask):
        plt.figure()
        plt.bar(idx[mask], pos[mask], linewidth=0.4)
        plt.xlabel('Patrol point')
        plt.ylabel('Position error [m]')
        plt.title('Actual checkpoint position error')
        plt.grid(True, axis='y', linewidth=GRID_LINE_WIDTH)
        save_svg(run_dir / 'checkpoint_position_error.svg')

    mask = np.isfinite(idx) & np.isfinite(yaw)
    if np.any(mask):
        plt.figure()
        plt.bar(idx[mask], yaw[mask], linewidth=0.4)
        plt.xlabel('Patrol point')
        plt.ylabel('Yaw error [deg]')
        plt.title('Actual checkpoint yaw error')
        plt.grid(True, axis='y', linewidth=GRID_LINE_WIDTH)
        save_svg(run_dir / 'checkpoint_yaw_error.svg')


def plot_spin_metrics(run_dir, spins):
    if not spins:
        return

    idx = numeric(spins, 'index')
    overshoot = numeric(spins, 'overshoot_deg')
    correction = numeric(spins, 'correction_after_peak_deg')

    mask = np.isfinite(idx) & np.isfinite(overshoot) & np.isfinite(correction)
    if not np.any(mask):
        return

    x = idx[mask]
    width = 0.35
    plt.figure()
    plt.bar(
        x - width / 2.0,
        overshoot[mask],
        width=width,
        linewidth=0.4,
        label='Overshoot',
    )
    plt.bar(
        x + width / 2.0,
        correction[mask],
        width=width,
        linewidth=0.4,
        label='Correction',
    )
    plt.xlabel('Patrol point')
    plt.ylabel('Angle [deg]')
    plt.title('360-degree scan overshoot and correction')
    plt.grid(True, axis='y', linewidth=GRID_LINE_WIDTH)
    plt.legend()
    save_svg(run_dir / 'spin_overshoot.svg')


def plot_cross_track(run_dir, samples):
    t = numeric(samples, 'ros_time')
    cte = numeric(samples, 'gt_plan_cross_track_error')
    mask = np.isfinite(t) & np.isfinite(cte)
    if not np.any(mask):
        return

    t = t - t[mask][0]
    plt.figure()
    plt.plot(
        t[mask],
        cte[mask],
        linewidth=LINE_WIDTH,
    )
    plt.xlabel('Time [s]')
    plt.ylabel('Cross-track error [m]')
    plt.title('Ground-truth tracking error to active Nav2 plan')
    plt.grid(True, linewidth=GRID_LINE_WIDTH)
    save_svg(run_dir / 'path_tracking_error.svg')


def main():
    parser = argparse.ArgumentParser(
        description='Analyze one SC Robot patrol trajectory run.'
    )
    parser.add_argument('run_directory')
    args = parser.parse_args()

    run_dir = Path(args.run_directory).expanduser().resolve()
    trajectory_path = run_dir / 'trajectory.csv'
    if not trajectory_path.exists():
        raise SystemExit(f'Missing {trajectory_path}')

    samples = load_csv(trajectory_path)
    checkpoints = (
        load_csv(run_dir / 'checkpoints.csv')
        if (run_dir / 'checkpoints.csv').exists()
        else []
    )
    spins = (
        load_csv(run_dir / 'spins.csv')
        if (run_dir / 'spins.csv').exists()
        else []
    )
    points = (
        load_csv(run_dir / 'patrol_points.csv')
        if (run_dir / 'patrol_points.csv').exists()
        else []
    )

    ros_time = numeric(samples, 'ros_time')
    loc_pos = numeric(samples, 'localization_position_error')
    loc_yaw = numeric(samples, 'localization_yaw_error')
    gt_cte = numeric(samples, 'gt_plan_cross_track_error')

    checkpoint_pos = numeric(checkpoints, 'gt_position_error_m')
    checkpoint_yaw = numeric(checkpoints, 'gt_yaw_error_deg')
    nav_time = numeric(checkpoints, 'navigation_time_s')
    nav_length = numeric(checkpoints, 'navigation_path_length_m')

    overshoot = numeric(spins, 'overshoot_deg')
    correction = numeric(spins, 'correction_after_peak_deg')
    spin_drift = numeric(spins, 'position_drift_m')
    spin_duration = numeric(spins, 'duration_s')
    spin_net = numeric(spins, 'net_rotation_deg')
    spin_abs = numeric(spins, 'absolute_rotation_deg')

    duration = float('nan')
    valid_time = finite(ros_time)
    if valid_time.size >= 2:
        duration = float(valid_time[-1] - valid_time[0])

    gt_total = numeric(samples, 'gt_distance_total')
    est_total = numeric(samples, 'est_distance_total')

    loc_yaw_rmse = rmse(loc_yaw)

    metrics = {
        'mission_logged_duration_s': duration,
        'patrol_points_recorded': len(points),
        'checkpoints_completed': len(checkpoints),
        'spins_completed': len(spins),
        'ground_truth_total_path_length_m': (
            float(finite(gt_total)[-1])
            if finite(gt_total).size
            else float('nan')
        ),
        'estimated_total_path_length_m': (
            float(finite(est_total)[-1])
            if finite(est_total).size
            else float('nan')
        ),
        'localization_position_rmse_m': rmse(loc_pos),
        'localization_yaw_rmse_deg': (
            math.degrees(loc_yaw_rmse)
            if math.isfinite(loc_yaw_rmse)
            else float('nan')
        ),
        'plan_cross_track_rmse_m': rmse(gt_cte),
        'plan_cross_track_max_m': maximum(gt_cte),
        'checkpoint_position_error_mean_m': mean(checkpoint_pos),
        'checkpoint_position_error_max_m': maximum(checkpoint_pos),
        'checkpoint_yaw_error_mean_deg': mean(checkpoint_yaw),
        'checkpoint_yaw_error_max_deg': maximum(checkpoint_yaw),
        'navigation_time_mean_s': mean(nav_time),
        'navigation_path_length_mean_m': mean(nav_length),
        'spin_duration_mean_s': mean(spin_duration),
        'spin_net_rotation_mean_deg': mean(spin_net),
        'spin_absolute_rotation_mean_deg': mean(spin_abs),
        'spin_overshoot_mean_deg': mean(overshoot),
        'spin_overshoot_max_deg': maximum(overshoot),
        'spin_correction_mean_deg': mean(correction),
        'spin_correction_max_deg': maximum(correction),
        'spin_position_drift_mean_m': mean(spin_drift),
        'spin_position_drift_max_m': maximum(spin_drift),
    }

    summary_path = save_summary(run_dir, metrics)
    plot_trajectory(run_dir, samples, points)
    plot_checkpoint_errors(run_dir, checkpoints)
    plot_spin_metrics(run_dir, spins)
    plot_cross_track(run_dir, samples)

    print(f'Analysis complete: {run_dir}')
    print(f'Summary: {summary_path}')
    print(f'Checkpoints: {len(checkpoints)}')
    print(
        'Checkpoint mean position error: '
        f'{metrics["checkpoint_position_error_mean_m"]:.4f} m'
    )
    print(
        f'Plan cross-track RMSE: '
        f'{metrics["plan_cross_track_rmse_m"]:.4f} m'
    )
    print(
        f'Spin mean overshoot: '
        f'{metrics["spin_overshoot_mean_deg"]:.3f} deg'
    )
    print(
        f'Spin max overshoot: '
        f'{metrics["spin_overshoot_max_deg"]:.3f} deg'
    )
    print(
        f'Spin mean correction: '
        f'{metrics["spin_correction_mean_deg"]:.3f} deg'
    )


if __name__ == '__main__':
    main()
