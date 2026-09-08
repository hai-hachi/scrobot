#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_csv(path):
    with open(path, newline='') as handle:
        return list(csv.DictReader(handle))


def col(rows, name):
    values = []
    for row in rows:
        try:
            values.append(float(row[name]))
        except (ValueError, KeyError):
            values.append(float('nan'))
    return np.asarray(values, dtype=float)


def finite(values):
    return values[np.isfinite(values)]


def rmse(values):
    values = finite(values)
    if values.size == 0:
        return float('nan')
    return float(np.sqrt(np.mean(values ** 2)))


def mean(values):
    values = finite(values)
    return float(np.mean(values)) if values.size else float('nan')


def maximum(values):
    values = finite(values)
    return float(np.max(values)) if values.size else float('nan')


def write_summary(output_dir, metrics):
    path = output_dir / 'summary.csv'
    with open(path, 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['metric', 'value'])
        for key, value in metrics.items():
            writer.writerow([key, value])
    return path


def plot_series(x, y, xlabel, ylabel, title, path):
    mask = np.isfinite(x) & np.isfinite(y)
    if not np.any(mask):
        return
    plt.figure()
    plt.plot(x[mask], y[mask])
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def plot_tracking(time_s, gt_cte, est_cte, path):
    mask_gt = np.isfinite(time_s) & np.isfinite(gt_cte)
    mask_est = np.isfinite(time_s) & np.isfinite(est_cte)
    if not np.any(mask_gt) and not np.any(mask_est):
        return
    plt.figure()
    if np.any(mask_gt):
        plt.plot(time_s[mask_gt], gt_cte[mask_gt], label='Ground-truth tracking error')
    if np.any(mask_est):
        plt.plot(time_s[mask_est], est_cte[mask_est], label='Estimated tracking error')
    plt.xlabel('Time [s]')
    plt.ylabel('Cross-track error [m]')
    plt.title('Path tracking error')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def plot_trajectory(gt_x, gt_y, est_x, est_y, plans_rows, path):
    plt.figure()
    gt_mask = np.isfinite(gt_x) & np.isfinite(gt_y)
    est_mask = np.isfinite(est_x) & np.isfinite(est_y)

    if np.any(gt_mask):
        plt.plot(gt_x[gt_mask], gt_y[gt_mask], label='Ground truth')
    if np.any(est_mask):
        plt.plot(est_x[est_mask], est_y[est_mask], label='Estimated')

    if plans_rows:
        latest_revision = max(int(row['plan_revision']) for row in plans_rows)
        latest = [row for row in plans_rows if int(row['plan_revision']) == latest_revision]
        if latest:
            px = [float(row['x']) for row in latest]
            py = [float(row['y']) for row in latest]
            plt.plot(px, py, '--', label=f'Latest Nav2 plan #{latest_revision}')

    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Trajectory comparison')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def build_event_summary(rows, event_rows, settle_time):
    if not event_rows:
        return []

    time_s = col(rows, 'ros_time')
    pos_error = col(rows, 'position_error')
    yaw_error = col(rows, 'yaw_error')
    result = []

    for event in event_rows:
        event_time = float(event['ros_time'])
        before_indices = np.where(np.isfinite(pos_error) & (time_s <= event_time))[0]
        after_indices = np.where(np.isfinite(pos_error) & (time_s >= event_time + settle_time))[0]

        before = before_indices[-1] if before_indices.size else None
        after = after_indices[0] if after_indices.size else None

        row = {
            'event_index': event['event_index'],
            'ros_time': event_time,
            'distance_since_previous_relocalization': float(event['distance_since_previous_relocalization']),
            'position_error_before': float(pos_error[before]) if before is not None else float('nan'),
            'position_error_after': float(pos_error[after]) if after is not None else float('nan'),
            'yaw_error_before_deg': math.degrees(float(yaw_error[before])) if before is not None and math.isfinite(yaw_error[before]) else float('nan'),
            'yaw_error_after_deg': math.degrees(float(yaw_error[after])) if after is not None and math.isfinite(yaw_error[after]) else float('nan'),
        }
        if math.isfinite(row['position_error_before']) and math.isfinite(row['position_error_after']):
            row['position_error_improvement'] = row['position_error_before'] - row['position_error_after']
        else:
            row['position_error_improvement'] = float('nan')
        result.append(row)

    return result


def write_event_summary(output_dir, rows):
    path = output_dir / 'relocalization_summary.csv'
    fields = [
        'event_index', 'ros_time', 'distance_since_previous_relocalization',
        'position_error_before', 'position_error_after', 'position_error_improvement',
        'yaw_error_before_deg', 'yaw_error_after_deg'
    ]
    with open(path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return path


def main():
    parser = argparse.ArgumentParser(description='Analyze one SC Robot evaluation run.')
    parser.add_argument('run_directory', help='Directory containing samples.csv')
    parser.add_argument('--event-settle-time', type=float, default=0.25, help='Seconds after relocalization before measuring post-fix error.')
    args = parser.parse_args()

    run_dir = Path(args.run_directory).expanduser().resolve()
    samples_path = run_dir / 'samples.csv'
    events_path = run_dir / 'relocalization_events.csv'
    plans_path = run_dir / 'plans.csv'

    if not samples_path.exists():
        raise SystemExit(f'Missing {samples_path}')

    rows = load_csv(samples_path)
    event_rows = load_csv(events_path) if events_path.exists() else []
    plans_rows = load_csv(plans_path) if plans_path.exists() else []

    t = col(rows, 'ros_time')
    if np.any(np.isfinite(t)):
        t = t - finite(t)[0]

    pos_error = col(rows, 'position_error')
    yaw_error = col(rows, 'yaw_error')
    gt_cte = col(rows, 'gt_cross_track_error')
    est_cte = col(rows, 'est_cross_track_error')
    dist_since_fix = col(rows, 'distance_since_relocalization')

    gt_x = col(rows, 'gt_x')
    gt_y = col(rows, 'gt_y')
    est_x = col(rows, 'est_x')
    est_y = col(rows, 'est_y')

    metrics = {
        'samples_total': len(rows),
        'localization_position_rmse_m': rmse(pos_error),
        'localization_position_mean_m': mean(pos_error),
        'localization_position_max_m': maximum(pos_error),
        'localization_yaw_rmse_deg': math.degrees(rmse(yaw_error)) if math.isfinite(rmse(yaw_error)) else float('nan'),
        'localization_yaw_mean_deg': math.degrees(mean(yaw_error)) if math.isfinite(mean(yaw_error)) else float('nan'),
        'localization_yaw_max_deg': math.degrees(maximum(yaw_error)) if math.isfinite(maximum(yaw_error)) else float('nan'),
        'actual_cross_track_rmse_m': rmse(gt_cte),
        'actual_cross_track_max_m': maximum(gt_cte),
        'estimated_cross_track_rmse_m': rmse(est_cte),
        'estimated_cross_track_max_m': maximum(est_cte),
        'relocalization_count': len(event_rows),
        'ground_truth_path_length_m': float(col(rows, 'gt_distance_total')[-1]) if rows else float('nan'),
        'estimated_path_length_m': float(col(rows, 'est_distance_total')[-1]) if rows else float('nan'),
    }

    write_summary(run_dir, metrics)

    plot_series(t, pos_error, 'Time [s]', 'Position error [m]', 'Localization position error', run_dir / 'localization_position_error.png')
    plot_series(t, np.degrees(yaw_error), 'Time [s]', 'Yaw error [deg]', 'Localization yaw error', run_dir / 'localization_yaw_error.png')
    plot_series(dist_since_fix, pos_error, 'Ground-truth distance since relocalization [m]', 'Position error [m]', 'Localization error vs distance since last fix', run_dir / 'error_vs_distance_since_fix.png')
    plot_tracking(t, gt_cte, est_cte, run_dir / 'path_tracking_error.png')
    plot_trajectory(gt_x, gt_y, est_x, est_y, plans_rows, run_dir / 'trajectory.png')

    event_summary = build_event_summary(rows, event_rows, args.event_settle_time)
    write_event_summary(run_dir, event_summary)

    print(f'Analysis complete: {run_dir}')
    print(f'Position RMSE: {metrics["localization_position_rmse_m"]:.4f} m')
    print(f'Yaw RMSE: {metrics["localization_yaw_rmse_deg"]:.3f} deg')
    print(f'Actual path tracking RMSE: {metrics["actual_cross_track_rmse_m"]:.4f} m')
    print(f'Relocalizations: {metrics["relocalization_count"]}')


if __name__ == '__main__':
    main()
