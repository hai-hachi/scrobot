#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ACTIVE_STATES = {
    'STRAIGHT_FWD',
    'STRAIGHT_REV',
    'ROTATE_CCW',
    'ROTATE_CW',
    'ARC_CCW',
    'ARC_CW',
}


def load_csv(path):
    with open(path, newline='') as handle:
        return list(csv.DictReader(handle))


def col(rows, name):
    values = []
    for row in rows:
        try:
            values.append(float(row[name]))
        except (KeyError, ValueError, TypeError):
            values.append(float('nan'))
    return np.asarray(values, dtype=float)


def text_col(rows, name):
    return np.asarray([row.get(name, '') for row in rows], dtype=object)


def wrap_angle_array(values):
    return np.arctan2(np.sin(values), np.cos(values))


def finite(values):
    return values[np.isfinite(values)]


def mean(values):
    values = finite(values)
    return float(np.mean(values)) if values.size else float('nan')


def maximum(values):
    values = finite(values)
    return float(np.max(values)) if values.size else float('nan')


def std(values):
    values = finite(values)
    return float(np.std(values)) if values.size else float('nan')


def rmse(values):
    values = finite(values)
    return float(np.sqrt(np.mean(values * values))) if values.size else float('nan')


def first_finite_index(*arrays):
    if not arrays:
        return None
    mask = np.ones(len(arrays[0]), dtype=bool)
    for values in arrays:
        mask &= np.isfinite(values)
    indices = np.where(mask)[0]
    return int(indices[0]) if indices.size else None


def relative_pose(x, y, yaw):
    index = first_finite_index(x, y, yaw)
    rx = np.full_like(x, np.nan, dtype=float)
    ry = np.full_like(y, np.nan, dtype=float)
    ryaw = np.full_like(yaw, np.nan, dtype=float)

    if index is None:
        return rx, ry, ryaw

    x0 = x[index]
    y0 = y[index]
    yaw0 = yaw[index]
    c = math.cos(yaw0)
    s = math.sin(yaw0)

    dx = x - x0
    dy = y - y0
    rx = c * dx + s * dy
    ry = -s * dx + c * dy
    ryaw = wrap_angle_array(yaw - yaw0)
    return rx, ry, ryaw


def pose_error(gt, estimate):
    gx, gy, gyaw = gt
    ex, ey, eyaw = estimate
    position = np.hypot(ex - gx, ey - gy)
    yaw = np.abs(wrap_angle_array(eyaw - gyaw))
    return position, yaw


def path_length(x, y, mask=None):
    valid = np.isfinite(x) & np.isfinite(y)
    if mask is not None:
        valid &= mask
    indices = np.where(valid)[0]
    if indices.size < 2:
        return float('nan')

    total = 0.0
    previous = indices[0]
    for current in indices[1:]:
        if current != previous + 1:
            previous = current
            continue
        step = math.hypot(x[current] - x[previous], y[current] - y[previous])
        if step < 1.0:
            total += step
        previous = current
    return total


def phase_delta(x, y, yaw, mask):
    indices = np.where(mask & np.isfinite(x) & np.isfinite(y) & np.isfinite(yaw))[0]
    if indices.size < 2:
        return float('nan'), float('nan')
    first = indices[0]
    last = indices[-1]
    distance = math.hypot(x[last] - x[first], y[last] - y[first])
    yaw_change = abs(math.atan2(
        math.sin(yaw[last] - yaw[first]),
        math.cos(yaw[last] - yaw[first]),
    ))
    return distance, yaw_change


def gt_velocity(t, x, y, yaw):
    vx = np.full_like(t, np.nan, dtype=float)
    wz = np.full_like(t, np.nan, dtype=float)
    valid = np.isfinite(t) & np.isfinite(x) & np.isfinite(y) & np.isfinite(yaw)
    indices = np.where(valid)[0]
    if indices.size < 3:
        return vx, wz

    tv = t[indices]
    xv = x[indices]
    yv = y[indices]
    yaw_unwrapped = np.unwrap(yaw[indices])

    dx = np.gradient(xv, tv)
    dy = np.gradient(yv, tv)
    omega = np.gradient(yaw_unwrapped, tv)
    forward = dx * np.cos(yaw[indices]) + dy * np.sin(yaw[indices])

    vx[indices] = forward
    wz[indices] = omega
    return vx, wz


def save_figure(path):
    base = Path(path).with_suffix('')
    plt.savefig(base.with_suffix('.svg'), bbox_inches='tight')


def plot_lines(path, title, xlabel, ylabel, x, series, equal=False):
    plt.figure()
    plotted = False
    for label, values in series:
        mask = np.isfinite(x) & np.isfinite(values)
        if np.any(mask):
            plt.plot(x[mask], values[mask], label=label, linewidth=0.8)
            plotted = True
    if not plotted:
        plt.close()
        return
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)
    if len(series) > 1:
        plt.legend()
    if equal:
        plt.axis('equal')
    plt.tight_layout()
    save_figure(path)
    plt.close()


def plot_trajectory(path, series):
    plt.figure()
    plotted = False
    for label, x, y in series:
        mask = np.isfinite(x) & np.isfinite(y)
        if np.any(mask):
            plt.plot(x[mask], y[mask], label=label, linewidth=0.8)
            plotted = True
    if not plotted:
        plt.close()
        return
    plt.xlabel('X from test start [m]')
    plt.ylabel('Y from test start [m]')
    plt.title('Local odometry trajectory')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    save_figure(path)
    plt.close()


def write_metric_csv(path, metrics):
    with open(path, 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['metric', 'value'])
        for key, value in metrics.items():
            writer.writerow([key, value])


def safe_ratio(a, b):
    if not math.isfinite(a) or not math.isfinite(b) or abs(b) < 1e-9:
        return float('nan')
    return a / b


def add_error_metrics(metrics, prefix, mask, wheel_pos_error, wheel_yaw_error,
                      ekf_pos_error, ekf_yaw_error, imu_yaw_error):
    metrics[f'{prefix}_samples'] = int(np.count_nonzero(mask))
    metrics[f'{prefix}_wheel_position_rmse_m'] = rmse(wheel_pos_error[mask])
    metrics[f'{prefix}_wheel_position_max_m'] = maximum(wheel_pos_error[mask])
    metrics[f'{prefix}_wheel_yaw_rmse_deg'] = math.degrees(rmse(wheel_yaw_error[mask]))
    metrics[f'{prefix}_wheel_yaw_max_deg'] = math.degrees(maximum(wheel_yaw_error[mask]))
    metrics[f'{prefix}_ekf_position_rmse_m'] = rmse(ekf_pos_error[mask])
    metrics[f'{prefix}_ekf_position_max_m'] = maximum(ekf_pos_error[mask])
    metrics[f'{prefix}_ekf_yaw_rmse_deg'] = math.degrees(rmse(ekf_yaw_error[mask]))
    metrics[f'{prefix}_ekf_yaw_max_deg'] = math.degrees(maximum(ekf_yaw_error[mask]))
    metrics[f'{prefix}_imu_yaw_rmse_deg'] = math.degrees(rmse(imu_yaw_error[mask]))


def main():
    parser = argparse.ArgumentParser(
        description='Analyze one SC Robot local-odometry evaluation run.'
    )
    parser.add_argument('run_directory')
    args = parser.parse_args()

    run_dir = Path(args.run_directory).expanduser().resolve()
    samples_path = run_dir / 'local_odom_samples.csv'
    if not samples_path.exists():
        raise SystemExit(f'Missing {samples_path}')

    rows = load_csv(samples_path)
    if not rows:
        raise SystemExit('No local odometry samples found.')

    t = col(rows, 'ros_time')
    finite_t = finite(t)
    if finite_t.size:
        t = t - finite_t[0]
    state = text_col(rows, 'test_state')

    gt = relative_pose(col(rows, 'gt_x'), col(rows, 'gt_y'), col(rows, 'gt_yaw'))
    wheel = relative_pose(col(rows, 'wheel_x'), col(rows, 'wheel_y'), col(rows, 'wheel_yaw'))
    ekf = relative_pose(col(rows, 'ekf_x'), col(rows, 'ekf_y'), col(rows, 'ekf_yaw'))

    imu_yaw = col(rows, 'imu_yaw')
    imu_yaw_rel = relative_pose(
        np.zeros_like(imu_yaw), np.zeros_like(imu_yaw), imu_yaw
    )[2]

    wheel_pos_error, wheel_yaw_error = pose_error(gt, wheel)
    ekf_pos_error, ekf_yaw_error = pose_error(gt, ekf)
    imu_yaw_error = np.abs(wrap_angle_array(imu_yaw_rel - gt[2]))

    raw_ekf_x = col(rows, 'ekf_x')
    raw_ekf_y = col(rows, 'ekf_y')
    raw_ekf_yaw = col(rows, 'ekf_yaw')
    raw_tf_x = col(rows, 'tf_x')
    raw_tf_y = col(rows, 'tf_y')
    raw_tf_yaw = col(rows, 'tf_yaw')
    tf_pos_disagreement = np.hypot(raw_tf_x - raw_ekf_x, raw_tf_y - raw_ekf_y)
    tf_yaw_disagreement = np.abs(wrap_angle_array(raw_tf_yaw - raw_ekf_yaw))

    gt_vx, gt_wz = gt_velocity(t, gt[0], gt[1], gt[2])
    wheel_vx = col(rows, 'wheel_vx')
    wheel_wz = col(rows, 'wheel_wz')
    ekf_vx = col(rows, 'ekf_vx')
    ekf_wz = col(rows, 'ekf_wz')
    requested_vx = col(rows, 'requested_vx')
    requested_wz = col(rows, 'requested_wz')
    final_vx = col(rows, 'final_vx')
    final_wz = col(rows, 'final_wz')
    imu_raw_wz = col(rows, 'imu_raw_wz')
    imu_wz = col(rows, 'imu_wz')

    active_mask = np.isin(state, list(ACTIVE_STATES))
    static_mask = state == 'STATIC'
    done_mask = state == 'DONE'
    full_mask = np.ones(len(rows), dtype=bool)

    gt_length = path_length(gt[0], gt[1])
    wheel_length = path_length(wheel[0], wheel[1])
    ekf_length = path_length(ekf[0], ekf[1])

    metrics = {
        'samples_total': len(rows),
        'duration_s': maximum(t),
        'active_samples': int(np.count_nonzero(active_mask)),
        'static_samples': int(np.count_nonzero(static_mask)),
        'done_samples': int(np.count_nonzero(done_mask)),
        'ground_truth_path_length_m': gt_length,
        'wheel_path_length_m': wheel_length,
        'ekf_path_length_m': ekf_length,
        'wheel_path_length_ratio': safe_ratio(wheel_length, gt_length),
        'ekf_path_length_ratio': safe_ratio(ekf_length, gt_length),
    }

    # Headline accuracy: only samples while the robot is intentionally moving.
    add_error_metrics(
        metrics, 'active', active_mask,
        wheel_pos_error, wheel_yaw_error,
        ekf_pos_error, ekf_yaw_error, imu_yaw_error,
    )

    # Keep full-run numbers for traceability, but do not use them as headline
    # motion-accuracy results because long stationary periods can bias RMSE.
    add_error_metrics(
        metrics, 'full_run', full_mask,
        wheel_pos_error, wheel_yaw_error,
        ekf_pos_error, ekf_yaw_error, imu_yaw_error,
    )

    # Static section is evaluated as stability / drift, not motion accuracy.
    if np.any(static_mask):
        add_error_metrics(
            metrics, 'static', static_mask,
            wheel_pos_error, wheel_yaw_error,
            ekf_pos_error, ekf_yaw_error, imu_yaw_error,
        )

        for label, pose in [('gt', gt), ('wheel', wheel), ('ekf', ekf)]:
            drift_m, drift_yaw = phase_delta(pose[0], pose[1], pose[2], static_mask)
            metrics[f'static_{label}_drift_m'] = drift_m
            metrics[f'static_{label}_yaw_drift_deg'] = math.degrees(drift_yaw)

        metrics['static_imu_raw_wz_mean_rps'] = mean(imu_raw_wz[static_mask])
        metrics['static_imu_raw_wz_std_rps'] = std(imu_raw_wz[static_mask])
        metrics['static_imu_yaw_std_deg'] = math.degrees(std(imu_yaw_rel[static_mask]))

    # DONE is kept only as final stationary stability information.
    if np.any(done_mask):
        add_error_metrics(
            metrics, 'done', done_mask,
            wheel_pos_error, wheel_yaw_error,
            ekf_pos_error, ekf_yaw_error, imu_yaw_error,
        )
        for label, pose in [('gt', gt), ('wheel', wheel), ('ekf', ekf)]:
            drift_m, drift_yaw = phase_delta(pose[0], pose[1], pose[2], done_mask)
            metrics[f'done_{label}_drift_m'] = drift_m
            metrics[f'done_{label}_yaw_drift_deg'] = math.degrees(drift_yaw)

    metrics.update({
        'tf_vs_ekf_position_rmse_m': rmse(tf_pos_disagreement),
        'tf_vs_ekf_yaw_rmse_deg': math.degrees(rmse(tf_yaw_disagreement)),
        'active_wheel_vx_rmse_vs_gt_mps': rmse((wheel_vx - gt_vx)[active_mask]),
        'active_ekf_vx_rmse_vs_gt_mps': rmse((ekf_vx - gt_vx)[active_mask]),
        'active_wheel_wz_rmse_vs_gt_rps': rmse((wheel_wz - gt_wz)[active_mask]),
        'active_ekf_wz_rmse_vs_gt_rps': rmse((ekf_wz - gt_wz)[active_mask]),
        'active_imu_raw_wz_rmse_vs_gt_rps': rmse((imu_raw_wz - gt_wz)[active_mask]),
        'active_imu_filtered_wz_rmse_vs_gt_rps': rmse((imu_wz - gt_wz)[active_mask]),
    })

    write_metric_csv(run_dir / 'local_odom_summary.csv', metrics)

    phase_rows = []
    excluded = {'UNKNOWN', 'START_DELAY', 'FINAL_SETTLE', 'DONE'}
    phases = []
    for phase in state:
        if phase not in phases and phase not in excluded and phase:
            phases.append(phase)

    for phase in phases:
        mask = state == phase
        gt_phase_length = path_length(gt[0], gt[1], mask)
        wheel_phase_length = path_length(wheel[0], wheel[1], mask)
        ekf_phase_length = path_length(ekf[0], ekf[1], mask)
        phase_rows.append({
            'phase': phase,
            'samples': int(np.count_nonzero(mask)),
            'gt_path_length_m': gt_phase_length,
            'wheel_path_length_m': wheel_phase_length,
            'ekf_path_length_m': ekf_phase_length,
            'wheel_path_length_ratio': safe_ratio(wheel_phase_length, gt_phase_length),
            'ekf_path_length_ratio': safe_ratio(ekf_phase_length, gt_phase_length),
            'wheel_position_rmse_m': rmse(wheel_pos_error[mask]),
            'wheel_yaw_rmse_deg': math.degrees(rmse(wheel_yaw_error[mask])),
            'ekf_position_rmse_m': rmse(ekf_pos_error[mask]),
            'ekf_yaw_rmse_deg': math.degrees(rmse(ekf_yaw_error[mask])),
            'imu_yaw_rmse_deg': math.degrees(rmse(imu_yaw_error[mask])),
        })

    if phase_rows:
        fields = list(phase_rows[0].keys())
        with open(run_dir / 'local_odom_phase_summary.csv', 'w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(phase_rows)

    plot_trajectory(
        run_dir / 'local_odom_trajectory.svg',
        [('Ground truth', gt[0], gt[1]), ('Wheel odom', wheel[0], wheel[1]), ('EKF', ekf[0], ekf[1])],
    )
    plot_lines(
        run_dir / 'local_odom_position_error.svg',
        'Local odometry position error', 'Time [s]', 'Position error [m]', t,
        [('Wheel odom', wheel_pos_error), ('EKF', ekf_pos_error)],
    )
    plot_lines(
        run_dir / 'local_odom_yaw_error.svg',
        'Local odometry yaw error', 'Time [s]', 'Yaw error [deg]', t,
        [('Wheel odom', np.degrees(wheel_yaw_error)), ('EKF', np.degrees(ekf_yaw_error)), ('IMU orientation', np.degrees(imu_yaw_error))],
    )
    plot_lines(
        run_dir / 'local_odom_yaw_sources.svg',
        'Relative yaw estimates', 'Time [s]', 'Yaw [deg]', t,
        [('Ground truth', np.degrees(gt[2])), ('Wheel odom', np.degrees(wheel[2])), ('EKF', np.degrees(ekf[2])), ('IMU', np.degrees(imu_yaw_rel))],
    )
    plot_lines(
        run_dir / 'local_odom_linear_velocity.svg',
        'Forward velocity', 'Time [s]', 'Linear velocity [m/s]', t,
        [('Requested', requested_vx), ('Final command', final_vx), ('Ground truth', gt_vx), ('Wheel odom', wheel_vx), ('EKF', ekf_vx)],
    )
    plot_lines(
        run_dir / 'local_odom_angular_velocity.svg',
        'Yaw rate', 'Time [s]', 'Angular velocity [rad/s]', t,
        [('Requested', requested_wz), ('Final command', final_wz), ('Ground truth', gt_wz), ('Wheel odom', wheel_wz), ('EKF', ekf_wz), ('IMU raw', imu_raw_wz), ('IMU filtered', imu_wz)],
    )
    plot_lines(
        run_dir / 'local_odom_tf_consistency.svg',
        'EKF message vs odom-to-base TF consistency', 'Time [s]', 'Difference', t,
        [('Position difference [m]', tf_pos_disagreement), ('Yaw difference [rad]', tf_yaw_disagreement)],
    )

    left_vel = col(rows, 'left_wheel_vel')
    right_vel = col(rows, 'right_wheel_vel')
    plot_lines(
        run_dir / 'local_odom_wheel_velocity.svg',
        'Wheel joint velocity', 'Time [s]', 'Wheel angular velocity [rad/s]', t,
        [('Left wheel', left_vel), ('Right wheel', right_vel)],
    )

    print(f'Analysis complete: {run_dir}')
    print('Active-motion RMSE:')
    print(f"  Wheel position: {metrics['active_wheel_position_rmse_m']:.4f} m")
    print(f"  EKF position:   {metrics['active_ekf_position_rmse_m']:.4f} m")
    print(f"  Wheel yaw:      {metrics['active_wheel_yaw_rmse_deg']:.3f} deg")
    print(f"  EKF yaw:        {metrics['active_ekf_yaw_rmse_deg']:.3f} deg")
    print(f"  IMU yaw:        {metrics['active_imu_yaw_rmse_deg']:.3f} deg")
    print(f"Static samples: {metrics['static_samples']}, DONE samples: {metrics['done_samples']}")


if __name__ == '__main__':
    main()
