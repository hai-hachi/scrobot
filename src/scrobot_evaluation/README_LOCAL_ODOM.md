# SC Robot Local Odometry Evaluation

This overlay extends the existing `scrobot_evaluation` package with an isolated local-odometry test suite.

## What is evaluated

- `/diff_drive_controller/odom` wheel odometry
- `/imu/data_raw` transformed raw IMU
- `/imu/data` Madgwick-filtered IMU orientation
- `/odometry/filtered` EKF local odometry
- `odom -> base_footprint` TF consistency
- `/joint_states` left/right wheel feedback
- requested `/cmd_vel_manual` versus final `/diff_drive_controller/cmd_vel`
- Gazebo `/evaluation/ground_truth_tf` as an evaluation-only reference
- message rate, jitter, and header age for local-odometry topics

No AprilTag/map/global-localization data are used in the local-odom metrics.

## Automatic tests

`test_type` can be:

- `static`: stationary drift/noise
- `straight`: forward straight-line motion
- `rotate`: in-place CCW rotation
- `arc`: combined linear/angular motion
- `suite`: static + forward/reverse + CCW/CW rotation + CCW/CW arcs

The default suite uses:

- straight: 2.0 m at 0.25 m/s
- rotation: 360 deg at 0.50 rad/s
- arc: 180 deg at 0.25 m/s and 0.25 rad/s (1 m radius)

Commands are published to `/cmd_vel_manual`, so they pass through the normal SC Robot command pipeline.

## Build

```bash
cd ~/scrobot_ws
colcon build --symlink-install --packages-select scrobot_evaluation
source install/setup.bash
```

## Run

Start simulation + control + localization first. For this local test, mission/global localization/Nav2 are not required.

Then:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch scrobot_evaluation local_odom_eval.launch.py test_type:=suite run_name:=local_odom_suite_01
```

Individual tests:

```bash
ros2 launch scrobot_evaluation local_odom_eval.launch.py test_type:=static run_name:=static_01
ros2 launch scrobot_evaluation local_odom_eval.launch.py test_type:=straight run_name:=straight_01
ros2 launch scrobot_evaluation local_odom_eval.launch.py test_type:=rotate run_name:=rotate_01
ros2 launch scrobot_evaluation local_odom_eval.launch.py test_type:=arc run_name:=arc_01
```

Logger only, for manual driving:

```bash
ros2 launch scrobot_evaluation local_odom_eval.launch.py run_test:=false run_name:=manual_01
```

Stop the logger with Ctrl+C after the test runner reports `Local odom test complete.`

## Analyze

```bash
ros2 run scrobot_evaluation analyze_local_odom \
  ~/scrobot_evaluation_runs/local_odom/local_odom_suite_01
```

Outputs include:

- `local_odom_samples.csv`
- `timing_summary.csv`
- `local_odom_summary.csv`
- `local_odom_phase_summary.csv`
- `local_odom_trajectory.png`
- `local_odom_position_error.png`
- `local_odom_yaw_error.png`
- `local_odom_yaw_sources.png`
- `local_odom_linear_velocity.png`
- `local_odom_angular_velocity.png`
- `local_odom_tf_consistency.png`
- `local_odom_wheel_velocity.png`
