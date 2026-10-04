# scrobot_localization

Localization for the SC Robot badminton shuttle collector.

This package owns local state estimation from wheel odometry and the RealSense
D435i IMU, plus AprilTag-based correction of the robot pose in the badminton
court global frame.

## Frame ownership

`map` is the badminton-court global coordinate frame used by Nav2.

```text
map                         tag_global_localizer
 |
 v
odom                        ekf_filter_node
 |
 v
base_footprint
 |
 v
base_link                   robot_state_publisher / URDF
 |
 +-- camera frames
 +-- collector
 +-- wheels
```

Production TF ownership is intentionally single-source:

- `tag_global_localizer` owns `map -> odom`.
- `ekf_filter_node` owns `odom -> base_footprint`.
- `diff_drive_controller` uses `enable_odom_tf: false` in production.
- URDF / robot_state_publisher owns the fixed robot transforms.

AprilTag relocalization does not reset wheel odometry or the EKF. It updates
`map -> odom` so local odometry remains continuous.

## Local estimator

```text
/camera/camera/imu
        |
        v
imu_transformer
camera_imu_optical_frame -> base_link
        |
        v
/imu/data_raw
        |
        v
imu_filter_madgwick
        |
        v
/imu/data -------------------+
                             |
/diff_drive_controller/odom -+--> ekf_filter_node
                                    |
                                    +--> /odometry/filtered
                                    +--> odom -> base_footprint
```

The EKF uses:

- wheel odometry for X/Y translation and the non-holonomic `vy = 0`
  constraint;
- D435i IMU yaw and yaw rate for heading;
- `imu0_relative: true` because the D435i has no absolute heading reference by
  default.

### Optional magnetometer

The final robot does not require a magnetometer, but the path remains available
for testing.

Default:

```bash
ros2 launch scrobot_localization localization.launch.py use_magnetometer:=false
```

Optional:

```bash
ros2 launch scrobot_localization localization.launch.py use_magnetometer:=true
```

When enabled, `/imu/mag` is fused by Madgwick. The default remains D435i-only.

## AprilTag localization

Four `tag16h5` tags are mounted near the net posts. Simulation and
localization use the same `court_landmarks.yaml` geometry.

Current fixed tag geometry:

- tag edge: 0.100 m
- tag height: 0.150 m
- mount radius: 0.075 m
- inward angle: 45 deg
- pole Y positions: +/-3.05 m

The global localizer performs expensive pose processing only during an explicit
`/relocalize` action.

```text
tag visible
    |
    v
robot stationary
    |
    v
collect 15 samples
    |
    v
reject bad / inconsistent samples
    |
    v
check batch position and yaw spread
    |
    v
commit map -> odom
```

Important checks include:

- decision margin;
- detection distance;
- view angle;
- cross-tag position disagreement;
- cross-tag yaw disagreement;
- stationary threshold;
- batch outlier rejection;
- batch position/yaw standard deviation.

## AprilTag search and approach

The current tag-specific approach controller is retained as working technical
debt. The long-term design is to reuse the common pose controller for both
shuttle and AprilTag stand-off poses.

Current flow:

```text
search
  |
  +-- rotate until tag found
  |
  +-- no tag after measured 2*pi yaw -> search failure

tag found
  |
  v
stop and observe multiple frames
  |
  v
lock best-facing tag
  |
  v
approach stand-off pose
  |
  v
final heading alignment
```

Search completion is based on accumulated measured odometry yaw, not elapsed
time. The configured search target is exactly `2*pi`.

Current requested stand-off:

- `default_target_distance: 0.80 m`
- mission `tag_approach_distance: 0.80 m`

The overall action timeout remains independent of the 2*pi search completion so
a tag found late in the rotation still has time for approach and alignment.

## Recovery after a failed search

The mission runtime keeps the last successful localization pose.

```text
full 2*pi search -> no usable tag
            |
            +-- no last-good pose -> retry in place
            |
            +-- last-good pose exists
                    |
                    v
                 Nav2 return
                    |
                    v
             retry full tag search
```

The newer AUTO/MANUAL pause/resume behavior is preserved through this recovery
state.

## Current range parameters

Current configuration:

- `default_target_distance: 0.80 m`
- `relocalization_max_distance: 10.5 m`
- `max_tag_distance: 9.0 m`
- `min_decision_margin: 20.0`

Note that `max_tag_distance` is still a separate 9.0 m visibility gate. A tag
farther than 9.0 m is rejected before the 10.5 m relocalization gate is
evaluated. This is intentionally left unchanged until the recognition-range
test is completed.

## Debug / systematic test

Build:

```bash
cd ~/scrobot_ws
colcon build --symlink-install --packages-up-to scrobot_debug
source install/setup.bash
```

Run:

```bash
ros2 launch scrobot_debug localization_check.launch.py
```

RViz is off by default:

```bash
ros2 launch scrobot_debug localization_check.launch.py launch_rviz:=true
```

Optional magnetometer:

```bash
ros2 launch scrobot_debug localization_check.launch.py use_magnetometer:=true
```

The debug monitor reports:

- `odom -> base_footprint` availability;
- `map -> odom` availability;
- color-camera intrinsics;
- AprilTag ID;
- decision margin;
- camera-to-tag range;
- maximum observed range;
- tag-center pixel offset from `(cx, cy)`;
- horizontal/vertical angular offset from the camera optical axis.

This test is used to determine the real stable recognition range and the
preferred centered relocalization stand-off.

### Test a complete 2*pi search

```bash
ros2 action send_goal /approach_tag scrobot_interfaces/action/ApproachTag \
  "{preferred_tag_id: -1, target_distance: 0.80, timeout_sec: 60.0}" --feedback
```

### Test the 15-sample global correction

```bash
ros2 action send_goal /relocalize scrobot_interfaces/action/Relocalize \
  "{preferred_tag_id: -1, sample_count: 15, timeout_sec: 15.0}" --feedback
```

## Important files

- `config/ekf.yaml` - local EKF configuration.
- `config/imu_filter.yaml` - Madgwick configuration.
- `config/court_landmarks.yaml` - court/tag geometry and localization tuning.
- `launch/localization.launch.py` - D435i IMU + EKF.
- `launch/global_localization.launch.py` - tag approach + global localizer.
- `scripts/tag_global_localizer.py` - 15-sample map-to-odom correction.
- `scripts/tag_approach_controller.py` - current tag search/stand-off controller.

## Known debt

- Replace the dedicated tag approach controller with the shared pose-controller
  abstraction when the subsystem interfaces are stable.
- Measure real stable AprilTag recognition range and then tune
  `max_tag_distance` and `relocalization_max_distance` consistently.
