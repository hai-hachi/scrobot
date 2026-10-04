# scrobot_localization systematic tests

This guide validates local EKF estimation, TF ownership, AprilTag recognition,
full-rotation search, stand-off control, and global map-to-odom correction.

## 1. Launch

```bash
ros2 launch scrobot_debug localization_check.launch.py
```

The normal test is headless. RViz is optional:

```bash
ros2 launch scrobot_debug localization_check.launch.py launch_rviz:=true
```

D435i-only localization is the default:

```bash
ros2 launch scrobot_debug localization_check.launch.py use_magnetometer:=false
```

Optional magnetometer path:

```bash
ros2 launch scrobot_debug localization_check.launch.py use_magnetometer:=true
```

## 2. Expected TF ownership

```text
map                  tag_global_localizer
 |
 v
odom                 ekf_filter_node
 |
 v
base_footprint
 |
 v
base_link            URDF / robot_state_publisher
```

Production `diff_drive_controller` must have:

```yaml
enable_odom_tf: false
```

Check:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom
```

Before the first successful global relocalization:

```text
odom -> base_footprint = expected
map -> odom            = may be unavailable
```

After successful relocalization:

```text
odom -> base_footprint = continuous
map -> odom            = available/corrected
```

## 3. IMU / EKF path

Check:

```bash
ros2 topic hz /camera/camera/imu
ros2 topic hz /imu/data_raw
ros2 topic hz /imu/data
ros2 topic hz /diff_drive_controller/odom
ros2 topic hz /odometry/filtered
```

Default path:

```text
/camera/camera/imu
  -> imu_transformer
  -> /imu/data_raw
  -> Madgwick
  -> /imu/data
  -> EKF

/diff_drive_controller/odom
  -> EKF
```

With the optional magnetometer enabled:

```bash
ros2 topic hz /imu/mag
ros2 param get /imu_filter_madgwick use_mag
```

Expected parameter:

```text
Boolean value is: True
```

## 4. AprilTag recognition-range / centering test

The `localization_monitor` reports:

```text
tag ID
decision margin
camera-to-tag range
maximum range observed this session
tag center (u, v)
pixel offset from calibrated (cx, cy)
horizontal/vertical optical-axis angle
TF health
```

Current tuning values:

```text
default_target_distance       = 0.80 m
relocalization_max_distance   = 10.5 m
max_tag_distance              = 9.0 m
min_decision_margin           = 20.0
```

Important: `max_tag_distance = 9.0 m` is currently a stricter upstream gate
than `relocalization_max_distance = 10.5 m`. Therefore the effective maximum
accepted range remains 9.0 m until the recognition test is complete and that
separate gate is deliberately retuned.

Drive toward/away from a tag and record where detection is stable rather than
using a single farthest lucky detection.

For the preferred observation pose, use the monitor's image-center offsets.
The desired tag pose is near the optical center:

```text
horizontal angle ~= 0 deg
vertical angle   ~= 0 deg
```

## 5. Full 2*pi search

```bash
ros2 action send_goal /approach_tag scrobot_interfaces/action/ApproachTag \
  "{preferred_tag_id: -1, target_distance: 0.80, timeout_sec: 60.0}" --feedback
```

With no acceptable tag, expected behavior is:

```text
search begins
  -> accumulated odometry yaw is measured
  -> robot rotates
  -> accumulated yaw >= 2*pi
  -> action reports explicit no-tag/full-rotation failure
```

It must not keep rotating until the 60 s overall action timeout.

If a tag is found:

```text
search
 -> stop/observe
 -> lock best-facing tag
 -> drive to 0.80 m stand-off
 -> final alignment
```

## 6. Fifteen-sample global correction

With a centered visible tag and the robot stopped:

```bash
ros2 action send_goal /relocalize scrobot_interfaces/action/Relocalize \
  "{preferred_tag_id: -1, sample_count: 15, timeout_sec: 15.0}" --feedback
```

Expected:

```text
stationary
 -> collect 15 samples
 -> reject outliers
 -> validate position/yaw spread
 -> commit map -> odom
```

Local odometry must not reset or jump as part of this operation.

## 7. Search recovery

Mission-level expected behavior:

```text
full 2*pi search, no usable tag
  |
  +-- no last-good localization pose
  |      -> retry in place
  |
  +-- last-good pose available
         -> Nav2 to last-good pose
         -> retry full tag search
```

The recovery state must coexist with MANUAL pause/resume.

## Pass criteria

- D435i-only path works with `use_magnetometer:=false`;
- optional magnetometer path can be enabled without changing architecture;
- EKF is the sole `odom -> base_footprint` owner in production;
- tag global localizer is the sole `map -> odom` owner;
- no-tag search stops at measured 2*pi;
- approach reaches the configured 0.80 m stand-off;
- 15-sample correction succeeds with valid centered observations;
- `odom -> base_footprint` remains continuous during global correction;
- recognition-range and image-center results are recorded before final range
  thresholds are frozen.
