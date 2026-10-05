# scrobot_localization

Local wheel/IMU state estimation and AprilTag global localization for SC Robot.

## TF ownership

```text
map                 tag_global_localizer
 |
 v
odom                ekf_filter_node
 |
 v
base_footprint
 |
 v
base_link           robot_state_publisher / URDF
```

Production ownership is single-source:

- `tag_global_localizer` owns `map -> odom`;
- the EKF owns `odom -> base_footprint`;
- `diff_drive_controller` keeps `enable_odom_tf: false`;
- URDF owns fixed robot transforms.

AprilTag correction updates `map -> odom`; it does not reset wheel odometry or
the EKF.

## Local estimator

```text
/camera/camera/imu
  -> imu_transformer
  -> /imu/data_raw
  -> imu_filter_madgwick
  -> /imu/data -----------+
                          |
/diff_drive_controller/odom
                          |
                          v
                    ekf_filter_node
                          |
                          +--> /odometry/filtered
                          +--> odom -> base_footprint
```

The default system uses the D435i IMU without a magnetometer. The optional
`/imu/mag` path remains available for controlled tests.

## AprilTag localization

Court tags:

```text
family             tag16h5
tag edge           0.100 m
tag height         0.150 m
mount radius       0.075 m
mount angle        45 deg inward
eligibility range  <= 10.0 m
decision margin    >= 20 for global correction
```

`/relocalize` collects 15 stationary samples, rejects outliers/inconsistent
measurements, and commits the global correction.

## Tag approach

The production tag approach uses the proven `main_branch` strategy:

```text
search up to measured 2*pi yaw
  -> observe and lock best tag
  -> turn toward desired XY
  -> drive to desired XY
  -> rotate to final tag-facing yaw
```

The final desired pose places `base_link` 0.90 m from the tag and faces the
tag. Production speed limits are 0.25 m/s linear and 0.60 rad/s angular.

Alternative SMC/biarc/normal-ray strategies remain debug regressions only.

## Launches

Local EKF:

```bash
ros2 launch scrobot_localization localization.launch.py \
  use_magnetometer:=false
```

AprilTag action servers / global correction:

```bash
ros2 launch scrobot_localization global_localization.launch.py
```

## Files

- `config/ekf.yaml`
- `config/imu_filter.yaml`
- `config/court_landmarks.yaml`
- `scripts/tag_global_localizer.py`
- `scripts/tag_approach_controller.py`

Regression procedures:
`../scrobot_debug/debug_md/scrobot_localization/README.md`.
