# scrobot_localization Regression

## EKF + AprilTag stack

```bash
ros2 launch scrobot_debug localization_check.launch.py \
  use_magnetometer:=false
```

Expected production TF owners:

```text
map -> odom             tag_global_localizer
odom -> base_footprint  EKF
fixed robot TF          robot_state_publisher
```

Check:

```bash
ros2 topic hz /imu/data_raw
ros2 topic hz /imu/data
ros2 topic hz /diff_drive_controller/odom
ros2 topic hz /odometry/filtered
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom
```

## Tag approach

Production strategy: `main_branch`.

```bash
ros2 action send_goal /approach_tag \
  scrobot_interfaces/action/ApproachTag \
  "{preferred_tag_id: -1, target_distance: 0.90, timeout_sec: 60.0}" \
  --feedback
```

Expected:

```text
measured 2*pi search when needed
 -> settle/lock best tag
 -> drive to 0.90 m base_link stand-off
 -> final tag-facing yaw
```

The completed strategy comparison ranked:

```text
1 main_branch
2 biarc_smc
3 pure_smc
4 normal_ray_smc
```

The alternative launches remain regression tools, not production defaults.

## Relocalization

```bash
ros2 action send_goal /relocalize \
  scrobot_interfaces/action/Relocalize \
  "{preferred_tag_id: -1, sample_count: 15, timeout_sec: 15.0}" \
  --feedback
```

Pass when `map -> odom` is corrected without resetting the continuous local
`odom -> base_footprint` estimate.
