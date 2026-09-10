# SCROBOT Debug Guide

## Launch

Simulation:

```bash
ros2 launch scrobot_simulation simulation.launch.py
```

Control stack:

```bash
ros2 launch scrobot_control control_stack.launch.py
```

Localization:

```bash
ros2 launch scrobot_localization localization.launch.py use_sim_time:=true
```

Perception:

```bash
ros2 launch scrobot_perception perception.launch.py
```

Optional scan-height override:

```bash
ros2 launch scrobot_perception perception.launch.py max_height:=0.70
```

## Velocity Topics

Priority:

```text
/cmd_vel_manual           highest
/cmd_vel_relocalization
/cmd_vel_approach
/cmd_vel_nav              lowest
```

Pipeline:

```text
/cmd_vel_manual
/cmd_vel_relocalization
/cmd_vel_approach
/cmd_vel_nav
       ↓
    twist_mux
       ↓
/cmd_vel_muxed
       ↓
velocity_smoother
       ↓
/cmd_vel_smoothed
       ↓
collision_monitor
       ↓
/diff_drive_controller/cmd_vel
```

Test manual command:

```bash
ros2 topic pub --rate 10 \
  /cmd_vel_manual \
  geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_footprint}, twist: {linear: {x: 0.0}, angular: {z: 0.5}}}"
```

Check command pipeline:

```bash
ros2 topic echo /cmd_vel_muxed
ros2 topic echo /cmd_vel_smoothed
ros2 topic echo /diff_drive_controller/cmd_vel
```

Check publisher, subscriber, and QoS:

```bash
ros2 topic info /cmd_vel_nav -v
ros2 topic info /cmd_vel_muxed -v
ros2 topic info /cmd_vel_smoothed -v
ros2 topic info /diff_drive_controller/cmd_vel -v
```

Check publish rate:

```bash
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel_muxed
ros2 topic hz /cmd_vel_smoothed
ros2 topic hz /diff_drive_controller/cmd_vel
```

## Perception

Check the point cloud and generated LaserScan:

```bash
ros2 topic hz /camera/camera/depth/points
ros2 topic hz /camera/camera/depth/scan
ros2 topic info /camera/camera/depth/points -v
ros2 topic info /camera/camera/depth/scan -v
```

Check the live scan-height filter:

```bash
ros2 param get /depth_pointcloud_to_scan min_height
ros2 param get /depth_pointcloud_to_scan max_height
```

Expected defaults:

```text
min_height = 0.08 m
max_height = 0.70 m
```

Check AprilTag detections:

```bash
ros2 topic hz /apriltag/detections
```

## RViz

Launch RViz through the simulation launch when desired:

```bash
ros2 launch scrobot_simulation simulation.launch.py rviz:=true
```

Useful displays:

- `PointCloud2` -> `/camera/camera/depth/points`
- `LaserScan` -> `/camera/camera/depth/scan`
- `TF`
- collision-monitor polygons
- robot model

Use `Best Effort` QoS for high-rate sensor displays such as the point cloud and LaserScan.

Typical fixed frame:

```text
odom
```

or, after global localization:

```text
map
```

## Useful rate checks

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_raw
ros2 topic hz /camera/camera/depth/points
ros2 topic hz /camera/camera/depth/scan
ros2 topic hz /apriltag/detections
ros2 topic hz /diff_drive_controller/odom
ros2 topic hz /imu/data
ros2 topic hz /odometry/filtered
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel_smoothed
```
