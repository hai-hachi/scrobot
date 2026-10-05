# SC Robot Interface Specification

Authoritative public ROS interfaces for the current ROS 2 Jazzy stack.

## Frames

```text
map -> odom -> base_footprint -> base_link
                         |
                         +--> collector_link
                         +--> camera_color_optical_frame
                         +--> camera_depth_optical_frame
                         +--> camera_imu_optical_frame
```

Ownership:

- `map -> odom`: `tag_global_localizer`
- `odom -> base_footprint`: EKF
- fixed robot transforms: robot_state_publisher

## Control

| Interface | Type / role |
| --- | --- |
| `/cmd_vel_nav` | stamped Nav2 command source |
| `/cmd_vel_approach` | stamped local collection command source |
| `/cmd_vel_relocalization` | stamped tag-approach command source |
| `/cmd_vel_manual_input` | operator input before manual-mode gating |
| `/cmd_vel_manual` | manual source owned by `manual_mode_manager` |
| `/cmd_vel_auto` | autonomous mux output |
| `/cmd_vel_selected` | AUTO/MANUAL mux output |
| `/cmd_vel_smoothed` | velocity-smoother output |
| `/diff_drive_controller/cmd_vel` | final collision-monitored command |
| `/control/set_manual_mode` | `std_srvs/srv/SetBool` |
| `/control/manual_mode` | latched `std_msgs/msg/Bool` |
| `/control/mode` | latched `std_msgs/msg/String` |

Odometry and feedback:

```text
/diff_drive_controller/odom
/joint_states
/odometry/filtered
```

## Localization actions

### /approach_tag

Type: `scrobot_interfaces/action/ApproachTag`

Find a usable tag and reach the requested tag-facing stand-off pose.

### /relocalize

Type: `scrobot_interfaces/action/Relocalize`

Collect a stationary AprilTag sample batch and update `map -> odom`.

## Mission action

### /local_collect

Type: `scrobot_interfaces/action/LocalCollect`

Collect currently eligible visible shuttles until no new eligible target
remains, cancellation occurs, or timeout is reached.

Mission status:

```text
/mission/state
/mission/local_collect_phase
/mission/sweep_path
/mission/relocalization_stops
/mission/current_goal
```

## Camera

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/depth/image_raw
/camera/camera/depth/camera_info
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/depth/points
/camera/camera/depth/scan_raw
/camera/camera/depth/scan
/camera/camera/imu
```

Validated simulated CameraInfo:

```text
color 1280x720  fx=906.94 fy=906.94 cx=640 cy=360
depth  848x480  fx=420.29 fy=420.29 cx=424 cy=240
```

## Shuttle perception

### /perception/shuttle_detections_2d

Type: `vision_msgs/msg/Detection2DArray`

YOLO image detections.

### /perception/shuttle_detections_3d

Type: `vision_msgs/msg/Detection3DArray`

Production YOLO + aligned-depth result in
`camera_color_optical_frame`. Camera-depth validity is 0.20-3.00 m.

### /perception/collectable_shuttle_detections_3d

Type: `vision_msgs/msg/Detection3DArray`

Mission-filtered detections. Eligibility is evaluated in `base_link` using
0.50-1.80 m planar range plus net-pole exclusion.

### /perception/shuttle_debug/image

Optional annotated RGB debug image.

## AprilTag perception

```text
/apriltag/detections
```

Type: `apriltag_msgs/msg/AprilTagDetectionArray`.

AprilTag quality gating and global correction belong to
`scrobot_localization`.

## Simulation-only truth

```text
/evaluation/ground_truth_odom       nav_msgs/msg/Odometry
/evaluation/shuttle_ground_truth    geometry_msgs/msg/PoseArray
/evaluation/shuttle_collected       geometry_msgs/msg/PoseArray
```

These interfaces are forbidden as production mission/navigation inputs.

## QoS and time

Simulation uses Gazebo `/clock` and `use_sim_time: true`.

High-rate camera, IMU, scan, and raw detection topics generally use sensor-data
Best Effort QoS. Mission state and selected debug geometry use explicit
reliable/transient-local QoS where late-joiner behavior is required.
