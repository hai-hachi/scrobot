# SC Robot ROS Interface Specification

This file mirrors `docs/interface_specification.md` for the current ROS 2 Jazzy / Gazebo Harmonic implementation.

## Frames

```text
map -> odom -> base_footprint -> base_link
                         |
                         +--> camera_color_optical_frame
                         +--> camera_depth_optical_frame
                         +--> camera_imu_optical_frame
```

`map -> odom` is owned by global localization. `odom -> base_footprint` is owned by the local odometry chain.

## Main control interfaces

- `/cmd_vel_manual`: manual velocity source.
- `/diff_drive_controller/cmd_vel`: final drive command.
- `/diff_drive_controller/odom`: wheel odometry.
- `/joint_states`: joint feedback.

## Localization interfaces

- `/imu/data_raw`: raw IMU.
- `/imu/data`: filtered/orientation IMU where enabled.
- `/odometry/filtered`: EKF odometry.
- `/approach_tag`: `scrobot_interfaces/action/ApproachTag`.
- `/relocalize`: `scrobot_interfaces/action/Relocalize`.

`/relocalize` establishes or corrects `map -> odom` while the robot is stationary.

## Camera interfaces

- `/camera/camera/color/camera_info`: `sensor_msgs/msg/CameraInfo`.
- `/camera/camera/depth/points`: `sensor_msgs/msg/PointCloud2`.

High-rate camera/depth streams use sensor-data QoS / Best Effort.

## Shuttle perception interfaces

### `/perception/detections_2d`

Type: `vision_msgs/msg/Detection2DArray`  
Frame: `camera_color_optical_frame`

Future real YOLO output.

### `/perception/shuttle_detections_3d`

Type: `vision_msgs/msg/Detection3DArray`  
Frame: `camera_depth_optical_frame`  
QoS: Sensor Data / Best Effort

Simulation publisher: `fake_shuttle_detector`.  
Future real publisher: `depth_localizer`.

The detector does not own persistent IDs.

### `/perception/tracked_shuttles`

Type: `vision_msgs/msg/Detection3DArray`  
Frame: `map`  
Publisher: `shuttle_tracker`  
QoS: Reliable

The tracker transforms detections into `map`, performs nearest-neighbor association, smooths positions, assigns stable IDs, and removes stale tracks.

Current defaults:

```text
publish_rate          10 Hz
association_distance  0.30 m
position_alpha        0.50
stale_timeout         1.50 s
tf_timeout            0.05 s
fallback_to_latest_tf true
```

## Simulation-only truth

- `/evaluation/ground_truth_odom`: `nav_msgs/msg/Odometry`; primary robot truth for evaluation.
- `/evaluation/ground_truth_tf`: `tf2_msgs/msg/TFMessage`; generic Gazebo dynamic-pose bridge, evaluation/debug only.
- `/evaluation/shuttle_ground_truth_gz`: `gz.msgs.Pose_V`; shuttle-only Gazebo truth.
- `/evaluation/shuttle_ground_truth`: `geometry_msgs/msg/PoseArray`; shuttle-only ROS truth consumed only by fake perception.

The shuttle-only truth stream is additive and does not change the existing `scrobot_evaluation` odometry truth path.

## Timing

All simulation nodes should use `use_sim_time: true`. Synthetic shuttle detections use the latest `/evaluation/ground_truth_odom` stamp so detection timestamps are in the Gazebo `/clock` domain. The tracker first requests exact-time TF and can fall back to the latest transform during startup or small timing skew.
