# SC Robot Interface Specification

Updated for the current ROS 2 Jazzy / Gazebo Harmonic branch.

## TF

Primary transform chain:

```text
map -> odom -> base_footprint -> base_link
                         |
                         +--> camera_color_optical_frame
                         +--> camera_depth_optical_frame
                         +--> camera_imu_optical_frame
```

`map -> odom` is produced by the global localization stack. `odom -> base_footprint` is the local odometry chain.

## Control

### `/cmd_vel_manual`
Manual velocity command source used by keyboard teleoperation and tests.

### `/diff_drive_controller/cmd_vel`
Final velocity command consumed by the differential-drive controller.

### `/diff_drive_controller/odom`
Wheel-odometry output.

### `/joint_states`
Wheel and joint feedback.

## Localization

### `/imu/data_raw`
Raw robot IMU input.

### `/imu/data`
Filtered/orientation-aware IMU output where configured.

### `/odometry/filtered`
EKF local odometry output.

### `/approach_tag`
Type: `scrobot_interfaces/action/ApproachTag`

Goal:

```text
int32 preferred_tag_id
float32 target_distance
float32 timeout_sec
```

Purpose: search for and approach a suitable AprilTag before localization correction.

### `/relocalize`
Type: `scrobot_interfaces/action/Relocalize`

Goal:

```text
int32 preferred_tag_id
int32 sample_count
float32 timeout_sec
```

Purpose: estimate/correct `map -> odom` while stationary using AprilTag observations.

## Camera

### `/camera/camera/color/camera_info`
Type: `sensor_msgs/msg/CameraInfo`

Used by the fake detector for RGB field-of-view projection and by the future real RGB-D localization path.

### `/camera/camera/depth/points`
Type: `sensor_msgs/msg/PointCloud2`

Used for geometric obstacle handling such as collision monitoring / Nav2 costmaps.

## Shuttle perception

### `/perception/detections_2d`
Type: `vision_msgs/msg/Detection2DArray`

Frame: `camera_color_optical_frame`

Future real YOLO output. Sensor-data QoS.

### `/perception/shuttle_detections_3d`
Type: `vision_msgs/msg/Detection3DArray`

Frame: `camera_depth_optical_frame`

Publisher:

- simulation: `fake_shuttle_detector`
- real robot: future `depth_localizer`

Subscriber: `shuttle_tracker`

QoS: sensor-data / best effort.

Persistent identity is intentionally not assigned here.

### `/perception/tracked_shuttles`
Type: `vision_msgs/msg/Detection3DArray`

Frame: `map`

Publisher: `shuttle_tracker`

Subscribers: mission logic, final approach, visualization/debug tools.

QoS: Reliable.

Each `Detection3D.id` is a stable tracker-owned integer string while the track remains alive.

Current tracker defaults:

```text
publish_rate             10 Hz
association_distance     0.30 m
position_alpha           0.50
stale_timeout            1.50 s
tf_timeout               0.05 s
fallback_to_latest_tf    true
```

## Simulation-only ground truth

### `/evaluation/ground_truth_odom`
Type: `nav_msgs/msg/Odometry`

Purpose: robot pose reference for simulation evaluation and fake-perception geometry.

This remains the primary truth source used by `scrobot_evaluation` local-odometry tests.

### `/evaluation/ground_truth_tf`
Type: `tf2_msgs/msg/TFMessage`

Source: Gazebo dynamic pose bridge.

Purpose: evaluation/debug only. Gazebo entity names may be lost by the `Pose_V -> TFMessage` bridge and therefore this topic is not used to identify shuttles.

### `/evaluation/shuttle_ground_truth_gz`
Gazebo type: `gz.msgs.Pose_V`

Publisher: `shuttle_activity_system`.

Contains only shuttle model world poses.

### `/evaluation/shuttle_ground_truth`
ROS type: `geometry_msgs/msg/PoseArray`

Bridge of `/evaluation/shuttle_ground_truth_gz`.

Subscriber: simulation-only `fake_shuttle_detector`.

This topic does not replace `/evaluation/ground_truth_odom` or alter `scrobot_evaluation`.

## Timing rule

All simulation nodes should use `use_sim_time: true` and consume `/clock`. Fake shuttle detections are stamped using the latest Gazebo ground-truth odometry stamp so the timestamp is in the same clock domain as TF. `shuttle_tracker` prefers exact measurement-time TF and can fall back to the latest TF during startup/timing skew.

## QoS rule

High-rate sensors use sensor-data QoS / Best Effort unless a specific consumer requires otherwise. Persistent mission-level products such as `/perception/tracked_shuttles` use Reliable QoS.
