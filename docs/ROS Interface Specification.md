# SC Robot Interface Specification

Updated for the current ROS 2 Jazzy / Gazebo Harmonic implementation.

## TF

Primary transform chain:

```text
map -> odom -> base_footprint -> base_link
                         |
                         +--> camera_color_optical_frame
                         +--> camera_depth_optical_frame
                         +--> camera_imu_optical_frame
```

`map -> odom` is produced by global localization. `odom -> base_footprint`
is the local odometry chain.

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
Type: `scrobot_interfaces/action/ApproachTag`.

### `/relocalize`
Type: `scrobot_interfaces/action/Relocalize`.

AprilTag localization owns global correction of `map -> odom`.

## Camera

### `/camera/camera/color/image_raw`
Rectified RGB stream used directly by AprilTag and YOLO.

### `/camera/camera/color/camera_info`
Type: `sensor_msgs/msg/CameraInfo`.

Used by AprilTag and by the YOLO aligned-depth deprojection path.

Validated simulation CameraInfo:

```text
color 1280x720
fx = 906.94
fy = 906.94
cx = 640.00
cy = 360.00
```

### `/camera/camera/depth/camera_info`
Validated simulation CameraInfo:

```text
depth 848x480
fx = 420.29
fy = 420.29
cx = 424.00
cy = 240.00
```

### `/camera/camera/aligned_depth_to_color/image_raw`
Depth registered into the RGB/color image geometry.

### `/camera/camera/depth/points`
Type: `sensor_msgs/msg/PointCloud2`.

Input to `pointcloud_to_laserscan`; the full point cloud is not used
downstream after scan conversion.

### `/camera/camera/depth/scan`
Type: `sensor_msgs/msg/LaserScan`.

Filtered obstacle scan used by collision monitoring and Nav2 costmaps.

Current scan geometry:

```text
height band = 0.12-0.70 m
HFOV        = +/-0.7897925312 rad
range       = 0.30-3.30 m
```

## Shuttle perception

### `/perception/shuttle_detections_2d`
Type: `vision_msgs/msg/Detection2DArray`.

Frame: `camera_color_optical_frame`.

Publisher: `yolo_shuttle_detector`.

Contains YOLO shuttle bounding boxes and confidence scores.

### `/perception/shuttle_detections_3d`
Type: `vision_msgs/msg/Detection3DArray`.

Frame: `camera_color_optical_frame`.

Production publisher: `yolo_shuttle_detector`.

The detector combines the YOLO bbox with RGB-aligned depth and deprojects the
measurement with the color CameraInfo intrinsic matrix.

Simulation/testing may alternatively publish this same interface using
`fake_shuttle_detector`, but the fake detector is not part of the production
vision path.

QoS: sensor-data / Best Effort.

The current production mission consumes these camera-relative detections
directly through `shuttle_collection_filter`; persistent IDs are not required.

### `/perception/collectable_shuttle_detections_3d`
Type: `vision_msgs/msg/Detection3DArray`.

Publisher: `shuttle_collection_filter`.

Contains detections that pass mission-specific range and net-pole exclusion
checks. The local collection controller consumes this topic.

### Optional tracker topics

`shuttle_tracker` remains available only if a future persistent court-wide
shuttle map is desired.

```text
/perception/tracked_shuttles
/perception/visible_tracked_shuttles
```

These topics are not required by the current production mission.

## AprilTag perception

### `/apriltag/detections`
Type: `apriltag_msgs/msg/AprilTagDetectionArray`.

Publisher: `apriltag_ros`.

The localization package performs quality gating and global pose correction.

## Simulation-only ground truth

### `/evaluation/ground_truth_odom`
Type: `nav_msgs/msg/Odometry`.

Robot truth for simulation evaluation.

### `/evaluation/shuttle_ground_truth`
Type: `geometry_msgs/msg/PoseArray`.

Shuttle-only Gazebo truth. It may drive the simulation-only fake detector, but
must not feed production mission logic directly.

### `/evaluation/shuttle_collected`
Type: `geometry_msgs/msg/PoseArray`.

One-shot simulated shuttle collection events.

## Timing and QoS

All simulation nodes use `use_sim_time: true` and Gazebo `/clock`.
High-rate image, depth, scan, and raw detection streams use sensor-data QoS /
Best Effort unless a specific consumer requires otherwise.
