# SC Robot ROS 2 Architecture

Updated for ROS 2 Jazzy + Gazebo Harmonic.

## System flow

```text
Gazebo / real hardware
        |
        +--> sensors / odometry
        |
        +--> localization ------------------------------+
        |                                               |
        |                                               v
        |                                          map -> odom
        |
        +--> perception
        |      |
        |      +--> filtered depth scan --> collision monitor + Nav2
        |      |
        |      +--> AprilTag detections --> global localization
        |      |
        |      +--> YOLO + aligned depth
        |              |
        |              v
        |      /perception/shuttle_detections_3d
        |              |
        |              v
        |      shuttle_collection_filter
        |              |
        +--> mission / local collection
                       |
                       v
                  command pipeline
                       |
                       v
              diff_drive_controller
```

## Frames

```text
map -> odom -> base_footprint -> base_link
                         |
                         +--> wheels / casters / collector
                         +--> camera_link
                               +--> camera_color_optical_frame
                               +--> camera_depth_optical_frame
                               +--> camera_imu_optical_frame
```

`map -> odom` is owned by global localization. `odom -> base_footprint`
comes from the local odometry / EKF chain.

## Production shuttle perception

```text
rectified RGB
 -> YOLO
 -> /perception/shuttle_detections_2d
                +
RGB-aligned depth + color CameraInfo
                |
                v
/perception/shuttle_detections_3d
                |
                v
shuttle_collection_filter
                |
                v
/perception/collectable_shuttle_detections_3d
                |
                v
local_collect_controller
```

The 3D detections are camera-relative in `camera_color_optical_frame`.
Mission-specific range and net-pole filtering happen after perception.

The legacy `shuttle_tracker` remains optional and is not on the current
production mission path.

## Obstacle perception

```text
D435i depth PointCloud2
 -> pointcloud_to_laserscan
 -> /camera/camera/depth/scan_raw
 -> depth_scan_self_filter
 -> /camera/camera/depth/scan
 -> collision_monitor + Nav2 costmaps
```

## AprilTag perception

```text
rectified RGB + color CameraInfo
 -> apriltag_ros
 -> /apriltag/detections
 -> scrobot_localization
 -> map -> odom correction
```

## Simulation-only truth

Simulation exposes:

```text
/evaluation/ground_truth_odom
/evaluation/shuttle_ground_truth
/evaluation/shuttle_collected
```

These are evaluation/test interfaces. Production mission logic must not consume
Gazebo truth directly.

## Navigation and control

Nav2 plans and tracks paths using Regulated Pure Pursuit. Commands pass through
AUTO/MANUAL arbitration, velocity smoothing, collision monitoring, and then the
differential-drive controller.
