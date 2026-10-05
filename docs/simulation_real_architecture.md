# Simulation vs Real Robot Architecture

The downstream ROS interfaces remain the same between simulation and the real
robot. Only the sensor source changes.

## Simulation

```text
Gazebo D435i-like RGB
        |
        v
yolo_shuttle_detector
        |
        +--> /perception/shuttle_detections_2d
        |
Gazebo depth -> depth-to-color registration
        |
        v
/perception/shuttle_detections_3d
        |
        v
shuttle_collection_filter
        |
        v
local_collect_controller
```

The simulation-only `fake_shuttle_detector` is retained for isolated tests
that intentionally use Gazebo shuttle ground truth. It is not required by the
production-style simulation path.

Simulation-only truth includes:

- `/evaluation/ground_truth_odom`
- `/evaluation/shuttle_ground_truth`
- `/evaluation/shuttle_collected`

No production mission or navigation component should consume these truth topics
directly.

## Real robot

```text
D435i rectified RGB
        |
        v
yolo_shuttle_detector
        |
        +--> /perception/shuttle_detections_2d
        |
D435i aligned depth-to-color + color CameraInfo
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

The detector publishes camera-relative 3D measurements in
`camera_color_optical_frame`.

## Shared obstacle path

```text
depth PointCloud2
 -> pointcloud_to_laserscan
 -> depth_scan_self_filter
 -> /camera/camera/depth/scan
 -> collision_monitor + Nav2 costmaps
```

## Shared AprilTag path

```text
rectified RGB + color CameraInfo
 -> apriltag_ros
 -> /apriltag/detections
 -> AprilTag global localization
```

## Optional tracker

`shuttle_tracker` remains available if a future persistent court-wide shuttle
map is needed. The current production mission does not depend on
`/perception/tracked_shuttles`.

## Localization

Both simulation and real operation use local wheel/IMU odometry plus AprilTag
global correction. `/relocalize` establishes or corrects `map -> odom`.
