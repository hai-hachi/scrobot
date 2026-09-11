# SC Robot ROS 2 Architecture

Updated for ROS 2 Jazzy + Gazebo Harmonic and the shuttle perception/tracking pipeline.

## System flow

```text
Gazebo / real hardware
        |
        +--> sensors / odometry
        |
        +--> localization --------------------+
        |                                     |
        |                                     v
        |                                map -> odom
        |                                     |
        +--> perception --> shuttle tracker --+
        |                         |
        |                         v
        |             /perception/tracked_shuttles
        |                         |
        +--> Nav2 <--- mission manager <-------+
                    |
                    v
               command pipeline
                    |
                    v
           diff_drive_controller
```

## Frames

Primary chain:

```text
map -> odom -> base_footprint -> base_link
                         |
                         +--> wheels / casters / collector
                         +--> camera_link
                               +--> camera_color_optical_frame
                               +--> camera_depth_optical_frame
                               +--> camera_imu_optical_frame
```

`map -> odom` is owned by global localization. `odom -> base_footprint` comes from the local odometry / EKF chain.

## Simulation-only truth

Simulation exposes three evaluation/perception references:

```text
/evaluation/ground_truth_odom
/evaluation/ground_truth_tf
/evaluation/shuttle_ground_truth
```

`/evaluation/shuttle_ground_truth` is a shuttle-only `geometry_msgs/msg/PoseArray` generated from Gazebo. It exists only to drive fake shuttle perception and does not replace the odometry evaluation topics.

## Shuttle perception

Simulation:

```text
Gazebo shuttle poses
      -> /evaluation/shuttle_ground_truth
      -> fake_shuttle_detector
      -> /perception/shuttle_detections_3d
      -> shuttle_tracker
      -> /perception/tracked_shuttles
```

Real robot target architecture:

```text
RGB -> YOLO -> /perception/detections_2d
                   +
aligned depth + CameraInfo
                   -> depth_localizer
                   -> /perception/shuttle_detections_3d
                   -> shuttle_tracker
                   -> /perception/tracked_shuttles
```

The detector owns measurements. The tracker owns persistent shuttle IDs.

## Localization and mission interaction

The global-localization stack provides `/approach_tag` and `/relocalize`. The patrol mission uses them during initial acquisition before starting Nav2. The shuttle tracker does not command localization; it waits until a valid transform to `map` is available.

## Navigation and control

Nav2 plans and tracks paths. The current path-following controller is Regulated Pure Pursuit. Velocity commands pass through the command pipeline and then the differential-drive controller.

## Design rule

Simulation-specific truth must stop at adapters such as `fake_shuttle_detector`. Mission, tracking, navigation, and control should use the same ROS interfaces intended for the real robot.
