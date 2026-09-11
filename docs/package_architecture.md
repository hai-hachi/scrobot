# SC Robot Package Architecture

## Workspace packages

```text
scrobot_bringup
scrobot_control
scrobot_description
scrobot_evaluation
scrobot_interfaces
scrobot_localization
scrobot_mission
scrobot_navigation
scrobot_perception
scrobot_simulation
```

## `scrobot_description`
Owns the robot model, meshes, links, joints, sensors, and static frame relationships.

Important frame family:

```text
base_footprint
 -> base_link
 -> wheels / casters / collector
 -> camera_link
    -> camera_color_optical_frame
    -> camera_depth_optical_frame
    -> camera_imu_optical_frame
```

## `scrobot_control`
Owns low-level motion command handling and ROS 2 control integration.

Responsibilities:

- differential-drive controller
- wheel feedback
- manual command source
- command arbitration/pipeline

Important topics:

```text
/cmd_vel_manual
/diff_drive_controller/cmd_vel
/diff_drive_controller/odom
/joint_states
```

## `scrobot_localization`
Owns local state estimation and AprilTag global correction.

Responsibilities:

- IMU filtering
- EKF local odometry
- AprilTag global pose estimation
- controlled tag approach
- `map -> odom`

Actions:

```text
/approach_tag  scrobot_interfaces/action/ApproachTag
/relocalize    scrobot_interfaces/action/Relocalize
```

## `scrobot_navigation`
Owns Nav2 configuration and startup.

Responsibilities:

- global/local costmaps
- planner
- behavior tree
- controller
- recovery behaviors

Current path-following controller: Regulated Pure Pursuit.

## `scrobot_mission`
Owns task-level behavior.

Current implementation:

```text
initial tag approach
 -> initial relocalization
 -> start Nav2
 -> patrol points
 -> 360 degree scan at each point
```

Packaging uses `ament_cmake` + `ament_cmake_python`. Launch/config files and `patrol_manager` are installed explicitly so `colcon build --symlink-install` is supported.

## `scrobot_perception`
Owns shuttle perception adapters and tracking.

### Simulation path

```text
/evaluation/shuttle_ground_truth
 -> fake_shuttle_detector
 -> /perception/shuttle_detections_3d
 -> shuttle_tracker
 -> /perception/tracked_shuttles
```

### Future real path

```text
RGB -> YOLO -> /perception/detections_2d
aligned depth + CameraInfo + detections_2d
 -> depth_localizer
 -> /perception/shuttle_detections_3d
 -> shuttle_tracker
```

The tracker is environment-independent.

## `scrobot_simulation`
Owns Gazebo world/model setup, sensors, bridge configuration, shuttle spawning, and simulation-only plugins.

Important simulation truth topics:

```text
/evaluation/ground_truth_odom
/evaluation/ground_truth_tf
/evaluation/shuttle_ground_truth
```

The shuttle-only stream is produced by `shuttle_activity_system` and bridged from `gz.msgs.Pose_V` to `geometry_msgs/msg/PoseArray`.

## `scrobot_evaluation`
Owns repeatable test runners, loggers, and result analysis.

Its local-odometry reference remains `/evaluation/ground_truth_odom`; the shuttle ground-truth change does not alter that evaluation interface.

## `scrobot_interfaces`
Owns project-specific action definitions.

Currently:

```text
ApproachTag.action
Relocalize.action
```

## `scrobot_bringup`
Reserved for integrated launch/orchestration of multiple subsystems.

## Dependency direction

Preferred architecture:

```text
description
   |
simulation / hardware
   |
control + localization + perception
   |
navigation
   |
mission
   |
evaluation observes the system without controlling production behavior
```

Simulation-specific ground truth must not leak directly into mission/navigation logic.
