# SC Robot Package Architecture

## Workspace packages

```text
scrobot_bringup
scrobot_control
scrobot_debug
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
Owns the robot model, meshes, links, joints, sensors, and static frame
relationships.

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
Owns low-level motion command handling, ROS 2 control integration, and the
production AUTO/MANUAL mode.

Responsibilities:

- differential-drive controller
- wheel feedback
- autonomous command arbitration
- velocity smoothing and autonomous collision monitoring
- explicit AUTO/MANUAL mode
- manual command gating
- keyboard manual-teleop client

Command flow:

```text
Nav2 / local collection / relocalization
 -> autonomous twist_mux
 -> velocity_smoother
 -> collision_monitor
 -> /cmd_vel_safe_auto
                       \
                        -> manual_override_mux
                       /
manual_teleop
 -> /cmd_vel_manual_input
 -> manual_mode_manager
 -> /cmd_vel_manual
```

`manual_mode_manager` owns `/cmd_vel_manual`. While MANUAL is active it
keeps the high-priority manual source alive with the latest operator command or
zero, so autonomous motion cannot reappear between key presses.

Control-mode interfaces:

```text
/control/set_manual_mode   std_srvs/srv/SetBool
/control/manual_mode       std_msgs/msg/Bool
/control/mode              std_msgs/msg/String
```

Manual motion currently bypasses collision_monitor intentionally so an operator
can back out of an autonomous stop condition.

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
- Regulated Pure Pursuit path controller
- recovery behaviors

## `scrobot_mission`
Owns task-level autonomous behavior.

Current flow:

```text
initial tag approach
 -> initial relocalization
 -> start Nav2
 -> join four-pass sweep
 -> fixed AprilTag relocalization stops
 -> shuttle diversion
 -> local collection spree
 -> return to saved sweep checkpoint
 -> resume sweep
```

The mission listens to `/control/manual_mode`. Entering MANUAL cancels the
active autonomous action and preserves mission context. Returning to AUTO
restores the interruption checkpoint where needed and resumes the saved phase.

Debug/telemetry monitors are not owned or launched by this package.

## `scrobot_perception`
Owns production camera perception and optional simulation adapters.

Production shuttle path:

```text
rectified RGB
 -> yolo_shuttle_detector
 -> /perception/shuttle_detections_2d
                +
RGB-aligned depth + color CameraInfo
                |
                v
/perception/shuttle_detections_3d
                |
                v
scrobot_mission/shuttle_collection_filter
```

Obstacle path:

```text
depth PointCloud2
 -> pointcloud_to_laserscan
 -> depth_scan_self_filter
 -> /camera/camera/depth/scan
 -> collision_monitor + Nav2 costmaps
```

AprilTag path:

```text
rectified RGB + color CameraInfo
 -> apriltag_ros
 -> /apriltag/detections
 -> scrobot_localization
```

`fake_shuttle_detector` is simulation/testing-only. `shuttle_tracker` is
retained as optional legacy infrastructure and is not required by the current
mission.

## `scrobot_simulation`
Owns Gazebo runtime functionality only:

- badminton world and safety boundary
- robot spawning
- Gazebo sensor definitions and ROS bridges
- depth registration
- detailed shuttle model and distributions
- shuttle collection/ground-truth Gazebo plugin
- generated AprilTag court model

RViz, MarkerArray court visualization, and test monitors are not part of this
package.

Important simulation truth topics:

```text
/evaluation/ground_truth_odom
/evaluation/ground_truth_tf
/evaluation/shuttle_ground_truth
/evaluation/shuttle_collected
```

Simulation ground truth is for perception adapters, evaluation, and debug only;
it must not feed mission/navigation decision logic directly.

## `scrobot_debug`
Owns test orchestration and observation.

Responsibilities:

- subsystem-specific debug launch files
- centralized terminal telemetry
- shuttle collision/physics monitor
- RViz and court MarkerArray visualization
- launching the production dependencies needed for each isolated test

It must not contain the production implementation of manual control, mission
logic, localization, or perception.

## `scrobot_evaluation`
Owns repeatable experiment runners, loggers, metrics, and result analysis.

Evaluation observes production behavior; it does not arbitrate robot control.

## `scrobot_interfaces`
Owns project-specific ROS action definitions.

## `scrobot_bringup`
Owns integrated production orchestration across multiple subsystems. Debug/test
composition belongs in `scrobot_debug`, not here.

## Dependency direction

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
bringup composes production subsystems

debug/evaluation observe or exercise the public interfaces
without owning production behavior
```
