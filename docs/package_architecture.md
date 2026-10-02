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

Owns robot geometry, meshes, links, joints, and static sensor transforms.

Current D435i tripod-screw mount relative to `base_link`:

```text
x = +0.110 m
y =  0.000 m
z = +0.2275 m
pitch = 15 deg downward
```

The separate HMC5883L path is no longer part of the current robot model.

## `scrobot_control`

Owns the differential-drive controller, command arbitration, velocity smoothing,
collision monitoring, and manual teleoperation.

Important command flow:

```text
Nav2 / local collect / relocalization
        -> autonomous twist mux
        -> velocity smoother
        -> collision monitor
        -> manual override mux
        -> diff_drive_controller
```

## `scrobot_localization`

Owns local wheel/IMU state estimation and AprilTag global correction.

```text
D435i IMU
 -> imu_transformer
 -> /imu/data_raw
 -> Madgwick (no magnetometer)
 -> /imu/data
       +
wheel odometry
 -> EKF
 -> odom -> base_footprint
```

AprilTag family: `16h5`, IDs 0-3, active edge 100 mm.

Actions:

```text
/approach_tag
/relocalize
```

A failed cold-start tag search is retried after a full in-place scan. The mission
also retains the last successful localization pose for recovery.

## `scrobot_navigation`

Owns Nav2 planning, costmaps, behavior servers, and Regulated Pure Pursuit.

Current long-traverse / sweep cruise command: 0.80 m/s.

The navigation footprint uses the full base envelope:

```text
front +0.325 m
rear  -0.450 m
left/right +/-0.225 m
```

## `scrobot_mission`

Owns the final four-pass coverage mission and local shuttle collection.

```text
initial tag acquisition
 -> initial relocalization
 -> join four-pass sweep
 -> fixed AprilTag relocalization stations
 -> shuttle interrupt when target <= 1.68 m
 -> local SMC collection
 -> return to exact saved sweep XY with sweep-tangent yaw
 -> resume sweep
```

Targets within 0.10 m of either net pole are intentionally excluded.

Local collection:

```text
freeze shuttle in odom
 -> construct collector pre-pose 0.50 m before shuttle
 -> SMC pose convergence
 -> straight pass at 0.30 m/s, omega = 0
 -> 0.10 m overrun
```

## `scrobot_perception`

Simulation path:

```text
Gazebo shuttle ground truth
 -> fake_shuttle_detector
 -> /perception/shuttle_detections_3d
```

The fake detector is camera-FOV limited and currently uses 0.17-1.68 m as the
useful range.

The next real-robot integration step is:

```text
D435i RGB -> YOLO 2D shuttle detection
aligned depth + camera intrinsics
 -> 3D shuttle position
 -> /perception/shuttle_detections_3d
```

## `scrobot_simulation`

Owns Gazebo Harmonic world/model setup, the D435i-like sensors, shuttle spawning,
bridges, court AprilTags, and simulation-only truth.

## `scrobot_evaluation`

Observes the mission without controlling it. Collection totals are authoritative
from periodic Gazebo shuttle ground truth; the one-shot collected event stream is
kept only as a diagnostic cross-check.

## `scrobot_interfaces`

Owns project actions:

```text
ApproachTag.action
LocalCollect.action
Relocalize.action
```

## `scrobot_bringup`

Reserved for integrated real-robot orchestration as the Orin/STM32 deployment is
completed.

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
evaluation observes only
```

Simulation-specific ground truth must not leak directly into production mission,
navigation, or control logic.
