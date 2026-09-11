# SC Robot Node Specification

Updated for the current ROS 2 Jazzy / Gazebo Harmonic stack.

## scrobot_control

### `wasd_teleop`
Manual keyboard driving utility.

- Publishes: `/cmd_vel_manual`
- Purpose: operator driving and low-level testing.

### `diff_drive_controller`
ROS 2 control differential-drive controller.

- Consumes final velocity command from the command pipeline.
- Publishes wheel odometry and joint feedback.
- Important outputs: `/diff_drive_controller/odom`, `/joint_states`.

## scrobot_localization

### `tag_global_localizer`
AprilTag-based global pose correction.

- Action server: `/relocalize`
- Action type: `scrobot_interfaces/action/Relocalize`
- Owns the global correction transform `map -> odom`.
- Should perform global correction while the robot is stationary.

### `tag_approach_controller`
Finds/approaches a suitable AprilTag before relocalization.

- Action server: `/approach_tag`
- Action type: `scrobot_interfaces/action/ApproachTag`
- Publishes relocalization motion commands through the localization command path.

### EKF / IMU filtering
Fuses wheel odometry and IMU for local motion estimation and maintains the local `odom -> base_footprint` estimate.

## scrobot_navigation

### Nav2 stack
Provides planning, behavior-tree execution, path following, recovery, and local/global costmaps.

- Current path tracking controller: Regulated Pure Pursuit.
- Primary mission action: `NavigateToPose`.
- Uses the standard `map -> odom -> base_footprint` TF chain.

## scrobot_mission

### `patrol_manager`
Mission state machine.

Current flow:

```text
initial tag approach
 -> initial /relocalize
 -> start Nav2
 -> navigate patrol point
 -> 360 degree scan
 -> next patrol point
```

Runtime shuttle diversion/collection is the next mission-stage integration after tracker validation.

`scrobot_mission` is installed with `ament_cmake` + `ament_cmake_python` so both normal and `--symlink-install` builds expose its launch/config/executable correctly.

## scrobot_perception

### `fake_shuttle_detector`
Simulation-only perception adapter.

Inputs:

- `/evaluation/shuttle_ground_truth` (`geometry_msgs/msg/PoseArray`)
- `/evaluation/ground_truth_odom`
- `/camera/camera/color/camera_info`
- static base/camera TF

Output:

- `/perception/shuttle_detections_3d`
- type: `vision_msgs/msg/Detection3DArray`
- frame: `camera_depth_optical_frame`
- sensor-data QoS

The detector limits Gazebo shuttle truth by camera range/FOV and emits the same 3D detection interface planned for the real RGB-D pipeline. It does not assign persistent IDs.

Synthetic detection timestamps are derived from Gazebo ground-truth odometry so they remain in simulation time.

### `shuttle_tracker`
Persistent map-frame shuttle tracker.

Input:

- `/perception/shuttle_detections_3d`

Output:

- `/perception/tracked_shuttles`
- type: `vision_msgs/msg/Detection3DArray`
- frame: `map`
- Reliable QoS
- default publish rate: 10 Hz

Processing:

```text
camera-frame detection
 -> TF into map
 -> nearest-neighbor association
 -> exponential position smoothing
 -> persistent integer ID
 -> stale-track removal
```

Default gate is 0.30 m, smoothing alpha 0.50, stale timeout 1.50 s. Exact measurement-time TF is preferred; the tracker can fall back to the latest transform during startup/timing skew.

## scrobot_simulation

### `shuttle_activity_system`
Gazebo system plugin that manages shuttle dynamic/static behavior and publishes shuttle-only ground truth.

Gazebo output:

- `/evaluation/shuttle_ground_truth_gz` (`gz.msgs.Pose_V`)

ROS bridge output:

- `/evaluation/shuttle_ground_truth` (`geometry_msgs/msg/PoseArray`)

This stream is separate from normal robot ground truth used by evaluation.

### Shuttle spawner
Spawns `single`, `random`, `cluster`, or `mixed` shuttle distributions from `shuttle_spawn.yaml`.

## scrobot_evaluation

### `local_odom_logger`
Records wheel odometry, EKF odometry, IMU, joint feedback, TF, commands, and Gazebo ground truth.

Primary truth input:

- `/evaluation/ground_truth_odom`

The shuttle-only truth topic does not replace or modify this input.

### `local_odom_test_runner`
Generates repeatable static, straight, rotation, and arc tests for local-odometry evaluation.
