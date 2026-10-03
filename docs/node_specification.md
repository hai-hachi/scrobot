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

### `sweep_mission_manager`
Final autonomous mission state machine.

Current flow:

```text
full-spin tag search / approach
 -> retry or recovery if no tag
 -> initial /relocalize
 -> start Nav2
 -> join four-pass sweep
 -> fixed-station AprilTag relocalization when reached
 -> shuttle diversion when an eligible shuttle is visible
 -> local collection spree
 -> return to saved sweep checkpoint
 -> resume sweep
```

The sweep checkpoint stores both robot pose and sweep-path progress so a local
collection diversion does not lose coverage progress.

### `shuttle_collection_filter`
Mission-specific shuttle eligibility filter.

- Input: `/perception/shuttle_detections_3d`
- Output: `/perception/collectable_shuttle_detections_3d`
- Accepts only shuttles within 1.68 m of the robot.
- Rejects shuttles within 0.10 m of either net pole.

### `local_collect_controller`
Action server: `/local_collect` using `scrobot_interfaces/action/LocalCollect`.

For each frozen eligible shuttle:

```text
construct collector pre-pose 0.50 m before shuttle
 -> SMC pose control
 -> position error <= 0.03 m and yaw error <= 5 deg
 -> straight collection at 0.30 m/s, omega = 0
 -> collector reaches shuttle
 -> 0.10 m overrun
 -> select next eligible visible shuttle
```

The SMC sliding surface is `s = e_theta + lambda*e_y`. Because the
pre-collection reference is a fixed pose, `v_R = 0` and `omega_R = 0`,
so the angular law is
`omega = [k_s*s + eta*sat(s/phi)] / [1 + lambda*c]`.
Current parameters are `lambda=2.0`, `k_s=1.6`, `eta=0.50`,
`phi=0.08`, `k_rho=0.8`, collector offset `c=0.165 m`, and
`|omega| <= 1.80 rad/s`. The turn-first gate at 70 deg is used only while
`rho > 0.15 m`; near the pre-pose, normal SMC remains active.

`scrobot_mission` is installed with `ament_cmake` + `ament_cmake_python`
so both normal and `--symlink-install` builds expose its launch/config/executable correctly.

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
