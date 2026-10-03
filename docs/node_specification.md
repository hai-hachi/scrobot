# SC Robot Node Specification

Updated for the current ROS 2 Jazzy / Gazebo Harmonic stack.

## scrobot_control

### `manual_mode_manager`
Production AUTO/MANUAL arbitration state owner.

- Service: `/control/set_manual_mode` (`std_srvs/srv/SetBool`)
- Publishes latched state on `/control/manual_mode` and `/control/mode`.
- Input: `/cmd_vel_manual_input`.
- Output: `/cmd_vel_manual`.
- While MANUAL is active, keeps the high-priority manual path alive with the
  latest operator command or zero so autonomous motion cannot leak through
  between keyboard commands.
- AUTO/MANUAL selection occurs before velocity smoothing and collision
  monitoring, so normal manual driving remains collision-protected.

### `manual_teleop`
Interactive keyboard client for the manual-mode manager.

- `M`: enter MANUAL.
- `R`: return to AUTO.
- `W/S/A/D`: drive.
- Publishes only to `/cmd_vel_manual_input`; it does not own the final command
  mux directly.

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
initial tag approach
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

The runtime manager also observes `/control/manual_mode`. MANUAL cancels the
active autonomous action and enters `PAUSED`. AUTO resumes directly when the
robot remained within 0.15 m and 10 deg of the interruption checkpoint;
otherwise Nav2 first returns to that checkpoint, then recreates the saved
autonomous phase.

### `shuttle_collection_filter`
Mission-specific shuttle eligibility filter.

- Input: `/perception/shuttle_detections_3d`
- Output: `/perception/collectable_shuttle_detections_3d`
- Accepts only shuttles within 2.0 m of the robot.
- Rejects shuttles within 0.60 m of either net pole.

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

The SMC sliding surface is `s = e_theta + lambda*e_y` with the accepted
parameters `lambda=2.0`, `k_s=2.0`, `eta=0.8`, `phi=0.05`,
`v_R=0.50 m/s`, `k_rho=0.8`, collector offset `c=0.165 m`, and
`|omega| <= 2.0 rad/s`.

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
Gazebo world plugin that publishes shuttle-only ground truth and removes a
shuttle when its geometry intersects the collector pickup envelope. Shuttle
models remain static after spawning so simplified feather collisions cannot
produce nonphysical rolling.

Gazebo output:

- `/evaluation/shuttle_ground_truth_gz` (`gz.msgs.Pose_V`)

ROS bridge output:

- `/evaluation/shuttle_ground_truth` (`geometry_msgs/msg/PoseArray`)

This stream is separate from normal robot ground truth used by evaluation.

### Shuttle spawner
Spawns `single`, `random`, `cluster`, or `mixed` distributions using one
detailed shuttle model suitable for future YOLO-on-simulated-RGB testing.

## scrobot_debug

### `telemetry_monitor`
Debug-only consolidated terminal monitor.

- Output topic: `/debug/telemetry`.
- Observes AUTO/MANUAL state, mission state, local collection phase,
  perception counts, shuttle ground truth/collection events, odometry versus
  Gazebo truth, command-path velocity, and selected `/rosout` streams.

### `debug_raw_teleop`
Debug-only keyboard driver that publishes directly to
`/diff_drive_controller/cmd_vel`. It intentionally bypasses command
arbitration, velocity smoothing, and collision monitoring and must not be used
as the production manual-control path.

### `shuttle_sim_monitor`
Measures shuttle drift and collector-envelope clearance during isolated shuttle
physics/collision tests.

### `court_visualizer`
RViz MarkerArray-only court visualization. This node and RViz launch ownership
were moved out of `scrobot_simulation`.

## scrobot_evaluation

### `local_odom_logger`
Records wheel odometry, EKF odometry, IMU, joint feedback, TF, commands, and Gazebo ground truth.

Primary truth input:

- `/evaluation/ground_truth_odom`

The shuttle-only truth topic does not replace or modify this input.

### `local_odom_test_runner`
Generates repeatable static, straight, rotation, and arc tests for local-odometry evaluation.
