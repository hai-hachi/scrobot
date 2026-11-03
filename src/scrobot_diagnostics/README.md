# SCROBOT diagnostics

Read-only ROS 2 Jazzy health monitoring for simulation and future hardware
bringup. It does not publish velocity, move the robot, modify parameters,
manage lifecycles, cancel actions, or override Nav2 safety.

## What is implemented

- **/diagnostics**: `diagnostic_msgs/msg/DiagnosticArray` at 1 Hz, with
  `SCROBOT/Overall` and per-subsystem status.
- **/debug/health_summary**: a transient-local JSON string containing the
  highest-priority blocker, suggested next step, mission/collection state,
  command velocities, and all active alerts.
- **/debug/incident_events**: a transient-local stream of structured
  `diagnostic_msgs/msg/DiagnosticStatus` events for mission transitions,
  SMC phases, control-mode changes, collision-monitor actions and health
  transitions. Events have `code` and UTC timestamp key/value fields.
- **/mission/failure_reason**: published by the existing mission manager. The
  error text survives as a transient-local ROS message instead of appearing
  exclusively in the original process log.
- Optional Foxglove WebSocket bridge.
- Optional bounded-memory rosbag2 MCAP snapshot recorder. The monitor can
  request snapshots on selected persistent faults, with a cooldown.

The health timer uses **steady/wall-clock time** so it keeps detecting missing
messages if Gazebo `/clock` is paused or DDS traffic is interrupted.

## Install on Ubuntu 24.04 / ROS 2 Jazzy

```bash
sudo apt update
sudo apt install \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-rosbag2-interfaces

cd ~/scrobot_ws
colcon build --symlink-install --packages-up-to scrobot_debug
source install/setup.bash
```

Foxglove is optional. To run the diagnostics node without the bridge, the
normal ROS 2 dependencies are sufficient.

## Launch alongside a full mission (recommended)

```bash
export SCROBOT_YOLO_MODEL=/absolute/path/to/gazebo_simple_v2.pt

ros2 launch scrobot_debug full_mission_check.launch.py \
  launch_telemetry:=false \
  monitor_foxglove:=true \
  monitor_recording:=true \
  monitor_auto_snapshot:=true
```

The built-in health node is on by default. Foxglove and incident recording are
opt-in. For a non-recording run set both `monitor_recording` and
`monitor_auto_snapshot` to `false`.

**Important:** do not run a second `monitor.launch.py` alongside the
integrated full-mission monitor. That would create duplicate health publishers,
events and potentially competing snapshot requests.

## Run separately against an already running stack

```bash
ros2 launch scrobot_diagnostics monitor.launch.py \
  enable_foxglove:=true \
  record_incidents:=true \
  auto_snapshot:=true
```

For the minimal Gazebo-only `simulation.launch.py` (without Nav2, EKF,
filtered scan or YOLO):

```bash
ros2 launch scrobot_diagnostics monitor.launch.py \
  enable_foxglove:=true \
  expect_mission:=false \
  expect_perception:=false \
  expect_localization:=false \
  expect_scan:=false
```

After starting, open Foxglove and choose **Foxglove WebSocket** at:

```text
ws://localhost:8765
```

The bridge may listen on network interfaces beyond localhost. Keep port 8765
off public/untrusted networks and do not enable remote access unless intended.
If Cloudflare WARP or another VPN interferes with DDS, disable it or configure
ROS discovery explicitly before launching the nodes.

## Foxglove workspace: suggested first saved layout

Foxglove's exported layout JSON format is not a stable public interchange
schema. Build the following panels once and export your working layout from
Foxglove's **Layouts -> Export** menu; do not fabricate an untested JSON file.

| Panel | Topic | Purpose |
| --- | --- | --- |
| Diagnostics | `/diagnostics` | Overall alarm and each subsystem status |
| Raw Messages | `/debug/health_summary` | One-stop code, reason and next step |
| Raw Messages | `/debug/incident_events` | Timestamped state, fault and safety events |
| Raw Messages | `/mission/failure_reason` | Exact mission failure |
| Plot | `/cmd_vel_smoothed`, `/diff_drive_controller/cmd_vel`, `/odometry/filtered` | Track command suppression and physical progress |
| 3D | `/tf`, `/mission/sweep_path`, `/mission/current_goal` | Robot location and planned route |
| Image | `/perception/shuttle_debug/image` | YOLO detections, if published |

Keep your existing master RViz for detailed TF, costmaps, and spatial debugging.
Optional controller work can still use PlotJuggler.

## Diagnosis examples

| Code | Meaning | Check |
| --- | --- | --- |
| `CLOCK_STALLED` | No recent Gazebo clock | Paused sim, Gazebo bridge, network/DDS |
| `ROBOT_DESCRIPTION_MISSING` | No durable description received | Xacro/robot_state_publisher |
| `GLOBAL_TF_MISSING` | Map transform absent during active mission | AprilTag relocalization |
| `RGB_STALE` or `DEPTH_STALE` | Image stream stopped | Gazebo sensor/bridge |
| `DEPTH_SCAN_STALE` | Nav2 safety scan stopped | Pointcloud-to-scan/filter |
| `COLLISION_STOP` | Safety polygon active | Collision state and scan |
| `FINAL_COMMAND_SUPPRESSED` | Smoothed velocity nonzero, drive velocity zero | Collision monitor/output |
| `MOTION_STALLED` | Nonzero final command, stationary odometry for >4 s | Drive/physics/controller |
| `AUTO_COMMAND_SUPPRESSED` | AUTO command did not pass mode mux | AUTO/MANUAL pipeline |
| `MISSION_NO_COMMAND` | Navigation state idle >15 s | Nav2 action/lifecycle |
| `MISSION_STATE_TIMEOUT` | Non-travel mission state exceeded a conservative threshold | Pending action |
| `MISSION_ERROR` | Mission explicitly failed | `/mission/failure_reason` |

Codes are *diagnostic hypotheses*, not commands to bypass collision safety.
Expected waits, MANUAL mode, initial lack of `map->odom`, and long healthy
SWEEPING runs are not automatically classified as stuck.

## Incident MCAP snapshots

When enabled, the recorder runs in rosbag2 `--snapshot-mode`, keeping at most
50 MB of recent message cache (implementation may require additional buffer
memory). It records selected state, control, TF, lidar/scan, diagnostics and
evaluation topics, **not** raw RGB/depth images by default.

The monitor requests `/rosbag2_recorder/snapshot` for new critical fault
codes and throttles subsequent requests. Bags are written below:

```text
~/scrobot_ws/evaluation_results/incident_bags/
```

A human can trigger a snapshot at any time:

```bash
ros2 service call /rosbag2_recorder/snapshot rosbag2_interfaces/srv/Snapshot "{}"
```

Inspect saved `*.mcap` files using Foxglove's **Open local file** flow. The
snapshot service is available only while recording was enabled. The captured
history is memory/bytes bounded rather than guaranteed to include a fixed
number of seconds. Snapshot mode is not a persistent recording of every event.

## Verification

```bash
ros2 topic echo /debug/health_summary --once
ros2 topic echo /diagnostics --once
ros2 topic echo /debug/incident_events
ros2 topic echo /mission/failure_reason \
  --qos-durability transient_local --once

colcon test --packages-select scrobot_diagnostics
colcon test-result --verbose
```

Fault-injection expectations are covered by
`test/test_health_rules.py` without requiring live Gazebo. ROS integration,
Foxglove display, bag writes and MCAP playback **must still be verified on the
actual simulation machine** before treating this as a validated release.
