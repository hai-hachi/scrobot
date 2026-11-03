# SC Robot Package Architecture

SC Robot uses ROS 2 Jazzy. Gazebo Harmonic provides the repeatable simulation
environment; the physical target is Jetson Orin Nano + STM32F411 + D435i.

## Packages

```text
scrobot_description
scrobot_simulation
scrobot_hardware
scrobot_bringup
scrobot_control
scrobot_localization
scrobot_perception
scrobot_navigation
scrobot_mission
scrobot_interfaces
scrobot_evaluation
scrobot_debug
```

## Ownership

### scrobot_description

Hardware-neutral geometry, meshes, joints, links, collector geometry and D435i
frame tree. Physical `ros2_control` is opt-in through
`use_real_hardware:=true`; simulation injects its own Gazebo control system.

### scrobot_simulation

Gazebo world, robot spawn, simulated sensors and bridges, repeatable shuttle
layouts, AprilTag assets, world-level physical shuttle removal, and
simulation-only evaluation truth.

### scrobot_hardware

Physical STM32 protocol-v2 `hardware_interface::SystemInterface`. It owns the
1 Mbaud serial link, wheel encoder/velocity exchange, ARM/DISARM handshake,
collector motor command transport, and hardware status reporting.

### scrobot_bringup

Physical Jetson composition: robot description, controller manager, D435i,
control stack, local localization and production perception. Global
localization, Nav2 and the mission are opt-in so hardware bringup does not move
the robot unexpectedly.

### scrobot_control

Controller spawners, autonomous command arbitration, AUTO/MANUAL mode,
velocity smoothing, collision monitoring and manual teleoperation.

### scrobot_localization

D435i IMU conversion/filtering, wheel+IMU EKF, tag approach, stationary
AprilTag global correction and ownership of `map -> odom`. Physical D435i
bringup uses the local SensorDataQoS-compatible IMU transformer; simulation can
use the stock `imu_transformer`.

### scrobot_perception

Production AprilTag detection, depth obstacle scan processing and
YOLO + aligned-depth shuttle 3D reconstruction. Simulation uses
PointCloud2 -> LaserScan plus self filtering. A lightweight direct
depth-image -> LaserScan node is retained for physical hardware fallback/tests.

### scrobot_navigation

Nav2 planner/controller/behavior configuration. Regulated Pure Pursuit is the
current path controller.

### scrobot_mission

Four-pass sweep generation, shuttle diversion, local collection, fixed-station
relocalization, checkpoint return/resume, AUTO/MANUAL mission pause/resume and
tag-search recovery.

### scrobot_interfaces

Project actions:
- `Relocalize.action`
- `ApproachTag.action`
- `LocalCollect.action`

Physical hardware messages:
- `CollectorCommand.msg`
- `HardwareStatus.msg`

### scrobot_evaluation

Repeatable local-odometry logging/analysis and full collection-session metrics.

### scrobot_debug

Debug-only launch composition, telemetry, RViz, test worlds, dataset tooling and
isolated regression fixtures. Production packages do not depend on it.

## TF ownership

```text
map                  tag_global_localizer
 |
 v
odom                 EKF
 |
 v
base_footprint
 |
 v
base_link            robot_state_publisher
 |
 +--> collector_link
 +--> wheels/casters
 +--> D435i frames
```

`diff_drive_controller` has `enable_odom_tf: false`; the EKF is the sole
owner of `odom -> base_footprint`.

## Shuttle perception and collection

```text
rectified RGB
   |
   v
YOLO 2D bbox
   +
aligned depth-to-color + color CameraInfo
   |
   v
camera_color_optical_frame 3D point
   |
   v
/perception/shuttle_detections_3d
   |
   v
shuttle_collection_filter
   |
   +-- transform to base_link
   +-- require 0.50-1.80 m planar range
   +-- reject 0.60 m net-pole exclusion regions
   |
   v
/perception/collectable_shuttle_detections_3d
   |
   v
/local_collect
   |
   +-- freeze target in odom
   +-- nominal base_link staging distance 1.10 m
   +-- clamp staging to current range for already-close targets
   +-- SMC pose convergence
   +-- stable 0.25 s
   +-- straight pickup at 0.30 m/s
   +-- 0.10 m overrun
```

The validated simulation off-axis lateral perception error is approximately
0.03 m at about +/-26.7 deg bearing.

## Obstacle paths

Simulation / production RGB-D path:

```text
D435i depth PointCloud2
   -> pointcloud_to_laserscan
   -> /camera/camera/depth/scan_raw
   -> depth_scan_self_filter
   -> /camera/camera/depth/scan
   -> collision_monitor + Nav2 costmaps
```

Physical low-load fallback/test path:

```text
D435i depth image + CameraInfo
   -> depth_obstacle_scan
   -> /camera/camera/depth/scan
```

## AprilTag path

```text
rectified RGB + color CameraInfo
   -> apriltag_ros
   -> /apriltag/detections
   -> tag approach
   -> 0.90 m tag-facing pose
   -> /relocalize (15 stationary samples)
   -> map -> odom correction
```

The production approach strategy is `main_branch`; alternative controller
strategies are debug-only.

## Mission flow

```text
initial tag search/approach
 -> initial relocalization
 -> start Nav2
 -> join four-pass sweep
 -> FollowPath
 -> eligible shuttle?
      yes -> save checkpoint pose/index
           -> cancel FollowPath
           -> local collection spree
           -> Nav2 return to checkpoint
           -> resume saved path index
 -> fixed tag station?
      yes -> relocalize -> restore sweep heading
 -> COMPLETE
```

## Simulation versus robot

Simulation-only truth:

```text
/evaluation/ground_truth_odom
/evaluation/shuttle_ground_truth
/evaluation/shuttle_collected
```

These interfaces are forbidden as production mission/navigation inputs.

Physical-only interfaces:

```text
/hardware/collector_command
/hardware/status
/dev/ttyTHS1 @ 1,000,000 baud
```
