# SC Robot Package Architecture

SC Robot uses ROS 2 Jazzy with Gazebo Harmonic in simulation. The same
production ROS interfaces are intended for the Jetson Orin Nano robot.

## Packages

```text
scrobot_description
scrobot_simulation
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

Hardware-neutral URDF/Xacro, meshes, joints, links, collector geometry, and
camera frame tree.

### scrobot_simulation

Gazebo world, robot spawn, simulated sensors, bridges, production shuttle
physics, shuttle spawning, generated AprilTag assets, and evaluation truth.

### scrobot_control

ROS 2 control, autonomous command arbitration, AUTO/MANUAL mode, velocity
smoothing, collision monitoring, and manual teleoperation.

### scrobot_localization

D435i IMU conversion/filtering, wheel+IMU EKF, tag approach, stationary
AprilTag global correction, and ownership of `map -> odom`.

### scrobot_perception

Production AprilTag detection, depth PointCloud2 -> LaserScan processing,
self-filtering, and YOLO + aligned-depth shuttle 3D reconstruction.

### scrobot_navigation

Nav2 planner/controller/behavior configuration. Regulated Pure Pursuit is the
current path controller.

### scrobot_mission

Four-pass sweep generation, shuttle diversion, local collection, fixed-station
relocalization, exact checkpoint return, AUTO/MANUAL mission pause/resume, and
tag-search recovery.

### scrobot_interfaces

Project action definitions only.

### scrobot_evaluation

Simulation logging, repeatable trajectories, mission metrics, and analysis.

### scrobot_debug

Debug-only launch composition, monitors, RViz, test worlds, dataset generation,
and isolated regression fixtures. Production packages do not depend on it.

## TF ownership

```text
map                  scrobot_localization/tag_global_localizer
 |
 v
odom                 scrobot_localization EKF
 |
 v
base_footprint
 |
 v
base_link            robot_state_publisher / scrobot_description
 |
 +--> collector_link
 +--> wheels/casters
 +--> D435i frames
```

Production `diff_drive_controller` has `enable_odom_tf: false`.

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
   +-- base_link pre-pose at 1.10 m
   +-- pure SMC convergence
   +-- stable 0.25 s
   +-- straight pickup at 0.30 m/s
   +-- 0.10 m overrun
```

The validated simulation off-axis lateral perception error is approximately
0.03 m at about +/-26.7 deg bearing.

## Obstacle path

```text
D435i depth PointCloud2
   -> pointcloud_to_laserscan
   -> /camera/camera/depth/scan_raw
   -> depth_scan_self_filter
   -> /camera/camera/depth/scan
   -> collision_monitor + Nav2 costmaps
```

Current scan envelope:

```text
height 0.12-0.70 m
HFOV   +/-0.7897925312 rad
range  0.30-3.30 m
```

## AprilTag path

```text
rectified RGB + color CameraInfo
   -> apriltag_ros
   -> /apriltag/detections
   -> tag approach
   -> 0.90 m base_link tag-facing pose
   -> /relocalize (15 stationary samples)
   -> map -> odom correction
```

The production tag approach uses `main_branch`; alternative SMC/biarc
strategies are retained only as debug regressions.

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
           -> Nav2 return to original checkpoint
           -> resume saved path index
 -> fixed tag station?
      yes -> relocalize -> restore sweep heading
 -> COMPLETE
```

## Simulation versus robot

Simulation substitutes Gazebo sources for physical devices but keeps production
topics and frames where practical.

Simulation-only truth:

```text
/evaluation/ground_truth_odom
/evaluation/shuttle_ground_truth
/evaluation/shuttle_collected
```

These topics are for debug/evaluation only and must not feed production mission
or navigation decisions.
