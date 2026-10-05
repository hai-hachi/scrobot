# Perception Check

This is the consolidated subsystem test for the production perception stack.

## Scope

The launch validates these paths together:

```text
RGB + CameraInfo -> apriltag_ros -> /apriltag/detections

Depth PointCloud2
 -> pointcloud_to_laserscan
 -> /camera/camera/depth/scan_raw
 -> depth_scan_self_filter
 -> /camera/camera/depth/scan

RGB
 -> YOLO
 -> 2D shuttle bbox
 + aligned depth
 -> /perception/shuttle_detections_3d
```

The shuttle tracker remains optional and is not launched.

## Build

```bash
cd ~/scrobot_ws

git checkout perception-yolo-v2
git pull

colcon build --symlink-install \
  --packages-up-to scrobot_debug scrobot_perception scrobot_simulation

source install/setup.bash
```

## Run

```bash
ros2 launch scrobot_debug perception_check.launch.py \
  model_path:=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt
```

Optional RViz:

```bash
ros2 launch scrobot_debug perception_check.launch.py \
  model_path:=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt \
  launch_rviz:=true
```

## Monitor output

The terminal monitor reports:

```text
[PERCEPTION]
NODES[yolo=OK,tag=OK,scan=OK,filter=OK]
RGB=OK
ALIGN=OK
SCAN=OK valid=...
TAG=OK n=...
YOLO3D=OK n=...
```

It also prints the live simulated CameraInfo:

```text
[CAMERA INFO] color 1280x720 fx=... fy=... cx=... cy=...
[CAMERA INFO] depth 848x480 fx=... fy=... cx=... cy=...
```

These lines are useful when checking the simulated intrinsics against the
recorded physical D435i calibration.

A zero AprilTag count is not itself a failure if no tag is currently visible.
The AprilTag node and topic should still be alive.

## Useful checks

```bash
ros2 topic hz /camera/camera/depth/scan
ros2 topic echo /camera/camera/depth/scan --once

ros2 topic hz /perception/shuttle_detections_3d
ros2 topic echo /perception/shuttle_detections_3d

ros2 topic hz /apriltag/detections
ros2 topic echo /apriltag/detections
```

Inspect CameraInfo directly:

```bash
ros2 topic echo /camera/camera/color/camera_info --once
ros2 topic echo /camera/camera/depth/camera_info --once
```

Annotated YOLO image:

```bash
rqt_image_view
```

Select:

```text
/perception/shuttle_debug/image
```

## Existing validated depth-scan result

The filtered depth scan was already validated during the manual teleop/control
test using RViz. The robot self-mask, visible scan geometry, and collision
monitor behavior were inspected there, so this consolidated check is not meant
to repeat that tuning.

Both collision monitoring and Nav2 are configured to consume:

```text
/camera/camera/depth/scan
```

Nav2 runtime behavior remains part of navigation/mission testing rather than
this perception-only check.

## Shuttle test commands

The full no-restart shuttle spawn/despawn procedure is documented in:

```text
debug_md/yolo_perception.md
```
