# scrobot_perception Regression

## Combined production perception

```bash
ros2 launch scrobot_debug perception_check.launch.py \
  model_path:=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt
```

This checks AprilTag input, the filtered depth scan, CameraInfo, YOLO 2D, and
YOLO/depth 3D output.

## Camera geometry

```bash
ros2 launch scrobot_debug camera_check.launch.py
```

Validated simulated D435i:

```text
RGB    1280x720 @ 15 Hz, HFOV ~70.42 deg
Depth  848x480  @ 15 Hz, HFOV ~90.50 deg
mount  15 deg downward
nearest floor-level RGB shuttle center ~0.46 m from base_link
```

Use `camera_test_ctl` for isolated spawn/move/frame-edge checks.

## YOLO + depth 3D accuracy

```bash
ros2 launch scrobot_debug yolo_perception_check.launch.py \
  model_path:=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt \
  shuttle_x:=1.50 shuttle_y:=0.30
```

Representative accepted simulation results:

```text
bearing +11.35 deg: planar RMSE 0.0270 m, lateral 0.0151 m
bearing +26.67 deg: planar RMSE 0.0342 m, lateral 0.0296 m
bearing -26.67 deg: planar RMSE 0.0512 m, lateral 0.0304 m
```

The approximately 0.03 m off-axis lateral bias is accepted for the 0.30 m
collector width.

## Mission range gate

```bash
ros2 launch scrobot_debug yolo_range_gate_check.launch.py \
  model_path:=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt \
  shuttle_x:=0.55 shuttle_y:=0.0
```

Validated boundary cases:

```text
0.45 m reject PASS
0.55 m accept PASS
1.75 m accept PASS
1.85 m reject PASS
```

## Synthetic YOLO dataset

The single supported generator is:

```bash
ros2 launch scrobot_debug yolo_dataset_capture.launch.py
```

Defaults:

```text
target_images          1200
shuttle_count          40
positive_pose_fraction 0.85
random_seed            42
```

It uses a lightweight RGB-only Gazebo world, the production shuttle STL, live
CameraInfo, geometric box projection, and automatic train/val/test output.
