# YOLO Perception Check

End-to-end validation of the production shuttle perception path.

## Purpose

This test validates the complete detector boundary without launching the mission
controller or optional shuttle tracker:

```text
Gazebo shuttle
    ↓
simulated D435i RGB
    ↓
YOLO gazebo_simple_v2
    ↓
2D shuttle bbox
    ↓
RGB-aligned depth
    ↓
color CameraInfo deprojection
    ↓
/perception/shuttle_detections_3d
```

## What the launch starts

```text
Gazebo badminton court
+ SCROBOT simulation
+ simulated D435i RGB/depth
+ depth-to-color registration
+ one dynamic shuttle at (1.00, 0.00) by default
+ scrobot_perception/yolo_shuttle_detector
+ scrobot_debug/yolo_perception_monitor
```

## 1. Pull and build

```bash
cd ~/scrobot_ws

git checkout perception-yolo-v2
git pull

colcon build --symlink-install \
  --packages-up-to scrobot_debug scrobot_perception

source install/setup.bash
```

Confirm the detector executable is installed:

```bash
ros2 pkg executables scrobot_perception | grep yolo
```

Expected:

```text
scrobot_perception yolo_shuttle_detector
```

## 2. Optional Python / CUDA sanity check

```bash
python3 - <<'PY'
import numpy
import cv2
import torch
import ultralytics
import rclpy
import cv_bridge
import message_filters

print("numpy          :", numpy.__version__)
print("opencv         :", cv2.__version__)
print("torch          :", torch.__version__)
print("ultralytics    :", ultralytics.__version__)
print("CUDA available :", torch.cuda.is_available())

if torch.cuda.is_available():
    print("GPU            :", torch.cuda.get_device_name(0))

print("ROS imports     : OK")
PY
```

Optional dependency check:

```bash
python3 -m pip check
```

## 3. Optional direct detector startup test

Use this if the full debug launch reports `NODE=MISSING` or the detection
topics show zero publishers:

```bash
ros2 run scrobot_perception yolo_shuttle_detector \
  --ros-args \
  -p use_sim_time:=true \
  -p model_path:=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt \
  -p device:=0 \
  -p publish_debug_image:=true
```

Healthy startup contains:

```text
Loading YOLO model: ...gazebo_simple_v2.pt
YOLO shuttle detector ready: ...
```

In another terminal:

```bash
ros2 node list | grep yolo
ros2 topic info /perception/shuttle_detections_2d -v
ros2 topic info /perception/shuttle_detections_3d -v
```

A healthy detector advertises one publisher on each detection topic.

## 4. Run the complete test

Using the training-laptop model directly:

```bash
ros2 launch scrobot_debug yolo_perception_check.launch.py \
  model_path:=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt
```

Or set the model once:

```bash
export SCROBOT_YOLO_MODEL=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt

ros2 launch scrobot_debug yolo_perception_check.launch.py
```

The launch enables the annotated debug image by default.

## 5. Expected terminal output

The monitor reports once per second:

```text
[YOLO CHECK] NODE=OK |
RGB=OK ...Hz |
ALIGNED_DEPTH=OK ...Hz |
INFO=OK |
2D=OK n=... |
3D=OK n=... |
best=score=... xyz=(...,...,...)m range=...m
```

For the default shuttle around 1.00 m ahead, the expected healthy state is:

```text
NODE=OK
RGB=OK
ALIGNED_DEPTH=OK
INFO=OK
2D=OK n>=1
3D=OK n>=1
```

The exact measured range will not be exactly 1.00 m because the output is
camera-relative and the shuttle settles dynamically in Gazebo.

## 6. View the annotated YOLO image

```bash
rqt_image_view
```

Select:

```text
/perception/shuttle_debug/image
```

Expected overlay:

```text
shuttle <confidence> <depth>m
```

## 7. Topic checks

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw

ros2 topic hz /perception/shuttle_detections_2d
ros2 topic hz /perception/shuttle_detections_3d

ros2 topic echo /perception/shuttle_detections_3d
```

Inspect publisher/subscriber state:

```bash
ros2 topic info /perception/shuttle_detections_2d -v
ros2 topic info /perception/shuttle_detections_3d -v
```

If the detector is alive, both detection topics should have one publisher even
when zero shuttle boxes are currently detected.

## 8. Spawn and despawn shuttles without restarting Gazebo

The default shuttle created by `yolo_perception_check.launch.py` is named:

```text
singleyolo_check
```

### Delete the default shuttle

```bash
gz service \
  -s /world/badminton_court/remove/blocking \
  --reqtype gz.msgs.Entity \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'name: "singleyolo_check", type: MODEL'
```

After deletion, the detector should settle to:

```text
2D n=0
3D n=0
```

### Spawn the default shuttle again at 1.00 m

```bash
ros2 run scrobot_simulation spawn_shuttles \
  --mode single \
  --world badminton_court \
  --name singleyolo_check \
  --x 1.00 \
  --y 0.00
```

### Test at 0.60 m

```bash
ros2 run scrobot_simulation spawn_shuttles \
  --mode single \
  --world badminton_court \
  --name singleyolo_check \
  --x 0.60 \
  --y 0.00
```

### Test at 1.20 m

```bash
ros2 run scrobot_simulation spawn_shuttles \
  --mode single \
  --world badminton_court \
  --name singleyolo_check \
  --x 1.20 \
  --y 0.00
```

### Test at 1.50 m

```bash
ros2 run scrobot_simulation spawn_shuttles \
  --mode single \
  --world badminton_court \
  --name singleyolo_check \
  --x 1.50 \
  --y 0.00
```

### Test an off-center shuttle

```bash
ros2 run scrobot_simulation spawn_shuttles \
  --mode single \
  --world badminton_court \
  --name singleyolo_check \
  --x 1.20 \
  --y 0.30
```

Delete the current model before spawning another model with the same name.

Recommended first distance sweep:

```text
0.50 m
0.60 m
0.80 m
1.00 m
1.20 m
1.40 m
1.60 m
1.68 m
```

Recommended first lateral sweep:

```text
y = -0.40 m ... +0.40 m
```

The configured YOLO/depth acceptance range is camera-relative 0.50-1.68 m.

## 9. Spawn several independent shuttle models

Use unique model names:

```bash
ros2 run scrobot_simulation spawn_shuttles \
  --mode single \
  --world badminton_court \
  --name test_shuttle_01 \
  --x 0.80 --y -0.20

ros2 run scrobot_simulation spawn_shuttles \
  --mode single \
  --world badminton_court \
  --name test_shuttle_02 \
  --x 1.20 --y 0.00

ros2 run scrobot_simulation spawn_shuttles \
  --mode single \
  --world badminton_court \
  --name test_shuttle_03 \
  --x 1.55 --y 0.25
```

Delete one selected shuttle without stopping Gazebo:

```bash
gz service \
  -s /world/badminton_court/remove/blocking \
  --reqtype gz.msgs.Entity \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'name: "test_shuttle_02", type: MODEL'
```

Delete the other names by replacing `test_shuttle_02` with the desired model
name.

## 10. Launch directly at a different initial target

Instead of respawning manually:

```bash
ros2 launch scrobot_debug yolo_perception_check.launch.py \
  model_path:=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt \
  shuttle_x:=1.40 \
  shuttle_y:=0.20
```

## 11. Timing defaults

```text
robot spawn       simulation.launch.py default timing
shuttle spawn     4 s
YOLO start        5 s
monitor start     6 s
```

These delays keep startup logs readable. The detector itself tolerates the
camera streams becoming ready later.

## 12. Pass criteria

A functional pass requires:

1. `NODE=OK`.
2. RGB is continuously received.
3. aligned depth is continuously received.
4. color CameraInfo is valid.
5. the shuttle appears in the annotated debug image.
6. at least one Detection2D is produced while a visible shuttle is present.
7. at least one Detection3D is produced while valid depth is available.
8. reported Z/range is physically plausible for the shuttle position.
9. deleting the shuttle causes detections to disappear.
10. respawning the shuttle causes detections to return without restarting Gazebo.

This check intentionally does not launch the optional shuttle tracker or mission
controller. It validates the production detector boundary first.
