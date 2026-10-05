# YOLO Perception Check

End-to-end validation of the production shuttle perception path.

## What the launch starts

```text
Gazebo badminton court
        +
SCROBOT simulation
        +
simulated D435i RGB/depth
        +
depth-to-color registration
        +
one dynamic shuttle at (1.00, 0.00) by default
        +
scrobot_perception/yolo_shuttle_detector
        +
scrobot_debug/yolo_perception_monitor
```

The production detector path being tested is:

```text
/camera/camera/color/image_raw
                │
                ▼
          YOLO shuttle bbox
                │
                ├────> /perception/shuttle_detections_2d
                │
/camera/camera/aligned_depth_to_color/image_raw
                │
                ▼
       bbox ROI depth sampling
                │
/camera/camera/color/camera_info
                ▼
        deproject to XYZ
                │
                ▼
/perception/shuttle_detections_3d
```

## Build

```bash
cd ~/scrobot_ws
colcon build --symlink-install \
  --packages-up-to scrobot_debug scrobot_perception
source install/setup.bash
```

## Run

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

## Expected terminal output

The monitor reports once per second:

```text
[YOLO CHECK]
RGB=OK ...Hz
ALIGNED_DEPTH=OK ...Hz
INFO=OK
2D=OK n=...
3D=OK n=...
best=score=... xyz=(...,...,...)m range=...m
```

For the default shuttle around 1.00 m ahead, the expected healthy state is:

```text
RGB=OK
ALIGNED_DEPTH=OK
INFO=OK
2D=OK n>=1
3D=OK n>=1
```

The exact measured range will not be exactly 1.00 m because the coordinates are
camera-relative and the shuttle settles dynamically in Gazebo.

## Debug image

The launch enables the annotated image by default.

```bash
rqt_image_view /perception/shuttle_debug/image
```

Expected overlay:

```text
shuttle <confidence> <depth>m
```

## Useful topic checks

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic hz /perception/shuttle_detections_2d
ros2 topic hz /perception/shuttle_detections_3d

ros2 topic echo /perception/shuttle_detections_3d
```

## Move the test shuttle

```bash
ros2 launch scrobot_debug yolo_perception_check.launch.py \
  model_path=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt \
  shuttle_x:=1.40 \
  shuttle_y:=0.20
```

The detector currently accepts camera-relative depth only from 0.50 m to 1.68 m,
so keep the first validation targets inside that region.

## Timing defaults

```text
robot spawn       simulation.launch.py default timing
shuttle spawn     4 s
YOLO start        5 s
monitor start     6 s
```

These delays keep startup logs readable; the detector itself can tolerate input
topics becoming ready later.

## Pass criteria

A first functional pass requires:

1. RGB is continuously received.
2. aligned depth is continuously received.
3. color CameraInfo is valid.
4. the shuttle appears in the annotated debug image.
5. at least one Detection2D is produced.
6. at least one Detection3D is produced.
7. reported Z/range is physically plausible for the shuttle position.

This check intentionally does not launch the optional shuttle tracker or mission
controller. It validates the production detector boundary first.
