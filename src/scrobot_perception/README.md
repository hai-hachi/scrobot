# scrobot_perception

Perception package for SCROBOT.

## Production shuttle pipeline

The production shuttle detector is:

```text
D435i rectified RGB
        │
        ▼
YOLO shuttle detector
        │
        ├── /perception/shuttle_detections_2d
        │
aligned depth-to-color image
        │
        ▼
robust bbox ROI depth
        │
color CameraInfo intrinsics
        ▼
camera-relative XYZ
        │
        ▼
/perception/shuttle_detections_3d
        │
        ▼
mission / local collection controller
```

The optional `shuttle_tracker` is not required by the production mission.
The `fake_shuttle_detector` remains simulation/testing infrastructure only.

## Current model

The current pinned training candidate is:

```text
gazebo_simple_v2.pt
```

It is intentionally **not stored in the scrobot Git repository**.

On the training laptop it normally lives at:

```text
~/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt
```

For production on the Jetson, use a stable deployment path such as:

```text
/opt/scrobot/models/shuttle_yolo.pt
```

The detector accepts the model through either:

```text
model_path:=/path/to/model.pt
```

or:

```bash
export SCROBOT_YOLO_MODEL=/path/to/model.pt
```

## Dependencies

ROS dependencies are declared in `package.xml`.

The detector also needs the Python Ultralytics package in the Python
environment used by ROS:

```bash
python3 -m pip install ultralytics
```

For Jetson deployment, verify the installed PyTorch build has CUDA support
before launching the node.

## Topics

Inputs:

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

Outputs:

```text
/perception/shuttle_detections_2d
/perception/shuttle_detections_3d
/perception/shuttle_debug/image     # optional
```

The 3D output is expressed in the color optical frame because depth is aligned
to RGB and the deprojection uses the color-camera intrinsic matrix.

## Default detector settings

```text
imgsz                 = 960
confidence_threshold  = 0.10
iou_threshold         = 0.70
max_detection_rate    = 15 Hz

camera depth validity = 0.20-3.00 m

depth ROI scale        = 0.40
depth percentile      = 25%
```

The low confidence threshold intentionally favors recall.

The detector's 0.20-3.00 m range is only a broad validity gate on aligned
camera depth. It is deliberately not the collection range.

Mission-side shuttle eligibility is evaluated later after transforming each 3D
detection into `base_link`:

```text
0.50 m <= planar base_link range <= 1.80 m
```

This separation prevents the camera-frame depth sampler from silently deciding
a mission-level collection condition.

## Run on the training laptop / simulation

Make sure RGB, color CameraInfo, and aligned depth are already being published.

```bash
source ~/scrobot_ws/install/setup.bash

ros2 launch scrobot_perception yolo_shuttle_detector.launch.py \
  use_sim_time:=true \
  model_path:=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt \
  device:=0 \
  publish_debug_image:=true
```

Inspect:

```bash
ros2 topic hz /perception/shuttle_detections_3d
ros2 topic echo /perception/shuttle_detections_3d
```

Debug RGB:

```bash
rqt_image_view /perception/shuttle_debug/image
```

## Run on Jetson Orin

Recommended model deployment:

```bash
sudo mkdir -p /opt/scrobot/models
sudo cp gazebo_simple_v2.pt /opt/scrobot/models/shuttle_yolo.pt
```

Then:

```bash
export SCROBOT_YOLO_MODEL=/opt/scrobot/models/shuttle_yolo.pt

ros2 launch scrobot_perception yolo_shuttle_detector.launch.py \
  use_sim_time:=false \
  device:=0
```

## Depth behavior

YOLO only supplies the 2D box.

For each box the detector:

1. takes the inner portion of the bbox,
2. removes invalid/non-finite depth and values outside the broad camera-depth
   validity interval,
3. uses a foreground-biased depth percentile,
4. deprojects the bbox center using the color CameraInfo matrix.

The resulting point remains in the color optical frame. Mission collection
range filtering is intentionally deferred until TF can express that point in
`base_link`.

This is more robust for a thin shuttlecock than trusting one center pixel.
