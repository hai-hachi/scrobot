# scrobot_perception

Production RGB-D and obstacle perception for SC Robot.

## Production paths

Shuttle detection:

```text
D435i rectified RGB
  -> YOLO bbox
  + aligned depth-to-color
  + color CameraInfo
  -> robust inner-bbox depth
  -> deproject in camera_color_optical_frame
  -> /perception/shuttle_detections_3d
  -> scrobot_mission/shuttle_collection_filter
```

Obstacle perception:

```text
depth PointCloud2
  -> pointcloud_to_laserscan
  -> /camera/camera/depth/scan_raw
  -> depth_scan_self_filter
  -> /camera/camera/depth/scan
  -> collision monitor + Nav2
```

AprilTag perception:

```text
rectified RGB + color CameraInfo
  -> apriltag_ros
  -> /apriltag/detections
  -> scrobot_localization
```

## YOLO detector

The trained model is external to this repository. Configure it with either
`model_path` or `SCROBOT_YOLO_MODEL`.

Current detector defaults:

```text
imgsz                  960
confidence threshold   0.10
IoU threshold          0.70
max detection rate     15 Hz
camera-depth validity  0.20-3.00 m
depth ROI scale        0.40
depth percentile       25%
```

The 0.20-3.00 m interval is only a sensor-depth validity gate. Mission
collection range is evaluated later in `base_link` and is 0.50-1.80 m.

Simulation validation measured about 0.03 m lateral error at approximately
+/-26.7 deg bearing; this is accepted for the 0.30 m collector width.

## Launch

Combined production perception:

```bash
ros2 launch scrobot_perception perception.launch.py \
  enable_yolo:=true \
  model_path:=/path/to/shuttle_yolo.pt
```

## Files

- `scripts/yolo_shuttle_detector.py`
- `scripts/depth_scan_self_filter.py`
- `config/yolo_shuttle_detector.yaml`
- `config/perception.yaml`
- `launch/perception.launch.py`

Regression and dataset procedures:
`../scrobot_debug/debug_md/scrobot_perception/README.md`.
