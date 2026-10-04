# Synthetic YOLO dataset generation

This workflow now uses a dedicated lightweight RGB-only Gazebo renderer under
`scrobot_debug`. It does not launch the production robot, ros2_control, depth,
IMU, EKF, collision monitor, shuttle manager, or shuttle ground-truth bridge.

## Fast pipeline

```text
debug-only court world
  + same green mat / lines / net / AprilTags / lighting
  + RGB-only D435i camera rig
  + static visual-only shuttle STL models
        ↓
random camera viewpoint
        ↓
wait for first fresh RGB frame
        ↓
project known shuttle STL poses through live CameraInfo
        ↓
save JPG + YOLO TXT
        ↓
repeat until target_images
        ↓
automatic shutdown
```

The camera rig keeps the SCROBOT RGB viewpoint:

- 1280x720;
- HFOV 1.229041640167034 rad;
- camera color-frame height 0.28683059 m above court;
- 15 deg downward pitch;
- same `camera_color_frame -> camera_color_optical_frame` convention.

The shuttle model uses the exact production `shuttle.STL`, but is static,
visual-only, and has no collision or physics.

## Output

Default destination:

```text
~/Desktop/yoloshuttle/dataset/gazebo_scrobot/
├── data.yaml
├── metadata.csv
├── images/
│   ├── train/
│   ├── val/
│   └── test/
└── labels/
    ├── train/
    ├── val/
    └── test/
```

## Build

```bash
cd ~/scrobot_ws
git checkout simulation-systematic-test2
git pull

colcon build --symlink-install --packages-up-to scrobot_debug
source install/setup.bash
```

## One command

Make sure the training repo exists:

```bash
cd ~/Desktop
git clone https://github.com/hai-hachi/yoloshuttle.git
```

Then:

```bash
ros2 launch scrobot_debug yolo_dataset_capture.launch.py
```

Defaults:

```text
target_images           1200
shuttle_count           40
positive_pose_fraction  0.85
random_seed             42
```

The launch is server-only/headless and exits automatically when the dataset is
complete.

### Larger dataset

```bash
ros2 launch scrobot_debug yolo_dataset_capture.launch.py \
  target_images:=2000
```

### Different randomized scene/view sequence

```bash
ros2 launch scrobot_debug yolo_dataset_capture.launch.py \
  target_images:=1200 \
  random_seed:=123
```

## Progress

```bash
ros2 topic echo /debug/yolo_dataset_status
```

Expected status includes:

```text
mode=fast
state=...
saved_images
target_images
progress_pct
pose_attempts
shuttle_count
last_boxes
last_focus_boxes
```

Common states:

```text
WAITING_FOR_CAMERA
SPAWNING_STATIC_SHUTTLES
STATIC_SCENE_READY
TELEPORTING_RGB_RIG
WAITING_FOR_FRESH_RGB
SAVING_FRAME
FRAME_SAVED
TARGET_VIEW_REJECTED
NEGATIVE_VIEW_SKIPPED
COMPLETE
```

## Dataset behavior

The static shuttle layout is randomized once per run. Each shuttle gets a
random court position/orientation, with a mix of uniform and clustered
placement.

For each image, the RGB camera pose is randomized:

- most views are biased toward a shuttle at approximately 0.50-1.68 m
  camera-relative range;
- heading jitter moves the target across the RGB image rather than keeping it
  centered;
- a smaller fraction of views are random court/background views;
- negative frames are subsampled;
- all visible projected shuttles are labeled, including those outside the
  preferred focus range.

Train/val/test assignment is grouped by quantized camera position and heading
rather than individual frame order.

## Label generation

No manual synthetic labeling is required.

For each static shuttle:

1. load the exact production STL vertices;
2. apply the known shuttle world pose;
3. transform into `camera_color_optical_frame`;
4. project with the live RGB `CameraInfo` matrix;
5. clip the projected box to the 1280x720 image;
6. write normalized YOLO `class x_center y_center width height`.

Class:

```text
0: Shuttlecock
```

## Important limitation

Labels are geometric projections of the full STL. The generator does not
perform pixel-level occlusion reasoning. Clustered shuttles are useful as hard
examples, but the dataset should not be dominated by severe overlap.

## Inspect counts

```bash
for s in train val test; do
  echo "$s images: $(find ~/Desktop/yoloshuttle/dataset/gazebo_scrobot/images/$s -type f | wc -l)"
  echo "$s labels: $(find ~/Desktop/yoloshuttle/dataset/gazebo_scrobot/labels/$s -type f | wc -l)"
done
```

## CUDA smoke

```bash
cd ~/Desktop/yoloshuttle
bash ubuntu_cuda_smoke.sh dataset/gazebo_scrobot/data.yaml
```
