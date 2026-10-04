# Synthetic YOLO dataset generation

This debug workflow creates a YOLO-format shuttlecock dataset directly from the
Gazebo simulation.

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

Each saved RGB image gets a matching YOLO label file.

## Label source

No manual annotation is required for the synthetic dataset.

The recorder uses:

- the same detailed `shuttle.STL` Gazebo renders;
- shuttle world poses from `/evaluation/shuttle_ground_truth`;
- robot world pose from `/evaluation/ground_truth_odom`;
- `base_footprint -> camera_color_optical_frame` TF;
- the live color `CameraInfo` calibration matrix.

All STL vertices are projected into the RGB image and converted into clipped
YOLO bounding boxes.

## Build

```bash
cd ~/scrobot_ws
colcon build --symlink-install --packages-up-to scrobot_debug
source install/setup.bash
```

## One-command capture

Make sure the training repository exists first:

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
output_dir     ~/Desktop/yoloshuttle/dataset/gazebo_scrobot
capture_rate   2 Hz
shuttle_mode   mixed
shuttle_count  40
```

The recorder automatically:

1. launches the SCROBOT simulation;
2. launches local EKF, perception and the production-safe control stack;
3. spawns the shuttle distribution;
4. captures RGB frames;
5. derives shuttle boxes from simulation truth + exact STL geometry;
6. writes labels;
7. assigns train/val/test by robot pose + heading group;
8. retains only a subset of repeated negative frames;
9. appends `metadata.csv`.

## Create viewpoint diversity

In a second terminal:

```bash
source ~/scrobot_ws/install/setup.bash
ros2 run scrobot_control manual_teleop
```

Enter MANUAL:

```text
M
```

Then drive around the court with W/A/S/D.

Do not remain stationary for most of the recording. The split is deliberately
grouped by quantized robot position and heading so adjacent frames from the same
viewpoint go into the same split.

The dataset is most valuable when the robot observes shuttles across:

- approximately 0.50-1.68 m camera-relative range;
- image center and image edges;
- different shuttle orientations;
- isolated and multi-shuttle scenes;
- court lines, net, posts, AprilTags and other negative background features.

The recorder labels every shuttle whose projected visual mesh is in the image,
including visible shuttles outside the preferred 0.50-1.68 m focus range. This
avoids teaching YOLO that a visible shuttle outside the focus range is
background.

## Optional launch overrides

```bash
ros2 launch scrobot_debug yolo_dataset_capture.launch.py \
  capture_rate:=1.0 \
  shuttle_mode:=mixed \
  shuttle_count:=60 \
  output_dir:=$HOME/Desktop/yoloshuttle/dataset/gazebo_scrobot
```

Repeated runs are safe. Each run gets a timestamp session prefix, so previous
images are not overwritten.

## Inspect counts

```bash
for s in train val test; do
  echo "$s images: $(find ~/Desktop/yoloshuttle/dataset/gazebo_scrobot/images/$s -type f | wc -l)"
  echo "$s labels: $(find ~/Desktop/yoloshuttle/dataset/gazebo_scrobot/labels/$s -type f | wc -l)"
done
```

A useful recording should populate all three splits. If one split is empty,
drive through more distinct robot positions/headings and capture another
session.

## Important limitation

The boxes are geometric projections of the full rendered shuttle mesh. They do
not perform pixel-level visibility/occlusion testing. Avoid generating most of
the dataset from heavily overlapping shuttle piles. Use mixed/random scenes for
the majority of training data and treat clustered scenes as a smaller hard-case
subset.

## Train

After capture:

```bash
cd ~/Desktop/yoloshuttle
bash ubuntu_cuda_smoke.sh dataset/gazebo_scrobot/data.yaml
```

If the CUDA smoke test passes, proceed to a longer fine-tune.
