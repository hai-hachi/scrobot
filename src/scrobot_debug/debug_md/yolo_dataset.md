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

## One-command automatic generation

Make sure the training repository exists first:

```bash
cd ~/Desktop
git clone https://github.com/hai-hachi/yoloshuttle.git
```

Then run exactly one command:

```bash
ros2 launch scrobot_debug yolo_dataset_capture.launch.py
```

Defaults:

```text
output_dir              ~/Desktop/yoloshuttle/dataset/gazebo_scrobot
target_images           1200
positive_pose_fraction  0.85
shuttle_mode            mixed
shuttle_count           50
settle_time             0.40 s
random_seed             42
```

Nothing else is required. Do not run manual teleop.

The launch automatically:

1. starts Gazebo and the simulated D435i;
2. spawns a randomized mixed shuttle distribution;
3. loads the exact rendered `shuttle.STL`;
4. teleports the robot to randomized safe court poses;
5. biases most poses around a randomly selected shuttle so its camera-relative
   range is approximately 0.50-1.68 m;
6. applies random heading jitter so shuttles appear across the RGB image rather
   than only in the center;
7. uses a smaller fraction of random court poses for background/negative views;
8. waits for a fresh settled camera frame after every teleport;
9. projects every visible shuttle mesh through the live RGB `CameraInfo`;
10. writes the RGB image and YOLO label;
11. assigns train/val/test by robot position + heading group;
12. repeats until `target_images` images have been saved;
13. exits the recorder and automatically shuts the launch down.

The output is therefore ready for training after the one launch command returns.

### Generate a different-size dataset

```bash
ros2 launch scrobot_debug yolo_dataset_capture.launch.py \
  target_images:=2000
```

### Generate a different randomized viewpoint sequence

```bash
ros2 launch scrobot_debug yolo_dataset_capture.launch.py \
  target_images:=1200 \
  random_seed:=123
```

Repeated runs are safe. Each run gets a timestamp session prefix, so previous
images are not overwritten.

The initial shuttle layout is randomized by the simulation shuttle spawner.
Robot viewpoints are independently randomized by the capture node.

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
