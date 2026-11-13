# SCROBOT YOLO Lab — independent Stage 1

A standalone, localhost-only **YOLO Lab** with a reusable Camera
configuration tab and a shuttle-inspection scene.

```bash
cd ~/scrobot_ws
git switch real-comparison
git pull --ff-only
bash tools/yolo_lab/start.sh
```

Open **http://127.0.0.1:8770**. The dashboard itself does not auto-launch
Opera/Chrome and does not launch Gazebo until **Start / restart YOLO
inspection scene** is pressed.

## Configuration chain and status

- **Camera tab** reuses the existing D435i USB3 RGB/depth resolution and
  frame-rate table and validators from `tools/camera_lab/modes.py`.
  **Save Camera Lab YAML** updates
  `src/scrobot_simulation/config/camera_lab.yaml`; other labs can read
  exactly these settings. This does not publish camera parameters to
  the main simulation.
- **YOLO Lab tab** stores shuttle placement seed, count, initial robot
  pose, and proposed dataset/training parameters in
  `src/scrobot_simulation/config/yolo_lab.yaml`. Its independent
  **Save YOLO Lab YAML** button is required before starting the
  inspection scene. This does **not** change production YOLO models.
- **Lighting Lab** owns
  `src/scrobot_simulation/config/lighting_lab.yaml`.
  The YOLO inspection scene imports its actual calibrated light array,
  shadow-caster allocation, venue clearances, and HSV materials. The
  court geometry, markings, net and safety walls come from the
  canonical production world SDF (then receive the saved Lighting
  Lab overrides).
- The inspection scene uses the **production shuttle STL visual mesh**,
  sampled deterministically across the doubles court. No substitute
  cubes/spheres or fabricated bounding boxes. It has a robot proxy,
  RGB/depth sensors and no Nav2, controller, AprilTag detector or
  YOLO detector yet.
- A **Start** action consumes only the two saved YAMLs and the latest
  saved Lighting YAML. It freezes a manifest
  `evaluation_results/yolo_lab/scene_manifest.json` with camera profile,
  lighting source, and shuttle poses. If any saved settings change,
  restart the scene to apply them.
- Gazebo runs server-only via `gz sim -r -s` on ROS domain 83,
  partition `scrobot_yolo_lab_<uid>_83`. **Open/close Gazebo UI**
  connects `gz sim -g` to the same running server and does not stop
  data transport on GUI closure.
- Actual RGB/depth **Wall FPS**, **Sim FPS** and uncompressed ROS **MB/s**
  are measured by the existing Camera Lab's raw ROS subscriber, after
  `ros_gz_image`. The preview is encoded at approximately 2 FPS by a
  separate process and never republishes all full-resolution frames.

## What was learned from the previous pipeline

The **old `yoloshuttle` repository contains the dataset validator, trainer,
model validator, live camera test and model artifact handling**. It does
**not** contain the Gazebo image generator or automatic label writer.
Those were implemented separately in:

`src/scrobot_debug/scripts/yolo_dataset_fast_capture.py`

1. Generate a static shuttle layout (historically 40 shuttle meshes),
   then sample randomized camera viewpoints. The old default requests
   1,200 images, biasing 85% of viewpoints toward a visible shuttle
   at 0.50–1.68 m, with deterministic seed 42.
2. After moving the camera via `/world/yolo_dataset/set_pose`, wait
   for **actual changed rendered pixels**, then a second stable render,
   before recording the frame. The same pose must supply image and
   label; otherwise the dataset gets fake accuracy from stale RGB.
3. Project the exact production shuttle STL geometry using the Gazebo
   camera's intrinsic matrix. Clip/reject boxes outside the view and
   write YOLO class `0` normalized `x_center y_center width height`
   labels. Empty labels represent negative frames. Save `metadata.csv`
   with poses, group IDs and image CRC32.
4. Split images by **quantized camera position/yaw group**, not naively
   consecutive frames, into train/val/test (roughly 70/15/15).
   This limits near-duplicate viewpoints leaking across splits.
5. Run `yoloshuttle/scripts/validate_generated_dataset.py` to
   check matching image/label pairs, normalized coordinates, exact
   duplicate SHA1 hashes, stale pose metadata, negative frames, and
   render bbox previews to inspect visually. Structural validation
   does not prove label-to-image alignment without preview review.
6. Fine-tune the existing `best.pt` rather than training from scratch.
   The original conservative recipe is **15 epochs / batch 8 /
   imgsz 640 / AdamW / lr0 1e-4 / patience 8**. Optionally continue
   from the resulting model with **40 epochs / imgsz 960**.
   Runs go to `runs/detect/<run_name>`; the best checkpoint is copied
   to `artifacts/models/<run_name>.pt` and indexed in
   `artifacts/latest_model.txt`.
7. Run `yoloshuttle/scripts/validate_model.py` on validation or held-out
   test images. Compare **precision**, **recall**, **mAP@50**,
   **mAP@50–95**, and speed. Finally test the baseline and new model
   on the **same real D435i badminton-court images**, comparing misses,
   false positives, detection range and frame-to-frame stability.

See [original repository](https://github.com/hai-hachi/yoloshuttle)
and [label capture source](https://github.com/hai-hachi/scrobot/blob/real-comparison/src/scrobot_debug/scripts/yolo_dataset_fast_capture.py).

### Intentionally NOT yet implemented

The new YOLO Lab **does not yet launch** the automated dataset collector,
model trainer, automatic 2D bbox predictions, or validation metrics.
This first step establishes a reproducible and independently measurable
scene and documents the actual old methods before integrating the
expensive pipeline. The GUI does not present nonfunctional Train /
Dataset buttons. The older `tools/perception_labs` and
`src/scrobot_debug` remain unmodified by this new lab.

## Run unit tests

```bash
python3 -m unittest discover -s tools/yolo_lab/tests -p 'test_*.py' -v
```

Only stdlib + PyYAML are required for offline world generation.
Live Gazebo requires ROS 2 Jazzy, Gazebo Harmonic, cv2 and numpy.

## Safety & reproducibility

All files generated from simulation live under
`evaluation_results/yolo_lab/`, apart from explicit lab YAML saves.
The generated scene is labelled `dataset_labels_created: false`; it
must **not** be mistaken for a ground-truth labelled dataset. Main
simulation camera, perception, localization and world files are untouched.
