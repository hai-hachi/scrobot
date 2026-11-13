# SCROBOT YOLO Lab — all stages in one repository

Standalone, localhost-only shuttlecock detection laboratory with **seven
tabs**: Camera, Scene, Dataset Generator, Label Inspector, Training,
Validation, and Live Detection.

Old research code is consolidated here from
[hai-hachi/yoloshuttle](https://github.com/hai-hachi/yoloshuttle)
and `scrobot_debug`, without requiring a second source checkout.

## Launch

```bash
cd ~/scrobot_ws
git switch real-comparison
git pull --ff-only
bash tools/yolo_lab/start.sh
```

Dashboard: **http://127.0.0.1:8770**

The server and Gazebo use ROS domain **83** and isolated Gazebo partition
`scrobot_yolo_lab_<uid>_83`. Gazebo runs headless until requested via
**Open Gazebo UI**, which attaches to the existing server.

### Dependencies

- Gazebo Harmonic and ROS 2 Jazzy with `ros_gz_image`,
  `ros_gz_bridge`, and `ros_gz_sim`.
- `PyYAML`, `numpy`, `opencv-python`, `ultralytics`, `torch`, and
  `matplotlib` for the full detector/training pipeline.
- `python3 -m pip install -r tools/yolo_lab/requirements.txt` in your
  intended environment.
- When using a separate Python environment for training, set
  `SCROBOT_YOLO_PYTHON=/absolute/path/to/venv/bin/python` before
  launching the lab. The live ROS inference node and Gazebo collector
  use the Python interpreter running the lab, which must also import
  `rclpy` and `ultralytics`. A venv created with
  `python3 -m venv --system-site-packages .venv` can expose system ROS
  Jazzy packages if versions are compatible.
- Existing compiled `scrobot_simulation` and `ros_gz_sim` packages
  must be installed for Gazebo to resolve models and `ros2 run`.
- Use `bash tools/yolo_lab/start.sh`, **not** `./start.sh`: repository
  API writes do not necessarily preserve executable flags.

## Ownership and shared YAML

| Owner | Saved source | Consumer |
|---|---|---|
| Camera tab | `src/scrobot_simulation/config/camera_lab.yaml` | All YOLO RGB/depth modes, camera scene and capture sensor |
| Lighting Lab | `src/scrobot_simulation/config/lighting_lab.yaml` | YOLO court lights, shadows, HSV materials and venue |
| YOLO Scene tab | `src/scrobot_simulation/config/yolo_lab.yaml` | Scene shuttles/seed, dataset size, training params, YOLO confidence |

**Save Camera Lab YAML** and **Save YOLO Lab YAML** are distinct actions.
A scene/capture is generated from the saved YAMLs, **not unsaved sliders**.
This does not change production perception YAML or deployed model weights.
If another lab saves lighting or camera settings, restart the YOLO scene or
start a fresh dataset to take the new snapshot.

## The seven tabs

**Camera.** Reuses the same D435i RGB/depth mode table and validator as
`tools/camera_lab/modes.py`. Measures raw ROS wall FPS, timestamp/sim FPS,
and image throughput *after* the `ros_gz_image` bridge.

**Scene.** Generates a full Lighting Lab-calibrated court and deterministic
production shuttle models using their real collision meshes. The scene
supports single, random (uniform/center/net), seven-line weighted clusters
(14 independent left/right weights), and mixed distributions. After an
adjustable dynamic interval, Gazebo poses must stabilize; the world is then
reloaded with all shuttle models static at their actual settled 3D poses.
If pose collection or stability checks fail, the scene stays dynamic and
shows a settling error rather than inventing static poses. The overhead
robot pose selector supports click-to-place and drag-to-set-yaw, and the
camera uses the real robot camera mount transform. It is only
a scene preview, **not an automatically labelled dataset**.

**1 · Dataset Generator.** Launches a fresh triggered-sensor Gazebo world
and the migrated `scripts/dataset_capture.py`, which previously lived
under `scrobot_debug`. It generates `dataset_<UTC timestamp>` under
`evaluation_results/yolo_lab/datasets/`; datasets are never silently
overwritten. The captured world has camera resolution and rate from the
saved Camera YAML, calibrated Lighting Lab court visuals, and static
shuttles spawned from the exact production shuttle mesh. The collector
uses actual Gazebo camera intrinsics, projection of the exact shuttle
STL vertices, and a **changed-render then stable-render** check after
every camera teleport before saving a frame and its YOLO label.
The captured frames include valid empty-label negative examples.

Data layout:

```text
evaluation_results/yolo_lab/datasets/dataset_<timestamp>/
    data.yaml
    metadata.csv                   # Pose, split, group, image CRC
    dataset_summary.json
    images/{train,val,test}/*.jpg
    labels/{train,val,test}/*.txt
    preview/                       # Made by Label Inspector
    inspection.json                # Passed / failed QA report
```

Train/val/test splitting uses a deterministic hash of camera position and
yaw groups (~70/15/15), not naive consecutive frame splitting. This
reduces viewpoint leakage and exact duplicates.

**2 · Label Inspector.** Runs the migrated
`scripts/validate_generated_dataset.py` via a dedicated background process.
Checks image-label pairing, all class IDs and normalized boxes, empty
negative labels, duplicate SHA1 hashes, old camera pose metadata and
train/val/test split sizes. Renders several **actual bounding-box overlay
images**, with Previous/Next controls. Training is enabled by the
backend only after inspection passes and the dataset has a complete
train/val split. This is structural QA only: exact mesh projections
can still include occluded/truncated geometry, which should be
visually reviewed.

**3 · Training.** Runs the migrated Ultralytics
`scripts/train_finetune.py`; uses saved epochs/batch/imgsz and an
explicitly selected model checkpoint. Default training recipe uses
AdamW with learning rate 0.0001 and patience 8. It saves raw experiments
under `tools/yolo_lab/runs/detect/<run_name>/` and copies the
best checkpoint to `tools/yolo_lab/artifacts/models/<run_name>.pt`,
updating `artifacts/latest_model.txt`. Training stops Gazebo to avoid
competing GPU load. Use **Stop training** to interrupt an active run.
Monitor epochs, recall and mAP from the actual `results.csv`.

The original `best.pt`, `yolo11n_shuttle.pt` and `yolo11s-ball.pt`
are **binary** weights, not Python code. They are intentionally not
duplicated as large Git objects in `scrobot`. Click **Import verified
baseline weights** to download them from the original repository into
`tools/yolo_lab/models/pretrained/`, checking the exact Git blob
SHA-1 and byte size. This requires network access. Alternatively
copy your original `.pt` files there manually. Neither action
modifies the normal robot's deployed weights.

**4 · Validation.** Runs the migrated `scripts/validate_model.py` on
the same inspected dataset using either validation or held-out
test split. The modified script exports *actual* measured precision,
recall, mAP@50, mAP@75, mAP@50–95, and per-image speed to
`evaluation_results/yolo_lab/validation_metrics.json`.

**5 · Live Detection.** Uses the selected trained/baseline `.pt` with
real Ultralytics predictions on the simulated RGB topic. Its ROS
subscriber runs in a **separate** Python process; it publishes
thumbnail JPEGs with detected bounding boxes and records inference
latency, processed FPS, count and confidence. Raw camera FPS remains
independently measured by the Camera tab. Inference is intentionally
limited to at most 15 FPS; this is not a camera FPS benchmark.
Starting live detection will create a continuous-stream inspection
scene if a triggered dataset scene was running.

## Original source migration map

The authoritative *active* sources are under `tools/yolo_lab/`:

| Old source | Migrated location |
|---|---|
| `yoloshuttle/scripts/train_finetune.py` | `scripts/train_finetune.py` |
| `yoloshuttle/scripts/validate_model.py` | `scripts/validate_model.py` |
| `yoloshuttle/scripts/validate_generated_dataset.py` | `scripts/validate_generated_dataset.py` |
| `yoloshuttle/scripts/test_shuttle.py` | `scripts/test_shuttle.py` |
| `yoloshuttle/scripts/plot_metrics.py` | `scripts/plot_metrics.py` |
| `yoloshuttle/scripts/download_original_dataset.py` | `scripts/download_original_dataset.py` |
| `yoloshuttle/scripts/ubuntu_cuda_smoke.sh` | `scripts/ubuntu_cuda_smoke.sh` |
| `yoloshuttle/docs/SCROBOT_YOLO_Training_Crash_Course.md` | `docs/SCROBOT_YOLO_Training_Crash_Course.md` |
| `src/scrobot_debug/scripts/yolo_dataset_fast_capture.py` | `scripts/dataset_capture.py` |
| `src/scrobot_debug/models/yolo_*/*` | `models/yolo_*/*` |
| `src/scrobot_debug/config/yolo_dataset_bridge.yaml` | `assets/yolo_dataset_bridge.yaml` |

The historical YOLO-specific debug monitors and launch scenarios also
have copies in `tools/yolo_lab/legacy_debug/` for reference. The older
`src/scrobot_debug/` entry points remain available so existing ROS
debug launches are **not** broken. The production ROS node
`src/scrobot_perception/scripts/yolo_shuttle_detector.py` is purposely
left in the perception package because the actual robot's installed
detector launch depends on it; this new lab is not changing that
production launch.

## Acceptance and limits

- The current lab covers 2D detection, datasets and model evaluation.
  It **does not automatically publish model checkpoints to production**.
  Compare new checkpoints to the old `best.pt` on held-out synthetic
  and ultimately real D435i green-court images before deployment.
- Auto-labels are geometric projections of STL vertices; without
  rendered instance/visibility masks, a shuttle blocked by another
  object may receive a loose box. Inspect label previews before
  training and consider instance masks in a later improvement.
- GitHub Actions performs **offline** tests of SDF geometry,
  Camera/Lighting handoffs, CLI wiring, saved datasets and HTML syntax.
  GPU training, Gazebo's triggered sensor, and real ROS topic throughput
  require live verification on the workstation.

## Test

```bash
python3 -m unittest discover -s tools/yolo_lab/tests -p 'test_*.py' -v
```

Generated images, `.pt` imports, run logs and artifacts remain local and
ignored by Git. The YOLO Lab server binds to localhost only.


## Scene debugging and logs

Scene spawn settings are saved in the existing YOLO Lab YAML: mode, shuttle
count, deterministic seed, random density, mixed fraction, and the 14 editable
left/right zone weights. The advanced physical settling interval is also a
YOLO Lab choice. Court length/width/margin, all seven court-line X positions,
cluster sigma X/Y, left/right cluster centers, fixed single-shuttle pose,
and spawn height are read directly from
`src/scrobot_simulation/config/shuttle_spawn.yaml` at scene generation
(and presented read-only in the Scene tab). Changing that source YAML
changes the next scene without copying dimensional values into YOLO YAML.
Previously saved YOLO YAMLs with copied dimension fields are migrated
safely: these legacy keys are ignored, and saving rewrites only the editable
settings. The Scene tab now puts shuttle spawning in one box, with a separate
robot pose map next to the actual robot camera preview.

The browser strips ANSI terminal control sequences and truncated partial
log lines using `tools/log_text.py`, while the original `*.log` files
remain untouched for full diagnostics. The same shared sanitizer is used
by the Camera Lab and the YOLO workflow task logs.

**Caveat:** Static freezing is an intentional Gazebo server reload. The
camera preview briefly disconnects; the Gazebo GUI is reopened if previously
active. Run on the workstation to verify physical settling and camera pose:

```bash
cd ~/scrobot_ws
git switch real-comparison
git pull --ff-only
python3 -m unittest discover -s tools/yolo_lab/tests -p 'test_*.py' -v
bash tools/yolo_lab/start.sh
```
