# SCROBOT YOLO Lab — all stages in one repository

Standalone, localhost-only shuttlecock detection laboratory with **five
tabs**: Camera, Scene, Dataset & Labels, Training, and Compare Models.

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
| YOLO tabs | `src/scrobot_simulation/config/yolo_lab.yaml` | Scene spawning, dataset capture, training, validation, and live inference settings, saved independently by stage |

**Save Camera Lab YAML** and the YOLO per-tab **Save settings** actions are distinct.
A scene/capture is generated from the saved YAMLs, **not unsaved sliders**.
This does not change production perception YAML or deployed model weights.
If another lab saves lighting or camera settings, restart the YOLO scene or
start a fresh dataset to take the new snapshot.

## The five tabs

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

**1 · Dataset & Labels.** Launches a fresh triggered-sensor Gazebo world
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

**Dataset QA in the same tab.** Runs the migrated
`scripts/validate_generated_dataset.py` via a dedicated background process.
Checks image-label pairing, all class IDs and normalized boxes, empty
negative labels, duplicate SHA1 hashes, old camera pose metadata and
train/val/test split sizes. Renders several **actual bounding-box overlay
images**, with Previous/Next controls. Training is enabled by the
backend only after inspection passes and the dataset has a complete
train/val split. This is structural QA only: exact mesh projections
can still include occluded/truncated geometry, which should be
visually reviewed.

**2 · Training.** Runs the migrated Ultralytics
`scripts/train_finetune.py`; uses saved epochs/batch/imgsz and an
explicitly selected model checkpoint. Default training recipe uses
AdamW with a user-configurable learning rate (default 0.0001) and patience 8. It saves raw experiments
under `tools/yolo_lab/runs/detect/<run_name>/` and copies the
best checkpoint to `tools/yolo_lab/artifacts/models/<run_name>.pt`,
updating `artifacts/latest_model.txt`. Training stops Gazebo to avoid
competing GPU load. Use **Stop training** to interrupt an active run.
Monitor epochs, recall and mAP from the actual `results.csv`.

Each workflow stage now has its own controls and **Save settings** action,
all persisted in the same YOLO Lab YAML without overriding the other
stages' saved parameters:

- **Scene:** spawn distribution, robot pose and shuttle settling
- **Dataset & Labels:** target images and positive camera-pose fraction
- **Training:** epochs, training batch size and training input size
- **Compare Models:** separate validation batch/input size and live confidence/input size

Previously saved YOLO YAMLs are accepted. Missing validation/live
parameters receive their current defaults when saved.

The Dataset & Labels tab supports checkbox selection, readable suffixes and
confirmed batch deletion. Historical training logs are not rewritten.

## Model selection and management

The **Starting checkpoint** picker automatically discovers local `.pt`
files under `models/pretrains/` (user-renamed originals), the legacy
`models/pretrained/`, `models/trained/`, `artifacts/models/`, and
`runs/detect/*/weights/best.pt`. No import-button or original filename is
required. **All contents of the pretrains/pretrained folders are immutable
from the dashboard**; original checkpoints are never renamed/deleted.
Training-run checkpoints are likewise read-only.

For user-trained checkpoints stored under `models/trained/` or
`artifacts/models/`, check their boxes under **Manage user-trained models
and artifacts** to rename one or delete several at once. A browser
confirmation is required before deletion. If an artifact is renamed,
`artifacts/latest_model.txt` is updated when it points to that file.

### Dataset names and management

New dataset folders use the **local Vietnam calendar day and a per-day
index**, e.g. `Nov14_01`, `Nov14_02`. Existing timestamp-style folders
remain supported. A renamed dataset gets a readable suffix such as
`Nov14_01__evening_720p`. Use checkboxes in **Dataset & Labels** to
rename one or permanently delete one or more. Confirmation is required,
and actions are blocked during data capture and training/evaluation.

## 3 · Compare Models

The former Validation and Live Detection tabs are consolidated. Pick two
different checkpoints A/B from local models.

- **Run A/B validation** evaluates A then B *sequentially* with the same
  inspected dataset, split, resolution, batch and CPU/GPU device. Both
  reports are stored separately in `evaluation_results/yolo_lab/compare/`,
  and the page renders a bar plot of precision, recall, mAP@50 and
  mAP@50–95, plus side-by-side measured metrics.
- **Start side-by-side live detection** runs two independent inference
  processes subscribed to the same `/camera_lab/color` ROS stream, up to
  8 processed FPS per model. Their annotated frames, frame counts,
  inference latency and throughput are shown side by side. Frames are
  **not guaranteed to be timestamp-synchronized**. This is a qualitative
  live comparison; use held-out dataset metrics for fair accuracy ranking.
- **Stop comparison** stops active A/B evaluation or inference. Running
  both detectors concurrently costs more GPU memory than a single model.

Validation and live inference keep separate tunable image sizes, with
shared saved YOLO Lab YAML and stage-specific Save buttons.

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


## Live label preview and early finish

Dataset capture writes the training JPEG, YOLO text labels, and metadata row
before generating an **independent, small annotated JPEG** in
`<dataset>/capture_preview/latest.jpg` (with `latest.json` metadata).
It is not part of the train/val/test data and is not a detector prediction.
The Dataset & Labels tab polls it to show the most recently saved frame,
with geometric ground-truth boxes or a NEGATIVE marker for empty labels.

**Finish early & keep dataset** writes `.finish_early.request` to the
active dataset directory. The capture ROS node notices this request at the
next capture timer step, finalizes `dataset_summary.json`, and exits without
discarding the completed image/label pairs. A partial dataset becomes
eligible for structural inspection when it contains matched images and labels
and has nonempty training and validation splits. Run **Validate labels &
generate previews** in the same tab to approve it before training.

The capture diagnostics show elapsed times for the Gazebo pose request,
image triggering, waiting for a changed and repeated-stable RGB render,
mesh-projected labels, JPEG/metadata writing, and live preview creation.
The changed/stable CRC safeguard remains active to avoid stale-image labels.
For faster collection, start by reducing RGB resolution and keeping Gazebo
GUI closed. Dataset originals are still saved with JPEG quality 95; a lower
quality preset could reduce disk/encoding time, but should be validated
against small-object label quality before adopting.
