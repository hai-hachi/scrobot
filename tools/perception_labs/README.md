# SCROBOT Perception Labs v2 — clean rebuild

The previous Perception Lab was reverted. This rebuild launches a minimal
Gazebo **server first**, checks the actual world service, then starts ROS
image bridges and a separate monitor. The GUI is launched only when asked.

## One-command launch

The Git checkout is the colcon workspace: the tools folder is
`~/scrobot_ws/tools` and ROS packages are `~/scrobot_ws/src`.

```bash
cd ~/scrobot_ws
git switch real-comparison
git pull --ff-only
bash tools/perception_labs/start.sh --build
```

Open **http://127.0.0.1:8768**. Subsequent launches:

```bash
bash ~/scrobot_ws/tools/perception_labs/start.sh
```

No Gazebo process starts automatically. This avoids hiding startup errors
behind an apparently healthy web server.

## Mandatory first check: Minimal smoke test

1. Click **1 · Minimal smoke test**. The scene has one 640×480 RGB camera
   at 15 FPS, a floor and a static robot proxy, with no depth, AprilTag,
   motor controller, collection manager, or full production court.
2. Observe the launch stages:
   **generating_world → waiting_for_world → waiting_for_rgb → streaming**.
   A running process PID alone does not count as success.
3. Click **Open Gazebo GUI** after the world service exists. If the server
   streams but no GUI appears, inspect `gui.log` separately.
4. Only when this succeeds, select **Camera performance** and click
   **2 · Start selected lab**. This replaces the smoke world with the
   published badminton court and lighting.

Log files live in
`~/scrobot_ws/evaluation_results/perception_labs/`. The dashboard exposes
Gazebo, bridges, monitor, detector and GUI logs even if a process exits.

Only this lab's process groups are stopped. It uses ROS domain **79**,
an isolated Gazebo partition and WARP-safe `GZ_IP=127.0.0.1`.

## Camera Performance

RGB and depth resolutions/FPS are selectable. The lab reports requested
vs actual simulation-time and wall-time frame rates, message age, jitter,
and effective real-time factor (RTF = wall FPS / sim FPS).

A camera producing 15 FPS in simulation but only 7.5 FPS in wall time
has an effective RTF of approximately 0.5; it is not necessarily dropping
frames. Rendering and transport cannot be conclusively separated without
additional Gazebo-level instrumentation.

The court scene copies static geometry and **exact current published lighting
from badminton_court.sdf**, not unsaved Lighting Lab sliders. It does not
start full robot physics, Nav2, controllers, or collection.

## AprilTag

Select **AprilTag**, start the lab, position and rotate the static robot
proxy. The production `apriltag_ros` detector receives RGB and CameraInfo.
The dashboard reports measured tag output rate, IDs, and decision margins
for existing 16h5 court tags 0–3.

This initial rebuilt version deliberately does **not** claim reliable
absolute pose RMSE until the detector/camera optical-frame convention has
been checked with actual Gazebo data. It does not change `map→odom`.

## Shuttle YOLO

Select **Shuttle YOLO**, choose local weights (blank = latest/best from
`yoloshuttle`), then start. Position/rotate the robot proxy, spawn/move
the production shuttle model, and view the genuine detector bounding boxes
and confidence.

In this lab's YOLO mode, simulated depth and RGB are **co-located and
pixel-aligned** (matching resolutions, FPS and FOV). This is a simplified
test stream, **not** the physical D435i stereo baseline. It permits a
clean 2D detection test with aligned depth sampling.

### Dataset/training pipeline reused from your yoloshuttle repo

This lab calls the existing `yolo_dataset_fast_capture` and your
`hai-hachi/yoloshuttle` training and validation scripts instead of
duplicating them. Provide a local checkout:

```bash
git clone https://github.com/hai-hachi/yoloshuttle.git ~/Desktop/yoloshuttle
cd ~/Desktop/yoloshuttle
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Or export `SCROBOT_YOLO_REPO=/path/to/yoloshuttle`.

The four buttons run these local tasks:

1. **Generate:** new headless scene using current published lighting and
   static court, retaining the existing triggered camera and CRC-based
   stale-frame checks. Records SHA256 scene provenance in a new dataset.
2. **Check labels:** runs `validate_generated_dataset.py` and creates
   labeled bbox previews.
3. **Fine-tune:** runs `train_finetune.py` only after dataset capture
   reports completion.
4. **Test set:** runs `validate_model.py latest --split test`.

Each run uses a new directory under `yoloshuttle/dataset`, never edits
old generated data. Offline jobs stop interactive Gazebo first to avoid GPU
contention.

## Current limitations and validation status

- The test suite checks **generated SDF structure, lighting inheritance,
  camera/depth geometry, shell and browser JavaScript**, but cannot test
  your machine's Ogre2 graphics startup.
- AprilTag absolute pose accuracy and YOLO 3D ground-truth RMSE are **not
  claimed** by this minimal rebuild; those require synchronized geometry
  comparisons after Gazebo is operational.
- This rebuild does not automatically publish camera settings or alter
  production simulation, Lighting/Mobility/Odometry Labs, or the real D435i.
- Dataset generation and training are local operations; they have not been
  run on the user workstation by GitHub Actions.

Offline tests:

```bash
python3 -m unittest discover -s tools/perception_labs/tests -p 'test_*.py' -v
```
