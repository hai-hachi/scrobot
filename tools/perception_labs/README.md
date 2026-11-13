# Legacy Perception Labs v3 (historical)

> **Not the active perception workflow.** Use `tools/camera_lab/` for
> camera/AprilTags and `tools/yolo_lab/` for shuttlecock detection.
> This folder is retained because its benchmark data and historical
> experiments may still be useful; it does not own lab YAML state.

# SCROBOT Perception Labs v3 — clean, staged Gazebo recovery

The previous Perception Lab was backed up and **perception-related simulation
changes were reverted to the last known pre-perception commit
`7d7fd4757a6e`**. Lighting, Mobility and Odometry work was preserved.
This v3 implementation is a standalone workspace-root tool with no changes
to the normal robot URDF, sensor Xacro or camera publishing defaults.

Gazebo is brought up in **three strictly separated stages**: a physics-only
world (no rendering plugin), a low-cost RGB camera smoke scene, then the
selected published-court perception lab. The GUI runs separately and the
dashboard continues to work when the Gazebo process fails.

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

## Camera Lab (first calibration stage)

Use the **Camera performance** tab only, with no AprilTag or YOLO detector
running. This is intentionally isolated from the normal robot, Lighting,
Mobility and Odometry tests, and reuses the tested v3 staged Gazebo startup.

1. **Gazebo physics only** establishes a baseline with no rendering engine,
   sensors, bridges, camera traffic, or detection.
2. **RGB camera smoke** tests a fixed lightweight 640×480 @ 15 FPS stream.
3. **Start selected lab** with the Camera tab active, and set RGB and depth
   independently (RGB 640×480 / 848×480 / 1280×720; depth 424×240 /
   640×480 / 848×480 / 1280×720; 5, 10, 15, 30 or 60 Hz).
   The full court uses the published Lighting Lab scene. Restart the lab
   for each camera configuration; changes are not hot-swapped in Gazebo.
4. Collect **at least 30 frames** on each enabled stream, then **Save
   benchmark**. Prefer several runs of each setting after warmup.
5. Compare the saved rows in **Saved camera benchmark comparison**.
   Track RGB/depth sim FPS, wall FPS, delivered ROS MB/s, effective camera
   RTF, Gazebo CPU, ROS image bridge CPU and monitor CPU/memory.
6. If and only if the camera benchmark passes the measured-frame checks,
   click **Publish saved camera benchmark** and confirm. The live camera
   configuration is **not** published by changing the UI alone.

### What is actually measured

- **ROS throughput:** each received raw RGB/depth `sensor_msgs/Image.data`
  length is counted without copying the image. The sliding time window
  reports decimal **MB per wall-clock second** at the ROS subscriber.
  This is NOT the Gazebo native transport bandwidth, DDS wire bandwidth,
  JPEG compression ratio, nor the RGB-depth registration node cost.
- **Frame rate:** the sensor message header produces sim FPS and jitter;
  the observer's monotonic clock produces wall FPS. A 50 Hz simulated
  topic can appear as 60 Hz wall clock when Gazebo runs at 1.2× RTF, or
  when queued samples arrive in bursts. MQTT/ROS QoS does not create new
  unique camera frames.
- **Effective camera RTF:** wall FPS / sim FPS. This estimates simulation
  speed from camera stamps, *not* a general Gazebo physics RTF diagnostic.
- **CPU:** sampled from Linux `/proc` for each launched process and its
  children. **100% means one fully utilized logical CPU core**. A
  multithreaded Gazebo process can exceed 100%; the bridge and image
  monitor are shown independently. Summed process RSS can double-count
  shared memory pages.
- **Preview:** the browser JPEG preview is sampled to ~2.5 FPS; the
  actual image-throughput meter counts full-rate raw ROS callbacks.
  The JPEG conversion still adds a small CPU load, especially on the
  observer process. For a pure transport measurement, compare with a
  separate subscriber that omits JPEG conversion.
- **Not yet measured:** native Gazebo `gz transport` bytes/sec, DDS
  packet overhead, GPU utilization/VRAM, independent renderer-only FPS,
  true per-stage end-to-end latency, and the depth-registration
  component's separate CPU usage. These are future additions rather
  than being inferred from ROS throughput or bridge process CPU.

The normal camera baseline before publication is **1280×720 RGB / 848×480
depth, both 15 Hz**, with depth enabled. A candidate below 80% of its
requested **simulated-time** RGB/depth frame rate cannot be published.
This is an initial guard, not a guarantee the machine achieves the
requested FPS in real time; compare wall FPS and RTF before accepting it.

**Publishing:** the lab copies only the saved, measured Camera Lab
settings into:

```text
~/scrobot_ws/src/scrobot_simulation/config/camera_calibration.yaml
```

A byte-exact previous-file backup is stored under
`evaluation_results/perception_labs/camera_published_backups/`.
The normal Gazebo `simulation.launch.py` uses this YAML for RGB/depth
geometry, FPS and depth enable, unless explicit launch arguments override
them. No physical RealSense driver, STM32 firmware, camera intrinsic
calibration on the real robot, or other lab setting is changed.

After publishing:

```bash
cd ~/scrobot_ws
colcon build --symlink-install --packages-up-to scrobot_bringup
source install/setup.bash
# Stop the previous simulator, then start the normal simulation:
ros2 launch scrobot_bringup sim.launch.py
```

The standalone camera benchmark's static camera proxy and the full
physical robot are deliberately different workloads, so validate the
selected profile once more with the production full robot before relying
on its observed CPU and real-time performance.

## Mandatory first check: Minimal smoke test

1. Click **1 · Gazebo physics only**. The generated world contains the
   minimum physics, UserCommands and SceneBroadcaster plugins, plus a
   simple floor. It contains **zero cameras, zero ROS bridges, zero
   detectors, and no Ogre2 Sensors plugin**. Wait for
   **world_verified**. If this fails, inspect `gazebo.log` before
   attempting any RGB rendering.
2. Click **2 · RGB camera smoke**. This starts a new minimal world with one
   640×480 RGB camera at 15 Hz and a simple proxy body. Wait for
   **streaming** and a fresh frame. A healthy Gazebo process alone does
   not establish that ROS images are actually arriving.
3. Click **Open Gazebo GUI** independently. If server-side RGB works but
   the GUI does not, inspect `gui.log`; the GUI is not a prerequisite for
   sensor benchmarking.
4. Select **Camera performance**, **AprilTag**, or **Shuttle YOLO**, then
   click **3 · Start selected lab**. This loads an independent court
   world with the current published Lighting Lab illumination and only
   the requested sensors/detector.
5. Click **Save benchmark** while streaming to retain a timestamped
   JSON snapshot (scene provenance, selected resolutions/rates and
   measured ROS stream statistics) under
   `evaluation_results/perception_labs/benchmarks/`.

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
The dashboard reports measured tag output rate, IDs, decision margins,
per-tag detection frequency, observed camera-relative range, translation
repeatability and quaternion-sign-invariant 3D orientation repeatability
for existing 16h5 court tags 0–3. Moving the robot clears the per-pose
observation history so statistics from different poses do not mix.

The repeatability statistics are **not absolute position or orientation
accuracy**. This rebuilt version deliberately does **not** claim reliable
absolute pose RMSE until the optical-frame convention, tag physical
mount transform and Gazebo ground truth have been checked on the
workstation. It does not change `map→odom`.

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

## Regression boundaries

- Original implementation backup: branch
  `perception-v2-backup-20261113`. It can be compared or restored without
  changing other labs.
- **No production camera-calibration publish action is enabled.** Camera
  resolution and FPS changes stay inside the isolated world.
- The detector nodes use the existing SCROBOT AprilTag/YOLO code.
  Only the selected detector is launched, not Nav2, motor control,
  local collection, or the normal localization stack.
- The YOLO dataset capture launch has one backward-compatible optional
  `world` argument. Its default remains the existing dataset world;
  the lab passes a temporary copy with published lighting when needed.

## Current limitations and validation status

- The test suite checks **physics-only and RGB-only SDF structure, lighting inheritance,
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
