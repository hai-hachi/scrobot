# SCROBOT Perception Labs

One web dashboard, three independent **simulation-only** experiments:
Camera Performance, AprilTag Quality, and Shuttle YOLO (dataset, training,
validation and simulation with real bounding-box overlays). Based on the
existing SCROBOT ROS2 Jazzy + Gazebo Harmonic pipeline and the author's
`hai-hachi/yoloshuttle` repository.

## One-command launch (Gazebo starts automatically)

**Do not start Gazebo in a separate terminal.** The Perception Labs
dashboard owns the complete **Gazebo + robot camera + bridges + optional
depth registration + selected detector** ROS launch process and shuts it
down with Ctrl+C.

After pulling `real-comparison`, run this once to build and start:

```bash
cd ~/scrobot_ws/src
git switch real-comparison
git pull --ff-only

cd ~/scrobot_ws
bash tools/perception_labs/start.sh --build
```

On subsequent uses, the entire workflow is a single command:

```bash
bash ~/scrobot_ws/tools/perception_labs/start.sh
```

The launcher sources ROS 2 Jazzy and this workspace automatically, starts
the local dashboard server, starts **Gazebo + the Camera Performance
experiment** using the saved settings (or defaults), and opens the browser.
If the browser cannot open automatically, visit
**http://127.0.0.1:8768**.

The three tabs share one Gazebo instance **at a time**, not three separate
simulators. To switch to AprilTag or Shuttle YOLO, select its tab and click
**Start / restart this lab**. The server automatically stops the previous
Gazebo session and launches the selected detector along with the camera and
bridges; you never need a standalone `ros2 launch` command. This is
intentional: running YOLO in the camera-only test would invalidate the
FPS baseline.

If Gazebo fails to start (e.g. missing ROS dependency), the web UI stays
available. It now applies the same **WARP-safe localhost network settings**
as Lighting Lab, and checks actual Gazebo world/topic discovery and ROS
publisher/message reception rather than treating a running `ros2 launch`
PID as proof that Gazebo is working.

Open **Gazebo startup diagnostics / error log** immediately below the
camera preview. The status pinpoints these different startup stages:

- **Gazebo launch stopped:** the launch exited, or failed before Gazebo was ready.
- **Gazebo world missing:** the process is alive, but the court world was
  not discovered yet. Look for plugin/rendering errors in its launch log.
- **Gazebo RGB sensor missing:** court world exists, but the robot camera
  was not spawned or the xacro/sensor failed.
- **ROS image bridge missing:** Gazebo has `/camera/color`, but the ROS
  `/camera/camera/color/image_raw` publisher is absent.
- **No fresh RGB messages:** the publisher exists, but images are not being
  delivered to the monitor; check QoS, topic rates and simulation progress.
- **Streaming:** new ROS RGB frames have arrived within the past five
  seconds; the camera path is functioning.

The dashboard exposes a **Retry lightweight RGB-only camera** button that
automatically saves and starts 640×480 RGB at 15 FPS with depth/registration
disabled. This separates a basic render/startup problem from the cost of
running both D435i channels. Return to the 1280×720/848×480 published
baseline after confirming camera frames.

The latest launch log is reset on every Gazebo restart and is available at
`~/scrobot_ws/evaluation_results/perception_labs/gazebo.log` and directly
in the diagnostics panel. If it still fails, copy the **launch-stage status
and the last errors**; do not start a second Gazebo process manually.

To open the dashboard without launching a browser on a headless machine:

```bash
bash ~/scrobot_ws/tools/perception_labs/start.sh --no-browser
```

To use a non-standard workspace root, set `SCROBOT_WS=/absolute/workspace`
when calling the launcher. The launcher accepts `--build` to rebuild before
starting. You do not need to re-run a build after changing only HTML/Python
files in `tools/perception_labs` when the ROS package dependencies were
already built.

The server binds only to
`127.0.0.1`, with `ROS_DOMAIN_ID=79` and a dedicated `GZ_PARTITION`.
It does not run Nav2, controllers, localization, the full mission, or
automatic shuttle collection commands. Do not run other applications using
domain 79 concurrently.

If your workspace checkout locates the `tools` directory under a different
path, use its actual `perception_labs/server.py`. SCROBOT ROS package shares
must be sourced from the built workspace.

Dependencies: ROS2 Jazzy, Gazebo Harmonic, `ros_gz_image`,
`ros_gz_bridge`, `apriltag_ros` / `apriltag_msgs`,
`scrobot_debug`, `scrobot_perception`, `depth_image_proc`, and Python 3
with `opencv-python`, `numpy`, and `PyYAML`. NVIDIA GPU measurements
require optional `nvidia-smi`; host CPU/memory stats require optional
`psutil`. Missing optional system metrics remain unreported, not zero.

### Camera baseline and FPS comparisons

| Channel | Previously installed baseline | Lab choices |
|---|---|---|
| RGB | 1280 × 720, 15 Hz | 640×360, 640×480, 848×480, 1280×720 |
| Depth | 848 × 480, 15 Hz | 424×240, 640×480, 848×480, 1280×720 |
| Requested FPS | 15 | 5, 15, 30, 60 Hz |

These are Gazebo sensor configurations, **not** guaranteed supported RealSense
D435i physical streaming profiles. RGB and depth HFOV remain at their
previously calibrated values; changing aspect ratio affects intrinsic pixel
scale, so use live CameraInfo for deprojection.

For a fair bottleneck comparison:

1. First test RGB alone: uncheck **Enable depth camera** and depth alignment.
2. Enable raw depth, with registration off.
3. Enable depth-to-color registration and compare aligned depth throughput.
4. Switch to AprilTag only, then YOLO, comparing detection output FPS.
5. Finally run a full production integration benchmark separately, because
   all together uses more CPU/GPU and interferes with camera-only baseline.

The dashboard reports header-based **sim FPS**, actual **wall FPS**, inter-frame
jitter, freshness, dropped/non-monotonic stamps, approximate sim-clock
header-to-receipt delay, overall CPU/GPU status and a stage bottleneck hint.
Because Gazebo and the ROS bridge are not separately instrumented, a low
RGB delivered FPS only isolates the combined renderer/bridge/subscriber stage
and cannot attribute cost to one of those components. A 30 Hz Gazebo stream
with RTF below 1 can have less than 30 real frames/second. Preview FPS is
intentionally rate-limited and does NOT determine the benchmark.

The main camera calibration publishes only
`src/scrobot_simulation/config/perception_calibration.yaml` and affects
normal Gazebo startup after build/restart. The real D435i driver configuration
is untouched. A backup is saved before changed publication.

### AprilTag quality and independent position checks

Select **AprilTag**, restart the lab to launch the existing
`apriltag_ros` detector, and set the robot's X/Y/yaw on the overhead court
or with sliders. `/debug/apriltag_overlay/image` supplies the genuine
visualization; the monitor subscribes to `/apriltag/detections` and
`/evaluation/ground_truth_odom`.

Test repeatable distances/angles/lighting, measuring:
- Correct AprilTag 16h5 IDs 0,1,2,3 (100 mm active edge)
- Detector messages, non-empty detections, decision margins, Hamming errors
- Detector throughput versus actual RGB throughput
- **Stationary** camera-frame tag XYZ error and range RMSE **if**
  `base_footprint→camera_color_optical_frame` and
  `camera_color_optical_frame→observed_tag_N` TF are available, against
  the production court tag geometry and independent Gazebo base pose.
- Tag 3D orientation **repeatability** from quaternion angular deviation
  relative to the first sample at the same stationary robot pose. This is
  not absolute orientation error, and includes any time drift.

Tag detection-to-RGB ratios are **not calibrated recall** unless a tag is
known to be visible in every evaluated frame. Translation results are
not reported as 6-DOF accuracy; camera and AprilTag coordinate conventions
need careful verification before rotation/reprojection scores. Missing TF
means unavailable error, not zero error. AprilTag's 3D pose will be sensitive
to small tags at long distance and oblique angles.

The world/robot camera are NOT relocalized using tags during this test;
only perception detection quality is evaluated.

### Shuttle YOLO interactive test

Select **Shuttle YOLO**, load a local .pt file or leave the model path blank
to use the latest model in the `yoloshuttle` checkout (falling back to
`best.pt`). Restart Gazebo. Move/rotate the robot, place an individual
shuttle, or spawn a repeatable random group. The live overlay comes from
the production `/perception/shuttle_debug/image` node; bbox pixel values,
confidence and 2D/3D output rates are collected from the real
`/perception/shuttle_detections_{2d,3d}` topics.

The simulation enables RGB-depth registration for YOLO regardless of the
camera-only benchmark toggle. The RGB image, aligned depth and CameraInfo
must agree. The plotted 3D detection rate is **not** itself a 3D position
accuracy metric; ground-truth-to-detection association and visible-vs-hidden
object filtering would be required for trustworthy 3D RMSE/recall.

### Reusing hai-hachi/yoloshuttle for datasets/training

The repository was inspected and can be accessed:
[hai-hachi/yoloshuttle](https://github.com/hai-hachi/yoloshuttle).

It includes `scripts/train_finetune.py`,
`scripts/validate_generated_dataset.py` and
`scripts/validate_model.py`. We intentionally invoke these scripts instead
of copying their implementations into SCROBOT. The SCROBOT dataset capture
already protects labels against stale rendered images using CRC tests and
groups train/val/test splits by viewpoint. That is retained.

Ensure the repository is cloned locally:

```bash
git clone https://github.com/hai-hachi/yoloshuttle.git ~/Desktop/yoloshuttle
# If already present, update it instead:
git -C ~/Desktop/yoloshuttle pull --ff-only
cd ~/Desktop/yoloshuttle
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
deactivate
```

You may instead set `SCROBOT_YOLO_REPO=/absolute/path/to/yoloshuttle`
before starting this lab.

The YOLO panel runs four local tasks in order:
1. **Generate current-light dataset**: rebuilds a *temporary* headless
   `yolo_dataset` world with the exact lighting, materials, court, walls
   and net from the currently **published**
   `src/scrobot_simulation/worlds/badminton_court.sdf` (output fingerprint
   recorded). Runs the existing `yolo_dataset_fast_capture` 100% unchanged
   except for its newly configurable input SDF path.
2. **Check dataset**: runs the existing `validate_generated_dataset.py`,
   checking label/image pairing, normalized bbox values, duplicate frames,
   grouped split integrity and bbox preview images.
3. **Fine-tune**: invokes `yoloshuttle/scripts/train_finetune.py`, selecting
   images/epochs/batch/imgsz, starting weights and GPU/CPU. It creates a
   new unique run name and outputs `artifacts/models`.
4. **Held-out validation**: uses
   `yoloshuttle/scripts/validate_model.py latest --split test`.
   Scores include precision, recall, mAP50, and mAP50–95.

Each generated dataset is stored in its own versioned directory under
`~/Desktop/yoloshuttle/dataset/gazebo_scrobot_lighting_YYYYMMDDThhmmssZ/`.
The latest dataset is selected via
`~/scrobot_ws/evaluation_results/perception_labs/latest_dataset.txt`.
This prevents mixing old and new lighting distributions in one dataset.
Each dataset has a `lighting_provenance.json` fingerprint of the actual
published court scene. The original `gazebo_scrobot_simple` dataset is not touched. The event-triggered
dataset camera currently uses 1280×720; the camera performance lab independently
varies live D435i RGB/depth resolution. The headless generator uses the same
production shuttle visual STL, not fabricated image compositing.

**Important:** this headless capture uses the latest *published* Lighting
Lab court scene, not unsaved Lighting Lab sliders. Publish lighting changes
first, then generate a new dataset. Training/validation starts only if the
generated `data.yaml` exists. Long-running jobs run locally and stream logs
into `~/scrobot_ws/evaluation_results/perception_labs/jobs`. The browser
starts/stops an actual local subprocess; ChatGPT itself does no asynchronous
training. Interactive Gazebo is shut down before expensive headless tasks.

After an offline job, restart the perception simulator to view live
YOLO detections. Training doesn't automatically overwrite SCROBOT's deployed
model, and synthetic-only scores must be compared to independent real
D435i images before deployment.

### Safe scene and experiment data boundaries

- Perception Lab changes **only** simulation camera-rate/resolution YAML
  through explicit confirmed publication, never the real D435i driver.
- Lighting is read from the *published* court SDF; this lab never changes it.
- AprilTag and YOLO thresholds/models are run as isolated tests; no mission,
  tag relocalizer, Nav2 or wheel command producer is started.
- ROS/Gazebo simulator domain 79 is isolated from Mobility (77), Odometry
  (78) and physical domain 0.
- Dataset job uses separate domain 80, unique Gazebo partition and stops
  interactive rendering first.
- Saved experiments and job logs are under
  `~/scrobot_ws/evaluation_results/perception_labs/`.

## Verification

```bash
python3 -m unittest discover -s ~/scrobot_ws/tools/perception_labs/tests -p 'test_*.py' -v
```

GitHub Actions validates Python syntax, browser script syntax, Xacro/SDF XML,
pose/camera math, publication backup and exact current-world lighting
replacement. Only a real Gazebo/ROS run can verify GPU performance,
AprilTag TF topic behavior, depth registration and actual detector throughput.
