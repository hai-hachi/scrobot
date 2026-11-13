# SCROBOT Camera Lab — minimal, independent camera performance test

This version is **rebuilt from scratch**. It uses none of the former
`tools/perception_labs` code, training logic, Nav2, lighting GUI
or YOLO files. It now includes a separate AprilTags test tab with only
one authentic AprilTag detector and a stationary camera rig.

The Gazebo world has only a floor, a box target and one stationary camera rig
with independently configurable RGB and depth sensors.

## Start

```bash
cd ~/scrobot_ws
git switch real-comparison
git pull --ff-only
bash tools/camera_lab/start.sh
```

Open **http://127.0.0.1:8769**. Click **Start / Restart test**.

ROS 2 Jazzy, Gazebo Harmonic, `ros_gz_image`, `rclpy` and
`sensor_msgs` must already be installed. The runner sources
`/opt/ros/jazzy/setup.bash` and, if present, the workspace
`install/setup.bash`. A ROS package build is not required for this lab.

The lab uses ROS domain 82, isolated Gazebo partition
`scrobot_camera_lab_<uid>_82`, and no Gazebo GUI. Do not run concurrent
Gazebo benchmarks while measuring CPU.

## AprilTags Lab — appended to this same standalone dashboard

Select **AprilTags Lab** at http://127.0.0.1:8769. This new implementation
does **not** import or launch any code from the old
`tools/perception_labs/` directory. The original Camera Lab benchmarking
mode is retained as a separate tab.

### Robot placement and live camera overlay

- Set a robot pose with the **interactive overhead court** (click to
  set X/Y) plus numeric X/Y/yaw, or use **Face selected tag at 2 m**.
  Click **Place / rotate robot** to move the static camera rig through
  Gazebo's `/world/camera_lab/set_pose` service. The camera uses the
  calibrated SCROBOT mount and looks down 15 degrees.
- Adjust the active AprilTag edge size (40–250 mm), height above court
  (120–1500 mm), or RGB Gaussian pixel-noise standard deviation
  (0–50 out of 255).
- **Restart AprilTags scene** after changing tag size or height. The four
  authentic 16h5 textures and OBJ meshes are generated with the existing
  canonical `scrobot_simulation/scripts/generate_court_apriltags.py`
  utility, using temporary lab YAML. Production landmarks, tag textures
  and localization parameters are **not** edited.
- **Apply noise live** applies the Gaussian intensity noise to RGB frames
  before `apriltag_ros` receives them. The noise and JPEG preview run
  in `tag_monitor`; the detector's CPU and memory are measured
  **separately**.
- Observe the actual RGB image with detected corner boundaries and
  coordinate axes from AprilTag TF pose estimates, when TF is available.
  The JPEG preview is intentionally throttled; it is not used for the
  detection FPS meter.

The mounted plate includes a one-cell quiet border on every side, so
`plate_edge = active_tag_edge * 8/6`. On each net pole, the two
plate normals are separated by 90 degrees. To prevent those two
orthogonal plates from intersecting, the lab calculates the center-to-pole
mount radius automatically:

```text
mount_radius = max(plate_edge/2 + 0.010, pole_radius + 0.006)
pole_radius = 0.025 m
```

This leaves at least 10 mm tangential separation at the projected
plane intersection, also clearing the actual pole radius. The requested
height must also leave at least 8 mm clearance above the floor at the
bottom of the rendered plate. These dimensions are only used in the
**AprilTags Lab test world**.

### Detector metrics and ground truth

The lab starts the real `apriltag_ros/apriltag_node` configured for
tag family **16h5, IDs 0–3**, and the current active edge size.
`/camera_lab/detections` provides real IDs/corners and detection margins,
while **`/tf`** carries detector-estimated 3D poses for
`observed_tag_0` through `observed_tag_3`. CameraInfo is bridged
separately with sensor-data QoS.

Known simulated robot and tag poses determine the expected relative
camera-optical translation, including the robot camera's measured offset
and pitch. This is independent of the AprilTag detector. The browser
displays:

- Detection success percentage per processed AprilTag detector frame.
- Detector wall FPS; separate detector CPU% and resident memory.
- Tag 3D translation RMSE and range RMSE, in meters versus true geometry.
- Bearing RMSE in degrees from the camera optical frame.
- Unsigned plane-normal axis error, which does **not** claim a full
  3-DOF tag orientation error or account for texture orientation.
- The count of raw camera frames, detector frames, tag hits and TF poses.

A missed tag is **not** a zero-error measurement. Detector error is
reported only for actual TF poses. Check both success rate and pose
accuracy when judging reliability. Color/depth resolution and frame
rate are still configurable in the shared Camera Lab controls.

### Automated experiments

**Distance sweep:** choose target tag, minimum/maximum robot-base
distance, and step (0.4–8.0 m). The robot moves along the selected
tag's outward normal and faces directly toward the tag.

**Heading sweep:** choose a fixed robot-base distance (0.4–8.0 m),
minimum/maximum angle offsets, and step (within ±90°). The robot stays
at the fixed location while rotating. 0° means it faces the tag.

Both modes stop the robot at every pose, wait for image/TF queues to
settle, then collect measurements for the selected dwell time. The
dashboard plots **translation RMSE and detection success** against
distance or heading offset, shows progress and offers Cancel.

Completed sweeps are automatically saved to
`evaluation_results/camera_lab/tag_sweeps/apriltag_*.json`, preserving
the camera profile, tag geometry, noise, test poses, and all per-pose
metrics. No automatic changes are made to real robot pose or physical
AprilTag parameters.

**Dependencies in the ROS environment:** `apriltag_ros`,
`apriltag_msgs`, `tf2_msgs`, `ros_gz_bridge`, `ros_gz_image`,
`python3-opencv` with `cv2.aruco`, `numpy`, `yaml` and Gazebo
Harmonic. The first AprilTag launch generates textures with the same
utility used by the existing SCROBOT simulation model.

**Known limitation:** The simulation-independent regression suite checks
geometry, mode selection and wiring, but cannot verify an actual
Gazebo → ROS image/CameraInfo → AprilTag → TF live pipeline in CI.
Confirm this once with a static robot facing tag 0 at 2 m before
trusting the plotted accuracy, because camera optical-frame conventions
must be checked with the exact installed bridge/detector versions.

## D435i USB 3 available modes

These USB 3.1 mode combinations come from RealSense D400 Series datasheet
Table 4-2 (2024 revision):

[RealSense D400 Datasheet](https://realsenseai.com/wp-content/uploads/2024/10/Intel-RealSense-D400-Series-Datasheet-October-2024.pdf)

| RGB resolution | RGB FPS |
| --- | --- |
| 1920x1080 | 6, 15, 30 |
| 1280x720 | 6, 15, 30 |
| 960x540 | 6, 15, 30, 60 |
| 848x480 | 6, 15, 30, 60 |
| 640x480 | 6, 15, 30, 60 |
| 640x360 | 6, 15, 30, 60 |
| 424x240 | 6, 15, 30, 60 |
| 320x240 | 6, 30, 60 |
| 320x180 | 6, 30, 60 |

| Depth resolution | Depth FPS |
| --- | --- |
| 1280x720 | 6, 15, 30 |
| 848x480 | 6, 15, 30, 60, 90 |
| 640x480 | 6, 15, 30, 60, 90 |
| 640x360 | 6, 15, 30, 60, 90 |
| 480x270 | 6, 15, 30, 60, 90 |
| 424x240 | 6, 15, 30, 60, 90 |

These are **per-stream USB 3 modes**, not a guarantee every RGB and depth
pair can stream together at full FPS. USB 2 supports fewer options.
Check the exact physically connected D435i using
`rs-enumerate-devices` or RealSense Viewer. This lab tests simulated
Gazebo camera throughput; it does not drive a real D435i over USB.

## What is measured

The smallest pipeline is:

```text
Gazebo RGB/depth sensors -> ros_gz_image image_bridge
                        -> ROS Image (BEST_EFFORT, KEEP_LAST=1)
                        -> camera_lab/monitor.py
```

The monitor never decodes or saves image pixels. It reads the image message
timestamp, width, height, and raw data length.

- **Delivered image data speed (MB/s):** sum of raw ROS `Image.data`
  lengths in a 3-second wall-clock window, divided by the elapsed time.
  It excludes Gazebo internal transport, DDS framing, and any compression.
- **Wall FPS:** frames received per monotonic *real* second.
- **Sim FPS:** received frames per simulated second, using message timestamps.
  If Gazebo runs at 1.2x real time, a 50 Hz simulated stream can appear
  at about 60 wall FPS. QoS cannot generate new unique sensor frames.
- **CPU utilization:** Gazebo, image bridge and monitor each have a
  separate CPU reading, including descendant processes. 100% is one
  fully occupied logical CPU core; multithreaded processes may exceed 100%.
- **RAM:** resident memory per process group (shared pages may double-count).
- **Requested bandwidth estimate:** RGB8 is 3 bytes/pixel and simulated
  Gazebo depth 32FC1 is 4 bytes/pixel. The physical USB depth stream Z16
  is 2 bytes/pixel; these ROS transfer figures are **not USB bitrates**.

The browser displays only text and tables. There is no camera preview or
image encoding, avoiding extra CPU cost in the measurement process.

## Save results and simulation settings

**Save benchmark JSON** writes the current requested settings, observed rates,
CPU and RAM to `evaluation_results/camera_lab/benchmark_*.json`. At least
30 frames per enabled stream must have arrived, with matching dimensions.

**Save tested settings to simulation YAML** requires at least 80% of the
requested *wall-clock* FPS (and 30 frames) for both enabled streams.
After browser confirmation, it updates
`src/scrobot_simulation/config/camera_calibration.yaml`, first preserving
its previous contents under
`evaluation_results/camera_lab/backups/`.

The main Gazebo `simulation.launch.py` loads this simulation-only YAML
after rebuilding or synchronizing the installed config. The physical D435i
hardware driver is not modified.

After changing the YAML:

```bash
cd ~/scrobot_ws
colcon build --symlink-install --packages-up-to scrobot_bringup
source install/setup.bash
ros2 launch scrobot_bringup sim.launch.py
```

**Verification caveat:** Offline regression tests check the mode table,
SDF/XML, endpoint contracts, and the camera configuration chain.
Actual Gazebo image throughput and CPU utilization require a live test on
your machine. We must confirm the Gazebo server and ROS image_bridge
start correctly there before trusting any benchmark.
