# SCROBOT Camera Lab — minimal, independent camera performance test

This version is **rebuilt from scratch**. It uses none of the former
`tools/perception_labs` code, training logic, Nav2, lighting GUI
or YOLO files. It now includes a separate AprilTags test tab with only
one authentic AprilTag detector and a stationary camera rig.

The Camera-only Gazebo world keeps a floor, target and camera rig but now
uses the **saved Lighting Lab YAML** for its full light array, shadows,
floor color and venue dimensions. AprilTags mode loads the same saved
Lighting YAML for the complete court, materials and walls. The scene
uses a frozen Lighting calibration snapshot until restarted.

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

The lab uses ROS domain 82 and the isolated Gazebo partition
`scrobot_camera_lab_<uid>_82`. It starts headless by default. Do not run
concurrent Gazebo benchmarks while measuring CPU.

### Optional Gazebo desktop UI

Both **Camera Lab** and **AprilTags Lab** have an **Open Gazebo UI /
Close Gazebo UI** toggle above the tabs. First start the selected lab,
then click **Open Gazebo UI**. It launches a separate, GUI-only process:

```bash
gz sim -g -v 3
```

The button launches the command from the same environment as the
existing `gz sim -s` server, including the matching `GZ_PARTITION`,
`GZ_IP`, and `GZ_SIM_RESOURCE_PATH`, so it attaches to the
**current running world**, not a second simulator.

In the Gazebo desktop 3D scene, use the normal orbit/pan/zoom controls
to explore the court geometry, tag mounting, lighting and robot proxy.
**Close Gazebo UI** terminates only this GUI process and does
**not** stop the server, image bridge, AprilTag detector or sweep.
Closing the main lab simulation automatically closes its GUI as well.
You can also close the desktop Gazebo window normally; the dashboard
then reflects the client process status.

This requires a graphical desktop (`DISPLAY` or `WAYLAND_DISPLAY`).
If the GUI exits without a window, expand **Gazebo UI diagnostics** in
the dashboard; the separate client log is also written to
`evaluation_results/camera_lab/gazebo_gui.log`.

**Performance:** Opening a 3D view increases GPU and sometimes CPU
rendering work. To compare true transport or detector FPS, close the
Gazebo GUI before running a benchmark. If it is closed, the headless
Gazebo server and camera sensors keep running.

## AprilTags Lab — appended to this same standalone dashboard

Select **AprilTags Lab** at http://127.0.0.1:8769. This new implementation
does **not** import or launch any code from the old
`tools/perception_labs/` directory. The original Camera Lab benchmarking
mode is retained as a separate tab.

### Camera range, live view rate and publishing

Both Gazebo RGB and depth sensors in the independent Camera Lab now
have a **20 m far clipping plane**; the standalone camera near plane
remains 0.1 m. The main simulation's D435i RGB far clip was already
20 m and its depth far clip is now 20 m as well. These are **simulated
rendering limits**, not a claim of 20 m reliable depth accuracy for the
physical D435i. Changing the far clip can increase rendering cost.

Beside the live AprilTag camera image, the dashboard reports:
- **RGB input resolution and measured FPS:** actual incoming ROS Image
  width/height and wall-clock frame arrival rate (rolling three seconds)
- **JPEG preview resolution and FPS:** actual dimensions after downsizing
  to at most 960 px wide, and wall-clock overlay generation rate; the
  browser requests an updated preview at most twice per second
- **Detector throughput:** independently measured from detection-array
  output, not the preview rate

**Save AprilTags Lab YAML** stores the selected shared tag geometry and
test pose/noise at `src/scrobot_simulation/config/apriltags_lab.yaml`,
without touching production. **Publish saved AprilTags YAML** reads this
saved YAML only, never unsaved sliders. On confirmation, it updates:

1. `src/scrobot_localization/config/court_landmarks.yaml`:
   `tag_edge_size`, `tag_height` and computed `tag_mount_radius`.
   The main Gazebo `court_apriltags` model is generated from these values
   at build time. The localization module uses the same tag geometry.
2. `src/scrobot_perception/config/perception.yaml`:
   the `apriltag` detector's `size` and all four `tag.sizes`, so
   PnP pose estimation agrees with the rendered geometry.

The original YAML files are backed up under
`evaluation_results/camera_lab/tag_published_backups/<timestamp>/`.
Changes are written via temporary files and verified; if a later write
fails, previously modified files are restored from their backups.

**Important:** `perception.yaml` is shared by the main simulator and
the physical robot launch. Publishing therefore changes the real
AprilTag detector's assumed physical tag edge size as well. Use this
button for production only if the real printed tags match the size.
The lab's Gaussian **RGB pixel noise is not published**; it is a
test-only image effect, not a physical tag property.

Rebuild the main simulation and production detector and restart Gazebo
after publishing:

```bash
cd ~/scrobot_ws
colcon build --symlink-install --packages-up-to scrobot_bringup
source install/setup.bash
ros2 launch scrobot_bringup sim.launch.py
```

The normal localizer's `max_tag_distance` is still 10 m, which is a
**separate localization acceptance threshold**, not the camera's new
20 m rendering far clip. The isolated AprilTags Lab can continue
measuring detections and pose error beyond 10 m.

### Reset pose errors and update all AprilTags together

**Every successful robot relocation**, including each distance/heading sweep
step, creates a new AprilTag measurement epoch. The lab clears prior
translation/range/bearing/normal errors, detector counts, and the old
camera preview immediately. The observer also discards delayed detection
messages and TF poses belonging to image frames from the previous pose,
and waits briefly for the static camera to settle before collecting
new samples. During that interval, RMSE values display **—** instead
of retaining the old location's error.

**All four tags share one geometry**: the *Active tag edge* and *Tag
center height* sliders apply to IDs 0–3, regardless of the selected
target tag. Press **Apply geometry to all 4 tags** to regenerate the
four physical plates and set all four detector size parameters in
one operation. This restarts the isolated AprilTags scene (required
for Gazebo mesh updates), but preserves the configured camera, robot
X/Y/yaw, target selection and RGB noise. The scene's calculated pole
radius continues to prevent tag-plate intersection. Errors are reset
because the detector and court geometry have changed.

**Publish all 4 tags to main simulation** is a *separate* explicitly
confirmed action. It saves the shared tag edge, height and mount radius
to production `court_landmarks.yaml` plus all four detector tag sizes
in `perception.yaml`, preserving YAML backups. It does not publish
the lab's simulated image noise. Rebuild and restart the main
simulation to regenerate the production tag model; the production
detector YAML is shared with the real robot, so printed tags must have
the same edge size.

### Robot placement and live camera overlay

- Set a robot pose with the **interactive overhead court** (click to
  set X/Y) plus numeric X/Y/yaw, or use **Face selected tag at 2 m**.
  Click **Place / rotate robot** to move the static camera rig through
  Gazebo's `/world/camera_lab/set_pose` service. The camera uses the
  calibrated SCROBOT mount and looks down 15 degrees.
  **Robot placement now extends outside the green court mat**, across the
  calibrated Lighting Lab venue floor. The limits are loaded directly from
  `src/scrobot_simulation/config/lighting_lab.yaml`, independent of
  whether they have been published to the main simulation SDF. The lab
  regenerates the floor/walls using those dimensions and retains a 0.30 m
  outer margin for the robot proxy. With the current Lighting Lab
  clearances (X=2.8 m, Y=2.4 m per side), the floor is 21 × 12 m
  and robot-center limits are ±10.2 m in X and ±5.7 m in Y.
  The overhead GUI displays both the mat and its surrounding clearance.
  Restart the lab after changing Lighting Lab venue settings.
- Adjust the active AprilTag edge size (40–250 mm), height above court
  (120–1500 mm), or RGB Gaussian pixel-noise standard deviation
  (0–50 out of 255).
- **Apply geometry to all 4 tags** after changing size or height. This
  regenerates the four authentic 16h5 textures and OBJ meshes with
  `scrobot_simulation/scripts/generate_court_apriltags.py` using temporary
  lab YAML and restarts only the AprilTags test scene. Production
  landmarks, tag textures and localization parameters are **not** edited
  until the separate Publish button is explicitly confirmed.
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

### If RGB arrives but AprilTag detector messages stay at zero

An operating `apriltag_ros` node publishes a detection-array message for
**every synchronized image/CameraInfo pair**, even if the array is empty.
Zero **detector messages** with many received RGB frames is therefore a
camera pipeline / timestamp / QoS issue, **not** proof that the tags are
out of range.

The lab explicitly republishes **synchronized** RGB and CameraInfo to
`/camera_lab/tag/image_rect` and `/camera_lab/tag/camera_info`,
preserving an identical image timestamp for each pair. CameraInfo intrinsics
are copied from the bridged Gazebo color sensor, with a projection matrix
filled from K if the bridge supplies no valid P. The detector subscribes
to these matching camera-namespace topics, not raw unmatched CameraInfo.
RGB noise is applied only to the image before this paired publication.

Check the browser's pipeline health and these counters:
`RGB received`, `synchronized image/info pairs`, `image subscribers`,
`info subscribers`, and `detector messages`. In a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/scrobot_ws/install/setup.bash
export ROS_DOMAIN_ID=82
ros2 node info /apriltag
ros2 topic info --verbose /camera_lab/tag/image_rect
ros2 topic info --verbose /camera_lab/tag/camera_info
ros2 topic info --verbose /camera_lab/detections
tail -n 60 ~/scrobot_ws/evaluation_results/camera_lab/detector.log
```

Expected: matching image and CameraInfo topic subscriptions, steadily
increasing synchronized pair and detector message counters. Only after
that should tag visibility, geometry, noise or detection range be assessed.
To rule out image quality, first use sigma=0 and the 2 m head-on preset.

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

The Gazebo ROS image bridge currently sets the actual RGB/AprilTag
parent frame to `color_optical_frame`. The evaluator now checks TF
against the **incoming RGB image's own optical frame ID** instead of
hard-coding `camera_color_optical_frame`. It still rejects transforms
from genuinely different frames. This resolves blank RMSE measurements
caused by a frame-ID mismatch when valid tag TF poses are available.

A missed tag is **not** a zero-error measurement. Detector error is
reported only for actual TF poses. Check both success rate and pose
accuracy when judging reliability. Color/depth resolution and frame
rate are still configurable in the shared Camera Lab controls.

### Automated experiments

**Distance sweep:** choose target tag, minimum/maximum robot-base
distance, and step (0.4–12.0 m). The robot moves along the selected
tag's outward normal and faces directly toward the tag.

**Heading sweep:** choose a fixed robot-base distance (0.4–12.0 m),
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

**Save Camera Lab YAML** writes the chosen RGB/depth resolution and FPS
to `src/scrobot_simulation/config/camera_lab.yaml`. It does not
change the production camera. **Publish saved Camera YAML** reads that
saved YAML only; after browser confirmation, it updates
`src/scrobot_simulation/config/camera_calibration.yaml`, preserving
the previous file in `evaluation_results/camera_lab/backups/`.
This does not implicitly publish recent unsaved slider values.

Use **Save benchmark JSON** independently for actual delivered
wall-clock FPS, sim FPS, MB/s and CPU results; performance measurements
remain separate from the saved camera configuration.

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
