# SCROBOT Camera Lab — minimal, independent camera performance test

This version is **rebuilt from scratch**. It uses none of the former
`tools/perception_labs` code, training logic, AprilTag detector, Nav2,
robot model, lighting GUI or YOLO files.

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
