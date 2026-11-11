# SC Robot

ROS 2 Jazzy software for an autonomous badminton-shuttle collection robot.

## Platform

- NVIDIA Jetson Orin Nano high-level computer
- STM32F411 motor/collector controller over 1,000,000 baud UART
- Intel RealSense D435i RGB-D camera + IMU
- differential-drive base
- AprilTag court localization
- Nav2 four-pass court sweep
- YOLO + aligned-depth shuttle detection
- Gazebo Harmonic simulation and evaluation

The Jetson host uses Ubuntu 22.04 / Jetson Linux; ROS 2 Jazzy runs in the
Ubuntu 24.04 Orin container. Simulation development uses Ubuntu 24.04 directly.

## Packages

```text
scrobot_description   robot URDF/Xacro and fixed frames
scrobot_simulation    Gazebo world, sensors, bridges, shuttle physics
scrobot_hardware      STM32 protocol-v2 ros2_control SystemInterface
scrobot_bringup       normal sim + physical Orin/STM32/D435i entrypoints
scrobot_control       safe AUTO/MANUAL velocity command pipeline
scrobot_localization  wheel/D435i-IMU EKF and AprilTag global correction
scrobot_perception    AprilTag, depth scan, YOLO + aligned-depth 3D detection
scrobot_navigation    Nav2 and AprilTag approach motion strategies
scrobot_collection    shuttle eligibility, local SMC and pickup action
scrobot_mission       sweep, diversion, recovery, checkpoint resume
scrobot_interfaces    project actions and hardware messages
scrobot_evaluation    repeatable odometry and mission metrics
scrobot_debug         optional telemetry, RViz, and legacy test fixtures
scrobot_scenarios     opt-in integration/regression scenario entrypoints
```

## Production flow

```text
D435i + wheel feedback
        |
        +--> localization --> map -> odom -> base_footprint
        |
        +--> depth scan --> collision monitor + Nav2
        |
        +--> AprilTag --> global relocalization
        |
        +--> YOLO + aligned depth
                    |
                    v
        /perception/shuttle_detections_3d
                    |
                    v
        shuttle_collection_filter
                    |
                    v
        sweep mission / local collection
                    |
                    v
        AUTO/MANUAL safety pipeline
                    |
                    v
        diff_drive_controller
                    |
                    v
        Gazebo system OR STM32 ros2_control hardware
```

Collection eligibility is 0.50-1.80 m planar range in `base_link`. The local
controller normally stages `base_link` 1.10 m from the frozen shuttle; when a
shuttle is already closer than 1.10 m, the staging distance is clamped to the
current range so the robot aligns in place instead of requesting a pose behind
it. After a stable pre-pose, collection is a straight 0.30 m/s pass plus a
0.10 m overrun.

## Build

```bash
cd ~/scrobot_ws
colcon build --symlink-install
source install/setup.bash
```

## Normal simulated autonomy (no developer-only processes)

```bash
export SCROBOT_YOLO_MODEL=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt
ros2 launch scrobot_bringup sim.launch.py
```

This starts Gazebo, controller/safety, localization, perception, shuttle
spawner, and the mission; it does **not** start the evaluator, RViz, terminal
telemetry, or annotated YOLO video. For the historical non-magnetic localization
profile add `use_magnetometer:=false`. Other runtime parameters and the
mission sequence are unchanged.

## Gazebo lighting and materials calibration (3-view GUI)

The opt-in lighting lab creates a **temporary derivative** badminton court,
then streams real Gazebo D435i, overhead and movable free-camera views into
one local browser dashboard. Configure light rows/columns/span/height,
intensity, shadow casting, attenuation, cone angles, HSV colors,
venue clearance, random shuttle batches and performance; no mission is started.

```bash
sudo apt install python3-gz-transport13 python3-gz-msgs10 python3-pil
cd ~/scrobot_ws/src
python3 scripts/setup_workspace_tools.py  # once; persistent across reboot
cd ~/scrobot_ws
python3 tools/lighting_lab/server.py
```

Open http://127.0.0.1:8765. WARP-safe loopback transport is enforced
automatically every launch. Lighting uses **center-based X/Y reach**,
**HSV color** and independent **intensity**, plus material HSV settings.
The saved/published calibration currently contains **2×3 lights, intensity
2.5**, with two shadow casters. The dashboard follows the Gazebo GUI orbit
camera. X/Y venue clearance sliders rebuild the calibration floor and walls;
net pole diameter remains fixed. See
[lighting-lab calibration workflow](docs/real_comparison/lighting_lab.md).
Lab preview alone does not change production; explicit **Publish** updates
the Git-tracked production SDF. The physical robot is never changed.

## Mobility Lab — floor-loaded motor and physics calibration

`tools/mobility_lab/` runs independent **90 RPM** wheel step tests
on the Gazebo floor with the real September 2026 STM32 PID recordings
built in. Adjust first-order closed-loop wheel response, contact friction,
slip, joint damping and contact geometry; compare RPM and Gazebo ground
truth. **Save Lab Settings** affects only the lab;
**Publish to Simulation** backs up and updates simulation-only
`src/scrobot_simulation/config/mobility_calibration.yaml`. The first-order
node is enabled only in normal *simulation* bringup, never real hardware.

```bash
python3 tools/mobility_lab/server.py
# http://127.0.0.1:8766
```

## Odometry Lab — publishing and estimated-vs-true motion

`tools/odometry_lab/` measures publishing frequency, timing, CPU cost,
encoder quantization and odometry error against Gazebo ground truth. It
requires a running SCROBOT simulation and observes production topics; its
lab-specific encoder odometry topic does not replace production odometry.
Motion tests are opt-in and use the existing manual/safety pipeline.

```bash
python3 tools/odometry_lab/web_server.py
# http://127.0.0.1:8767
```

The three independent web labs occupy ports **8765 (Lighting)**,
**8766 (Mobility)** and **8767 (Odometry)** by default.

For a combined calibration workflow, run Lighting and Mobility as **separate
Gazebo calibration sessions**, publish their simulation-only settings, then
launch one normal sim without an autonomous mission for Odometry:

```bash
ros2 launch scrobot_bringup sim.launch.py \\
  launch_mission:=false enable_yolo:=false spawn_shuttles:=false
# separate sourced terminal:
python3 tools/odometry_lab/web_server.py
```

See [three-lab coexistence and validation](docs/real_comparison/lab_coexistence.md)
for ROS domain, Gazebo ownership, safety preflight and offline checks.

## Full simulation regression

The historical regression command remains supported:

```bash
export SCROBOT_YOLO_MODEL=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt
ros2 launch scrobot_debug full_mission_check.launch.py
```

The new opt-in scenario name invokes the same test:

```bash
ros2 launch scrobot_scenarios full_mission_check.launch.py
```

For automated acceptance on the Jazzy/Gazebo workstation, run from the
repository root:

```bash
python3 tools/real_comparison/run_full_mission.py \
  --model /path/to/gazebo_simple_v2.pt \
  --profile october_baseline \
  --no-rviz --no-debug-image
```

It checks the evaluator's actual `summary.csv` for `COMPLETE`, all 50
shuttle removals, and position RMSE. This mission run still requires your
ROS 2 Jazzy/Gazebo workstation and the real model artifact. See the
[workstation regression procedure](docs/real_comparison/full_mission_regression.md).

Architecture and test boundaries are documented in
[real-comparison migration](docs/real_comparison/architecture_migration.md).


The historical validated repeatable run collected 50/50 shuttles and reached
`COMPLETE`. Master RViz is enabled by default. That run used the
**magnetometer-disabled** configuration.

The simulated 5883L magnetometer is now enabled by default for new full-mission
runs (`/imu/mag` at 50 Hz, fused by Madgwick). Use
`use_magnetometer:=false` to reproduce the historical benchmark. The physical
Orin bringup still disables external magnetometer fusion.

## Physical Orin bringup

See [docker/ORIN.md](docker/ORIN.md). The physical launch is deliberately
non-moving by default:

```bash
ros2 launch scrobot_bringup robot.launch.py
```

Autonomous motion must be requested explicitly with `launch_mission:=true`.

## Documentation

- [Package/system architecture](docs/package_architecture.md)
- [ROS interfaces](docs/interface_specification.md)
- [Final full-mission validation](docs/full_mission_validation.md)
- [Repository overhaul / branch disposition](docs/repository_overhaul.md)
