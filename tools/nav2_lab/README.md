# SCROBOT Nav2 Lab — Phase 1: Path Planning

This lab is deliberately scoped to **path planning** first. Controller
tracking and full Nav2 integration are not started by this phase.

## Shared calibration — source of truth

**This lab consumes the saved YAML from your other labs, not their last
publication to the main simulation.** The canonical YAML registry is
`tools/shared/lab_configs.py`, and the five inputs are:

| Owner | Saved YAML | Values applied to Nav2 Lab |
| --- | --- | --- |
| Lighting | `tools/configs/labs/lighting_lab.yaml` | Lights, shadows, colors, resized physical venue/court floor |
| Camera | `tools/configs/labs/camera_lab.yaml` | RealSense RGB/depth widths, heights, frame rates and depth enablement |
| AprilTags | `tools/configs/labs/apriltags_lab.yaml` | Actual generated tag size, height and mount geometry |
| Mobility | `tools/configs/labs/mobility_lab.yaml` | Calibrated wheel contact friction/slip/damping and motor model snapshot |
| Odometry | `tools/configs/labs/odometry_lab.yaml` | Wheel odometry, IMU, magnetometer, ground-truth and EKF rates |

The Nav2 Lab saves **its own** experiment settings at
`tools/configs/labs/nav2_lab.yaml` on **Save YAML**, using the same
versioned `{schema_version: 1, lab: nav2, settings: ...}` contract.
The historical `tools/nav2_lab/config/lab.yaml` is a first-run template,
not the authority once saved.

Lighting, Camera and AprilTags saved YAMLs are **required** for the scene.
If any is missing, startup fails with an actionable message; it will not
silently use production defaults. Saved Mobility and Odometry YAMLs may exist
only on your local workstation and are not committed in this branch. They
are used when present. Otherwise the lab explicitly labels an individual
`published_fallback` and shows a warning instead of claiming it is a
saved measurement.

**At launch** the lab snapshots every loaded input, including source file
paths and SHA-256 hashes, into
`evaluation_results/nav2_lab/_planning_active/calibration_snapshot.yaml`.
The simulator receives an entirely **generated** SDF with your saved
Lighting and tag assets, and passes the saved D435i modes, Mobility physics
and Odometry rates as launch-time overrides. Odometry Lab's existing
`odometry_rate_config.prepare_configurations` creates temporary controller
and EKF YAMLs. The current Gazebo robot stays stationary in the
**planner-only** test, so Mobility's first-order motor response is recorded
for later tracking but is not needed for planner computation.

If any upstream saved calibration changes, the lab refuses further
benchmarks until you restart, preventing mismatched world/map/sensor settings.
Each run copies its own calibration snapshot. The lab never writes the
upstream YAMLs, ROS production configuration or production court SDF.

The saved Camera Lab settings currently checked into `real-comparison` are
**RGB 1920×1080 @30 Hz and depth 1280×720 @30 Hz**, rather than the
original 1280×720 @15 Hz / 848×480 @15 Hz Gazebo defaults.
FOV remains the existing simulated D435i lens FOV. Saved Lighting currently
specifies a 2×3 light array with six shadow casters and resized venue
clearance. Your current local YAMLs take precedence over repository snapshots.

YOLO Lab owns training and shuttle detection; its YAML is intentionally not
used for depth-based path planning.

## Purpose

Compare the *actual* Nav2 Jazzy global planner plugins on the existing SCROBOT
Gazebo Harmonic badminton court, using a common explicit start, goal and
costmap. View paths in **RViz** (not a duplicate web map), and measure planning
time, path length, turning severity, obstacle clearance and the Nav2 planner
server's CPU/RAM consumption.

Planners available:

- NavFn Dijkstra (`use_astar: false`)
- NavFn A* (`use_astar: true`)
- Smac Planner 2D (cost-aware A*)
- Theta* (any-angle)

All four variants are loaded as separate IDs into the same Nav2 planner_server.
Choose one or run all four sequentially without moving the robot.

## Run

Requires the existing SCROBOT workspace with `src/` and `tools/` beside
one another, ROS 2 Jazzy, Gazebo Harmonic and the repository already built.

```bash
cd ~/scrobot_ws
git pull
sudo apt install ros-jazzy-navigation2 python3-yaml python3-psutil
colcon build --symlink-install
source install/setup.bash
python3 tools/nav2_lab/server.py
```

The dashboard opens **http://127.0.0.1:8772** in a new browser tab unless
`--no-browser` is specified. The simulator starts headlessly. Use the
**Open Gazebo UI / Close Gazebo UI** and **Open RViz / Close RViz** buttons
without stopping the planning stack. Logs are always available at the end.

1. Start simulation; give Gazebo, localization, static map, perception, and
   planner lifecycle servers time to activate.
2. Open RViz using the web button. The `odom` frame, court static occupancy,
   depth scan, live global costmap and four colored planner-path topics are
   configured in `planning/config/planning.rviz`.
3. Choose explicit start and goal poses, then **Save YAML**.
4. Press **Plan / compare**. The robot does not move. The real
   `/compute_path_to_pose` action receives `use_start=true`, so all planners
   receive the same start even though the physical robot stays stationary.
5. Read **Benchmarks**. All runs are reproducibly written to
   `~/scrobot_ws/evaluation_results/nav2_lab/`.

### Environment and map ownership

The lab **does not edit** the production simulation, lighting, camera or Nav2
configuration. It uses:

- `src/scrobot_simulation/worlds/badminton_court.sdf`: pole geometry and static court objects. The floor is resized from the saved Lighting Lab venue clearances (currently 21 × 12 m).
- `src/scrobot_localization/config/court_landmarks.yaml`: baseline pole positions and tag headings; size/height/mount radius are regenerated using **saved AprilTags Lab YAML**.
- Production `src/scrobot_navigation/config/nav2_params.yaml`: robot footprint,
  obstacle-layer ranges and inflation parameters, copied and overlaid into a
  temporary **planner-only** YAML.
- Production `scrobot_perception` scan with **saved Camera Lab** RGB/depth resolution and rates (currently depth **1280 × 720 @ 30 FPS**), retaining approximately **90.5° depth HFOV** and the existing depth cloud / scan pipeline.

The generated free-space occupancy map contains **only** poles and the
conservative 2D envelopes of the AprilTag mounting plates as permanent
obstacles. The court markings and suspended badminton net are visually
rendered but not occupied. The floor still collides physically with the robot,
as necessary to support motion; safety boundary walls remain in Gazebo but
are not permanently marked as obstacles within the planning map.

**Important:** The Gazebo AprilTag model generator currently renders its plates
without physical collisions. This lab conservatively treats their mounted
footprint as occupied in Nav2, without changing the main simulation's tag
models.

The planner uses `odom` as its world-aligned global frame. The lab **always
spawns the physical robot at world (0,0,0)** for that alignment. The requested
planner start is a *virtual pose* anywhere inside the venue. If you want to
move the sensor during obstacle discovery, that belongs in a later integration
test; here the RealSense remains at the stationary robot's actual location.

### Static vs depth-only dynamic obstacles

The web **Test obstacles** tab spawns/removes kinematically fixed boxes or
human-shaped solid fixtures. A human can move in a simple side-to-side route
using Gazebo's `set_pose` service. This is a collision-enabled approximation
of a person, not a skeletal animation or pedestrian agent.

Boxes and people are **never inserted into the static map or global costmap
by the web app**. They enter Nav2 solely if the actual depth camera sees them
and the existing scan pipeline marks them. They disappear from perception if
occluded or outside the camera FOV. Inspect the depth scan and global costmap
in RViz before benchmarking; an unseen obstacle is *not* a planner failure.

The static map is published by Nav2 map_server with frame `odom`; the global
costmap combines:

```text
StaticLayer(/map)
  + ObstacleLayer(/camera/camera/depth/scan)
  + InflationLayer(existing production footprint/inflation)
```

Use **one or more stationary boxes** for comparisons where all four planners
must see an identical costmap. A moving human is useful for testing costmap
responsiveness, but sequential planner requests may see different world states;
the benchmark marks the result when costmap fingerprints differ.

### What CPU and RAM mean

Planning time has two measurements:

- `planning_nav2_s`: Nav2-reported algorithm/server planning duration.
- `planning_wall_s`: action client round-trip wall time, including transport.

CPU and RSS are sampled with `psutil` on the **shared planner_server
process** while each action is executing, not on the entire Gazebo process.
Because all four plugins reside inside one server, the RSS is *not*
exclusively attributable to the selected planner and the CPU delta includes
costmap/ROS callbacks. A fully isolated per-plugin benchmark would run each
plugin in a separately restarted planner_server process and subtract baseline
cost; that is not implemented in this first phase. If `psutil` or the planner
PID cannot be resolved, resource values are unavailable rather than guessed.

### Saved artifacts

Each run gets a timestamped directory with:

- `calibration_snapshot.yaml`: exact validated saved Lighting, Camera, AprilTags, Mobility and Odometry inputs plus provenance;

- `config_snapshot.yaml`: start/goal, selected planner and depth settings;
- `nav2_params_snapshot.yaml`: exact generated planner configuration;
- `NavFnDijkstra.json`, `NavFnAStar.json`, `Smac2D.json`,
  `ThetaStar.json`: actual returned Nav2 path samples, for successful plans;
- `summary.json`: per-planner status, timing, length, clearance, angle changes,
  CPU/RAM and costmap fingerprints;
- `trial.log`: ROS action runtime logs.

The active generated calibrated court world and tag meshes, generated sensor/odom controller YAMLs, static map (PGM, YAML and extracted geometry), runtime
Nav2 YAML and RViz overlay cache live under
`evaluation_results/nav2_lab/_planning_active/`.

### ROS/Gazebo isolation and safety

The phase-1 launch uses `ROS_DOMAIN_ID=77` and the Gazebo partition
`scrobot_nav2_lab_77` by default. Both may be different in your workspace.
The lab refuses to launch if the selected ROS domain already has robot,
clock or map publishers.

The lab launches the existing robot controller/odometry/localization/perception
but **does not start Nav2's controller_server, FollowPath, NavigateToPose, or
the sweep mission**. No motion experiment is available in the web GUI.

### Inspect ROS directly

From another terminal after starting the lab:

```bash
cd ~/scrobot_ws && source install/setup.bash
export ROS_DOMAIN_ID=77
export GZ_PARTITION=scrobot_nav2_lab_77
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 topic hz /camera/camera/depth/scan
ros2 topic echo /map --once --qos-durability transient_local
ros2 topic echo /global_costmap/costmap --once
ros2 action list | grep compute_path_to_pose
```

The lifecycle servers should be `active`. Only after the map, depth scan and
planner are active should you judge a path. For test failures, inspect the
**Runtime logs** panel and the recorded trial/stack logs.

### Offline verification

```bash
python3 -m pytest -q tools/nav2_lab/tests tools/nav2_lab/planning/tests
python3 -m compileall -q tools/nav2_lab
```

CI compiles all Python sources and executes pure map/geometry/config tests,
but **does not execute Gazebo, RViz or Nav2 ROS lifecycle/integration tests**.
Physical camera, ROS action availability and Gazebo moving obstacle behavior
must be checked on the SCROBOT machine.

## Layout

```text
tools/nav2_lab/
  config/lab.yaml               # Lab settings; preserved production files
  web_server.py                 # Default entrypoint -> planning phase
  planning/
    map_builder.py              # Court geometry -> occupancy PGM/map yaml
    core.py                     # Planner-only Nav2 config + path metrics
    obstacles.py                # Spawn/remove/move Gazebo fixtures
    runner.py                   # ComputePathToPose + metrics
    path_display.py             # Publish retained paths in RViz
    web_server.py               # Planning-only local dashboard service
    web/                        # HTML/CSS/JavaScript UI
    config/planning.rviz        # RViz costmaps + planner paths
    launch/planning.launch.py   # Isolated planner-only stack
    tests/                      # Offline tests
```

The earlier experimental controller/integration modules remain in the
repository for later development, but are **not launched by Phase 1**.
