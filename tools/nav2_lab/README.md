# SCROBOT Nav2 Lab — Phase 1 (path planning)

**Branch:** `real-comparison`. Only the Nav2 **global planning** phase is
enabled; the robot is stationary while paths are computed. Controllers and
NavigateToPose integration are deferred.

## Start

```bash
cd ~/scrobot_ws
git switch real-comparison
git pull --ff-only origin real-comparison
sudo apt install ros-jazzy-navigation2 python3-yaml python3-psutil
colcon build --symlink-install
source install/setup.bash
python3 tools/nav2_lab/server.py
```

Open **http://127.0.0.1:8772**. The UI is **one page**: scene setup,
planner selection, map diagnostics, calibration provenance, benchmarks, logs.
The simulation is headless; use the existing Open/Close Gazebo GUI and RViz
buttons to inspect the world without affecting the server.

## Planner tab and scene controls

The navigation tab is named **Planner**. Scene setup, planner selection,
performance, results and logs remain on one page.

- Choose Robot or Goal and click the court to set X/Y, then **drag in the
  heading direction** to set yaw. This reuses the gesture from YOLO Lab.
- Choose Cube, click its center and select side length 0.20 to 1.50 m.
- Choose Moving cylinder and click A, then B to define the current route.
  Later clicks replace A and then B of **the same preview** (the two latest
  selected points). No model is created on a court click. Press **Add cylinder**
  to spawn exactly one object, just like Add cube. Each is 1.74 m tall and
  0.48 m in diameter; the speed slider supports 0.10–2.00 m per simulation
  second, based on Gazebo /clock. Numeric A/B input also remains available.
- Use the **Cell size** numeric field (0.02 to 0.20 m per cell) for the
  planner costmap. Changing it requires restarting the simulation.

**Save Scene YAML** writes tools/configs/labs/nav2_scene.yaml (robot pose,
cube dimensions, cylinder endpoints and speed). **Save Lab YAML** writes
tools/configs/labs/nav2_lab.yaml (planner options, goal, cell size, depth
layer). Neither button publishes to the production simulation.

## Lightweight planner-only launch

The former full simulation/controller/localization/perception graph is
replaced by a dedicated minimal launch. The active ROS nodes are:

- Headless Gazebo and the existing robot spawn / robot_state_publisher
- ROS/Gazebo bridge for /clock and native depth PointCloud2 only
- pointcloud_to_laserscan and the existing depth self-filter
- Two fixed frame publishers for map -> odom -> base_footprint
- Nav2 map_server, planner_server and lifecycle manager
- Path overlay publisher and one combined map/topic-rate/RViz monitor

No diff-drive controller spawners, command mux, velocity smoother,
collision monitor, EKF, IMU filter, AprilTag detector, YOLO, color image
bridge or depth-to-color alignment nodes are started. The underlying Gazebo
robot model still contains its color sensor and gz_ros2_control plugin;
removing those would require a separate lab-specific URDF variant.

**Performance** measures actual wall-clock message Hz for /clock, depth
cloud, raw scan, filtered scan, /map and the global costmap. The /map
rate is displayed as **Latched** instead of inventing a periodic rate.
ANSI color escape codes are stripped from all four log windows.

## RViz map/costmap visibility fixes

The previous code passed the global costmap width and height as Python
**floats**, despite Nav2 expecting integer metres for these parameters.
These are now integer values. The other important correction is the frame:

- The court occupancy image, map server and global costmap all use the
  **court-fixed `map` frame**.
- For robot spawn pose `(x, y, yaw)`, an explicit static
  `map -> odom` transform is published. The stationary planner publishes an additional static
  `odom -> base_footprint` frame, without starting wheel odometry or EKF.
- RViz now displays a **simplified robot body** via RobotModel (lightweight
  URDF proxy visual), plus the **actual Nav2 global-costmap collision
  footprint** as a magenta polygon. This polygon is the robot footprint
  configured for Nav2 collision checks; it is not the inflated costmap radius
  or an estimate from the STL mesh.
- Court lines are bright white; the court outline is lime and the outermost
  venue clearance boundary is pink. Static map opacity is **0.30**, global
  costmap opacity **0.22**, both drawn behind the high-contrast overlays.
  Robot and goal heading arrows, camera-link marker and four planner paths
  remain visible.
- RViz's Fixed Frame is `map`, with an independently configured static
  `/map` display (reliable/transient-local QoS) and live
  `/global_costmap/costmap` display (reliable/volatile QoS).
- The Nav2 static layer reads `/map` using transient-local QoS.
  The map and live costmap have independent display layers and opacity.
- `tools/nav2_lab/planning/map_status.py` subscribes to the actual
  `/map`, `/global_costmap/costmap`, `/clock` and TF. Its results appear in
  **RViz map readiness** on the dashboard; no success is claimed merely
  because RViz opened.

**A typical first check:** Start the simulation; allow map_server and
planner_server to activate; open RViz. Confirm three dashboard indicators:
`/map: Receiving`, `global costmap: Receiving`, and
`map -> odom -> robot: Receiving`. If any indicator remains Waiting,
read its specific hint and the stack log below the dashboard.

The *permanent* occupancy map marks only the net poles and conservative
AprilTag mount footprints. Court paint and the suspended net are not
navigation obstacles. The physical floor remains a support collision.
The optional cubes/cylinders **do not** go into the static map: they must enter
the obstacle layer through the calibrated RealSense depth pipeline and be
within its actual field of view/range.

### Manual ROS checks if the displays are still empty

Use a second terminal **in this lab's ROS domain**:

```bash
cd ~/scrobot_ws
source install/setup.bash
export ROS_DOMAIN_ID=77
export GZ_PARTITION=scrobot_nav2_lab_77

ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 topic info -v /map
ros2 topic info -v /global_costmap/costmap
ros2 topic echo /map --once --qos-durability transient_local
ros2 topic echo /global_costmap/costmap --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic hz /camera/camera/depth/scan
```

The map/costmap indicator reports topic reception rather than unverified
RViz rendering. If **all** receive yet the picture is blank, inspect the RViz
display's status (QoS and fixed frame) and the RViz log. This code was
tested offline but not run inside your local Gazebo desktop.

## Saved calibration sources

This lab **reads the YAML saved in the owning lab**, without requiring an
upstream Publish to the main simulation. The Nav2 Lab never rewrites those
files.

| Lab | Source | How Nav2 consumes it |
| --- | --- | --- |
| Lighting | `tools/configs/labs/lighting_lab.yaml` | Court color, lighting, shadows, floor/venue dimensions |
| Camera | `tools/configs/labs/camera_lab.yaml` | D435i RGB/depth resolutions/FPS; FOV unchanged |
| AprilTags | `tools/configs/labs/apriltags_lab.yaml` | Canonical meshes and tag geometry regenerated |
| Mobility | `tools/configs/labs/mobility_lab.yaml` | Wheel contact/physics calibration; motor dynamics reserved for tracking |
| Odometry | `tools/configs/labs/odometry_lab.yaml` | Odom/IMU/magnetometer/ground-truth/EKF rates |

The currently committed Camera Lab settings are RGB **1920×1080 @30 FPS**,
depth **1280×720 @30 FPS**. The lab uses **your locally saved files** if
these differ. If Mobility or Odometry has never been saved, the lab visibly
labels a fallback to their *published* simulation YAML rather than silently
claiming it is a saved calibration.

Every launch freezes this set of inputs and produces:

```text
evaluation_results/nav2_lab/_planning_active/
  calibration_snapshot.yaml
  nav2_planning.yaml
  map_status.json
  static_map/court_static.pgm
  static_map/court_static.yaml
  nav2_saved_calibration_court.sdf
  paths/
```

Each benchmark further saves a `scene_snapshot.yaml`,
`config_snapshot.yaml`, `calibration_snapshot.yaml`, actual returned
Nav2 paths, planner CPU/RAM and timing results, and logs.

If an upstream saved YAML changes, restart the simulator to apply a new
snapshot. The main `src/` simulation and all other labs remain unchanged.

## Continuous live replanning

Choose **Live replan — all four planners** and a refresh interval
(0.5–10 seconds; default **2 seconds**), then press **Start live replanning**.
This starts one persistent action client which invokes NavFn Dijkstra,
NavFn A*, Smac 2D, and Theta* **sequentially** every cycle. Each new
successful Nav2 result replaces that planner's retained RViz path.
A failed planner clears its stale path. The lab records latest planning
latency/path length/clearance, completed cycles, wall-clock cycle duration,
success/failure counts and changing-costmap fingerprints.

The robot **never moves** in this test, and a moving cylinder is registered
as an obstacle only if the real simulated depth-camera pipeline observes it.
Because the four requests are sequential, a person may move and change the
costmap between them: this mode measures **adaptation to new perception**,
not a strictly controlled same-costmap planner benchmark.

Use **Stop live planning** to end the loop. Cubes/cylinders can be
added/removed and the scene YAML saved while live planning is running.
To change the goal, robot pose, costmap resolution or depth-layer mode,
stop live planning first (and restart simulation for changes requiring
a new map/robot pose). The requested interval is a **minimum cycle period**:
if all four planners take longer, they are not queued concurrently.

## Performance diagnostics (why depth is below 30 Hz)

The **Performance** section now has two related measurement blocks:

1. Topic arrival rates for /clock, native depth PointCloud2, raw LaserScan,
   filtered LaserScan, /map and the live global costmap.
2. CPU (% of **one** logical core, so multithreaded processes can exceed
   100%) and resident RAM (RSS in MB) for **this lab's** Gazebo server,
   depth bridge, pointcloud-to-laserscan, depth scan filter, Nav2 planner,
   map server, robot_state_publisher and RViz if opened. There are also
   totals and whole-system CPU/RAM. Requires `python3-psutil`.

**Gazebo real-time factor** is computed from /clock versus wall-clock time.
If Gazebo runs at 0.5× speed and your Camera Lab calibration is 30
simulated FPS, approximately **15 wall-clock depth FPS** is expected, even if
the sensor is configured correctly. Compare:

- **Depth cloud Hz** against `depth_fps × real_time_factor`. A deficit at
  this first stage suggests Gazebo rendering or bridge throughput.
- **Raw scan Hz** against depth cloud Hz. A significant drop here suggests
  pointcloud-to-laserscan conversion, transform availability or scheduling.
- **Filtered scan Hz** against raw scan Hz. A drop here suggests self-filter
  or scheduling overhead.

The diagnostics are **heuristics**, not proof of a particular bottleneck.
A high Gazebo CPU rate suggests costly simulation but a depth camera is
often **GPU-render-bound**, which CPU and RAM alone cannot confirm. Monitor
the GPU separately (e.g. `nvidia-smi` on a supported NVIDIA driver).
The Python monitor uses ROS serialized subscription for the large depth cloud
to avoid unnecessary Python PointCloud2 deserialization.

### RViz style

Robot and goal markers use explicit two-point RViz ARROW geometry: **0.35 m
shaft + 0.35 m taper**, shaft diameter **0.10 m** (one third thinner than
the former 0.15 m), pointed head diameter **0.16 m**. The four planner
paths are now **0.08 m** wide instead of 0.035 m. These are overlay
settings only, not robot dimensions or collision geometry.

## Planner benchmark

Choose NavFn Dijkstra, NavFn A*, Smac 2D, Theta*, or **all four**.
The runner uses actual `ComputePathToPose` requests, explicit start/goal
in `map` and costmaps created through real Nav2 plugins. The robot does
not drive. For fair comparisons, leave any moving human stopped or remove
it so costmap snapshots remain consistent across requests.

Measurements: planner-reported algorithm time, action roundtrip,
path length, turning, lethal-cell clearance, process CPU-time delta,
shared planner_server RSS / RSS delta, success/error, and a costmap
fingerprint. These are **shared process** RAM numbers, not per-plugin
exclusive allocations. For a strictly isolated per-algorithm memory test
we would restart the Nav2 process between trials.

## Test and limitations

```bash
python3 -m pytest -q tools/nav2_lab/tests tools/nav2_lab/planning/tests
python3 -m compileall -q tools/nav2_lab
```

CI covers Python, JavaScript, calibration sourcing, generated scene
geometry, YAML validation and planner configuration. It **cannot**
validate Gazebo runtime services, actual RealSense obstacle observation
or RViz rendering on your workstation. Use the readiness indicators
and the debug commands above for the first actual run.

The Nav2 planner lab currently runs in isolated `ROS_DOMAIN_ID=77`.
Keep real robot control off that domain. Do not start a separate full
Nav2 mission stack in the same domain.
