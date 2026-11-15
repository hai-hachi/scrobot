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

## Scene setup — one reusable court pose picker

The small SVG court is the same kind of *coordinate input* used by the
AprilTags Lab, **not** a browser replacement for RViz. Choose a placement mode
and click on the venue floor, or type exact values:

- **Robot pose:** X, Y, heading in degrees. A new live placement restarts the
  Nav2-only simulation to reset odometry and regenerate `map -> odom` TF.
  No controller commands are sent to move the robot.
- **Goal pose:** X, Y, heading in the fixed badminton-court `map` frame.
- **Cube:** select its center and side length **0.20–1.50 m** using the slider;
  a fixed collision + visual box is created in Gazebo.
- **Walking human:** pick independent A and B points, then a speed
  **0.10–2.00 m/s**. The fixture has a fixed 0.48 m diameter, 1.74 m height
  and walks A -> B -> A repeatedly. Speed and pause handling are based on
  **Gazebo /clock simulation seconds**. This is a simple collision-enabled
  human-shaped geometry, not animated human skeletal motion.
- **Active obstacles:** remove individual items or clear the scene, before or
  during a simulation (but not mid-benchmark).

Use **Save Scene YAML** to persist the complete robot/obstacle layout to:

```text
tools/configs/labs/nav2_scene.yaml
```

This is separate from the **Save Lab YAML** button, which saves planner
selection, goals and costmap parameters through the common lab registry at:

```text
tools/configs/labs/nav2_lab.yaml
```

Load Saved Scene works with simulation stopped. The scene file is *not* a
production publication. The scene is loaded when the Nav2 Lab starts.
At startup all saved obstacle models are inserted directly into the generated
Gazebo world, so they do not race the Gazebo create service during launch.

## RViz map/costmap visibility fixes

The previous code passed the global costmap width and height as Python
**floats**, despite Nav2 expecting integer metres for these parameters.
These are now integer values. The other important correction is the frame:

- The court occupancy image, map server and global costmap all use the
  **court-fixed `map` frame**.
- For robot spawn pose `(x, y, yaw)`, an explicit static
  `map -> odom` transform is published. The existing wheel odometry and
  localization continue to publish `odom -> base_footprint`.
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
The optional cubes/human **do not** go into the static map: they must enter
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
