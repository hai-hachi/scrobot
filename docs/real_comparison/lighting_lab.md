# Gazebo lighting and materials calibration lab

Branch: real-comparison. This is an **opt-in** simulator calibration workflow.
It does not change the original badminton_court.sdf, the main mission, the
physical Orin bringup, or the historical 50-shuttle benchmark.

## What you get

A responsive browser dashboard at http://127.0.0.1:8765 with **three genuine
Gazebo camera feeds in the same view**, plus interactive controls:

1. **Robot POV:** the existing simulated D435i RGB, not a fake rendering.
   Choose the robot pose with a clickable court map or X/Y/yaw inputs.
2. **Elevated broadcast view:** independent fixed RGB camera at
   (10.5, 0, 9.5) m, looking over the badminton court.
3. **Free camera:** independent movable Gazebo RGB camera. Drag in its
   image to rotate the view, use WASD / Q / E to move, or enter absolute
   X/Y/Z/yaw/pitch coordinates. Each change uses the Gazebo set_pose service.
4. **Light grid:** rows, columns, centered X/Y span, height, spot/point type,
   HSV color, intensity, specular fraction, range, constant/linear/quadratic
   attenuation, spot cone inner/outer angle, falloff and shadow toggle.
   Up to 32 lights to prevent accidental expensive shadow configurations.
5. **Surface and geometry:** HSV material controls for venue floor, court mat,
   safety walls and net poles. Pole diameter (15 to 150 mm) rebuilds the
   static net model with matching collision geometry.
6. **Shuttle layout:** replace previous lab batch with a fresh uniform,
   center-biased, or net-biased random layout. Number and seed selectable;
   existing scrobot_simulation's production shuttle model/spawner is reused.
7. **Performance:** real-time factor (RTF), simulation time, robot, overhead
   and free image FPS, CPU system load, and camera pixel clipping/brightness
   heuristics. These are *not* measured lux values.
8. **Presets:** save experiment settings as JSON for repeatable comparisons.

## Requirements

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (gz-sim8), a functioning SCROBOT
colcon build, and the local Python 3 Gazebo Transport bindings:

    sudo apt update
    sudo apt install python3-gz-transport13 python3-gz-msgs10 python3-pil

Python 3 must be the system /usr/bin/python3 or another interpreter that
can import gz.transport13, gz.msgs10 and PIL. The simulated robot's D435i
sensor is published natively over Gazebo Transport; the web UI does not
require a trained YOLO model or any Nav2 nodes.

## Start the first calibration session

**Stop existing Gazebo instances** before launching this scenario.

    cd ~/scrobot_ws/src
    git fetch origin
    git switch real-comparison
    git pull --ff-only

    cd ~/scrobot_ws
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install --packages-up-to scrobot_scenarios
    source install/setup.bash

    cd ~/scrobot_ws/src
    python3 -m unittest discover -s tools/lighting_lab -v
    python3 tools/lighting_lab/server.py

Then visit http://127.0.0.1:8765 (the script also tries to open your
default browser). **One command launches both the Gazebo simulation and the
local web control server.** It starts the standard simulation launch (world,
robot, bridges and cameras) and does **not** start Nav2, YOLO, the mission,
evaluation, RViz, or telemetry monitoring.

The Gazebo GUI is still available in its own window; the lab browser contains
its own three sensor views for side-by-side lighting comparison.

The dashboard may show "Waiting" for a few moments until Gazebo loads and
the robot is spawned. A missing camera feed is reported explicitly, not
replaced with a placeholder photograph.

For Cloudflare WARP/VPN, optionally use local-only transport in the shell
**before** starting the lab:

    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    export GZ_IP=127.0.0.1
    ros2 daemon stop
    python3 tools/lighting_lab/server.py

Gazebo and the browser backend must run on the same PC/transport network.
The HTTP dashboard is loopback-only (not exposed to the network).

Other options:

    python3 tools/lighting_lab/server.py --port 8766 --no-browser
    python3 tools/lighting_lab/server.py --preset /path/to/saved_preset.json

## Recommended lighting experiment

1. Keep the **same robot pose and shuttle seed** for the whole comparison.
2. Record baseline using 2 rows by 3 columns, 9.0 by 9.2 m X/Y span,
   7.0 m height, intensity 1.0, attenuation 0.45 / 0.015 / 0.001, shadows on.
   This recreates the original six symmetric side-luminaire positions.
3. Save a JSON preset named baseline_6_lights.
4. Vary intensity while keeping camera pose, number of lights, and shadows
   constant. Watch robot POV highlight clipping, dark pixels, and RTF.
5. Compare shadows on/off at identical settings. Avoid declaring "more
   accurate illumination" based solely on brightness, because brightness
   is not calibrated illuminance.
6. Try more/fewer lights, smaller/larger array span, other heights, spot
   cone angles and attenuation. Save each promising preset.
7. Compare shuttle contrast under realistic court/wall colors. Measure
   real D435i frames in the actual venue before selecting a final profile.

A low RTF means the machine cannot keep up with real-time simulation, NOT
that lighting is too dim. A high clipping fraction indicates a rendered
image with near-black or near-white pixels, NOT a physical lux measurement.
The best profile balances image realism, stable detection and render budget.

## How the live changes work

- Light configuration: Gazebo /world/badminton_court/light_config service;
  create/remove use the world's UserCommands services.
- HSV colors: Gazebo /world/badminton_court/visual_config service, applying
  real material values to existing visuals.
- Pole diameter: removes/recreates net_assembly using a copy of the original
  net SDF with modified visual **and collision** cylinder radii. This is
  intentionally applied only when you click Apply (or turn on auto on release).
- Cameras/robot position: /world/badminton_court/set_pose; avoid issuing
  repeated commands at very high rates because it can overload SceneBroadcaster.
- Shuttle spawning: calls the original ros2 run scrobot_simulation spawn_shuttles.
- World creation: builds a fresh *temporary* copy of the base SDF with
  additional camera entities, generated lights and calibration materials.
  It does NOT edit installed or checked-in worlds.

A browser slider does not directly change the underlying physics model,
nor should it. Each Apply sends actual Gazebo Transport messages; failures
are shown as errors.

## Logs and output

- The Gazebo launch log is printed at startup (in a temporary directory).
  It is removed when the lab exits, so copy it first if investigating failure.
- Presets are written under
  ~/scrobot_ws/evaluation_results/lighting_lab/presets/<name>.json.
- The browser uses the same generated simulation session. Saving a preset
  makes it possible to launch the *next* session with identical parameters.
- Neither experimental values nor generated SDFs are committed into normal
  simulation defaults until validated against the real D435i camera.

## Limitations and next tests

- A live Gazebo/GPU smoke test on the SCROBOT PC is required. CI checks
  generator logic and Python syntax, not rendering or camera pose services.
- Gz rendering and sensor camera data may have different auto-exposure and
  material responses than a physical D435i. Do not calibrate using visual
  impressions alone; save paired real/sim camera frames.
- Free-camera navigation is discrete: mouse drag changes orientation,
  WASD/Q/E changes pose in the dashboard. It is not the native Gazebo
  3D-view mouse controller.
- Image transport is compressed to JPEG for display. Calibration metrics
  use decoded Gazebo RGB before the dashboard's final JPEG compression.
- No artificial illuminance/lux values are calculated.
- The standalone lab is a calibration test, not the 50-shuttle regression.
  Use the original full_mission_check scenario after accepting any model
  changes into an explicitly named profile.
