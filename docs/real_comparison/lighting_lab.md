# SCROBOT Lighting Lab: Gazebo Harmonic calibration

The **real-comparison** branch adds a local three-camera lighting and materials
calibration dashboard. It is a separate, optional experiment. It never changes
the checked-in competition world, the actual physical robot, Nav2, or the
historical 50-shuttle mission settings.

## What is adjustable

| Area | Control | Defaults |
| --- | --- | --- |
| Light grid | Rows × columns (max 32 lights), spot/point type | 2 × 3 spot |
| X/Y light reach | Distance from court center **to outermost lights**, each side | ±4.5 m X, ±4.6 m Y |
| Light elevation | Z above venue floor | 7.0 m |
| Light HSV color | Hue (0–360°), saturation (0–100%), value (0–100%) | H=55°, S=3%, V=95% |
| Light intensity | Gazebo light intensity 0–4× (not calibrated lux) | 1× |
| Light rendering | Shadow toggle, range, attenuation, spot inner/outer cone, falloff, specular | Original court approximations |
| Surface colors | Independent HSV sliders for venue floor, court mat, safety walls, net poles | Original appearance approximations |
| Venue X clearance | **Per side** beyond the 15.4 m-long mat | 2.3 m |
| Venue Y clearance | **Per side** beyond the 7.2 m-wide mat | 2.4 m |
| Net poles | Original fixed 50 mm diameter | Not adjustable |
| Shuttle scene | Seeded random uniform / center / net distributions; selectable count | 20, seed 20261003 |
| Performance | RTF chart, actual sensor FPS, image clipping, CPU load, GPU utilization/VRAM if available | Measured from Gazebo |

**Venue geometry:** venue floor X length = `15.4 + 2 × clearance_x`
meters and Y width = `7.2 + 2 × clearance_y` meters. The collision floor,
visible floor and four safety walls are generated consistently. The
badminton markings, mat, pole geometry, court tag positions, robot geometry
and pickup logic in the production world are unchanged.

**Rebuilding venue:** Click **Apply venue · restart Gazebo**. The dashboard
stays open, but the lab simulation is restarted to avoid dynamically changing
the robot's supporting collision floor. Existing random shuttle models are
cleared; use **Replace shuttle layout** to spawn them again. Lighting and
material settings are retained.

## Three simultaneous camera views

- **Robot POV:** the actual simulated D435i RGB camera. Click on the court
  plan or type X/Y/yaw to place the robot.
- **Broadcast:** fixed elevated end-court Gazebo camera for observing
  lighting coverage and shadows.
- **Free inspector camera:** independently rendered Gazebo camera controlled
  by the Gazebo desktop orbit view. **Copy current orbit view** transfers
  the Gazebo GUI camera pose to this view. Check **Follow orbit live**
  to keep the inspector following its moving/orbiting camera, sampling
  roughly twice a second. The dashboard no longer shows manual XYZ,
  yaw/pitch, drag, or WASD controls for the free inspector.
  It reads the documented `/gui/camera/pose` Gazebo Transport topic
  from the Gazebo Camera Tracking plugin. Gazebo desktop must be open and
  advertise that topic for orbit sync to work. If unavailable, the dashboard
  shows an actionable warning; the inspector retains its last/default pose.

Orbit sync copies world pose and quaternion, not necessarily field of view:
the inspection camera retains its own sensor FOV and output resolution.
The full quaternion (including roll) is preserved in saved JSON presets
and the derivative SDF when the lab restarts after a venue-clearance change.
This is a live camera view, not a screenshot of the desktop GUI.

## One-time setup: workspace-root tools

The Git repository itself is checked out at `~/scrobot_ws/src`. Git does
not allow files tracked in that repository to appear physically outside its
working tree. The one-time setup creates a **persistent symbolic link**
`~/scrobot_ws/tools` pointing to the tracked tools. No code is duplicated.
The link survives reboot, Git pulls and branch switches.

```bash
cd ~/scrobot_ws/src
git fetch origin
git switch real-comparison
git pull --ff-only
python3 scripts/setup_workspace_tools.py

cd ~/scrobot_ws
ls -l tools
```

If a real `~/scrobot_ws/tools/` directory already exists, the setup script
leaves your files untouched and links only missing tool entries. It refuses
to overwrite conflicts.

## ROS 2 / Gazebo dependencies

Ubuntu 24.04; ROS 2 Jazzy; Gazebo Harmonic; SCROBOT workspace; and Python 3
Gazebo Transport bindings:

```bash
sudo apt update
sudo apt install python3-gz-transport13 python3-gz-msgs10 python3-pil python3-yaml
cd ~/scrobot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to scrobot_scenarios
source install/setup.bash
```

Python 3 should be `/usr/bin/python3` or another interpreter able to import
`gz.transport13`, `gz.msgs10` and `PIL`.

## Start Lighting Lab

Stop other Gazebo simulation sessions first:

```bash
cd ~/scrobot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m unittest discover -s tools/lighting_lab -v
python3 tools/lighting_lab/server.py
```

Open <http://127.0.0.1:8765>. The Gazebo desktop GUI opens in its own window.
The dashboard shows the three **live Gazebo images** side by side. Nav2,
mission, YOLO, RViz, evaluator and the other developer monitors do not launch.

### Update lab config YAML

The **Update lab config YAML** button in **Performance and presets** saves
the *current light, material, and venue slider values* to this tracked file:

```text
~/scrobot_ws/src/src/scrobot_simulation/config/lighting_lab.yaml
```

The first `src` is your Git checkout; the second `src` belongs to the ROS
workspace layout inside the repository. When launched from
`~/scrobot_ws/tools/lighting_lab/server.py`, the same path is used.

The file is loaded automatically as the lab's defaults on each launch, so
the settings survive restarts. CLI `--preset` JSON values override the
YAML defaults for that session. **Updating YAML is not the same as applying
slider changes to the running world**: use **Apply lighting to Gazebo**,
**Apply material colors**, or **Apply venue · restart Gazebo** to change
the live simulation. To use freshly saved YAML as a full scene, restart
the Lighting Lab.

Saving YAML intentionally modifies a Git-tracked *lab-only* config in the
local working tree. Review or commit it with Git when you want to share
the calibration. No production SDF, hardware settings, or mission config
are overwritten. Camera orbit poses and shuttle layouts remain in JSON
presets rather than this shared YAML.

**WARP/VPN compatibility is automatic on every run, including after reboot.**
The Python launcher sets `GZ_IP=127.0.0.1` and
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` **before** creating any Gazebo
Transport node or ROS/Gazebo subprocess. It removes the deprecated
`ROS_LOCALHOST_ONLY` setting in the lab's process if inherited. It binds the
dashboard to `127.0.0.1`. There are no shell exports or startup-file edits
to redo after reboot. This setup assumes Gazebo and the Lighting Lab run
on the same computer, not in separate containers or remote machines.

Loopback configuration avoids interface selection via WARP but cannot
guarantee operation if OS firewall, routing, graphics drivers or Gazebo
server initialization itself is faulty. The standalone real robot remains
unaffected.

**For separate Gazebo diagnostic terminal commands** while WARP is active,
set the same scope in that terminal:

```bash
export GZ_IP=127.0.0.1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
gz topic -l | grep -E '/gui/camera/pose|/lighting_lab/'
gz service -l | grep '/world/badminton_court'
```

Other commands:

```bash
python3 tools/lighting_lab/server.py --port 8766 --no-browser
python3 tools/lighting_lab/server.py --preset /path/to/saved_preset.json
```

## Recommended first real-comparison experiment

1. Fix the robot pose, camera settings, shuttle seed and venue clearance.
2. Use **2 × 3** spotlights with **reach X=4.5 m**, **reach Y=4.6 m**,
   **height=7 m**, **H=55°, S=3%, V=95%**, **intensity=1×**.
3. Spawn 20 random shuttles, seed 20261003. Save preset
   `baseline_6_lights`.
4. Change only **intensity** while keeping HSV, camera pose and light count
   constant. Compare image clipping and mean brightness with RTF.
5. Adjust **H/S/V** at fixed intensity to compare light color. HSV models
   appearance, **not actual lamp spectral response or measured lux**.
6. Toggle shadows without changing lights to evaluate rendering cost.
7. Increase/decrease light rows, columns, center-based X/Y reach, height,
   attenuation and spotlight angle. Inspect the broadcast and robot POVs.
8. Adjust venue clearance only when you want to study the physical floor
   boundary; this restarts the lab. Re-spawn the same shuttle seed after it.

For any candidate, save a JSON preset and camera snapshots. Presets include
the Git SHA, source-world checksum, selected settings and a measurement
snapshot. They live in
`~/scrobot_ws/evaluation_results/lighting_lab/presets/`.

Actual performance (RTF, sensor frame rate, GPU load) measures compute cost;
image brightness and clipped-pixel statistics measure visual appearance,
**not photometric lux or YOLO pickup accuracy**.

## Safety and scope

The lab generates a temporary derivative SDF, disables automatic shuttle
pickup in that calibration-only world and never modifies the production
`badminton_court.sdf`. The light/material endpoints update actual Gazebo
world objects. Venue resizing restarts the simulated world as described
above. Older Kelvin/brightness presets are converted to HSV/intensity;
older HSV and full-span light presets remain supported and the latter are
migrated to center-based reaches.

Offline CI validates source generation, YAML persistence, scene geometry,
HSV/intensity conversion, browser startup and Python 3 syntax. **Gazebo Transport runtime service
behavior, GUI orbit synchronization, rendering performance and WARP behavior
still require a hands-on run on the SCROBOT workstation.**
