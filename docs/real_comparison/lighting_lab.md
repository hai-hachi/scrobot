# SCROBOT Lighting Lab: Gazebo Harmonic calibration

The **real-comparison** branch adds a local three-camera lighting and materials
calibration dashboard. It is a separate, optional experiment. Its
regular adjustments operate on a temporary derivative world. Only the
explicit **Apply saved YAML to main simulation** action changes the Git-tracked
production world SDF. The robot, Nav2 and mission code are not modified.

## What is adjustable

| Area | Control | Defaults |
| --- | --- | --- |
| Light grid | Rows × columns (max 32 lights), spot/point type | **2 × 4 spot** |
| X/Y light reach | Distance from court center **to outermost fixtures**, each side; Y requires ≥2 rows | ±4.5 m X, ±4.6 m Y |
| Light elevation | Z above venue floor | 7.0 m |
| Light XYZ frames | GUI-only RGB axes at fixture locations with yellow floor-projection guides | Shown by default; togglable |
| Light HSV color | Hue (0–360°), saturation (0–100%), value (0–100%) | H=55°, S=3%, V=95% |
| Light intensity | Gazebo light intensity 0–4× (not calibrated lux) | 1× |
| Light rendering | Per-array shadow-caster budget, global shadow toggle, range, attenuation, spot cone, falloff, specular | **2 of 8** lights cast shadows; all 8 still illuminate |
| Badminton net | Net mesh and white top tape do not cast shadows | Shadows disabled for these two visuals |
| Surface colors | Independent HSV sliders for venue floor, court mat, safety walls, net poles | Original appearance approximations |
| Venue X clearance | **Per side** beyond the 15.4 m-long mat | 2.3 m |
| Venue Y clearance | **Per side** beyond the 7.2 m-wide mat | 2.4 m |
| Net poles | Original fixed 50 mm diameter | Not adjustable |
| Shuttle scene | Seeded random uniform / center / net distributions; selectable count | 20, seed 20261003 |
| Performance | RTF chart, raw sensor vs browser preview FPS, JPEG processing time, image age, image clipping, CPU load, device-wide GPU utilization/VRAM, downloadable JSON samples | Real sensor/host metrics; **not** GUI FPS |
| Gazebo inspection rendering | Lean/full overhead/free camera profiles and low-detail/full/hidden robot visuals | **Lean 640×360 @ 4 Hz** each; simple robot proxy; D435i depth off |

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

## Fast-rendering eight-light baseline

The tracked `lighting_lab.yaml` and production `badminton_court.sdf` use
**2 rows × 4 columns**, X reach **±4.5 m** and Y reach **±4.6 m**.
All eight lights illuminate the court, but just **two center-biased,
diagonally distributed fixtures cast shadows**. In the default 2×4 grid
these are `lab_light_00_01` and `lab_light_01_02`, corresponding to
`court_light_00_01` and `court_light_01_02` in production.

Adjust **Shadow-casting lights** (0–32) to compare 0, 1, 2, 4, or all eight.
The global **Allow selected lights to cast shadows** checkbox disables all
shadows without disabling illumination. The effective caster count cannot
exceed the number of spawned lights. An explicit temporary shadow-free
preview overrides the caster budget only in the running scene and never
changes the saved calibration. Lower caster count generally saves more
rendering work than adjusting HSV, intensity or a matte material.

The production court lights now point directly **downward (0,0,−1)**.
Court and venue floor plus perimeter-wall materials use plain matte diffuse
colors with zero specular; no new textures, PBR maps or scene geometry were
introduced. The court markings, shuttle models and net-post highlights
remain unchanged.

## Light XYZ frames and Y-range calibration

With **Show light XYZ frames in Gazebo GUI** enabled, the Gazebo GUI
MarkerManager draws an XYZ axis at each light position, plus a yellow
vertical guide down to the venue floor. Frames are markers, **not physical
models**, so they do not affect collision, the lab camera sensors, shadows,
saved YAML or the production SDF. They can be hidden in the dashboard.

The **Applied frame positions** readout shows the last successfully
applied light locations. For **2 rows** and Y reach **6 m**, Y positions
must be **−6 m** and **+6 m**. For **1 row**, every light must be at
**Y=0** regardless of the Y-reach slider; with **1 column**, X is likewise
fixed at 0. This is necessary to keep a single row/column centered.

Spotlights now aim **vertically down** along (0,0,−1), instead of aiming
inward toward Y=0. This makes the illuminated pools align more naturally
with the slider positions. Spot cone angle, attenuation and mounting height
still determine the actual footprint on the floor; Y reach describes
fixture positions, not the pool boundaries.

The frame markers are shown only when Gazebo's GUI provides its
`/marker_array` service. If frames are missing, consult the dashboard's
**Frame overlay** status, ensure the Gazebo GUI is open with the Scene
Manager, and verify it with `gz service -l | grep marker_array`. The
frame marker worker is asynchronous and never blocks lighting commands.
If the GUI is closed, the dashboard and normal simulation still work.

**Why Gazebo's 3D UI may lag more than the dashboard:** Gazebo's desktop
scene renders all shaded objects and shadow maps in its own client-side Ogre2
scene, while the browser displays compressed and rate-limited camera JPEGs.
A seemingly smooth dashboard does not prove the Gazebo 3D renderer is fast.
In addition, GUI orbit-follow previously sent `set_pose` requests roughly
2.2 times per second; those can generate large scene broadcasts. The
follow loop now waits briefly for camera motion to settle and updates at
most once per second to avoid feeding back changes while you drag.

If the GUI still stutters, compare it with (1) orbit follow disabled,
(2) temporarily disabled shadows in Fast Preview, (3) fewer active lights,
and (4) light frames hidden. Watch GPU utilization and RTF; the GUI
renderer has no separate FPS meter in this dashboard. The lab's Fast
Preview option only reduces browser JPEG work, **not** the desktop GUI
rendering resolution or Gazebo sensor load.

## Three simultaneous camera views

- **Robot POV:** the actual simulated D435i RGB camera. Click on the court
  plan or type X/Y/yaw to place the robot.
- **Broadcast:** fixed elevated end-court Gazebo camera for observing
  lighting coverage and shadows.
- **Free inspector camera:** independently rendered Gazebo camera controlled
  by the Gazebo desktop orbit view. **Copy current orbit view** transfers
  the Gazebo GUI camera pose to this view. Check **Follow orbit live**
  to keep the inspector following its moving/orbiting camera, sampling
  at most once a second after movement settles. The dashboard no longer shows manual XYZ,
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
~/scrobot_ws/src/tools/configs/labs/lighting_lab.yaml
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
the calibration. Camera orbit poses and shuttle layouts remain in JSON
presets rather than this shared YAML.

### Apply a saved Lighting Lab result to the main simulation

After obtaining a good result in the lab:

1. Click **Update lab config YAML** to save the sliders.
2. Click **Apply saved YAML to main simulation** and confirm.
3. Rebuild/re-source the simulator if necessary and **restart the normal
   Gazebo simulation**. The already-running lab or normal Gazebo instance
   does not hot-reload its source SDF.

The second button reads only the saved
`tools/configs/labs/lighting_lab.yaml`, validates it, then
**atomically updates** `src/scrobot_simulation/worlds/badminton_court.sdf`.
It transfers the complete light array (count, center-based positions, HSV
color, intensity, type, shadow settings and attenuation), venue and wall
dimensions, and the four material groups. The net mesh and white top
tape explicitly use `<cast_shadows>false</cast_shadows>` in both the
calibration and main simulation worlds. The net posts retain their
original shadow behavior.

The publisher does **not** import the lab's free inspector/broadcast
cameras, disable the production shuttle pickup plugin, change court
lines or AprilTag geometry, or modify robot controllers/mission logic.
Because floor/wall dimensions can affect motion near venue boundaries,
run the normal navigation and full-mission regression before accepting
a new published calibration.

The production SDF is Git-tracked: review its diff before committing or
merging the `real-comparison` branch into `main`. Publishing edits your
current workspace file; it does not push a Git commit or merge branches.
There is also a command-line equivalent:

```bash
cd ~/scrobot_ws
python3 tools/lighting_lab/publish.py --check   # preview without writing
python3 tools/lighting_lab/publish.py           # apply saved YAML to SDF
colcon build --symlink-install --packages-select scrobot_simulation
source install/setup.bash
```

The branch's initial tracked YAML contains a *baseline*, not measured
post-experiment results. The normal simulation on this branch was
initialized with those baseline light/material values and net shadows
disabled. Saving and publishing a new experiment replaces that baseline.

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

## Actual Gazebo sensor and robot rendering budget

Use **Performance and presets → Actual Gazebo rendering budget** to select
the inspection sensor profile, robot visual detail and whether the physical
D435i depth sensor is enabled. This requires clicking **Apply render budget
· restart Gazebo** because changing sensor rates or robot geometry must
rebuild the derivative calibration world. Current shuttles are cleared.

| Mode | Overhead camera | Free camera | Robot body | Robot RGB | Robot depth |
| --- | --- | --- | --- | --- | --- |
| Lighting Lab default | 640×360 @ 4 Hz | 640×360 @ 4 Hz | Simple box proxy | **1280×720 @ 15 Hz** | Disabled |
| Full inspection (select in UI) | 960×540 @ 8 Hz | 800×450 @ 8 Hz | Selectable full STL | **1280×720 @ 15 Hz** | Optional |
| Normal production ROS 2 launch | No lab inspection sensors | No lab inspection sensors | Original full STL | **1280×720 @ 15 Hz** | **848×480 @ 15 Hz** |

The simple robot mode replaces expensive robot STL visuals with one low box
that remains below the D435i camera position. **Physical collisions,
wheel/caster joints, mass/inertia, motor controllers, TF frames, robot
spawn position, camera extrinsics and RGB calibration are unchanged.**
The D435i still sees the court and shuttles; if it sees the proxy at an
extreme view angle, select `none` to remove every robot visual only in the
lab. A missing robot in the Gazebo desktop view does not imply missing
physics or odometry.

The depth-sensor option disables actual simulated depth rendering and the
depth-to-color registration launch in the lab. Do not run perception tests
requiring depth in this mode. Normal production launches default to depth
enabled and full original robot visuals; these options are fully opt-in.
Commands used to spawn in the lab pass `enable_depth_camera:=false`
and `robot_visual_mode:=simple` by default. The user can also pass
`--inspection-profile full --robot-visual-mode full --depth-camera`
to `server.py` to start a full-quality experiment.

The **Download performance samples (JSON)** button exports up to the last
120 dashboard status samples (roughly two minutes), including simulated
RTF, raw RGB/inspection sensor rates, JPEG preview rates, frame age,
average JPEG processing milliseconds, host CPU load, device-wide NVIDIA
GPU utilization (if reported), shadow-caster count, and last Gazebo command
timings. The Gazebo desktop **GUI FPS is not directly measured** by this
dashboard. Use one consistent robot pose and shuttle layout when comparing
scenarios.

Note that lowering dashboard JPEG quality does **not** reduce Gazebo
sensor-rendering work; the lean inspection profile and disabling unused
depth do. The camera options affect the **lab derivative**, not the
tracked production world or real robot. To return to faithful sensor
comparisons, re-enable depth when needed, use the full inspection cameras
and restore full robot visuals if robot-generated shadows/occlusion matter.

## Interactive preview performance

The Lighting Lab has two **dashboard preview-quality modes**, independent
of the Gazebo sensor configuration:

- **Fast** (default): previews are JPEG 60, resized to at most 640 × 360
  and decoded at a reduced rate. Gazebo sensor frequencies, resolutions,
  camera geometry, and physics remain unchanged.
- **Accurate**: larger JPEG 82 at up to 860 × 490, with a faster display
  refresh. This is a dashboard preview-quality setting, **not** a substitute
  for saving the original camera stream for pixel-level comparison.

**Temporarily disable light shadows** in Fast mode to check whether
shadow rendering dominates your latency. This is applied to the running
Gazebo lights, not to the saved calibration shadow toggle, YAML, or main SDF.
Turn this option off before visually assessing shadow realism or capturing
a reference screenshot. Switching to Accurate mode automatically restores
the saved shadow setting. The final comparison must use calibrated shadows.

The **Last lighting update** and **Last material update** timing labels
show total server elapsed time, number of synchronous Gazebo service
requests and slowest individual service request. A slow command time
with normal real-time factor suggests service latency. A low real-time
factor that recovers when shadows are off suggests rendering pressure.
These metrics are diagnostic, not calibrated physical measurements.

The dashboard now coalesces slider updates: it sends at most one HTTP
request at a time per light/material category, replacing pending slider
values with the newest state. The server also rejects superseded requests
queued by other tabs and skips identical settings. Material changes send
updates only for changed material groups. Gazebo light updates still send
a complete message per light because partial light messages are not used
without confirmed supported semantics. Camera preview requests no longer
overlap for the same camera.

## Recommended first real-comparison experiment

1. Fix the robot pose, camera settings, shuttle seed and venue clearance.
2. Use **2 × 4** spotlights, **2 shadow casters**, **reach X=4.5 m**,
   **reach Y=4.6 m**, **height=7 m**, **H=55°, S=3%, V=95%**, **intensity=1×**.
3. Spawn 20 random shuttles, seed 20261003. Save preset
   `baseline_8_lights_2_shadows`.
4. Change only **intensity** while keeping HSV, camera pose and light count
   constant. Compare image clipping and mean brightness with RTF.
5. Adjust **H/S/V** at fixed intensity to compare light color. HSV models
   appearance, **not actual lamp spectral response or measured lux**.
6. Compare **0, 2, 4 and 8 shadow-casting lights** without changing any
   other parameter; use Download performance samples (JSON) to retain RTF,
   sensor FPS, GPU load and JPEG processing timings.
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

The lab generates a temporary derivative SDF and disables automatic shuttle
pickup only in that calibration-only world. Its live light/material
endpoints update Gazebo world objects, not source files. Only the explicit
**Apply saved YAML to main simulation** action changes the production
`badminton_court.sdf` on disk. The net geometry, net posts and collision
model are unchanged when shadow casting is disabled on its mesh and tape.
Venue resizing restarts the simulated lab world as described above.
Older Kelvin/brightness presets are converted to HSV/intensity; older
HSV and full-span light presets remain supported and the latter are
migrated to center-based reaches.

Offline CI validates source generation, YAML persistence, scene geometry,
production SDF publishing, pickup/plugin preservation, net shadow settings,
HSV/intensity conversion, browser startup and Python 3 syntax. **Gazebo Transport runtime service
behavior, compiled Xacro launch behavior, GUI orbit synchronization,
rendering performance and WARP behavior still require a hands-on run on
the SCROBOT workstation.**
