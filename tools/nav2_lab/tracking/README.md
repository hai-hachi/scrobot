# SCROBOT Nav2 Tracker Lab — Phase 2

Branch: `real-comparison`. This lab compares **real Nav2 `FollowPath`
actions** using the **same calibrated physical Gazebo drivetrain**, reference
paths, initial pose and simulated-time clock. It is not a 2D kinematic toy.

## Launch

```bash
cd ~/scrobot_ws
git switch real-comparison
git pull --ff-only origin real-comparison
colcon build --symlink-install --packages-up-to scrobot_tracker_controller scrobot_simulation
source install/setup.bash
python3 tools/nav2_lab/server.py
```

Open **http://127.0.0.1:8772/tracker** or select **Tracker** in the Planner
Lab top navigation. Planner Lab stays at the root URL. Planner uses ROS
domain 77; Tracker **prefers domain 78, but automatically selects a free
domain from 78–89** if an earlier Tracker/Gazebo process or another ROS
participant is still active. Each Tracker session receives a distinct
`GZ_PARTITION`. The Tracker status shows the selected domain, and it does
**not** kill unrelated or orphaned ROS processes automatically.

The preflight probes `ros2 topic list --no-daemon` separately for each
candidate, avoiding false positives from ROS 2's cached discovery daemon.
If all candidate domains are occupied, the error names the domains and
observed topics rather than giving a generic "active simulator" message.

**If a previous run was interrupted:** Stop the old simulation from its
original Tracker UI if that server still exists. Otherwise inspect ROS domain
78 before cleaning up any processes:

```bash
ROS_DOMAIN_ID=78 ros2 topic list --no-daemon
ps -eo pid,ppid,pgid,cmd | grep -E 'tracking_world|tracker_|controller_server|gz sim'
```

Only stop process groups that you have positively identified as the old
Tracker launch. Never use a broad `pkill gz` or `killall ros2`, because
that would also affect the Planner and other simulation labs. Domain fallback
lets the new Tracker proceed without destroying someone else's run, but
orphaned processes still consume CPU/RAM until explicitly terminated.
When you press **Stop** in a server that still owns a Tracker session, the
lab now attempts to clean up any surviving descendants **only when both**
their randomized Gazebo partition and ROS domain match that particular
session. This protects Planner, Performance, and all other Gazebo instances.
An old session abandoned by an already terminated lab server has no retained
owner token in the new server, so it is left untouched and isolated instead
of being killed without verification.

**Important:** the Tracker requires the Performance Lab's verified
Nav2 Jazzy ROS-clock controller overlay, because stock Jazzy uses
`rclcpp::WallRate` for the controller loop. If the overlay is not installed
and sourced, starting the simulator fails with actionable instructions
instead of silently running 25 wall Hz at 0.25 RTF:

```bash
python3 tools/performance_lab/prepare_nav2_sim_clock.py --build
source install/setup.bash
```

## Tracking environment

The experiment creates `evaluation_results/nav2_lab/tracking/_tracking_active/
tracking_flat_world.sdf`. It contains a **single flat floor** and no
badminton court, net, shuttlecocks, expensive light array, camera rendering,
or scene complexity. Its contact physics matches the original court floor:
**0.020 m collision thickness at Z=0, mu=mu2=1.0, physics step 1 ms**.
The flat floor is widened to 30 × 30 m to contain all test paths and spawn
offsets; floor material color does not affect the physics.

The robot is the real `scrobot_simulation` xacro with unchanged mass,
inertial properties, wheel/caster joints, friction, contact width and
`gz_ros2_control` drivetrain. The lab reads saved Mobility and Odometry Lab
calibration YAMLs (explicit published fallback when unsaved), independent of
Camera, Lighting and AprilTag saves. It also reuses the existing
`scrobot_simulation/sim_motor_response` first-order wheel RPM response
surrogate with the saved `tau_left`, `tau_right`, and gain values whenever
the Mobility Lab's motor model is `first_order`. The `baseline` model
bypasses that surrogate and sends safe commands directly to ros2_control.
Both RPP and SMC pass through the **same selected motor model**.
The physical wheel feedback stays 100 simulated Hz.

**Deliberate isolation:** both controllers use a perfect simulation ground-
truth TF `odom -> base_footprint`, published from Gazebo's independent
`/evaluation/ground_truth_odom`. This isolates *controller tracking*
from localization error. The independent ground-truth stream also evaluates
RMSE. It is **not** proof of how production EKF performs; validate feedback
from estimated odometry later in a separate integration experiment.

No planner, BT navigator, global costmap, depth processing or YOLO is started.
The local costmap is free space and has **two valid Nav2 plugin IDs**:
`obstacle_layer` (with no observation sources and no depth subscriptions)
and `inflation_layer`. An empty `plugins: []` list causes a Jazzy
`InvalidParameterValueException` at ControllerServer startup; never
use it for the tracker. This compares following a known geometric
path, not collision avoidance.
The reference path is published in `odom`, with visualization overlays in
RViz's odom fixed frame.

## Four fixed paths

- Straight from (-4,0) to (4,0) m.
- Circle: radius 2.3 m, **open 300-degree arc**. We deliberately avoid a
  closed start==goal loop, which Nav2 may mark complete before one lap.
- Smooth sinusoidal S-curve with continuous curvature.
- Rectilinear path (-3.5,-2.8) -> (0,-2.8) -> (0,2.8) -> (3.5,2.8),
  retaining **exact 90-degree corners** without replacing them with arcs.

Waypoint spacing is <= 0.075 m. Yaw follows the local tangent.
The user can click/drag the preview map to pick x/y and yaw manually,
set path-relative progress/lateral/heading errors, or select a seeded
random initial pose. A seed reproduces the exact same pose.

**For RPP-vs-SMC fairness:** select one path, the same random seed or manual
pose, the same desired speed and physics, run RPP, then run SMC.
Each subsequent run respawns a fresh world at the same start rather than
teleporting the displaced robot without resetting wheel/odom state.

## Controllers

- `RPP`: genuine
  `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`,
  with the published `FollowPath` production settings as the baseline
  (and the explicitly chosen test speed). Collision detection is disabled
  solely because the empty test world has no obstacles.
- `SMC`: experimental C++ `nav2_core::Controller` plugin,
  `scrobot_tracker_controller::SMCController`, using a sliding surface
  combining body yaw and path-relative cross-track errors, bounded
  saturation in place of discontinuous `sign(s)`, a curvature lookahead
  feedforward, and a near-goal heading alignment step.
- Both are hosted by the same Nav2 ControllerServer at **25 simulated Hz**,
  with the same `FollowPath` action, progress/goal checkers and
  100 simulated Hz `diff_drive_controller`. Both send their
  `TwistStamped` commands through **the same** safety router, enforcing
  `|v| + 0.21|omega| <= 0.50 m/s`; the router scales linear and angular
  velocities together to preserve curvature. The number of raw requests
  requiring clipping is reported separately.

The lab uses the same `use_sim_time` synchronization contract as the
Performance Lab. The **Planner and Tracker tabs share one Gazebo speed cap**
in `tools/configs/labs/simulation_speed.yaml` (the canonical `nav2`
entry, default **0.25**). Both tabs display the **same control above the Lab
heading**. Saving in either tab updates the common value, but a running
Planner or Tracker keeps its own applied RTF cap until that specific Gazebo
simulation restarts. The panel shows both applied caps and highlights a
pending restart; it never restarts the other tab implicitly. Existing saved
`tracking` entries are supported for migration. The other labs' speed
settings remain independent. Controller/sensor settings stay in **simulation
Hz**, not divided by four.

The saved controller YAML under
`evaluation_results/nav2_lab/tracking/_tracking_active/tracking_controller.yaml`
is lab-only. **No controller is automatically published to production.**
The SMC algorithm and gains need real-world/controller-stability validation
before considering it for the normal mission stack.

## Tracker not moving / startup diagnostics

If Gazebo and the differential-drive controller are active but the robot
doesn't move, inspect the Tracker **Tracking simulation stack** log.
The `controller_server` subprocess can crash while the Gazebo process
continues, making a running simulator an invalid readiness indicator.

The Tracker now detects a fatal `controller_server` crash, shows it in
the interface, disables Run, and fails active trial startup immediately
instead of waiting 160 wall seconds. The trial also reports the live
`/evaluation/ground_truth_odom` and wheel-odom message counts and whether
the `/follow_path` action server is ready.

**FollowPath goal rejected right before activation:** Nav2 Jazzy advertises
the action endpoint while `controller_server` is still in the lifecycle
`activating` transition, so `ActionClient.server_is_ready()` is not enough
to safely send the goal. If the simulation logs show
`Action server is inactive. Rejecting the goal.` followed by
`Managed nodes are active`, this is a launch synchronization race, not
a failed RPP or wheel controller. The Tracker now polls
`/controller_server/get_state` (`lifecycle_msgs/srv/GetState`) and
requires lifecycle **ACTIVE**, adequate Gazebo ground truth and wheel odom,
and action endpoint availability before dispatch. It includes one bounded
retry if a lifecycle transition races that check. The trial view displays
the lifecycle state alongside the advertised action availability. It does
**not** alter configured control Hz or the 0.25 RTF synchronization.

For the previously observed error
`parameter_value_from failed for parameter 'plugins': No parameter value set`,
update the branch, **Stop** the old tracker, then **Start / Reset** so
`_tracking_active/tracking_controller.yaml` is regenerated with valid
free-space costmap plugins. Merely refreshing the browser does not regenerate
the YAML or revive a crashed Nav2 controller.

## Three-box Controller tuning

The **Controller tuning** tab presents three separate, responsive cards.
All three remain visible; the selected controller is highlighted and the
other controller can still be edited for later comparisons.

1. **Shared parameters:** reference speed, simulated trial timeout,
   Nav2 goal XY and yaw tolerances, progress-check radius and progress
   time allowance. These values apply equally to RPP and SMC, including
   the common wheel-speed safety envelope, which remains locked to the
   calibrated differential drive.
2. **RPP tuning:** fixed and velocity-scaled lookahead (minimum,
   maximum and lookahead time), curvature speed regulation, minimum
   regulated speed, rotate-to-heading controls, angular acceleration
   and final approach speed. Every field updates the corresponding
   `nav2_regulated_pure_pursuit_controller` parameter in the generated
   `tracking_controller.yaml`. Minimum RPP speeds are safely capped
   by the Shared desired forward speed.
3. **SMC tuning:** lateral and heading gains, sliding-surface heading
   weight (lambda), switching gain and continuous boundary layer,
   curvature lookahead, curvature feedforward gain, goal heading gain,
   final-heading approach threshold and maximum angular command.
   The SMC C++ plugin declares and uses all of these parameters.

Each card has its **own Reset** button. Reset changes only that category
in the browser (it does not discard the path/pose or change the other
controller). **Save Lab YAML** commits the settings to
`tools/configs/labs/tracking_lab.yaml`, and **Load Saved YAML** restores
them. Earlier YAML files with flat desired speed and trial timeout and
five original SMC gains are migrated automatically, preserving saved
values and filling only the new defaults. This does not publish anything
to production.

Changing tuning parameters requires a new controller_server configuration:
the Lab's **Run tracking** action regenerates the tracking-world
configuration and restarts the environment when settings differ from
the current launch. It does not dynamically reconfigure an already
running `FollowPath` action.

## Metrics and performance

Each run saves ground-truth samples, reported Nav2 action success/failure,
cross-track RMSE, heading RMSE, maximum error, travelled distance,
simulated duration, wheel-speed envelope checks and the full time series.
The cross-track error uses the nearest **path segment projection**, not
closest waypoint. Error integrals are weighted by actual simulation time
increments. A discontinuous path has undefined tangent precisely at a
90-degree corner; we use a deterministically selected adjacent segment.

The **Performance** tab reports measured simulation/wall Hz for `/clock`,
ground truth, wheel odometry, wheel joint states and safe controller command,
plus measured Gazebo RTF and per-process CPU/RSS. No configured frequency
is mistaken for observed throughput.

Results automatically save to `evaluation_results/nav2_lab/tracking/
YYYYMMDD_HHMMSS_tracking_xxxxx/`, containing calibration snapshots,
reference path, Nav2 controller YAML, configuration, summary, trace,
time series and trial logs. The Results tab compares RPP and SMC runs,
supports individual deletion and Delete all, while preserving active
`_tracking_active` and other labs' results.

**Buttons:** Save Lab YAML, Load Saved YAML, Start, Reset/Respawn, Run,
Cancel, Stop, Open/Close Gazebo, Open/Close RViz; all four log windows are
at the bottom. Saved settings live in
`tools/configs/labs/tracking_lab.yaml`.

## Saved-run error chart comparison

The **Run & Results** tab now plots **signed cross-track error** against
**elapsed simulated time** with numeric ticks, labelled axes (seconds and
metres), a visible zero-error baseline, and a shared automatically adjusted
scale for every curve. The chart legend identifies the controller, path and
individual saved run. No per-run normalization is performed; overlaying runs
with different paths is allowed but displays a comparability warning.

The **Saved comparisons** table has a **Plot** checkbox in each row.
Select up to eight completed runs to overlay their error curves; deselect
any run to remove it. The **Include current / latest run** toggle keeps the
live trace visible or hides it; selecting the current run's saved trace
avoids drawing it twice. **Clear plot selection** removes all overlays
without deleting any files. Deleting a saved experiment removes its overlay,
and history refreshes do not erase selected comparison curves.

The read-only endpoint `/api/tracker/history/trace?name=<run-id>`
loads `series.json` from the managed experiment directory. It verifies
the run identifier, rejects symlinks/path traversal, bounds payload size
and downsamples older large series. Only saved runs with finalized traces
offer an enabled Plot checkbox; initializing runs without `series.json`
remain listed but cannot be plotted. Saved historical data is not modified,
and choosing overlays does not launch Gazebo or affect control timing.

## Tests and limitations

```bash
python3 -m pytest -q tools/nav2_lab/tracking/tests
python3 -m compileall -q tools/nav2_lab/tracking
node --check tools/nav2_lab/tracking/web/app.js
```

The offline checks cover path geometry, random seed reproducibility,
coupled command limits, time-weighted RMSE, world contact physics,
planner settings, C++ plugin registration, REST and UI contracts.
The separate ROS Jazzy colcon workflow checks plugin compilation.
**Actual Gazebo controller integration, start/stop behavior and RPP-vs-SMC
tracking quality require an on-machine run; offline tests are not proof of
successful physics tracking.**
