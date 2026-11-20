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
- `SMC_1S` and `SMC_2S`: experimental **full** C++ `nav2_core::Controller`
  laws with the same PP-circle carrot, body-frame `ex,ey,etheta`, nominal
  `vR,omegaR`, denominator `D`, longitudinal `gx`, and angular reaching
  `q`. See the exact mathematical definitions below.
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

## Four-box Controller tuning

The **Controller tuning** tab has separate cards for **Shared**, **RPP**,
**SMC-1s full**, and **SMC-2s full**. Selecting one controller highlights its
card while keeping the others available for fair follow-up comparisons.

The two full-SMC tuning cards now show **typeset LaTeX equations** for
their sliding surface(s), reaching corrections `g_x` and `q`, the
complete pre-clamp angular command `omega`, linear command `v`, and
conditional ideal reaching laws. The common equation guide covers
robot-frame tracking errors, the PP-circle geometry and curvature sampling,
nominal `vR/omegaR`, the saturation/boundary-layer function and switch,
the singularity guard, and every adjustable angular/acceleration limit.
The complete original TeX source is attached to each displayed formula.
Native browser **MathML works offline**; where online access permits, an
optional asynchronously loaded KaTeX renderer replaces it. Each SMC card
also shows its current parameter values, updated after edits, YAML loads,
or card resets. No control laws are changed by this display update.

Shared settings specify nominal reference speed (default 0.40m/s from the
source SMC lab), goal/progress checkers and a PP-circle lookahead radius
(default 0.60m) and three-point curvature half-window (0.14m) for both SMCs.
RPP uses its independent Nav2-regulated carrot algorithm with its own
lookahead settings; the speed reference is shared.

SMC-1s gains include `lambda_y,lambda_x,k_s,k_x,eta,phi,eta_x,phi_x`,
saturation switch, denominator guard and angular/acceleration limits.
SMC-2s has the same gains except `lambda_x`, which is not in its law.
Both are implemented as instances of the same C++ controller plugin and use
separate ROS parameter namespaces `SMC_1S` and `SMC_2S`.

Each card has an independent Reset button. **Save Lab YAML** persists all
tuning into `tools/configs/labs/tracking_lab.yaml`. Existing simplified
SMC saved settings are migrated to fresh full-law defaults, **not mapped
numerically into unrelated gains**; older experiment snapshots remain
untouched. No lab controller is published to production automatically.
Changing parameters requires a fresh Tracker run to apply them to Nav2.

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
scale for every curve. The chart legend identifies the controller, path,
individual saved run, and its RMSE when available. No per-run normalization
is performed; overlaying runs with different paths, starting poses, or
requested speeds displays a comparability warning.

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

## Full SMC-1s and SMC-2s (reference: hai-hachi/smc)

The Tracker runs **three** Nav2 FollowPath controllers: RPP, `SMC_1S`,
and `SMC_2S`. The latter two implement the **full** SMC laws from the
2026-11-20 Python lab summary, not the earlier simplified heading/lateral
controller. Their saved gains live under independent `smc_1s` and `smc_2s`
blocks. The shared `lookahead_radius_m` and `curvature_window_m` configure
the same robot-centered PP-circle reference generator for both SMC versions.

At each control cycle, the SMC plugin projects the robot on the current
path, finds the first forward intersection of that path and a circle centered
on the robot, computes a three-point tangent and signed curvature around the
carrot, and sets nominal `vR` to the shared reference speed (with an
open-path terminal slowdown) and `omegaR=vR*kappaR`. Fallbacks for
no-intersection conditions are explicitly marked. **RPP also uses a
carrot**, but uses Nav2's own regulated lookahead algorithm, not the SMC
fixed-radius circle. RPP's published `/lookahead_point` is captured for
replay when available.

Errors in the robot frame:

```text
ex     =  cos(theta) (xR-x) + sin(theta) (yR-y)
ey     = -sin(theta) (xR-x) + cos(theta) (yR-y)
etheta = wrap(thetaR-theta)
D      = 1 + lambda_y * ex

sat(z) = clamp(z,-1,1)
gx     = kx*ex + (use_saturation ? eta_x*sat(ex/phi_x) : 0)
q(s)   = ks*s  + (use_saturation ? eta  *sat(s/phi)   : 0)

SMC-1s:
  s     = etheta + lambda_y*ey + lambda_x*ex
  omega = (omegaR + lambda_y*vR*sin(etheta) - lambda_x*gx + q(s))/D
  v     = vR*cos(etheta) + ey*omega + gx

SMC-2s:
  s1    = ex
  s2    = etheta + lambda_y*ey
  omega = (omegaR + lambda_y*vR*sin(etheta) + q(s2))/D
  v     = vR*cos(etheta) + ey*omega + gx
```

The plugin explicitly fails if `|D|` is smaller than the configured guard
or changes sign. Both versions use the same motor/wheel envelope, speed and
acceleration limits **after** calculating the full control law. The source
Python SMC lab's free-space kinematic actuator limits `vmax=0.60m/s` differ
from the calibrated SCROBOT physical wheel limit, so this Gazebo lab retains
the more restrictive shared `|v|+0.21|omega|<=0.50m/s` rule.

**Critical theoretical limitation:** because a robot-centered PP carrot is
not guaranteed to satisfy the assumed unicycle reference kinematics, the
nominal `(vR,omegaR)` are not generally the actual carrot pose derivatives.
Even when it does satisfy those kinematics, actuator saturation invalidates
the ideal reaching behavior `ex_dot=-gx`, `s_dot=-q(s)`. The displayed
phase portraits plot *observed* motion and theoretical curves distinctly;
they are for comparison and diagnosis, not a proof of stability.

### Timeline telemetry and diagnostic portraits

New run directories include `timeline.json` with samples at about 10
simulated Hz. Each contains the robot pose and commands, carrot pose and
radius, nominal velocities, body-frame errors, and for SMC the actual
controller-computed surfaces, reaching terms, denominator `D`, and
pre-limiting commands. The SMC controller publishes
`/tracker/smc_carrot` and `/tracker/smc_diagnostics`; RPP publishes
`/lookahead_point`, which is transformed to world coordinates from the
robot's ground-truth pose. If the RPP topic is unavailable, the inspector
labels its geometric position and nominal reference fields as **estimated**,
not as the exact Nav2 carrot.

Select a saved run using its **Inspect** button or the timeline dropdown,
then scrub the slider. The view shows the reference polyline, robot,
carrot, **true circular robot-centered lookahead radius**, nominal
vR/omegaR, tracking errors, and sliding variables. The history map uses
**equal metres per SVG unit on X and Y**, numeric X/Y tick marks and
axis titles in metres. The view auto-fits the full recorded trajectory,
reference points and all recorded lookahead ranges without distorting circles.
Five plots show
`ey vs etheta`, `ex vs ey`, the angular surface against its observed derivative,
`ex vs ex_dot`, and `sat(z)` with boundary regions. Saved experiments
predating `timeline.json` remain comparable in the error chart, but their
Inspect button is disabled; measurements are never invented or reconstructed
as if they had been logged.

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

### Rounded 100 RPM physical drivetrain limit

Wheel diameter is **0.100 m** and the drive wheel-centre separation is
**0.420 m**, from the actual SCROBOT URDF. The rounded motor maximum is
**100 RPM**, which by geometry implies **0.52360 m/s** rim speed.
However, the separately specified maximum **0.50 m/s per wheel** is lower,
so that rating is the effective speed limit in the controller and router.
The maximum straight velocity is `0.50 m/s`, and the maximum pure spin is
`2*0.50/0.420 = 2.38095 rad/s`.

For simultaneous motion, the exact coupled drivetrain restriction is
`|v| + 0.210*|omega| <= 0.50 m/s`, equivalent to imposing both
`|v - 0.210*omega| <= 0.50` and `|v + 0.210*omega| <= 0.50`.
This is enforced in the common command router (RPP, SMC-1s, SMC-2s) and
internally in the full SMC plugins. Separate requested linear/angular
maxima do **not** mean both limits are reachable at the same time.
