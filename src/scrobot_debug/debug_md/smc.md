# SMC and AprilTag Approach Strategy Tests

This guide compares four AprilTag local-approach strategies and validates the
simplified shuttle SMC geometry.

## Shared pose convention

The local SMC controlled point is now the planar `base_link` pose itself:

```text
c = 0
```

The rigid camera offset is absorbed into the desired base pose instead of being
carried inside the controller model.

Current desired poses:

```text
AprilTag:
base_link = 0.90 m in front of the tag
heading   = directly toward the tag

Shuttle:
base_link = 1.10 m from the shuttle
heading   = directly toward the shuttle
then straight collect at 0.30 m/s
```

The local controller is best understood as two coupled parts:

```text
angular steering:
s = e_theta + lambda * e_y

omega =
    lambda * v_R * sin(e_theta)
  + k_s * s
  + eta * sat(s / phi)

forward translation:
v = k_rho * rho * cos(alpha)
```

The sliding-mode law primarily generates `omega`. The linear velocity is a
separate proportional pose-approach law. Definitions:

```text
rho   = Euclidean distance from current base_link XY to desired XY
alpha = angle from current robot heading to the desired XY position
k_rho = proportional gain converting position error into forward speed
v_R   = reference speed appearing inside the SMC angular law
```

`v_R` is not the same variable as the commanded `v`. During pose approach,
`v` naturally decreases as `rho -> 0`. For large `alpha`, translation is
stopped so the robot can correct its heading first.

AprilTag SMC baseline parameters remain:

```text
v_R      = 0.50 m/s
lambda   = 2.00
k_s      = 1.60
eta      = 0.50
phi      = 0.08
k_rho    = 0.80
```

---

# AprilTag strategy comparison

All four launches use the same:

- Gazebo court and tags
- AprilTag detector
- tag selection logic
- 10.0 m approach eligibility limit
- desired base_link pose at 0.90 m
- EKF/control stack
- RViz court visualizer
- robot-start launch arguments

Only the local motion strategy changes.

The shared test launch remains:

```text
smc_tag_check.launch.py
```

The four wrapper launches below select one controller strategy each.

## 1. Pure SMC

This is the direct pose SMC already tested:

```text
current pose
   ↓
SMC directly to final base pose
   ↓
0.90 m + final heading
```

Run:

```bash
ros2 launch scrobot_debug tag_strategy_1_pure_smc.launch.py
```

This is the baseline that showed strong dependence on initial lateral error.

## 2. Original main-branch controller

This reproduces the original main-branch motion law:

```text
turn toward goal position
   ↓
drive directly toward goal position
   ↓
position tolerance reached
   ↓
rotate in place to final tag-facing heading
```

Run:

```bash
ros2 launch scrobot_debug tag_strategy_2_main_branch.launch.py
```

For a fair comparison, it uses the current 0.90 m desired pose and current tag
selection/range logic, while the movement law and original speed limits are:

```text
k_position      = 0.80
k_heading       = 1.80
k_final_yaw     = 1.80
heading limit   = 35 deg
max linear      = 0.25 m/s
max angular     = 0.60 rad/s
```

## 3. Tangent biarc path feeding SMC

A single circular arc cannot generally satisfy arbitrary start/end positions
and both endpoint headings. Therefore this experiment uses two circular arcs
with a common tangent at their join:

```text
robot pose P0, heading t0
   ╲
    ) arc 1
     )── Pm, common tangent ──(
                              ) arc 2
                             ╱
                goal P1, heading t1
```

For tangent distances `d1` and `d2`:

```text
q1 = P0 + d1*t0
q2 = P1 - d2*t1

d2 =
  [0.5*(v·v) - d1*(v·t0)]
  / [v·t1 - d1*(t0·t1 - 1)]

v = P1 - P0
```

The join is the weighted blend:

```text
Pm = q1 * d2/(d1+d2)
   + q2 * d1/(d1+d2)
```

The previous implementation forced `d1 = d2`. That balanced solution is valid
but can bow far away from the direct route when the starting heading is
oblique. The current implementation instead searches the positive-`d1,d2`
G1-continuous biarc family and chooses a compact candidate using:

```text
short path length
small maximum deviation from the endpoint chord
no very tight-radius preference
penalty for near-loop arc sweeps
```

A second important biarc issue was found in the circular-arc sampler. A circular
arc is not defined only by endpoint position and endpoint heading modulo
`2*pi`; its forward travel direction must also remain consistent with the
stored start tangent. Replacing a physically valid forward sweep such as
`+270 deg` with the angle-equivalent `-90 deg` branch can reverse the motion
relative to that tangent and make the path bow to the wrong side.

The current implementation therefore keeps the physically consistent signed
forward sweep produced by the tangent geometry. Compactness is enforced
separately: a candidate is rejected if either individual forward arc exceeds
`biarc_max_arc_sweep = 175 deg`, and the search continues through the
positive-`d1,d2` biarc family.

The sampler also verifies the natural sampled endpoint before any numerical
snapping:

```text
start position error <= 1e-5 m
end position error   <= 1e-5 m
start yaw error      <= 0.01 deg
end yaw error        <= 0.01 deg
```

A candidate that does not actually interpolate the requested final pose is
discarded.

The equal-`d` solution is retained as a guaranteed search anchor.

The generated path is not followed with RPP. A pose approximately
`biarc_lookahead` ahead of the nearest path point is fed to the same SMC as
a moving reference.

The first implementation froze the measured Controller Goal as soon as the
initial biarc was generated. Testing showed why this reduced final accuracy:
`main_branch` kept filtering later AprilTag observations while driving, whereas
the frozen biarc stayed committed to the earlier measurement.

Strategy 3 now uses live filtered goal refinement with gated replanning:

```text
new AprilTag observation
   ↓
update/filter Controller Goal
   ↓
goal moved enough?
   ├─ no  → keep current biarc
   └─ yes → rebuild from CURRENT robot pose to latest goal
   ↓
SMC tracks biarc look-ahead pose
```

Current replanning defaults:

```text
position change threshold = 0.04 m
yaw change threshold      = 2.5 deg
minimum replan interval   = 0.25 s
terminal handoff distance = 1.25 m
```

Inside 1.25 m of the live goal, the biarc is cleared and control is handed to
the original `main_branch` pose controller for terminal XY and yaw convergence.

Run:

```bash
ros2 launch scrobot_debug tag_strategy_3_biarc_smc.launch.py
```

Current path parameters:

```text
biarc_spacing   = 0.08 m
biarc_lookahead = 0.30 m
```

`biarc_spacing` is only the discretization interval used to sample the
continuous circular arcs into `nav_msgs/Path` poses. With 0.08 m spacing,
successive reference-path poses are approximately 8 cm apart. Reducing it gives
more points and a smoother numerical path but does not change the underlying
two circles.

`biarc_lookahead` is different: it determines how far ahead of the robot the
moving SMC reference is selected. The current value is 0.30 m.

At path generation the controller logs the selected geometry:

```text
Biarc selected:
d1=...
d2=...
length=...
max_deviation=...
min_radius=...
sweeps=(..., ...) deg
```

This makes badly looping/overshooting candidates visible during testing.

## 4. Return to the tag normal ray, then SMC

This deliberately implements the simple geometric baseline:

```text
lock tag
  ↓
rotate perpendicular to tag normal
  ↓
drive sideways in world geometry until near normal ray
  ↓
stop
  ↓
rotate to face tag
  ↓
pure SMC to final 0.90 m pose
```

Run:

```bash
ros2 launch scrobot_debug tag_strategy_4_normal_ray.launch.py
```

Current handoff parameters:

```text
normal-ray lateral tolerance = 0.12 m
ray heading tolerance        = 8 deg
crossing speed               = 0.30 m/s
```

This strategy is intentionally inefficient. During the perpendicular crossing
phase it does almost nothing to reduce longitudinal error, and it requires extra
in-place rotations. It is retained because it gives a very clear comparison:
if it is robust but slow, that confirms the main issue is the pure SMC capture
region rather than final pose accuracy.

---

# Common AprilTag test arguments

All four strategy launches expose:

```text
target_distance
preferred_tag_id
robot_x
robot_y
robot_z
robot_yaw
position_tolerance
yaw_tolerance_deg
launch_rviz
```

Default desired pose:

```text
target_distance = 0.90 m
position tolerance = 0.05 m
yaw tolerance = 5 deg
stable time = 0.25 s
```

Example far start:

```bash
ros2 launch scrobot_debug tag_strategy_3_biarc_smc.launch.py \
  robot_x:=6.70 \
  robot_y:=3.05 \
  robot_yaw:=3.14159
```

Use the same start pose for all four tests when comparing them.

## RViz

The tests reuse the master `scrobot_debug/rviz.launch.py` and existing
`court_visualizer`.

The lightweight comparison view shows:

```text
court / net / poles
selected tag
ideal desired base_link pose + heading
measured controller goal
robot
actual robot trajectory
strategy reference path
```

Topics:

```text
/court_markers
/debug/smc_tag/tag_marker
/debug/smc_tag/ideal_base_pose
/debug/tag_controller/goal_pose
/debug/smc_tag/trajectory
/debug/tag_controller/reference_path
```

The two pose arrows intentionally show different quantities:

```text
Desired Base Pose (yellow)
  = exact ideal pose from the known court tag frame
  = tag_mount + 0.90 m along the tag normal
  = heading directly toward the tag

Controller Goal (magenta)
  = pose estimated from the current AprilTag observation
  = the actual final pose used to generate/control the biarc
```

Therefore the biarc reference path must terminate at the magenta
`Controller Goal`, while the separation between that endpoint and the yellow
`Desired Base Pose` directly visualizes AprilTag pose-estimation error.

For `biarc_smc`, the measured Controller Goal is no longer frozen. Later
AprilTag observations continue to update the filtered goal. A replan occurs
only when the filtered pose changes beyond the configured thresholds and the
minimum replan interval has elapsed. Every replan starts from the current robot
pose, not the original start pose.

While the biarc phase is active, the published path endpoint must coincide with
the Controller Goal used for that plan. At terminal handoff an explicit empty
transient-local `Path` is published so RViz does not retain a stale biarc.

At each generation/replan the controller verifies:

```text
endpoint_error=0.000000 m
endpoint_yaw_error=0.000000 deg
```

up to floating-point precision.

## What to compare

For each strategy, use the same initial pose and record:

```text
success / failure
total approach time
travel distance
final rho
final e_y
final e_theta
final base-to-tag range
path shape
number of stop/spin phases
```

The most important test is robustness over different starting lateral offsets,
not only one favorable start near the tag normal ray.


---

# AprilTag conclusions and lessons learned

Final practical ranking from the strategy tests:

```text
1. main_branch
   robust, simple, accurate from oblique starts

2. biarc_smc
   good large-error capture; live replan + main_branch terminal handoff

3. pure_smc
   useful local pose stabilizer but sensitive to large initial lateral error

4. normal_ray_smc
   robust geometric baseline but inefficient
```

Key lessons:

- Pure pose SMC has a capture-region problem, not automatically a steady-state
  error problem. With `s = e_theta + lambda*e_y`, large initial `e_y` can
  dominate the surface and produce poor global approach behavior.
- The yellow Desired Base Pose is the ideal pose from known court/tag geometry.
  The magenta Controller Goal is estimated from the observed AprilTag and is
  the pose actually used by the local controller.
- `main_branch` performed unexpectedly well partly because it kept refining its
  filtered goal from later AprilTag observations while the robot moved.
- Biarc geometry must be validated at its natural sampled endpoint; do not hide
  a bad candidate by overwriting its last pose.
- Angle-equivalent wrapped sweeps are not always motion-equivalent. Preserve the
  forward tangent direction and reject/search non-compact biarc candidates.
- The hybrid tag controller works because each stage has a clear job: biarc +
  SMC handles large-error capture, while live `main_branch` handles terminal
  pose accuracy.
- For SMC path-tracking lateral error, `lambda` is the first parameter to
  inspect. Increasing it gives `e_y` more weight in the sliding surface, but
  excessive `lambda` can again reduce the practical capture region.
- `k_s` mainly changes convergence toward the sliding surface, `eta` strengthens
  the reaching/robustness term, and `phi` sets the boundary-layer/chattering
  tradeoff.
- Final stopped error can also be dominated by `position_tolerance`. Once the
  controller has handed off to `main_branch`, changing SMC gains cannot improve
  an error that the terminal controller already accepts.

The AprilTag SMC investigation is now considered complete enough to move on to
the shuttle local-collection controller.

---

# Shuttle SMC

The first shuttle test deliberately uses **pure SMC only**. No biarc is used
for shuttle approach in this baseline. The goal is to characterize the local
SMC capture region directly before adding any path-shaping method.

Current sequence:

```text
detect eligible shuttle
   ↓
freeze shuttle point in odom
   ↓
build fixed base_link pre-pose
   ↓
PURE SMC to pre-pose
   ↓
rho <= 0.03 m AND |e_theta| <= 5 deg
   ↓
remain continuously valid for 0.25 s
   ↓
STRAIGHT_COLLECT: v = 0.30 m/s, omega = 0
   ↓
collector reaches shuttle
   ↓
0.10 m overrun
```

The shuttle controller uses the same simplified convention:

```text
controlled point = base_link
c = 0
desired stand-off = 1.10 m from frozen shuttle
usable mission range = 0.50 ... 1.80 m from base_link
```

The 0.50–1.80 m range is a mission-side eligibility range evaluated after the
detection has been transformed into `base_link`; it is not a camera-depth
range.

After convergence:

```text
STRAIGHT_COLLECT:
v     = 0.30 m/s
omega = 0
```

Run:

```bash
ros2 launch scrobot_debug smc_shuttle_check.launch.py \
  model_path:=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt
```

Override the base stand-off if needed:

```bash
ros2 launch scrobot_debug smc_shuttle_check.launch.py \
  model_path:=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt \
  base_standoff:=1.10
```

Expected convergence line:

```text
SMC pre-pose reached and stable:
rho=...
e_y=...
e_theta=...
s=...
stable=0.25 s
base_range=... m
```

At the handoff:

```text
base_range ~= 1.10 m
```

For the first pure-SMC shuttle tests, record:

```text
initial shuttle range / bearing
initial rho
initial e_y
initial e_theta
maximum |e_y|
pre-pose settling time
final rho
final e_y
final e_theta
base_range at handoff
success / timeout / circling behavior
```

The main question was the shuttle SMC capture region: how much lateral and
heading error can be corrected while the shuttle remains inside the useful
camera region. The pure-SMC test was retained; no biarc was added to shuttle
collection.

## Final shuttle SMC tuning

The first shuttle runs showed acceptable lateral convergence but strong heading
overshoot. Tuning was therefore focused on the angular dynamics while keeping
the lateral behavior intact. The final values currently pushed in
`local_collect.yaml` are:

```text
v_R                    = 0.50 m/s
lambda                 = 2.50
k_s                    = 1.20
eta                    = 0.40
phi                    = 0.30
k_rho                  = 0.80
max angular speed      = 1.00 rad/s
max linear speed       = 0.50 m/s
max angular accel      = 2.00 rad/s^2
max linear accel       = 1.00 m/s^2
pre-pose position tol  = 0.03 m
pre-pose yaw tol       = 5 deg
pre-pose stable time   = 0.25 s
```

Important tuning lessons from the shuttle test:

- `lambda` weights lateral error inside `s = e_theta + lambda*e_y`; once
  lateral tracking is acceptable, do not keep changing it to solve pure
  heading overshoot.
- Increasing `phi` softens the boundary layer around `s = 0` and reduces
  aggressive sign-changing angular correction.
- Reducing `eta` weakens the reaching/switching term and can reduce repeated
  heading crossings.
- Reducing `k_s` reduces proportional correction on the sliding surface.
- `smc_max_angular_speed` limits how fast the robot may rotate.
- `max_angular_accel` is a slew-rate limit. If it is too small, the controller
  can request a reversal while the commanded angular velocity is still slowly
  decelerating in the old direction, increasing overshoot. The final shuttle
  setup therefore uses a lower angular-speed ceiling but a higher angular
  acceleration limit.
- A stable-time gate is required before `STRAIGHT_COLLECT`; otherwise an
  oscillating heading can cross the yaw tolerance for one sample and trigger
  the straight pass too early.

## Shuttle RViz debug view

The shuttle debug launch now shows:

```text
robot model
frozen shuttle target
desired 1.10 m base_link pre-pose
actual robot trajectory
court / net / poles
```

Relevant topics:

```text
/debug/smc_shuttle/target
/debug/smc_shuttle/pre_pose
/debug/smc_shuttle/target_marker
/debug/smc_shuttle/pre_pose_view
/debug/smc_shuttle/trajectory
```

The frozen target and pre-pose debug topics use Reliable + Transient Local QoS
so RViz can start after target lock without losing the geometry. The trajectory
is generated from `/odometry/filtered` using Reliable + Volatile QoS.

The robot then runs straight over the frozen shuttle target.

At this point the AprilTag and shuttle SMC investigation is considered closed;
future shuttle work should return to perception accuracy and mission integration
unless new control failures appear.


## Biarc branch validation

Current path parameters:

```text
biarc_spacing          = 0.08 m
biarc_lookahead        = 0.30 m
biarc_max_arc_sweep    = 175 deg
d1 family search       = 0.15 ... 6.0 x balanced d1
d1 samples             = 81
```

The arc generator keeps the physically correct **forward signed sweep**.
It must not wrap a +270 deg forward left-turn into a -90 deg arc: although
those angles share the same endpoint orientation modulo 2*pi, the -90 deg
version reverses motion relative to the stored tangent and makes the reference
bow to the wrong side. Candidates requiring more than 175 deg on either arc are
rejected and another member of the biarc family is tried.

The controller also verifies that the sampled path truly interpolates the
requested start and controller-goal pose before accepting it.

RViz contains two goal markers:

```text
Desired Base Pose   = ideal pose from known court/tag geometry
Controller Goal     = pose produced from the locked AprilTag observation
```

The biarc must end exactly at **Controller Goal**. For a healthy detection,
Controller Goal should also nearly overlap Desired Base Pose.

This is the most promising experiment when pure pose SMC has a small capture
region, because the SMC sees much smaller local lateral and heading errors.
