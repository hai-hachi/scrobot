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

The SMC law is therefore:

```text
s = e_theta + lambda * e_y

omega =
    lambda * v_R * sin(e_theta)
  + k_s * s
  + eta * sat(s / phi)

v = k_rho * rho * cos(alpha)
```

For large `alpha`, linear motion is stopped until the heading is acceptable.

Current baseline gains:

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

A single circular arc cannot generally satisfy two arbitrary endpoint positions
and both endpoint headings. Therefore this experiment uses a **biarc**:

```text
robot pose
   ╲
    ) circular arc 1
     )── tangent join ──(
                       ) circular arc 2
                      ╱
                desired tag pose
```

The two circular arcs join with a continuous tangent. The first arc is tangent
to the robot starting heading and the second arc is tangent to the desired final
heading.

The generated path is not followed with RPP. Instead, a look-ahead pose on the
biarc is continuously fed to the same SMC as a moving reference.

Run:

```bash
ros2 launch scrobot_debug tag_strategy_3_biarc_smc.launch.py
```

Current path parameters:

```text
biarc_spacing   = 0.08 m
biarc_lookahead = 0.35 m
```

This is the most promising experiment when pure pose SMC has a small capture
region, because the SMC sees much smaller local lateral and heading errors.

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
desired base_link pose + heading
robot
actual robot trajectory
strategy reference path
```

Topics:

```text
/court_markers
/debug/smc_tag/tag_marker
/debug/smc_tag/desired_base_pose
/debug/smc_tag/trajectory
/debug/tag_controller/reference_path
```

The biarc strategy publishes its generated path on the reference-path topic.
The other strategies leave that path empty.

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

# Shuttle SMC

The shuttle controller now uses the same simplified convention:

```text
controlled point = base_link
c = 0
desired stand-off = 1.10 m from frozen shuttle
```

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
SMC pre-pose reached:
rho=...
e_y=...
e_theta=...
s=...
base_range=... m
```

At the handoff:

```text
base_range ~= 1.10 m
```

The robot then runs straight over the frozen shuttle target.
