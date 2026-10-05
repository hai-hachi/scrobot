# YOLO Local Collection Check

This test validates the first complete autonomous collection chain using the
trained YOLO model instead of Gazebo-truth fake perception.

## Chain under test

```text
Gazebo shuttle
    ↓
D435i-like RGB
    ↓
yolo_shuttle_detector
    ↓
/perception/shuttle_detections_3d
    ↓
shuttle_collection_filter
    ↓
/perception/collectable_shuttle_detections_3d
    ↓
local_collect_controller
    ↓
/cmd_vel_approach
    ↓
AUTO command mux
    ↓
velocity smoother
    ↓
collision monitor
    ↓
diff_drive_controller
    ↓
physical Gazebo collection event
```

The default shuttle is intentionally placed slightly off-center at:

```text
x = 1.20 m
y = 0.20 m
```

so the SMC pre-pose stage must correct both heading and lateral error.

## 1. Build

```bash
cd ~/scrobot_ws

git checkout perception-yolo-v2
git pull

colcon build --symlink-install \
  --packages-up-to \
  scrobot_debug \
  scrobot_perception \
  scrobot_mission \
  scrobot_control \
  scrobot_localization \
  scrobot_simulation

source install/setup.bash
```

## 2. Launch the integration test

```bash
ros2 launch scrobot_debug yolo_local_collect_check.launch.py \
  model_path:=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt
```

The launch starts:

```text
Gazebo + SCROBOT
production perception with YOLO
local EKF localization
production command/safety pipeline
shuttle_collection_filter
local_collect_controller
one dynamic test shuttle
central telemetry
```

A debug-only identity `map -> odom` transform is used because this isolated
test does not run AprilTag global localization. EKF still owns
`odom -> base_footprint`.

## 3. Confirm the perception/filter boundary before moving

Wait for telemetry similar to:

```text
[PERCEPTION] raw_visible=1
[PERCEPTION] collectable_visible=1
```

or a summary containing:

```text
raw=1 eligible=1
```

Useful direct checks:

```bash
ros2 topic echo /perception/shuttle_detections_3d
ros2 topic echo /perception/collectable_shuttle_detections_3d
```

Check the action server:

```bash
ros2 action info /local_collect
```

Expected:

```text
Action clients: 0
Action servers: 1
```

## 4. Start the local collection action

In another terminal:

```bash
source ~/scrobot_ws/install/setup.bash

ros2 action send_goal \
  /local_collect \
  scrobot_interfaces/action/LocalCollect \
  "{timeout_sec: 30.0}" \
  --feedback
```

Expected controller phases:

```text
SELECT
  ↓
SMC_POSE
  ↓
STRAIGHT_COLLECT
  ↓
OVERRUN
  ↓
SELECT
  ↓
DONE
```

Expected behavior:

1. YOLO sees the shuttle.
2. The collection filter publishes it as eligible.
3. The controller freezes the detected target in `odom`.
4. SMC moves base_link to the 1.10 m pre-collection pose while facing the shuttle.
5. The robot switches to straight collection at 0.30 m/s.
6. The collector passes through the shuttle.
7. Gazebo removes the shuttle and emits a collection event.
8. YOLO/raw/eligible counts return to zero.
9. The action ends successfully when no next eligible shuttle is visible.

## 5. Watch useful topics

```bash
ros2 topic echo /mission/local_collect_phase
```

```bash
ros2 topic echo /cmd_vel_approach
```

```bash
ros2 topic echo /debug/telemetry
```

```bash
ros2 topic echo /evaluation/shuttle_collected
```

YOLO debug image:

```bash
rqt_image_view
```

Select:

```text
/perception/shuttle_debug/image
```

## 6. Respawn and repeat without restarting Gazebo

The launch-created shuttle name is:

```text
singleyolo_local_collect
```

If it was successfully collected, spawn it again:

```bash
ros2 run scrobot_simulation spawn_shuttles \
  --mode single \
  --world badminton_court \
  --name singleyolo_local_collect \
  --x 1.20 \
  --y 0.20
```

Then send the `/local_collect` action again.

If the previous shuttle still exists, delete it first:

```bash
gz service \
  -s /world/badminton_court/remove/blocking \
  --reqtype gz.msgs.Entity \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'name: "singleyolo_local_collect", type: MODEL'
```

## 7. Position tests

Centered:

```bash
ros2 run scrobot_simulation spawn_shuttles \
  --mode single \
  --world badminton_court \
  --name local_center \
  --x 1.20 --y 0.00
```

Left/right SMC correction:

```bash
ros2 run scrobot_simulation spawn_shuttles \
  --mode single \
  --world badminton_court \
  --name local_left \
  --x 1.20 --y 0.25
```

```bash
ros2 run scrobot_simulation spawn_shuttles \
  --mode single \
  --world badminton_court \
  --name local_right \
  --x 1.20 --y -0.25
```

Keep each initial target inside the validated mission envelope of
0.50-1.80 m planar range from `base_link`. The YOLO detector itself uses a
broader 0.20-3.00 m camera-depth validity interval.

## 8. Pass criteria

The integration passes when:

1. YOLO produces a 3D shuttle detection.
2. The collection filter reports at least one eligible shuttle.
3. `/local_collect` accepts the goal.
4. SMC_POSE converges to the 1.10 m base_link pre-collection pose.
5. STRAIGHT_COLLECT uses approximately 0.30 m/s with zero commanded yaw rate.
6. The shuttle is physically removed by the Gazebo collection system.
7. `/evaluation/shuttle_collected` emits the collection event.
8. Raw and eligible detection counts return to zero.
9. The action completes successfully.
10. Collision monitoring remains active throughout the motion.

This single-shuttle integration passed. Multi-shuttle filtering, selection,
and reacquisition were then validated separately as described below.


## 9. Multi-shuttle selection and reacquisition

The next integration test uses the production detector, mission filter, local
SMC controller, and physical Gazebo collection with four shuttles:

```text
A = (1.25, -0.50) m  initial range about 1.35 m  eligible
B = (1.45, -0.15) m  initial range about 1.46 m  eligible
C = (1.60, +0.20) m  initial range about 1.61 m  eligible
D = (1.75, +0.60) m  initial range about 1.85 m  initially rejected
```

Run:

```bash
ros2 launch scrobot_debug yolo_multi_collect_check.launch.py \
  model_path:=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt
```

The test validates:

```text
4 raw YOLO detections
        |
        v
initial mission gate
A / B / C eligible
D initially rejected
        |
        v
local_collect_controller
locks first eligible detection in array order
        |
        v
collect
        |
        v
exclude attempted target
        |
        v
reacquire first new eligible detection
```

Important controller behavior:

```text
_lock_first_target()
```

does not currently sort by nearest range or smallest bearing. It walks the
filtered `Detection3DArray` in order and freezes the first eligible detection
that is not within the reacquisition exclusion radius of a previously attempted
target.

The monitor latches the pre-motion filter result and reports:

```text
initial_filter=PASS
selection=PASS
reacquire=PASS
```

The multi-shuttle test passed. Therefore the existing first-visible/array-order
policy is retained for the current mission. A more explicit cost-based selector
is an optional future optimization, not a blocker.

Note that shuttle D may legitimately become eligible after the robot moves.
Only the initial gate is expected to reject D.

## 10. Final local-collection validation status

The complete local collection subsystem is now considered validated:

```text
production YOLO detection                 PASS
aligned-depth 3D reconstruction           PASS
base_link collection range filter         PASS
single-shuttle SMC collection             PASS
stable pre-pose handoff                   PASS
straight 0.30 m/s collection pass         PASS
Gazebo physical collection event          PASS
multiple simultaneous shuttles            PASS
first-target selection                     PASS
post-collection target reacquisition      PASS
```

The remaining simulation milestone is no longer a local-perception or
local-controller test. It is the full mission integration:

```text
initial AprilTag acquisition / relocalization
        |
        v
Nav2 join + court sweep
        |
        v
eligible shuttle interrupt
        |
        v
save sweep checkpoint / path index
        |
        v
local collection spree
        |
        v
Nav2 return to saved checkpoint
        |
        v
resume sweep progress
        |
        v
periodic/fixed-station AprilTag relocalization
        |
        v
mission COMPLETE
```
