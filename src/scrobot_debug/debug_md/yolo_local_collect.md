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

Keep each initial target inside the validated YOLO camera-relative range.

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

After this passes, the next test is to replace the manual action trigger with
the sweep mission's automatic shuttle diversion.
