# Shuttle Collection Geometry Test

This file contains the copy-ready commands for the persistent shuttle collection
geometry test.

## Collection rule

The shuttle collection point is defined as:

```text
shuttle model origin + 0.045 m along shuttle local +Z
```

The collector pickup rectangle is fixed in the robot frame:

```text
center x = +0.165 m
center y =  0.000 m

size X = 0.060 m
size Y = 0.300 m
```

Therefore the shuttle collection-center point is collected when:

```text
0.135 m <= center_x <= 0.195 m
-0.150 m <= center_y <= +0.150 m
```

Shuttle orientation matters because the 45 mm center offset follows the
shuttle's local +Z axis.

## 1. Update and rebuild

```bash
cd ~/scrobot_ws

git checkout simulation-systematic-test-c98fee65
git pull

rm -rf \
  build/scrobot_simulation install/scrobot_simulation \
  build/scrobot_debug install/scrobot_debug

colcon build --symlink-install \
  --packages-up-to scrobot_debug

source install/setup.bash
```

## 2. Start the persistent collection test

```bash
ros2 launch scrobot_debug collection_check.launch.py
```

This starts Gazebo, the robot, the low-level diff-drive controllers, the
collection monitor, and central debug telemetry. The shuttle is controlled
separately so Gazebo does not need to restart between tests.

## 3. Open a command terminal

```bash
cd ~/scrobot_ws
source install/setup.bash
```

## 4. Monitor collection geometry

Collection-only debug stream:

```bash
ros2 topic echo /debug/collection_test
```

Central debug stream:

```bash
ros2 topic echo /debug/telemetry
```

Useful output includes:

```text
GEOM
center_robot
origin_world
shuttle quaternion
x_margin
y_margin
expected_collect
COLLECTED
GT_COUNT
```

## 5. Immediate pickup at collector center

```bash
ros2 run scrobot_debug collection_test_ctl spawn
```

Equivalent explicit command:

```bash
ros2 run scrobot_debug collection_test_ctl spawn \
  --center-x 0.165 \
  --center-y 0.000
```

Expected result: immediate collection.

## 6. Delete the current test shuttle

```bash
ros2 run scrobot_debug collection_test_ctl delete
```

Ignore an already-missing shuttle:

```bash
ros2 run scrobot_debug collection_test_ctl delete \
  --ignore-missing
```

## 7. Respawn without restarting Gazebo

```bash
ros2 run scrobot_debug collection_test_ctl respawn
```

Immediate respawn:

```bash
ros2 run scrobot_debug collection_test_ctl respawn \
  --delay 0
```

## 8. Rectangle boundary tests

Clearly inside:

```bash
ros2 run scrobot_debug collection_test_ctl respawn \
  --center-x 0.165 \
  --center-y 0.100
```

Near the inside corner:

```bash
ros2 run scrobot_debug collection_test_ctl respawn \
  --center-x 0.194 \
  --center-y 0.149
```

Just outside X:

```bash
ros2 run scrobot_debug collection_test_ctl respawn \
  --center-x 0.196 \
  --center-y 0.000
```

Just outside Y:

```bash
ros2 run scrobot_debug collection_test_ctl respawn \
  --center-x 0.165 \
  --center-y 0.151
```

## 9. Shuttle orientation tests

Side-resting, local +Z forward:

```bash
ros2 run scrobot_debug collection_test_ctl respawn \
  --center-x 0.165 \
  --center-y 0.000 \
  --roll-deg 0 \
  --pitch-deg 90 \
  --yaw-deg 0
```

Side-resting and rotated 90 deg:

```bash
ros2 run scrobot_debug collection_test_ctl respawn \
  --center-x 0.165 \
  --center-y 0.000 \
  --roll-deg 0 \
  --pitch-deg 90 \
  --yaw-deg 90
```

Side-resting with local +Z reversed:

```bash
ros2 run scrobot_debug collection_test_ctl respawn \
  --center-x 0.165 \
  --center-y 0.000 \
  --roll-deg 0 \
  --pitch-deg -90 \
  --yaw-deg 0
```

Upright:

```bash
ros2 run scrobot_debug collection_test_ctl respawn \
  --center-x 0.165 \
  --center-y 0.000 \
  --roll-deg 0 \
  --pitch-deg 0 \
  --yaw-deg 0 \
  --origin-z 0.001
```

## 10. Straight-drive collection

Spawn a shuttle 0.600 m ahead of the robot:

```bash
ros2 run scrobot_debug collection_test_ctl respawn \
  --center-x 0.600 \
  --center-y 0.000 \
  --delay 0
```

Drive straight through it:

```bash
ros2 run scrobot_debug collection_test_ctl drive \
  --speed 0.10 \
  --duration 5.0
```

The drive command publishes directly to:

```text
/diff_drive_controller/cmd_vel
```

and always publishes stop commands at the end.

## 11. Lateral-offset straight-drive tests

Centerline:

```bash
ros2 run scrobot_debug collection_test_ctl respawn \
  --center-x 0.600 \
  --center-y 0.000 \
  --delay 0

ros2 run scrobot_debug collection_test_ctl drive \
  --speed 0.10 \
  --duration 5.0
```

100 mm offset:

```bash
ros2 run scrobot_debug collection_test_ctl respawn \
  --center-x 0.600 \
  --center-y 0.100 \
  --delay 0

ros2 run scrobot_debug collection_test_ctl drive \
  --speed 0.10 \
  --duration 5.0
```

140 mm offset:

```bash
ros2 run scrobot_debug collection_test_ctl respawn \
  --center-x 0.600 \
  --center-y 0.140 \
  --delay 0

ros2 run scrobot_debug collection_test_ctl drive \
  --speed 0.10 \
  --duration 5.0
```

160 mm offset, geometrically outside the pickup rectangle:

```bash
ros2 run scrobot_debug collection_test_ctl respawn \
  --center-x 0.600 \
  --center-y 0.160 \
  --delay 0

ros2 run scrobot_debug collection_test_ctl drive \
  --speed 0.10 \
  --duration 5.0
```

## 12. Test another orientation while approaching

Example: 90 deg shuttle yaw at 100 mm lateral offset.

```bash
ros2 run scrobot_debug collection_test_ctl respawn \
  --center-x 0.600 \
  --center-y 0.100 \
  --pitch-deg 90 \
  --yaw-deg 90 \
  --delay 0

ros2 run scrobot_debug collection_test_ctl drive \
  --speed 0.10 \
  --duration 5.0
```

## 13. Typical persistent test loop

```bash
ros2 run scrobot_debug collection_test_ctl respawn \
  --center-x 0.600 --center-y 0.000 --delay 0

ros2 run scrobot_debug collection_test_ctl drive \
  --speed 0.10 --duration 5.0

ros2 run scrobot_debug collection_test_ctl respawn \
  --center-x 0.600 --center-y 0.140 --delay 0

ros2 run scrobot_debug collection_test_ctl drive \
  --speed 0.10 --duration 5.0

ros2 run scrobot_debug collection_test_ctl delete
```

Gazebo does not need to restart between these operations.
