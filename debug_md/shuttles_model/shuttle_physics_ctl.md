# Shuttle Physics Control Commands

This file collects the commands used for the isolated shuttle-physics test in
`scrobot_debug`.

The test world is persistent: launch Gazebo once, then spawn, delete, respawn,
and apply impulses without restarting Gazebo.

## 1. Update and build

```bash
cd ~/scrobot_ws

git checkout simulation-systematic-test-c98fee65
git pull

colcon build --symlink-install \
  --packages-up-to scrobot_debug

source install/setup.bash
```

If `scrobot_debug` or `scrobot_simulation` was changed and a clean package
rebuild is needed:

```bash
cd ~/scrobot_ws

rm -rf \
  build/scrobot_simulation install/scrobot_simulation \
  build/scrobot_debug install/scrobot_debug

colcon build --symlink-install \
  --packages-up-to scrobot_debug

source install/setup.bash
```

Do not use `git clean -fd` while the local collision STL is untracked.

The collision mesh currently expected by the test model is:

```text
~/scrobot_ws/src/scrobot_simulation/models/shuttle/meshes/shuttle_collision_octagonal.stl
```

## 2. Check installed shuttle debug executables

```bash
ros2 pkg executables scrobot_debug | grep shuttle
```

Expected relevant executables include:

```text
scrobot_debug shuttle_physics_ctl
scrobot_debug shuttle_physics_monitor
scrobot_debug shuttle_impulse_test
scrobot_debug spawn_shuttle_physics
```

The physics harness now spawns the production shuttle directly from:

```text
scrobot_simulation/models/shuttle/model.sdf
```

so the isolated test and mission simulation use exactly the same dynamic
physics model.

## 3. Start the persistent shuttle physics world

```bash
ros2 launch scrobot_debug shuttle_physics_check.launch.py
```

The default launch starts:

- the isolated Gazebo shuttle-physics world;
- the Gazebo-to-ROS pose bridge;
- `shuttle_physics_monitor`;
- the central `telemetry_monitor`.

It does not automatically spawn a shuttle by default.

## 4. Open another terminal

```bash
cd ~/scrobot_ws
source install/setup.bash
```

Use this terminal for the commands below.

## 5. Spawn one shuttle

Default side-resting shuttle with a 5 s countdown:

```bash
ros2 run scrobot_debug shuttle_physics_ctl spawn
```

Spawn immediately:

```bash
ros2 run scrobot_debug shuttle_physics_ctl spawn --delay 0
```

Spawn upright:

```bash
ros2 run scrobot_debug shuttle_physics_ctl spawn \
  --orientation upright
```

Spawn from a 50 mm drop:

```bash
ros2 run scrobot_debug shuttle_physics_ctl spawn \
  --drop-height 0.05
```

Spawn upright from a 50 mm drop:

```bash
ros2 run scrobot_debug shuttle_physics_ctl spawn \
  --orientation upright \
  --drop-height 0.05
```

## 6. Delete shuttle

Delete the default single shuttle:

```bash
ros2 run scrobot_debug shuttle_physics_ctl delete
```

Ignore the error if it is already absent:

```bash
ros2 run scrobot_debug shuttle_physics_ctl delete \
  --ignore-missing
```

## 7. Respawn shuttle

Delete the current shuttle and create a fresh one:

```bash
ros2 run scrobot_debug shuttle_physics_ctl respawn
```

Immediate respawn after the Gazebo removal wait:

```bash
ros2 run scrobot_debug shuttle_physics_ctl respawn \
  --delay 0
```

Respawn from a 50 mm drop:

```bash
ros2 run scrobot_debug shuttle_physics_ctl respawn \
  --drop-height 0.05
```

The default post-delete wait is 0.5 s. It can be changed with:

```bash
ros2 run scrobot_debug shuttle_physics_ctl respawn \
  --delete-wait 1.0
```

## 8. Apply an impulse without restarting Gazebo

Default impulse, after a 5 s countdown:

```bash
ros2 run scrobot_debug shuttle_physics_ctl impulse
```

Default values:

```text
force_y   = 0.03 N
duration  = 0.05 s
impulse_y = 0.0015 N*s
```

Apply immediately:

```bash
ros2 run scrobot_debug shuttle_physics_ctl impulse \
  --delay 0
```

Stronger Y impulse:

```bash
ros2 run scrobot_debug shuttle_physics_ctl impulse \
  --force-y 0.06 \
  --duration 0.05
```

Impulse in X:

```bash
ros2 run scrobot_debug shuttle_physics_ctl impulse \
  --force-x 0.03 \
  --force-y 0.0 \
  --duration 0.05
```

Apply torque around Z:

```bash
ros2 run scrobot_debug shuttle_physics_ctl impulse \
  --force-y 0.0 \
  --torque-z 0.001 \
  --duration 0.05
```

Full impulse command options:

```bash
ros2 run scrobot_debug shuttle_physics_ctl impulse \
  --delay 5.0 \
  --duration 0.05 \
  --force-x 0.0 \
  --force-y 0.03 \
  --force-z 0.0 \
  --torque-x 0.0 \
  --torque-y 0.0 \
  --torque-z 0.0
```

## 9. Clear an applied persistent wrench

Normally the impulse command clears its own wrench after the requested
duration. If a test is interrupted and the wrench needs to be cleared manually:

```bash
ros2 run scrobot_debug shuttle_physics_ctl clear
```

## 10. Monitor shuttle physics

The shuttle physics monitor is started automatically by
`shuttle_physics_check.launch.py`.

To watch only the shuttle-physics data:

```bash
ros2 topic echo /debug/shuttle_physics
```

To watch the central consolidated debug stream:

```bash
ros2 topic echo /debug/telemetry
```

The physics stream reports values such as:

```text
COUNT
PERF
SETTLED
REMOVED
RTF
moving shuttle count
linear velocity
angular velocity
maximum velocity
maximum angular velocity
position
path length
net displacement
```

The physics monitor can also be run manually against an already-running test
world:

```bash
ros2 run scrobot_debug shuttle_physics_monitor
```

## 11. Many-shuttle performance test

Spawn 25 shuttles:

```bash
ros2 run scrobot_debug shuttle_physics_ctl spawn \
  --count 25 \
  --spacing 0.15 \
  --delay 0
```

Delete them:

```bash
ros2 run scrobot_debug shuttle_physics_ctl delete \
  --count 25
```

Spawn 50:

```bash
ros2 run scrobot_debug shuttle_physics_ctl spawn \
  --count 50 \
  --spacing 0.15 \
  --delay 0
```

Delete 50:

```bash
ros2 run scrobot_debug shuttle_physics_ctl delete \
  --count 50
```

Respawn 50 without restarting Gazebo:

```bash
ros2 run scrobot_debug shuttle_physics_ctl respawn \
  --count 50 \
  --spacing 0.15 \
  --delay 0
```

Spawn 100:

```bash
ros2 run scrobot_debug shuttle_physics_ctl spawn \
  --count 100 \
  --spacing 0.15 \
  --delay 0
```

Delete 100:

```bash
ros2 run scrobot_debug shuttle_physics_ctl delete \
  --count 100
```

## 12. Typical single-shuttle test loop

Keep Gazebo running and repeat:

```bash
ros2 run scrobot_debug shuttle_physics_ctl spawn
ros2 run scrobot_debug shuttle_physics_ctl impulse
ros2 run scrobot_debug shuttle_physics_ctl impulse --force-y 0.06
ros2 run scrobot_debug shuttle_physics_ctl respawn
ros2 run scrobot_debug shuttle_physics_ctl impulse --delay 0
ros2 run scrobot_debug shuttle_physics_ctl delete
```

No Gazebo restart is required between these operations.
