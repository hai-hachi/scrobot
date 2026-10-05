# Final Full-Mission Validation

All major isolated simulation subsystems have passed. This is the remaining
integrated acceptance run.

## Validated prerequisites

```text
robot description / TF                 PASS
control command pipeline               PASS
AUTO/MANUAL arbitration                PASS
depth scan + collision monitor         PASS
wheel/IMU EKF                          PASS
AprilTag detection/relocalization      PASS
tag approach strategy                  PASS
YOLO 2D + aligned-depth 3D             PASS
3D error (~0.03 m lateral off-axis)    ACCEPTED
0.50-1.80 m collection range gate      PASS
local SMC shuttle collection           PASS
multi-shuttle selection/reacquisition  PASS
Nav2/RPP subsystem tests               PASS
```

## Build

```bash
cd ~/scrobot_ws
git pull
colcon build --symlink-install
source install/setup.bash
```

Set the production YOLO model:

```bash
export SCROBOT_YOLO_MODEL=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt
```

## Start the full simulation stack

Use separate terminals so failures remain readable.

### 1. Simulation

```bash
ros2 launch scrobot_simulation simulation.launch.py
```

Optionally spawn a repeatable shuttle distribution:

```bash
ros2 launch scrobot_simulation spawn_shuttles.launch.py \
  mode:=mixed count:=50
```

### 2. Production perception

```bash
ros2 launch scrobot_perception perception.launch.py \
  enable_yolo:=true \
  model_path:=$SCROBOT_YOLO_MODEL
```

### 3. Control stack

```bash
ros2 launch scrobot_control control_stack.launch.py
```

### 4. Local EKF

```bash
ros2 launch scrobot_localization localization.launch.py \
  use_magnetometer:=false
```

### 5. Mission / global localization / Nav2

```bash
ros2 launch scrobot_mission sweep_mission.launch.py
```

### 6. Evaluation

```bash
ros2 launch scrobot_evaluation collection_session_eval.launch.py
```

Optional telemetry:

```bash
ros2 launch scrobot_debug telemetry.launch.py
```

## Required state sequence

Normal mission:

```text
INITIAL_TAG_APPROACH
 -> INITIAL_RELOCALIZATION
 -> STARTING_NAV2
 -> JOIN_SWEEP
 -> SWEEPING
```

Shuttle diversion:

```text
SWEEPING
 -> save checkpoint pose + sweep index
 -> cancel FollowPath
 -> LOCAL_COLLECT
 -> RETURN_TO_SWEEP
 -> return to original checkpoint
 -> resume saved sweep index
 -> SWEEPING
```

Fixed relocalization:

```text
SWEEPING
 -> fixed station
 -> TURN_TO_TAG
 -> RELOCALIZING
 -> RESTORE_SWEEP_HEADING
 -> SWEEPING
```

Final state:

```text
COMPLETE
```

## Watch

```bash
ros2 topic echo /mission/state
ros2 topic echo /mission/local_collect_phase
ros2 topic echo /evaluation/shuttle_collected
```

Check current checkpoint/path behavior in mission logs and use
`/mission/sweep_path` / `/mission/current_goal` when visualization is needed.

## Acceptance criteria

The final integrated run passes when:

1. initial tag search/approach succeeds;
2. initial 15-sample relocalization establishes a valid global pose;
3. Nav2 starts only after localization;
4. the robot joins and follows the four-pass sweep;
5. a collectable shuttle interrupts FollowPath;
6. both checkpoint pose and sweep index are preserved;
7. local collection successfully clears currently reachable visible shuttles;
8. the robot returns to the original checkpoint with Nav2;
9. sweep progress resumes instead of restarting;
10. fixed-station AprilTag corrections succeed;
11. collision monitoring remains active;
12. the sweep reaches `COMPLETE` without manual intervention.

## Record

For the final report, record:

```text
mission duration
travel distance
detected shuttle count
collected shuttle count
remaining normal-area count
near-pole excluded count
collection rate
sweep time
local-collection time
return-to-sweep time
AprilTag relocalization count
position RMSE
mission errors/timeouts
```

Any failure discovered here should be treated as an integration issue first.
Re-open a completed subsystem only when the mission log isolates the failure to
that subsystem.
