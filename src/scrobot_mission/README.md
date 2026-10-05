# scrobot_mission

Autonomous court-clearing mission and local shuttle collection for SC Robot.

## Mission flow

```text
initial AprilTag approach
  -> initial relocalization
  -> start Nav2
  -> join four-pass sweep
  -> SWEEPING
       |
       +-- eligible shuttle detected
       |     -> save checkpoint pose + sweep index
       |     -> cancel FollowPath
       |     -> LOCAL_COLLECT
       |     -> return to saved checkpoint with Nav2
       |     -> resume saved sweep progress
       |
       +-- fixed AprilTag station
             -> relocalize
             -> restore sweep heading
             -> continue
  -> COMPLETE
```

The installed executable `sweep_mission_manager` is the runtime wrapper around
the base sweep state machine. It adds fixed-station relocalization,
AUTO/MANUAL pause/resume, tag-search recovery, and optional test hooks.

## Collection eligibility

`shuttle_collection_filter` transforms each production 3D detection into
`base_link` and accepts it only when:

```text
0.50 m <= planar range <= 1.80 m
outside 0.60 m exclusion radius around either net pole
```

Output:

```text
/perception/collectable_shuttle_detections_3d
```

## Local collection

`/local_collect` freezes the first eligible detection in `odom`.

```text
freeze target
  -> build base_link pre-pose 1.10 m from shuttle
  -> pure SMC pose approach
  -> rho <= 0.03 m and |yaw error| <= 5 deg
  -> remain valid for 0.25 s
  -> straight collect at 0.30 m/s, omega = 0
  -> 0.10 m overrun
  -> select next unattempted eligible shuttle
```

Current shuttle SMC parameters:

```text
v_R      0.50 m/s
lambda   2.50
k_s      1.20
eta      0.40
phi      0.30
k_rho    0.80
omega max 1.00 rad/s
v max     0.50 m/s
```

## Launch

`sweep_mission.launch.py` starts global localization, Nav2, the collection
filter/controller, and the mission manager. It expects the robot/sensors,
production perception, control stack, and local EKF to already be running.

```bash
ros2 launch scrobot_mission sweep_mission.launch.py use_sim_time:=true
```

## Files

- `scrobot_mission/sweep_mission_manager.py` - base mission state machine.
- `scrobot_mission/runtime_sweep_mission_manager.py` - installed runtime extension.
- `scrobot_mission/patrol_sweep_path.py` - continuous four-pass path geometry.
- `scrobot_mission/shuttle_collection_filter.py` - mission eligibility gate.
- `scrobot_mission/local_collect_controller.py` - local collection action.
- `config/sweep_params.yaml`
- `config/local_collect.yaml`

Regression procedures: `../scrobot_debug/debug_md/scrobot_mission/README.md`.
