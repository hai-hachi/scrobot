# scrobot_navigation

Nav2 configuration for SC Robot court traversal, sweep following, recovery, and
return-to-sweep navigation.

## Current controller

Regulated Pure Pursuit (`FollowPath`) is tuned for:

```text
desired linear velocity  0.80 m/s
lookahead time           0.80 s
lookahead distance       0.45-0.64 m
minimum regulated radius 0.50 m
rotate-to-heading        enabled
```

Both local and global costmaps consume the filtered depth scan:

```text
/camera/camera/depth/scan
```

The robot footprint matches the physical base envelope:

```text
front +0.325 m
rear  -0.450 m
left/right +/-0.225 m
```

## Lifecycle

`navigation.launch.py` starts controller, planner, behavior server,
BT navigator, and the navigation lifecycle manager with `autostart: false`.

The mission manager starts Nav2 only after initial AprilTag localization has
established the global transform.

## Launch

```bash
ros2 launch scrobot_navigation navigation.launch.py use_sim_time:=true
```

Normally this launch is included by `scrobot_mission/sweep_mission.launch.py`.

## Files

- `config/nav2_params.yaml` - planner, RPP, costmaps, behaviors.
- `launch/navigation.launch.py` - Nav2 node composition.

Regression procedures:
`../scrobot_debug/debug_md/scrobot_navigation/README.md`.
