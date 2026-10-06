# scrobot_navigation Regression

Nav2 is normally exercised through the mission because the mission controls
lifecycle startup after global localization.

## Basic checks

After Nav2 is active:

```bash
ros2 node list | grep -E 'controller_server|planner_server|bt_navigator|behavior_server'
ros2 action list | grep -E 'navigate_to_pose|follow_path|spin'
ros2 topic hz /camera/camera/depth/scan
```

The current controller is Regulated Pure Pursuit with 0.80 m/s desired cruise
speed and a 0.50 m minimum regulated radius.

## Costmap / footprint

Both costmaps use:

```text
front +0.325 m
rear  -0.450 m
left/right +/-0.225 m
```

and consume `/camera/camera/depth/scan`.

## Mission integration

The most meaningful regression is:

```text
join sweep
 -> FollowPath
 -> interrupt for local collection
 -> NavigateToPose back to saved checkpoint
 -> resume FollowPath from saved index
```

Use the full-mission procedure in
`../../../../docs/full_mission_validation.md` for final acceptance.
