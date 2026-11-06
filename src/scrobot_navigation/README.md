# scrobot_navigation

Nav2 navigation and the AprilTag **approach** action live here.

- Nav2 follows sweep routes and returns to saved checkpoints.
- `tag_approach_controller.py` provides `/approach_tag` and
  `/cmd_vel_relocalization` with the same controller strategies and
  parameters as before this package split.
- The `scrobot_localization/global_localization.launch.py` composition
  still launches the approach controller for existing regression callers, but
  its executable now comes from `scrobot_navigation`.

The original localization executable remains as a thin compatibility wrapper.
Global AprilTag *measurement*, stationary sample gating, and ownership of
`map -> odom` stay in `scrobot_localization`.

No approach strategy, geometry, ROS topics/actions, or controller gains changed.
