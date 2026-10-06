# scrobot_interfaces

Project-specific ROS 2 action definitions for SC Robot. This package contains
interfaces only and owns no runtime behavior.

## Actions

| Action | Server | Owner | Purpose |
| --- | --- | --- | --- |
| `ApproachTag` | `/approach_tag` | `scrobot_localization` | Search for a tag and reach the requested tag-facing stand-off pose. |
| `Relocalize` | `/relocalize` | `scrobot_localization` | Collect stationary tag samples and update `map -> odom`. |
| `LocalCollect` | `/local_collect` | `scrobot_mission` | Collect currently eligible visible shuttles until none remain or the action ends. |

Primary client for all three actions is
`scrobot_mission/sweep_mission_manager`.

## Rules

- Prefer standard ROS interfaces when they already fit.
- Treat action field changes as breaking interface changes.
- Keep units and sentinel semantics documented in the `.action` files.
- Runtime packages may depend on `scrobot_interfaces`; this package must not
  depend on runtime packages.
