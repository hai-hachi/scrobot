# scrobot_interfaces

Project-specific ROS 2 actions used by the SC Robot stack.

This package contains interface definitions only. It does not own any runtime
behavior.

## Actions

### `ApproachTag.action`

Server:

```text
/approach_tag
```

Owned by:

```text
scrobot_localization/tag_approach_controller
```

Primary client:

```text
scrobot_mission/sweep_mission_manager
```

Purpose:

Search for a suitable AprilTag, lock a tag, and position the robot at a requested
stand-off distance before relocalization.

### `Relocalize.action`

Server:

```text
/relocalize
```

Owned by:

```text
scrobot_localization/tag_global_localizer
```

Primary client:

```text
scrobot_mission/sweep_mission_manager
```

Purpose:

Collect stationary AprilTag samples and commit a corrected `map -> odom`
transform.

### `LocalCollect.action`

Server:

```text
/local_collect
```

Owned by:

```text
scrobot_mission/local_collect_controller
```

Primary client:

```text
scrobot_mission/sweep_mission_manager
```

Purpose:

Run one local shuttle-collection spree until no eligible shuttle remains, the
action is canceled, or the overall timeout is reached.

## Design rules

- Keep this package free of runtime nodes.
- Do not add debug-only interfaces unless production code genuinely needs them.
- Prefer standard ROS messages/services/actions when they already fit.
- Changing field names, types, or order is a breaking interface change.
- Add comments for units, sentinels, and semantics before adding redundant
  fields.
- Production packages may depend on `scrobot_interfaces`; this package must not
  depend on production packages.

## Current dependencies

```text
ament_cmake
rosidl_default_generators
rosidl_default_runtime
```
