# Mission / Navigation / Control Debug Commands

## Build

```bash
cd ~/scrobot_ws
colcon build --symlink-install --packages-select scrobot_mission
source install/setup.bash
```

## Start full patrol mission

```bash
ros2 launch scrobot_mission patrol_mission.launch.py
```

This launches:

- `scrobot_localization/global_localization.launch.py`
- `scrobot_navigation/navigation.launch.py`
- `shuttle_target_selector`
- `patrol_manager`

## Mission state

```bash
ros2 topic echo /mission/state
```

Important states:

```text
GO_TO_PATROL
PATROL_SCAN
SELECT_SHUTTLE
GO_TO_STAGING
WAIT_COLLECTION
CHECK_VISIBLE_SHUTTLES
LOCAL_SCAN
RETURN_TO_PATROL
FINAL_PATROL_SCAN
COMPLETE
ERROR
```

## Patrol sequence

```text
Pi
 -> 360 deg patrol scan
 -> shuttle found?
    -> staging pose
    -> collection request
    -> collection complete
    -> another shuttle currently visible?
       YES -> collect next immediately
       NO  -> local 360 deg scan
              -> shuttle found? collect again
              -> none found? return to Pi
 -> final 360 deg scan at Pi
 -> no shuttle found -> Pi+1
```

`Pi` remains the anchor for the entire collection excursion.

## Patrol points / current Nav2 goal

```bash
ros2 topic echo /mission/patrol_points --once
ros2 topic echo /mission/current_goal
```

## Target selector outputs

```bash
ros2 topic echo /mission/selected_shuttle_id
ros2 topic echo /mission/selected_shuttle
ros2 topic echo /mission/shuttle_staging_pose
```

## Target-selection control

```bash
ros2 topic echo /mission/target_selection_enabled
```

The patrol manager owns this topic during an integrated mission.

## Collection handoff

When the robot reaches the shuttle staging pose, the patrol manager publishes:

```bash
ros2 topic echo /mission/collection_request
```

Until the final-approach / collector node exists, simulate successful collection by publishing the same shuttle ID:

```bash
ros2 topic pub --once /mission/collection_complete std_msgs/msg/String "{data: '1'}"
```

Replace `1` with the ID shown on `/mission/collection_request`.

The mission will then:

1. ignore that completed track ID;
2. check for another shuttle currently in view;
3. collect it immediately if available;
4. otherwise perform a local 360 deg scan;
5. return to the original patrol point if the local scan is clear;
6. perform the final patrol-point scan.

## Nav2 actions

```bash
ros2 action list | grep -E 'navigate_to_pose|spin'
ros2 action info /navigate_to_pose
ros2 action info /spin
```

## Teleop

```bash
ros2 run scrobot_control wasd_teleop.py
```

Do not run teleop while the autonomous patrol mission is actively commanding Nav2 unless intentionally testing command arbitration.
