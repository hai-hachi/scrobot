# scrobot_collection

Owns **shuttle collection eligibility, local SMC approach, and pickup sequence**.
The existing `/local_collect` action, `/cmd_vel_approach` output,
`/mission/local_collect_phase` status, and eligibility topic are unchanged.

```bash
ros2 launch scrobot_collection collection.launch.py
```

The production `scrobot_mission/sweep_mission.launch.py` composes this launch.
For backward compatibility, the old `scrobot_mission` executable names still work
as thin forwarding entrypoints, but the implementation lives in this package.

**Parameters**: `config/local_collect.yaml` is canonical. The old copy in
`scrobot_mission/config/local_collect.yaml` is temporarily retained for legacy
test launches and must match the canonical file until those references migrate.

The ROS-side collection sequence is shared between simulation and the real
robot; matching physical brush/conveyor behavior is a separate calibrated task.
