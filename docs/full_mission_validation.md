# Full-Mission Validation

The integrated Gazebo mission has completed successfully with the production
autonomy stack. The 50/50 result below is the **pre-freeze benchmark** recorded
before the final RGB calibration correction on 3 November 2026. It remains a
useful system benchmark, but it must not be labeled as the frozen simulation
baseline until the post-calibration acceptance run passes.

## Final repeatable run

Configuration:

```text
robot start     x=2.0 m, y=3.05 m, yaw=3.14159 rad
layout          mixed
shuttle count   50
layout seed     20261003
magnetometer    disabled
YOLO            gazebo_simple_v2.pt
evaluation      enabled
master RViz     enabled
```

Result:

```text
terminal state                   COMPLETE
duration                         652.438 s
total shuttles                   50
collected by ground truth        50
collection events received       50
GT - event delta                 0
remaining eligible               0
eligible collection rate         100.0%
overall collection rate          100.0%

local-collect passes             36
capture checks started           32
capture checks passed            31
capture checks failed            1
verified attempt success rate    96.875%

ground-truth path                140.318 m
estimated path                   141.524 m
position RMSE                    0.1116 m

fixed relocalizations            4
relocalization time              38.514 s
sweep time                       88.457 s
local-collection time            317.927 s
return-to-sweep time             181.982 s
```

The single failed capture check was a genuine missed pickup attempt: ground
truth stayed unchanged and no collection event was received for that attempt.
The mission subsequently recovered and still cleared all 50 shuttles. Therefore
mission completion/collection rate and per-attempt pickup reliability are
reported separately.

Several successful targeted passes removed more than one nearby shuttle, so the
number of verified target attempts is smaller than the number of physical
shuttle removals.

## Validated prerequisites

```text
robot description / TF                    PASS
control command pipeline                  PASS
AUTO/MANUAL arbitration                   PASS
depth scan + collision monitor            PASS
wheel/IMU EKF                             PASS
AprilTag detection/relocalization         PASS
tag approach strategy                     PASS
YOLO 2D + aligned-depth 3D                PASS
0.50-1.80 m collection range gate         PASS
local SMC shuttle collection              PASS
close-target pre-pose handling            PASS
multi-shuttle selection/reacquisition     PASS
physical Gazebo shuttle removal           PASS
local odometry evaluator                  PASS
collection-session evaluator              PASS
Nav2/RPP integration                      PASS
full 50-shuttle mission                   PASS
```

## Physical shuttle removal preflight

For isolated regression:

```bash
ros2 launch scrobot_debug smc_shuttle_check.launch.py \
  model_path:=$SCROBOT_YOLO_MODEL \
  launch_rviz:=false
```

A successful physical removal produces:

```text
GT_COUNT shuttles=0
COLLECTED total=1 ...
REMOVAL_PASS ...
```

The monitor verifies authoritative ground-truth disappearance and the
collection event independently of its lower-rate geometry display.

## One-command full regression

```bash
export SCROBOT_YOLO_MODEL=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt
ros2 launch scrobot_debug full_mission_check.launch.py
```

The master RViz is enabled by default and shows the court, robot, sweep path,
Nav2 active plan, current mission goal, relocalization stops, local collection
pre-pose, costmaps, shuttle truth, and ground-truth/estimated trajectories.

## Expected mission sequence

```text
INITIAL_TAG_APPROACH
 -> INITIAL_RELOCALIZATION
 -> STARTING_NAV2
 -> JOIN_SWEEP
 -> SWEEPING

SWEEPING
 -> save checkpoint
 -> cancel FollowPath
 -> LOCAL_COLLECT
 -> RETURN_TO_SWEEP
 -> resume saved sweep index
 -> SWEEPING

SWEEPING
 -> TURN_TO_TAG
 -> RELOCALIZING
 -> RESTORE_SWEEP_HEADING
 -> SWEEPING

 -> COMPLETE
```

## Evaluation

After the run:

```bash
ros2 run scrobot_evaluation analyze_collection_session
```

Mission-level acceptance:

1. terminal state is `COMPLETE`;
2. all eligible shuttles are removed;
3. authoritative GT count and collection-event count agree;
4. remaining eligible count is zero;
5. localization/relocalization remain operational;
6. collision monitoring remains active;
7. no manual intervention is required.

Per-attempt capture checks are a reliability diagnostic. A failed attempt is
reported as such, but it is not automatically a mission failure if the state
machine recovers and later removes the shuttle. Report both the final mission
collection rate and the verified pickup-attempt success rate.

## Optimization target

The final run shows that mission time is dominated by local collection and
return-to-sweep behavior rather than the sweep itself:

```text
sweep                88.457 s
local collection    317.927 s
return to sweep     181.982 s
```

These phases are the primary targets for future performance optimization.


## Simulation-freeze acceptance run

After the frozen camera calibration is applied, run the complete acceptance
sequence again before tagging the simulation release.

Camera requirements:

```text
RGB       1280x720 @ 15 Hz
fx, fy    1024.2141 px
cx, cy    640 px, 360 px
HFOV      64.0 deg

Depth     848x480 @ 15 Hz
fx        420.289855957031 px
HFOV      90.503557 deg
```

Acceptance sequence:

1. verify `/camera/color/camera_info` and `/camera/depth/camera_info`;
2. run AprilTag detection and global-localization regression;
3. run YOLO/aligned-depth regression across the 0.50-1.80 m collection range;
4. run the isolated SMC shuttle-removal regression;
5. run the multi-shuttle local-collection regression;
6. run sweep diversion, checkpoint return, and resume;
7. run the 50-shuttle mixed layout with seed `20261003`.

Frozen-baseline requirements:

```text
terminal state                   COMPLETE
collected by ground truth        50 / 50
collection events received       50
GT - event delta                 0
remaining eligible               0
eligible collection rate         100%
overall collection rate          100%
manual interventions             0
```

Record the exact YOLO model SHA256 and environment manifest produced by
`yoloshuttle/scripts/freeze_model_manifest.py` with the final result. Only
after this post-calibration run passes should the result replace the
pre-freeze benchmark above and the simulation release be tagged.
