# Simulation Systematic Test Plan

Baseline branch: `simulation-systematic-test-c98fee65`  
Baseline commit: `c98fee65b3829f0fefbeb3930ca6d3c46c483e58`

## Test rule

Only one subsystem or parameter group is changed at a time.

For every test:

1. Use the same robot spawn pose.
2. Use the same shuttle layout seed.
3. Use the same shuttle count and distribution.
4. Run the collection-session evaluator.
5. Record mission completion, collected/remaining shuttles, total time, path length,
   sweep time, local-collection time, return-to-sweep time, relocalization count,
   and any controller/localization errors.
6. If the change improves the intended behavior without causing a regression,
   keep it and use that commit as the new test baseline.
7. If it causes a regression, revert only that change.

Default repeatable mission layout:

- mode: mixed
- count: 50
- seed: 20261003
- visual: detail

## Stage 0 - Historical baseline

- [ ] Build the branch at exact commit c98fee65.
- [ ] Run the fixed-seed full mission.
- [ ] Confirm SMC local collection works with the historical parameters.
- [ ] Record baseline evaluator results.
- [ ] Save important warnings/errors and mission-completion status.

Historical local-collect values:

- max target range: 2.00 m
- pole exclusion radius: 0.60 m
- precollect distance: 0.50 m
- precollect position tolerance: 0.03 m
- precollect yaw tolerance: 5 deg
- straight collect speed: 0.30 m/s
- overrun distance: 0.10 m
- SMC v_R: 0.50 m/s
- lambda: 2.00
- k_s: 1.60
- eta: 0.50
- phi: 0.08
- k_rho: 0.80
- omega max: 1.80 rad/s
- heading stop: 70 deg

Do not modify SMC before Stage 0 is complete.

## Stage 1 - Robot geometry and footprint

Test independently:

- [ ] Nav2 rear footprint: -0.335 m -> -0.450 m.
- [ ] Confirm no new path-planning or return-to-sweep failures.
- [ ] Confirm collision geometry matches the physical base envelope.

## Stage 2 - Camera mounting geometry

Historical baseline:

- x = 0.120 m
- z = 0.220 m
- pitch = 7 deg

Target project geometry:

- x = 0.110 m
- z = 0.2275 m
- pitch = 15 deg downward

Tests:

- [ ] Change camera position only.
- [ ] Test perception/FOV and full mission.
- [ ] Change pitch only.
- [ ] Test perception/FOV and full mission.
- [ ] Keep the final geometry only if shuttle visibility and navigation remain valid.

## Stage 3 - Shuttle perception range

Historical fake-detector range:

- min = 0.20 m
- max = 3.00 m

Target project range:

- min = 0.17 m
- max = 1.68 m

Tests:

- [ ] Change min range only: 0.20 -> 0.17 m.
- [ ] Change max range only: 3.00 -> 1.68 m.
- [ ] Confirm interruption timing remains practical.
- [ ] Confirm enough shuttles become visible on the four-pass sweep.
- [ ] Confirm local collection is not triggered too late for difficult bearings.

## Stage 4 - Mission collection eligibility

Historical values:

- mission max target range = 2.00 m
- pole exclusion radius = 0.60 m

Target values:

- mission max target range = 1.68 m
- pole exclusion radius = 0.10 m

Tests:

- [ ] Set mission max target range to 1.68 m only.
- [ ] Run fixed-seed mission.
- [ ] Set pole exclusion radius to 0.10 m only.
- [ ] Confirm net-post collisions do not occur.
- [ ] Confirm evaluator uses the same exclusion radius as the mission.

## Stage 5 - Sweep geometry

Required final behavior:

- four continuous serpentine passes
- complete court coverage
- preserved sweep progress during local collection

Tests:

- [ ] Verify baseline already produces four passes.
- [ ] Inspect lane spacing and edge coverage from camera FOV.
- [ ] Verify U-turns stay feasible.
- [ ] Verify shuttle diversion preserves checkpoint pose and path index.
- [ ] Verify return-to-sweep resumes the original path rather than restarting.

## Stage 6 - Local collection controller

Do not change this stage until Stages 0-5 have identified whether external
geometry/perception changes are responsible for regressions.

Tests:

- [ ] Single shuttle directly ahead.
- [ ] Single shuttle at +30 deg bearing.
- [ ] Single shuttle at -30 deg bearing.
- [ ] Single shuttle near maximum eligible range.
- [ ] Multiple shuttles in the same FOV.
- [ ] Confirm SMC reaches the 0.50 m pre-pose.
- [ ] Confirm transition to straight collection.
- [ ] Confirm collector reaches target.
- [ ] Confirm 0.10 m overrun.
- [ ] Confirm next target selection works.
- [ ] Only tune SMC if the historical controller fails under unchanged baseline geometry.

## Stage 7 - Localization sensor model

Historical baseline includes a separate magnetometer.

Final project target uses D435i IMU without the HMC5883L.

Tests:

- [ ] Measure wheel + IMU EKF drift with historical sensor configuration.
- [ ] Disable magnetometer input only.
- [ ] Compare yaw drift on identical trajectories.
- [ ] Verify AprilTag global correction still restores map pose.
- [ ] Keep no-magnetometer configuration only after confirming acceptable mission behavior.

## Stage 8 - AprilTag localization

- [ ] Startup tag detection works.
- [ ] Spin-in-place can acquire a tag.
- [ ] Initial global relocalization succeeds.
- [ ] Fixed relocalization stations succeed.
- [ ] Tag correction does not create large map/odom jumps during motion.
- [ ] Verify tag16h5 configuration and fixed map poses remain unchanged.

## Stage 9 - Startup recovery logic

Add only after normal startup localization is validated.

- [ ] Full spin with a visible tag succeeds normally.
- [ ] Full spin with no visible tag ends cleanly.
- [ ] Robot returns to last successful relocalization area using Nav2.
- [ ] Nav2 stops before the next spin.
- [ ] Tag search retries.
- [ ] Retry count prevents an infinite recovery loop.

## Stage 10 - Obstacle and Nav2 behavior

- [ ] D435i depth scan reaches Nav2 costmaps.
- [ ] Static poles/net are avoided.
- [ ] Autonomous collision monitor can stop unsafe motion.
- [ ] RPP reaches 0.80 m/s on long sweep traverses.
- [ ] U-turns remain stable.
- [ ] Return-to-sweep is reliable.
- [ ] Investigate controller-frequency warnings separately from mission logic.

## Stage 11 - Full mission robustness

Run identical evaluation procedure for at least:

- [ ] mixed, seed 20261003
- [ ] mixed, second fixed seed
- [ ] random, fixed seed
- [ ] cluster, fixed seed

For the final configuration, record:

- total mission time
- path length
- collected count
- remaining normal-area count
- excluded near-pole count
- collection rate
- sweep time
- local-collection time
- return-to-sweep time
- AprilTag relocalization count
- localization RMSE
- mission errors/timeouts

## Final acceptance

The selected configuration should:

- complete the entire four-pass sweep
- autonomously interrupt for reachable shuttles
- collect them without local-controller deadlocks
- return to the saved sweep checkpoint
- periodically correct localization using AprilTags
- avoid net posts and obstacles
- leave only intentionally excluded/unreachable shuttles
- complete without human intervention
