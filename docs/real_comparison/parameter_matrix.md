# Simulation-to-real parameter comparison matrix

Record every proposed change as **real measurement -> sim parameter -> paired
test result**. No unmeasured physics tuning.

| Topic | Simulated current reference | Real-hardware source/limit | Measurement needed |
| --- | --- | --- | --- |
| Drive radius | 0.050 m (URDF/controllers) | CAD/nominal 0.050 m | Loaded rolling radius under robot weight |
| Wheel separation | 0.420 m | CAD/nominal | Effective track width from in-place rotations |
| Drive limit | Nav2 desired speed **0.35 m/s**, smoother max **0.50 m/s** straight; separate yaw cap 2.30 rad/s | Real interface caps wheel commands at **±95 RPM** (~0.497 m/s); STM32 hard cap remains 100 RPM | Coupled wheel-speed envelope and loaded RPM response at 90 RPM |
| Wheel command | Direct Gazebo joint velocity by default; optional published simulation-only first-order RPM surrogate at 100 Hz | STM32 100 Hz PIDF + encoder feedback; separate floor-loaded WL and WR 90 RPM validation logs | Left/right transient RMSE, 2% settling (~0.21 / 0.22 s), deadband and saturation |
| Wheel traction | Published Mobility Lab calibration YAML (baseline: 1 mm collision width, drive mu=5, caster mu=0.05, zero contact slip and bearing damping) | Rubber wheels + loaded court floor | Requires real pose/yaw or displacement, not only wheel RPM; use straight/turn slip and stopping distance |
| Robot inertia | URDF/CAD values | Assembled robot | Measured mass and COM |
| RGB/depth | 1280x720/848x480 at 15 Hz | Real D435i configured same nominal rates | Intrinsics, frame lag, depth precision |
| IMU | Gazebo simulated D435i at 100 Hz | D435i gyro + accel | Bias, gyro noise, lag and frame alignment |
| Magnetometer | Simulation enabled on this branch baseline | Physical bringup disabled | Whether 5883L physically exists, calibration |
| Tags | Four ideal 16h5 tags | Real court mounts | Corner positions, camera reprojection error |
| Collection | Ideal geometric removal rectangle | Brushes + conveyor | Pickup probability vs offset/speed |

## Profiles

- `october_baseline`: full mission `use_magnetometer:=false`;
  historical 50/50 evaluation result (not a fresh test).
- `magnetometer_enabled`: `use_magnetometer:=true`; not yet validated
  against the historical full-mission benchmark.
- `hardware_matched`: future profile after paired measurements exist.

Keep paired test runs with both code commit SHA and physical measurements.

## Calibration ownership and test isolation

- `tools/mobility_lab/` measures floor-loaded motor response. The September 22 WL/WR 90 RPM experiments are independent tests; their combined CSV columns are aligned for plotting, **not simultaneous real chassis measurements**. **Save** writes lab settings; **Publish** updates `src/scrobot_simulation/config/mobility_calibration.yaml`. A simulation restart is required; `robot.launch.py` and physical STM32 remain unchanged.
- `tools/odometry_lab/` measures timing, encoder fidelity and estimator-vs-Gazebo ground truth. It does **not** change the production wheel odometry publish rate or motor model. Verify the simulated motor calibration before using it to infer real motion accuracy.
- `tools/lighting_lab/` owns the published court lighting/visual materials only. Its saved 2×3 / intensity 2.5 calibration is independent of drivetrain contact physics.
- Command-path simplification to a mission-authorized source router is a **design proposal, not yet implemented** on this branch. The current pipeline still has autonomous and AUTO/MANUAL twist mux arbitration, velocity smoothing, collision monitoring and differential-drive control.
