# Simulation-to-real parameter comparison matrix

Record every proposed change as **real measurement -> sim parameter -> paired
test result**. No unmeasured physics tuning.

| Topic | Simulated current reference | Real-hardware source/limit | Measurement needed |
| --- | --- | --- | --- |
| Drive radius | 0.050 m (URDF/controllers) | CAD/nominal 0.050 m | Loaded rolling radius under robot weight |
| Wheel separation | 0.420 m | CAD/nominal | Effective track width from in-place rotations |
| Drive limit | 0.80 m/s Nav2 desired speed | STM32 ±100 RPM/wheel (~0.524 m/s straight) | Wheel response at safe speeds |
| Wheel command | Gazebo joint velocity | STM32 100 Hz PIDF + encoder feedback | Delay, rise time, deadband, saturation |
| Wheel traction | 1 mm Gazebo collision, mu=5 | Rubber and floor | Straight/turn slip and stopping distance |
| Robot inertia | URDF/CAD values | Assembled robot | Measured mass and COM |
| RGB/depth | 1280x720/848x480 at 15 Hz | Real D435i configured same nominal rates | Intrinsics, frame lag, depth precision |
| IMU | Gazebo simulated D435i at 100 Hz | D435i gyro + accel | Bias, gyro noise, lag and frame alignment |
| Magnetometer | Simulation enabled on this branch baseline | Physical bringup disabled | Whether 5883L physically exists, calibration |
| Tags | Four ideal 16h5 tags | Real court mounts | Corner positions, camera reprojection error |
| Collection | Ideal geometric removal rectangle | Brushes + conveyor | Pickup probability vs offset/speed |

## Profiles

- \`october_baseline\`: full mission \`use_magnetometer:=false\`;
  historical 50/50 evaluation result (not a fresh test).
- \`magnetometer_enabled\`: \`use_magnetometer:=true\`; not yet validated
  against the historical full-mission benchmark.
- \`hardware_matched\`: future profile after paired measurements exist.

Keep paired test runs with both code commit SHA and physical measurements.
