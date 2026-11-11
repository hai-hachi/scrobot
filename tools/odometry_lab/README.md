# SCROBOT Odometry Lab

A standalone instrumented test bench for **four** questions: publishing timing,
resource/performance cost, real encoder fidelity, and wheel/EKF odometry accuracy
versus independent Gazebo ground truth. Lives in `scrobot_ws/tools/odometry_lab`.

## Architecture and independence

This lab reads the existing Jazzy/Gazebo Harmonic SCROBOT topics:

- `/joint_states` (left_wheel_joint and right_wheel_joint)
- `/diff_drive_controller/odom` (production wheel odometry, default 50 Hz)
- `/odometry/filtered` (production EKF)
- `/evaluation/ground_truth_odom` (Gazebo world truth; **never** used in estimator)
- `/evaluation/local_odom_test_state` (optional test segment label)
- `/clock` (simulation-time vs monotonic wall-time RTF)

It publishes **only** `/odometry_lab/encoder_odom` (lab-specific coordinate
frame, **no TF broadcast**) for the encoder quantization/delay experiment.
It will not overwrite `controllers.yaml`, the production EKF, `/odom`, or `/tf`.
Change the simulator itself in the Mobility Lab; encoder emulation should never
be used to hide missing physical wheel slip or floor-contact effects.

**Prerequisites:** SCROBOT workspace built and sourced, `ros2 run scrobot_evaluation
local_odom_test_runner` installed, `rclpy`, `nav_msgs`, `sensor_msgs`,
`rosgraph_msgs`, `std_msgs`, `python3-yaml`, `python3-numpy`, `python3-matplotlib`,
optionally `python3-psutil`. Start the Gazebo simulation,
production control stack, perception depth scan / collision monitor and
localization before issuing test motion. Do not launch two test runners at once.

```bash
cd ~/scrobot_ws
colcon build --symlink-install
source install/setup.bash
# Separate terminal: start your normal simulation + production control stack.
python3 tools/odometry_lab/gui.py
```

Headless (passive observation; **no motion**):

```bash
python3 tools/odometry_lab/run_lab.py --scenario none
# Ctrl+C to finish and analyze.
```

Straight-line motion (90-RPM **target**, 0.4712 m/s for 0.05 m wheel radius):

```bash
python3 tools/odometry_lab/run_lab.py --scenario straight --publish-hz 50
```

Other scenarios: `static`, `rotate`, `arc`, `suite`. Motion is through the
**existing** `local_odom_test_runner` → production manual mode / smoother /
collision monitor / diff_drive_controller. The test runner restores AUTO when
it completes normally. On interruption, the launcher attempts a zero command
and mode restoration (not a hardware E-stop). This is intended for simulation.

## Four test areas (plus Overview)

1. **Timing**: independent encoder-sample, estimator and lab-publication Hz,
   lab-only QoS reliability and queue depth, plus the existing motion
   test runner command publish rate,
   measured **wall receive rate** versus **message-stamp rate**, p95
   interarrival period and repeated stamps. Producer internal cycles cannot be
   proven from subscriber Hz alone. Missing intervals are *not* proven DDS drops.
2. **Performance**: simulation RTF derived from `/clock`, lab/system CPU and
   RSS, approximate Gazebo process CPU, pending delayed-sample queue, real-time monitoring while Gazebo renders.
3. **Hardware**: wheel encoder ticks/rev, gear ratio, wheel radius/separation,
   radius mismatch, noise, feedback delay and jitter, drop probability. Uses
   *wheel joints*, not ground-truth pose. Gear ratio converts wheel joint angle
   into encoder motor-shaft ticks. Noise and dropped ticks are **experimental
   assumptions**, not measured hardware facts.
4. **Accuracy**: trajectories and offline metrics for both production wheel
   odom, EKF, and lab encoder odom. Analysis interpolates to ground-truth
   timestamps and aligns origins/heading before computing pose RMSE, yaw RMSE,
   endpoint drift and velocity error. Stationary test automatically scores all
   matched samples when no active-motion samples are labeled.

**Apply settings writes `config/lab.yaml`**; a currently running test keeps its immutable
`config_snapshot.yaml` and is unaffected. Start begins a fresh run directory.

## Reproducibility and sweeps

Run 10, 20, 50, 100, 200 Hz with `--publish-hz`; this changes **only the lab
stream** and measures the added topic cost. It does **not** silently alter the
production `diff_drive_controller` publish rate. To test production rate,
change its `publish_rate` in a separate controlled simulation launch, then
restore it. Keep source feedback/estimator Hz fixed when sweeping publish Hz.
For head-to-head accuracy, reset robot pose and simulator state between trials.
With `stress.cpu_duty` > 0 in YAML, a separate bounded process introduces
synthetic CPU competition; production lighting/camera load must still be
configured in Gazebo itself.
Record RTF and the complete config for every trial; repeat several times.

Output root by default: `~/scrobot_ws/evaluation_results/odometry_lab`.
Each run includes `config_snapshot.yaml`, `samples.csv`, `clock.csv`,
`live.json`, `wheel_speeds.csv`, `summary.json`, `summary.csv`, `trajectory.svg`, `monitor.log`
and `motion.log` (if motion was requested). Use:

```bash
python3 tools/odometry_lab/analyze.py PATH_TO_RUN
```

## Real hardware comparison

The two real 90-RPM PID validation files are not stored in this repository.
Replay their actual time/RPM columns once available:

```bash
python3 tools/odometry_lab/replay.py \
  --left WL_pid_validation_p90_0rpm_20260922_121710.csv \
  --right WR_pid_validation_p90_0rpm_20260922_121700.csv \
  --time-column TIME_COLUMN --rpm-column RPM_COLUMN \
  --sim-wheel-speeds PATH_TO_RUN/wheel_speeds.csv
```

The script outputs real motor-speed sample rate, mean RPM, standard deviation,
and RMSE versus a 90-RPM target. With simulated joint-speed CSV it also
computes time-aligned RPM RMSE (assuming the recordings have matching test
start and command profiles). Replace placeholder column names with the
actual CSV headers; verify which recording is left/right before use. **Real
wheel speed data is not real pose ground truth**: hardware trajectory accuracy
requires a separate measured pose reference (e.g. motion capture or survey).

## Known boundaries

- This is a stand-alone lab. The actual robot's measured Mobility Lab PID motor
  transfer function must already be configured in Gazebo for a realistic motion
  comparison; this tool does not install or recalibrate a Gazebo motor plugin.
- Encoder emulation is fed by `/joint_states`: its sampling information cannot
  exceed the actual upstream joint-state refresh rate.
- Delay and jitter affect lab encoder data in **simulation time**; separate
  ROS executor/transport delays are observed in timing metrics.
- System-wide CPU and a best-effort Gazebo process CPU estimate are reported
  when psutil is installed; use `htop`, `pidstat`, and Gazebo profiling for
  more accurate per-process resource attribution.
- Live plot is a quick-look starting-pose-aligned trajectory; offline SVG
  and accuracy metrics additionally use timestamp interpolation.
- No hardware controller commands are sent directly by the monitor. Use the
  real robot emergency stop when operating physical equipment.

## Browser dashboard (recommended)

The lab now uses a **local web dashboard** instead of Tkinter. The server uses
Python's standard-library HTTP server, bundled HTML/CSS/JavaScript, and the
same ROS 2 monitoring backend as the previous desktop GUI. No external CDN,
Node.js installation, or internet connection is needed. Only localhost can
access the web server; it binds to `127.0.0.1` and requires same-origin JSON
for control requests. This interface does not depend on the Lighting Lab web
source, which is not currently present in the connected GitHub repository.

```bash
cd ~/scrobot_ws
source install/setup.bash
python3 tools/odometry_lab/web_server.py
# Open http://127.0.0.1:8767 (opens automatically unless --no-browser)
```

To launch headless or choose another port:

```bash
python3 tools/odometry_lab/web_server.py --no-browser --port 8767
```

`python3 tools/odometry_lab/gui.py` remains as a backwards-compatible alias
and now opens the web dashboard. Tkinter is no longer required.

The browser has five areas: Overview, Publishing, Performance, Hardware model,
and Accuracy. Sliders and numeric inputs are linked; **Apply settings** writes
`config/lab.yaml`, **Start experiment** invokes `run_lab.py`, and **Stop**
requests the launcher's normal cleanup, which attempts a zero manual command
and AUTO restoration. You can export CSV, YAML, summary JSON and SVG when the
run files become available. Each new experiment snapshots its saved settings.

For motion scenarios (anything except `none`), the web server checks that
`/clock`, the Gazebo ground-truth topic and `/joint_states` exist, and that the
first two topics have active publishers **before allowing Start**. It also
requires an explicit browser confirmation. This is a simulation-only guard,
not proof that no physical motor controller is connected and not a replacement
for an emergency stop. Do not expose the dashboard publicly or port-forward it
without a separate authentication/security layer.

**Important frequency distinction:** the Publishing controls change the lab's
isolated encoder sampling / estimation / publication rates. They never edit
the production diff-drive controller `publish_rate` or EKF transform ownership.
The `P95 interval` KPI is an inter-arrival interval, not the formal jitter
standard deviation. Published performance numbers and trajectory previews are
illustrative until live Gazebo signals are present; accuracy is finalized only
by time-aligned offline analysis after a run.

Validation without ROS 2:

```bash
python3 -m pytest -q tools/odometry_lab/tests
python3 -m py_compile tools/odometry_lab/web_server.py
```
