# SCROBOT Mobility Lab — floor-loaded motor and tire calibration

A localhost browser dashboard, intentionally separate from Lighting Lab and
the normal SCROBOT autonomy stack. The experiment uses the **real SCROBOT
diff_drive_controller**, the **Gazebo floor-loaded wheel/caster contacts**, and
the standard wheel-joint velocity feedback.

**Current milestone: 90 RPM wheel step-response identification.** You can
measure the default simulator first, then compare a switchable first-order
closed-loop RPM surrogate (tau initially 0.05 s) against your real MATLAB data.
Wheel-ground friction, caster friction, viscous **joint** damping, and collision
width are adjustable at respawn. The actual Gazebo ground truth is recorded
independently of encoder-derived body velocity.

## Start

Prerequisites: ROS 2 Jazzy, Gazebo Harmonic, `gz_ros2_control`,
`ros_gz_bridge`, built SCROBOT workspace, and Python 3. No added pip
dependencies (browser/web server use Python standard library).

```bash
cd ~/scrobot_ws/src
git switch real-comparison
git pull --ff-only
cd ~/scrobot_ws
colcon build --symlink-install --packages-up-to scrobot_simulation
source /opt/ros/jazzy/setup.bash
source ~/scrobot_ws/install/setup.bash
python3 ~/scrobot_ws/tools/mobility_lab/server.py
```

Open **http://127.0.0.1:8766**. A separate Gazebo GUI starts automatically
using the existing badminton court world and a lightweight visual robot. For
laptops without a browser, use `--no-browser`.

The tool defaults to **ROS_DOMAIN_ID=77** and a dedicated
`GZ_PARTITION` to avoid commanding any robot or pre-existing Gazebo process
in the default production domain. A deliberate `--ros-domain-id` override
is available. The lab publishes **only** to the simulation
`/diff_drive_controller/cmd_vel` inside its isolated domain.

Do not launch the full mission in the same lab domain. The standalone
`--attach` option is only for a separately managed **isolated Mobility Lab**
Gazebo process; it never launches or stops that process.

## First test (matches floor-loaded MATLAB identification)

1. Set **Left wheel only**, **90 RPM**, **Baseline**.
2. Keep default physical settings (drive mu 5.0, caster mu 0.05, joint damping
   0.0 N m s/rad, collision width 0.001 m, and contact slip1/slip2 0). These are **existing simulation
   values**, **not measured material constants**.
3. Wait for fresh joint states. Click **Run 90 RPM test**. The tool holds
   zero for 0.30 s, applies +90 RPM to the left wheel for 1.50 s, then commands
   zero and records another 0.65 s.
4. Repeat on the right wheel, then forward, reverse and spin profiles. For
   identical starting poses, click **Restart Gazebo with physics** before each
   run. Wait for wheel states to become ready again.
5. Save each recording; a complete CSV and metadata JSON are saved beneath
   `~/scrobot_ws/evaluation_results/mobility_lab/recordings/`.

The command generator uses `r=0.050 m` and wheel separation `L=0.420 m`:

```text
v   = pi*r*(left_rpm+right_rpm)/60
yaw = 2*pi*r*(right_rpm-left_rpm)/(60*L)
```

At 90 RPM, each tire tangential speed is ~0.471 m/s, below the measured
0.50 m/s maximum. Opposite-sign ±90 RPM gives ±2.244 rad/s body yaw
command. These are **wheel kinematic targets**, not a claim that actual
floor motion is identical.

### Data streams and interpretation

```text
Requested wheel RPM   -> 1st-order optional command prefilter
                       -> differential-drive ROS2 controller
                       -> gz_ros2_control wheel velocity interface
                       -> Gazebo wheel joints + floor contacts

Actual wheel RPM          /joint_states [wheel joint velocity]
Ground-truth chassis      /evaluation/ground_truth_odom
Controller limited twist  /diff_drive_controller/cmd_vel_out
```

The sample clock is Gazebo/ROS simulation time, not browser poll time; web
plots may be downsampled but saved CSV retains all recorded messages. Metrics
for each active wheel include the 10–90% rise time, 63.2% crossing
(approximate time constant), final RPM, overshoot and **2% settling time**.
If the wheel never stays inside the 2% target band through the step hold,
settling is **unavailable** rather than silently counted as success.

A true `0.20 s` **2%-criterion** settling time for a unity-gain first-order
system gives

```text
tau = 0.20 / -ln(0.02) = 0.0511 s.
```

The initial `tau_left=0.05`, `tau_right=0.05` are sensible starting values,
not calibrated outcomes. If MATLAB reported a 5% criterion, the equivalent
first-order tau is about 0.0668 s.

### Bundled real floor-loaded 90 RPM PID validation (2026-09-22)

Two original STM32 logs are committed under
`tools/mobility_lab/data/raw/`:

- `WL_pid_validation_p90_0rpm_20260922_121710.csv` — **903** left-wheel
  samples; independent left-wheel-only test
- `WR_pid_validation_p90_0rpm_20260922_121700.csv` — **903** right-wheel
  samples; independent right-wheel-only test

Each run records approximately **100 Hz** for **9.02 seconds**: roughly two
seconds at 0 RPM, five seconds commanded +90 RPM, then two seconds commanded
zero. The command's actual rising-edge timestamp is approximately 2.0106 s
(left) and 2.0104 s (right), so aligning the file's first sample to
`t=0` would be incorrect.

The lab reformats these files into `data/floor_loaded_20260922/`:

- `WL_90rpm_aligned.csv` / `WR_90rpm_aligned.csv`: all original samples,
  exact test phases, command and measured RPM, control ticks and encoders;
  `time_s=0` at that motor's rising command edge
- `reference_step_90rpm.csv`: 501 paired side-by-side *individually aligned*
  values for the 0–5.0 s step; uniform 10 ms grid via interpolation
- `reference_stop_90rpm.csv`: 101 separately aligned values after the
  falling command edge (0–1.0 s)
- `manifest.json`: PID gains, original filenames, actual step/stop
  timestamps, timing/provenance and non-simultaneity warning

**Important: WL and WR were separate experiments, not a simultaneous
two-wheel drive test.** We compare the real left reference ONLY to a
`Left wheel only` 90 RPM lab test, and the right reference ONLY to a
`Right wheel only` 90 RPM test. For simultaneous forward, reverse, or
in-place spin experiments, the GUI shows the ideal first-order curve
but intentionally suppresses real-vs-simulation RMSE rather than implying
the datasets validate those chassis motions.

The real file's 2%-band settling times from each command edge are
approximately **0.210 s (WL)** and **0.220 s (WR)**. The recorded response
is not a perfect delay-free single exponential: both have an initial
~20–30 ms latency and a slower small residual approach to 90 RPM. The
ideal `tau=0.05 s` model is a starting point, not a proven best fit.

**These references load automatically** every time you launch the lab.
There is nothing to import manually for this September dataset. The
lab time-aligns each source's command rising edge for the acceleration
phase and separately time-aligns the falling edge to the lab's selected
`hold_s`, so a 1.5 s simulated hold is not compared to a real motor that
continued at 90 RPM for five seconds.

### Import or override real MATLAB results

Export a CSV whose time zero is aligned with the **start of the 90 RPM
command**. A single motor column is enough for single-wheel tests:

```csv
time_s,left_actual_rpm,right_actual_rpm
0.00,0.0,0.0
0.01,8.3,0.0
0.02,24.5,0.0
```

Alternatively include only `time_s,left_actual_rpm` or
`time_s,right_actual_rpm`. The lab accepts alternative names `time`, `t`,
`left_rpm`, `right_rpm`. The importer rejects duplicate/nonmonotonic times
and non-finite samples. A manually imported CSV **overrides** the
automatically bundled reference in the running GUI and persists unchanged
under `evaluation_results/mobility_lab/real_reference/`. The override
is treated as a step-only reference unless a suitable independent stop
file is added. Click **Restore bundled data** to return to the immutable
September WL/WR recordings.

In MATLAB, for aligned vectors `t`, `leftRPM` and `rightRPM`:

```matlab
T = table(t(:), leftRPM(:), rightRPM(:), ...
  'VariableNames', {'time_s','left_actual_rpm','right_actual_rpm'});
writetable(T, 'scrobot_real_90rpm.csv');
```

The comparison displays **interpolated RPM RMSE**, based on available
time overlap. It does not automatically identify gain or dead time from the
real data.

## Tuning physics correctly

The experiment keeps the loaded robot and actual floor geometry active.
Parameters in the web page:

| Setting | Scope | Baseline | Applied |
| --- | --- | ---: | --- |
| Drive tire `mu1/mu2` | Wheel-ground traction | 5.0 | Respawn |
| Caster `mu1/mu2` | Caster-ground drag/traction | 0.05 | Respawn |
| Wheel joint damping | Bearing viscous torque | 0.0 | Respawn |
| Rolling contact slip1 | Tire longitudinal force-dependent slip | 0.0 | Respawn |
| Lateral contact slip2 | Tire lateral force-dependent slip | 0.0 | Respawn |
| Wheel collision width | Tire contact geometry | 0.001 m | Respawn |
| Motor tau / gain | Closed-loop RPM command surrogate | 0.05 s / 1.0 | Next test |

Use **joint RPM response** to evaluate the motor model, and compare
**encoder-predicted translation/yaw against Gazebo ground-truth body motion**
to evaluate slip/contact effects. Real chassis position, IMU yaw-rate or
external motion capture are needed before ground-friction parameters can be
calibrated to reality; a 90 RPM encoder trace alone cannot identify tire
friction, damping or wheel slip uniquely.

Gazebo contact-slip parameters are available for experiments but may be treated
differently by different physics engines. A setting has not been physically
validated merely because it is accepted in SDF. Verify effects with independent
ground-truth body displacement and yaw before tuning them against real data.

**Physical-model limitation (important):** stock
`gz_ros2_control/GazeboSimSystem` applies the requested wheel joint velocity
directly. The optional first-order mode implemented here filters the wheel
**command** before that interface. It creates the desired closed-loop RPM
response for comparison, but does NOT model torque/current limits or make
the RPM transient dynamically load dependent. Joint damping changes may be
masked by velocity actuation. Future torque/effort-based simulation can
investigate load-dependent dynamics if the measured data justifies it.

The GUI marks all physical parameter edits as **restart required**.
A restart respawns the robot using the supplied Xacro parameters. It never
writes to the production badminton court, the main robot URDF defaults,
Lighting Lab calibration, or the normal control pipeline. User settings
persist in `evaluation_results/mobility_lab/settings.json`.

## Save vs Publish calibration to the NORMAL simulation

The dashboard now has **two distinct actions**:

| Action | Writes | When values take effect |
| --- | --- | --- |
| **Save Lab Settings** | `~/scrobot_ws/evaluation_results/mobility_lab/settings.json` | Changes motor surrogate settings on the next lab run; contact physics after restarting lab Gazebo |
| **Publish to Simulation** | `~/scrobot_ws/src/scrobot_simulation/config/mobility_calibration.yaml` | After rebuilding/sourcing if needed and restarting NORMAL Gazebo |

**Publish** only reads previously saved settings, validates all values and
asks for a browser confirmation; unsaved slider changes prevent publishing.
The publication is an atomic single-file replacement and first creates a
byte-for-byte backup under
`~/scrobot_ws/evaluation_results/mobility_lab/published_backups/`.

To use published settings in the full SCROBOT simulation:

```bash
# After publishing from the Mobility Lab web GUI:
cd ~/scrobot_ws
colcon build --symlink-install --packages-up-to scrobot_bringup
source install/setup.bash

# Restart the normal simulation (stop any previous instance first):
ros2 launch scrobot_bringup sim.launch.py
```

The **simulation-only** YAML has two sections: `physics` (drive/caster contact
friction, contact slip, wheel-joint damping and collision width), and `motor`
(model selection, gains, independent left/right time constants). The
`scrobot_simulation/simulation.launch.py` defaults read this file for all
normal Gazebo physical settings; explicit launch arguments can override
individual physics values for isolated experiments.

When `motor.model: first_order`, the **normal**
`scrobot_bringup/sim.launch.py` additionally launches
`scrobot_simulation/sim_motor_response`. The collision monitor publishes to
`/cmd_vel_motor_target` and the response node translates this to filtered
left/right wheel setpoints before publishing to
`/diff_drive_controller/cmd_vel`. This uses the existing downstream
differential-drive joint velocity controller. When `model: baseline`, the
sim-only node is absent and the collision monitor goes directly to
`/diff_drive_controller/cmd_vel`, exactly as before.

The motor surrogate is **not** an effort/torque-based load-dependent model.
It approximates real closed-loop wheel RPM response (including both the real
STM32's PID and motor) by filtering the desired RPM. Do not confuse its lag
with wheel contact friction or the already-present 25 Hz velocity smoother;
the full mission response includes the effects of both smoothing and lag.

The real `scrobot_bringup/robot.launch.py` does not load mobility calibration,
does not start the simulation motor node, and retains the same hardware
velocity commands and STM32 PID.

**Validation and reproducibility:** Publishing copies settings to the
simulation; it does *not* certify them against real data. The original
floor-loaded WL/WR 90 RPM step recordings alone do not validate the robot's
ground friction, traction or skid. Keep the lab's motor-vs-real RMSE and
independent Gazebo-vs-wheel odometry results for each candidate and calibrate
against real robot motion when those measurements are available.

**Standalone test caveat:** The published *physics* defaults apply when using
`scrobot_simulation/simulation.launch.py` on its own. The published *motor*
surrogate is activated by the normal
`scrobot_bringup/sim.launch.py`; standalone debug launches that bypass that
bringup continue to use direct wheel velocity unless separately wired.

### Recovering the previous published calibration

Each successful changed publication saves the exact prior YAML in
`evaluation_results/mobility_lab/published_backups/`. To roll back, copy the
desired backup over `src/scrobot_simulation/config/mobility_calibration.yaml`,
rebuild/sync the installed config and restart Gazebo. The lab's local
`settings.json` is unaffected by this rollback.

## Troubleshooting

- If `/joint_states` never becomes ready, check
  `~/scrobot_ws/evaluation_results/mobility_lab/gazebo.log` for xacro,
  ros2_control and controller spawner errors. Source the built workspace.
- If ground-truth body speed reads "Missing", inspect ROS bridge
  `/evaluation/ground_truth_odom`. It is published by the existing Gazebo
  bridge with **best-effort sensor QoS**; the lab subscribes accordingly.
- Never compare wheel RPM alone and claim floor friction was identified.
- Don't increase wheel RPM beyond 95. The code clamps both target wheel
  commands to the SCROBOT real-hardware-safe nominal 95 RPM.
- Keep separate runs' physical settings in the saved JSON metadata.

## Offline tests

```bash
python3 -m unittest discover -s ~/scrobot_ws/tools/mobility_lab/tests -p 'test_*.py' -v
```

The repository's `SCROBOT Mobility Lab` GitHub Action also checks Python
syntax, Xacro XML parsing and the pure wheel-response mathematics. **These
tests do not replace a real Gazebo runtime trial.**
