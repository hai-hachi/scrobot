# SCROBOT Odometry Lab — wheel, D435i, magnetometer and EKF

The Odometry Lab is a stand-alone browser laboratory in the same
`~/scrobot_ws/tools` workspace directory as Lighting Lab and Mobility Lab.

Its purpose is to distinguish:
1. **Transport / publication correctness**: actual message Hz, jitter,
   freshness, frame IDs, timestamps, dropouts.
2. **Sensor interpretation / fusion correctness**: wheel encoder integration;
   D435i gyro, acceleration, frame conversion; optional simulated magnetic
   field -> Madgwick attitude; wheel x/y plus IMU heading -> EKF.
3. **Localization accuracy**: wheel odometry, IMU heading, and EKF position/yaw
   compared to the *independent* Gazebo body ground-truth stream.

The experiment never launches Nav2, velocity smoother, SMC, AprilTags or the
mission manager, avoiding implicit global corrections and competing command
producers. It runs the real drivetrain differential-drive controller and the
floor-loaded Gazebo robot. A simple, isolated test generator writes directly to
`/diff_drive_controller/cmd_vel` at 100 Hz.

## Launch

Prerequisites: sourced ROS 2 Jazzy, Gazebo Harmonic, `robot_localization`,
`imu_filter_madgwick`, `imu_transformer`, `gz_ros2_control`, `ros_gz_bridge`,
and built SCROBOT packages.

```bash
cd ~/scrobot_ws/src
git switch real-comparison
git pull --ff-only
cd ~/scrobot_ws
colcon build --symlink-install --packages-up-to scrobot_simulation scrobot_bringup
source /opt/ros/jazzy/setup.bash
source ~/scrobot_ws/install/setup.bash
python3 ~/scrobot_ws/tools/odometry_lab/server.py
```

Open **http://127.0.0.1:8767**. Gazebo is started automatically.
The lab defaults to isolated `ROS_DOMAIN_ID=78` plus a unique
`GZ_PARTITION`, unlike Mobility Lab (77) or default hardware domain 0.
Do not launch normal production driving in the lab's ROS domain.

## 100 Hz sensor and fusion configuration

The physical STM32 PID is **100 Hz** and the ROS 2 controller manager is
**100 Hz**. The default lab/published-simulation configuration now targets:

| Component | Topic / source | Before | Sim/lab default |
|---|---|---:|---:|
| Controller manager | wheel joint state update | 100 Hz | 100 Hz, unchanged |
| Differential-drive odom | `/diff_drive_controller/odom` | 50 Hz | **100 Hz** |
| Simulated D435i IMU | `/camera/camera/imu` | 100 Hz | **100 Hz** |
| Transformed raw IMU | `/imu/data_raw` | input-driven | follows source IMU |
| Simulated 5883L magnetometer | `/imu/mag` | 50 Hz | **100 Hz** |
| Madgwick orientation | `/imu/data` | input-driven | follows arriving IMU/mag messages |
| EKF fused odometry | `/odometry/filtered` | 50 Hz | **100 Hz** |
| Gazebo independent truth | `/evaluation/ground_truth_odom` | 50 Hz | **100 Hz** |

The GUI can set the wheel odometry publish rate **10–100 Hz**, while IMU,
magnetometer, EKF and Gazebo truth can be requested at **10–200 Hz**. A
controller output above its physical 100 Hz update cadence would duplicate
information rather than give independent measurements, so the lab deliberately
does not permit >100 Hz wheel odom or change the controller update rate.
At 200 Hz EKF, the filter predicts more often but does not create additional
sensor measurements. The GUI table reports the **measured** Hz and timestamp
jitter, which can be lower because of Gazebo scheduling, RTF, QoS or CPU load.

The lab changes only **simulation sensor acquisition frequencies** and
generates simulation-only controller/EKF overrides. Additionally, the shared
ROS controller YAML now requests **100 Hz wheel odometry publishing**, and
the shared EKF YAML now requests **100 Hz fused-odom output**, including on
the real robot. Those two changes do NOT increase the STM32's existing
100 Hz PID loop rate, nor do they increase a physical IMU's native sampling
rate. Actual delivered rate must be verified using topic-header timing.

The rate distribution is:
```text
src/scrobot_simulation/config/odometry_calibration.yaml
  -> normal simulation.launch.py
      -> generated simulator controller YAML (wheel odom rate, NOT PID Hz)
      -> Gazebo sensor Xacro (IMU / mag / ground-truth Hz)
  -> normal sim.launch.py
      -> generated simulator EKF YAML (output frequency)

Odometry Lab saved odometry_lab.yaml (legacy JSON fallback)
  -> isolated odometry_lab.launch.py, generated test-only controller/EKF YAML
```

## What is actually fused?

```text
left/right wheel encoder positions
  -> diff_drive_controller
  -> /diff_drive_controller/odom
        x and y pose, non-holonomic vy=0 constraint
            -> robot_localization EKF
D435i raw 6-axis IMU -> imu_transformer -> /imu/data_raw
      \                                  /
       \-> Madgwick orientation <--------- /imu/mag (optional)
                   -> /imu/data  (yaw and yaw rate)
                   -> EKF        (relative IMU yaw, angular velocity z)
                                    |
                                    v
                       /odometry/filtered (odom -> base_footprint)

independent Gazebo /evaluation/ground_truth_odom
             -> EVALUATION ONLY, never fused
```

Current `ekf.yaml` *intentionally does not fuse wheel yaw / angular
velocity*, despite those being available in wheel odom. Thus the lab compares
wheel-only odometry (the unfiltered controller output), the Madgwick IMU yaw,
and the EKF fused pose; it does not pretend that simply switching off the IMU
produces an equivalent, correctly tuned wheel-only EKF. Optional magnetometer
can be enabled/disabled at launch, but changing it requires restarting the lab.

The **magnetometer does not connect directly to the EKF**: it influences
Madgwick orientation first, and EKF uses this orientation. The physical robot
currently lacks an active restored 5883L ROS driver; magnetometer testing in
this milestone is **Gazebo-only**. Magnetic field strength in the GUI is raw
microtesla magnitude, not calibrated geographic heading.

## Nominal 0.50 m/s wheel-speed tests

The SCROBOT drive wheel nominal rim-speed limit is **0.50 m/s per wheel**,
not 0.50 m/s added to the yaw contribution. For wheel radius **0.05 m**,
0.50 m/s corresponds to **95.49 RPM**. The real STM32 interface currently
caps commands at **95 RPM** (about **0.497 m/s**) for headroom; the optional
published Mobility Lab sim motor filter has the same 95 RPM ceiling.

The Odometry Lab defaults to **0.50 m/s** linear command for new settings
files; a previously saved `settings.json` retains its user-chosen value.
To exercise full nominal wheel speed, select **Straight** (or **Reverse**)
and set the linear slider to **0.50 m/s**, or click **Set 0.50 m/s straight**.

Per-profile envelope validation now checks the **actual simultaneous
left/right wheel commands** generated by each motion:
- **Straight / reverse**: both wheels run at ±linear speed. The unused
  angular slider does not restrict the test.
- **Spin**: opposite wheel rim speeds are ±`(L/2) * yaw_rate`. The
  existing differential-drive yaw cap is **2.30 rad/s**, corresponding
  to about **0.483 m/s** per wheel during a spin.
- **Square**: straight legs and in-place corners are sequential, so
  the maximum speed is the larger of linear speed and turning rim speed,
  not the sum.
- **Figure-eight**: translation and yaw happen simultaneously; the
  maximum instantaneous wheel rim speed is
  `0.6 * linear_m_s + 0.21 * angular_rad_s`. Profiles exceeding 0.50
  m/s at either wheel are explicitly rejected with a reason.
- **Stationary**: zero speed regardless of stored motion sliders.

The dashboard shows calculated peak commanded wheel speed and the
actual left/right wheel RPM and rim speeds from `/joint_states`.

### Why the Run test button can appear greyed out

The **motion profile options are all enabled**. A grey **Run test** button
means the simulator is not running, the wheel controller hasn't published
fresh states, an experiment is already running, the selected profile would
exceed the wheel-speed envelope, or sensor-frequency/magnetometer changes
require a Gazebo restart. The dashboard now lists the blocking reason.
Changing **only motion settings** (profile, speed or rest time) no longer
disables Run: those values are automatically saved when starting the test.
If sensor rates are changed, click **Save lab settings** followed by
**Restart Gazebo with rates** first.

## Experiments

Select:
- Stationary 30 s: verify zero motion, IMU stability, gyro and magnetometer
  health, and yaw / EKF drift (longer holds can be added later).
- Straight / reverse 4 s: wheel linear scale, odometry consistency.
- Spin 90°, 180°, or 360°: yaw tracking and gyro/IMU fusion.
- Square (four 0.5 m legs): positional consistency and turn slip.
- Figure-eight: translation/rotation simultaneously.

The open-loop test generator is intentionally *not* a precise path controller:
commanded distances and degrees are targets, while true realized motion is
determined from Gazebo body odometry. This prevents the robot from using its
own estimated pose to define the reference trajectory.

Each run uses a short stationary prefix then logs all available streams using
their **simulation header timestamps**. For every topic, the GUI reports actual
header-stamp rate, jitter, age since last received message and frame ID.
After completion it displays:

- wheel pose RMSE vs independent truth, with initial pose aligned
- EKF pose RMSE vs independent truth
- wheel yaw, Madgwick IMU yaw, EKF yaw RMSE vs independent truth
- final position and yaw errors

Additional recorded estimates include wheel forward speed and yaw-rate RMSE,
transformed-IMU gyro-z angular-rate RMSE, mean gyro-z and standard deviation,
and the relative rotation of the magnetic field vector versus true heading.
The magnetic metric is **relative only** and must not be presented as an
absolute compass heading. IMU gyro mean is a bias estimate only in the
stationary test; during motion it includes genuine rotation.

The lab aligns the initial poses once and does *not* continuously correct
estimated trajectories. AprilTag corrections are disabled. Error plots and
metrics should be interpreted in the **local frame**, not as a guaranteed
absolute north/map heading estimate. Magnetometer heading requires field
convention, magnetic declination and hardware calibration.

## Save, restart, and Publish to Simulation

Click **Save Odometry Lab YAML** to persist the authoritative GUI values at:
`~/scrobot_ws/src/scrobot_simulation/config/odometry_lab.yaml`.
The older `evaluation_results/odometry_lab/settings.json` remains
an internal working draft / migration fallback, never the production
publication source.

Click **Restart Gazebo with rates** to regenerate isolated test controller/EKF
configs, respawn sensors, and apply requested IMU/mag/truth rates. Running
test profiles does not edit source ROS configuration.

Click **Publish to Simulation** after reviewing actual rate and error plots.
This action reads the *saved Odometry Lab YAML* only, not an unsaved
slider setting or the working JSON.
This explicitly updates only:

```text
~/scrobot_ws/src/src/scrobot_simulation/config/odometry_calibration.yaml
```

(the repository lives in `~/scrobot_ws/src`; the `scrobot_simulation` ROS
package is in that repository's nested `src` directory). A byte-exact
timestamped backup is saved to:
`~/scrobot_ws/evaluation_results/odometry_lab/published_backups/`.

After publishing, rebuild if needed and restart normal Gazebo:
```bash
cd ~/scrobot_ws
colcon build --symlink-install --packages-up-to scrobot_bringup
source install/setup.bash
ros2 launch scrobot_bringup sim.launch.py
```

The physical `robot.launch.py` is not modified. It still uses its native
hardware controller and sensor drivers, although the shared
`controllers.yaml` wheel-odometry publish rate and `ekf.yaml` output rate
have both been raised to 100 Hz. No real motor PID, STM32 transport, or
physical IMU acquisition rate is changed by this lab.

## Limitations / future real-world comparison

Gazebo's current D435i and simulated magnetometer have no explicit
noise/bias profiles in the authored sensor Xacro. Very low simulator error
**does not establish real-world sensor accuracy**. The simulator ground-truth
topic is a separate physics-based 2D pose publisher with zero configured
Gaussian measurement noise, and represents a reference **inside Gazebo only**.

To validate actual wheel encoder, IMU and magnetometer accuracy, collect
synchronized raw real-sensor logs *plus independent motion ground truth*
(e.g. surveyed displacement or visual tracking). Compare at matched sampling
rates and frame conventions. Real gyro bias, magnetic interference, slip,
extrinsic misalignment and timestamp transport cannot be calibrated from a
perfectly simulated sensor alone.

The current lab does not overwrite the production wheel geometry, EKF sensor
selection, Madgwick gain, magnetic declination, or robot reference frames.
These are subsequent calibration stages after a trusted baseline.

## Offline tests

```bash
python3 -m unittest discover -s ~/scrobot_ws/tools/odometry_lab/tests -p 'test_*.py' -v
```

CI checks Python syntax, simulated sensor XML, web JavaScript, rate-profile
generation and odometry error calculations. Real Gazebo rendering/TF
conformance must still be checked on your workstation.
