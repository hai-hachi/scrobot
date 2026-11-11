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

This only changes **simulation** defaults and generated **simulation-only**
controller/EKF configuration copies. Source
`src/scrobot_control/config/controllers.yaml` and
`src/scrobot_localization/config/ekf.yaml` remain at their hardware-safe
existing values (50 Hz publication). The real STM32 firmware and IMU drivers
remain unchanged.

The rate distribution is:
```text
src/scrobot_simulation/config/odometry_calibration.yaml
  -> normal simulation.launch.py
      -> generated simulator controller YAML (wheel odom rate, NOT PID Hz)
      -> Gazebo sensor Xacro (IMU / mag / ground-truth Hz)
  -> normal sim.launch.py
      -> generated simulator EKF YAML (output frequency)

Odometry Lab local settings.json
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

The lab aligns the initial poses once and does *not* continuously correct
estimated trajectories. AprilTag corrections are disabled. Error plots and
metrics should be interpreted in the **local frame**, not as a guaranteed
absolute north/map heading estimate. Magnetometer heading requires field
convention, magnetic declination and hardware calibration.

## Save, restart, and Publish to Simulation

Click **Save lab settings** to persist GUI values at:
`~/scrobot_ws/evaluation_results/odometry_lab/settings.json`.

Click **Restart Gazebo with rates** to regenerate isolated test controller/EKF
configs, respawn sensors, and apply requested IMU/mag/truth rates. Running
test profiles does not edit source ROS configuration.

Click **Publish to Simulation** after reviewing actual rate and error plots.
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

The physical `robot.launch.py` is not modified; it still uses its native
hardware controller parameters and existing EKF defaults. No real motor PID,
STM32 transport, or physical IMU rate is changed by this lab.

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
