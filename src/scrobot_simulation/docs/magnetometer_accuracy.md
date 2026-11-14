# 5883L-family magnetometer accuracy and rate

## Identify the actual chip first

The previous Gazebo sensor was labeled generically `5883L`; that does not prove
that the physical module is a QMC5883L rather than an HMC5883L or a compatible
replacement. Read the part marking and verify the device address/ID with the
actual I2C driver before selecting a hardware profile.

- QMC5883L (QST): continuous output rates 10/50/100/200 Hz. For 100 Hz
  select continuous mode and ODR=100 Hz; check OSR, ±2 G vs ±8 G range and
  data-ready freshness.
- HMC5883L: maximum normal continuous-mode output data rate 75 Hz.
  100 Hz ROS publication requires explicit timestamp-aware resampling, not
  repeated measurements misrepresented as 100 Hz independent samples.

The Gazebo `/imu/mag` virtual measurement is configured for 100 Hz as a
**QMC5883L-class assumption**, not proof of physical sensor capability.

## Make magnetometer measurements correspond to the robot

1. Mount the physical module at its actual measured position and orientation;
   replace the currently co-located, axis-aligned `mag_link` placeholder TF.
2. Log a full 360-degree sweep and multiple pitch/roll orientations with the
   motors OFF; fit hard-iron offset b (3-vector) and soft-iron correction A
   (3x3 matrix), then use `m_calibrated = A (m_raw - b)`.
3. Repeat with drive motors/brushes/conveyor ON at representative load and
   duty. Motor wiring, chassis steel, magnets and current loops can distort
   the magnetic field; measure offsets versus motor operating state.
4. Compare calibrated magnetic field magnitude and heading with a surveyed
   reference; distinguish random noise from state-dependent magnetic bias.
5. Estimate per-axis noise standard deviation and temporal correlation from
   stationary logs, plus field saturation and response delay. Add those
   empirical values to the virtual sensor only after verifying Gazebo/SDF
   noise units and ROS MagneticField Tesla conversion.
6. Set a measured covariance on downstream MagneticField messages; prefer
   rejecting/deweighting magnetic fusion during disturbances rather than
   silently assuming heading is trustworthy.
7. Record ROS source stamps, receive time and true data-ready sample count.
   A 100 Hz publisher does not by itself guarantee 100 distinct measurements.

## Frequency plan

- Gazebo virtual IMU: 100 Hz already.
- Gazebo virtual magnetometer: 100 Hz after this update.
- Gazebo differential-drive odometry: 100 Hz controller publication.
- Gazebo ground-truth odometry: 100 Hz evaluation-only.
- EKF: 100 Hz configured.
- Odometry Lab: 100 Hz initial sampling/estimator/publisher settings.
- Real D435i gyro/accel and real magnetometer: device-specific acquisition
  frequencies; do not claim native 100 Hz until hardware logs confirm.
- Publication rates can be slower in wall time if Gazebo RTF < 1.

## Validate in ROS 2

```bash
ros2 topic hz /camera/camera/imu
ros2 topic hz /imu/mag
ros2 topic hz /diff_drive_controller/odom
ros2 topic hz /odometry/filtered
ros2 topic hz /evaluation/ground_truth_odom
```

Review both ROS-stamp deltas and actual receive intervals; compare to `/clock`.
The magnetometer is optional in physical bringup, and hardware magnetometer
fusion remains disabled until the mounted sensor is identified and calibrated.

## References

- QST QMC5883L Datasheet Rev. B:
  https://www.qstcorp.com/upload/pdf/202512/13-52-04%20QMC5883L%20Datasheet%20Rev.%20B.pdf
- Honeywell HMC5883L datasheet: continuous-rate maximum 75 Hz.
