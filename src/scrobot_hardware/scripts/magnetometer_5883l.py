#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus


class Magnetometer5883L(Node):
    HMC_ADDR = 0x1E
    QMC_ADDR = 0x0D

    def __init__(self):
        super().__init__('magnetometer_5883l')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('chip', 'auto')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('hard_iron_offset_t', [0.0, 0.0, 0.0])
        self.declare_parameter(
            'soft_iron_matrix',
            [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0])
        self.declare_parameter('axis_map', [0, 1, 2])
        self.declare_parameter('axis_sign', [1, 1, 1])
        self.declare_parameter(
            'covariance',
            [1.0e-10, 0.0, 0.0,
             0.0, 1.0e-10, 0.0,
             0.0, 0.0, 1.0e-10])

        self.bus_number = int(self.get_parameter('i2c_bus').value)
        self.requested_chip = str(self.get_parameter('chip').value).lower()
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.offset = [float(v) for v in self.get_parameter('hard_iron_offset_t').value]
        self.matrix = [float(v) for v in self.get_parameter('soft_iron_matrix').value]
        self.axis_map = [int(v) for v in self.get_parameter('axis_map').value]
        self.axis_sign = [int(v) for v in self.get_parameter('axis_sign').value]
        self.covariance = [float(v) for v in self.get_parameter('covariance').value]

        if len(self.offset) != 3 or len(self.matrix) != 9:
            raise ValueError('hard_iron_offset_t must have 3 values and soft_iron_matrix 9')
        if sorted(self.axis_map) != [0, 1, 2] or len(self.axis_sign) != 3:
            raise ValueError('axis_map must be a permutation of [0,1,2]; axis_sign needs 3 values')

        self.publisher = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.bus = None
        self.chip = None
        self.address = None
        self.last_connect_attempt = 0.0
        self.last_io_error = 0.0

        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info(
            f'5883L magnetometer node: I2C bus {self.bus_number}, chip={self.requested_chip}')

    def _probe(self, address):
        try:
            self.bus.read_byte_data(address, 0x00)
            return True
        except OSError:
            return False

    def _connect(self):
        now = time.monotonic()
        if now - self.last_connect_attempt < 2.0:
            return False
        self.last_connect_attempt = now

        try:
            if self.bus is None:
                self.bus = SMBus(self.bus_number)

            chip = self.requested_chip
            if chip == 'auto':
                if self._probe(self.HMC_ADDR):
                    chip = 'hmc5883l'
                elif self._probe(self.QMC_ADDR):
                    chip = 'qmc5883l'
                else:
                    raise OSError('no device responded at 0x1E or 0x0D')

            if chip in ('hmc5883l', 'hmc'):
                self.address = self.HMC_ADDR
                # 8-sample average, 75 Hz, normal measurement.
                self.bus.write_byte_data(self.address, 0x00, 0x78)
                # Gain = 1.3 Ga, 1090 LSB/Gauss.
                self.bus.write_byte_data(self.address, 0x01, 0x20)
                self.bus.write_byte_data(self.address, 0x02, 0x00)
                self.chip = 'hmc5883l'
            elif chip in ('qmc5883l', 'qmc'):
                self.address = self.QMC_ADDR
                # Soft reset, set/reset period, then OSR512 / 2G / 200 Hz / continuous.
                self.bus.write_byte_data(self.address, 0x0A, 0x80)
                time.sleep(0.01)
                self.bus.write_byte_data(self.address, 0x0B, 0x01)
                self.bus.write_byte_data(self.address, 0x09, 0x0D)
                self.chip = 'qmc5883l'
            else:
                raise ValueError(f'unsupported chip setting: {self.requested_chip}')

            self.get_logger().info(
                f'Connected to {self.chip} at 0x{self.address:02X} on /dev/i2c-{self.bus_number}')
            return True
        except (OSError, ValueError) as exc:
            self.chip = None
            self.get_logger().error(f'5883L connect failed: {exc}')
            return False

    @staticmethod
    def _s16_le(lo, hi):
        value = (hi << 8) | lo
        return value - 65536 if value & 0x8000 else value

    @staticmethod
    def _s16_be(hi, lo):
        value = (hi << 8) | lo
        return value - 65536 if value & 0x8000 else value

    def _read_raw_tesla(self):
        if self.chip == 'hmc5883l':
            data = self.bus.read_i2c_block_data(self.address, 0x03, 6)
            x = self._s16_be(data[0], data[1])
            z = self._s16_be(data[2], data[3])
            y = self._s16_be(data[4], data[5])
            scale = 1.0e-4 / 1090.0
            return [x * scale, y * scale, z * scale]

        if self.chip == 'qmc5883l':
            data = self.bus.read_i2c_block_data(self.address, 0x00, 6)
            x = self._s16_le(data[0], data[1])
            y = self._s16_le(data[2], data[3])
            z = self._s16_le(data[4], data[5])
            # 2 Gauss range: approximately 12000 LSB/Gauss.
            scale = 1.0e-4 / 12000.0
            return [x * scale, y * scale, z * scale]

        raise OSError('magnetometer is not connected')

    def _calibrate(self, raw):
        mapped = [
            float(self.axis_sign[i]) * raw[self.axis_map[i]]
            for i in range(3)
        ]
        v = [mapped[i] - self.offset[i] for i in range(3)]
        m = self.matrix
        return [
            m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
            m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
            m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
        ]

    def _timer_cb(self):
        if self.chip is None and not self._connect():
            return

        try:
            field = self._calibrate(self._read_raw_tesla())
            if not all(math.isfinite(v) for v in field):
                raise ValueError('non-finite magnetic field')

            msg = MagneticField()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.magnetic_field.x = field[0]
            msg.magnetic_field.y = field[1]
            msg.magnetic_field.z = field[2]
            msg.magnetic_field_covariance = self.covariance
            self.publisher.publish(msg)
        except (OSError, ValueError) as exc:
            now = time.monotonic()
            if now - self.last_io_error > 2.0:
                self.get_logger().error(f'5883L read failed: {exc}; reconnecting')
                self.last_io_error = now
            self.chip = None

    def destroy_node(self):
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Magnetometer5883L()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
