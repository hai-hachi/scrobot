#!/usr/bin/env python3

import subprocess

import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.node import Node


class ShuttleImpulseTest(Node):
    """Apply a short, repeatable Gazebo wrench to one candidate shuttle."""

    def __init__(self):
        super().__init__('shuttle_impulse_test')

        self.declare_parameter('world', 'shuttle_physics_test')
        self.declare_parameter('model_name', 'shuttle_physics_000')
        self.declare_parameter('link_name', 'shuttle_link')
        self.declare_parameter('delay', 2.0)
        self.declare_parameter('duration', 0.05)
        self.declare_parameter('force_x', 0.0)
        self.declare_parameter('force_y', 0.03)
        self.declare_parameter('force_z', 0.0)
        self.declare_parameter('torque_x', 0.0)
        self.declare_parameter('torque_y', 0.0)
        self.declare_parameter('torque_z', 0.0)

        self.world = str(self.get_parameter('world').value)
        self.model_name = str(self.get_parameter('model_name').value)
        self.link_name = str(self.get_parameter('link_name').value)
        self.entity_name = f'{self.model_name}::{self.link_name}'
        self.delay = max(0.0, float(self.get_parameter('delay').value))
        self.duration = max(0.001, float(self.get_parameter('duration').value))

        self.force = (
            float(self.get_parameter('force_x').value),
            float(self.get_parameter('force_y').value),
            float(self.get_parameter('force_z').value),
        )
        self.torque = (
            float(self.get_parameter('torque_x').value),
            float(self.get_parameter('torque_y').value),
            float(self.get_parameter('torque_z').value),
        )

        self.start_time = None
        self.force_start_time = None
        self.applied = False
        self.cleared = False

        self.create_subscription(Clock, '/clock', self._clock_cb, 10)

        impulse = tuple(component * self.duration for component in self.force)
        self.get_logger().info(
            'Impulse test armed: '
            f'link={self.entity_name}, force={self.force} N, '
            f'duration={self.duration:.3f} s, '
            f'nominal impulse=({impulse[0]:.6f}, '
            f'{impulse[1]:.6f}, {impulse[2]:.6f}) N*s.'
        )

    @staticmethod
    def _time_seconds(msg):
        return float(msg.clock.sec) + float(msg.clock.nanosec) * 1.0e-9

    def _publish_wrench(self):
        topic = f'/world/{self.world}/wrench/persistent'
        payload = (
            f'entity: {{name: "{self.entity_name}", type: LINK}}, '
            'wrench: {'
            f'force: {{x: {self.force[0]}, y: {self.force[1]}, z: {self.force[2]}}}, '
            f'torque: {{x: {self.torque[0]}, y: {self.torque[1]}, z: {self.torque[2]}}}'
            '}'
        )
        result = subprocess.run(
            [
                'gz', 'topic',
                '-t', topic,
                '-m', 'gz.msgs.EntityWrench',
                '-p', payload,
            ],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        if result.returncode != 0:
            self.get_logger().error(
                f'Failed to apply wrench: {result.stdout.strip()}'
            )
        else:
            self.get_logger().info('Persistent test wrench applied.')

    def _clear_wrench(self):
        topic = f'/world/{self.world}/wrench/clear'
        payload = f'name: "{self.entity_name}", type: LINK'
        result = subprocess.run(
            [
                'gz', 'topic',
                '-t', topic,
                '-m', 'gz.msgs.Entity',
                '-p', payload,
            ],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        if result.returncode != 0:
            self.get_logger().error(
                f'Failed to clear wrench: {result.stdout.strip()}'
            )
        else:
            self.get_logger().info('Test wrench cleared.')

    def _clock_cb(self, msg):
        now = self._time_seconds(msg)
        if self.start_time is None:
            self.start_time = now
            return

        elapsed = now - self.start_time

        if not self.applied and elapsed >= self.delay:
            self._publish_wrench()
            self.applied = True
            self.force_start_time = now
            return

        if (
            self.applied
            and not self.cleared
            and self.force_start_time is not None
            and now - self.force_start_time >= self.duration
        ):
            self._clear_wrench()
            self.cleared = True
            self.get_logger().info('Impulse sequence complete.')


def main(args=None):
    rclpy.init(args=args)
    node = ShuttleImpulseTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.applied and not node.cleared:
            node._clear_wrench()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
