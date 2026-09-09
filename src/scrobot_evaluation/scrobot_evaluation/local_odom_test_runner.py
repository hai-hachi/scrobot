#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class LocalOdomTestRunner(Node):
    def __init__(self):
        super().__init__('local_odom_test_runner')

        self.declare_parameter('test_type', 'suite')
        self.declare_parameter('cmd_topic', '/cmd_vel_manual')
        self.declare_parameter('state_topic', '/evaluation/local_odom_test_state')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 20.0)

        self.declare_parameter('start_delay', 3.0)
        self.declare_parameter('static_duration', 10.0)
        self.declare_parameter('stop_duration', 2.0)
        self.declare_parameter('final_settle_duration', 3.0)

        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('straight_distance', 2.0)

        self.declare_parameter('angular_speed', 0.50)
        self.declare_parameter('rotation_angle_deg', 360.0)

        self.declare_parameter('arc_linear_speed', 0.25)
        self.declare_parameter('arc_angular_speed', 0.25)
        self.declare_parameter('arc_angle_deg', 180.0)

        self.test_type = str(self.get_parameter('test_type').value).strip().lower()
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.start_delay = max(0.0, float(self.get_parameter('start_delay').value))
        self.static_duration = max(0.0, float(self.get_parameter('static_duration').value))
        self.stop_duration = max(0.0, float(self.get_parameter('stop_duration').value))
        self.final_settle_duration = max(
            0.0,
            float(self.get_parameter('final_settle_duration').value),
        )

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.straight_distance = abs(
            float(self.get_parameter('straight_distance').value)
        )

        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.rotation_angle = math.radians(
            abs(float(self.get_parameter('rotation_angle_deg').value))
        )

        self.arc_linear_speed = float(
            self.get_parameter('arc_linear_speed').value
        )
        self.arc_angular_speed = float(
            self.get_parameter('arc_angular_speed').value
        )
        self.arc_angle = math.radians(
            abs(float(self.get_parameter('arc_angle_deg').value))
        )

        control_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.cmd_topic,
            control_qos,
        )
        self.state_pub = self.create_publisher(
            String,
            self.state_topic,
            state_qos,
        )

        self.segments = self.build_segments()
        self.segment_index = 0
        self.segment_start_time = None
        self.done = False

        self.timer = self.create_timer(1.0 / publish_rate, self.update)

        self.get_logger().info(
            f'Local odom test runner ready: test={self.test_type}, '
            f'segments={len(self.segments)}'
        )

    @staticmethod
    def segment(name, duration, vx=0.0, wz=0.0):
        return {
            'name': name,
            'duration': max(0.0, float(duration)),
            'vx': float(vx),
            'wz': float(wz),
        }

    def straight_duration(self):
        speed = abs(self.linear_speed)
        if speed < 1e-6:
            raise ValueError('linear_speed must be non-zero for straight test')
        return self.straight_distance / speed

    def rotate_duration(self):
        speed = abs(self.angular_speed)
        if speed < 1e-6:
            raise ValueError('angular_speed must be non-zero for rotate test')
        return self.rotation_angle / speed

    def arc_duration(self):
        speed = abs(self.arc_angular_speed)
        if speed < 1e-6:
            raise ValueError('arc_angular_speed must be non-zero for arc test')
        return self.arc_angle / speed

    def stop(self, name='STOP'):
        return self.segment(name, self.stop_duration, 0.0, 0.0)

    def build_segments(self):
        settle = self.segment('START_DELAY', self.start_delay)
        final_settle = self.segment(
            'FINAL_SETTLE',
            self.final_settle_duration,
        )

        if self.test_type == 'static':
            return [
                settle,
                self.segment('STATIC', self.static_duration),
                final_settle,
            ]

        if self.test_type == 'straight':
            return [
                settle,
                self.segment(
                    'STRAIGHT_FWD',
                    self.straight_duration(),
                    vx=abs(self.linear_speed),
                ),
                final_settle,
            ]

        if self.test_type == 'rotate':
            return [
                settle,
                self.segment(
                    'ROTATE_CCW',
                    self.rotate_duration(),
                    wz=abs(self.angular_speed),
                ),
                final_settle,
            ]

        if self.test_type == 'arc':
            return [
                settle,
                self.segment(
                    'ARC_CCW',
                    self.arc_duration(),
                    vx=abs(self.arc_linear_speed),
                    wz=abs(self.arc_angular_speed),
                ),
                final_settle,
            ]

        if self.test_type != 'suite':
            raise ValueError(
                'test_type must be one of: static, straight, rotate, arc, suite'
            )

        straight_t = self.straight_duration()
        rotate_t = self.rotate_duration()
        arc_t = self.arc_duration()

        return [
            settle,
            self.segment('STATIC', self.static_duration),
            self.stop('STOP_AFTER_STATIC'),

            self.segment(
                'STRAIGHT_FWD',
                straight_t,
                vx=abs(self.linear_speed),
            ),
            self.stop('STOP_AFTER_FWD'),

            self.segment(
                'STRAIGHT_REV',
                straight_t,
                vx=-abs(self.linear_speed),
            ),
            self.stop('STOP_AFTER_REV'),

            self.segment(
                'ROTATE_CCW',
                rotate_t,
                wz=abs(self.angular_speed),
            ),
            self.stop('STOP_AFTER_CCW'),

            self.segment(
                'ROTATE_CW',
                rotate_t,
                wz=-abs(self.angular_speed),
            ),
            self.stop('STOP_AFTER_CW'),

            self.segment(
                'ARC_CCW',
                arc_t,
                vx=abs(self.arc_linear_speed),
                wz=abs(self.arc_angular_speed),
            ),
            self.stop('STOP_AFTER_ARC_CCW'),

            self.segment(
                'ARC_CW',
                arc_t,
                vx=abs(self.arc_linear_speed),
                wz=-abs(self.arc_angular_speed),
            ),
            final_settle,
        ]

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def publish_state(self, state):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def publish_command(self, vx, wz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = float(vx)
        msg.twist.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def update(self):
        if self.done:
            return

        if self.segment_index >= len(self.segments):
            self.publish_command(0.0, 0.0)
            self.publish_state('DONE')
            self.timer.cancel()
            self.done = True
            self.get_logger().info('Local odom test complete.')
            return

        segment = self.segments[self.segment_index]
        now = self.now_seconds()

        if self.segment_start_time is None:
            self.segment_start_time = now
            self.publish_state(segment['name'])
            self.get_logger().info(
                f"Segment {self.segment_index + 1}/{len(self.segments)}: "
                f"{segment['name']} for {segment['duration']:.2f}s, "
                f"vx={segment['vx']:.3f}, wz={segment['wz']:.3f}"
            )

        self.publish_command(segment['vx'], segment['wz'])

        if now - self.segment_start_time >= segment['duration']:
            self.publish_command(0.0, 0.0)
            self.segment_index += 1
            self.segment_start_time = None

    def destroy_node(self):
        # Send several zeros before shutdown to leave the command chain safe.
        for _ in range(3):
            self.publish_command(0.0, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LocalOdomTestRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
