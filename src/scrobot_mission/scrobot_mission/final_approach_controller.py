#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray


def quat_normalize(q):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    inv = 1.0 / norm
    return x * inv, y * inv, z * inv, w * inv


def quat_conjugate(q):
    x, y, z, w = q
    return -x, -y, -z, w


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q, v):
    qn = quat_normalize(q)
    vq = (v[0], v[1], v[2], 0.0)
    out = quat_multiply(quat_multiply(qn, vq), quat_conjugate(qn))
    return out[0], out[1], out[2]


def transform_point(transform, point):
    q = transform.rotation
    t = transform.translation
    rotated = quat_rotate(
        (float(q.x), float(q.y), float(q.z), float(q.w)),
        point,
    )
    return (
        rotated[0] + float(t.x),
        rotated[1] + float(t.y),
        rotated[2] + float(t.z),
    )


def detection_position(detection):
    if detection.results:
        p = detection.results[0].pose.pose.position
    else:
        p = detection.bbox.center.position
    return float(p.x), float(p.y), float(p.z)


def clamp(value, low, high):
    return max(low, min(high, value))


class FinalApproachController(Node):
    """Drive a selected shuttle from Nav2 staging into pickup_link."""

    def __init__(self):
        super().__init__('final_approach_controller')

        self.declare_parameter('tracked_topic', '/perception/tracked_shuttles')
        self.declare_parameter('request_topic', '/mission/collection_request')
        self.declare_parameter('complete_topic', '/mission/collection_complete')
        self.declare_parameter('failed_topic', '/mission/collection_failed')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_approach')
        self.declare_parameter('tracking_frame', 'map')
        self.declare_parameter('pickup_frame', 'pickup_link')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('timeout', 12.0)
        self.declare_parameter('tf_timeout', 0.05)
        self.declare_parameter('linear_kp', 0.8)
        self.declare_parameter('angular_kp', 2.0)
        self.declare_parameter('min_linear_speed', 0.04)
        self.declare_parameter('max_linear_speed', 0.18)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('heading_slowdown_angle', 0.45)
        self.declare_parameter('rotate_only_angle', 0.90)
        self.declare_parameter('hold_distance', 0.025)

        self.tracked_topic = str(self.get_parameter('tracked_topic').value)
        self.request_topic = str(self.get_parameter('request_topic').value)
        self.complete_topic = str(self.get_parameter('complete_topic').value)
        self.failed_topic = str(self.get_parameter('failed_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.tracking_frame = str(self.get_parameter('tracking_frame').value)
        self.pickup_frame = str(self.get_parameter('pickup_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.linear_kp = float(self.get_parameter('linear_kp').value)
        self.angular_kp = float(self.get_parameter('angular_kp').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.heading_slowdown_angle = float(
            self.get_parameter('heading_slowdown_angle').value
        )
        self.rotate_only_angle = float(self.get_parameter('rotate_only_angle').value)
        self.hold_distance = float(self.get_parameter('hold_distance').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tracks = {}
        self.active_id = ''
        self.start_ns = 0
        self.last_wait_log_ns = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Detection3DArray,
            self.tracked_topic,
            self._tracks_callback,
            qos,
        )
        self.create_subscription(String, self.request_topic, self._request_callback, qos)
        self.create_subscription(String, self.complete_topic, self._complete_callback, qos)

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, qos)
        self.failed_pub = self.create_publisher(String, self.failed_topic, qos)

        self.timer = self.create_timer(1.0 / self.control_rate, self._control)

        self.get_logger().info(
            'Final approach controller started: '
            f'{self.request_topic} -> {self.cmd_vel_topic}, pickup={self.pickup_frame}'
        )

    def _tracks_callback(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.tracking_frame:
            return
        self.tracks = {
            detection.id: detection
            for detection in msg.detections
            if detection.id
        }

    def _request_callback(self, msg):
        shuttle_id = msg.data.strip()
        if not shuttle_id:
            return
        if self.active_id and self.active_id != shuttle_id:
            self.get_logger().warn(
                f'Ignoring collection request {shuttle_id}; already approaching {self.active_id}.'
            )
            return

        self.active_id = shuttle_id
        self.start_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(f'Final approach started for shuttle {shuttle_id}.')

    def _complete_callback(self, msg):
        shuttle_id = msg.data.strip()
        if not self.active_id or shuttle_id != self.active_id:
            return
        self._publish_stop()
        self.get_logger().info(f'Final approach complete for shuttle {shuttle_id}.')
        self.active_id = ''
        self.start_ns = 0

    def _publish_cmd(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def _publish_stop(self):
        self._publish_cmd(0.0, 0.0)

    def _fail(self, reason):
        shuttle_id = self.active_id
        if not shuttle_id:
            return
        self._publish_stop()
        self.get_logger().error(f'Final approach failed for shuttle {shuttle_id}: {reason}')
        msg = String()
        msg.data = shuttle_id
        self.failed_pub.publish(msg)
        self.active_id = ''
        self.start_ns = 0

    def _target_in_pickup_frame(self, detection):
        target_map = detection_position(detection)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.pickup_frame,
                self.tracking_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_wait_log_ns > int(2.0e9):
                self.last_wait_log_ns = now_ns
                self.get_logger().warn(
                    f'Waiting for TF {self.pickup_frame} <- {self.tracking_frame}: {exc}'
                )
            return None
        return transform_point(transform, target_map)

    def _control(self):
        if not self.active_id:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self.start_ns and (now_ns - self.start_ns) > int(self.timeout * 1.0e9):
            self._fail(f'timeout after {self.timeout:.1f} s')
            return

        detection = self.tracks.get(self.active_id)
        if detection is None:
            self._publish_stop()
            if now_ns - self.last_wait_log_ns > int(2.0e9):
                self.last_wait_log_ns = now_ns
                self.get_logger().warn(
                    f'Waiting for persistent track {self.active_id}.'
                )
            return

        local = self._target_in_pickup_frame(detection)
        if local is None:
            self._publish_stop()
            return

        x, y, _ = local
        distance = math.hypot(x, y)
        heading = math.atan2(y, x) if distance > 1e-6 else 0.0

        if distance <= self.hold_distance:
            self._publish_stop()
            return

        angular = clamp(
            self.angular_kp * heading,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        if abs(heading) >= self.rotate_only_angle or x <= 0.0:
            linear = 0.0
        else:
            linear = clamp(
                self.linear_kp * x,
                self.min_linear_speed,
                self.max_linear_speed,
            )
            if abs(heading) >= self.heading_slowdown_angle:
                linear *= 0.35

        self._publish_cmd(linear, angular)


def main(args=None):
    rclpy.init(args=args)
    node = FinalApproachController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
