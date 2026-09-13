#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseArray, TwistStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scrobot_interfaces.action import CollectShuttle
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
    out = quat_multiply(
        quat_multiply(qn, (v[0], v[1], v[2], 0.0)),
        quat_conjugate(qn),
    )
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
    """Action server that drives one persistent shuttle track into pickup_link."""

    def __init__(self):
        super().__init__('final_approach_controller')

        self.declare_parameter('tracked_topic', '/perception/tracked_shuttles')
        self.declare_parameter('action_name', '/collect_shuttle')
        self.declare_parameter('simulation_collection_topic', '/evaluation/shuttle_collected')
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
        self.declare_parameter('collection_event_match_distance', 0.50)

        self.tracked_topic = str(self.get_parameter('tracked_topic').value)
        self.action_name = str(self.get_parameter('action_name').value)
        self.simulation_collection_topic = str(
            self.get_parameter('simulation_collection_topic').value
        )
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
        self.collection_event_match_distance = float(
            self.get_parameter('collection_event_match_distance').value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.callback_group = ReentrantCallbackGroup()

        self.lock = threading.Lock()
        self.tracks = {}
        self.active_id = ''
        self.collected_id = ''

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Detection3DArray,
            self.tracked_topic,
            self._tracks_callback,
            qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            PoseArray,
            self.simulation_collection_topic,
            self._simulation_collection_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, qos)

        self.action_server = ActionServer(
            self,
            CollectShuttle,
            self.action_name,
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            f'CollectShuttle action server ready on {self.action_name}; '
            f'cmd={self.cmd_vel_topic}, pickup={self.pickup_frame}.'
        )

    def _tracks_callback(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.tracking_frame:
            return
        with self.lock:
            self.tracks = {
                detection.id: detection
                for detection in msg.detections
                if detection.id
            }

    def _goal_callback(self, goal_request):
        shuttle_id = goal_request.shuttle_id.strip()
        if not shuttle_id:
            return GoalResponse.REJECT
        with self.lock:
            if self.active_id:
                self.get_logger().warn(
                    f'Rejecting collection of {shuttle_id}; already collecting '
                    f'{self.active_id}.'
                )
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _simulation_collection_callback(self, msg):
        if not msg.poses:
            return
        with self.lock:
            active_id = self.active_id
            detection = self.tracks.get(active_id) if active_id else None
        if not active_id or detection is None:
            return

        tx, ty, tz = detection_position(detection)
        best = min(
            math.sqrt(
                (float(p.position.x) - tx) ** 2
                + (float(p.position.y) - ty) ** 2
                + (float(p.position.z) - tz) ** 2
            )
            for p in msg.poses
        )
        if best <= self.collection_event_match_distance:
            with self.lock:
                self.collected_id = active_id
            self.get_logger().info(
                f'Physical collection matched shuttle {active_id} '
                f'(event error={best:.3f} m).'
            )

    def _publish_cmd(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def _publish_stop(self):
        self._publish_cmd(0.0, 0.0)

    def _target_in_pickup_frame(self, detection):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.pickup_frame,
                self.tracking_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException:
            return None
        return transform_point(transform, detection_position(detection))

    def _execute(self, goal_handle):
        shuttle_id = goal_handle.request.shuttle_id.strip()
        result = CollectShuttle.Result()
        feedback = CollectShuttle.Feedback()

        with self.lock:
            self.active_id = shuttle_id
            self.collected_id = ''

        self.get_logger().info(f'Collecting shuttle {shuttle_id}.')
        start = time.monotonic()
        period = 1.0 / max(self.control_rate, 1.0)

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._publish_stop()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Collection canceled.'
                    return result

                if time.monotonic() - start > self.timeout:
                    self._publish_stop()
                    goal_handle.abort()
                    result.success = False
                    result.message = f'Collection timed out after {self.timeout:.1f} s.'
                    self.get_logger().error(result.message)
                    return result

                with self.lock:
                    collected = self.collected_id == shuttle_id
                    detection = self.tracks.get(shuttle_id)

                if collected:
                    self._publish_stop()
                    goal_handle.succeed()
                    result.success = True
                    result.message = f'Shuttle {shuttle_id} collected.'
                    self.get_logger().info(result.message)
                    return result

                if detection is None:
                    self._publish_stop()
                    feedback.distance_to_target = float('nan')
                    goal_handle.publish_feedback(feedback)
                    time.sleep(period)
                    continue

                local = self._target_in_pickup_frame(detection)
                if local is None:
                    self._publish_stop()
                    time.sleep(period)
                    continue

                x, y, _ = local
                distance = math.hypot(x, y)
                feedback.distance_to_target = float(distance)
                goal_handle.publish_feedback(feedback)

                if distance <= self.hold_distance:
                    self._publish_stop()
                    time.sleep(period)
                    continue

                heading = math.atan2(y, x) if distance > 1e-6 else 0.0
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
                time.sleep(period)
        finally:
            self._publish_stop()
            with self.lock:
                self.active_id = ''
                self.collected_id = ''

        result.success = False
        result.message = 'Collection stopped because ROS shut down.'
        return result

    def destroy_node(self):
        self._publish_stop()
        self.action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FinalApproachController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
