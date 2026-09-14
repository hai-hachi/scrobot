#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scrobot_interfaces.action import CollectShuttle
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray


def clamp(value, low, high):
    return max(low, min(high, value))


def detection_position(detection):
    if detection.results:
        p = detection.results[0].pose.pose.position
    else:
        p = detection.bbox.center.position
    return float(p.x), float(p.y), float(p.z)


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conjugate(q):
    return -q[0], -q[1], -q[2], q[3]


def quat_rotate(q, v):
    out = quat_multiply(quat_multiply(q, (v[0], v[1], v[2], 0.0)), quat_conjugate(q))
    return out[0], out[1], out[2]


def transform_point(transform, point):
    q = transform.rotation
    rotated = quat_rotate(
        (float(q.x), float(q.y), float(q.z), float(q.w)),
        point,
    )
    return (
        rotated[0] + float(transform.translation.x),
        rotated[1] + float(transform.translation.y),
        rotated[2] + float(transform.translation.z),
    )


class VisualInterceptController(Node):
    """Continuously steer the collector onto one currently visible shuttle."""

    def __init__(self):
        super().__init__('visual_intercept_controller')

        self.declare_parameter('visible_topic', '/perception/visible_tracked_shuttles')
        self.declare_parameter('action_name', '/visual_intercept')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_approach')
        self.declare_parameter('tracking_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('control_rate', 25.0)
        self.declare_parameter('tf_timeout', 0.05)

        self.declare_parameter('acquisition_timeout', 2.0)
        self.declare_parameter('lost_grace', 0.35)
        self.declare_parameter('intercept_timeout', 8.0)
        self.declare_parameter('pickup_offset_x', 0.165)
        self.declare_parameter('target_capture_x', 0.23)
        self.declare_parameter('target_capture_y', 0.11)
        self.declare_parameter('blind_overrun_time', 0.55)

        self.declare_parameter('linear_kp', 0.85)
        self.declare_parameter('angular_kp', 2.4)
        self.declare_parameter('min_linear_speed', 0.08)
        self.declare_parameter('max_linear_speed', 0.28)
        self.declare_parameter('max_angular_speed', 0.70)
        self.declare_parameter('heading_slowdown_angle', 0.30)

        self.visible_topic = str(self.get_parameter('visible_topic').value)
        self.action_name = str(self.get_parameter('action_name').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.tracking_frame = str(self.get_parameter('tracking_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.acquisition_timeout = float(self.get_parameter('acquisition_timeout').value)
        self.lost_grace = float(self.get_parameter('lost_grace').value)
        self.intercept_timeout = float(self.get_parameter('intercept_timeout').value)
        self.pickup_offset_x = float(self.get_parameter('pickup_offset_x').value)
        self.target_capture_x = float(self.get_parameter('target_capture_x').value)
        self.target_capture_y = float(self.get_parameter('target_capture_y').value)
        self.blind_overrun_time = float(self.get_parameter('blind_overrun_time').value)
        self.linear_kp = float(self.get_parameter('linear_kp').value)
        self.angular_kp = float(self.get_parameter('angular_kp').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.heading_slowdown_angle = float(self.get_parameter('heading_slowdown_angle').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.callback_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.visible = {}
        self.active = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Detection3DArray,
            self.visible_topic,
            self._visible_callback,
            qos,
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
            'Visual intercept ready: continuously tracks visible shuttle position '
            'and steers the collector through it.'
        )

    def _visible_callback(self, msg):
        with self.lock:
            self.visible = {d.id: d for d in msg.detections if d.id}

    def _goal_callback(self, goal_request):
        ids = [x.strip() for x in goal_request.shuttle_ids if x.strip()]
        if len(ids) != 1:
            return GoalResponse.REJECT
        with self.lock:
            if self.active:
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _publish_cmd(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def _stop(self):
        self._publish_cmd(0.0, 0.0)

    def _target_local(self, track_id):
        with self.lock:
            detection = self.visible.get(track_id)
        if detection is None:
            return None

        point = detection_position(detection)
        source = detection.header.frame_id or self.tracking_frame
        if source == self.base_frame:
            return point

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                source,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException:
            return None
        return transform_point(tf, point)

    def _blind_overrun(self, goal_handle, speed):
        end = time.monotonic() + self.blind_overrun_time
        period = 1.0 / max(self.control_rate, 1.0)
        while rclpy.ok() and time.monotonic() < end:
            if goal_handle.is_cancel_requested:
                self._stop()
                return False
            self._publish_cmd(speed, 0.0)
            time.sleep(period)
        self._stop()
        return True

    def _execute(self, goal_handle):
        track_id = next(x.strip() for x in goal_handle.request.shuttle_ids if x.strip())
        result = CollectShuttle.Result()
        feedback = CollectShuttle.Feedback()

        with self.lock:
            self.active = True

        try:
            acquire_deadline = time.monotonic() + self.acquisition_timeout
            target = None
            while rclpy.ok() and time.monotonic() < acquire_deadline:
                if goal_handle.is_cancel_requested:
                    self._stop()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Visual intercept canceled while acquiring target.'
                    return result
                target = self._target_local(track_id)
                if target is not None:
                    break
                self._stop()
                time.sleep(1.0 / max(self.control_rate, 1.0))

            if target is None:
                self._stop()
                goal_handle.succeed()
                result.success = False
                result.message = f'Shuttle {track_id} not visible at visual handoff.'
                return result

            self.get_logger().info(f'Visual intercept acquired shuttle {track_id}.')
            started = time.monotonic()
            last_seen = started
            was_capture_close = False
            period = 1.0 / max(self.control_rate, 1.0)

            while rclpy.ok() and time.monotonic() - started < self.intercept_timeout:
                if goal_handle.is_cancel_requested:
                    self._stop()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Visual intercept canceled.'
                    return result

                target = self._target_local(track_id)
                if target is None:
                    if was_capture_close or time.monotonic() - last_seen >= self.lost_grace:
                        if was_capture_close:
                            completed = self._blind_overrun(goal_handle, self.min_linear_speed)
                            if not completed:
                                goal_handle.canceled()
                                result.success = False
                                result.message = 'Canceled during blind collection overrun.'
                                return result
                            goal_handle.succeed()
                            result.success = True
                            result.message = f'Visual intercept completed for shuttle {track_id}.'
                            result.collected_ids = [track_id]
                            return result
                        self._stop()
                        goal_handle.succeed()
                        result.success = False
                        result.message = f'Lost shuttle {track_id} before capture corridor.'
                        return result
                    self._stop()
                    time.sleep(period)
                    continue

                last_seen = time.monotonic()
                x, y, _ = target
                distance = math.hypot(x, y)
                feedback.distance_to_target = float(distance)
                goal_handle.publish_feedback(feedback)

                if x <= self.target_capture_x and abs(y) <= self.target_capture_y:
                    was_capture_close = True

                # Aim the robot centerline at the live target. As the shuttle
                # approaches the collector, reduce speed but never stall.
                heading = math.atan2(y, max(x, 1e-4))
                angular = clamp(
                    self.angular_kp * heading,
                    -self.max_angular_speed,
                    self.max_angular_speed,
                )
                forward_error = max(0.0, x - self.pickup_offset_x)
                linear = clamp(
                    self.linear_kp * forward_error,
                    self.min_linear_speed,
                    self.max_linear_speed,
                )
                if abs(heading) > self.heading_slowdown_angle:
                    linear *= 0.45

                self._publish_cmd(linear, angular)
                time.sleep(period)

            self._stop()
            goal_handle.succeed()
            result.success = False
            result.message = f'Visual intercept timed out for shuttle {track_id}.'
            return result
        finally:
            self._stop()
            with self.lock:
                self.active = False

    def destroy_node(self):
        self._stop()
        self.action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisualInterceptController()
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
