#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Point, TwistStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scrobot_interfaces.action import LocalCollect
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
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


class LocalCollectController(Node):
    """Odom-only visual collection spree.

    The first current raw detection is locked immediately. Its measured point is
    transformed once into odom and never updated from perception while that
    target is active. Motion continuously drives collector_link toward the
    frozen odom point with coupled linear/angular control. When the attempt is
    complete, the first currently visible eligible detection is locked and the
    process repeats. No scan or target ranking is performed.
    """

    def __init__(self):
        super().__init__('local_collect_controller')

        self.declare_parameter('raw_detection_topic', '/perception/shuttle_detections_3d')
        self.declare_parameter('action_name', '/local_collect')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_approach')
        self.declare_parameter('phase_topic', '/mission/local_collect_phase')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('collector_frame', 'collector_link')
        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('tf_timeout', 0.05)
        self.declare_parameter('detection_max_age', 0.30)

        self.declare_parameter('position_tolerance', 0.07)
        self.declare_parameter('target_timeout', 12.0)
        self.declare_parameter('reacquire_exclusion_radius', 0.12)

        self.declare_parameter('linear_kp', 0.85)
        self.declare_parameter('angular_kp', 2.4)
        self.declare_parameter('max_linear_speed', 0.45)
        self.declare_parameter('max_angular_speed', 1.20)
        self.declare_parameter('max_linear_accel', 0.65)
        self.declare_parameter('max_angular_accel', 2.4)
        self.declare_parameter('heading_stop_deg', 70.0)
        self.declare_parameter('slow_radius', 0.60)

        self.raw_detection_topic = str(self.get_parameter('raw_detection_topic').value)
        self.action_name = str(self.get_parameter('action_name').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.phase_topic = str(self.get_parameter('phase_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.collector_frame = str(self.get_parameter('collector_frame').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.detection_max_age = float(self.get_parameter('detection_max_age').value)
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.target_timeout = float(self.get_parameter('target_timeout').value)
        self.reacquire_exclusion_radius = float(
            self.get_parameter('reacquire_exclusion_radius').value
        )
        self.linear_kp = float(self.get_parameter('linear_kp').value)
        self.angular_kp = float(self.get_parameter('angular_kp').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.max_linear_accel = float(self.get_parameter('max_linear_accel').value)
        self.max_angular_accel = float(self.get_parameter('max_angular_accel').value)
        self.heading_stop = math.radians(float(self.get_parameter('heading_stop_deg').value))
        self.slow_radius = float(self.get_parameter('slow_radius').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.callback_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.raw_points = []
        self.raw_frame = ''
        self.raw_stamp_monotonic = 0.0
        self.action_active = False
        self.last_v = 0.0
        self.last_w = 0.0

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, reliable_qos)
        self.phase_pub = self.create_publisher(String, self.phase_topic, reliable_qos)
        self.create_subscription(
            Detection3DArray,
            self.raw_detection_topic,
            self._detections_cb,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            LocalCollect,
            self.action_name,
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self.callback_group,
        )
        self._phase('IDLE')
        self.get_logger().info('LocalCollect ready: first-visible lock, odom-only pursuit, no scan.')

    def _detections_cb(self, msg):
        points = [detection_position(d) for d in msg.detections]
        with self.lock:
            self.raw_points = points
            self.raw_frame = msg.header.frame_id
            self.raw_stamp_monotonic = time.monotonic()

    def _goal_cb(self, _request):
        with self.lock:
            return GoalResponse.REJECT if self.action_active else GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _phase(self, text):
        msg = String()
        msg.data = text
        self.phase_pub.publish(msg)

    def _publish_cmd(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def _stop(self):
        self.last_v = 0.0
        self.last_w = 0.0
        self._publish_cmd(0.0, 0.0)

    def _lookup(self, target, source):
        try:
            return self.tf_buffer.lookup_transform(
                target, source, Time(), timeout=Duration(seconds=self.tf_timeout)
            )
        except TransformException:
            return None

    def _point_to_frame(self, target_frame, source_frame, xyz):
        if not source_frame:
            return None
        point = Point()
        point.x, point.y, point.z = xyz
        if source_frame == target_frame:
            return point.x, point.y, point.z
        tf = self._lookup(target_frame, source_frame)
        if tf is None:
            return None
        stamped = type('PointStampedLike', (), {})()
        # tf2_geometry_msgs requires a real PointStamped-like ROS message.
        from geometry_msgs.msg import PointStamped
        ps = PointStamped()
        ps.header.frame_id = source_frame
        ps.point = point
        out = do_transform_point(ps, tf)
        return float(out.point.x), float(out.point.y), float(out.point.z)

    def _fresh_points_snapshot(self):
        with self.lock:
            age = time.monotonic() - self.raw_stamp_monotonic
            if age > self.detection_max_age:
                return '', []
            return self.raw_frame, list(self.raw_points)

    def _lock_first_target(self, attempted_odom):
        source_frame, points = self._fresh_points_snapshot()
        for point in points:
            odom = self._point_to_frame(self.odom_frame, source_frame, point)
            if odom is None:
                continue
            duplicate = any(
                math.hypot(odom[0] - old[0], odom[1] - old[1])
                <= self.reacquire_exclusion_radius
                for old in attempted_odom
            )
            if duplicate:
                continue
            local = self._point_to_frame(self.base_frame, self.odom_frame, odom)
            if local is None:
                continue
            bearing = math.atan2(local[1], max(local[0], 1e-6))
            rng = math.hypot(local[0], local[1])
            self.get_logger().info(
                f'Locked first-visible shuttle: range={rng:.3f} m, '
                f'bearing={math.degrees(bearing):+.1f} deg.'
            )
            return odom, bearing, rng
        return None, 0.0, 0.0

    def _slew(self, current, desired, max_rate, dt):
        delta = clamp(desired - current, -max_rate * dt, max_rate * dt)
        return current + delta

    def _drive_target(self, target_odom, goal_handle, feedback):
        period = 1.0 / max(self.control_rate, 1.0)
        started = time.monotonic()
        previous = started
        self.last_v = 0.0
        self.last_w = 0.0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                return False, 'canceled'
            now = time.monotonic()
            if now - started >= self.target_timeout:
                self._stop()
                return True, 'target timeout'

            local = self._point_to_frame(self.collector_frame, self.odom_frame, target_odom)
            if local is None:
                self._stop()
                time.sleep(period)
                continue

            x, y, _ = local
            distance = math.hypot(x, y)
            bearing = math.atan2(y, max(x, 1e-6))
            feedback.bearing_deg = float(math.degrees(bearing))
            feedback.range_m = float(distance)
            goal_handle.publish_feedback(feedback)

            if distance <= self.position_tolerance:
                self._stop()
                return True, 'collector reached target'

            # Smooth coupled pursuit. Linear motion fades out for large heading
            # errors; angular speed naturally eases out with bearing, and both
            # channels are acceleration-limited to avoid command steps.
            heading_gate = max(0.0, math.cos(bearing)) ** 2
            if abs(bearing) >= self.heading_stop or x <= 0.0:
                heading_gate = 0.0
            distance_gate = clamp(distance / max(self.slow_radius, 1e-6), 0.15, 1.0)
            desired_v = clamp(self.linear_kp * distance, 0.0, self.max_linear_speed)
            desired_v *= heading_gate * distance_gate
            desired_w = clamp(
                self.angular_kp * bearing,
                -self.max_angular_speed,
                self.max_angular_speed,
            )

            dt = max(1e-3, now - previous)
            previous = now
            self.last_v = self._slew(self.last_v, desired_v, self.max_linear_accel, dt)
            self.last_w = self._slew(self.last_w, desired_w, self.max_angular_accel, dt)
            self._publish_cmd(self.last_v, self.last_w)
            time.sleep(period)

        self._stop()
        return False, 'ROS shutdown'

    def _execute(self, goal_handle):
        result = LocalCollect.Result()
        feedback = LocalCollect.Feedback()
        with self.lock:
            self.action_active = True
        attempted_odom = []
        attempted_count = 0
        spree_started = time.monotonic()
        overall_timeout = float(goal_handle.request.timeout_sec)
        if overall_timeout <= 0.0:
            overall_timeout = 120.0

        try:
            self._phase('SELECT')
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._stop()
                    self._phase('CANCELED')
                    goal_handle.canceled()
                    result.success = False
                    result.targets_attempted = attempted_count
                    result.message = 'Local collection canceled.'
                    return result
                if time.monotonic() - spree_started >= overall_timeout:
                    self._stop()
                    self._phase('DONE')
                    goal_handle.succeed()
                    result.success = True
                    result.targets_attempted = attempted_count
                    result.message = 'Local collection ended at overall timeout.'
                    return result

                target, bearing, rng = self._lock_first_target(attempted_odom)
                if target is None:
                    self._stop()
                    self._phase('DONE')
                    goal_handle.succeed()
                    result.success = True
                    result.targets_attempted = attempted_count
                    result.message = 'No eligible shuttle currently visible.'
                    return result

                attempted_count += 1
                attempted_odom.append(target)
                feedback.target_index = attempted_count
                feedback.bearing_deg = float(math.degrees(bearing))
                feedback.range_m = float(rng)
                feedback.phase = 'TRACK'
                goal_handle.publish_feedback(feedback)
                self._phase('TRACK')

                completed, reason = self._drive_target(target, goal_handle, feedback)
                if not completed:
                    self._phase('CANCELED')
                    goal_handle.canceled()
                    result.success = False
                    result.targets_attempted = attempted_count
                    result.message = reason
                    return result

                self.get_logger().info(
                    f'Local target {attempted_count} finished ({reason}); '
                    'checking current FOV for the first next shuttle.'
                )
                self._phase('SELECT')
        finally:
            self._stop()
            with self.lock:
                self.action_active = False

    def destroy_node(self):
        self._stop()
        self.action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LocalCollectController()
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
