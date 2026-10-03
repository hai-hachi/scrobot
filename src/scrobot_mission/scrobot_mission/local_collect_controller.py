#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Point, PointStamped, TwistStamped
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


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def detection_position(detection):
    if detection.results:
        p = detection.results[0].pose.pose.position
    else:
        p = detection.bbox.center.position
    return float(p.x), float(p.y), float(p.z)


class LocalCollectController(Node):
    """Local shuttle collection with an SMC pre-pose and straight pickup pass.

    The first currently eligible shuttle is frozen once in odom. A fixed
    pre-collection pose is constructed with collector_link 0.50 m in front of
    that shuttle and facing it. Sliding-mode control drives the collector point
    to this pose. Once position and heading tolerances are satisfied, the robot
    switches to the deliberately simple straight collection rule:
        v = 0.30 m/s, omega = 0.
    The validated SMC law includes the reference-speed heading term and the
    original turn-first gate for large position-bearing error. When
    collector_link reaches the frozen shuttle point, the controller continues
    through it by the configured 0.10 m overrun.
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
        self.declare_parameter('target_timeout', 12.0)
        self.declare_parameter('smc_debug_period', 1.0)

        self.declare_parameter('position_tolerance', 0.08)
        self.declare_parameter('reacquire_exclusion_radius', 0.12)
        self.declare_parameter('precollect_distance', 0.50)
        self.declare_parameter('precollect_position_tolerance', 0.03)
        self.declare_parameter('precollect_yaw_tolerance_deg', 5.0)
        self.declare_parameter('straight_collect_speed', 0.30)
        self.declare_parameter('overrun_distance', 0.10)
        self.declare_parameter('overrun_speed', 0.25)

        self.declare_parameter('smc_reference_speed', 0.50)
        self.declare_parameter('smc_lambda', 2.0)
        self.declare_parameter('smc_ks', 1.60)
        self.declare_parameter('smc_eta', 0.50)
        self.declare_parameter('smc_phi', 0.08)
        self.declare_parameter('smc_krho', 0.8)
        self.declare_parameter('collector_offset_c', 0.165)
        self.declare_parameter('smc_max_angular_speed', 1.80)
        self.declare_parameter('smc_max_linear_speed', 0.50)

        # Preserve the old turn-first behavior only while the collector is
        # still far enough from the pre-pose for alpha to be geometrically
        # meaningful. Near the goal, continue normal SMC.
        self.declare_parameter('heading_stop_deg', 70.0)

        self.declare_parameter('max_linear_accel', 1.0)
        self.declare_parameter('max_angular_accel', 1.0)

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
        self.target_timeout = float(self.get_parameter('target_timeout').value)
        self.smc_debug_period = max(
            0.1, float(self.get_parameter('smc_debug_period').value)
        )

        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.reacquire_exclusion_radius = float(
            self.get_parameter('reacquire_exclusion_radius').value
        )
        self.precollect_distance = float(self.get_parameter('precollect_distance').value)
        self.precollect_position_tolerance = float(
            self.get_parameter('precollect_position_tolerance').value
        )
        self.precollect_yaw_tolerance = math.radians(
            float(self.get_parameter('precollect_yaw_tolerance_deg').value)
        )
        self.straight_collect_speed = max(
            0.0, float(self.get_parameter('straight_collect_speed').value)
        )
        self.overrun_distance = max(
            0.0, float(self.get_parameter('overrun_distance').value)
        )
        self.overrun_speed = max(
            0.0, float(self.get_parameter('overrun_speed').value)
        )

        self.smc_reference_speed = float(
            self.get_parameter('smc_reference_speed').value
        )
        self.smc_lambda = float(self.get_parameter('smc_lambda').value)
        self.smc_ks = float(self.get_parameter('smc_ks').value)
        self.smc_eta = float(self.get_parameter('smc_eta').value)
        self.smc_phi = max(1e-6, float(self.get_parameter('smc_phi').value))
        self.smc_krho = float(self.get_parameter('smc_krho').value)
        self.collector_offset_c = float(
            self.get_parameter('collector_offset_c').value
        )
        self.smc_max_angular_speed = abs(
            float(self.get_parameter('smc_max_angular_speed').value)
        )
        self.smc_max_linear_speed = abs(
            float(self.get_parameter('smc_max_linear_speed').value)
        )
        self.heading_stop = math.radians(
            float(self.get_parameter('heading_stop_deg').value)
        )

        self.max_linear_accel = float(self.get_parameter('max_linear_accel').value)
        self.max_angular_accel = float(self.get_parameter('max_angular_accel').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.callback_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()

        self.raw_points = []
        self.raw_frame = ''
        self.raw_stamp_ros_s = 0.0
        self.action_active = False
        self.last_v = 0.0
        self.last_w = 0.0

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(
            TwistStamped, self.cmd_vel_topic, reliable_qos
        )
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
        self.get_logger().info(
            'LocalCollect ready: SMC pre-pose at '
            f'{self.precollect_distance:.2f} m, straight collect '
            f'{self.straight_collect_speed:.2f} m/s, '
            f'overrun={self.overrun_distance:.2f} m.'
        )

    # ------------------------------------------------------------------
    # Perception / action plumbing
    # ------------------------------------------------------------------

    def _now_ros_s(self):
        """Return ROS time in seconds so simulation obeys /clock."""
        return self.get_clock().now().nanoseconds / 1e9

    def _detections_cb(self, msg):
        points = [detection_position(d) for d in msg.detections]
        with self.lock:
            self.raw_points = points
            self.raw_frame = msg.header.frame_id
            self.raw_stamp_ros_s = self._now_ros_s()

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

    # ------------------------------------------------------------------
    # TF / geometry
    # ------------------------------------------------------------------

    def _lookup(self, target, source):
        try:
            return self.tf_buffer.lookup_transform(
                target,
                source,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
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

        ps = PointStamped()
        ps.header.frame_id = source_frame
        ps.point = point
        out = do_transform_point(ps, tf)
        return float(out.point.x), float(out.point.y), float(out.point.z)

    def _frame_pose_in_odom(self, frame):
        tf = self._lookup(self.odom_frame, frame)
        if tf is None:
            return None

        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return (
            float(tf.transform.translation.x),
            float(tf.transform.translation.y),
            wrap_angle(yaw),
        )

    def _fresh_points_snapshot(self):
        with self.lock:
            age = max(0.0, self._now_ros_s() - self.raw_stamp_ros_s)
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
                f'Locked first-visible eligible shuttle: range={rng:.3f} m, '
                f'bearing={math.degrees(bearing):+.1f} deg.'
            )
            return odom, bearing, rng

        return None, 0.0, 0.0

    def _build_precollect_pose(self, target_odom):
        collector = self._frame_pose_in_odom(self.collector_frame)
        if collector is None:
            return None

        dx = target_odom[0] - collector[0]
        dy = target_odom[1] - collector[1]
        if math.hypot(dx, dy) < 1e-6:
            heading = collector[2]
        else:
            heading = math.atan2(dy, dx)

        return (
            target_odom[0] - self.precollect_distance * math.cos(heading),
            target_odom[1] - self.precollect_distance * math.sin(heading),
            wrap_angle(heading),
        )

    # ------------------------------------------------------------------
    # Control helpers
    # ------------------------------------------------------------------

    def _slew(self, current, desired, max_rate, dt):
        delta = clamp(desired - current, -max_rate * dt, max_rate * dt)
        return current + delta

    def _send_slewed(self, desired_v, desired_w, dt):
        self.last_v = self._slew(
            self.last_v, desired_v, self.max_linear_accel, dt
        )
        self.last_w = self._slew(
            self.last_w, desired_w, self.max_angular_accel, dt
        )
        self._publish_cmd(self.last_v, self.last_w)

    def _smc_command(self, pre_pose, collector_pose):
        px, py, ptheta = pre_pose
        x, y, theta = collector_pose

        dx = px - x
        dy = py - y
        rho = math.hypot(dx, dy)
        alpha = wrap_angle(math.atan2(dy, dx) - theta) if rho > 1e-9 else 0.0

        # Lateral target error expressed in the collector/body frame.
        e_y = -math.sin(theta) * dx + math.cos(theta) * dy
        e_theta = wrap_angle(ptheta - theta)

        s = e_theta + self.smc_lambda * e_y
        sat = clamp(s / self.smc_phi, -1.0, 1.0)

        # Accepted SMC law used by the validated simulation controller.
        # The reference heading is straight, so omega_R = 0.
        denominator = 1.0 + self.smc_lambda * self.collector_offset_c
        omega = (
            self.smc_lambda
            * self.smc_reference_speed
            * math.sin(e_theta)
            + self.smc_ks * s
            + self.smc_eta * sat
        ) / denominator
        omega = clamp(
            omega,
            -self.smc_max_angular_speed,
            self.smc_max_angular_speed,
        )

        v = self.smc_krho * rho * math.cos(alpha)

        if abs(alpha) >= self.heading_stop:
            v = 0.0

        v = clamp(v, 0.0, self.smc_max_linear_speed)

        return v, omega, rho, alpha, e_y, e_theta, s

    # ------------------------------------------------------------------
    # Collection phases
    # ------------------------------------------------------------------

    def _drive_precollect_pose(self, pre_pose, goal_handle, feedback, deadline):
        self._phase('SMC_POSE')
        feedback.phase = 'SMC_POSE'
        goal_handle.publish_feedback(feedback)

        self.get_logger().info(
            'SMC pre-pose: '
            f'x={pre_pose[0]:.3f}, y={pre_pose[1]:.3f}, '
            f'yaw={math.degrees(pre_pose[2]):+.1f} deg.'
        )

        period = 1.0 / max(self.control_rate, 1.0)
        previous = self._now_ros_s()
        next_debug = previous
        last_metrics = None
        self.last_v = 0.0
        self.last_w = 0.0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                return False, 'canceled during SMC pose'

            now = self._now_ros_s()
            if now >= deadline:
                self._stop()
                if last_metrics is None:
                    self.get_logger().error(
                        'SMC target timeout before a valid collector pose was available.'
                    )
                else:
                    (
                        rho,
                        alpha,
                        e_y,
                        e_theta,
                        s,
                        desired_v,
                        desired_w,
                    ) = last_metrics
                    turn_first = abs(alpha) >= self.heading_stop
                    self.get_logger().error(
                        'SMC target timeout: '
                        f'mode={"TURN_FIRST" if turn_first else "SMC"}, '
                        f'rho={rho:.4f} m, alpha={math.degrees(alpha):+.2f} deg, '
                        f'e_y={e_y:+.4f} m, '
                        f'e_theta={math.degrees(e_theta):+.2f} deg, '
                        f's={s:+.4f}, v_cmd={desired_v:+.3f} m/s, '
                        f'w_cmd={desired_w:+.3f} rad/s.'
                    )
                return False, 'target timeout during SMC pose'

            collector = self._frame_pose_in_odom(self.collector_frame)
            if collector is None:
                self._stop()
                time.sleep(period)
                continue

            desired_v, desired_w, rho, alpha, e_y, e_theta, s = self._smc_command(
                pre_pose, collector
            )

            last_metrics = (
                rho,
                alpha,
                e_y,
                e_theta,
                s,
                desired_v,
                desired_w,
            )

            feedback.range_m = float(rho)
            feedback.bearing_deg = float(math.degrees(alpha))
            goal_handle.publish_feedback(feedback)

            if now >= next_debug:
                turn_first = abs(alpha) >= self.heading_stop
                self.get_logger().info(
                    'SMC state: '
                    f'mode={"TURN_FIRST" if turn_first else "SMC"}, '
                    f'rho={rho:.4f} m, alpha={math.degrees(alpha):+.2f} deg, '
                    f'e_y={e_y:+.4f} m, '
                    f'e_theta={math.degrees(e_theta):+.2f} deg, '
                    f's={s:+.4f}, v_cmd={desired_v:+.3f} m/s, '
                    f'w_cmd={desired_w:+.3f} rad/s.'
                )
                next_debug = now + self.smc_debug_period

            if (
                rho <= self.precollect_position_tolerance
                and abs(e_theta) <= self.precollect_yaw_tolerance
            ):
                self._stop()
                self.get_logger().info(
                    'SMC pre-pose reached: '
                    f'rho={rho:.4f} m, e_y={e_y:+.4f} m, '
                    f'e_theta={math.degrees(e_theta):+.2f} deg, '
                    f's={s:+.4f}.'
                )
                return True, 'pre-pose reached'

            dt = max(0.0, now - previous)
            previous = now
            self._send_slewed(desired_v, desired_w, dt)
            time.sleep(period)

        self._stop()
        return False, 'ROS shutdown during SMC pose'

    def _drive_straight_to_target(self, target_odom, goal_handle, feedback, deadline):
        self._phase('STRAIGHT_COLLECT')
        feedback.phase = 'STRAIGHT_COLLECT'
        goal_handle.publish_feedback(feedback)

        self.get_logger().info(
            f'Straight collection: v={self.straight_collect_speed:.2f} m/s, omega=0.'
        )

        period = 1.0 / max(self.control_rate, 1.0)
        previous = self._now_ros_s()
        desired_speed = min(
            self.straight_collect_speed,
            self.smc_max_linear_speed,
        )

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                return False, 'canceled during straight collection'

            now = self._now_ros_s()
            if now >= deadline:
                self._stop()
                self.get_logger().error('Target timeout during straight collection.')
                return False, 'target timeout during straight collection'

            local = self._point_to_frame(
                self.collector_frame,
                self.odom_frame,
                target_odom,
            )
            if local is None:
                self._stop()
                time.sleep(period)
                continue

            x, y, _ = local
            distance = math.hypot(x, y)
            bearing = math.atan2(y, max(x, 1e-6))
            feedback.range_m = float(distance)
            feedback.bearing_deg = float(math.degrees(bearing))
            goal_handle.publish_feedback(feedback)

            if distance <= self.position_tolerance:
                self._stop()
                return True, 'collector reached target'

            dt = max(0.0, now - previous)
            previous = now
            self._send_slewed(desired_speed, 0.0, dt)
            time.sleep(period)

        self._stop()
        return False, 'ROS shutdown during straight collection'

    def _drive_overrun(self, goal_handle, feedback, deadline):
        if self.overrun_distance <= 0.0 or self.overrun_speed <= 0.0:
            return True, 'collector reached target'

        period = 1.0 / max(self.control_rate, 1.0)
        start = None
        while rclpy.ok() and start is None:
            if goal_handle.is_cancel_requested:
                self._stop()
                return False, 'canceled before overrun'
            if self._now_ros_s() >= deadline:
                self._stop()
                self.get_logger().error(
                    'Target timeout while waiting for collector TF before overrun.'
                )
                return False, 'target timeout before overrun'
            start = self._frame_pose_in_odom(self.collector_frame)
            if start is None:
                self._stop()
                time.sleep(period)

        self._phase('OVERRUN')
        feedback.phase = 'OVERRUN'
        goal_handle.publish_feedback(feedback)

        self.get_logger().info(
            f'Collector reached target; overrunning {self.overrun_distance:.2f} m.'
        )

        previous = self._now_ros_s()
        desired_speed = min(self.overrun_speed, self.smc_max_linear_speed)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                return False, 'canceled during overrun'

            now = self._now_ros_s()
            if now >= deadline:
                self._stop()
                self.get_logger().error('Target timeout during overrun.')
                return False, 'target timeout during overrun'

            current = self._frame_pose_in_odom(self.collector_frame)
            if current is None:
                self._stop()
                time.sleep(period)
                continue

            traveled = math.hypot(current[0] - start[0], current[1] - start[1])
            feedback.range_m = max(0.0, self.overrun_distance - traveled)
            feedback.bearing_deg = 0.0
            goal_handle.publish_feedback(feedback)

            if traveled >= self.overrun_distance:
                self._stop()
                return True, 'collector reached target + overrun'

            dt = max(0.0, now - previous)
            previous = now
            self._send_slewed(desired_speed, 0.0, dt)
            time.sleep(period)

        self._stop()
        return False, 'ROS shutdown during overrun'

    def _drive_target(self, target_odom, goal_handle, feedback):
        deadline = self._now_ros_s() + self.target_timeout

        pre_pose = self._build_precollect_pose(target_odom)
        if pre_pose is None:
            self._stop()
            return False, 'pre-pose TF unavailable'

        completed, reason = self._drive_precollect_pose(
            pre_pose, goal_handle, feedback, deadline
        )
        if not completed or reason.startswith('target timeout'):
            return completed, reason

        completed, reason = self._drive_straight_to_target(
            target_odom, goal_handle, feedback, deadline
        )
        if not completed or reason.startswith('target timeout'):
            return completed, reason

        return self._drive_overrun(goal_handle, feedback, deadline)

    # ------------------------------------------------------------------
    # Action execution
    # ------------------------------------------------------------------

    def _execute(self, goal_handle):
        result = LocalCollect.Result()
        feedback = LocalCollect.Feedback()

        with self.lock:
            self.action_active = True

        attempted_odom = []
        attempted_count = 0
        spree_started = self._now_ros_s()
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

                if self._now_ros_s() - spree_started >= overall_timeout:
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
                feedback.phase = 'SMC_POSE'
                goal_handle.publish_feedback(feedback)

                completed, reason = self._drive_target(
                    target,
                    goal_handle,
                    feedback,
                )

                if not completed:
                    self._stop()
                    result.success = False
                    result.targets_attempted = attempted_count
                    result.message = reason

                    if reason.startswith('canceled'):
                        self._phase('CANCELED')
                        goal_handle.canceled()
                    else:
                        self._phase('FAILED')
                        goal_handle.abort()
                        self.get_logger().error(
                            f'Local target {attempted_count} failed: {reason}.'
                        )
                    return result

                self.get_logger().info(
                    f'Local target {attempted_count} finished ({reason}); '
                    'checking current FOV for the first next eligible shuttle.'
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
