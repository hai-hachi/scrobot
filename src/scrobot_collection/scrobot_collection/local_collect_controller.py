#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, TwistStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
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

    The first currently eligible shuttle is frozen once in odom. For a target
    farther than the nominal stand-off, a pre-collection pose is constructed
    with base_link 1.10 m from the shuttle in the ground plane and facing it.
    If the shuttle is already closer than that stand-off, the pre-pose is
    clamped to the current base position so the controller aligns in place
    instead of requesting an unreachable pose behind the robot. Sliding-mode
    control acts directly on
    the base_link planar pose with controlled-point offset c = 0. Once position
    and heading tolerances are satisfied, the robot
    switches to the deliberately simple straight collection rule:
        v = 0.30 m/s, omega = 0.
    When collector_link reaches the frozen shuttle point, the controller
    continues through it by the configured 0.10 m overrun.
    """

    def __init__(self):
        super().__init__('local_collect_controller')

        self.declare_parameter('raw_detection_topic', '/perception/collectable_shuttle_detections_3d')
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

        self.declare_parameter('position_tolerance', 0.08)
        self.declare_parameter('reacquire_exclusion_radius', 0.12)
        self.declare_parameter('base_standoff_distance', 1.10)
        self.declare_parameter('precollect_position_tolerance', 0.03)
        self.declare_parameter('precollect_yaw_tolerance_deg', 5.0)
        self.declare_parameter('precollect_stable_time', 0.25)
        self.declare_parameter('straight_collect_speed', 0.30)
        self.declare_parameter('overrun_distance', 0.10)
        self.declare_parameter('overrun_speed', 0.25)

        self.declare_parameter('smc_reference_speed', 0.50)
        self.declare_parameter('smc_lambda', 2.0)
        self.declare_parameter('smc_ks', 2.0)
        self.declare_parameter('smc_eta', 0.8)
        self.declare_parameter('smc_phi', 0.05)
        self.declare_parameter('smc_krho', 0.8)
        self.declare_parameter('smc_max_angular_speed', 2.0)
        self.declare_parameter('smc_max_linear_speed', 0.50)
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

        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.reacquire_exclusion_radius = float(
            self.get_parameter('reacquire_exclusion_radius').value
        )
        self.base_standoff_distance = float(
            self.get_parameter('base_standoff_distance').value
        )
        self.precollect_position_tolerance = float(
            self.get_parameter('precollect_position_tolerance').value
        )
        self.precollect_yaw_tolerance = math.radians(
            float(self.get_parameter('precollect_yaw_tolerance_deg').value)
        )
        self.precollect_stable_time = max(
            0.0, float(self.get_parameter('precollect_stable_time').value)
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
        self.raw_stamp_monotonic = 0.0
        self.action_active = False
        self.last_v = 0.0
        self.last_w = 0.0

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        debug_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.cmd_pub = self.create_publisher(
            TwistStamped, self.cmd_vel_topic, reliable_qos
        )
        self.phase_pub = self.create_publisher(String, self.phase_topic, reliable_qos)
        self.debug_target_pub = self.create_publisher(
            PointStamped,
            '/debug/smc_shuttle/target',
            debug_qos,
        )
        self.debug_pre_pose_pub = self.create_publisher(
            PoseStamped,
            '/debug/smc_shuttle/pre_pose',
            debug_qos,
        )
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
            'LocalCollect ready: SMC base_link pose at '
            f'{self.base_standoff_distance:.2f} m from shuttle, straight collect '
            f'{self.straight_collect_speed:.2f} m/s, '
            f'overrun={self.overrun_distance:.2f} m.'
        )

    # ------------------------------------------------------------------
    # Perception / action plumbing
    # ------------------------------------------------------------------

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

    def _publish_debug_geometry(self, target_odom, pre_pose):
        stamp = self.get_clock().now().to_msg()

        target = PointStamped()
        target.header.frame_id = self.odom_frame
        target.header.stamp = stamp
        target.point.x = float(target_odom[0])
        target.point.y = float(target_odom[1])
        target.point.z = float(target_odom[2])
        self.debug_target_pub.publish(target)

        goal = PoseStamped()
        goal.header.frame_id = self.odom_frame
        goal.header.stamp = stamp
        goal.pose.position.x = float(pre_pose[0])
        goal.pose.position.y = float(pre_pose[1])
        goal.pose.position.z = 0.03
        half = 0.5 * pre_pose[2]
        goal.pose.orientation.z = math.sin(half)
        goal.pose.orientation.w = math.cos(half)
        self.debug_pre_pose_pub.publish(goal)

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
                f'Locked first-visible eligible shuttle: range={rng:.3f} m, '
                f'bearing={math.degrees(bearing):+.1f} deg.'
            )
            return odom, bearing, rng

        return None, 0.0, 0.0

    def _build_precollect_pose(self, target_odom):
        base = self._frame_pose_in_odom(self.base_frame)
        if base is None:
            return None

        dx = target_odom[0] - base[0]
        dy = target_odom[1] - base[1]
        if math.hypot(dx, dy) < 1e-6:
            heading = base[2]
        else:
            heading = math.atan2(dy, dx)

        current_range = math.hypot(dx, dy)

        # Do not place the staging pose behind the robot when a shuttle is
        # already closer than the nominal stand-off. That geometry creates a
        # deadlock for the forward-only SMC law: alpha ~= pi stops v while
        # e_y ~= 0 and e_theta ~= 0 also make omega ~= 0.
        #
        # For close targets, clamp the stand-off to the current range. The
        # resulting goal position is the current base position, so SMC only
        # aligns the heading toward the frozen shuttle before the deliberate
        # straight pickup pass.
        effective_standoff = min(
            self.base_standoff_distance,
            current_range,
        )

        goal_x = (
            target_odom[0]
            - effective_standoff * math.cos(heading)
        )
        goal_y = (
            target_odom[1]
            - effective_standoff * math.sin(heading)
        )

        return (
            goal_x,
            goal_y,
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

    def _smc_command(self, pre_pose, base_pose):
        px, py, ptheta = pre_pose
        x, y, theta = base_pose

        dx = px - x
        dy = py - y
        rho = math.hypot(dx, dy)
        alpha = wrap_angle(math.atan2(dy, dx) - theta) if rho > 1e-9 else 0.0

        # Lateral target error expressed in the base_link frame.
        e_y = -math.sin(theta) * dx + math.cos(theta) * dy
        e_theta = wrap_angle(ptheta - theta)

        s = e_theta + self.smc_lambda * e_y
        sat = clamp(s / self.smc_phi, -1.0, 1.0)

        # c = 0 and omega_R = 0.
        omega = (
            self.smc_lambda
            * self.smc_reference_speed
            * math.sin(e_theta)
            + self.smc_ks * s
            + self.smc_eta * sat
        )
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

    def _drive_precollect_pose(
        self,
        pre_pose,
        target_odom,
        goal_handle,
        feedback,
        deadline,
    ):
        self._phase('SMC_POSE')
        feedback.phase = 'SMC_POSE'
        goal_handle.publish_feedback(feedback)

        effective_standoff = math.hypot(
            target_odom[0] - pre_pose[0],
            target_odom[1] - pre_pose[1],
        )
        self.get_logger().info(
            'SMC base-link pre-pose: '
            f'goal_xy=({pre_pose[0]:.3f},{pre_pose[1]:.3f}), '
            f'yaw={math.degrees(pre_pose[2]):+.1f} deg, '
            f'standoff={effective_standoff:.3f} m '
            f'(nominal={self.base_standoff_distance:.2f} m).'
        )

        period = 1.0 / max(self.control_rate, 1.0)
        previous = time.monotonic()
        stable_since = None
        self.last_v = 0.0
        self.last_w = 0.0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                return False, 'canceled during SMC pose'

            now = time.monotonic()
            if now >= deadline:
                self._stop()
                return True, 'target timeout during SMC pose'

            base = self._frame_pose_in_odom(self.base_frame)
            if base is None:
                self._stop()
                time.sleep(period)
                continue

            desired_v, desired_w, rho, alpha, e_y, e_theta, s = self._smc_command(
                pre_pose, base
            )

            feedback.range_m = float(rho)
            feedback.bearing_deg = float(math.degrees(alpha))
            goal_handle.publish_feedback(feedback)

            pose_in_tolerance = (
                rho <= self.precollect_position_tolerance
                and abs(e_theta) <= self.precollect_yaw_tolerance
            )

            if pose_in_tolerance:
                # Do not trigger STRAIGHT_COLLECT just because an oscillating
                # heading crosses the yaw tolerance for one control sample.
                # Stop and require the full pre-pose to remain valid
                # continuously for precollect_stable_time.
                self._stop()
                if stable_since is None:
                    stable_since = now

                if now - stable_since >= self.precollect_stable_time:
                    base_range = math.hypot(
                        target_odom[0] - base[0],
                        target_odom[1] - base[1],
                    )
                    self.get_logger().info(
                        'SMC pre-pose reached and stable: '
                        f'rho={rho:.4f} m, e_y={e_y:+.4f} m, '
                        f'e_theta={math.degrees(e_theta):+.2f} deg, '
                        f's={s:+.4f}, '
                        f'stable={self.precollect_stable_time:.2f} s, '
                        f'base_range={base_range:.3f} m.'
                    )
                    return True, 'pre-pose reached and stable'

                time.sleep(period)
                continue

            stable_since = None

            dt = max(1e-3, now - previous)
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
        previous = time.monotonic()
        desired_speed = min(
            self.straight_collect_speed,
            self.smc_max_linear_speed,
        )

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                return False, 'canceled during straight collection'

            now = time.monotonic()
            if now >= deadline:
                self._stop()
                return True, 'target timeout during straight collection'

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

            dt = max(1e-3, now - previous)
            previous = now
            self._send_slewed(desired_speed, 0.0, dt)
            time.sleep(period)

        self._stop()
        return False, 'ROS shutdown during straight collection'

    def _drive_overrun(self, goal_handle, feedback, deadline):
        if self.overrun_distance <= 0.0 or self.overrun_speed <= 0.0:
            return True, 'collector reached target'

        start = self._frame_pose_in_odom(self.collector_frame)
        if start is None:
            self._stop()
            return True, 'collector reached target; overrun TF unavailable'

        self._phase('OVERRUN')
        feedback.phase = 'OVERRUN'
        goal_handle.publish_feedback(feedback)

        self.get_logger().info(
            f'Collector reached target; overrunning {self.overrun_distance:.2f} m.'
        )

        period = 1.0 / max(self.control_rate, 1.0)
        previous = time.monotonic()
        desired_speed = min(self.overrun_speed, self.smc_max_linear_speed)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                return False, 'canceled during overrun'

            now = time.monotonic()
            if now >= deadline:
                self._stop()
                return True, 'target timeout during overrun'

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

            dt = max(1e-3, now - previous)
            previous = now
            self._send_slewed(desired_speed, 0.0, dt)
            time.sleep(period)

        self._stop()
        return False, 'ROS shutdown during overrun'

    def _drive_target(self, target_odom, goal_handle, feedback):
        deadline = time.monotonic() + self.target_timeout

        pre_pose = self._build_precollect_pose(target_odom)
        if pre_pose is None:
            self._stop()
            return True, 'pre-pose TF unavailable'

        # Latched debug geometry lets RViz start after the target lock without
        # losing the exact frozen shuttle point or the desired SMC pre-pose.
        self._publish_debug_geometry(target_odom, pre_pose)

        completed, reason = self._drive_precollect_pose(
            pre_pose,
            target_odom,
            goal_handle,
            feedback,
            deadline,
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
                feedback.phase = 'SMC_POSE'
                goal_handle.publish_feedback(feedback)

                completed, reason = self._drive_target(
                    target,
                    goal_handle,
                    feedback,
                )

                if not completed:
                    self._phase('CANCELED')
                    goal_handle.canceled()
                    result.success = False
                    result.targets_attempted = attempted_count
                    result.message = reason
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
