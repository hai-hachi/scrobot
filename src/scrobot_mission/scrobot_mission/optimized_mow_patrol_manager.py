#!/usr/bin/env python3

import copy
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scrobot_interfaces.action import Relocalize
from scrobot_mission.patrol_manager import (
    MissionState,
    PatrolManager,
    detection_position,
)
from std_msgs.msg import String
from tf2_ros import TransformException


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value, low, high):
    return max(low, min(high, value))


class OptimizedMowPatrolManager(PatrolManager):
    """Fast single-target mission: observe -> farthest visible -> Nav2 mow."""

    def __init__(self):
        super().__init__()

        # ------------------------------------------------------------------
        # Optimized mowing parameters
        # ------------------------------------------------------------------
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('scan_cmd_vel_topic', '/cmd_vel_scan')
        self.declare_parameter('mow_target_max_range', 3.0)
        self.declare_parameter('pickup_offset_x', 0.165)
        self.declare_parameter('mow_overrun', 0.03)

        # Normal empty-court scanning still uses Nav2 Spin. As soon as the
        # first shuttle appears, that action is canceled and this much slower
        # direct angular command takes over until the first shuttle approaches
        # the opposite image edge.
        self.declare_parameter('slow_observe_angular_velocity', 0.18)
        self.declare_parameter('slow_observe_edge_bearing', 0.28)
        self.declare_parameter('slow_observe_timeout', 4.0)
        self.declare_parameter('scan_first_lost_grace', 0.30)
        self.declare_parameter('scan_control_rate', 30.0)

        # Runtime relocalization remains distance-triggered, but this branch
        # bypasses /approach_tag. DUE is opportunistic only after a mow pass.
        # REQUIRED may Nav2 directly to the closest known initial watch pose.
        self.declare_parameter('relocalize_due_distance', 15.0)
        self.declare_parameter('relocalize_required_distance', 30.0)
        self.declare_parameter('quick_relocalize_radius', 0.50)
        self.declare_parameter('quick_relocalize_sample_count', 8)
        self.declare_parameter('quick_relocalize_timeout', 6.0)
        self.declare_parameter('tag_mount_frame_prefix', 'tag_mount_')
        self.declare_parameter('quick_align_kp', 1.5)
        self.declare_parameter('quick_align_min_speed', 0.10)
        self.declare_parameter('quick_align_max_speed', 0.35)
        self.declare_parameter('quick_align_tolerance_deg', 3.0)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.scan_cmd_vel_topic = str(
            self.get_parameter('scan_cmd_vel_topic').value
        )
        self.mow_target_max_range = float(
            self.get_parameter('mow_target_max_range').value
        )
        self.pickup_offset_x = float(self.get_parameter('pickup_offset_x').value)
        self.mow_overrun = float(self.get_parameter('mow_overrun').value)

        self.slow_observe_angular_velocity = float(
            self.get_parameter('slow_observe_angular_velocity').value
        )
        self.slow_observe_edge_bearing = float(
            self.get_parameter('slow_observe_edge_bearing').value
        )
        self.slow_observe_timeout = float(
            self.get_parameter('slow_observe_timeout').value
        )
        self.scan_first_lost_grace = float(
            self.get_parameter('scan_first_lost_grace').value
        )
        scan_control_rate = float(self.get_parameter('scan_control_rate').value)

        self.relocalize_due_distance = float(
            self.get_parameter('relocalize_due_distance').value
        )
        self.relocalize_required_distance = float(
            self.get_parameter('relocalize_required_distance').value
        )
        self.quick_relocalize_radius = float(
            self.get_parameter('quick_relocalize_radius').value
        )
        self.quick_relocalize_sample_count = int(
            self.get_parameter('quick_relocalize_sample_count').value
        )
        self.quick_relocalize_timeout = float(
            self.get_parameter('quick_relocalize_timeout').value
        )
        self.tag_mount_frame_prefix = str(
            self.get_parameter('tag_mount_frame_prefix').value
        )
        self.quick_align_kp = float(self.get_parameter('quick_align_kp').value)
        self.quick_align_min_speed = float(
            self.get_parameter('quick_align_min_speed').value
        )
        self.quick_align_max_speed = float(
            self.get_parameter('quick_align_max_speed').value
        )
        self.quick_align_tolerance = math.radians(
            float(self.get_parameter('quick_align_tolerance_deg').value)
        )

        if self.relocalize_required_distance <= self.relocalize_due_distance:
            raise ValueError(
                'relocalize_required_distance must be greater than '
                'relocalize_due_distance'
            )

        # ------------------------------------------------------------------
        # Mission state specific to this optimized branch
        # ------------------------------------------------------------------
        self.runtime_phase = ''
        self.patrol_order_initialized = False
        self.patrol_scan_had_detection = False

        self.slow_observation_active = False
        self.slow_observation_scan_state = None
        self.observation_first_id = ''
        self.observation_first_last_seen = 0.0
        self.observation_started = 0.0
        self.observation_seen = {}

        self.quick_align_active = False
        self.quick_watch = None
        self.quick_relocalize_goal_handle = None
        self.quick_relocalize_in_progress = False

        self.odom_tracking_enabled = False
        self.last_odom_xy = None
        self.distance_since_relocalize = 0.0
        self.relocalization_status = 'NORMAL'

        self.mow_origin_state = None

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        control_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.scan_cmd_pub = self.create_publisher(
            TwistStamped,
            self.scan_cmd_vel_topic,
            control_qos,
        )
        self.collection_phase_pub = self.create_publisher(
            String,
            '/mission/collection_phase',
            transient_qos,
        )
        self.relocalization_status_pub = self.create_publisher(
            String,
            '/mission/relocalization_status',
            transient_qos,
        )

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

        self.scan_control_timer = self.create_timer(
            1.0 / max(scan_control_rate, 1.0),
            self.scan_control_loop,
        )

        self.publish_relocalization_status(force=True)
        self.publish_collection_phase('IDLE')
        self.get_logger().info(
            'Optimized mow mission enabled: no grouping/staging/local collection; '
            'farthest visible <= 3 m -> Nav2 places pickup link on target.'
        )

    # ==================================================================
    # Common helpers
    # ==================================================================

    def publish_state(self):
        msg = String()
        phase = getattr(self, 'runtime_phase', '')
        msg.data = phase if phase else self.state.name
        self.state_pub.publish(msg)

    def _set_runtime_phase(self, phase):
        self.runtime_phase = phase
        self.publish_state()

    def _clear_runtime_phase(self):
        self.runtime_phase = ''
        self.publish_state()

    def publish_collection_phase(self, phase):
        msg = String()
        msg.data = phase
        self.collection_phase_pub.publish(msg)

    def _publish_scan_twist(self, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.angular.z = float(angular_z)
        self.scan_cmd_pub.publish(msg)

    def _robot_pose_in_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException:
            return None
        return (
            float(tf.translation.x),
            float(tf.translation.y),
            quaternion_to_yaw(tf.rotation),
        )

    # ==================================================================
    # Initial patrol ordering: nearest point, then least-turn direction
    # ==================================================================

    @staticmethod
    def _segment_heading(a, b):
        return math.atan2(
            float(b.position.y) - float(a.position.y),
            float(b.position.x) - float(a.position.x),
        )

    def _initial_turn(self, route, robot_yaw):
        if len(route) < 2:
            return 0.0
        return abs(
            wrap_angle(self._segment_heading(route[0], route[1]) - robot_yaw)
        )

    def _orient_patrol_route(self, route, fallback_yaw):
        for i, pose in enumerate(route):
            if len(route) == 1:
                yaw = fallback_yaw
            elif i < len(route) - 1:
                yaw = self._segment_heading(route[i], route[i + 1])
            else:
                yaw = self._segment_heading(route[i - 1], route[i])
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)

    def _publish_reordered_patrol_points(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.poses = [copy.deepcopy(pose) for pose in self.patrol_points]
        self.patrol_points_pub.publish(msg)

    def _optimize_initial_patrol_order(self):
        if self.patrol_order_initialized or not self.patrol_points:
            return
        robot = self._robot_pose_in_map()
        if robot is None:
            self.get_logger().warn(
                'No map->base pose for patrol optimization; using generated order.'
            )
            self.patrol_order_initialized = True
            return

        rx, ry, robot_yaw = robot
        original = [copy.deepcopy(pose) for pose in self.patrol_points]
        n = len(original)
        nearest = min(
            range(n),
            key=lambda i: math.hypot(
                float(original[i].position.x) - rx,
                float(original[i].position.y) - ry,
            ),
        )

        fidx = list(range(nearest, n)) + list(range(0, nearest))
        ridx = [nearest] + list(range(nearest - 1, -1, -1)) + list(
            range(n - 1, nearest, -1)
        )
        forward = [copy.deepcopy(original[i]) for i in fidx]
        reverse = [copy.deepcopy(original[i]) for i in ridx]

        fturn = self._initial_turn(forward, robot_yaw)
        rturn = self._initial_turn(reverse, robot_yaw)
        if rturn < fturn:
            route, direction, turn = reverse, 'reverse', rturn
        else:
            route, direction, turn = forward, 'forward', fturn

        self._orient_patrol_route(route, robot_yaw)
        self.patrol_points = route
        self.current_patrol_index = 0
        self.active_patrol_pose = None
        self.patrol_order_initialized = True
        self._publish_reordered_patrol_points()

        p = route[0].position
        self.get_logger().info(
            f'Patrol starts at nearest P{nearest} ({p.x:.2f}, {p.y:.2f}); '
            f'{direction} order needs {math.degrees(turn):.1f} deg initial turn.'
        )

    def start_nav2_then_patrol(self):
        self._optimize_initial_patrol_order()
        super().start_nav2_then_patrol()

    # ==================================================================
    # Distance-based runtime relocalization status
    # ==================================================================

    def odom_callback(self, msg):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self.last_odom_xy is not None and self.odom_tracking_enabled:
            step = math.hypot(x - self.last_odom_xy[0], y - self.last_odom_xy[1])
            if 0.0 <= step <= 1.0:
                self.distance_since_relocalize += step
                self.publish_relocalization_status()
        self.last_odom_xy = (x, y)

    def publish_relocalization_status(self, force=False):
        status = 'NORMAL'
        if self.distance_since_relocalize >= self.relocalize_required_distance:
            status = 'REQUIRED'
        elif self.distance_since_relocalize >= self.relocalize_due_distance:
            status = 'DUE'

        if force or status != self.relocalization_status:
            self.relocalization_status = status
            msg = String()
            msg.data = status
            self.relocalization_status_pub.publish(msg)
            self.get_logger().info(
                f'Relocalization status -> {status} '
                f'({self.distance_since_relocalize:.2f} m odom path)'
            )

    def _reset_relocalization_distance(self):
        self.distance_since_relocalize = 0.0
        self.last_odom_xy = None
        self.odom_tracking_enabled = True
        self.publish_relocalization_status(force=True)

    def relocalize_result(self, future):
        # Initial localization still uses the existing /approach_tag ->
        # /relocalize transaction. Runtime quick relocalization has its own
        # callbacks below and never reaches this function.
        wrapped = future.result()
        super().relocalize_result(future)
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.success:
            self._reset_relocalization_distance()

    # ==================================================================
    # Visible shuttle handling
    # ==================================================================

    def _cache_visible(self, msg):
        ordered = []
        for detection in msg.detections:
            track_id = detection.id.strip()
            if not track_id or track_id in self.attempted_ids:
                continue
            ordered.append((track_id, copy.deepcopy(detection)))
        self.visible_ordered = ordered
        return ordered

    def visible_tracks_callback(self, msg):
        ordered = self._cache_visible(msg)

        if self.quick_align_active or self.quick_relocalize_in_progress:
            return

        if self.slow_observation_active:
            now = time.monotonic()
            for track_id, detection in ordered:
                self.observation_seen[track_id] = copy.deepcopy(detection)
                if track_id == self.observation_first_id:
                    self.observation_first_last_seen = now
            return

        if self.state in (
            MissionState.PATROL_SCAN,
            MissionState.LOCAL_SCAN,
            MissionState.FINAL_PATROL_SCAN,
        ) and ordered:
            self._begin_slow_observation(ordered)
            return

        # Returning to Pi is also useful search time. Do not stop to group or
        # observe; immediately choose the farthest current candidate and mow.
        if self.state == MissionState.RETURN_TO_PATROL and ordered:
            selected = self._select_farthest(ordered)
            if selected is not None and self.navigation_cancel_reason is None:
                self.active_target_id, self.active_target_detection = selected
                self.mow_origin_state = MissionState.RETURN_TO_PATROL
                self.get_logger().info(
                    f'Return path sees shuttle {self.active_target_id}; '
                    'canceling return and mowing it.'
                )
                self._cancel_navigation('return_target_seen')

    def _begin_slow_observation(self, ordered):
        first_id, _ = ordered[0]
        self.slow_observation_active = True
        self.slow_observation_scan_state = self.state
        self.observation_first_id = first_id
        self.observation_first_last_seen = time.monotonic()
        self.observation_started = time.monotonic()
        self.observation_seen = {
            track_id: copy.deepcopy(detection)
            for track_id, detection in ordered
        }

        self.pending_spin = False
        self.spin_cancel_requested = True
        if self.spin_goal_handle is not None:
            self.spin_goal_handle.cancel_goal_async()

        self._set_runtime_phase('SLOW_OBSERVE')
        self.get_logger().info(
            f'First shuttle {first_id} seen: canceling fast Spin and slowing to '
            f'{self.slow_observe_angular_velocity:.2f} rad/s before selection.'
        )

    def _select_farthest(self, candidates):
        robot = self._robot_pose_in_map()
        if robot is None:
            return None
        rx, ry, _ = robot

        best = None
        best_range = -1.0
        iterable = (
            candidates.items()
            if isinstance(candidates, dict)
            else candidates
        )
        for track_id, detection in iterable:
            if track_id in self.attempted_ids:
                continue
            x, y, _ = detection_position(detection)
            distance = math.hypot(x - rx, y - ry)
            if distance <= self.mow_target_max_range and distance > best_range:
                best = (track_id, copy.deepcopy(detection))
                best_range = distance

        if best is not None:
            self.get_logger().info(
                f'Farthest eligible shuttle is {best[0]} at {best_range:.2f} m.'
            )
        return best

    # ==================================================================
    # Conservative scan handoff + quick tag heading controller
    # ==================================================================

    def scan_control_loop(self):
        if self.slow_observation_active:
            self._slow_observation_step()
            return
        if self.quick_align_active:
            self._quick_align_step()

    def _slow_observation_step(self):
        sign = 1.0 if self.spin_angle >= 0.0 else -1.0
        self._publish_scan_twist(sign * self.slow_observe_angular_velocity)

        now = time.monotonic()
        first = self._visible_detection(self.observation_first_id)
        if first is not None:
            self.observation_seen[self.observation_first_id] = copy.deepcopy(first)
            self.observation_first_last_seen = now
            bearing = self._primary_bearing(first)
            if bearing is not None:
                if sign > 0.0:
                    edge_reached = bearing <= -abs(self.slow_observe_edge_bearing)
                else:
                    edge_reached = bearing >= abs(self.slow_observe_edge_bearing)
                if edge_reached:
                    self._finish_slow_observation('first shuttle reached safe image edge')
                    return

        if now - self.observation_first_last_seen >= self.scan_first_lost_grace:
            self._finish_slow_observation('first shuttle left frame')
            return

        if now - self.observation_started >= self.slow_observe_timeout:
            self._finish_slow_observation('slow observation timeout')

    def _finish_slow_observation(self, reason):
        scan_state = self.slow_observation_scan_state
        self.slow_observation_active = False
        self._publish_scan_twist(0.0)
        self.spin_cancel_requested = False
        self._clear_runtime_phase()

        selected = self._select_farthest(self.observation_seen)
        self.observation_seen = {}
        self.observation_first_id = ''

        if selected is None:
            self.get_logger().warn(
                f'Observation ended ({reason}) but no <= {self.mow_target_max_range:.1f} m '
                'candidate remained; restarting the scan.'
            )
            self.start_scan(scan_state)
            return

        self.active_target_id, self.active_target_detection = selected
        self.mow_origin_state = scan_state
        if scan_state == MissionState.PATROL_SCAN:
            self.patrol_scan_had_detection = True
        self.get_logger().info(f'Observation ended: {reason}.')
        self._start_mow_to_active_target()

    def _quick_align_step(self):
        robot = self._robot_pose_in_map()
        if robot is None or self.quick_watch is None:
            self._publish_scan_twist(0.0)
            return

        error = wrap_angle(self.quick_watch['yaw'] - robot[2])
        if abs(error) <= self.quick_align_tolerance:
            self._publish_scan_twist(0.0)
            self.quick_align_active = False
            self._start_quick_relocalize()
            return

        speed = clamp(
            self.quick_align_kp * error,
            -self.quick_align_max_speed,
            self.quick_align_max_speed,
        )
        if abs(speed) < self.quick_align_min_speed:
            speed = math.copysign(self.quick_align_min_speed, error)
        self._publish_scan_twist(speed)

    # ==================================================================
    # Scan action behavior
    # ==================================================================

    def start_scan(self, scan_state):
        self._clear_active_target()
        self.slow_observation_active = False
        self.observation_seen = {}
        self.observation_first_id = ''
        self.runtime_phase = ''
        if scan_state == MissionState.PATROL_SCAN:
            self.patrol_scan_had_detection = False
        self.set_state(scan_state)
        self.pending_spin = True
        self.process_pending_actions()

    def spin_result(self, future):
        wrapped = future.result()
        self.spin_goal_handle = None

        # A canceled Nav2 Spin is expected after the first shuttle appears.
        # The higher-priority /cmd_vel_scan controller is already doing the
        # slow observation, so there is nothing else to do here.
        if self.slow_observation_active:
            return

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            if wrapped.status == GoalStatus.STATUS_CANCELED:
                return
            self.enter_error(f'Spin failed with status={wrapped.status}.')
            return

        if self.state == MissionState.PATROL_SCAN:
            self.advance_patrol_point()
        elif self.state == MissionState.FINAL_PATROL_SCAN:
            self.advance_patrol_point()
        elif self.state == MissionState.LOCAL_SCAN:
            self.return_to_active_patrol_point()

    # ==================================================================
    # Nav2 mowing
    # ==================================================================

    def _compute_mow_goal(self):
        if self.active_target_detection is None:
            return None
        robot = self._robot_pose_in_map()
        if robot is None:
            return None

        tx, ty, _ = detection_position(self.active_target_detection)
        rx, ry, _ = robot
        dx = tx - rx
        dy = ty - ry
        distance = math.hypot(dx, dy)
        if distance <= 1e-6:
            return None

        ux = dx / distance
        uy = dy / distance
        yaw = math.atan2(uy, ux)

        # Final base pose is chosen so pickup_link, not base_footprint, reaches
        # the shuttle. A tiny overrun places the collection link just beyond it.
        base_to_goal = self.pickup_offset_x - self.mow_overrun
        pose = Pose()
        pose.position.x = tx - base_to_goal * ux
        pose.position.y = ty - base_to_goal * uy
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _start_mow_to_active_target(self):
        if not self.active_target_id or self.active_target_detection is None:
            self._resume_after_mow()
            return
        goal = self._compute_mow_goal()
        if goal is None:
            self.get_logger().warn('Could not compute Nav2 mow goal; skipping target.')
            self.attempted_ids.add(self.active_target_id)
            self._clear_active_target()
            self._resume_after_mow()
            return

        self.get_logger().info(
            f'Nav2 mowing toward shuttle {self.active_target_id}; '
            'pickup_link is the goal point.'
        )
        self.queue_navigation(goal, 'mow_target')

    def send_navigation_goal_now(self, pose, purpose):
        self.active_navigation_pose = copy.deepcopy(pose)
        self.active_navigation_purpose = purpose
        self.publish_current_goal(pose)

        if purpose == 'patrol':
            self.runtime_phase = ''
            self.set_state(MissionState.GO_TO_PATROL)
        elif purpose == 'return_patrol':
            self.runtime_phase = ''
            self.set_state(MissionState.RETURN_TO_PATROL)
        elif purpose == 'mow_target':
            self.state = MissionState.COLLECTING
            self._set_runtime_phase('MOW_TO_TARGET')
            self.publish_collection_phase('NAV2_MOW')
        elif purpose == 'quick_watch':
            self.state = MissionState.RETURN_TO_PATROL
            self._set_runtime_phase('GO_TO_QUICK_RELOCALIZE')

        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.pose = pose
        self.navigation_goal_request_pending = True
        self.navigate_client.send_goal_async(goal).add_done_callback(
            self.navigation_goal_response
        )

    def navigation_result(self, future):
        wrapped = future.result()
        purpose = self.active_navigation_purpose
        cancel_reason = self.navigation_cancel_reason

        self.navigate_goal_handle = None
        self.navigation_goal_request_pending = False

        if cancel_reason is not None:
            self.navigation_cancel_reason = None
            self.active_navigation_pose = None
            self.active_navigation_purpose = None
            if cancel_reason == 'return_target_seen':
                self._start_mow_to_active_target()
            return

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.retry_navigation(f'status={wrapped.status}')
            return

        self.active_navigation_pose = None
        self.active_navigation_purpose = None

        if purpose == 'patrol':
            self.start_scan(MissionState.PATROL_SCAN)
            return

        if purpose == 'return_patrol':
            if self.patrol_scan_had_detection:
                self.start_scan(MissionState.FINAL_PATROL_SCAN)
            else:
                self.advance_patrol_point()
            return

        if purpose == 'mow_target':
            completed_id = self.active_target_id
            if completed_id:
                self.attempted_ids.add(completed_id)
            self.publish_collection_phase('DONE')
            self.get_logger().info(
                f'Nav2 mow pass completed for target {completed_id}; '
                'mission does not infer physical collection success.'
            )
            self._clear_active_target()
            self.runtime_phase = ''
            self._after_mow_pass()
            return

        if purpose == 'quick_watch':
            self._start_quick_align(self.quick_watch)

    def retry_navigation(self, reason):
        purpose = self.active_navigation_purpose

        if self.navigation_retry_count < self.navigation_goal_retries:
            self.navigation_retry_count += 1
            self.navigate_goal_handle = None
            self.navigation_goal_request_pending = False
            self.pending_navigation = copy.deepcopy(self.active_navigation_pose)
            self.pending_navigation_purpose = purpose
            self.get_logger().warn(
                f'Navigation {purpose} failed ({reason}); retry '
                f'{self.navigation_retry_count}/{self.navigation_goal_retries}.'
            )
            self.update_action_retry_timer()
            return

        self.navigate_goal_handle = None
        self.navigation_goal_request_pending = False

        if purpose == 'mow_target':
            failed_id = self.active_target_id
            if failed_id:
                self.attempted_ids.add(failed_id)
            self.get_logger().warn(
                f'Skipping unreachable mow target {failed_id} after retries.'
            )
            self.publish_collection_phase('ABORTED')
            self._clear_active_target()
            self.runtime_phase = ''
            self._resume_after_mow()
            return

        if purpose == 'quick_watch':
            self.get_logger().warn(
                'Could not reach quick-relocalization watch pose; resuming mission.'
            )
            self.runtime_phase = ''
            self.quick_watch = None
            self._resume_after_mow()
            return

        self.enter_error(f'Navigation {purpose} failed: {reason}')

    def _handle_navigation_cancel_without_goal(self):
        reason = self.navigation_cancel_reason
        self.navigation_cancel_reason = None
        if reason == 'return_target_seen':
            self._start_mow_to_active_target()
            return
        super()._handle_navigation_cancel_without_goal()

    # ==================================================================
    # Direct quick runtime relocalization, no /approach_tag
    # ==================================================================

    def _watch_poses(self):
        poses = []
        for tag_id in range(4):
            frame = f'{self.tag_mount_frame_prefix}{tag_id}'
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.frame_id,
                    frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout),
                ).transform
            except TransformException:
                continue

            mount_yaw = quaternion_to_yaw(tf.rotation)
            x = float(tf.translation.x) + self.tag_approach_distance * math.cos(
                mount_yaw
            )
            y = float(tf.translation.y) + self.tag_approach_distance * math.sin(
                mount_yaw
            )
            poses.append(
                {
                    'tag_id': tag_id,
                    'x': x,
                    'y': y,
                    'yaw': wrap_angle(mount_yaw + math.pi),
                }
            )
        return poses

    def _nearest_watch(self):
        robot = self._robot_pose_in_map()
        if robot is None:
            return None, float('inf')
        watches = self._watch_poses()
        if not watches:
            return None, float('inf')
        watch = min(
            watches,
            key=lambda p: math.hypot(p['x'] - robot[0], p['y'] - robot[1]),
        )
        distance = math.hypot(watch['x'] - robot[0], watch['y'] - robot[1])
        return watch, distance

    def _watch_nav_pose(self, watch):
        pose = Pose()
        pose.position.x = watch['x']
        pose.position.y = watch['y']
        pose.orientation.z = math.sin(watch['yaw'] / 2.0)
        pose.orientation.w = math.cos(watch['yaw'] / 2.0)
        return pose

    def _maybe_start_runtime_relocalize(self):
        if self.relocalization_status == 'NORMAL':
            return False

        watch, distance = self._nearest_watch()
        if watch is None:
            return False

        if distance <= self.quick_relocalize_radius:
            self.get_logger().info(
                f'{self.relocalization_status}: mow ended {distance:.2f} m from '
                f'initial watch pose for tag {watch["tag_id"]}; quick relocalizing.'
            )
            self._start_quick_align(watch)
            return True

        if self.relocalization_status == 'REQUIRED':
            # Hard threshold: still bypass /approach_tag. Nav2 goes directly to
            # the known initial watch pose, then a short heading correction is
            # followed by /relocalize.
            self.quick_watch = watch
            self.get_logger().info(
                f'REQUIRED: Nav2 directly to tag {watch["tag_id"]} watch pose '
                f'({distance:.2f} m away), bypassing /approach_tag.'
            )
            self.queue_navigation(self._watch_nav_pose(watch), 'quick_watch')
            return True

        return False

    def _start_quick_align(self, watch):
        if watch is None:
            self._resume_after_mow()
            return
        self.quick_watch = watch
        self.quick_align_active = True
        self._set_runtime_phase('QUICK_TAG_ALIGN')

    def _start_quick_relocalize(self):
        if self.quick_watch is None:
            self._resume_after_mow()
            return
        if not self.relocalize_client.server_is_ready():
            self.get_logger().warn('/relocalize not ready; resuming mission.')
            self.runtime_phase = ''
            self.quick_watch = None
            self._resume_after_mow()
            return

        self.quick_relocalize_in_progress = True
        self._set_runtime_phase('QUICK_RELOCALIZATION')
        goal = Relocalize.Goal()
        goal.preferred_tag_id = int(self.quick_watch['tag_id'])
        goal.sample_count = self.quick_relocalize_sample_count
        goal.timeout_sec = self.quick_relocalize_timeout
        self.relocalize_client.send_goal_async(goal).add_done_callback(
            self._quick_relocalize_goal_response
        )

    def _quick_relocalize_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Quick /relocalize goal rejected; resuming.')
            self.quick_relocalize_in_progress = False
            self.runtime_phase = ''
            self.quick_watch = None
            self._resume_after_mow()
            return
        self.quick_relocalize_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            self._quick_relocalize_result
        )

    def _quick_relocalize_result(self, future):
        wrapped = future.result()
        self.quick_relocalize_goal_handle = None
        self.quick_relocalize_in_progress = False

        if wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.success:
            self.get_logger().info(
                f'Quick direct relocalization succeeded using '
                f'{list(wrapped.result.used_tag_ids)}.'
            )
            self._reset_relocalization_distance()
        else:
            self.get_logger().warn(
                'Quick direct relocalization failed; keeping DUE/REQUIRED status '
                'and resuming collection.'
            )

        self.runtime_phase = ''
        self.quick_watch = None
        self.publish_state()
        self._resume_after_mow()

    # ==================================================================
    # Fast post-mow continuation
    # ==================================================================

    def _after_mow_pass(self):
        if self._maybe_start_runtime_relocalize():
            return
        self._resume_after_mow()

    def _resume_after_mow(self):
        # Do not automatically stop/scan after every target. If another shuttle
        # is already visible, immediately mow the farthest current one.
        selected = self._select_farthest(self.visible_ordered)
        if selected is not None:
            self.active_target_id, self.active_target_detection = selected
            self._start_mow_to_active_target()
            return

        self.runtime_phase = ''
        self.return_to_active_patrol_point()


def main(args=None):
    rclpy.init(args=args)
    node = OptimizedMowPatrolManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
