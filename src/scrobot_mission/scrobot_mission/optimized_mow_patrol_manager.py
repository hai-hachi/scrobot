#!/usr/bin/env python3

import copy
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray, TwistStamped
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
from scrobot_mission.patrol_points import (
    generate_tag_watch_poses,
    precompute_tag_patrol_routes,
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
    """Minimal optimized mission: patrol scan -> observe -> Nav2 mow -> local scan."""

    def __init__(self):
        super().__init__()

        # ------------------------------------------------------------------
        # Mowing and scan parameters
        # ------------------------------------------------------------------
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('scan_cmd_vel_topic', '/cmd_vel_scan')
        self.declare_parameter('mow_target_max_range', 3.0)
        self.declare_parameter('pickup_offset_x', 0.165)
        self.declare_parameter('mow_overrun', 0.03)

        # Empty-area scan uses Nav2 Spin. Once a shuttle appears, the Spin is
        # canceled and a deliberately slow direct angular command takes over.
        self.declare_parameter('slow_observe_angular_velocity', 0.18)
        self.declare_parameter('slow_observe_edge_bearing', 0.28)
        self.declare_parameter('slow_observe_timeout', 4.0)
        self.declare_parameter('scan_first_lost_grace', 0.30)
        self.declare_parameter('scan_control_rate', 30.0)

        # Tag-aware patrol entry is calculated beforehand from known court/tag
        # geometry in patrol_points.py. Heading is intentionally weighted enough
        # that a point already in front of the robot can beat the nearest point.
        self.declare_parameter('patrol_entry_heading_weight', 1.50)

        # Runtime relocalization is opportunistic only. No runtime
        # /approach_tag and no forced Nav2 detour to a tag position.
        self.declare_parameter('relocalize_due_distance', 15.0)
        self.declare_parameter('relocalize_required_distance', 30.0)
        self.declare_parameter('quick_relocalize_radius', 0.50)
        self.declare_parameter('quick_relocalize_sample_count', 8)
        self.declare_parameter('quick_relocalize_timeout', 6.0)
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
        self.patrol_entry_heading_weight = float(
            self.get_parameter('patrol_entry_heading_weight').value
        )

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
        # Effective optimized state
        # ------------------------------------------------------------------
        # Published mission flow contains only:
        # IDLE -> INITIAL_TAG_APPROACH -> INITIAL_RELOCALIZATION -> STARTING_NAV2
        # -> GO_TO_PATROL -> PATROL_SCAN -> SLOW_OBSERVE -> MOW_TO_TARGET
        # -> LOCAL_SCAN -> RETURN_TO_PATROL -> ... -> COMPLETE.
        # QUICK_TAG_ALIGN / QUICK_RELOCALIZATION may be inserted after a mow.
        self.runtime_phase = ''

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

        self.tag_patrol_routes = {}
        self.tag_watch_poses = generate_tag_watch_poses(
            target_distance=self.tag_approach_distance
        )
        self.patrol_route_applied = False

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

        self._precompute_tag_patrol_routes()
        self.publish_relocalization_status(force=True)
        self.publish_collection_phase('IDLE')
        self.get_logger().info(
            'Optimized mow mission: patrol scan -> slow observe -> farthest <=3m '
            '-> Nav2 mow -> local scan. No grouping/staging/final-approach states.'
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
    # Tag-aware patrol route is calculated before the mission starts
    # ==================================================================

    def _patrol_points_as_tuples(self):
        return [
            (
                float(pose.position.x),
                float(pose.position.y),
                quaternion_to_yaw(pose.orientation),
            )
            for pose in self.patrol_points
        ]

    @staticmethod
    def _tuple_route_to_poses(route):
        poses = []
        for x, y, yaw in route:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(pose)
        return poses

    def _precompute_tag_patrol_routes(self):
        original = self._patrol_points_as_tuples()
        self.tag_patrol_routes = precompute_tag_patrol_routes(
            original,
            target_distance=self.tag_approach_distance,
            heading_weight_m_per_rad=self.patrol_entry_heading_weight,
        )

        for tag_id in sorted(self.tag_patrol_routes):
            info = self.tag_patrol_routes[tag_id]
            direction = 'forward' if info['direction'] > 0 else 'reverse'
            self.get_logger().info(
                f'Precomputed tag {tag_id} patrol entry: P{info["start_index"]}, '
                f'{direction}, entry={info["entry_distance"]:.2f} m, '
                f'turn={math.degrees(info["entry_turn"]):.1f} deg.'
            )

    def _publish_patrol_route(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.poses = [copy.deepcopy(pose) for pose in self.patrol_points]
        self.patrol_points_pub.publish(msg)

    def _apply_precomputed_tag_route(self):
        if self.patrol_route_applied:
            return
        self.patrol_route_applied = True

        tag_id = int(self.last_relocalize_tag)
        info = self.tag_patrol_routes.get(tag_id)
        if info is None:
            self.get_logger().warn(
                f'No precomputed patrol route for tag {tag_id}; using default order.'
            )
            return

        self.patrol_points = self._tuple_route_to_poses(info['route'])
        self.current_patrol_index = 0
        self.active_patrol_pose = None
        self._publish_patrol_route()

        direction = 'forward' if info['direction'] > 0 else 'reverse'
        self.get_logger().info(
            f'Initial relocalization used tag {tag_id}: starting at original '
            f'P{info["start_index"]} with {direction} patrol order.'
        )

    def start_nav2_then_patrol(self):
        self._apply_precomputed_tag_route()
        super().start_nav2_then_patrol()

    # ==================================================================
    # Distance-based localization status
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
        wrapped = future.result()
        success = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped.result.success
        )
        super().relocalize_result(future)
        if success:
            self._reset_relocalization_distance()

    # ==================================================================
    # Shuttle observation and target selection
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

        if self.state in (MissionState.PATROL_SCAN, MissionState.LOCAL_SCAN) and ordered:
            self._begin_slow_observation(ordered)
            return

        # Returning to the patrol anchor remains useful search time. If a target
        # appears, interrupt the return and mow it; a local scan follows the mow.
        if self.state == MissionState.RETURN_TO_PATROL and ordered:
            selected = self._select_farthest(ordered)
            if selected is not None and self.navigation_cancel_reason is None:
                self.active_target_id, self.active_target_detection = selected
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
            f'First shuttle {first_id} seen: canceling Spin and slowing to '
            f'{self.slow_observe_angular_velocity:.2f} rad/s.'
        )

    def _select_farthest(self, candidates):
        robot = self._robot_pose_in_map()
        if robot is None:
            return None
        rx, ry, _ = robot

        best = None
        best_range = -1.0
        iterable = candidates.items() if isinstance(candidates, dict) else candidates
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
    # Conservative scan handoff and quick tag heading controller
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
                edge_reached = (
                    bearing <= -abs(self.slow_observe_edge_bearing)
                    if sign > 0.0
                    else bearing >= abs(self.slow_observe_edge_bearing)
                )
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
                f'Observation ended ({reason}) but no candidate remained within '
                f'{self.mow_target_max_range:.1f} m; restarting scan.'
            )
            self.start_scan(scan_state)
            return

        self.active_target_id, self.active_target_detection = selected
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
    # Patrol/local spin behavior
    # ==================================================================

    def start_scan(self, scan_state):
        # Only two scan states exist in the optimized flow.
        if scan_state not in (MissionState.PATROL_SCAN, MissionState.LOCAL_SCAN):
            self.enter_error(f'Unexpected optimized scan state: {scan_state.name}')
            return

        self._clear_active_target()
        self.slow_observation_active = False
        self.observation_seen = {}
        self.observation_first_id = ''
        self.runtime_phase = ''
        self.set_state(scan_state)
        self.pending_spin = True
        self.process_pending_actions()

    def spin_result(self, future):
        wrapped = future.result()
        self.spin_goal_handle = None

        # Canceled Spin is expected when first detection hands control to the
        # conservative slow-observation controller.
        if self.slow_observation_active:
            return

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            if wrapped.status == GoalStatus.STATUS_CANCELED:
                return
            self.enter_error(f'Spin failed with status={wrapped.status}.')
            return

        if self.state == MissionState.PATROL_SCAN:
            self.advance_patrol_point()
        elif self.state == MissionState.LOCAL_SCAN:
            self.return_to_active_patrol_point()

    # ==================================================================
    # Nav2 mowing and navigation progression
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

        # Base goal places pickup_link slightly beyond the target shuttle.
        base_to_goal = self.pickup_offset_x - self.mow_overrun
        pose = Pose()
        pose.position.x = tx - base_to_goal * ux
        pose.position.y = ty - base_to_goal * uy
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _start_mow_to_active_target(self):
        if not self.active_target_id or self.active_target_detection is None:
            self.start_scan(MissionState.LOCAL_SCAN)
            return

        goal = self._compute_mow_goal()
        if goal is None:
            self.get_logger().warn('Could not compute Nav2 mow goal; skipping target.')
            self.attempted_ids.add(self.active_target_id)
            self._clear_active_target()
            self.start_scan(MissionState.LOCAL_SCAN)
            return

        self.get_logger().info(
            f'Nav2 mowing toward shuttle {self.active_target_id}; '
            'pickup_link is the collection goal.'
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
        else:
            self.enter_error(f'Unknown optimized navigation purpose: {purpose}')
            return

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
            # No FINAL_PATROL_SCAN. The anchor was already scanned before the
            # excursion and every mow now has its own LOCAL_SCAN.
            self.advance_patrol_point()
            return

        if purpose == 'mow_target':
            completed_id = self.active_target_id
            if completed_id:
                self.attempted_ids.add(completed_id)
            self.publish_collection_phase('DONE')
            self.get_logger().info(
                f'Nav2 mow pass completed for target {completed_id}; starting '
                'local search before returning to patrol.'
            )
            self._clear_active_target()
            self.runtime_phase = ''
            self._after_mow_pass()

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
            self.start_scan(MissionState.LOCAL_SCAN)
            return

        self.enter_error(f'Navigation {purpose} failed: {reason}')

    def _handle_navigation_cancel_without_goal(self):
        reason = self.navigation_cancel_reason
        self.navigation_cancel_reason = None
        if reason == 'return_target_seen':
            self._start_mow_to_active_target()
            return
        self.enter_error(f'Unexpected optimized navigation cancel: {reason}')

    # ==================================================================
    # Opportunistic direct runtime /relocalize
    # ==================================================================

    def _nearest_watch(self):
        robot = self._robot_pose_in_map()
        if robot is None:
            return None, float('inf')

        watch = None
        best_distance = float('inf')
        for tag_id, (x, y, yaw) in self.tag_watch_poses.items():
            distance = math.hypot(x - robot[0], y - robot[1])
            if distance < best_distance:
                watch = {'tag_id': tag_id, 'x': x, 'y': y, 'yaw': yaw}
                best_distance = distance
        return watch, best_distance

    def _maybe_start_runtime_relocalize(self):
        if self.relocalization_status == 'NORMAL':
            return False

        watch, distance = self._nearest_watch()
        if watch is None or distance > self.quick_relocalize_radius:
            return False

        self.get_logger().info(
            f'{self.relocalization_status}: mow ended {distance:.2f} m from '
            f'tag {watch["tag_id"]} watch pose; injecting direct /relocalize.'
        )
        self._start_quick_align(watch)
        return True

    def _start_quick_align(self, watch):
        self.quick_watch = watch
        self.quick_align_active = True
        self._set_runtime_phase('QUICK_TAG_ALIGN')

    def _start_quick_relocalize(self):
        if self.quick_watch is None:
            self.start_scan(MissionState.LOCAL_SCAN)
            return

        if not self.relocalize_client.server_is_ready():
            self.get_logger().warn('/relocalize not ready; continuing with local scan.')
            self.runtime_phase = ''
            self.quick_watch = None
            self.start_scan(MissionState.LOCAL_SCAN)
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
            self.get_logger().warn('Quick /relocalize rejected; continuing local scan.')
            self.quick_relocalize_in_progress = False
            self.runtime_phase = ''
            self.quick_watch = None
            self.start_scan(MissionState.LOCAL_SCAN)
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
                'Quick direct relocalization failed; preserving localization '
                'status and continuing local scan.'
            )

        self.runtime_phase = ''
        self.quick_watch = None
        self.publish_state()
        self.start_scan(MissionState.LOCAL_SCAN)

    # ==================================================================
    # Post-mow continuation
    # ==================================================================

    def _after_mow_pass(self):
        # Relocalization may be injected here, but only if the mow naturally
        # ended near a known initial watch pose. Otherwise immediately search
        # locally for another shuttle before going back to the patrol anchor.
        if self._maybe_start_runtime_relocalize():
            return
        self.start_scan(MissionState.LOCAL_SCAN)


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
