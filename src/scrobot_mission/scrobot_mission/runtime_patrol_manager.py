#!/usr/bin/env python3

import copy
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scrobot_mission.patrol_manager import MissionState, PatrolManager
from std_msgs.msg import String
from tf2_ros import TransformException


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RuntimePatrolManager(PatrolManager):
    """PatrolManager with runtime relocalization and startup route optimization."""

    def __init__(self):
        super().__init__()

        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('relocalize_due_distance', 8.0)
        self.declare_parameter('relocalize_required_distance', 15.0)
        self.declare_parameter('relocalize_opportunity_radius', 2.50)
        self.declare_parameter('left_tag_region_x', 0.0)
        self.declare_parameter('left_tag_region_y', 3.05)
        self.declare_parameter('right_tag_region_x', 0.0)
        self.declare_parameter('right_tag_region_y', -3.05)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.relocalize_due_distance = float(
            self.get_parameter('relocalize_due_distance').value
        )
        self.relocalize_required_distance = float(
            self.get_parameter('relocalize_required_distance').value
        )
        self.relocalize_opportunity_radius = float(
            self.get_parameter('relocalize_opportunity_radius').value
        )
        self.tag_regions = [
            (
                float(self.get_parameter('left_tag_region_x').value),
                float(self.get_parameter('left_tag_region_y').value),
            ),
            (
                float(self.get_parameter('right_tag_region_x').value),
                float(self.get_parameter('right_tag_region_y').value),
            ),
        ]

        if self.relocalize_due_distance <= 0.0:
            raise ValueError('relocalize_due_distance must be > 0')
        if self.relocalize_required_distance <= self.relocalize_due_distance:
            raise ValueError(
                'relocalize_required_distance must be greater than relocalize_due_distance'
            )

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.relocalization_status_pub = self.create_publisher(
            String,
            '/mission/relocalization_status',
            state_qos,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

        self.runtime_localization_active = False
        self.runtime_phase = ''
        self.odom_tracking_enabled = False
        self.last_odom_xy = None
        self.distance_since_relocalize = 0.0
        self.relocalization_status = 'NORMAL'

        # Performance optimizations.
        self.patrol_order_initialized = False
        self.patrol_scan_had_detection = False

        self.publish_relocalization_status(force=True)

        self.get_logger().info(
            'Runtime relocalization enabled: '
            f'DUE={self.relocalize_due_distance:.1f} m, '
            f'REQUIRED={self.relocalize_required_distance:.1f} m, '
            f'opportunity radius={self.relocalize_opportunity_radius:.1f} m.'
        )

    # ------------------------------------------------------------------
    # State publishing
    # ------------------------------------------------------------------

    def publish_state(self):
        msg = String()
        phase = getattr(self, 'runtime_phase', '')
        msg.data = phase if phase else self.state.name
        self.state_pub.publish(msg)

    def enter_error(self, reason):
        self.runtime_phase = ''
        self.runtime_localization_active = False
        super().enter_error(reason)

    def _set_runtime_phase(self, phase):
        self.runtime_phase = phase
        self.get_logger().info(f'Runtime localization phase -> {phase}')
        self.publish_state()

    # ------------------------------------------------------------------
    # Initial patrol order optimization
    # ------------------------------------------------------------------

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

    @staticmethod
    def _segment_heading(a, b):
        return math.atan2(
            float(b.position.y) - float(a.position.y),
            float(b.position.x) - float(a.position.x),
        )

    def _initial_turn(self, route, robot_yaw):
        if len(route) < 2:
            return 0.0
        heading = self._segment_heading(route[0], route[1])
        return abs(wrap_angle(heading - robot_yaw))

    def _orient_patrol_route(self, route, fallback_yaw):
        if not route:
            return
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
                'Could not read map->base pose after initial localization; '
                'keeping generated patrol order.'
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

        forward_indices = list(range(nearest, n)) + list(range(0, nearest))
        reverse_indices = (
            [nearest]
            + list(range(nearest - 1, -1, -1))
            + list(range(n - 1, nearest, -1))
        )
        forward = [copy.deepcopy(original[i]) for i in forward_indices]
        reverse = [copy.deepcopy(original[i]) for i in reverse_indices]

        forward_turn = self._initial_turn(forward, robot_yaw)
        reverse_turn = self._initial_turn(reverse, robot_yaw)
        if reverse_turn < forward_turn:
            route = reverse
            direction = 'reverse'
            turn = reverse_turn
        else:
            route = forward
            direction = 'forward'
            turn = forward_turn

        self._orient_patrol_route(route, robot_yaw)
        self.patrol_points = route
        self.current_patrol_index = 0
        self.active_patrol_pose = None
        self.patrol_order_initialized = True
        self._publish_reordered_patrol_points()

        first = self.patrol_points[0].position
        self.get_logger().info(
            'Optimized patrol start after global localization: '
            f'nearest original P{nearest} at ({first.x:.2f}, {first.y:.2f}), '
            f'direction={direction}, initial route turn={math.degrees(turn):.1f} deg.'
        )

    def start_nav2_then_patrol(self):
        self._optimize_initial_patrol_order()
        super().start_nav2_then_patrol()

    # ------------------------------------------------------------------
    # Runtime relocalization status
    # ------------------------------------------------------------------

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
                f'(odom path={self.distance_since_relocalize:.2f} m)'
            )

    def _relocalization_required(self):
        return self.relocalization_status == 'REQUIRED'

    def odom_callback(self, msg):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)

        if self.last_odom_xy is not None and self.odom_tracking_enabled:
            step = math.hypot(x - self.last_odom_xy[0], y - self.last_odom_xy[1])
            if 0.0 <= step <= 1.0:
                self.distance_since_relocalize += step
                self.publish_relocalization_status()

        self.last_odom_xy = (x, y)
        self._evaluate_return_relocalization()

    def _reset_relocalization_distance(self):
        self.distance_since_relocalize = 0.0
        self.last_odom_xy = None
        self.odom_tracking_enabled = True
        self.publish_relocalization_status(force=True)

    def _near_relocalization_opportunity(self):
        robot_xy = self._robot_xy_in_map()
        if robot_xy is None:
            return False
        return any(
            math.hypot(robot_xy[0] - tx, robot_xy[1] - ty)
            <= self.relocalize_opportunity_radius
            for tx, ty in self.tag_regions
        )

    def _evaluate_return_relocalization(self):
        if self.runtime_localization_active:
            return
        if self.state != MissionState.RETURN_TO_PATROL:
            return
        if self.active_target_id or self.navigation_cancel_reason is not None:
            return

        if self._relocalization_required():
            self.get_logger().info(
                'RELOCALIZE_REQUIRED while returning; interrupting RETURN_TO_PATROL.'
            )
            self._cancel_navigation('runtime_required')
            return

        if (
            self.relocalization_status == 'DUE'
            and not self.visible_ordered
            and self._near_relocalization_opportunity()
        ):
            self.get_logger().info(
                'RELOCALIZE_DUE + near tag region + no visible shuttle; '
                'taking the convenient relocalization opportunity.'
            )
            self._cancel_navigation('runtime_due')

    # ------------------------------------------------------------------
    # Shuttle priority and patrol scan bookkeeping
    # ------------------------------------------------------------------

    def _update_visible_cache_only(self, msg):
        ordered = []
        for detection in msg.detections:
            track_id = detection.id.strip()
            if not track_id or track_id in self.attempted_ids:
                continue
            ordered.append((track_id, copy.deepcopy(detection)))
        self.visible_ordered = ordered

    def visible_tracks_callback(self, msg):
        if self.runtime_localization_active:
            self._update_visible_cache_only(msg)
            return

        if self.state == MissionState.RETURN_TO_PATROL and self._relocalization_required():
            self._update_visible_cache_only(msg)
            if self.navigation_cancel_reason is None:
                self._cancel_navigation('runtime_required')
            return

        super().visible_tracks_callback(msg)

    def acquire_target(self, track_id, detection, continue_scan=False):
        if self.state == MissionState.PATROL_SCAN:
            self.patrol_scan_had_detection = True
        super().acquire_target(track_id, detection, continue_scan=continue_scan)

    # ------------------------------------------------------------------
    # Atomic /approach_tag -> /relocalize transaction
    # ------------------------------------------------------------------

    def start_runtime_relocalization(self, reason):
        if self.runtime_localization_active:
            return

        self.runtime_localization_active = True
        self.pending_spin = False
        self._clear_active_target()
        self.get_logger().info(
            f'Starting atomic runtime localization ({reason}): '
            '/approach_tag -> /relocalize.'
        )
        self._set_runtime_phase('RUNTIME_TAG_APPROACH')
        self.pending_approach = True
        self.process_pending_actions()

    def approach_result(self, future):
        if not self.runtime_localization_active:
            return super().approach_result(future)

        wrapped = future.result()
        self.approach_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            self.enter_error('Runtime ApproachTag failed.')
            return

        self.last_relocalize_tag = int(wrapped.result.tag_id)
        self.relocalize_retry_count = 0
        self._set_runtime_phase('RUNTIME_RELOCALIZATION')
        self.pending_relocalize_tag = self.last_relocalize_tag
        self.process_pending_actions()

    def relocalize_result(self, future):
        if not self.runtime_localization_active:
            wrapped = future.result()
            super().relocalize_result(future)
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.success:
                self._reset_relocalization_distance()
            return

        wrapped = future.result()
        self.relocalize_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            self.retry_relocalization('runtime action failed')
            return

        self._reset_relocalization_distance()
        self.runtime_localization_active = False
        self.runtime_phase = ''
        self.get_logger().info(
            'Runtime relocalization complete; checking visible shuttles at the tag spot.'
        )
        self.publish_state()

        for track_id, detection in self.visible_ordered:
            if track_id not in self.attempted_ids:
                self.acquire_target(track_id, detection, continue_scan=False)
                return

        self.return_to_active_patrol_point()

    # ------------------------------------------------------------------
    # Post-collection / scan decisions
    # ------------------------------------------------------------------

    def _after_collection_attempt(self, drive_completed):
        attempted_group = list(self.active_group_ids)
        if drive_completed:
            self.attempted_ids.update(attempted_group)
        self._clear_active_target()

        if self._relocalization_required():
            self.start_runtime_relocalization('required after collection attempt')
            return

        for next_id, detection in self.visible_ordered:
            if next_id not in self.attempted_ids:
                self.acquire_target(next_id, detection, continue_scan=False)
                return

        self.start_scan(MissionState.LOCAL_SCAN)

    def start_scan(self, scan_state):
        if self._relocalization_required() and not self.runtime_localization_active:
            self._clear_active_target()
            self.start_runtime_relocalization('required before next scan')
            return

        if scan_state == MissionState.PATROL_SCAN:
            # A FINAL_PATROL_SCAN is only justified when this original anchor
            # scan actually discovered a shuttle and caused an excursion.
            self.patrol_scan_had_detection = False

        if (
            scan_state == MissionState.FINAL_PATROL_SCAN
            and not self.patrol_scan_had_detection
        ):
            self.get_logger().info(
                'Skipping FINAL_PATROL_SCAN: the original PATROL_SCAN at this '
                'anchor found no shuttle.'
            )
            self.advance_patrol_point()
            return

        super().start_scan(scan_state)

    def return_to_active_patrol_point(self):
        if self.active_patrol_pose is None:
            self.enter_error('No patrol anchor stored.')
            return

        if self._relocalization_required():
            self.start_runtime_relocalization('required before return to patrol')
            return

        if (
            self.relocalization_status == 'DUE'
            and not self.visible_ordered
            and self._near_relocalization_opportunity()
        ):
            self.start_runtime_relocalization('due near tag before return')
            return

        super().return_to_active_patrol_point()

    def _handle_navigation_cancel_without_goal(self):
        reason = self.navigation_cancel_reason
        if reason in ('runtime_due', 'runtime_required'):
            self.navigation_cancel_reason = None
            self.start_runtime_relocalization(reason)
            return
        super()._handle_navigation_cancel_without_goal()

    def navigation_result(self, future):
        cancel_reason = self.navigation_cancel_reason
        if cancel_reason not in ('runtime_due', 'runtime_required'):
            return super().navigation_result(future)

        future.result()
        self.navigate_goal_handle = None
        self.navigation_goal_request_pending = False
        self.navigation_cancel_reason = None
        self.active_navigation_pose = None
        self.active_navigation_purpose = None
        self.start_runtime_relocalization(cancel_reason)


def main(args=None):
    rclpy.init(args=args)
    node = RuntimePatrolManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
