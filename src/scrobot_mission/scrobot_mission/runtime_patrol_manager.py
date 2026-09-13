#!/usr/bin/env python3

import copy
import math

import rclpy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scrobot_mission.patrol_manager import MissionState, PatrolManager
from std_msgs.msg import String


class RuntimePatrolManager(PatrolManager):
    """PatrolManager with deferred distance-based runtime relocalization."""

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
        self.publish_relocalization_status(force=True)

        self.get_logger().info(
            'Runtime relocalization enabled: '
            f'DUE={self.relocalize_due_distance:.1f} m, '
            f'REQUIRED={self.relocalize_required_distance:.1f} m, '
            f'opportunity radius={self.relocalize_opportunity_radius:.1f} m.'
        )

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
