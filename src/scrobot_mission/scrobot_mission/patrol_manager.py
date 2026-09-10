#!/usr/bin/env python3

import math
import time
from enum import Enum, auto

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scrobot_interfaces.action import ApproachTag, Relocalize
from std_msgs.msg import String


class MissionState(Enum):
    WAITING_FOR_PATROL_POINTS = auto()
    INITIAL_TAG_APPROACH = auto()
    INITIAL_RELOCALIZATION = auto()
    STARTING_NAV2 = auto()
    GO_TO_PATROL = auto()
    PATROL_SCAN = auto()
    COMPLETE = auto()
    ERROR = auto()


class PatrolManager(Node):
    """Bare-bones patrol mission V1.

    Mission flow:
      patrol points
        -> initial tag approach
        -> one initial /relocalize
        -> start Nav2
        -> NavigateToPose(P0)
        -> Spin(360 deg)
        -> NavigateToPose(P1)
        -> ...
        -> COMPLETE

    There is deliberately NO runtime relocalization and NO shuttle handling in
    this version. The goal is to validate the patrol/Nav2 sequence using only
    the initial absolute localization fix.
    """

    def __init__(self):
        super().__init__('patrol_manager')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('autostart', True)

        # Initial absolute localization only.
        self.declare_parameter('initial_global_localization', True)
        self.declare_parameter('initial_tag_id', -1)
        self.declare_parameter('initial_relocalization_retries', 3)
        self.declare_parameter('tag_approach_distance', 1.70)
        self.declare_parameter('tag_approach_timeout', 60.0)
        self.declare_parameter('relocalize_sample_count', 15)
        self.declare_parameter('relocalize_timeout', 7.0)

        # Nav2 startup.
        self.declare_parameter(
            'nav2_lifecycle_service',
            '/lifecycle_manager_navigation/manage_nodes',
        )
        self.declare_parameter('nav2_startup_timeout', 30.0)
        self.declare_parameter('nav2_startup_retries', 3)
        self.declare_parameter('nav2_tf_settle_time', 0.75)
        self.declare_parameter('navigation_goal_retries', 3)

        # Patrol behavior.
        self.declare_parameter('action_retry_period', 0.5)
        self.declare_parameter('spin_angle', 2.0 * math.pi)
        self.declare_parameter('spin_time_allowance', 20.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.autostart = bool(self.get_parameter('autostart').value)

        self.initial_global_localization = bool(
            self.get_parameter('initial_global_localization').value
        )
        self.initial_tag_id = int(self.get_parameter('initial_tag_id').value)
        self.initial_relocalization_retries = int(
            self.get_parameter('initial_relocalization_retries').value
        )
        self.tag_approach_distance = float(
            self.get_parameter('tag_approach_distance').value
        )
        self.tag_approach_timeout = float(
            self.get_parameter('tag_approach_timeout').value
        )
        self.relocalize_sample_count = int(
            self.get_parameter('relocalize_sample_count').value
        )
        self.relocalize_timeout = float(
            self.get_parameter('relocalize_timeout').value
        )

        self.nav2_lifecycle_service = str(
            self.get_parameter('nav2_lifecycle_service').value
        )
        self.nav2_startup_timeout = float(
            self.get_parameter('nav2_startup_timeout').value
        )
        self.nav2_startup_retries = int(
            self.get_parameter('nav2_startup_retries').value
        )
        self.nav2_tf_settle_time = float(
            self.get_parameter('nav2_tf_settle_time').value
        )
        self.navigation_goal_retries = int(
            self.get_parameter('navigation_goal_retries').value
        )

        self.action_retry_period = float(
            self.get_parameter('action_retry_period').value
        )
        self.spin_angle = float(self.get_parameter('spin_angle').value)
        self.spin_time_allowance = float(
            self.get_parameter('spin_time_allowance').value
        )

        self.state = MissionState.WAITING_FOR_PATROL_POINTS
        self.patrol_points = []
        self.current_patrol_index = 0

        # Initial-localization action state.
        self.initial_retry_count = 0
        self.pending_approach = False
        self.pending_relocalize_tag = None
        self.approach_goal_handle = None
        self.relocalize_goal_handle = None

        # Nav2 lifecycle startup state.
        self.nav2_startup_pending = False
        self.nav2_startup_future = None
        self.nav2_startup_attempts = 0
        self.nav2_startup_begin_time = None
        self.navigation_allowed_time = None

        # Nav2 action state.
        self.pending_navigation = None
        self.pending_spin = False
        self.navigate_goal_handle = None
        self.spin_goal_handle = None
        self.active_navigation_pose = None
        self.navigation_retry_count = 0

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.patrol_points_sub = self.create_subscription(
            PoseArray,
            '/mission/patrol_points',
            self.patrol_points_callback,
            state_qos,
        )

        self.state_pub = self.create_publisher(
            String,
            '/mission/state',
            state_qos,
        )
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/mission/current_goal',
            state_qos,
        )

        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
        )
        self.spin_client = ActionClient(
            self,
            Spin,
            '/spin',
        )
        self.approach_client = ActionClient(
            self,
            ApproachTag,
            '/approach_tag',
        )
        self.relocalize_client = ActionClient(
            self,
            Relocalize,
            '/relocalize',
        )
        self.nav2_lifecycle_client = self.create_client(
            ManageLifecycleNodes,
            self.nav2_lifecycle_service,
        )

        # This timer is active only while waiting for an action/service/server.
        self.action_retry_timer = self.create_timer(
            self.action_retry_period,
            self.process_pending_actions,
        )
        self.action_retry_timer.cancel()

        self.publish_state()
        self.get_logger().info(
            'Patrol manager V1 started: initial localization once, then '
            'NavigateToPose + 360-degree scan through all patrol points.'
        )

    # ============================================================
    # State / outputs
    # ============================================================

    def set_state(self, new_state):
        if self.state != new_state:
            self.get_logger().info(f'{self.state.name} -> {new_state.name}')
        self.state = new_state
        self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    def publish_current_goal(self, pose):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose = pose
        self.goal_pub.publish(msg)

    # ============================================================
    # Patrol points input
    # ============================================================

    def patrol_points_callback(self, msg):
        if self.patrol_points:
            return

        if not msg.poses:
            self.get_logger().error('Received an empty patrol-point list.')
            self.enter_error('No patrol points available.')
            return

        self.frame_id = msg.header.frame_id or self.frame_id
        self.patrol_points = list(msg.poses)
        self.current_patrol_index = 0

        self.get_logger().info(
            f'Received {len(self.patrol_points)} patrol points in '
            f'frame {self.frame_id}.'
        )

        if not self.autostart:
            self.get_logger().info('Autostart is disabled; mission remains idle.')
            return

        if self.initial_global_localization:
            self.start_initial_global_localization()
        else:
            self.get_logger().warn(
                'Initial global localization is disabled; starting Nav2 patrol '
                'using the existing map -> odom transform.'
            )
            self.start_nav2_then_patrol()

    # ============================================================
    # Initial localization: ApproachTag -> Relocalize exactly once
    # ============================================================

    def start_initial_global_localization(self):
        self.get_logger().info('Starting initial global pose acquisition.')
        self.set_state(MissionState.INITIAL_TAG_APPROACH)
        self.pending_approach = True
        self.process_pending_actions()

    def send_approach_goal_now(self):
        self.pending_approach = False

        goal = ApproachTag.Goal()
        goal.preferred_tag_id = int(self.initial_tag_id)
        goal.target_distance = float(self.tag_approach_distance)
        goal.timeout_sec = float(self.tag_approach_timeout)

        self.get_logger().info(
            f'Sending initial ApproachTag goal: preferred_tag={self.initial_tag_id}, '
            f'target_distance={self.tag_approach_distance:.2f} m.'
        )

        future = self.approach_client.send_goal_async(goal)
        future.add_done_callback(self.approach_goal_response_callback)

    def approach_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.handle_initial_localization_failure(
                f'ApproachTag send failed: {exc}'
            )
            return

        if not goal_handle.accepted:
            self.handle_initial_localization_failure(
                'ApproachTag goal rejected.'
            )
            return

        self.approach_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            self.approach_result_callback
        )

    def approach_result_callback(self, future):
        try:
            wrapped = future.result()
        except Exception as exc:
            self.approach_goal_handle = None
            self.handle_initial_localization_failure(
                f'ApproachTag result failed: {exc}'
            )
            return

        self.approach_goal_handle = None

        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or not wrapped.result.success
        ):
            self.handle_initial_localization_failure(
                f'ApproachTag failed: {wrapped.result.message}'
            )
            return

        tag_id = int(wrapped.result.tag_id)
        self.get_logger().info(
            f'Initial observation pose reached using tag {tag_id}; '
            f'final_distance={wrapped.result.final_distance:.2f} m.'
        )

        self.set_state(MissionState.INITIAL_RELOCALIZATION)
        self.pending_relocalize_tag = tag_id
        self.process_pending_actions()

    def send_relocalize_goal_now(self, tag_id):
        self.pending_relocalize_tag = None

        goal = Relocalize.Goal()
        goal.preferred_tag_id = int(tag_id)
        goal.sample_count = int(self.relocalize_sample_count)
        goal.timeout_sec = float(self.relocalize_timeout)

        self.get_logger().info(
            f'Sending one initial Relocalize goal: tag={tag_id}, '
            f'samples={self.relocalize_sample_count}.'
        )

        future = self.relocalize_client.send_goal_async(goal)
        future.add_done_callback(self.relocalize_goal_response_callback)

    def relocalize_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.handle_initial_localization_failure(
                f'Relocalize send failed: {exc}'
            )
            return

        if not goal_handle.accepted:
            self.handle_initial_localization_failure(
                'Relocalize goal rejected.'
            )
            return

        self.relocalize_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            self.relocalize_result_callback
        )

    def relocalize_result_callback(self, future):
        try:
            wrapped = future.result()
        except Exception as exc:
            self.relocalize_goal_handle = None
            self.handle_initial_localization_failure(
                f'Relocalize result failed: {exc}'
            )
            return

        self.relocalize_goal_handle = None

        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or not wrapped.result.success
        ):
            self.handle_initial_localization_failure(
                f'Relocalize failed: {wrapped.result.message}'
            )
            return

        self.initial_retry_count = 0

        self.get_logger().info(
            f'Initial localization succeeded using tags '
            f'{list(wrapped.result.used_tag_ids)}; '
            f'std=({wrapped.result.std_x:.3f}, '
            f'{wrapped.result.std_y:.3f}, '
            f'{math.degrees(wrapped.result.std_yaw):.2f} deg).'
        )

        # This is the ONLY successful relocalization path in patrol V1.
        self.start_nav2_then_patrol()

    def handle_initial_localization_failure(self, reason):
        self.approach_goal_handle = None
        self.relocalize_goal_handle = None
        self.pending_approach = False
        self.pending_relocalize_tag = None

        self.initial_retry_count += 1

        if self.initial_retry_count <= self.initial_relocalization_retries:
            self.get_logger().warn(
                f'Initial localization failed: {reason} '
                f'Retry {self.initial_retry_count}/'
                f'{self.initial_relocalization_retries}.'
            )
            self.set_state(MissionState.INITIAL_TAG_APPROACH)
            self.pending_approach = True
            self.process_pending_actions()
            return

        self.enter_error(
            f'Initial localization failed permanently: {reason}'
        )

    # ============================================================
    # Nav2 lifecycle startup
    # ============================================================

    def start_nav2_then_patrol(self):
        self.navigation_allowed_time = (
            time.monotonic() + self.nav2_tf_settle_time
        )

        if (
            self.navigate_client.server_is_ready()
            and self.spin_client.server_is_ready()
        ):
            self.get_logger().info(
                'Nav2 is already active. Waiting briefly for the initial '
                'map -> odom transform to settle before the first goal.'
            )
            self.queue_current_patrol_goal()
            return

        self.nav2_startup_pending = True
        self.nav2_startup_future = None
        self.nav2_startup_attempts = 0
        self.nav2_startup_begin_time = time.monotonic()

        self.set_state(MissionState.STARTING_NAV2)
        self.get_logger().info(
            'Initial map -> odom is available. Starting Nav2 lifecycle.'
        )
        self.process_pending_actions()

    def process_nav2_startup(self):
        if not self.nav2_startup_pending:
            return

        if (
            self.navigate_client.server_is_ready()
            and self.spin_client.server_is_ready()
        ):
            self.nav2_startup_pending = False
            self.navigation_allowed_time = (
                time.monotonic() + self.nav2_tf_settle_time
            )
            self.get_logger().info(
                'Nav2 action servers are ready. Waiting for TF settle, then '
                'starting patrol.'
            )
            self.queue_current_patrol_goal()
            return

        if self.nav2_startup_begin_time is not None:
            elapsed = time.monotonic() - self.nav2_startup_begin_time
            if elapsed > self.nav2_startup_timeout:
                self.nav2_startup_pending = False
                self.enter_error(
                    f'Nav2 startup timed out after {elapsed:.1f} s.'
                )
                return

        if self.nav2_startup_future is not None:
            return

        if not self.nav2_lifecycle_client.service_is_ready():
            return

        if self.nav2_startup_attempts >= self.nav2_startup_retries:
            self.nav2_startup_pending = False
            self.enter_error('Nav2 lifecycle startup retries exhausted.')
            return

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request().STARTUP

        self.nav2_startup_attempts += 1
        self.get_logger().info(
            f'Requesting Nav2 lifecycle STARTUP '
            f'({self.nav2_startup_attempts}/{self.nav2_startup_retries}).'
        )

        self.nav2_startup_future = self.nav2_lifecycle_client.call_async(
            request
        )
        self.nav2_startup_future.add_done_callback(
            self.nav2_startup_response_callback
        )

    def nav2_startup_response_callback(self, future):
        self.nav2_startup_future = None

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'Nav2 lifecycle STARTUP service failed: {exc}'
            )
            self.update_action_retry_timer()
            return

        if response is None:
            self.get_logger().error(
                'Nav2 lifecycle STARTUP returned no response.'
            )
            self.update_action_retry_timer()
            return

        success = bool(getattr(response, 'success', True))
        if not success:
            self.get_logger().warn(
                'Nav2 lifecycle manager reported STARTUP failure; retrying.'
            )

        self.update_action_retry_timer()

    # ============================================================
    # Pending asynchronous work
    # ============================================================

    def has_pending_work(self):
        return (
            self.nav2_startup_pending
            or self.pending_approach
            or self.pending_relocalize_tag is not None
            or self.pending_navigation is not None
            or self.pending_spin
        )

    def update_action_retry_timer(self):
        if self.has_pending_work():
            self.action_retry_timer.reset()
        else:
            self.action_retry_timer.cancel()

    def process_pending_actions(self):
        if self.state in [MissionState.ERROR, MissionState.COMPLETE]:
            self.action_retry_timer.cancel()
            return

        self.process_nav2_startup()

        if self.pending_approach and self.approach_goal_handle is None:
            if self.approach_client.server_is_ready():
                self.send_approach_goal_now()

        if (
            self.pending_relocalize_tag is not None
            and self.relocalize_goal_handle is None
        ):
            if self.relocalize_client.server_is_ready():
                tag_id = self.pending_relocalize_tag
                self.send_relocalize_goal_now(tag_id)

        if self.pending_navigation is not None and self.navigate_goal_handle is None:
            nav_time_ready = (
                self.navigation_allowed_time is None
                or time.monotonic() >= self.navigation_allowed_time
            )
            if self.navigate_client.server_is_ready() and nav_time_ready:
                pose = self.pending_navigation
                self.pending_navigation = None
                self.send_navigation_goal_now(pose)

        if self.pending_spin and self.spin_goal_handle is None:
            if self.spin_client.server_is_ready():
                self.pending_spin = False
                self.send_spin_goal_now()

        self.update_action_retry_timer()

    # ============================================================
    # Patrol navigation
    # ============================================================

    def queue_current_patrol_goal(self):
        if self.current_patrol_index >= len(self.patrol_points):
            self.finish_mission()
            return

        pose = self.patrol_points[self.current_patrol_index]
        self.pending_navigation = pose
        self.process_pending_actions()

    def send_navigation_goal_now(self, pose):
        self.active_navigation_pose = pose
        self.set_state(MissionState.GO_TO_PATROL)

        self.get_logger().info(
            f'Going to patrol point P{self.current_patrol_index}/'
            f'{len(self.patrol_points) - 1}.'
        )
        self.publish_current_goal(pose)

        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.pose = pose

        future = self.navigate_client.send_goal_async(goal)
        future.add_done_callback(self.navigation_goal_response_callback)

    def navigation_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.enter_error(f'NavigateToPose send failed: {exc}')
            return

        if not goal_handle.accepted:
            self.navigation_retry_count += 1

            if (
                self.active_navigation_pose is not None
                and self.navigation_retry_count <= self.navigation_goal_retries
            ):
                self.get_logger().warn(
                    f'NavigateToPose goal rejected; retrying '
                    f'{self.navigation_retry_count}/'
                    f'{self.navigation_goal_retries}.'
                )
                self.pending_navigation = self.active_navigation_pose
                self.update_action_retry_timer()
                return

            self.enter_error(
                'NavigateToPose goal rejected after all retries.'
            )
            return

        self.navigate_goal_handle = goal_handle
        self.navigation_retry_count = 0
        goal_handle.get_result_async().add_done_callback(
            self.navigation_result_callback
        )

    def navigation_result_callback(self, future):
        try:
            wrapped = future.result()
        except Exception as exc:
            self.navigate_goal_handle = None
            self.enter_error(f'NavigateToPose result failed: {exc}')
            return

        self.navigate_goal_handle = None
        self.active_navigation_pose = None

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.enter_error(
                f'Navigation to P{self.current_patrol_index} failed with '
                f'status {wrapped.status}.'
            )
            return

        self.get_logger().info(
            f'Reached patrol point P{self.current_patrol_index}.'
        )
        self.start_patrol_scan()

    # ============================================================
    # 360-degree scan at each patrol point
    # ============================================================

    def start_patrol_scan(self):
        self.set_state(MissionState.PATROL_SCAN)
        self.pending_spin = True
        self.process_pending_actions()

    def send_spin_goal_now(self):
        goal = Spin.Goal()
        goal.target_yaw = float(self.spin_angle)

        sec = int(self.spin_time_allowance)
        goal.time_allowance.sec = sec
        goal.time_allowance.nanosec = int(
            (self.spin_time_allowance - sec) * 1e9
        )

        self.get_logger().info(
            f'Scanning at P{self.current_patrol_index}: '
            f'{math.degrees(self.spin_angle):.1f} deg spin.'
        )

        future = self.spin_client.send_goal_async(goal)
        future.add_done_callback(self.spin_goal_response_callback)

    def spin_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.enter_error(f'Spin send failed: {exc}')
            return

        if not goal_handle.accepted:
            self.enter_error('Spin goal rejected.')
            return

        self.spin_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            self.spin_result_callback
        )

    def spin_result_callback(self, future):
        try:
            wrapped = future.result()
        except Exception as exc:
            self.spin_goal_handle = None
            self.enter_error(f'Spin result failed: {exc}')
            return

        self.spin_goal_handle = None

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.enter_error(
                f'Spin at P{self.current_patrol_index} failed with '
                f'status {wrapped.status}.'
            )
            return

        self.get_logger().info(
            f'Completed scan at P{self.current_patrol_index}.'
        )
        self.current_patrol_index += 1

        if self.current_patrol_index >= len(self.patrol_points):
            self.finish_mission()
        else:
            self.queue_current_patrol_goal()

    # ============================================================
    # Terminal states
    # ============================================================

    def finish_mission(self):
        self.action_retry_timer.cancel()
        self.pending_navigation = None
        self.pending_spin = False
        self.pending_approach = False
        self.pending_relocalize_tag = None

        self.set_state(MissionState.COMPLETE)
        self.get_logger().info(
            f'Patrol V1 complete: visited and scanned all '
            f'{len(self.patrol_points)} patrol points using only the initial '
            'global localization.'
        )

    def enter_error(self, reason):
        self.get_logger().error(reason)

        self.action_retry_timer.cancel()
        self.nav2_startup_pending = False
        self.pending_navigation = None
        self.pending_spin = False
        self.pending_approach = False
        self.pending_relocalize_tag = None

        if self.navigate_goal_handle is not None:
            self.navigate_goal_handle.cancel_goal_async()
            self.navigate_goal_handle = None
        if self.spin_goal_handle is not None:
            self.spin_goal_handle.cancel_goal_async()
            self.spin_goal_handle = None
        if self.approach_goal_handle is not None:
            self.approach_goal_handle.cancel_goal_async()
            self.approach_goal_handle = None
        if self.relocalize_goal_handle is not None:
            self.relocalize_goal_handle.cancel_goal_async()
            self.relocalize_goal_handle = None

        self.set_state(MissionState.ERROR)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
