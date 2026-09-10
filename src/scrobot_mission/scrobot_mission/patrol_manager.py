#!/usr/bin/env python3

import math
import time
from enum import Enum, auto

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scrobot_interfaces.action import ApproachTag, Relocalize
from std_msgs.msg import String

from scrobot_mission.patrol_points import (
    find_minimum_grid,
    generate_patrol_points,
)


class MissionState(Enum):
    IDLE = auto()
    INITIAL_TAG_APPROACH = auto()
    INITIAL_RELOCALIZATION = auto()
    STARTING_NAV2 = auto()
    GO_TO_PATROL = auto()
    PATROL_SCAN = auto()
    COMPLETE = auto()
    ERROR = auto()


class PatrolManager(Node):
    """Bare-bones patrol mission.

    Flow:
      generate patrol points
        -> initial tag approach
        -> one initial relocalization
        -> start Nav2
        -> P0 -> 360 deg spin -> P1 -> ... -> COMPLETE

    Runtime relocalization and shuttle handling are intentionally excluded.
    """

    def __init__(self):
        super().__init__('patrol_manager')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('autostart', True)

        # Patrol grid.
        self.declare_parameter('court_length', 13.40)
        self.declare_parameter('court_width', 6.10)
        self.declare_parameter('camera_range', 3.0)
        self.declare_parameter('range_factor', 0.90)
        self.declare_parameter('max_grid_size', 20)

        # Initial localization only.
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

        self.court_length = float(self.get_parameter('court_length').value)
        self.court_width = float(self.get_parameter('court_width').value)
        self.camera_range = float(self.get_parameter('camera_range').value)
        self.range_factor = float(self.get_parameter('range_factor').value)
        self.max_grid_size = int(self.get_parameter('max_grid_size').value)

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

        if self.court_length <= 0.0 or self.court_width <= 0.0:
            raise ValueError('Court dimensions must be > 0.')
        if self.camera_range <= 0.0 or self.range_factor <= 0.0:
            raise ValueError('camera_range and range_factor must be > 0.')

        self.state = MissionState.IDLE
        self.patrol_points = []
        self.current_patrol_index = 0

        # Initial localization action state.
        self.initial_retry_count = 0
        self.pending_approach = False
        self.pending_relocalize_tag = None
        self.approach_goal_handle = None
        self.relocalize_goal_handle = None

        # Nav2 lifecycle state.
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
        self.patrol_points_pub = self.create_publisher(
            PoseArray,
            '/mission/patrol_points',
            state_qos,
        )

        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
        )
        self.spin_client = ActionClient(self, Spin, '/spin')
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

        # Only active while waiting for servers, retry delays or TF-settle delay.
        self.action_retry_timer = self.create_timer(
            self.action_retry_period,
            self.process_pending_actions,
        )
        self.action_retry_timer.cancel()

        # One-shot deferred mission start. Avoid dispatching actions from __init__.
        self.start_timer = self.create_timer(0.10, self.start_once)

        self.generate_and_publish_patrol_points()
        self.publish_state()

        self.get_logger().info(
            'Patrol manager V1-refactor started: integrated patrol-point '
            'generation, initial localization once, event-driven Nav2 patrol.'
        )

    # ============================================================
    # Patrol point generation / outputs
    # ============================================================

    def generate_and_publish_patrol_points(self):
        effective_range = self.camera_range * self.range_factor
        nx, ny, dx, dy, worst = find_minimum_grid(
            self.court_length,
            self.court_width,
            effective_range,
            self.max_grid_size,
        )
        xyz_yaw = generate_patrol_points(
            self.court_length,
            self.court_width,
            nx,
            ny,
        )

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        for x, y, yaw in xyz_yaw:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(pose)

        self.patrol_points = list(msg.poses)
        self.patrol_points_pub.publish(msg)

        self.get_logger().info(
            f'Generated patrol grid {nx}x{ny}: {len(self.patrol_points)} points; '
            f'cell={dx:.2f}x{dy:.2f} m, worst-case range={worst:.2f} m, '
            f'effective range={effective_range:.2f} m.'
        )

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

    def start_once(self):
        self.start_timer.cancel()

        if not self.autostart:
            self.get_logger().info('Autostart disabled; patrol remains IDLE.')
            return

        if not self.patrol_points:
            self.enter_error('No patrol points were generated.')
            return

        self.current_patrol_index = 0
        if self.initial_global_localization:
            self.start_initial_global_localization()
        else:
            self.get_logger().warn(
                'Initial localization disabled; using existing map -> odom.'
            )
            self.start_nav2_then_patrol()

    # ============================================================
    # Initial localization
    # ============================================================

    def start_initial_global_localization(self):
        self.set_state(MissionState.INITIAL_TAG_APPROACH)
        self.pending_approach = True
        self.get_logger().info('Starting initial global pose acquisition.')
        self.process_pending_actions()

    def send_approach_goal_now(self):
        self.pending_approach = False

        goal = ApproachTag.Goal()
        goal.preferred_tag_id = int(self.initial_tag_id)
        goal.target_distance = float(self.tag_approach_distance)
        goal.timeout_sec = float(self.tag_approach_timeout)

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
            self.handle_initial_localization_failure('ApproachTag rejected.')
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
            f'distance={wrapped.result.final_distance:.2f} m.'
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
            self.handle_initial_localization_failure('Relocalize rejected.')
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
            f'Initial localization succeeded; tags='
            f'{list(wrapped.result.used_tag_ids)}, '
            f'std=({wrapped.result.std_x:.3f}, {wrapped.result.std_y:.3f}, '
            f'{math.degrees(wrapped.result.std_yaw):.2f} deg).'
        )
        self.start_nav2_then_patrol()

    def handle_initial_localization_failure(self, reason):
        self.approach_goal_handle = None
        self.relocalize_goal_handle = None
        self.pending_approach = False
        self.pending_relocalize_tag = None
        self.initial_retry_count += 1

        if self.initial_retry_count <= self.initial_relocalization_retries:
            self.get_logger().warn(
                f'Initial localization failed: {reason} Retry '
                f'{self.initial_retry_count}/{self.initial_relocalization_retries}.'
            )
            self.set_state(MissionState.INITIAL_TAG_APPROACH)
            self.pending_approach = True
            self.process_pending_actions()
            return

        self.enter_error(f'Initial localization failed permanently: {reason}')

    # ============================================================
    # Nav2 startup / event-driven pending work
    # ============================================================

    def start_nav2_then_patrol(self):
        self.navigation_allowed_time = time.monotonic() + self.nav2_tf_settle_time

        if (
            self.navigate_client.server_is_ready()
            and self.spin_client.server_is_ready()
        ):
            self.queue_current_patrol_goal()
            return

        self.nav2_startup_pending = True
        self.nav2_startup_future = None
        self.nav2_startup_attempts = 0
        self.nav2_startup_begin_time = time.monotonic()
        self.set_state(MissionState.STARTING_NAV2)
        self.process_pending_actions()

    def process_nav2_startup(self):
        if not self.nav2_startup_pending:
            return

        if (
            self.navigate_client.server_is_ready()
            and self.spin_client.server_is_ready()
        ):
            self.nav2_startup_pending = False
            self.navigation_allowed_time = time.monotonic() + self.nav2_tf_settle_time
            self.queue_current_patrol_goal()
            return

        elapsed = time.monotonic() - self.nav2_startup_begin_time
        if elapsed > self.nav2_startup_timeout:
            self.nav2_startup_pending = False
            self.enter_error(f'Nav2 startup timed out after {elapsed:.1f} s.')
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
        self.nav2_startup_future = self.nav2_lifecycle_client.call_async(request)
        self.nav2_startup_future.add_done_callback(
            self.nav2_startup_response_callback
        )

    def nav2_startup_response_callback(self, future):
        self.nav2_startup_future = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Nav2 startup service failed: {exc}')
            self.update_action_retry_timer()
            return

        if response is None or not bool(getattr(response, 'success', True)):
            self.get_logger().warn('Nav2 lifecycle STARTUP not yet successful.')
        self.update_action_retry_timer()

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
            and self.relocalize_client.server_is_ready()
        ):
            self.send_relocalize_goal_now(self.pending_relocalize_tag)

        if self.pending_navigation is not None and self.navigate_goal_handle is None:
            time_ready = (
                self.navigation_allowed_time is None
                or time.monotonic() >= self.navigation_allowed_time
            )
            if self.navigate_client.server_is_ready() and time_ready:
                pose = self.pending_navigation
                self.pending_navigation = None
                self.send_navigation_goal_now(pose)

        if self.pending_spin and self.spin_goal_handle is None:
            if self.spin_client.server_is_ready():
                self.pending_spin = False
                self.send_spin_goal_now()

        self.update_action_retry_timer()

    # ============================================================
    # Patrol navigation / scanning
    # ============================================================

    def queue_current_patrol_goal(self):
        if self.current_patrol_index >= len(self.patrol_points):
            self.finish_mission()
            return
        self.pending_navigation = self.patrol_points[self.current_patrol_index]
        self.process_pending_actions()

    def send_navigation_goal_now(self, pose):
        self.active_navigation_pose = pose
        self.publish_current_goal(pose)
        self.set_state(MissionState.GO_TO_PATROL)

        self.get_logger().info(
            f'Going to P{self.current_patrol_index}/'
            f'{len(self.patrol_points) - 1}.'
        )

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
                    f'NavigateToPose rejected; retry '
                    f'{self.navigation_retry_count}/{self.navigation_goal_retries}.'
                )
                self.pending_navigation = self.active_navigation_pose
                self.update_action_retry_timer()
                return
            self.enter_error('NavigateToPose rejected after all retries.')
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

        self.get_logger().info(f'Reached P{self.current_patrol_index}.')
        self.start_patrol_scan()

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
            f'Scanning P{self.current_patrol_index}: '
            f'{math.degrees(self.spin_angle):.1f} deg.'
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

        self.get_logger().info(f'Completed scan at P{self.current_patrol_index}.')
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
            f'Patrol complete: {len(self.patrol_points)} points visited/scanned.'
        )

    def enter_error(self, reason):
        self.get_logger().error(reason)
        self.action_retry_timer.cancel()
        self.nav2_startup_pending = False
        self.pending_navigation = None
        self.pending_spin = False
        self.pending_approach = False
        self.pending_relocalize_tag = None

        for handle in [
            self.navigate_goal_handle,
            self.spin_goal_handle,
            self.approach_goal_handle,
            self.relocalize_goal_handle,
        ]:
            if handle is not None:
                handle.cancel_goal_async()

        self.navigate_goal_handle = None
        self.spin_goal_handle = None
        self.approach_goal_handle = None
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
