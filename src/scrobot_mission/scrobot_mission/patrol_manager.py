#!/usr/bin/env python3

import math
import time
from enum import Enum, auto

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scrobot_interfaces.action import ApproachTag, Relocalize
from std_msgs.msg import Bool, Int32MultiArray, String

from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class MissionState(Enum):
    WAITING_FOR_PATROL_POINTS = auto()
    INITIAL_TAG_APPROACH = auto()
    INITIAL_RELOCALIZATION = auto()
    STARTING_NAV2 = auto()
    GO_TO_PATROL = auto()
    PATROL_SCAN = auto()
    GO_TO_TARGET = auto()
    COLLECT_TARGET = auto()
    LOCAL_LOOK = auto()
    LOCAL_SPIN = auto()
    RETURN_TO_PATROL = auto()
    VERIFY_PATROL = auto()
    RELOCALIZATION_APPROACH = auto()
    RELOCALIZATION_ACQUIRE = auto()
    COMPLETE = auto()
    ERROR = auto()


class PatrolManager(Node):
    def __init__(self):
        super().__init__('patrol_manager')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('autostart', True)
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('active_tags_topic', '/global_localization/active_tags')

        self.declare_parameter('initial_global_localization', True)
        self.declare_parameter('initial_tag_id', -1)
        self.declare_parameter('initial_relocalization_retries', 3)

        self.declare_parameter('tag_approach_distance', 1.70)
        self.declare_parameter('tag_approach_timeout', 45.0)
        self.declare_parameter('relocalize_sample_count', 15)
        self.declare_parameter('relocalize_timeout', 7.0)

        self.declare_parameter('nav2_lifecycle_service', '/lifecycle_manager_navigation/manage_nodes')
        self.declare_parameter('nav2_startup_timeout', 30.0)
        self.declare_parameter('nav2_startup_retries', 3)

        self.declare_parameter('soft_relocalization_distance', 5.0)
        self.declare_parameter('hard_relocalization_distance', 9.0)
        self.declare_parameter('hard_relocalization_retries', 2)

        self.declare_parameter('local_fov_wait_time', 0.5)
        self.declare_parameter('action_retry_period', 0.5)
        self.declare_parameter('spin_angle', 2.0 * math.pi)
        self.declare_parameter('spin_time_allowance', 20.0)
        self.declare_parameter('max_odom_step', 1.0)

        self.declare_parameter('nav2_tf_settle_time', 0.75)
        self.declare_parameter('navigation_goal_retries', 3)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.autostart = bool(self.get_parameter('autostart').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.active_tags_topic = str(self.get_parameter('active_tags_topic').value)

        self.initial_global_localization = bool(self.get_parameter('initial_global_localization').value)
        self.initial_tag_id = int(self.get_parameter('initial_tag_id').value)
        self.initial_relocalization_retries = int(self.get_parameter('initial_relocalization_retries').value)

        self.tag_approach_distance = float(self.get_parameter('tag_approach_distance').value)
        self.tag_approach_timeout = float(self.get_parameter('tag_approach_timeout').value)
        self.relocalize_sample_count = int(self.get_parameter('relocalize_sample_count').value)
        self.relocalize_timeout = float(self.get_parameter('relocalize_timeout').value)

        self.nav2_lifecycle_service = str(self.get_parameter('nav2_lifecycle_service').value)
        self.nav2_startup_timeout = float(self.get_parameter('nav2_startup_timeout').value)
        self.nav2_startup_retries = int(self.get_parameter('nav2_startup_retries').value)

        self.soft_relocalization_distance = float(self.get_parameter('soft_relocalization_distance').value)
        self.hard_relocalization_distance = float(self.get_parameter('hard_relocalization_distance').value)
        self.hard_relocalization_retries = int(self.get_parameter('hard_relocalization_retries').value)

        self.local_fov_wait_time = float(self.get_parameter('local_fov_wait_time').value)
        self.action_retry_period = float(self.get_parameter('action_retry_period').value)
        self.spin_angle = float(self.get_parameter('spin_angle').value)
        self.spin_time_allowance = float(self.get_parameter('spin_time_allowance').value)
        self.max_odom_step = float(self.get_parameter('max_odom_step').value)

        self.nav2_tf_settle_time = float(self.get_parameter('nav2_tf_settle_time').value)
        self.navigation_goal_retries = int(self.get_parameter('navigation_goal_retries').value)

        if self.soft_relocalization_distance <= 0.0:
            raise ValueError('soft_relocalization_distance must be > 0.')
        if self.hard_relocalization_distance <= self.soft_relocalization_distance:
            raise ValueError('hard_relocalization_distance must be greater than soft_relocalization_distance.')

        self.state = MissionState.WAITING_FOR_PATROL_POINTS
        self.patrol_points = []
        self.current_patrol_index = 0
        self.current_target = None
        self.local_look_start_time = None

        self.initial_localization_complete = False
        self.nav2_ready = False
        self.nav2_startup_pending = False
        self.nav2_startup_future = None
        self.nav2_startup_attempts = 0
        self.nav2_startup_begin_time = None

        self.distance_since_relocalization = 0.0
        self.last_odom_xy = None
        self.tags_seen_this_checkpoint = set()

        self.navigation_purpose = None
        self.pending_navigation = None
        self.pending_spin = False
        self.pending_approach = None
        self.pending_relocalize = None

        self.navigate_goal_handle = None
        self.spin_goal_handle = None
        self.approach_goal_handle = None
        self.relocalize_goal_handle = None

        self.approach_context = None
        self.relocalize_context = None
        self.runtime_relocalization_mode = None
        self.initial_retry_count = 0
        self.hard_retry_count = 0

        self.nav2_tf_ready_since = None
        self.active_navigation_request = None
        self.navigation_retry_count = 0

        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        event_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        self.patrol_points_sub = self.create_subscription(PoseArray, '/mission/patrol_points', self.patrol_points_callback, state_qos)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.active_tags_sub = self.create_subscription(Int32MultiArray, self.active_tags_topic, self.active_tags_callback, event_qos)

        # Temporary until shuttle perception and pickup feedback are connected.
        self.target_detected_sub = self.create_subscription(PoseStamped, '/mission/test/target_detected', self.target_detected_callback, event_qos)
        self.target_collected_sub = self.create_subscription(Bool, '/mission/test/target_collected', self.target_collected_callback, event_qos)

        self.state_pub = self.create_publisher(String, '/mission/state', state_qos)
        self.goal_pub = self.create_publisher(PoseStamped, '/mission/current_goal', state_qos)
        self.collect_request_pub = self.create_publisher(Bool, '/mission/collect_request', event_qos)

        self.navigate_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, '/spin')
        self.approach_client = ActionClient(self, ApproachTag, '/approach_tag')
        self.relocalize_client = ActionClient(self, Relocalize, '/relocalize')
        self.nav2_lifecycle_client = self.create_client(ManageLifecycleNodes, self.nav2_lifecycle_service)

        self.update_timer = self.create_timer(0.05, self.update)
        self.action_retry_timer = self.create_timer(self.action_retry_period, self.process_pending_actions)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publish_state()
        self.get_logger().info('Patrol manager started.')

    def set_state(self, new_state):
        if self.state != new_state:
            self.get_logger().info(f'{self.state.name} -> {new_state.name}')
        self.state = new_state
        self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    def publish_collect_request(self, enable):
        msg = Bool()
        msg.data = bool(enable)
        self.collect_request_pub.publish(msg)

    def publish_current_goal(self, pose):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose = pose
        self.goal_pub.publish(msg)

    # ============================================================
    # Mission inputs
    # ============================================================

    def patrol_points_callback(self, msg):
        if self.patrol_points:
            return

        self.frame_id = msg.header.frame_id
        self.patrol_points = list(msg.poses)
        self.get_logger().info(f'Received {len(self.patrol_points)} patrol points.')

        if not self.autostart or not self.patrol_points:
            return

        self.current_patrol_index = 0

        if self.initial_global_localization:
            self.start_initial_global_localization()
        else:
            self.get_logger().warn('Initial global localization is disabled.')
            self.initial_localization_complete = True
            self.start_nav2_then_patrol()

    def odom_callback(self, msg):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)

        if self.last_odom_xy is None:
            self.last_odom_xy = (x, y)
            return

        dx = x - self.last_odom_xy[0]
        dy = y - self.last_odom_xy[1]
        step = math.hypot(dx, dy)
        self.last_odom_xy = (x, y)

        if step <= self.max_odom_step:
            self.distance_since_relocalization += step

    def active_tags_callback(self, msg):
        if self.state not in [MissionState.PATROL_SCAN, MissionState.VERIFY_PATROL]:
            return

        for tag_id in msg.data:
            self.tags_seen_this_checkpoint.add(int(tag_id))

    # ============================================================
    # Initial localization
    # ============================================================

    def start_initial_global_localization(self):
        self.get_logger().info('Starting initial global pose acquisition.')
        self.queue_approach('initial', self.initial_tag_id)

    # ============================================================
    # Nav2 lifecycle startup
    # ============================================================

    def start_nav2_then_patrol(self):
        if self.navigate_client.server_is_ready() and self.spin_client.server_is_ready():
            self.nav2_ready = True
            self.get_logger().info('Nav2 is already active.')
            self.send_current_patrol_goal()
            return

        self.nav2_ready = False
        self.nav2_startup_pending = True
        self.nav2_startup_future = None
        self.nav2_startup_attempts = 0
        self.nav2_startup_begin_time = time.monotonic()
        self.set_state(MissionState.STARTING_NAV2)
        self.get_logger().info('Initial map->odom is ready. Starting Nav2 lifecycle.')
        self.process_nav2_startup()

    def process_nav2_startup(self):
        if not self.nav2_startup_pending:
            return

        if self.navigate_client.server_is_ready() and self.spin_client.server_is_ready():
            self.nav2_startup_pending = False
            self.nav2_ready = True
            self.get_logger().info('Nav2 action servers are ready.')
            self.send_current_patrol_goal()
            return

        if self.nav2_startup_begin_time is not None:
            elapsed = time.monotonic() - self.nav2_startup_begin_time
            if elapsed > self.nav2_startup_timeout:
                self.nav2_startup_pending = False
                self.get_logger().error(
                    f'Nav2 startup timed out after {elapsed:.1f} s. '
                    f'Check {self.nav2_lifecycle_service} and Nav2 lifecycle states.'
                )
                self.set_state(MissionState.ERROR)
                return

        if self.nav2_startup_future is not None:
            return

        if not self.nav2_lifecycle_client.service_is_ready():
            return

        if self.nav2_startup_attempts >= self.nav2_startup_retries:
            self.nav2_startup_pending = False
            self.get_logger().error('Nav2 lifecycle startup retries exhausted.')
            self.set_state(MissionState.ERROR)
            return

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request().STARTUP

        self.nav2_startup_attempts += 1
        self.get_logger().info(
            f'Requesting Nav2 lifecycle STARTUP '
            f'({self.nav2_startup_attempts}/{self.nav2_startup_retries}).'
        )

        self.nav2_startup_future = self.nav2_lifecycle_client.call_async(request)
        self.nav2_startup_future.add_done_callback(self.nav2_startup_response_callback)

    def nav2_startup_response_callback(self, future):
        self.nav2_startup_future = None

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Nav2 lifecycle STARTUP service failed: {exc}')
            return

        if response is None:
            self.get_logger().error('Nav2 lifecycle STARTUP returned no response.')
            return

        success = bool(getattr(response, 'success', True))
        if not success:
            self.get_logger().warn('Nav2 lifecycle manager reported STARTUP failure; retrying.')
            return

        self.get_logger().info('Nav2 lifecycle STARTUP accepted; waiting for action servers.')

    # ============================================================
    # Pending action dispatch
    # ============================================================

    def process_pending_actions(self):
        self.process_nav2_startup()

        if self.pending_navigation is not None and self.navigate_goal_handle is None:
            if self.navigate_client.server_is_ready() and self.navigation_tf_ready():
                pose, purpose = self.pending_navigation
                self.pending_navigation = None
                self.send_navigation_goal_now(pose, purpose)
            elif self.navigate_client.server_is_ready():
                self.get_logger().info('NavigateToPose ready; waiting for stable map -> base_footprint TF.', throttle_duration_sec=2.0)

        if self.pending_spin and self.spin_goal_handle is None:
            if self.spin_client.server_is_ready():
                self.pending_spin = False
                self.send_spin_goal_now()

        if self.pending_approach is not None and self.approach_goal_handle is None:
            if self.approach_client.server_is_ready():
                context, preferred_tag = self.pending_approach
                self.pending_approach = None
                self.send_approach_goal_now(context, preferred_tag)

        if self.pending_relocalize is not None and self.relocalize_goal_handle is None:
            if self.relocalize_client.server_is_ready():
                context, preferred_tag = self.pending_relocalize
                self.pending_relocalize = None
                self.send_relocalize_goal_now(context, preferred_tag)

    # ============================================================
    # Nav2 NavigateToPose
    # ============================================================

    def queue_navigation_goal(self, pose, purpose):
        self.pending_navigation = (pose, purpose)
        self.process_pending_actions()

    def send_navigation_goal_now(self, pose, purpose):
        self.navigation_purpose = purpose
        self.active_navigation_request = (pose, purpose)

        if purpose == 'patrol':
            self.set_state(MissionState.GO_TO_PATROL)
        elif purpose == 'return_patrol':
            self.set_state(MissionState.RETURN_TO_PATROL)
        elif purpose == 'target':
            self.set_state(MissionState.GO_TO_TARGET)

        self.publish_current_goal(pose)

        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.pose = pose

        future = self.navigate_client.send_goal_async(goal)
        future.add_done_callback(self.navigation_goal_response_callback)

    def navigation_tf_ready(self):
        ready = self.tf_buffer.can_transform(self.frame_id, 'base_footprint', Time(), timeout=Duration(seconds=0.0))

        if not ready:
            self.nav2_tf_ready_since = None
            return False

        if self.nav2_tf_ready_since is None:
            self.nav2_tf_ready_since = time.monotonic()
            return False

        return time.monotonic() - self.nav2_tf_ready_since >= self.nav2_tf_settle_time

    def navigation_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'NavigateToPose send failed: {exc}')
            self.set_state(MissionState.ERROR)
            return

        if not goal_handle.accepted:
            self.navigation_retry_count += 1

            if self.active_navigation_request is not None and self.navigation_retry_count <= self.navigation_goal_retries:
                pose, purpose = self.active_navigation_request

                self.get_logger().warn(
                    f'NavigateToPose goal rejected; retrying '
                    f'{self.navigation_retry_count}/{self.navigation_goal_retries}.'
                )

                self.nav2_tf_ready_since = None
                self.pending_navigation = (pose, purpose)
                return

            self.get_logger().error('NavigateToPose goal rejected after all retries.')
            self.set_state(MissionState.ERROR)
            return

        self.navigate_goal_handle = goal_handle
        self.navigation_retry_count = 0
        goal_handle.get_result_async().add_done_callback(self.navigation_result_callback)
        self.active_navigation_request = None

    def navigation_result_callback(self, future):
        try:
            wrapped = future.result()
        except Exception as exc:
            self.navigate_goal_handle = None
            self.get_logger().error(f'NavigateToPose result failed: {exc}')
            self.set_state(MissionState.ERROR)
            return

        status = wrapped.status
        purpose = self.navigation_purpose
        self.navigate_goal_handle = None
        self.navigation_purpose = None

        if status == GoalStatus.STATUS_CANCELED:
            return

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Navigation failed with status {status}.')
            self.set_state(MissionState.ERROR)
            return

        if purpose == 'patrol':
            self.start_checkpoint_scan(MissionState.PATROL_SCAN)
        elif purpose == 'return_patrol':
            self.start_checkpoint_scan(MissionState.VERIFY_PATROL)
        elif purpose == 'target':
            self.set_state(MissionState.COLLECT_TARGET)
            self.publish_collect_request(True)

    def send_current_patrol_goal(self):
        if self.current_patrol_index >= len(self.patrol_points):
            self.finish_mission()
            return

        pose = self.patrol_points[self.current_patrol_index]
        self.get_logger().info(f'Going to patrol point P{self.current_patrol_index}.')
        self.queue_navigation_goal(pose, 'patrol')

    def send_target_goal(self):
        if self.current_target is None:
            return

        self.get_logger().info('Going to detected shuttle.')
        self.queue_navigation_goal(self.current_target.pose, 'target')

    def return_to_patrol(self):
        pose = self.patrol_points[self.current_patrol_index]
        self.get_logger().info(f'Returning to patrol point P{self.current_patrol_index}.')
        self.queue_navigation_goal(pose, 'return_patrol')

    # ============================================================
    # Nav2 Spin
    # ============================================================

    def start_checkpoint_scan(self, scan_state):
        self.tags_seen_this_checkpoint.clear()
        self.set_state(scan_state)
        self.start_spin()

    def start_spin(self):
        self.pending_spin = True
        self.process_pending_actions()

    def send_spin_goal_now(self):
        goal = Spin.Goal()
        goal.target_yaw = self.spin_angle
        sec = int(self.spin_time_allowance)
        goal.time_allowance.sec = sec
        goal.time_allowance.nanosec = int((self.spin_time_allowance - sec) * 1e9)

        future = self.spin_client.send_goal_async(goal)
        future.add_done_callback(self.spin_goal_response_callback)

    def spin_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Spin send failed: {exc}')
            self.set_state(MissionState.ERROR)
            return

        if not goal_handle.accepted:
            self.get_logger().error('Spin goal rejected.')
            self.set_state(MissionState.ERROR)
            return

        self.spin_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.spin_result_callback)

    def spin_result_callback(self, future):
        try:
            wrapped = future.result()
        except Exception as exc:
            self.spin_goal_handle = None
            self.get_logger().error(f'Spin result failed: {exc}')
            self.set_state(MissionState.ERROR)
            return

        status = wrapped.status
        self.spin_goal_handle = None

        if status == GoalStatus.STATUS_CANCELED:
            return

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Spin failed with status {status}.')
            self.set_state(MissionState.ERROR)
            return

        if self.state in [MissionState.PATROL_SCAN, MissionState.VERIFY_PATROL]:
            self.handle_completed_patrol_scan()
        elif self.state == MissionState.LOCAL_SPIN:
            self.return_to_patrol()

    def cancel_spin(self):
        self.pending_spin = False

        if self.spin_goal_handle is not None:
            self.spin_goal_handle.cancel_goal_async()
            self.spin_goal_handle = None

    # ============================================================
    # Active tag approach
    # ============================================================

    def queue_approach(self, context, preferred_tag=-1):
        self.approach_context = context
        self.set_state(MissionState.INITIAL_TAG_APPROACH if context == 'initial' else MissionState.RELOCALIZATION_APPROACH)
        self.pending_approach = (context, preferred_tag)
        self.process_pending_actions()

    def send_approach_goal_now(self, context, preferred_tag):
        goal = ApproachTag.Goal()
        goal.preferred_tag_id = int(preferred_tag)
        goal.target_distance = float(self.tag_approach_distance)
        goal.timeout_sec = float(self.tag_approach_timeout)

        future = self.approach_client.send_goal_async(goal)
        future.add_done_callback(self.approach_goal_response_callback)

    def approach_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.handle_relocalization_failure(f'ApproachTag send failed: {exc}')
            return

        if not goal_handle.accepted:
            self.handle_relocalization_failure('ApproachTag goal rejected.')
            return

        self.approach_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.approach_result_callback)

    def approach_result_callback(self, future):
        try:
            wrapped = future.result()
        except Exception as exc:
            self.approach_goal_handle = None
            self.handle_relocalization_failure(f'ApproachTag result failed: {exc}')
            return

        context = self.approach_context
        self.approach_goal_handle = None
        self.approach_context = None

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            self.handle_relocalization_failure(wrapped.result.message)
            return

        tag_id = int(wrapped.result.tag_id)
        self.get_logger().info(f'Observation pose reached using tag {tag_id}.')
        self.queue_relocalize(context, tag_id)

    # ============================================================
    # Relocalize
    # ============================================================

    def queue_relocalize(self, context, preferred_tag):
        self.relocalize_context = context
        self.set_state(MissionState.INITIAL_RELOCALIZATION if context == 'initial' else MissionState.RELOCALIZATION_ACQUIRE)
        self.pending_relocalize = (context, preferred_tag)
        self.process_pending_actions()

    def send_relocalize_goal_now(self, context, preferred_tag):
        goal = Relocalize.Goal()
        goal.preferred_tag_id = int(preferred_tag)
        goal.sample_count = int(self.relocalize_sample_count)
        goal.timeout_sec = float(self.relocalize_timeout)

        future = self.relocalize_client.send_goal_async(goal)
        future.add_done_callback(self.relocalize_goal_response_callback)

    def relocalize_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.handle_relocalization_failure(f'Relocalize send failed: {exc}')
            return

        if not goal_handle.accepted:
            self.handle_relocalization_failure('Relocalize goal rejected.')
            return

        self.relocalize_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.relocalize_result_callback)

    def relocalize_result_callback(self, future):
        try:
            wrapped = future.result()
        except Exception as exc:
            self.relocalize_goal_handle = None
            self.handle_relocalization_failure(f'Relocalize result failed: {exc}')
            return

        context = self.relocalize_context
        self.relocalize_goal_handle = None
        self.relocalize_context = None

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            self.handle_relocalization_failure(wrapped.result.message)
            return

        self.distance_since_relocalization = 0.0
        self.initial_retry_count = 0
        self.hard_retry_count = 0

        self.get_logger().info(
            f'Relocalization succeeded using tags {list(wrapped.result.used_tag_ids)}; '
            f'std=({wrapped.result.std_x:.3f}, {wrapped.result.std_y:.3f}, '
            f'{math.degrees(wrapped.result.std_yaw):.2f} deg).'
        )

        if context == 'initial':
            self.initial_localization_complete = True
            self.start_nav2_then_patrol()
        else:
            self.runtime_relocalization_mode = None
            self.advance_patrol()

    def handle_relocalization_failure(self, reason):
        self.approach_goal_handle = None
        self.relocalize_goal_handle = None
        self.approach_context = None
        self.relocalize_context = None
        self.pending_approach = None
        self.pending_relocalize = None

        if not self.initial_localization_complete:
            self.initial_retry_count += 1

            if self.initial_retry_count <= self.initial_relocalization_retries:
                self.get_logger().warn(
                    f'Initial localization failed: {reason} '
                    f'Retry {self.initial_retry_count}/{self.initial_relocalization_retries}.'
                )
                self.queue_approach('initial', self.initial_tag_id)
                return

            self.get_logger().error(f'Initial localization failed permanently: {reason}')
            self.set_state(MissionState.ERROR)
            return

        if self.runtime_relocalization_mode == 'soft':
            self.get_logger().warn(f'Soft relocalization skipped after failure: {reason}')
            self.runtime_relocalization_mode = None
            self.advance_patrol()
            return

        if self.runtime_relocalization_mode == 'hard':
            self.hard_retry_count += 1

            if self.hard_retry_count <= self.hard_relocalization_retries:
                self.get_logger().warn(
                    f'Hard relocalization failed: {reason} '
                    f'Retry {self.hard_retry_count}/{self.hard_relocalization_retries}.'
                )
                self.queue_approach('runtime', -1)
                return

            self.get_logger().error(f'Hard relocalization failed permanently: {reason}')
            self.set_state(MissionState.ERROR)
            return

        self.get_logger().error(f'Relocalization failure in unknown context: {reason}')
        self.set_state(MissionState.ERROR)

    # ============================================================
    # Runtime relocalization policy
    # ============================================================

    def handle_completed_patrol_scan(self):
        distance = self.distance_since_relocalization
        seen_tags = sorted(self.tags_seen_this_checkpoint)

        self.get_logger().info(
            f'Localization check: {distance:.2f} m since last fix, '
            f'tags seen this scan={seen_tags}.'
        )

        if distance >= self.hard_relocalization_distance:
            self.runtime_relocalization_mode = 'hard'
            self.hard_retry_count = 0
            self.get_logger().info('HARD relocalization due: actively finding a tag.')
            self.queue_approach('runtime', -1)
            return

        if distance >= self.soft_relocalization_distance and seen_tags:
            self.runtime_relocalization_mode = 'soft'
            preferred_tag = seen_tags[0] if len(seen_tags) == 1 else -1
            self.get_logger().info('SOFT relocalization due and this checkpoint saw a tag.')
            self.queue_approach('runtime', preferred_tag)
            return

        if distance >= self.soft_relocalization_distance:
            self.get_logger().info('SOFT relocalization due, but this checkpoint saw no tag; continuing patrol.')

        self.advance_patrol()

    # ============================================================
    # Temporary shuttle interfaces
    # ============================================================

    def target_detected_callback(self, msg):
        valid_states = [
            MissionState.PATROL_SCAN,
            MissionState.LOCAL_LOOK,
            MissionState.LOCAL_SPIN,
            MissionState.VERIFY_PATROL,
        ]

        if self.state not in valid_states:
            return

        self.current_target = msg
        self.cancel_spin()
        self.send_target_goal()

    def target_collected_callback(self, msg):
        if not msg.data or self.state != MissionState.COLLECT_TARGET:
            return

        self.publish_collect_request(False)
        self.current_target = None
        self.local_look_start_time = time.monotonic()
        self.set_state(MissionState.LOCAL_LOOK)

    def update(self):
        if self.state != MissionState.LOCAL_LOOK or self.local_look_start_time is None:
            return

        if time.monotonic() - self.local_look_start_time < self.local_fov_wait_time:
            return

        self.local_look_start_time = None
        self.set_state(MissionState.LOCAL_SPIN)
        self.start_spin()

    # ============================================================
    # Patrol progression
    # ============================================================

    def advance_patrol(self):
        self.current_patrol_index += 1

        if self.current_patrol_index >= len(self.patrol_points):
            self.finish_mission()
            return

        self.send_current_patrol_goal()

    def finish_mission(self):
        self.cancel_spin()
        self.publish_collect_request(False)
        self.set_state(MissionState.COMPLETE)
        self.get_logger().info('All patrol points completed.')


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
