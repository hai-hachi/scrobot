#!/usr/bin/env python3

import copy
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
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection3DArray

from scrobot_mission.patrol_points import find_minimum_grid, generate_patrol_points


class MissionState(Enum):
    IDLE = auto()
    INITIAL_TAG_APPROACH = auto()
    INITIAL_RELOCALIZATION = auto()
    STARTING_NAV2 = auto()
    GO_TO_PATROL = auto()
    PATROL_SCAN = auto()
    SELECT_SHUTTLE = auto()
    GO_TO_STAGING = auto()
    WAIT_COLLECTION = auto()
    CHECK_VISIBLE_SHUTTLES = auto()
    LOCAL_SCAN = auto()
    RETURN_TO_PATROL = auto()
    FINAL_PATROL_SCAN = auto()
    COMPLETE = auto()
    ERROR = auto()


class PatrolManager(Node):
    """Patrol mission with Pi-anchored shuttle collection excursions."""

    def __init__(self):
        super().__init__('patrol_manager')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('autostart', True)
        self.declare_parameter('court_length', 13.40)
        self.declare_parameter('court_width', 6.10)
        self.declare_parameter('camera_range', 3.0)
        self.declare_parameter('range_factor', 0.90)
        self.declare_parameter('max_grid_size', 20)

        self.declare_parameter('initial_global_localization', True)
        self.declare_parameter('initial_tag_id', -1)
        self.declare_parameter('initial_relocalization_retries', 3)
        self.declare_parameter('tag_approach_distance', 1.70)
        self.declare_parameter('tag_approach_timeout', 60.0)
        self.declare_parameter('relocalize_sample_count', 15)
        self.declare_parameter('relocalize_timeout', 7.0)

        self.declare_parameter(
            'nav2_lifecycle_service',
            '/lifecycle_manager_navigation/manage_nodes',
        )
        self.declare_parameter('nav2_startup_timeout', 30.0)
        self.declare_parameter('nav2_startup_retries', 3)
        self.declare_parameter('nav2_tf_settle_time', 0.75)
        self.declare_parameter('navigation_goal_retries', 3)

        self.declare_parameter('action_retry_period', 0.5)
        self.declare_parameter('spin_angle', 2.0 * math.pi)
        self.declare_parameter('spin_time_allowance', 20.0)
        self.declare_parameter('selection_wait_timeout', 2.0)

        self.declare_parameter(
            'visible_tracks_topic',
            '/perception/visible_tracked_shuttles',
        )
        self.declare_parameter(
            'selected_id_topic',
            '/mission/selected_shuttle_id',
        )
        self.declare_parameter(
            'staging_pose_topic',
            '/mission/shuttle_staging_pose',
        )
        self.declare_parameter(
            'selection_enabled_topic',
            '/mission/target_selection_enabled',
        )
        self.declare_parameter(
            'collection_request_topic',
            '/mission/collection_request',
        )
        self.declare_parameter(
            'collection_complete_topic',
            '/mission/collection_complete',
        )

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
        self.selection_wait_timeout = float(
            self.get_parameter('selection_wait_timeout').value
        )

        self.visible_tracks_topic = str(
            self.get_parameter('visible_tracks_topic').value
        )
        self.selected_id_topic = str(self.get_parameter('selected_id_topic').value)
        self.staging_pose_topic = str(self.get_parameter('staging_pose_topic').value)
        self.selection_enabled_topic = str(
            self.get_parameter('selection_enabled_topic').value
        )
        self.collection_request_topic = str(
            self.get_parameter('collection_request_topic').value
        )
        self.collection_complete_topic = str(
            self.get_parameter('collection_complete_topic').value
        )

        self.state = MissionState.IDLE
        self.patrol_points = []
        self.current_patrol_index = 0
        self.active_patrol_pose = None

        self.initial_retry_count = 0
        self.pending_approach = False
        self.pending_relocalize_tag = None
        self.approach_goal_handle = None
        self.relocalize_goal_handle = None

        self.nav2_startup_pending = False
        self.nav2_startup_future = None
        self.nav2_startup_attempts = 0
        self.nav2_startup_begin_time = None
        self.navigation_allowed_time = None

        self.pending_navigation = None
        self.pending_navigation_purpose = None
        self.active_navigation_pose = None
        self.active_navigation_purpose = None
        self.navigate_goal_handle = None
        self.navigation_retry_count = 0

        self.pending_spin = False
        self.spin_goal_handle = None
        self.scan_seen_ids = set()

        self.visible_ids = set()
        self.latest_selected_id = ''
        self.latest_staging_pose = None
        self.active_target_id = ''
        self.selection_wait_begin = None
        self.waiting_collection_id = ''
        self.completed_collection_ids = set()

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.state_pub = self.create_publisher(String, '/mission/state', state_qos)
        self.goal_pub = self.create_publisher(
            PoseStamped, '/mission/current_goal', state_qos
        )
        self.patrol_points_pub = self.create_publisher(
            PoseArray, '/mission/patrol_points', state_qos
        )
        self.selection_enabled_pub = self.create_publisher(
            Bool, self.selection_enabled_topic, state_qos
        )
        self.collection_request_pub = self.create_publisher(
            String, self.collection_request_topic, reliable_qos
        )

        self.create_subscription(
            Detection3DArray,
            self.visible_tracks_topic,
            self.visible_tracks_callback,
            reliable_qos,
        )
        self.create_subscription(
            String,
            self.selected_id_topic,
            self.selected_id_callback,
            reliable_qos,
        )
        self.create_subscription(
            PoseStamped,
            self.staging_pose_topic,
            self.staging_pose_callback,
            reliable_qos,
        )
        self.create_subscription(
            String,
            self.collection_complete_topic,
            self.collection_complete_callback,
            reliable_qos,
        )

        self.navigate_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, '/spin')
        self.approach_client = ActionClient(self, ApproachTag, '/approach_tag')
        self.relocalize_client = ActionClient(self, Relocalize, '/relocalize')
        self.nav2_lifecycle_client = self.create_client(
            ManageLifecycleNodes, self.nav2_lifecycle_service
        )

        self.action_retry_timer = self.create_timer(
            self.action_retry_period, self.process_pending_actions
        )
        self.action_retry_timer.cancel()
        self.start_timer = self.create_timer(0.10, self.start_once)

        self.generate_and_publish_patrol_points()
        self.set_target_selection(False)
        self.publish_state()

        self.get_logger().info(
            'Patrol manager started with Pi-anchored shuttle excursion sequence.'
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

    def set_target_selection(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)
        self.selection_enabled_pub.publish(msg)
        if not enabled:
            self.latest_selected_id = ''
            self.latest_staging_pose = None
            self.selection_wait_begin = None

    def visible_tracks_callback(self, msg):
        self.visible_ids = {
            detection.id
            for detection in msg.detections
            if detection.id and detection.id not in self.completed_collection_ids
        }
        if self.state in [
            MissionState.PATROL_SCAN,
            MissionState.LOCAL_SCAN,
            MissionState.FINAL_PATROL_SCAN,
        ]:
            self.scan_seen_ids.update(self.visible_ids)

    def selected_id_callback(self, msg):
        selected_id = msg.data.strip()
        if selected_id in self.completed_collection_ids:
            return
        self.latest_selected_id = selected_id
        self.try_dispatch_selected_target()

    def staging_pose_callback(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            return
        self.latest_staging_pose = copy.deepcopy(msg)
        self.try_dispatch_selected_target()

    def collection_complete_callback(self, msg):
        completed_id = msg.data.strip()
        if self.state != MissionState.WAIT_COLLECTION:
            return
        if completed_id != self.waiting_collection_id:
            return

        self.get_logger().info(f'Shuttle {completed_id} collection complete.')
        self.completed_collection_ids.add(completed_id)
        self.waiting_collection_id = ''
        self.active_target_id = ''
        self.set_target_selection(False)
        self.set_state(MissionState.CHECK_VISIBLE_SHUTTLES)
        self.evaluate_after_collection()

    def generate_and_publish_patrol_points(self):
        effective_range = self.camera_range * self.range_factor
        nx, ny, dx, dy, worst = find_minimum_grid(
            self.court_length,
            self.court_width,
            effective_range,
            self.max_grid_size,
        )
        xyz_yaw = generate_patrol_points(
            self.court_length, self.court_width, nx, ny
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
            f'cell={dx:.2f}x{dy:.2f} m, worst-case={worst:.2f} m.'
        )

    def publish_current_goal(self, pose):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose = pose
        self.goal_pub.publish(msg)

    def start_once(self):
        self.start_timer.cancel()
        if not self.autostart or not self.patrol_points:
            return
        self.current_patrol_index = 0
        if self.initial_global_localization:
            self.start_initial_global_localization()
        else:
            self.start_nav2_then_patrol()

    def start_initial_global_localization(self):
        self.set_state(MissionState.INITIAL_TAG_APPROACH)
        self.pending_approach = True
        self.process_pending_actions()

    def send_approach_goal_now(self):
        self.pending_approach = False
        goal = ApproachTag.Goal()
        goal.preferred_tag_id = int(self.initial_tag_id)
        goal.target_distance = float(self.tag_approach_distance)
        goal.timeout_sec = float(self.tag_approach_timeout)
        self.approach_client.send_goal_async(goal).add_done_callback(
            self.approach_goal_response_callback
        )

    def approach_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.enter_error('ApproachTag rejected.')
            return
        self.approach_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.approach_result_callback)

    def approach_result_callback(self, future):
        wrapped = future.result()
        self.approach_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            self.enter_error('ApproachTag failed.')
            return
        self.set_state(MissionState.INITIAL_RELOCALIZATION)
        self.pending_relocalize_tag = int(wrapped.result.tag_id)
        self.process_pending_actions()

    def send_relocalize_goal_now(self, tag_id):
        self.pending_relocalize_tag = None
        goal = Relocalize.Goal()
        goal.preferred_tag_id = int(tag_id)
        goal.sample_count = int(self.relocalize_sample_count)
        goal.timeout_sec = float(self.relocalize_timeout)
        self.relocalize_client.send_goal_async(goal).add_done_callback(
            self.relocalize_goal_response_callback
        )

    def relocalize_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.enter_error('Relocalize rejected.')
            return
        self.relocalize_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.relocalize_result_callback)

    def relocalize_result_callback(self, future):
        wrapped = future.result()
        self.relocalize_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            self.enter_error('Relocalize failed.')
            return
        self.start_nav2_then_patrol()

    def start_nav2_then_patrol(self):
        self.navigation_allowed_time = time.monotonic() + self.nav2_tf_settle_time
        if self.navigate_client.server_is_ready() and self.spin_client.server_is_ready():
            self.queue_current_patrol_goal()
            return
        self.nav2_startup_pending = True
        self.nav2_startup_begin_time = time.monotonic()
        self.set_state(MissionState.STARTING_NAV2)
        self.process_pending_actions()

    def process_nav2_startup(self):
        if not self.nav2_startup_pending:
            return
        if self.navigate_client.server_is_ready() and self.spin_client.server_is_ready():
            self.nav2_startup_pending = False
            self.queue_current_patrol_goal()
            return
        if time.monotonic() - self.nav2_startup_begin_time > self.nav2_startup_timeout:
            self.enter_error('Nav2 startup timed out.')

    def has_pending_work(self):
        return (
            self.nav2_startup_pending
            or self.pending_approach
            or self.pending_relocalize_tag is not None
            or self.pending_navigation is not None
            or self.pending_spin
            or self.state == MissionState.SELECT_SHUTTLE
        )

    def update_action_retry_timer(self):
        if self.has_pending_work():
            self.action_retry_timer.reset()
        else:
            self.action_retry_timer.cancel()

    def process_pending_actions(self):
        if self.state in [MissionState.ERROR, MissionState.COMPLETE]:
            return
        self.process_nav2_startup()
        if self.pending_approach and self.approach_client.server_is_ready():
            self.send_approach_goal_now()
        if self.pending_relocalize_tag is not None and self.relocalize_client.server_is_ready():
            self.send_relocalize_goal_now(self.pending_relocalize_tag)
        if self.pending_navigation is not None and self.navigate_goal_handle is None:
            pose = self.pending_navigation
            purpose = self.pending_navigation_purpose
            self.pending_navigation = None
            self.pending_navigation_purpose = None
            self.send_navigation_goal_now(pose, purpose)
        if self.pending_spin and self.spin_goal_handle is None and self.spin_client.server_is_ready():
            self.pending_spin = False
            self.send_spin_goal_now()
        if self.state == MissionState.SELECT_SHUTTLE:
            self.try_dispatch_selected_target()
        self.update_action_retry_timer()

    def queue_navigation(self, pose, purpose):
        self.pending_navigation = copy.deepcopy(pose)
        self.pending_navigation_purpose = purpose
        self.process_pending_actions()

    def queue_current_patrol_goal(self):
        if self.current_patrol_index >= len(self.patrol_points):
            self.finish_mission()
            return
        self.active_patrol_pose = copy.deepcopy(self.patrol_points[self.current_patrol_index])
        self.queue_navigation(self.active_patrol_pose, 'patrol')

    def send_navigation_goal_now(self, pose, purpose):
        self.active_navigation_pose = copy.deepcopy(pose)
        self.active_navigation_purpose = purpose
        self.publish_current_goal(pose)
        if purpose == 'patrol':
            self.set_state(MissionState.GO_TO_PATROL)
            self.get_logger().info(f'Going to P{self.current_patrol_index}.')
        elif purpose == 'staging':
            self.set_state(MissionState.GO_TO_STAGING)
            self.get_logger().info(
                f'Going to staging pose for shuttle {self.active_target_id}.'
            )
        elif purpose == 'return_patrol':
            self.set_state(MissionState.RETURN_TO_PATROL)

        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.pose = pose
        self.navigate_client.send_goal_async(goal).add_done_callback(
            self.navigation_goal_response_callback
        )

    def navigation_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.enter_error('NavigateToPose rejected.')
            return
        self.navigate_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        wrapped = future.result()
        purpose = self.active_navigation_purpose
        self.navigate_goal_handle = None
        self.active_navigation_pose = None
        self.active_navigation_purpose = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.enter_error(f'Navigation purpose={purpose} failed.')
            return
        if purpose == 'patrol':
            self.start_scan(MissionState.PATROL_SCAN)
        elif purpose == 'staging':
            self.request_collection()
        elif purpose == 'return_patrol':
            self.start_scan(MissionState.FINAL_PATROL_SCAN)

    def start_scan(self, scan_state):
        self.set_target_selection(False)
        self.scan_seen_ids.clear()
        self.set_state(scan_state)
        self.pending_spin = True
        self.process_pending_actions()

    def send_spin_goal_now(self):
        goal = Spin.Goal()
        goal.target_yaw = float(self.spin_angle)
        sec = int(self.spin_time_allowance)
        goal.time_allowance.sec = sec
        goal.time_allowance.nanosec = int((self.spin_time_allowance - sec) * 1e9)
        self.get_logger().info(
            f'{self.state.name}: spinning {math.degrees(self.spin_angle):.1f} deg.'
        )
        self.spin_client.send_goal_async(goal).add_done_callback(
            self.spin_goal_response_callback
        )

    def spin_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.enter_error('Spin rejected.')
            return
        self.spin_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.spin_result_callback)

    def spin_result_callback(self, future):
        wrapped = future.result()
        self.spin_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.enter_error('Spin failed.')
            return
        scan_state = self.state
        found = sorted(self.scan_seen_ids)
        self.get_logger().info(f'{scan_state.name} complete; shuttle IDs seen={found}.')
        if found:
            self.begin_target_selection()
            return
        if scan_state == MissionState.PATROL_SCAN:
            self.advance_patrol_point()
        elif scan_state == MissionState.LOCAL_SCAN:
            self.return_to_active_patrol_point()
        elif scan_state == MissionState.FINAL_PATROL_SCAN:
            self.advance_patrol_point()

    def begin_target_selection(self):
        self.latest_selected_id = ''
        self.latest_staging_pose = None
        self.selection_wait_begin = time.monotonic()
        self.set_state(MissionState.SELECT_SHUTTLE)
        self.set_target_selection(True)
        self.update_action_retry_timer()

    def try_dispatch_selected_target(self):
        if self.state != MissionState.SELECT_SHUTTLE:
            return
        if not self.latest_selected_id or self.latest_staging_pose is None:
            return
        if self.latest_selected_id in self.completed_collection_ids:
            return

        self.active_target_id = self.latest_selected_id
        staging_pose = copy.deepcopy(self.latest_staging_pose.pose)
        self.set_target_selection(False)
        self.queue_navigation(staging_pose, 'staging')

    def request_collection(self):
        target_id = self.active_target_id
        if not target_id:
            self.enter_error('Reached staging pose without active target ID.')
            return
        self.waiting_collection_id = target_id
        self.set_state(MissionState.WAIT_COLLECTION)
        msg = String()
        msg.data = target_id
        self.collection_request_pub.publish(msg)
        self.get_logger().info(f'Collection requested for shuttle {target_id}.')

    def evaluate_after_collection(self):
        remaining_visible = sorted(
            sid for sid in self.visible_ids
            if sid not in self.completed_collection_ids
        )
        if remaining_visible:
            self.begin_target_selection()
        else:
            self.start_scan(MissionState.LOCAL_SCAN)

    def return_to_active_patrol_point(self):
        if self.active_patrol_pose is None:
            self.enter_error('No active patrol anchor pose stored.')
            return
        self.queue_navigation(self.active_patrol_pose, 'return_patrol')

    def advance_patrol_point(self):
        self.current_patrol_index += 1
        if self.current_patrol_index >= len(self.patrol_points):
            self.finish_mission()
        else:
            self.queue_current_patrol_goal()

    def finish_mission(self):
        self.set_state(MissionState.COMPLETE)

    def enter_error(self, reason):
        self.get_logger().error(reason)
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
