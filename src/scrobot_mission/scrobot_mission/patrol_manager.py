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
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scrobot_interfaces.action import ApproachTag, CollectShuttle, Relocalize
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray

from scrobot_mission.patrol_points import find_minimum_grid, generate_patrol_points


class MissionState(Enum):
    IDLE = auto()
    INITIAL_TAG_APPROACH = auto()
    INITIAL_RELOCALIZATION = auto()
    STARTING_NAV2 = auto()
    GO_TO_PATROL = auto()
    PATROL_SCAN = auto()
    GO_TO_STAGING = auto()
    COLLECTING = auto()
    LOCAL_SCAN = auto()
    RETURN_TO_PATROL = auto()
    FINAL_PATROL_SCAN = auto()
    COMPLETE = auto()
    ERROR = auto()


SCAN_STATES = {
    MissionState.PATROL_SCAN,
    MissionState.LOCAL_SCAN,
    MissionState.FINAL_PATROL_SCAN,
}


def detection_position(detection):
    if detection.results:
        p = detection.results[0].pose.pose.position
    else:
        p = detection.bbox.center.position
    return float(p.x), float(p.y), float(p.z)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
    cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
    return math.atan2(siny_cosp, cosy_cosp)


class PatrolManager(Node):
    """Pi-anchored patrol using first visible shuttle and action-based collection."""

    def __init__(self):
        super().__init__('patrol_manager')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('autostart', True)
        self.declare_parameter('court_length', 13.40)
        self.declare_parameter('court_width', 6.10)
        self.declare_parameter('camera_range', 3.0)
        self.declare_parameter('range_factor', 0.90)
        self.declare_parameter('max_grid_size', 20)
        self.declare_parameter('staging_distance', 0.75)
        self.declare_parameter('tf_timeout', 0.05)

        self.declare_parameter('initial_global_localization', True)
        self.declare_parameter('initial_tag_id', -1)
        self.declare_parameter('tag_approach_distance', 1.70)
        self.declare_parameter('tag_approach_timeout', 60.0)
        self.declare_parameter('relocalize_sample_count', 15)
        self.declare_parameter('relocalize_timeout', 7.0)

        self.declare_parameter('nav2_lifecycle_service', '/lifecycle_manager_navigation/manage_nodes')
        self.declare_parameter('nav2_startup_timeout', 30.0)
        self.declare_parameter('nav2_startup_retries', 3)
        self.declare_parameter('nav2_tf_settle_time', 0.75)
        self.declare_parameter('navigation_goal_retries', 3)
        self.declare_parameter('action_retry_period', 0.5)
        self.declare_parameter('spin_angle', 2.0 * math.pi)
        self.declare_parameter('spin_time_allowance', 20.0)

        self.declare_parameter('visible_tracks_topic', '/perception/visible_tracked_shuttles')
        self.declare_parameter('collect_action_name', '/collect_shuttle')

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.autostart = bool(self.get_parameter('autostart').value)
        self.court_length = float(self.get_parameter('court_length').value)
        self.court_width = float(self.get_parameter('court_width').value)
        self.camera_range = float(self.get_parameter('camera_range').value)
        self.range_factor = float(self.get_parameter('range_factor').value)
        self.max_grid_size = int(self.get_parameter('max_grid_size').value)
        self.staging_distance = float(self.get_parameter('staging_distance').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)

        self.initial_global_localization = bool(self.get_parameter('initial_global_localization').value)
        self.initial_tag_id = int(self.get_parameter('initial_tag_id').value)
        self.tag_approach_distance = float(self.get_parameter('tag_approach_distance').value)
        self.tag_approach_timeout = float(self.get_parameter('tag_approach_timeout').value)
        self.relocalize_sample_count = int(self.get_parameter('relocalize_sample_count').value)
        self.relocalize_timeout = float(self.get_parameter('relocalize_timeout').value)

        self.nav2_lifecycle_service = str(self.get_parameter('nav2_lifecycle_service').value)
        self.nav2_startup_timeout = float(self.get_parameter('nav2_startup_timeout').value)
        self.nav2_startup_retries = int(self.get_parameter('nav2_startup_retries').value)
        self.nav2_tf_settle_time = float(self.get_parameter('nav2_tf_settle_time').value)
        self.navigation_goal_retries = int(self.get_parameter('navigation_goal_retries').value)
        self.action_retry_period = float(self.get_parameter('action_retry_period').value)
        self.spin_angle = float(self.get_parameter('spin_angle').value)
        self.spin_time_allowance = float(self.get_parameter('spin_time_allowance').value)
        self.visible_tracks_topic = str(self.get_parameter('visible_tracks_topic').value)
        self.collect_action_name = str(self.get_parameter('collect_action_name').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = MissionState.IDLE
        self.patrol_points = []
        self.current_patrol_index = 0
        self.active_patrol_pose = None

        self.visible_ordered = []
        self.completed_ids = set()
        self.active_target_id = ''
        self.active_target_detection = None

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
        self.spin_goal_request_pending = False
        self.spin_goal_handle = None
        self.spin_cancel_requested = False

        self.collect_goal_handle = None
        self.collect_goal_request_pending = False

        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.state_pub = self.create_publisher(String, '/mission/state', state_qos)
        self.goal_pub = self.create_publisher(PoseStamped, '/mission/current_goal', state_qos)
        self.patrol_points_pub = self.create_publisher(PoseArray, '/mission/patrol_points', state_qos)

        self.create_subscription(
            Detection3DArray,
            self.visible_tracks_topic,
            self.visible_tracks_callback,
            reliable_qos,
        )

        self.navigate_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, '/spin')
        self.collect_client = ActionClient(self, CollectShuttle, self.collect_action_name)
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
        self.publish_state()
        self.get_logger().info('Patrol manager started: first-seen + direct staging + CollectShuttle action.')

    def set_state(self, state):
        if self.state != state:
            self.get_logger().info(f'{self.state.name} -> {state.name}')
        self.state = state
        self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    def visible_tracks_callback(self, msg):
        ordered = []
        for detection in msg.detections:
            track_id = detection.id.strip()
            if not track_id or track_id in self.completed_ids:
                continue
            ordered.append((track_id, copy.deepcopy(detection)))
        self.visible_ordered = ordered

        if self.state in SCAN_STATES and not self.active_target_id and ordered:
            track_id, detection = ordered[0]
            self.acquire_target(track_id, detection)

    def acquire_target(self, track_id, detection):
        if self.active_target_id or track_id in self.completed_ids:
            return

        staging = self.make_staging_pose(detection)
        if staging is None:
            self.get_logger().warn(f'Cannot compute staging pose for shuttle {track_id}: TF unavailable.')
            return

        self.active_target_id = track_id
        self.active_target_detection = copy.deepcopy(detection)
        self.get_logger().info(
            f'{self.state.name}: first confirmed visible shuttle is {track_id}; stopping scan.'
        )

        self.spin_cancel_requested = True
        if self.pending_spin:
            self.pending_spin = False
        if self.spin_goal_handle is not None:
            self.spin_goal_handle.cancel_goal_async()

        self.queue_navigation(staging.pose, 'staging')

    def robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException:
            return None
        t = tf.translation
        q = tf.rotation
        return float(t.x), float(t.y), yaw_from_quaternion(q)

    def make_staging_pose(self, detection):
        robot = self.robot_pose()
        if robot is None:
            return None
        rx, ry, robot_yaw = robot
        tx, ty, _ = detection_position(detection)
        dx = tx - rx
        dy = ty - ry
        distance = math.hypot(dx, dy)

        if distance > 1e-6:
            ux = dx / distance
            uy = dy / distance
            travel = max(0.0, distance - self.staging_distance)
            sx = rx + travel * ux
            sy = ry + travel * uy
            yaw = math.atan2(ty - sy, tx - sx)
        else:
            sx, sy, yaw = rx, ry, robot_yaw

        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = sx
        msg.pose.position.y = sy
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        return msg

    def generate_and_publish_patrol_points(self):
        effective_range = self.camera_range * self.range_factor
        nx, ny, dx, dy, worst = find_minimum_grid(
            self.court_length, self.court_width, effective_range, self.max_grid_size
        )
        xyz_yaw = generate_patrol_points(self.court_length, self.court_width, nx, ny)
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
            self.set_state(MissionState.INITIAL_TAG_APPROACH)
            self.pending_approach = True
            self.process_pending_actions()
        else:
            self.start_nav2_then_patrol()

    def send_approach_goal_now(self):
        self.pending_approach = False
        goal = ApproachTag.Goal()
        goal.preferred_tag_id = self.initial_tag_id
        goal.target_distance = self.tag_approach_distance
        goal.timeout_sec = self.tag_approach_timeout
        self.approach_client.send_goal_async(goal).add_done_callback(self.approach_goal_response)

    def approach_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.enter_error('ApproachTag rejected.')
            return
        self.approach_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.approach_result)

    def approach_result(self, future):
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
        goal.sample_count = self.relocalize_sample_count
        goal.timeout_sec = self.relocalize_timeout
        self.relocalize_client.send_goal_async(goal).add_done_callback(self.relocalize_goal_response)

    def relocalize_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.enter_error('Relocalize rejected.')
            return
        self.relocalize_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.relocalize_result)

    def relocalize_result(self, future):
        wrapped = future.result()
        self.relocalize_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            self.enter_error('Relocalize failed.')
            return
        self.start_nav2_then_patrol()

    def start_nav2_then_patrol(self):
        self.navigation_allowed_time = time.monotonic() + self.nav2_tf_settle_time
        self.nav2_startup_pending = True
        self.nav2_startup_future = None
        self.nav2_startup_attempts = 0
        self.nav2_startup_begin_time = time.monotonic()
        self.set_state(MissionState.STARTING_NAV2)
        self.process_pending_actions()

    def process_nav2_startup(self):
        if not self.nav2_startup_pending:
            return
        if time.monotonic() - self.nav2_startup_begin_time > self.nav2_startup_timeout:
            self.enter_error('Nav2 startup timed out.')
            return
        if self.navigate_client.server_is_ready() and self.spin_client.server_is_ready():
            self.nav2_startup_pending = False
            self.queue_current_patrol_goal()
            return
        if self.nav2_startup_future is not None:
            if not self.nav2_startup_future.done():
                return
            response = self.nav2_startup_future.result()
            self.nav2_startup_future = None
            if response is None or not response.success:
                return
        if not self.nav2_lifecycle_client.service_is_ready():
            return
        if self.navigation_allowed_time and time.monotonic() < self.navigation_allowed_time:
            return
        self.navigation_allowed_time = None
        if self.nav2_startup_attempts >= self.nav2_startup_retries:
            return
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        self.nav2_startup_attempts += 1
        self.nav2_startup_future = self.nav2_lifecycle_client.call_async(request)

    def has_pending_work(self):
        return any([
            self.nav2_startup_pending,
            self.pending_approach,
            self.pending_relocalize_tag is not None,
            self.pending_navigation is not None,
            self.pending_spin,
            self.spin_goal_request_pending,
            self.collect_goal_request_pending,
        ])

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
            if self.navigate_client.server_is_ready():
                pose = self.pending_navigation
                purpose = self.pending_navigation_purpose
                self.pending_navigation = None
                self.pending_navigation_purpose = None
                self.send_navigation_goal_now(pose, purpose)
        if self.pending_spin and not self.spin_goal_request_pending and self.spin_goal_handle is None:
            if self.spin_client.server_is_ready():
                self.pending_spin = False
                self.send_spin_goal_now()
        self.update_action_retry_timer()

    def queue_current_patrol_goal(self):
        if self.current_patrol_index >= len(self.patrol_points):
            self.finish_mission()
            return
        self.active_patrol_pose = copy.deepcopy(self.patrol_points[self.current_patrol_index])
        self.queue_navigation(self.active_patrol_pose, 'patrol')

    def queue_navigation(self, pose, purpose):
        self.pending_navigation = copy.deepcopy(pose)
        self.pending_navigation_purpose = purpose
        self.navigation_retry_count = 0
        self.process_pending_actions()

    def send_navigation_goal_now(self, pose, purpose):
        self.active_navigation_pose = copy.deepcopy(pose)
        self.active_navigation_purpose = purpose
        self.publish_current_goal(pose)
        if purpose == 'patrol':
            self.set_state(MissionState.GO_TO_PATROL)
        elif purpose == 'staging':
            self.set_state(MissionState.GO_TO_STAGING)
        else:
            self.set_state(MissionState.RETURN_TO_PATROL)

        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.pose = pose
        self.navigate_client.send_goal_async(goal).add_done_callback(self.navigation_goal_response)

    def navigation_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.retry_navigation(str(exc))
            return
        if not goal_handle.accepted:
            self.retry_navigation('goal rejected')
            return
        self.navigate_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.navigation_result)

    def retry_navigation(self, reason):
        if self.navigation_retry_count >= self.navigation_goal_retries:
            self.enter_error(f'Navigation failed: {reason}')
            return
        self.navigation_retry_count += 1
        self.navigate_goal_handle = None
        self.pending_navigation = copy.deepcopy(self.active_navigation_pose)
        self.pending_navigation_purpose = self.active_navigation_purpose
        self.update_action_retry_timer()

    def navigation_result(self, future):
        wrapped = future.result()
        purpose = self.active_navigation_purpose
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.retry_navigation(f'status={wrapped.status}')
            return
        self.navigate_goal_handle = None
        self.active_navigation_pose = None
        self.active_navigation_purpose = None
        if purpose == 'patrol':
            self.start_scan(MissionState.PATROL_SCAN)
        elif purpose == 'staging':
            self.start_collection()
        elif purpose == 'return_patrol':
            self.start_scan(MissionState.FINAL_PATROL_SCAN)

    def start_scan(self, scan_state):
        self.active_target_id = ''
        self.active_target_detection = None
        self.spin_cancel_requested = False
        self.set_state(scan_state)
        self.pending_spin = True
        self.process_pending_actions()

    def send_spin_goal_now(self):
        goal = Spin.Goal()
        goal.target_yaw = self.spin_angle
        sec = int(self.spin_time_allowance)
        goal.time_allowance.sec = sec
        goal.time_allowance.nanosec = int((self.spin_time_allowance - sec) * 1e9)
        self.spin_goal_request_pending = True
        self.spin_client.send_goal_async(goal).add_done_callback(self.spin_goal_response)

    def spin_goal_response(self, future):
        self.spin_goal_request_pending = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            if self.active_target_id:
                return
            self.enter_error('Spin rejected.')
            return
        self.spin_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.spin_result)
        if self.spin_cancel_requested or self.state == MissionState.GO_TO_STAGING:
            goal_handle.cancel_goal_async()

    def spin_result(self, future):
        wrapped = future.result()
        self.spin_goal_handle = None
        if self.active_target_id:
            self.spin_cancel_requested = False
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.enter_error(f'Spin failed with status={wrapped.status}.')
            return
        if self.state == MissionState.PATROL_SCAN:
            self.advance_patrol_point()
        elif self.state == MissionState.LOCAL_SCAN:
            self.return_to_active_patrol_point()
        elif self.state == MissionState.FINAL_PATROL_SCAN:
            self.advance_patrol_point()

    def start_collection(self):
        if not self.active_target_id:
            self.enter_error('Reached staging without an active shuttle.')
            return
        if not self.collect_client.server_is_ready():
            self.enter_error('CollectShuttle action server is not ready.')
            return
        self.set_state(MissionState.COLLECTING)
        goal = CollectShuttle.Goal()
        goal.shuttle_id = self.active_target_id
        self.collect_goal_request_pending = True
        self.collect_client.send_goal_async(goal).add_done_callback(self.collect_goal_response)

    def collect_goal_response(self, future):
        self.collect_goal_request_pending = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.enter_error(f'CollectShuttle rejected shuttle {self.active_target_id}.')
            return
        self.collect_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.collect_result)

    def collect_result(self, future):
        wrapped = future.result()
        self.collect_goal_handle = None
        shuttle_id = self.active_target_id
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            self.enter_error(
                f'Collection failed for shuttle {shuttle_id}: {wrapped.result.message}'
            )
            return

        self.get_logger().info(f'Shuttle {shuttle_id} collected successfully.')
        self.completed_ids.add(shuttle_id)
        self.active_target_id = ''
        self.active_target_detection = None

        for next_id, detection in self.visible_ordered:
            if next_id not in self.completed_ids:
                self.acquire_target(next_id, detection)
                return
        self.start_scan(MissionState.LOCAL_SCAN)

    def return_to_active_patrol_point(self):
        if self.active_patrol_pose is None:
            self.enter_error('No patrol anchor stored.')
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
