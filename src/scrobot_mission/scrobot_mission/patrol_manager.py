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
from vision_msgs.msg import Detection3DArray

from scrobot_mission.patrol_points import find_minimum_grid, generate_patrol_points


class MissionState(Enum):
    IDLE = auto()
    INITIAL_TAG_APPROACH = auto()
    INITIAL_RELOCALIZATION = auto()
    STARTING_NAV2 = auto()
    GO_TO_PATROL = auto()
    PATROL_SCAN = auto()
    GROUPING = auto()
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


def quat_normalize(q):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    inv = 1.0 / norm
    return x * inv, y * inv, z * inv, w * inv


def quat_conjugate(q):
    x, y, z, w = q
    return -x, -y, -z, w


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q, v):
    qn = quat_normalize(q)
    out = quat_multiply(
        quat_multiply(qn, (v[0], v[1], v[2], 0.0)),
        quat_conjugate(qn),
    )
    return out[0], out[1], out[2]


def transform_point(transform, point):
    q = transform.rotation
    t = transform.translation
    rotated = quat_rotate(
        (float(q.x), float(q.y), float(q.z), float(q.w)), point
    )
    return (
        rotated[0] + float(t.x),
        rotated[1] + float(t.y),
        rotated[2] + float(t.z),
    )


class PatrolManager(Node):
    """Pi-anchored hybrid patrol and shuttle collection mission manager."""

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
        self.declare_parameter('tf_timeout', 0.05)

        # First visible track remains the primary for simplicity. During an
        # active scan, keep rotating until it approaches the opposite FOV edge
        # so nearby shuttles around that primary have time to enter the image.
        self.declare_parameter('primary_scan_edge_bearing', 0.45)
        self.declare_parameter('target_lost_timeout', 0.75)
        self.declare_parameter('group_settle_time', 0.40)
        self.declare_parameter('group_lateral_tolerance', 0.15)
        self.declare_parameter('group_longitudinal_tolerance', 0.25)

        # Hybrid collection: Nav2 moves to a staging pose; local CollectShuttle
        # performs final ALIGN -> straight DRIVE.
        self.declare_parameter('staging_distance', 0.80)

        self.declare_parameter('initial_global_localization', True)
        self.declare_parameter('initial_tag_id', -1)
        self.declare_parameter('tag_approach_distance', 1.70)
        self.declare_parameter('tag_approach_timeout', 60.0)
        self.declare_parameter('relocalize_sample_count', 15)
        self.declare_parameter('relocalize_timeout', 15.0)
        self.declare_parameter('relocalize_retries', 4)

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

        self.declare_parameter(
            'visible_tracks_topic',
            '/perception/visible_tracked_shuttles',
        )
        self.declare_parameter('collect_action_name', '/collect_shuttle')

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.autostart = bool(self.get_parameter('autostart').value)
        self.court_length = float(self.get_parameter('court_length').value)
        self.court_width = float(self.get_parameter('court_width').value)
        self.camera_range = float(self.get_parameter('camera_range').value)
        self.range_factor = float(self.get_parameter('range_factor').value)
        self.max_grid_size = int(self.get_parameter('max_grid_size').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)

        self.primary_scan_edge_bearing = float(
            self.get_parameter('primary_scan_edge_bearing').value
        )
        self.target_lost_timeout = float(
            self.get_parameter('target_lost_timeout').value
        )
        self.group_settle_time = float(self.get_parameter('group_settle_time').value)
        self.group_lateral_tolerance = float(
            self.get_parameter('group_lateral_tolerance').value
        )
        self.group_longitudinal_tolerance = float(
            self.get_parameter('group_longitudinal_tolerance').value
        )
        self.staging_distance = float(self.get_parameter('staging_distance').value)

        self.initial_global_localization = bool(
            self.get_parameter('initial_global_localization').value
        )
        self.initial_tag_id = int(self.get_parameter('initial_tag_id').value)
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
        self.relocalize_retries = int(
            self.get_parameter('relocalize_retries').value
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
        self.visible_tracks_topic = str(
            self.get_parameter('visible_tracks_topic').value
        )
        self.collect_action_name = str(
            self.get_parameter('collect_action_name').value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = MissionState.IDLE
        self.patrol_points = []
        self.current_patrol_index = 0
        self.active_patrol_pose = None

        self.visible_ordered = []
        # IDs are mission-side attempted objects, not Gazebo-confirmed pickups.
        # Gazebo collection outcome must never determine mission progression.
        self.attempted_ids = set()
        self.active_target_id = ''
        self.active_target_detection = None
        self.active_group_ids = []
        self.active_group_detections = {}
        self.target_origin_state = None
        self.primary_last_visible_time = 0.0
        self.grouping_timer = None
        self.grouping_pending = False
        self.primary_scan_tracking = False

        self.pending_approach = False
        self.pending_relocalize_tag = None
        self.approach_goal_handle = None
        self.relocalize_goal_handle = None
        self.relocalize_retry_count = 0
        self.last_relocalize_tag = -1

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
        self.navigation_goal_request_pending = False
        self.navigation_retry_count = 0
        self.navigation_cancel_reason = None

        self.pending_spin = False
        self.spin_goal_request_pending = False
        self.spin_goal_handle = None
        self.spin_cancel_requested = False

        self.collect_goal_handle = None
        self.collect_goal_request_pending = False

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.state_pub = self.create_publisher(String, '/mission/state', state_qos)
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

        self.create_subscription(
            Detection3DArray,
            self.visible_tracks_topic,
            self.visible_tracks_callback,
            reliable_qos,
        )

        self.navigate_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, '/spin')
        self.collect_client = ActionClient(
            self,
            CollectShuttle,
            self.collect_action_name,
        )
        self.approach_client = ActionClient(self, ApproachTag, '/approach_tag')
        self.relocalize_client = ActionClient(self, Relocalize, '/relocalize')
        self.nav2_lifecycle_client = self.create_client(
            ManageLifecycleNodes,
            self.nav2_lifecycle_service,
        )

        self.action_retry_timer = self.create_timer(
            self.action_retry_period,
            self.process_pending_actions,
        )
        self.action_retry_timer.cancel()
        self.start_timer = self.create_timer(0.10, self.start_once)

        self.generate_and_publish_patrol_points()
        self.publish_state()
        self.get_logger().info(
            'Patrol manager started: first-visible primary -> edge-aware scan -> '
            'single grouping -> Nav2 staging -> local ALIGN/DRIVE.'
        )

    # ------------------------------------------------------------------
    # Common state / perception helpers
    # ------------------------------------------------------------------

    def set_state(self, state):
        if self.state != state:
            self.get_logger().info(f'{self.state.name} -> {state.name}')
        self.state = state
        self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    def _visible_detection(self, track_id):
        for visible_id, detection in self.visible_ordered:
            if visible_id == track_id:
                return detection
        return None

    def _clear_active_target(self):
        self.active_target_id = ''
        self.active_target_detection = None
        self.active_group_ids = []
        self.active_group_detections = {}
        self.target_origin_state = None
        self.primary_last_visible_time = 0.0
        self.grouping_pending = False
        self.primary_scan_tracking = False
        self.spin_cancel_requested = False

    def visible_tracks_callback(self, msg):
        now = time.monotonic()
        ordered = []
        for detection in msg.detections:
            track_id = detection.id.strip()
            if not track_id or track_id in self.attempted_ids:
                continue
            ordered.append((track_id, copy.deepcopy(detection)))
        self.visible_ordered = ordered

        if self.active_target_id:
            primary = self._visible_detection(self.active_target_id)
            if primary is not None:
                self.active_target_detection = copy.deepcopy(primary)
                self.primary_last_visible_time = now

                if self.state in SCAN_STATES and self.primary_scan_tracking:
                    self._maybe_stop_scan_at_primary_edge(primary)
            else:
                lost_for = now - self.primary_last_visible_time
                if self.state in SCAN_STATES and lost_for >= self.target_lost_timeout:
                    # The primary vanished before we committed to grouping.
                    # Keep the scan running and simply allow another primary.
                    old = self.active_target_id
                    self.get_logger().info(
                        f'Primary {old} left view before grouping; continuing scan.'
                    )
                    self._clear_active_target()
                elif (
                    self.state == MissionState.GO_TO_STAGING
                    and lost_for >= self.target_lost_timeout
                    and self.navigation_cancel_reason is None
                ):
                    self.get_logger().info(
                        f'Primary {self.active_target_id} lost while going to staging; '
                        'canceling staging navigation.'
                    )
                    self._cancel_navigation('target_lost')

        # Search/scan states may select the first currently visible eligible ID.
        if self.state in SCAN_STATES and not self.active_target_id and ordered:
            track_id, detection = ordered[0]
            self.acquire_target(track_id, detection, continue_scan=True)
            return

        # RETURN_TO_PATROL is opportunistic searching too. A visible shuttle
        # interrupts the return, but unlike an active spin there is no need to
        # rotate it across the whole FOV first.
        if (
            self.state == MissionState.RETURN_TO_PATROL
            and not self.active_target_id
            and ordered
        ):
            track_id, detection = ordered[0]
            self.acquire_target(track_id, detection, continue_scan=False)

    def acquire_target(self, track_id, detection, continue_scan=False):
        if self.active_target_id or track_id in self.attempted_ids:
            return

        self.target_origin_state = self.state
        self.active_target_id = track_id
        self.active_target_detection = copy.deepcopy(detection)
        self.active_group_ids = [track_id]
        self.active_group_detections = {track_id: copy.deepcopy(detection)}
        self.primary_last_visible_time = time.monotonic()
        self.grouping_pending = True
        self.primary_scan_tracking = bool(continue_scan and self.state in SCAN_STATES)

        if self.primary_scan_tracking:
            self.get_logger().info(
                f'{self.state.name}: first confirmed visible shuttle is {track_id}; '
                'keeping the scan running until the primary nears the opposite FOV edge.'
            )
            self._maybe_stop_scan_at_primary_edge(detection)
            return

        if self.state == MissionState.RETURN_TO_PATROL:
            self.get_logger().info(
                f'RETURN_TO_PATROL: visible shuttle {track_id}; canceling return '
                'and collecting opportunistically.'
            )
            self._cancel_navigation('target_seen')
            return

        self.begin_grouping()

    def _primary_bearing(self, detection):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.frame_id,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException:
            return None
        local = transform_point(tf, detection_position(detection))
        return math.atan2(local[1], max(local[0], 1e-6))

    def _maybe_stop_scan_at_primary_edge(self, detection):
        bearing = self._primary_bearing(detection)
        if bearing is None:
            return

        # Positive Nav2 Spin rotates the robot CCW, so a fixed world target
        # moves from +bearing toward -bearing in the robot frame. Reverse this
        # test automatically if a negative spin angle is configured.
        if self.spin_angle >= 0.0:
            edge_reached = bearing <= -abs(self.primary_scan_edge_bearing)
        else:
            edge_reached = bearing >= abs(self.primary_scan_edge_bearing)

        if not edge_reached or self.spin_cancel_requested:
            return

        self.get_logger().info(
            f'Primary {self.active_target_id} reached scan-edge bearing '
            f'{math.degrees(bearing):.1f} deg; stopping scan for grouping.'
        )
        self.spin_cancel_requested = True
        self.primary_scan_tracking = False
        self.pending_spin = False
        if self.spin_goal_handle is not None:
            self.spin_goal_handle.cancel_goal_async()

    # ------------------------------------------------------------------
    # Grouping and staging
    # ------------------------------------------------------------------

    def begin_grouping(self):
        if not self.active_target_id or not self.grouping_pending:
            return
        if self.spin_goal_handle is not None or self.spin_goal_request_pending:
            return
        if self.navigate_goal_handle is not None or self.navigation_goal_request_pending:
            return

        self.grouping_pending = False
        self.set_state(MissionState.GROUPING)
        self.get_logger().info(
            f'Holding still for {self.group_settle_time:.2f} s to group shuttles '
            f'around primary {self.active_target_id}.'
        )
        self.grouping_timer = self.create_timer(
            max(self.group_settle_time, 0.01),
            self.finish_grouping,
        )

    def finish_grouping(self):
        if self.grouping_timer is not None:
            self.grouping_timer.cancel()
            self.destroy_timer(self.grouping_timer)
            self.grouping_timer = None

        if self.state != MissionState.GROUPING or not self.active_target_id:
            return

        primary_detection = self._visible_detection(self.active_target_id)
        if primary_detection is None:
            self._abandon_target('primary disappeared during grouping')
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.frame_id,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot form multi-shuttle local group ({exc}); using primary only.'
            )
            self.active_group_ids = [self.active_target_id]
            self.active_group_detections = {
                self.active_target_id: copy.deepcopy(primary_detection)
            }
            self._queue_staging_for_active_group()
            return

        primary_local = transform_point(
            tf,
            detection_position(primary_detection),
        )
        group_ids = [self.active_target_id]
        group_detections = {
            self.active_target_id: copy.deepcopy(primary_detection)
        }

        for track_id, detection in self.visible_ordered:
            if track_id == self.active_target_id or track_id in self.attempted_ids:
                continue
            local = transform_point(tf, detection_position(detection))
            if (
                abs(local[1] - primary_local[1]) <= self.group_lateral_tolerance
                and abs(local[0] - primary_local[0])
                <= self.group_longitudinal_tolerance
            ):
                group_ids.append(track_id)
                group_detections[track_id] = copy.deepcopy(detection)

        self.active_group_ids = group_ids
        self.active_group_detections = group_detections
        self.active_target_detection = copy.deepcopy(primary_detection)
        self.get_logger().info(
            f'Locked collection group around {self.active_target_id}: '
            f'{self.active_group_ids}'
        )
        self._queue_staging_for_active_group()

    def _robot_xy_in_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException:
            return None
        return float(tf.translation.x), float(tf.translation.y)

    def _compute_staging_pose(self):
        detections = [
            self.active_group_detections[track_id]
            for track_id in self.active_group_ids
            if track_id in self.active_group_detections
        ]
        if not detections:
            return None

        positions = [detection_position(d) for d in detections]
        group_x = sum(p[0] for p in positions) / float(len(positions))
        group_y = sum(p[1] for p in positions) / float(len(positions))

        robot_xy = self._robot_xy_in_map()
        if robot_xy is None:
            return None

        dx = group_x - robot_xy[0]
        dy = group_y - robot_xy[1]
        norm = math.hypot(dx, dy)
        if norm <= 1e-6:
            return None

        ux = dx / norm
        uy = dy / norm
        pose = Pose()
        pose.position.x = group_x - self.staging_distance * ux
        pose.position.y = group_y - self.staging_distance * uy
        yaw = math.atan2(uy, ux)
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _queue_staging_for_active_group(self):
        if not self.active_target_id or not self.active_group_ids:
            self._abandon_target('no locked group available for staging')
            return

        staging_pose = self._compute_staging_pose()
        if staging_pose is None:
            self._abandon_target('cannot compute staging pose')
            return

        self.get_logger().info(
            f'Nav2 staging for primary {self.active_target_id}: '
            f'distance={self.staging_distance:.2f} m, '
            f'group={self.active_group_ids}.'
        )
        self.queue_navigation(staging_pose, 'staging')

    def _primary_is_recently_visible(self):
        if not self.active_target_id:
            return False
        if self._visible_detection(self.active_target_id) is not None:
            return True
        return (
            time.monotonic() - self.primary_last_visible_time
            < self.target_lost_timeout
        )

    def _abandon_target(self, reason):
        old_id = self.active_target_id
        origin = self.target_origin_state
        self.get_logger().warn(
            f'Abandoning shuttle target {old_id}: {reason}. '
            'This is not a mission failure.'
        )
        self._clear_active_target()

        # Prefer another currently visible shuttle immediately.
        if self.visible_ordered:
            track_id, detection = self.visible_ordered[0]
            if track_id not in self.attempted_ids:
                self.acquire_target(track_id, detection, continue_scan=False)
                return

        if origin == MissionState.RETURN_TO_PATROL:
            self.return_to_active_patrol_point()
        else:
            self.start_scan(MissionState.LOCAL_SCAN)

    # ------------------------------------------------------------------
    # Patrol points and initial localization
    # ------------------------------------------------------------------

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
        self.approach_client.send_goal_async(goal).add_done_callback(
            self.approach_goal_response
        )

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
        self.last_relocalize_tag = int(wrapped.result.tag_id)
        self.relocalize_retry_count = 0
        self.pending_relocalize_tag = self.last_relocalize_tag
        self.process_pending_actions()

    def send_relocalize_goal_now(self, tag_id):
        self.pending_relocalize_tag = None
        goal = Relocalize.Goal()
        goal.preferred_tag_id = int(tag_id)
        goal.sample_count = self.relocalize_sample_count
        goal.timeout_sec = self.relocalize_timeout
        self.relocalize_client.send_goal_async(goal).add_done_callback(
            self.relocalize_goal_response
        )

    def relocalize_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.retry_relocalization('goal rejected')
            return
        self.relocalize_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.relocalize_result)

    def relocalize_result(self, future):
        wrapped = future.result()
        self.relocalize_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            self.retry_relocalization('action failed')
            return
        self.start_nav2_then_patrol()

    def retry_relocalization(self, reason):
        if self.relocalize_retry_count >= self.relocalize_retries:
            self.enter_error(
                f'Relocalize failed after {self.relocalize_retry_count + 1} attempts: '
                f'{reason}'
            )
            return
        self.relocalize_retry_count += 1
        self.get_logger().warn(
            f'Relocalize attempt failed ({reason}); retry '
            f'{self.relocalize_retry_count}/{self.relocalize_retries}.'
        )
        self.pending_relocalize_tag = self.last_relocalize_tag
        self.update_action_retry_timer()

    # ------------------------------------------------------------------
    # Nav2 lifecycle and goal handling
    # ------------------------------------------------------------------

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
        return any(
            [
                self.nav2_startup_pending,
                self.pending_approach,
                self.pending_relocalize_tag is not None,
                self.pending_navigation is not None,
                self.navigation_goal_request_pending,
                self.pending_spin,
                self.spin_goal_request_pending,
                self.collect_goal_request_pending,
            ]
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

        if (
            self.pending_relocalize_tag is not None
            and self.relocalize_client.server_is_ready()
        ):
            self.send_relocalize_goal_now(self.pending_relocalize_tag)

        if (
            self.pending_navigation is not None
            and self.navigate_goal_handle is None
            and not self.navigation_goal_request_pending
            and self.navigate_client.server_is_ready()
        ):
            pose = self.pending_navigation
            purpose = self.pending_navigation_purpose
            self.pending_navigation = None
            self.pending_navigation_purpose = None
            self.send_navigation_goal_now(pose, purpose)

        if (
            self.pending_spin
            and not self.spin_goal_request_pending
            and self.spin_goal_handle is None
            and self.spin_client.server_is_ready()
        ):
            self.pending_spin = False
            self.send_spin_goal_now()

        self.update_action_retry_timer()

    def queue_current_patrol_goal(self):
        if self.current_patrol_index >= len(self.patrol_points):
            self.finish_mission()
            return
        self.active_patrol_pose = copy.deepcopy(
            self.patrol_points[self.current_patrol_index]
        )
        self.queue_navigation(self.active_patrol_pose, 'patrol')

    def queue_navigation(self, pose, purpose):
        self.pending_navigation = copy.deepcopy(pose)
        self.pending_navigation_purpose = purpose
        self.navigation_retry_count = 0
        self.navigation_cancel_reason = None
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
        self.navigation_goal_request_pending = True
        self.navigate_client.send_goal_async(goal).add_done_callback(
            self.navigation_goal_response
        )

    def navigation_goal_response(self, future):
        self.navigation_goal_request_pending = False
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
        if self.navigation_cancel_reason is not None:
            goal_handle.cancel_goal_async()

    def _cancel_navigation(self, reason):
        self.navigation_cancel_reason = reason
        self.pending_navigation = None
        self.pending_navigation_purpose = None
        if self.navigate_goal_handle is not None:
            self.navigate_goal_handle.cancel_goal_async()
        elif not self.navigation_goal_request_pending:
            self._handle_navigation_cancel_without_goal()

    def _handle_navigation_cancel_without_goal(self):
        reason = self.navigation_cancel_reason
        self.navigation_cancel_reason = None
        if reason == 'target_seen':
            self.begin_grouping()
        elif reason == 'target_lost':
            self._abandon_target('primary lost before staging')

    def retry_navigation(self, reason):
        purpose = self.active_navigation_purpose
        if self.navigation_retry_count >= self.navigation_goal_retries:
            if purpose == 'staging':
                self.navigate_goal_handle = None
                self.navigation_goal_request_pending = False
                self._abandon_target(f'Nav2 staging failed: {reason}')
                return
            self.enter_error(f'Navigation failed: {reason}')
            return

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
            if cancel_reason == 'target_seen':
                self.begin_grouping()
            elif cancel_reason == 'target_lost':
                self._abandon_target('primary lost while navigating to staging')
            return

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.retry_navigation(f'status={wrapped.status}')
            return

        self.active_navigation_pose = None
        self.active_navigation_purpose = None

        if purpose == 'patrol':
            self.start_scan(MissionState.PATROL_SCAN)
        elif purpose == 'return_patrol':
            if self.active_target_id:
                self.begin_grouping()
            else:
                self.start_scan(MissionState.FINAL_PATROL_SCAN)
        elif purpose == 'staging':
            if not self._primary_is_recently_visible():
                self._abandon_target('primary not visible at staging pose')
                return
            self.get_logger().info(
                f'Staging reached and primary {self.active_target_id} is visible; '
                'starting local collection.'
            )
            self.start_collection()

    # ------------------------------------------------------------------
    # Scan handling
    # ------------------------------------------------------------------

    def start_scan(self, scan_state):
        self._clear_active_target()
        self.set_state(scan_state)
        self.pending_spin = True
        self.process_pending_actions()

    def send_spin_goal_now(self):
        goal = Spin.Goal()
        goal.target_yaw = self.spin_angle
        sec = int(self.spin_time_allowance)
        goal.time_allowance.sec = sec
        goal.time_allowance.nanosec = int(
            (self.spin_time_allowance - sec) * 1e9
        )
        self.spin_goal_request_pending = True
        self.spin_client.send_goal_async(goal).add_done_callback(
            self.spin_goal_response
        )

    def spin_goal_response(self, future):
        self.spin_goal_request_pending = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            if self.active_target_id:
                self.begin_grouping()
                return
            self.enter_error('Spin rejected.')
            return

        self.spin_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.spin_result)
        if self.spin_cancel_requested:
            goal_handle.cancel_goal_async()

    def spin_result(self, future):
        wrapped = future.result()
        self.spin_goal_handle = None

        if self.active_target_id:
            self.spin_cancel_requested = False
            self.primary_scan_tracking = False
            self.begin_grouping()
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

    # ------------------------------------------------------------------
    # Local collection action
    # ------------------------------------------------------------------

    def start_collection(self):
        if not self.active_target_id or not self.active_group_ids:
            self._abandon_target('cannot start collection without locked group')
            return
        if not self.collect_client.server_is_ready():
            self.enter_error('CollectShuttle action server is not ready.')
            return

        self.set_state(MissionState.COLLECTING)
        goal = CollectShuttle.Goal()
        # First ID is always the primary; final controller aligns to it.
        goal.shuttle_ids = list(self.active_group_ids)
        self.collect_goal_request_pending = True
        self.collect_client.send_goal_async(goal).add_done_callback(
            self.collect_goal_response
        )

    def collect_goal_response(self, future):
        self.collect_goal_request_pending = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                f'CollectShuttle rejected group {self.active_group_ids}; moving on.'
            )
            self._after_collection_attempt(drive_completed=False)
            return
        self.collect_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.collect_result)

    def collect_result(self, future):
        wrapped = future.result()
        self.collect_goal_handle = None

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'CollectShuttle action ended with status={wrapped.status}; moving on.'
            )
            self._after_collection_attempt(drive_completed=False)
            return

        drive_completed = bool(wrapped.result.success)
        self.get_logger().info(
            f'Collection action finished: drive_completed={drive_completed}; '
            f'evaluation_collected={list(wrapped.result.collected_ids)}; '
            f'message="{wrapped.result.message}"'
        )
        self._after_collection_attempt(drive_completed=drive_completed)

    def _after_collection_attempt(self, drive_completed):
        attempted_group = list(self.active_group_ids)
        if drive_completed:
            # Real mission logic assumes the attempted pass is done and moves on.
            # It never uses Gazebo-confirmed collected IDs to decide this.
            self.attempted_ids.update(attempted_group)

        self._clear_active_target()

        # Immediately chain to another currently visible eligible shuttle.
        for next_id, detection in self.visible_ordered:
            if next_id not in self.attempted_ids:
                self.acquire_target(next_id, detection, continue_scan=False)
                return

        self.start_scan(MissionState.LOCAL_SCAN)

    # ------------------------------------------------------------------
    # Patrol progression
    # ------------------------------------------------------------------

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
