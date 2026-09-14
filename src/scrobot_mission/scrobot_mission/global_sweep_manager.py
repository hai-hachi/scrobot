#!/usr/bin/env python3

import math
import time
from enum import Enum, auto

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scrobot_interfaces.action import ApproachTag, Relocalize
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray

from scrobot_mission.global_sweep_planner import (
    generate_snake_sweep,
    plan_collection_order,
    reverse_oriented_polyline,
)


class SweepState(Enum):
    IDLE = auto()
    INITIAL_TAG_APPROACH = auto()
    INITIAL_RELOCALIZATION = auto()
    STARTING_NAV2 = auto()
    GLOBAL_SWEEP = auto()
    PLAN_COLLECTION_ROUTE = auto()
    COLLECT_ROUTE = auto()
    COMPLETE = auto()
    ERROR = auto()


def detection_position(detection):
    if detection.results:
        p = detection.results[0].pose.pose.position
    else:
        p = detection.bbox.center.position
    return float(p.x), float(p.y), float(p.z)


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GlobalSweepManager(Node):
    """Survey the whole court, freeze shuttle map, plan once, then collect."""

    def __init__(self):
        super().__init__('global_sweep_manager')

        # Frames and court.
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('autostart', True)
        self.declare_parameter('court_length', 13.40)
        self.declare_parameter('court_width', 6.10)
        self.declare_parameter('tf_timeout', 0.05)

        # Coverage path. Camera sees roughly 3 m, but use a deliberately
        # overlapping lane spacing for robust floor coverage.
        self.declare_parameter('sweep_lane_spacing', 2.20)
        self.declare_parameter('sweep_waypoint_spacing', 0.75)
        self.declare_parameter('sweep_margin_x', 0.45)
        self.declare_parameter('sweep_margin_y', 0.45)

        # Collection path geometry and route optimizer.
        self.declare_parameter('pickup_offset_x', 0.165)
        self.declare_parameter('collection_overrun', 0.03)
        self.declare_parameter('two_opt_max_passes', 100)

        # Persistent tracker is the survey map source.
        self.declare_parameter(
            'tracked_shuttles_topic',
            '/perception/tracked_shuttles',
        )

        # Initial absolute localization remains the already validated startup.
        self.declare_parameter('initial_global_localization', True)
        self.declare_parameter('initial_tag_id', -1)
        self.declare_parameter('tag_approach_distance', 1.70)
        self.declare_parameter('tag_approach_timeout', 60.0)
        self.declare_parameter('relocalize_sample_count', 15)
        self.declare_parameter('relocalize_timeout', 15.0)
        self.declare_parameter('relocalize_retries', 4)

        # Nav2 lifecycle / action retry policy.
        self.declare_parameter(
            'nav2_lifecycle_service',
            '/lifecycle_manager_navigation/manage_nodes',
        )
        self.declare_parameter('nav2_startup_timeout', 30.0)
        self.declare_parameter('nav2_startup_retries', 3)
        self.declare_parameter('nav2_activation_guard', 0.20)
        self.declare_parameter('route_goal_retries', 2)
        self.declare_parameter('control_period', 0.10)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.autostart = bool(self.get_parameter('autostart').value)
        self.court_length = float(self.get_parameter('court_length').value)
        self.court_width = float(self.get_parameter('court_width').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)

        self.sweep_lane_spacing = float(
            self.get_parameter('sweep_lane_spacing').value
        )
        self.sweep_waypoint_spacing = float(
            self.get_parameter('sweep_waypoint_spacing').value
        )
        self.sweep_margin_x = float(self.get_parameter('sweep_margin_x').value)
        self.sweep_margin_y = float(self.get_parameter('sweep_margin_y').value)

        self.pickup_offset_x = float(
            self.get_parameter('pickup_offset_x').value
        )
        self.collection_overrun = float(
            self.get_parameter('collection_overrun').value
        )
        self.two_opt_max_passes = int(
            self.get_parameter('two_opt_max_passes').value
        )
        self.tracked_shuttles_topic = str(
            self.get_parameter('tracked_shuttles_topic').value
        )

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
        self.nav2_activation_guard = float(
            self.get_parameter('nav2_activation_guard').value
        )
        self.route_goal_retries = int(
            self.get_parameter('route_goal_retries').value
        )
        control_period = float(self.get_parameter('control_period').value)

        if self.sweep_lane_spacing <= 0.0 or self.sweep_waypoint_spacing <= 0.0:
            raise ValueError('Sweep spacings must be positive.')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = SweepState.IDLE
        self.started = False

        # Survey / frozen map.
        self.sweep_route = []
        self.sweep_metadata = {}
        self.recorded_shuttles = {}  # latest confirmed track ID -> (x,y,z)
        self.frozen_shuttles = {}
        self.collection_order = []
        self.collection_route = []

        # Startup actions.
        self.approach_goal_handle = None
        self.relocalize_goal_handle = None
        self.relocalize_retry_count = 0
        self.last_relocalize_tag = -1

        # Nav2 lifecycle.
        self.nav2_startup_future = None
        self.nav2_startup_attempts = 0
        self.nav2_startup_started = None
        self.nav2_startup_complete = False
        self.nav2_activation_ready_time = None

        # NavigateThroughPoses route action.
        self.route_goal_handle = None
        self.route_goal_pending = False
        self.route_kind = ''
        self.route_retry_count = 0
        self.active_route_poses = []

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.state_pub = self.create_publisher(
            String, '/mission/state', transient_qos
        )
        self.goal_pub = self.create_publisher(
            PoseStamped, '/mission/current_goal', transient_qos
        )
        self.sweep_path_pub = self.create_publisher(
            Path, '/mission/sweep_path', transient_qos
        )
        self.collection_path_pub = self.create_publisher(
            Path, '/mission/collection_route', transient_qos
        )
        self.recorded_pub = self.create_publisher(
            PoseArray, '/mission/recorded_shuttles', transient_qos
        )

        self.create_subscription(
            Detection3DArray,
            self.tracked_shuttles_topic,
            self.tracked_shuttles_callback,
            reliable_qos,
        )

        self.approach_client = ActionClient(self, ApproachTag, '/approach_tag')
        self.relocalize_client = ActionClient(self, Relocalize, '/relocalize')
        self.route_client = ActionClient(
            self, NavigateThroughPoses, '/navigate_through_poses'
        )
        self.nav2_lifecycle_client = self.create_client(
            ManageLifecycleNodes,
            self.nav2_lifecycle_service,
        )

        self.control_timer = self.create_timer(
            max(0.02, control_period), self.control_loop
        )

        self._build_sweep_route()
        self.publish_state()
        self.get_logger().info(
            'Global sweep mission ready: survey entire court -> freeze tracks -> '
            'nearest-neighbor + 2-opt route -> continuous collection pass.'
        )

    # ==================================================================
    # State and visualization
    # ==================================================================

    def set_state(self, state):
        if self.state != state:
            self.get_logger().info(f'{self.state.name} -> {state.name}')
        self.state = state
        self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    def _pose_stamped(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        return msg

    def _publish_path(self, route, publisher):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.poses = [self._pose_stamped(x, y, yaw) for x, y, yaw in route]
        publisher.publish(msg)

    def _publish_recorded(self, positions):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for track_id in sorted(positions, key=lambda value: int(value) if str(value).isdigit() else str(value)):
            x, y, z = positions[track_id]
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.recorded_pub.publish(msg)

    # ==================================================================
    # Sweep generation and perception recording
    # ==================================================================

    def _build_sweep_route(self):
        route, metadata = generate_snake_sweep(
            self.court_length,
            self.court_width,
            self.sweep_lane_spacing,
            self.sweep_waypoint_spacing,
            margin_x=self.sweep_margin_x,
            margin_y=self.sweep_margin_y,
        )
        self.sweep_route = route
        self.sweep_metadata = metadata
        self._publish_path(self.sweep_route, self.sweep_path_pub)
        self.get_logger().info(
            f'Sweep generated: {metadata["lane_count"]} lanes, '
            f'{metadata["actual_lane_spacing"]:.2f} m actual lane spacing, '
            f'{metadata["waypoint_count"]} waypoints.'
        )

    def tracked_shuttles_callback(self, msg):
        # Survey map changes only during GLOBAL_SWEEP. At the end it is frozen.
        if self.state != SweepState.GLOBAL_SWEEP:
            return

        changed = False
        for detection in msg.detections:
            track_id = detection.id.strip()
            if not track_id:
                continue
            x, y, z = detection_position(detection)
            old = self.recorded_shuttles.get(track_id)
            self.recorded_shuttles[track_id] = (x, y, z)
            if old is None or math.hypot(x - old[0], y - old[1]) > 0.02:
                changed = True

        if changed:
            self._publish_recorded(self.recorded_shuttles)

    def _robot_pose(self):
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

    def _orient_sweep_for_start(self):
        robot = self._robot_pose()
        if robot is None or len(self.sweep_route) < 2:
            return
        rx, ry, _ = robot
        sx, sy, _ = self.sweep_route[0]
        ex, ey, _ = self.sweep_route[-1]
        if math.hypot(ex - rx, ey - ry) < math.hypot(sx - rx, sy - ry):
            self.sweep_route = reverse_oriented_polyline(self.sweep_route)
            self._publish_path(self.sweep_route, self.sweep_path_pub)
            self.get_logger().info(
                'Reversed sweep route because its far endpoint is closer to the '
                'post-localization robot pose.'
            )

    # ==================================================================
    # Main state progression
    # ==================================================================

    def control_loop(self):
        if self.state in (SweepState.COMPLETE, SweepState.ERROR):
            return

        if not self.started:
            self.started = True
            if not self.autostart:
                return
            if self.initial_global_localization:
                self.set_state(SweepState.INITIAL_TAG_APPROACH)
            else:
                self.begin_nav2_startup()

        if self.state == SweepState.INITIAL_TAG_APPROACH:
            self._try_start_approach()
        elif self.state == SweepState.INITIAL_RELOCALIZATION:
            self._try_start_relocalize()
        elif self.state == SweepState.STARTING_NAV2:
            self._process_nav2_startup()

    # ==================================================================
    # Initial localization
    # ==================================================================

    def _try_start_approach(self):
        if self.approach_goal_handle is not None:
            return
        if not self.approach_client.server_is_ready():
            return

        goal = ApproachTag.Goal()
        goal.preferred_tag_id = self.initial_tag_id
        goal.target_distance = self.tag_approach_distance
        goal.timeout_sec = self.tag_approach_timeout
        self.approach_goal_handle = 'pending'
        self.approach_client.send_goal_async(goal).add_done_callback(
            self._approach_goal_response
        )

    def _approach_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.approach_goal_handle = None
            self.enter_error('Initial /approach_tag goal rejected.')
            return
        self.approach_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._approach_result)

    def _approach_result(self, future):
        wrapped = future.result()
        self.approach_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            self.enter_error('Initial /approach_tag failed.')
            return
        self.last_relocalize_tag = int(wrapped.result.tag_id)
        self.relocalize_retry_count = 0
        self.set_state(SweepState.INITIAL_RELOCALIZATION)

    def _try_start_relocalize(self):
        if self.relocalize_goal_handle is not None:
            return
        if not self.relocalize_client.server_is_ready():
            return

        goal = Relocalize.Goal()
        goal.preferred_tag_id = self.last_relocalize_tag
        goal.sample_count = self.relocalize_sample_count
        goal.timeout_sec = self.relocalize_timeout
        self.relocalize_goal_handle = 'pending'
        self.relocalize_client.send_goal_async(goal).add_done_callback(
            self._relocalize_goal_response
        )

    def _relocalize_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.relocalize_goal_handle = None
            self._retry_relocalize('goal rejected')
            return
        self.relocalize_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._relocalize_result)

    def _relocalize_result(self, future):
        wrapped = future.result()
        self.relocalize_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            self._retry_relocalize('action failed')
            return
        self.begin_nav2_startup()

    def _retry_relocalize(self, reason):
        if self.relocalize_retry_count >= self.relocalize_retries:
            self.enter_error(
                f'Initial relocalization failed after retries: {reason}'
            )
            return
        self.relocalize_retry_count += 1
        self.get_logger().warn(
            f'Relocalize failed ({reason}); retry '
            f'{self.relocalize_retry_count}/{self.relocalize_retries}.'
        )

    # ==================================================================
    # Nav2 lifecycle startup gate
    # ==================================================================

    def begin_nav2_startup(self):
        self.nav2_startup_future = None
        self.nav2_startup_attempts = 0
        self.nav2_startup_started = time.monotonic()
        self.nav2_startup_complete = False
        self.nav2_activation_ready_time = None
        self.set_state(SweepState.STARTING_NAV2)

    def _process_nav2_startup(self):
        now = time.monotonic()
        if now - self.nav2_startup_started > self.nav2_startup_timeout:
            self.enter_error('Nav2 lifecycle startup timed out.')
            return

        if not self.nav2_startup_complete:
            if self.nav2_startup_future is not None:
                if not self.nav2_startup_future.done():
                    return
                try:
                    response = self.nav2_startup_future.result()
                except Exception as exc:
                    response = None
                    self.get_logger().warn(f'Nav2 STARTUP service error: {exc}')
                self.nav2_startup_future = None

                if response is not None and response.success:
                    self.nav2_startup_complete = True
                    self.nav2_activation_ready_time = now + self.nav2_activation_guard
                    return

                if self.nav2_startup_attempts >= self.nav2_startup_retries:
                    self.enter_error('Nav2 STARTUP failed after retries.')
                    return

            if not self.nav2_lifecycle_client.service_is_ready():
                return
            if self.nav2_startup_future is None:
                request = ManageLifecycleNodes.Request()
                request.command = ManageLifecycleNodes.Request.STARTUP
                self.nav2_startup_attempts += 1
                self.get_logger().info(
                    f'Requesting Nav2 lifecycle STARTUP '
                    f'({self.nav2_startup_attempts}/{self.nav2_startup_retries}).'
                )
                self.nav2_startup_future = self.nav2_lifecycle_client.call_async(request)
            return

        if now < self.nav2_activation_ready_time:
            return
        if not self.route_client.server_is_ready():
            return

        self._orient_sweep_for_start()
        self.recorded_shuttles = {}
        self._publish_recorded(self.recorded_shuttles)
        self.set_state(SweepState.GLOBAL_SWEEP)
        self._send_route(self.sweep_route, 'sweep')

    # ==================================================================
    # NavigateThroughPoses execution
    # ==================================================================

    def _route_to_pose_stamped(self, route):
        return [self._pose_stamped(x, y, yaw) for x, y, yaw in route]

    def _send_route(self, route, kind):
        if not route:
            if kind == 'collection':
                self.finish_mission()
            else:
                self.enter_error('Cannot execute an empty sweep route.')
            return
        if self.route_goal_pending or self.route_goal_handle is not None:
            self.enter_error('Attempted to overlap NavigateThroughPoses goals.')
            return

        self.route_kind = kind
        self.active_route_poses = list(route)
        self.route_retry_count = 0
        self._send_active_route()

    def _send_active_route(self):
        goal = NavigateThroughPoses.Goal()
        goal.poses = self._route_to_pose_stamped(self.active_route_poses)
        self.route_goal_pending = True

        last = goal.poses[-1]
        self.goal_pub.publish(last)
        self.route_client.send_goal_async(goal).add_done_callback(
            self._route_goal_response
        )

    def _route_goal_response(self, future):
        self.route_goal_pending = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._retry_route('goal rejected')
            return
        self.route_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._route_result)

    def _route_result(self, future):
        wrapped = future.result()
        self.route_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._retry_route(f'status={wrapped.status}')
            return

        kind = self.route_kind
        self.route_kind = ''
        self.active_route_poses = []

        if kind == 'sweep':
            self._finish_sweep_and_plan()
        elif kind == 'collection':
            self.finish_mission()
        else:
            self.enter_error(f'Unknown completed route kind: {kind}')

    def _retry_route(self, reason):
        if self.route_retry_count >= self.route_goal_retries:
            self.enter_error(
                f'NavigateThroughPoses {self.route_kind} failed: {reason}'
            )
            return
        self.route_retry_count += 1
        self.get_logger().warn(
            f'Route {self.route_kind} failed ({reason}); retry '
            f'{self.route_retry_count}/{self.route_goal_retries}.'
        )
        self._send_active_route()

    # ==================================================================
    # Freeze survey map -> TSP-style route -> collection pass
    # ==================================================================

    def _finish_sweep_and_plan(self):
        self.frozen_shuttles = dict(self.recorded_shuttles)
        self._publish_recorded(self.frozen_shuttles)
        self.get_logger().info(
            f'Global sweep complete: froze {len(self.frozen_shuttles)} '
            'confirmed shuttle tracks.'
        )
        self.set_state(SweepState.PLAN_COLLECTION_ROUTE)

        robot = self._robot_pose()
        if robot is None:
            self.enter_error('No map->base pose available for collection planning.')
            return

        if not self.frozen_shuttles:
            self.get_logger().info('No shuttles recorded during sweep.')
            self.finish_mission()
            return

        order, stats = plan_collection_order(
            (robot[0], robot[1]),
            self.frozen_shuttles,
            max_two_opt_passes=self.two_opt_max_passes,
        )
        self.collection_order = order
        self.collection_route = self._build_collection_route(
            (robot[0], robot[1]), order
        )
        self._publish_path(self.collection_route, self.collection_path_pub)

        self.get_logger().info(
            f'Collection route planned for {len(order)} shuttles: '
            f'nearest-neighbor={stats["nearest_neighbor_length"]:.2f} m, '
            f'2-opt={stats["optimized_length"]:.2f} m, '
            f'order={order}.'
        )

        self.set_state(SweepState.COLLECT_ROUTE)
        self._send_route(self.collection_route, 'collection')

    def _build_collection_route(self, start_xy, order):
        route = []
        px, py = start_xy
        base_to_target = self.pickup_offset_x - self.collection_overrun

        for track_id in order:
            tx, ty, _ = self.frozen_shuttles[track_id]
            dx = tx - px
            dy = ty - py
            distance = math.hypot(dx, dy)
            if distance <= 1e-6:
                continue

            ux = dx / distance
            uy = dy / distance
            yaw = math.atan2(uy, ux)

            # Place base so pickup_link passes slightly beyond the shuttle.
            bx = tx - base_to_target * ux
            by = ty - base_to_target * uy
            route.append((bx, by, yaw))
            px, py = bx, by

        return route

    # ==================================================================
    # Terminal states
    # ==================================================================

    def finish_mission(self):
        self.set_state(SweepState.COMPLETE)

    def enter_error(self, reason):
        self.get_logger().error(reason)
        self.set_state(SweepState.ERROR)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalSweepManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
