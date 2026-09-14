#!/usr/bin/env python3

import copy
import math
from enum import Enum, auto

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav2_msgs.action import FollowPath, NavigateToPose, Spin
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scrobot_interfaces.action import ApproachTag, LocalCollect, Relocalize
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray

from scrobot_mission.patrol_sweep_path import (
    choose_fixed_relocalization_stops,
    generate_sweep_path,
    nearest_path_index,
    wrap_angle,
)


class MissionState(Enum):
    IDLE = auto()
    INITIAL_TAG_APPROACH = auto()
    INITIAL_RELOCALIZATION = auto()
    STARTING_NAV2 = auto()
    JOIN_SWEEP = auto()
    SWEEPING = auto()
    TURN_TO_TAG = auto()
    RELOCALIZING = auto()
    RESTORE_SWEEP_HEADING = auto()
    LOCAL_COLLECT = auto()
    RETURN_TO_SWEEP = auto()
    COMPLETE = auto()
    ERROR = auto()


class SweepMissionManager(Node):
    """Four-pass coverage sweep with odom-only interruptible collection sprees."""

    def __init__(self):
        super().__init__('sweep_mission_manager')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('raw_detection_topic', '/perception/shuttle_detections_3d')
        self.declare_parameter('autostart', True)
        self.declare_parameter('tf_timeout', 0.05)

        self.declare_parameter('court_length', 13.40)
        self.declare_parameter('court_width', 6.10)
        self.declare_parameter('sweep_passes', 4)
        self.declare_parameter('sweep_extension', 1.0)
        self.declare_parameter('sweep_path_resolution', 0.25)
        self.declare_parameter('turn_samples', 12)

        self.declare_parameter('pole_x', 0.0)
        self.declare_parameter('left_pole_y', 3.05)
        self.declare_parameter('right_pole_y', -3.05)
        self.declare_parameter('tag_mount_radius', 0.075)
        self.declare_parameter('tag_inward_angle_deg', 45.0)
        self.declare_parameter('fixed_stop_max_tag_distance', 4.0)

        self.declare_parameter('initial_tag_id', -1)
        self.declare_parameter('tag_approach_distance', 1.70)
        self.declare_parameter('tag_approach_timeout', 60.0)
        self.declare_parameter('relocalize_sample_count', 15)
        self.declare_parameter('relocalize_timeout', 15.0)
        self.declare_parameter('relocalize_due_distance', 15.0)

        self.declare_parameter('local_collect_timeout', 120.0)
        self.declare_parameter('spin_time_allowance', 10.0)
        self.declare_parameter('nav2_lifecycle_service', '/lifecycle_manager_navigation/manage_nodes')
        self.declare_parameter('action_retry_period', 0.25)
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('goal_checker_id', '')
        self.declare_parameter('progress_checker_id', '')

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.raw_detection_topic = str(self.get_parameter('raw_detection_topic').value)
        self.autostart = bool(self.get_parameter('autostart').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.court_length = float(self.get_parameter('court_length').value)
        self.court_width = float(self.get_parameter('court_width').value)
        self.sweep_passes = int(self.get_parameter('sweep_passes').value)
        self.sweep_extension = float(self.get_parameter('sweep_extension').value)
        self.sweep_path_resolution = float(self.get_parameter('sweep_path_resolution').value)
        self.turn_samples = int(self.get_parameter('turn_samples').value)
        self.pole_x = float(self.get_parameter('pole_x').value)
        self.left_pole_y = float(self.get_parameter('left_pole_y').value)
        self.right_pole_y = float(self.get_parameter('right_pole_y').value)
        self.tag_mount_radius = float(self.get_parameter('tag_mount_radius').value)
        self.tag_inward_angle_deg = float(self.get_parameter('tag_inward_angle_deg').value)
        self.fixed_stop_max_tag_distance = float(
            self.get_parameter('fixed_stop_max_tag_distance').value
        )
        self.initial_tag_id = int(self.get_parameter('initial_tag_id').value)
        self.tag_approach_distance = float(self.get_parameter('tag_approach_distance').value)
        self.tag_approach_timeout = float(self.get_parameter('tag_approach_timeout').value)
        self.relocalize_sample_count = int(self.get_parameter('relocalize_sample_count').value)
        self.relocalize_timeout = float(self.get_parameter('relocalize_timeout').value)
        self.relocalize_due_distance = float(self.get_parameter('relocalize_due_distance').value)
        self.local_collect_timeout = float(self.get_parameter('local_collect_timeout').value)
        self.spin_time_allowance = float(self.get_parameter('spin_time_allowance').value)
        self.action_retry_period = float(self.get_parameter('action_retry_period').value)
        self.controller_id = str(self.get_parameter('controller_id').value)
        self.goal_checker_id = str(self.get_parameter('goal_checker_id').value)
        self.progress_checker_id = str(self.get_parameter('progress_checker_id').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = MissionState.IDLE
        self.start_tag_id = self.initial_tag_id
        self.sweep_points = []
        self.relocalization_stops = []
        self.current_path_index = 0
        self.active_relocalization_stop = None
        self.relocalization_due = False

        self.checkpoint_pose = None
        self.checkpoint_index = 0
        self.diversion_pending = False

        self.last_odom_xy = None
        self.distance_since_relocalize = 0.0

        self.approach_goal_handle = None
        self.relocalize_goal_handle = None
        self.navigate_goal_handle = None
        self.follow_goal_handle = None
        self.spin_goal_handle = None
        self.collect_goal_handle = None
        self.follow_purpose = ''
        self.follow_cancel_reason = ''
        self.nav_cancel_reason = ''

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.state_pub = self.create_publisher(String, '/mission/state', state_qos)
        self.path_pub = self.create_publisher(Path, '/mission/sweep_path', state_qos)
        self.stops_pub = self.create_publisher(PoseArray, '/mission/relocalization_stops', state_qos)
        self.goal_pub = self.create_publisher(PoseStamped, '/mission/current_goal', state_qos)

        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, reliable_qos)
        self.create_subscription(
            Detection3DArray,
            self.raw_detection_topic,
            self._detections_cb,
            qos_profile_sensor_data,
        )

        self.approach_client = ActionClient(self, ApproachTag, '/approach_tag')
        self.relocalize_client = ActionClient(self, Relocalize, '/relocalize')
        self.navigate_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.follow_client = ActionClient(self, FollowPath, '/follow_path')
        self.spin_client = ActionClient(self, Spin, '/spin')
        self.collect_client = ActionClient(self, LocalCollect, '/local_collect')
        self.lifecycle_client = self.create_client(
            ManageLifecycleNodes,
            str(self.get_parameter('nav2_lifecycle_service').value),
        )

        self.start_timer = self.create_timer(0.10, self._start_once)
        self.tick_timer = self.create_timer(self.action_retry_period, self._tick)
        self._publish_state()

    # ------------------------------------------------------------------
    # State / TF / geometry helpers
    # ------------------------------------------------------------------

    def _set_state(self, state):
        if self.state != state:
            self.get_logger().info(f'{self.state.name} -> {state.name}')
        self.state = state
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

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
        q = tf.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        pose = Pose()
        pose.position.x = float(tf.translation.x)
        pose.position.y = float(tf.translation.y)
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        return pose, yaw

    @staticmethod
    def _pose_xy_yaw(x, y, yaw):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _pose_stamped(self, pose):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose = copy.deepcopy(pose)
        return msg

    def _path_from_points(self, start_index, end_index=None, final_stop=None):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        if not self.sweep_points:
            return path
        start = max(0, min(int(start_index), len(self.sweep_points) - 1))
        end = len(self.sweep_points) - 1 if end_index is None else min(
            int(end_index), len(self.sweep_points) - 1
        )
        for p in self.sweep_points[start:end + 1]:
            path.poses.append(self._pose_stamped(self._pose_xy_yaw(p.x, p.y, p.yaw)))
        if final_stop is not None:
            exact = self._pose_xy_yaw(
                final_stop.x, final_stop.y, final_stop.sweep_yaw
            )
            path.poses.append(self._pose_stamped(exact))
        return path

    def _generate_sweep(self):
        if self.start_tag_id not in (0, 1, 2, 3):
            self.start_tag_id = 0
        self.sweep_points = generate_sweep_path(
            court_length=self.court_length,
            court_width=self.court_width,
            passes=self.sweep_passes,
            sweep_extension=self.sweep_extension,
            turn_samples=self.turn_samples,
            path_resolution=self.sweep_path_resolution,
            start_tag_id=self.start_tag_id,
        )
        self.relocalization_stops = choose_fixed_relocalization_stops(
            self.sweep_points,
            court_length=self.court_length,
            pole_x=self.pole_x,
            left_pole_y=self.left_pole_y,
            right_pole_y=self.right_pole_y,
            tag_mount_radius=self.tag_mount_radius,
            inward_angle_deg=self.tag_inward_angle_deg,
            max_tag_distance=self.fixed_stop_max_tag_distance,
        )
        self.current_path_index = 0
        self.path_pub.publish(self._path_from_points(0))

        stops_msg = PoseArray()
        stops_msg.header.stamp = self.get_clock().now().to_msg()
        stops_msg.header.frame_id = self.frame_id
        for stop in self.relocalization_stops:
            stops_msg.poses.append(
                self._pose_xy_yaw(stop.x, stop.y, stop.face_tag_yaw)
            )
            self.get_logger().info(
                f'Fixed relocalization stop: tag={stop.tag_id}, '
                f'path_index={stop.path_index}, x={stop.x:.2f}, y={stop.y:.2f}, '
                f'face={math.degrees(stop.face_tag_yaw):.1f} deg.'
            )
        self.stops_pub.publish(stops_msg)
        if len(self.relocalization_stops) != 4:
            self.get_logger().warn(
                f'Expected 4 fixed relocalization stops but generated '
                f'{len(self.relocalization_stops)}. Check sweep/tag geometry.'
            )
        self.get_logger().info(
            f'Generated {self.sweep_passes}-pass sweep from tag {self.start_tag_id}: '
            f'{len(self.sweep_points)} path samples.'
        )

    # ------------------------------------------------------------------
    # Odom distance and shuttle-trigger handling
    # ------------------------------------------------------------------

    def _odom_cb(self, msg):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self.last_odom_xy is not None:
            step = math.hypot(x - self.last_odom_xy[0], y - self.last_odom_xy[1])
            if step < 1.0:
                self.distance_since_relocalize += step
        self.last_odom_xy = (x, y)
        if self.distance_since_relocalize >= self.relocalize_due_distance:
            self.relocalization_due = True

    def _reset_relocalization_distance(self):
        self.distance_since_relocalize = 0.0
        self.relocalization_due = False
        self.last_odom_xy = None

    def _detections_cb(self, msg):
        if not msg.detections or self.diversion_pending:
            return
        if self.state == MissionState.SWEEPING:
            pose_info = self._robot_pose()
            if pose_info is None:
                return
            pose, _ = pose_info
            self.checkpoint_pose = copy.deepcopy(pose)
            self.checkpoint_index = nearest_path_index(
                self.sweep_points,
                pose.position.x,
                pose.position.y,
                self.current_path_index,
            )
            self.current_path_index = self.checkpoint_index
            self.diversion_pending = True
            self.get_logger().info(
                f'Shuttle seen during sweep; fixed checkpoint index={self.checkpoint_index}, '
                f'x={pose.position.x:.2f}, y={pose.position.y:.2f}.'
            )
            self._cancel_follow('diversion')
        elif self.state == MissionState.RETURN_TO_SWEEP:
            self.diversion_pending = True
            self.get_logger().info(
                'Shuttle seen while returning; interrupting return but preserving '
                'the original sweep checkpoint.'
            )
            self._cancel_navigation('diversion')

    # ------------------------------------------------------------------
    # Startup sequence
    # ------------------------------------------------------------------

    def _start_once(self):
        self.start_timer.cancel()
        if not self.autostart:
            return
        self._set_state(MissionState.INITIAL_TAG_APPROACH)
        self._send_initial_approach()

    def _send_initial_approach(self):
        if not self.approach_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('/approach_tag not ready; retrying.')
            self.start_timer.reset()
            return
        goal = ApproachTag.Goal()
        goal.preferred_tag_id = self.initial_tag_id
        goal.target_distance = self.tag_approach_distance
        goal.timeout_sec = self.tag_approach_timeout
        future = self.approach_client.send_goal_async(goal)
        future.add_done_callback(self._approach_goal_response)

    def _approach_goal_response(self, future):
        self.approach_goal_handle = future.result()
        if self.approach_goal_handle is None or not self.approach_goal_handle.accepted:
            self._fail('Initial tag approach rejected.')
            return
        result_future = self.approach_goal_handle.get_result_async()
        result_future.add_done_callback(self._approach_result)

    def _approach_result(self, future):
        wrapped = future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            self._fail(f'Initial tag approach failed: {wrapped.result.message}')
            return
        self.start_tag_id = int(wrapped.result.tag_id)
        self.get_logger().info(f'Initial approach selected tag {self.start_tag_id}.')
        self._set_state(MissionState.INITIAL_RELOCALIZATION)
        self._send_relocalize(self.start_tag_id, initial=True)

    def _start_nav2(self):
        self._set_state(MissionState.STARTING_NAV2)
        if not self.lifecycle_client.wait_for_service(timeout_sec=2.0):
            self._fail('Nav2 lifecycle service unavailable.')
            return
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        future = self.lifecycle_client.call_async(request)
        future.add_done_callback(self._nav2_started)

    def _nav2_started(self, future):
        response = future.result()
        if response is None or not response.success:
            self._fail('Nav2 startup failed.')
            return
        self._join_sweep()

    def _join_sweep(self):
        if not self.sweep_points:
            self._fail('Sweep path is empty.')
            return
        p = self.sweep_points[0]
        self._set_state(MissionState.JOIN_SWEEP)
        self._send_navigation(self._pose_xy_yaw(p.x, p.y, p.yaw), 'join')

    # ------------------------------------------------------------------
    # Navigation / FollowPath
    # ------------------------------------------------------------------

    def _send_navigation(self, pose, purpose):
        if not self.navigate_client.wait_for_server(timeout_sec=1.0):
            self._fail('/navigate_to_pose unavailable.')
            return
        msg = self._pose_stamped(pose)
        self.goal_pub.publish(msg)
        goal = NavigateToPose.Goal()
        goal.pose = msg
        future = self.navigate_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._navigation_goal_response(f, purpose))

    def _navigation_goal_response(self, future, purpose):
        self.navigate_goal_handle = future.result()
        if self.navigate_goal_handle is None or not self.navigate_goal_handle.accepted:
            self._fail(f'Navigation goal rejected ({purpose}).')
            return
        result_future = self.navigate_goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._navigation_result(f, purpose))

    def _navigation_result(self, future, purpose):
        wrapped = future.result()
        self.navigate_goal_handle = None
        if wrapped.status == GoalStatus.STATUS_CANCELED and self.nav_cancel_reason == 'diversion':
            self.nav_cancel_reason = ''
            self._start_local_collect()
            return
        self.nav_cancel_reason = ''
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._fail(f'Navigation failed ({purpose}), status={wrapped.status}.')
            return
        if purpose == 'join':
            self.current_path_index = 0
            self._start_sweep_follow()
        elif purpose == 'return':
            self.current_path_index = self.checkpoint_index
            self.checkpoint_pose = None
            self.diversion_pending = False
            self._start_sweep_follow()

    def _cancel_navigation(self, reason):
        self.nav_cancel_reason = reason
        if self.navigate_goal_handle is not None:
            self.navigate_goal_handle.cancel_goal_async()

    def _start_sweep_follow(self):
        self._set_state(MissionState.SWEEPING)
        self.diversion_pending = False
        stop = self._next_due_stop()
        if stop is not None:
            self.active_relocalization_stop = stop
            self._send_follow(stop.path_index, 'relocalize_stop', stop)
        else:
            self.active_relocalization_stop = None
            self._send_follow(None, 'sweep_end', None)

    def _send_follow(self, end_index, purpose, stop):
        if not self.follow_client.wait_for_server(timeout_sec=1.0):
            self._fail('/follow_path unavailable.')
            return
        path = self._path_from_points(
            self.current_path_index,
            end_index=end_index,
            final_stop=stop,
        )
        if len(path.poses) < 2:
            if purpose == 'sweep_end':
                self._set_state(MissionState.COMPLETE)
                return
            self._begin_relocalization_stop()
            return
        self.follow_purpose = purpose
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id
        goal.progress_checker_id = self.progress_checker_id
        future = self.follow_client.send_goal_async(goal)
        future.add_done_callback(self._follow_goal_response)

    def _follow_goal_response(self, future):
        self.follow_goal_handle = future.result()
        if self.follow_goal_handle is None or not self.follow_goal_handle.accepted:
            self._fail('FollowPath goal rejected.')
            return
        result_future = self.follow_goal_handle.get_result_async()
        result_future.add_done_callback(self._follow_result)
        if self.diversion_pending:
            self._cancel_follow('diversion')

    def _follow_result(self, future):
        wrapped = future.result()
        self.follow_goal_handle = None
        reason = self.follow_cancel_reason
        self.follow_cancel_reason = ''
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            if reason == 'diversion':
                self._start_local_collect()
                return
            if reason == 'reschedule_relocalization':
                stop = self.active_relocalization_stop
                if stop is not None:
                    self._send_follow(stop.path_index, 'relocalize_stop', stop)
                return
            self._fail('FollowPath canceled unexpectedly.')
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._fail(f'FollowPath failed, status={wrapped.status}.')
            return
        if self.follow_purpose == 'relocalize_stop':
            if self.active_relocalization_stop is not None:
                self.current_path_index = self.active_relocalization_stop.path_index
            self._begin_relocalization_stop()
        else:
            self.current_path_index = max(0, len(self.sweep_points) - 1)
            self._set_state(MissionState.COMPLETE)
            self.get_logger().info('Sweep complete.')

    def _cancel_follow(self, reason):
        self.follow_cancel_reason = reason
        if self.follow_goal_handle is not None:
            self.follow_goal_handle.cancel_goal_async()

    def _update_sweep_progress(self):
        if self.state != MissionState.SWEEPING or not self.sweep_points:
            return
        pose_info = self._robot_pose()
        if pose_info is None:
            return
        pose, _ = pose_info
        self.current_path_index = nearest_path_index(
            self.sweep_points,
            pose.position.x,
            pose.position.y,
            self.current_path_index,
        )

    # ------------------------------------------------------------------
    # Fixed-stop relocalization
    # ------------------------------------------------------------------

    def _next_due_stop(self):
        if not self.relocalization_due:
            return None
        for stop in self.relocalization_stops:
            if stop.path_index > self.current_path_index + 1:
                return stop
        return None

    def _maybe_reschedule_for_relocalization(self):
        if (
            self.state != MissionState.SWEEPING
            or not self.relocalization_due
            or self.active_relocalization_stop is not None
            or self.follow_goal_handle is None
            or self.diversion_pending
        ):
            return
        stop = self._next_due_stop()
        if stop is None:
            return
        self.active_relocalization_stop = stop
        self.get_logger().info(
            f'Relocalization due after {self.distance_since_relocalize:.1f} m; '
            f'next fixed station is tag {stop.tag_id} at path index {stop.path_index}.'
        )
        self._cancel_follow('reschedule_relocalization')

    def _begin_relocalization_stop(self):
        stop = self.active_relocalization_stop
        if stop is None:
            self._start_sweep_follow()
            return
        self._set_state(MissionState.TURN_TO_TAG)
        self._send_spin_to_absolute(stop.face_tag_yaw, 'face_tag')

    def _send_spin_to_absolute(self, target_yaw, purpose):
        pose_info = self._robot_pose()
        if pose_info is None:
            self._fail('Cannot read robot yaw for relocalization turn.')
            return
        _, current_yaw = pose_info
        delta = wrap_angle(target_yaw - current_yaw)
        if abs(delta) < math.radians(1.0):
            if purpose == 'face_tag':
                self._set_state(MissionState.RELOCALIZING)
                self._send_relocalize(self.active_relocalization_stop.tag_id, initial=False)
            else:
                self.active_relocalization_stop = None
                self._start_sweep_follow()
            return
        if not self.spin_client.wait_for_server(timeout_sec=1.0):
            self._fail('/spin unavailable.')
            return
        goal = Spin.Goal()
        goal.target_yaw = float(delta)
        allowance = DurationMsg()
        allowance.sec = int(math.ceil(self.spin_time_allowance))
        goal.time_allowance = allowance
        future = self.spin_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._spin_goal_response(f, purpose))

    def _spin_goal_response(self, future, purpose):
        self.spin_goal_handle = future.result()
        if self.spin_goal_handle is None or not self.spin_goal_handle.accepted:
            self._fail(f'Spin rejected ({purpose}).')
            return
        result_future = self.spin_goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._spin_result(f, purpose))

    def _spin_result(self, future, purpose):
        wrapped = future.result()
        self.spin_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._fail(f'Spin failed ({purpose}), status={wrapped.status}.')
            return
        if purpose == 'face_tag':
            self._set_state(MissionState.RELOCALIZING)
            self._send_relocalize(self.active_relocalization_stop.tag_id, initial=False)
        else:
            self.active_relocalization_stop = None
            self._start_sweep_follow()

    def _send_relocalize(self, tag_id, initial):
        if not self.relocalize_client.wait_for_server(timeout_sec=1.0):
            self._fail('/relocalize unavailable.')
            return
        goal = Relocalize.Goal()
        goal.preferred_tag_id = int(tag_id)
        goal.sample_count = self.relocalize_sample_count
        goal.timeout_sec = self.relocalize_timeout
        future = self.relocalize_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._relocalize_goal_response(f, initial))

    def _relocalize_goal_response(self, future, initial):
        self.relocalize_goal_handle = future.result()
        if self.relocalize_goal_handle is None or not self.relocalize_goal_handle.accepted:
            self._fail('Relocalize goal rejected.')
            return
        result_future = self.relocalize_goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._relocalize_result(f, initial))

    def _relocalize_result(self, future, initial):
        wrapped = future.result()
        self.relocalize_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            self._fail(f'Relocalization failed: {wrapped.result.message}')
            return
        self._reset_relocalization_distance()
        if initial:
            if wrapped.result.used_tag_ids:
                self.start_tag_id = int(wrapped.result.used_tag_ids[0])
            self._generate_sweep()
            self._start_nav2()
            return

        stop = self.active_relocalization_stop
        self.get_logger().info(
            f'Fixed-stop relocalization complete using tag {stop.tag_id}; '
            f'delta=({wrapped.result.delta_x:.3f}, {wrapped.result.delta_y:.3f}, '
            f'{math.degrees(wrapped.result.delta_yaw):.2f} deg).'
        )
        self._set_state(MissionState.RESTORE_SWEEP_HEADING)
        self._send_spin_to_absolute(stop.sweep_yaw, 'restore_sweep')

    # ------------------------------------------------------------------
    # Local collect and exact checkpoint return
    # ------------------------------------------------------------------

    def _start_local_collect(self):
        if self.checkpoint_pose is None:
            self.diversion_pending = False
            self._start_sweep_follow()
            return
        self._set_state(MissionState.LOCAL_COLLECT)
        if not self.collect_client.wait_for_server(timeout_sec=1.0):
            self._fail('/local_collect unavailable.')
            return
        goal = LocalCollect.Goal()
        goal.timeout_sec = self.local_collect_timeout
        future = self.collect_client.send_goal_async(goal)
        future.add_done_callback(self._collect_goal_response)

    def _collect_goal_response(self, future):
        self.collect_goal_handle = future.result()
        if self.collect_goal_handle is None or not self.collect_goal_handle.accepted:
            self._fail('LocalCollect goal rejected.')
            return
        result_future = self.collect_goal_handle.get_result_async()
        result_future.add_done_callback(self._collect_result)

    def _collect_result(self, future):
        wrapped = future.result()
        self.collect_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            self._fail(f'Local collection failed: {wrapped.result.message}')
            return
        self.get_logger().info(
            f'Local spree finished after {wrapped.result.targets_attempted} target attempt(s); '
            'returning to the original sweep checkpoint.'
        )
        self.diversion_pending = False
        self._set_state(MissionState.RETURN_TO_SWEEP)
        self._send_navigation(self.checkpoint_pose, 'return')

    # ------------------------------------------------------------------
    # Timer / failure
    # ------------------------------------------------------------------

    def _tick(self):
        if self.state == MissionState.SWEEPING:
            self._update_sweep_progress()
            self._maybe_reschedule_for_relocalization()

    def _fail(self, message):
        self.get_logger().error(message)
        self._set_state(MissionState.ERROR)


def main(args=None):
    rclpy.init(args=args)
    node = SweepMissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
