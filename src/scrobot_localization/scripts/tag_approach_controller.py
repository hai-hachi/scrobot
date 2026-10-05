#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rclpy.time import Time
from scrobot_interfaces.action import ApproachTag
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import (
    concatenate_matrices,
    euler_from_quaternion,
    inverse_matrix,
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_matrix,
)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_difference(target, source):
    return wrap_angle(target - source)


def clamp(value, low, high):
    return max(low, min(high, value))


def circular_mean(values):
    if not values:
        return 0.0
    return math.atan2(
        sum(math.sin(value) for value in values),
        sum(math.cos(value) for value in values),
    )


def median(values):
    ordered = sorted(values)
    n = len(ordered)
    if n == 0:
        return float('nan')
    if n % 2 == 1:
        return ordered[n // 2]
    return 0.5 * (ordered[n // 2 - 1] + ordered[n // 2])


def transform_to_matrix(transform):
    translation = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ]
    quaternion = [
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    ]
    return concatenate_matrices(
        translation_matrix(translation),
        quaternion_matrix(quaternion),
    )


def xyz_rpy_to_matrix(xyz, rpy):
    quaternion = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    return concatenate_matrices(
        translation_matrix(xyz),
        quaternion_matrix(quaternion),
    )


class TagApproachController(Node):
    """
    Stationary multi-frame tag selection with selectable local approach law.

    The desired pose is expressed at base_link, with a default 0.90 m stand-off
    from the physical tag face. The default production strategy is pure_smc.

    Experimental strategies are exposed for controlled debug comparison:
      pure_smc       direct pose SMC
      main_branch    original direct position + final-yaw controller
      biarc_smc      tangent biarc reference path feeding the same SMC
      normal_ray_smc rotate/cross/face the tag normal ray, then SMC

    Once a tag is locked, other tags cannot steal it.
    """

    def __init__(self):
        super().__init__('tag_approach_controller')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('detections_topic', '/apriltag/detections')
        self.declare_parameter('observed_tag_prefix', 'observed_tag_')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_relocalization')

        self.declare_parameter('min_decision_margin', 10.0)
        self.declare_parameter('max_detection_distance', 10.0)
        self.declare_parameter('default_target_distance', 0.90)
        self.declare_parameter('default_timeout', 60.0)
        self.declare_parameter('tag_lost_timeout', 1.0)

        self.declare_parameter('selection_settle_time', 0.18)
        self.declare_parameter('selection_window', 0.30)
        self.declare_parameter('selection_min_samples', 3)
        self.declare_parameter('pending_detection_max_age', 0.25)

        self.declare_parameter('search_angular_velocity', 0.45)
        self.declare_parameter('search_rotation_rad', 2.0 * math.pi)
        self.declare_parameter('max_linear_velocity', 0.45)
        self.declare_parameter('max_angular_velocity', 0.75)

        # Shared nonlinear SMC pose law. The controlled point is base_link,
        # therefore the kinematic offset is c = 0.
        self.declare_parameter('smc_reference_speed', 0.50)
        self.declare_parameter('smc_lambda', 2.00)
        self.declare_parameter('smc_ks', 1.60)
        self.declare_parameter('smc_eta', 0.50)
        self.declare_parameter('smc_phi', 0.08)
        self.declare_parameter('smc_krho', 0.80)
        self.declare_parameter('heading_stop_deg', 70.0)

        self.declare_parameter('control_strategy', 'pure_smc')

        # Original main-branch direct-goal controller.
        self.declare_parameter('main_k_position', 0.80)
        self.declare_parameter('main_k_heading', 1.80)
        self.declare_parameter('main_k_final_yaw', 1.80)
        self.declare_parameter('main_drive_heading_limit_deg', 35.0)

        # Biarc reference path. A single circle cannot in general satisfy two
        # arbitrary endpoint poses, so the tangent-path experiment uses two
        # circular arcs joined with continuous tangent.
        self.declare_parameter('biarc_spacing', 0.08)
        self.declare_parameter('biarc_lookahead', 0.30)
        self.declare_parameter('biarc_d1_factor_min', 0.15)
        self.declare_parameter('biarc_d1_factor_max', 6.0)
        self.declare_parameter('biarc_d1_factor_samples', 81)
        self.declare_parameter('biarc_max_arc_sweep_deg', 175.0)

        # Deliberately simple normal-ray baseline.
        self.declare_parameter('ray_position_tolerance', 0.12)
        self.declare_parameter('ray_heading_tolerance_deg', 8.0)
        self.declare_parameter('ray_cross_speed', 0.30)
        self.declare_parameter('ray_heading_gain', 1.8)

        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance_deg', 5.0)
        self.declare_parameter('stable_time', 0.25)
        self.declare_parameter('goal_filter_alpha', 0.30)
        self.declare_parameter('control_rate', 25.0)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.detections_topic = str(self.get_parameter('detections_topic').value)
        self.observed_tag_prefix = str(
            self.get_parameter('observed_tag_prefix').value
        )
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.min_decision_margin = float(
            self.get_parameter('min_decision_margin').value
        )
        self.max_detection_distance = float(
            self.get_parameter('max_detection_distance').value
        )
        self.default_target_distance = float(
            self.get_parameter('default_target_distance').value
        )
        self.default_timeout = float(self.get_parameter('default_timeout').value)
        self.tag_lost_timeout = float(
            self.get_parameter('tag_lost_timeout').value
        )

        self.selection_settle_time = float(
            self.get_parameter('selection_settle_time').value
        )
        self.selection_window = float(
            self.get_parameter('selection_window').value
        )
        self.selection_min_samples = int(
            self.get_parameter('selection_min_samples').value
        )
        self.pending_detection_max_age = float(
            self.get_parameter('pending_detection_max_age').value
        )

        self.search_angular_velocity = float(
            self.get_parameter('search_angular_velocity').value
        )
        self.search_rotation_rad = max(
            0.1,
            float(self.get_parameter('search_rotation_rad').value),
        )
        self.max_linear_velocity = float(
            self.get_parameter('max_linear_velocity').value
        )
        self.max_angular_velocity = float(
            self.get_parameter('max_angular_velocity').value
        )
        self.smc_reference_speed = float(
            self.get_parameter('smc_reference_speed').value
        )
        self.smc_lambda = float(self.get_parameter('smc_lambda').value)
        self.smc_ks = float(self.get_parameter('smc_ks').value)
        self.smc_eta = float(self.get_parameter('smc_eta').value)
        self.smc_phi = max(
            1e-6, float(self.get_parameter('smc_phi').value)
        )
        self.smc_krho = float(self.get_parameter('smc_krho').value)
        self.heading_stop = math.radians(
            float(self.get_parameter('heading_stop_deg').value)
        )

        self.control_strategy = str(
            self.get_parameter('control_strategy').value
        ).strip().lower()
        if self.control_strategy not in (
            'pure_smc',
            'main_branch',
            'biarc_smc',
            'normal_ray_smc',
        ):
            raise ValueError(
                'control_strategy must be pure_smc, main_branch, '
                'biarc_smc, or normal_ray_smc.'
            )

        self.main_k_position = float(
            self.get_parameter('main_k_position').value
        )
        self.main_k_heading = float(
            self.get_parameter('main_k_heading').value
        )
        self.main_k_final_yaw = float(
            self.get_parameter('main_k_final_yaw').value
        )
        self.main_drive_heading_limit = math.radians(
            float(self.get_parameter('main_drive_heading_limit_deg').value)
        )

        self.biarc_spacing = max(
            0.02, float(self.get_parameter('biarc_spacing').value)
        )
        self.biarc_lookahead = max(
            self.biarc_spacing,
            float(self.get_parameter('biarc_lookahead').value),
        )
        self.biarc_d1_factor_min = max(
            1e-3, float(self.get_parameter('biarc_d1_factor_min').value)
        )
        self.biarc_d1_factor_max = max(
            self.biarc_d1_factor_min,
            float(self.get_parameter('biarc_d1_factor_max').value),
        )
        self.biarc_d1_factor_samples = max(
            3, int(self.get_parameter('biarc_d1_factor_samples').value)
        )
        self.biarc_max_arc_sweep = math.radians(
            max(
                90.0,
                min(
                    179.9,
                    float(
                        self.get_parameter(
                            'biarc_max_arc_sweep_deg'
                        ).value
                    ),
                ),
            )
        )

        self.ray_position_tolerance = max(
            0.01, float(self.get_parameter('ray_position_tolerance').value)
        )
        self.ray_heading_tolerance = math.radians(
            float(self.get_parameter('ray_heading_tolerance_deg').value)
        )
        self.ray_cross_speed = max(
            0.0, float(self.get_parameter('ray_cross_speed').value)
        )
        self.ray_heading_gain = float(
            self.get_parameter('ray_heading_gain').value
        )
        self.position_tolerance = float(
            self.get_parameter('position_tolerance').value
        )
        self.yaw_tolerance = math.radians(
            float(self.get_parameter('yaw_tolerance_deg').value)
        )
        self.stable_time = float(self.get_parameter('stable_time').value)
        self.goal_filter_alpha = float(
            self.get_parameter('goal_filter_alpha').value
        )
        control_rate = float(self.get_parameter('control_rate').value)

        if self.selection_settle_time < 0.0:
            raise ValueError('selection_settle_time must be >= 0.')
        if self.selection_window <= 0.0:
            raise ValueError('selection_window must be > 0.')
        if self.selection_min_samples < 1:
            raise ValueError('selection_min_samples must be >= 1.')
        if self.pending_detection_max_age <= 0.0:
            raise ValueError('pending_detection_max_age must be > 0.')

        # Same user-validated mount -> apriltag_ros PnP transform as global localizer.
        self.T_mount_apriltag = xyz_rpy_to_matrix(
            [0.0, 0.0, 0.0],
            [-math.pi / 2.0, 0.0, -math.pi / 2.0],
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        self.lock = threading.Lock()

        self.active = False
        self.phase = 'idle'
        self.preferred_tag_id = -1
        self.target_distance = self.default_target_distance

        self.goal_pose = None
        self.tracked_tag_id = -1
        self.last_tag_seen = None
        self.last_tag_distance = float('nan')
        self.last_tag_bearing = float('nan')
        self.last_face_angle = float('nan')
        self.stable_since = None

        # Strategy-specific experimental state.
        self.reference_path = []
        self.path_progress_index = 0
        self.ray_heading = None

        # Search progress is measured from odometry yaw so one search attempt
        # means one physical rotation even if simulation timing or smoothing varies.
        self.search_last_yaw = None
        self.search_accumulated_yaw = 0.0

        # Stationary multi-frame selection state.
        self.observe_started = None
        self.selection_samples = {}

        # Keep the latest detection and process it from the control timer.
        # This gives the associated AprilTag TF a chance to arrive first.
        self.pending_detection = None
        self.pending_detection_received = None

        self.tf_reject_count = 0
        self.last_tf_warning_time = None

        # Non-blocking action state.
        self.current_goal_handle = None
        self.action_future = None
        self.action_deadline = None

        latest_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        control_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.cmd_vel_topic,
            control_qos,
        )

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.reference_path_pub = self.create_publisher(
            Path,
            '/debug/tag_controller/reference_path',
            path_qos,
        )

        self.goal_pose_pub = self.create_publisher(
            PoseStamped,
            '/debug/tag_controller/goal_pose',
            path_qos,
        )
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self.detection_callback,
            latest_qos,
            callback_group=self.cb_group,
        )

        self.control_timer = self.create_timer(
            1.0 / control_rate,
            self.control_loop,
            callback_group=self.cb_group,
        )
        self.control_timer.cancel()

        self.action_server = ActionServer(
            self,
            ApproachTag,
            '/approach_tag',
            execute_callback=self.execute_approach,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            'Tag approach controller started: stationary multi-frame '
            f'selection, base-link target={self.default_target_distance:.2f} m, '
            f'strategy={self.control_strategy}.'
        )

    def goal_callback(self, goal_request):
        with self.lock:
            if self.active:
                return GoalResponse.REJECT
        if goal_request.preferred_tag_id not in [-1, 0, 1, 2, 3]:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_approach(self, goal_handle):
        target_distance = (
            float(goal_handle.request.target_distance)
            if goal_handle.request.target_distance > 0.0
            else self.default_target_distance
        )
        timeout = (
            float(goal_handle.request.timeout_sec)
            if goal_handle.request.timeout_sec > 0.0
            else self.default_timeout
        )

        future = Future()

        with self.lock:
            self.active = True
            self.phase = 'search'
            self.preferred_tag_id = int(goal_handle.request.preferred_tag_id)
            self.target_distance = target_distance

            self.goal_pose = None
            self.tracked_tag_id = -1
            self.last_tag_seen = None
            self.last_tag_distance = float('nan')
            self.last_tag_bearing = float('nan')
            self.last_face_angle = float('nan')
            self.stable_since = None
            self.reference_path = []
            self.path_progress_index = 0
            self.ray_heading = None
            self.search_last_yaw = None
            self.search_accumulated_yaw = 0.0

            self.observe_started = None
            self.selection_samples = {}

            self.pending_detection = None
            self.pending_detection_received = None
            self.tf_reject_count = 0
            self.last_tf_warning_time = None

            self.current_goal_handle = goal_handle
            self.action_future = future
            self.action_deadline = time.monotonic() + timeout

        self.control_timer.reset()

        self.get_logger().info(
            f'ApproachTag started: strategy={self.control_strategy}, '
            f'preferred_tag={self.preferred_tag_id}, '
            f'base_target_distance={target_distance:.2f} m'
        )

        result = await future
        return result

    # ============================================================
    # AprilTag observations
    # ============================================================

    def detection_callback(self, msg):
        with self.lock:
            if not self.active:
                return
            self.pending_detection = msg
            self.pending_detection_received = time.monotonic()

    def build_candidate(self, detection, stamp, target_distance):
        tag_id = int(detection.id)
        observed_tag_frame = self.observed_tag_prefix + str(tag_id)

        if not self.tf_buffer.can_transform(
            self.odom_frame,
            observed_tag_frame,
            stamp,
            timeout=Duration(seconds=0.0),
        ):
            return None, 'tf'
        if not self.tf_buffer.can_transform(
            self.base_frame,
            observed_tag_frame,
            stamp,
            timeout=Duration(seconds=0.0),
        ):
            return None, 'tf'

        try:
            tf_odom_tag = self.tf_buffer.lookup_transform(
                self.odom_frame,
                observed_tag_frame,
                stamp,
                timeout=Duration(seconds=0.0),
            )
            tf_base_tag = self.tf_buffer.lookup_transform(
                self.base_frame,
                observed_tag_frame,
                stamp,
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            return None, 'tf'

        T_odom_april = transform_to_matrix(tf_odom_tag.transform)
        T_base_april = transform_to_matrix(tf_base_tag.transform)

        p = tf_base_tag.transform.translation
        distance = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
        bearing = math.atan2(p.y, p.x)

        if distance > self.max_detection_distance:
            return None, 'distance'

        # Convert apriltag_ros PnP frame back to the intuitive physical mount.
        T_odom_mount = T_odom_april @ inverse_matrix(self.T_mount_apriltag)
        T_base_mount = T_base_april @ inverse_matrix(self.T_mount_apriltag)

        # Robot origin expressed in tag-mount coordinates.
        # +X_mount is the outward normal of the visible tag face.
        T_mount_base = inverse_matrix(T_base_mount)
        robot_x_in_mount = float(T_mount_base[0, 3])
        robot_y_in_mount = float(T_mount_base[1, 3])
        face_angle = math.atan2(robot_y_in_mount, robot_x_in_mount)

        # A backside solution should not be physically visible with the backing plate.
        if robot_x_in_mount <= 0.0:
            return None, 'backside'

        # Desired pose is defined directly at base_link. The rigid camera
        # offset is absorbed into the selected stand-off distance, so c = 0.
        T_mount_goal = xyz_rpy_to_matrix(
            [target_distance, 0.0, 0.0],
            [0.0, 0.0, math.pi],
        )
        T_odom_goal = T_odom_mount @ T_mount_goal

        gx = float(T_odom_goal[0, 3])
        gy = float(T_odom_goal[1, 3])
        q_goal = quaternion_from_matrix(T_odom_goal)
        _, _, gyaw = euler_from_quaternion(q_goal)
        gyaw = wrap_angle(gyaw)

        return {
            'tag_id': tag_id,
            'margin': float(detection.decision_margin),
            'distance': distance,
            'bearing': bearing,
            'face_angle': face_angle,
            'goal_x': gx,
            'goal_y': gy,
            'goal_yaw': gyaw,
        }, None

    def process_pending_detection(self):
        with self.lock:
            if not self.active or self.pending_detection is None:
                return

            msg = self.pending_detection
            received = self.pending_detection_received
            phase = self.phase
            preferred = self.preferred_tag_id
            locked_tag = self.tracked_tag_id
            target_distance = self.target_distance

        now = time.monotonic()
        if received is not None and now - received > self.pending_detection_max_age:
            with self.lock:
                if self.pending_detection is msg:
                    self.pending_detection = None
                    self.pending_detection_received = None
            return

        stamp = Time.from_msg(msg.header.stamp)
        candidates = []
        saw_tf_reject = False

        for detection in msg.detections:
            tag_id = int(detection.id)

            if tag_id not in [0, 1, 2, 3] or detection.hamming != 0:
                continue
            if float(detection.decision_margin) < self.min_decision_margin:
                continue

            if locked_tag >= 0:
                if tag_id != locked_tag:
                    continue
            elif preferred >= 0 and tag_id != preferred:
                continue

            candidate, reject_reason = self.build_candidate(
                detection,
                stamp,
                target_distance,
            )
            if candidate is not None:
                candidates.append(candidate)
            elif reject_reason == 'tf':
                saw_tf_reject = True

        # If the detection arrived before its TF, keep this frame and retry it.
        if not candidates and saw_tf_reject:
            self.tf_reject_count += 1
            if self.last_tf_warning_time is None or now - self.last_tf_warning_time >= 2.0:
                self.get_logger().warn(
                    'AprilTag detection received but matching TF is not ready yet; '
                    'retrying the same frame.'
                )
                self.last_tf_warning_time = now
            return

        with self.lock:
            if self.pending_detection is msg:
                self.pending_detection = None
                self.pending_detection_received = None

        if not candidates:
            return

        if phase == 'search':
            with self.lock:
                if self.phase == 'search':
                    self.phase = 'observe'
                    self.observe_started = now
                    self.selection_samples = {}
            self.get_logger().info(
                'Tag candidate detected. Stopping for stationary multi-frame selection.'
            )
            return

        if phase == 'observe':
            with self.lock:
                observe_started = self.observe_started

            if observe_started is None:
                return
            if now - observe_started < self.selection_settle_time:
                return

            with self.lock:
                if self.phase != 'observe':
                    return
                for candidate in candidates:
                    tag_id = candidate['tag_id']
                    self.selection_samples.setdefault(tag_id, []).append(candidate)
            return

        if locked_tag < 0:
            return

        # After locking, only the locked tag reaches this point.
        candidate = candidates[0]

        with self.lock:
            if self.tracked_tag_id != candidate['tag_id']:
                return

            self.last_tag_seen = now
            self.last_tag_distance = candidate['distance']
            self.last_tag_bearing = candidate['bearing']
            self.last_face_angle = candidate['face_angle']

            # Safe to filter now because the tag ID cannot switch.
            #
            # For biarc_smc, freeze the final pose once the reference path has
            # been generated. Otherwise the measured goal would continue
            # moving while the already-published path endpoint remained fixed.
            if (
                self.control_strategy == 'biarc_smc'
                and self.reference_path
            ):
                pass
            elif self.goal_pose is None:
                self.goal_pose = [
                    candidate['goal_x'],
                    candidate['goal_y'],
                    candidate['goal_yaw'],
                ]
            else:
                a = self.goal_filter_alpha
                old_x, old_y, old_yaw = self.goal_pose
                self.goal_pose = [
                    old_x + a * (candidate['goal_x'] - old_x),
                    old_y + a * (candidate['goal_y'] - old_y),
                    wrap_angle(
                        old_yaw
                        + a * angle_difference(candidate['goal_yaw'], old_yaw)
                    ),
                ]

    def select_and_lock_tag(self):
        with self.lock:
            samples_by_tag = {
                tag_id: list(samples)
                for tag_id, samples in self.selection_samples.items()
            }

        summaries = []

        for tag_id, samples in samples_by_tag.items():
            if len(samples) < self.selection_min_samples:
                continue

            summary = {
                'tag_id': tag_id,
                'count': len(samples),
                'face_angle': circular_mean(
                    [sample['face_angle'] for sample in samples]
                ),
                'margin': median([sample['margin'] for sample in samples]),
                'distance': median([sample['distance'] for sample in samples]),
                'bearing': circular_mean(
                    [sample['bearing'] for sample in samples]
                ),
                'goal_x': median([sample['goal_x'] for sample in samples]),
                'goal_y': median([sample['goal_y'] for sample in samples]),
                'goal_yaw': circular_mean(
                    [sample['goal_yaw'] for sample in samples]
                ),
            }
            summaries.append(summary)

        if not summaries:
            return False

        # Primary objective: closest to normal incidence. Then prefer stronger
        # decision margin, and finally the nearer tag.
        best = min(
            summaries,
            key=lambda item: (
                abs(item['face_angle']),
                -item['margin'],
                item['distance'],
            ),
        )

        with self.lock:
            if not self.active or self.phase != 'observe':
                return False

            self.tracked_tag_id = best['tag_id']
            self.goal_pose = [
                best['goal_x'],
                best['goal_y'],
                best['goal_yaw'],
            ]
            self.last_tag_seen = time.monotonic()
            self.last_tag_distance = best['distance']
            self.last_tag_bearing = best['bearing']
            self.last_face_angle = best['face_angle']
            self.reference_path = []
            self.path_progress_index = 0
            self.ray_heading = None

            if self.control_strategy == 'main_branch':
                self.phase = 'main_approach'
            elif self.control_strategy == 'biarc_smc':
                self.phase = 'biarc_smc'
            elif self.control_strategy == 'normal_ray_smc':
                self.phase = 'ray_turn'
            else:
                self.phase = 'smc_pose'

            self.selection_samples = {}

        self.get_logger().info(
            f'Locked tag {best["tag_id"]}: '
            f'samples={best["count"]}, '
            f'face_angle={math.degrees(best["face_angle"]):.1f} deg, '
            f'distance={best["distance"]:.2f} m, '
            f'margin={best["margin"]:.1f}. '
            f'strategy={self.control_strategy}, '
            f'base-link target={self.target_distance:.2f} m.'
        )

        return True

    # ============================================================
    # Motion control
    # ============================================================

    def get_robot_pose(self):
        try:
            tf_odom_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None

        t = tf_odom_base.transform.translation
        q = tf_odom_base.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return [float(t.x), float(t.y), wrap_angle(yaw)]

    def _smc_command(self, robot, reference):
        x, y, yaw = robot
        gx, gy, gyaw = reference

        dx = gx - x
        dy = gy - y
        rho = math.hypot(dx, dy)
        alpha = (
            angle_difference(math.atan2(dy, dx), yaw)
            if rho > 1e-9
            else 0.0
        )

        e_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        e_theta = angle_difference(gyaw, yaw)
        s = e_theta + self.smc_lambda * e_y
        sat = clamp(s / self.smc_phi, -1.0, 1.0)

        # c = 0: the controlled point is base_link itself.
        angular = (
            self.smc_lambda
            * self.smc_reference_speed
            * math.sin(e_theta)
            + self.smc_ks * s
            + self.smc_eta * sat
        )
        angular = clamp(
            angular,
            -self.max_angular_velocity,
            self.max_angular_velocity,
        )

        linear = self.smc_krho * rho * math.cos(alpha)
        if abs(alpha) >= self.heading_stop:
            linear = 0.0
        linear = clamp(
            linear,
            0.0,
            self.max_linear_velocity,
        )

        return linear, angular, rho, alpha, e_y, e_theta, s

    def _arc_geometry(self, start, start_yaw, end):
        """Return the no-loop circular arc from a start pose to an end point."""
        sx, sy = start
        ex, ey = end
        dx = ex - sx
        dy = ey - sy
        chord = math.hypot(dx, dy)

        if chord < 1e-9:
            return {
                'straight': True,
                'radius': float('inf'),
                'sweep': 0.0,
                'length': 0.0,
                'center': None,
            }

        chord_yaw = math.atan2(dy, dx)
        half_sweep = angle_difference(chord_yaw, start_yaw)

        # Forward-collinear chord -> straight segment. A chord exactly behind
        # the requested tangent has no finite no-loop circular-arc solution.
        if abs(math.sin(half_sweep)) < 1e-8:
            if math.cos(half_sweep) > 0.0:
                return {
                    'straight': True,
                    'radius': float('inf'),
                    'sweep': 0.0,
                    'length': chord,
                    'center': None,
                }
            return None

        # IMPORTANT: do not wrap a forward sweep into [-pi, pi].
        #
        # For a forward-moving circle, sign(sweep) must match sign(radius).
        # Example: a valid +270 deg left-turn arc has the same endpoint
        # position/orientation as a -90 deg geometric arc, but replacing
        # +270 deg by -90 deg reverses the direction of travel relative to the
        # stored tangent. That was the source of the wrong-side biarc bow.
        #
        # Keep the physically consistent signed sweep here. Compactness is
        # handled later by rejecting biarc candidates whose individual arcs
        # exceed biarc_max_arc_sweep.
        sweep = 2.0 * half_sweep

        if abs(sweep) < 1e-8:
            return None

        radius = chord / (2.0 * math.sin(half_sweep))

        nx = -math.sin(start_yaw)
        ny = math.cos(start_yaw)
        center = (
            sx + radius * nx,
            sy + radius * ny,
        )

        return {
            'straight': False,
            'radius': radius,
            'sweep': sweep,
            'length': abs(radius * sweep),
            'center': center,
        }

    def _sample_tangent_arc(self, start, start_yaw, end):
        """Sample a circular arc tangent to start_yaw at the start point."""
        geometry = self._arc_geometry(start, start_yaw, end)
        if geometry is None:
            return None, None

        sx, sy = start
        ex, ey = end

        if geometry['straight']:
            length = math.hypot(ex - sx, ey - sy)
            count = max(
                1,
                int(math.ceil(length / self.biarc_spacing)),
            )
            yaw = (
                math.atan2(ey - sy, ex - sx)
                if length > 1e-9
                else start_yaw
            )
            points = [
                (
                    sx + (ex - sx) * (i / count),
                    sy + (ey - sy) * (i / count),
                    wrap_angle(yaw),
                )
                for i in range(count + 1)
            ]
            points[0] = (sx, sy, wrap_angle(start_yaw))
            points[-1] = (ex, ey, wrap_angle(yaw))
            return points, geometry

        radius = geometry['radius']
        sweep = geometry['sweep']
        cx, cy = geometry['center']
        count = max(
            1,
            int(math.ceil(geometry['length'] / self.biarc_spacing)),
        )

        points = []
        for i in range(count + 1):
            fraction = i / count
            tangent_yaw = start_yaw + fraction * sweep

            # center = p + R * left_normal(tangent), so
            # p = center - R * left_normal(tangent).
            px = cx + radius * math.sin(tangent_yaw)
            py = cy - radius * math.cos(tangent_yaw)
            points.append(
                (
                    float(px),
                    float(py),
                    wrap_angle(tangent_yaw),
                )
            )

        points[0] = (sx, sy, wrap_angle(start_yaw))
        points[-1] = (
            ex,
            ey,
            wrap_angle(start_yaw + sweep),
        )
        return points, geometry

    def _path_length(self, path):
        return sum(
            math.hypot(
                path[index + 1][0] - path[index][0],
                path[index + 1][1] - path[index][1],
            )
            for index in range(len(path) - 1)
        )

    def _max_chord_deviation(self, path, start, end):
        sx, sy = start
        ex, ey = end
        vx = ex - sx
        vy = ey - sy
        vv = vx * vx + vy * vy

        if vv < 1e-12:
            return 0.0

        maximum = 0.0
        for px, py, _ in path:
            wx = px - sx
            wy = py - sy
            u = clamp((wx * vx + wy * vy) / vv, 0.0, 1.0)
            qx = sx + u * vx
            qy = sy + u * vy
            maximum = max(
                maximum,
                math.hypot(px - qx, py - qy),
            )
        return maximum

    def _equal_biarc_distance(self, start_pose, goal_pose):
        """Balanced d1=d2 biarc distance; used as the family search anchor."""
        x0, y0, yaw0 = start_pose
        x1, y1, yaw1 = goal_pose

        t0x = math.cos(yaw0)
        t0y = math.sin(yaw0)
        t1x = math.cos(yaw1)
        t1y = math.sin(yaw1)
        vx = x1 - x0
        vy = y1 - y0
        vv = vx * vx + vy * vy
        dot_t = t0x * t1x + t0y * t1y

        a = 1.0 - dot_t
        b = vx * (t0x + t1x) + vy * (t0y + t1y)
        cc = -0.5 * vv

        roots = []
        if abs(a) < 1e-10:
            if abs(b) < 1e-10:
                return None
            roots.append(-cc / b)
        else:
            discriminant = b * b - 4.0 * a * cc
            if discriminant < 0.0:
                return None
            root = math.sqrt(max(0.0, discriminant))
            roots.extend([
                (-b + root) / (2.0 * a),
                (-b - root) / (2.0 * a),
            ])

        positive = [value for value in roots if value > 1e-6]
        return min(positive) if positive else None

    def _build_biarc_candidate(self, start_pose, goal_pose, d1):
        """Build one exact G1 biarc for a chosen first tangent distance d1."""
        x0, y0, yaw0 = start_pose
        x1, y1, yaw1 = goal_pose

        t0x = math.cos(yaw0)
        t0y = math.sin(yaw0)
        t1x = math.cos(yaw1)
        t1y = math.sin(yaw1)

        vx = x1 - x0
        vy = y1 - y0
        vv = vx * vx + vy * vy
        if vv < 1e-10:
            return {
                'path': [tuple(start_pose), tuple(goal_pose)],
                'd1': 0.0,
                'd2': 0.0,
                'length': 0.0,
                'deviation': 0.0,
                'min_radius': float('inf'),
                'sweep_1': 0.0,
                'sweep_2': 0.0,
            }

        v_dot_t0 = vx * t0x + vy * t0y
        v_dot_t1 = vx * t1x + vy * t1y
        t_dot = t0x * t1x + t0y * t1y

        # Exact biarc relation:
        # d2 = (0.5*v.v - d1*v.t0)
        #      / (v.t1 - d1*(t0.t1 - 1))
        denominator = (
            v_dot_t1
            - d1 * (t_dot - 1.0)
        )
        if abs(denominator) < 1e-9:
            return None

        d2 = (
            0.5 * vv
            - d1 * v_dot_t0
        ) / denominator

        # Positive d1,d2 select the short, non-spiraling biarc family.
        if d1 <= 1e-6 or d2 <= 1e-6:
            return None

        q1x = x0 + d1 * t0x
        q1y = y0 + d1 * t0y
        q2x = x1 - d2 * t1x
        q2y = y1 - d2 * t1y

        total_d = d1 + d2
        join_x = (
            q1x * d2 + q2x * d1
        ) / total_d
        join_y = (
            q1y * d2 + q2y * d1
        ) / total_d

        first, geometry_1 = self._sample_tangent_arc(
            (x0, y0),
            yaw0,
            (join_x, join_y),
        )
        if first is None:
            return None

        # Construct the second arc backwards from the final pose. Reversing it
        # gives forward motion with the requested goal tangent.
        second_reverse, geometry_2_reverse = self._sample_tangent_arc(
            (x1, y1),
            wrap_angle(yaw1 + math.pi),
            (join_x, join_y),
        )
        if second_reverse is None:
            return None

        second = [
            (
                px,
                py,
                wrap_angle(pyaw + math.pi),
            )
            for px, py, pyaw in reversed(second_reverse)
        ]

        forward_sweep_1 = geometry_1['sweep']
        forward_sweep_2 = -geometry_2_reverse['sweep']

        # This experiment is meant to produce a compact tangent path, not a
        # loop around the court. Reject any branch requiring an individual arc
        # close to or beyond 180 deg, then continue searching the biarc family.
        if (
            abs(forward_sweep_1) > self.biarc_max_arc_sweep
            or abs(forward_sweep_2) > self.biarc_max_arc_sweep
        ):
            return None

        join_yaw_error = abs(
            angle_difference(first[-1][2], second[0][2])
        )
        if join_yaw_error > math.radians(0.5):
            return None

        path = first + second[1:]

        # Do not overwrite the endpoint to hide a geometry error. Verify that
        # the sampled biarc actually interpolates the requested endpoint pose.
        start_position_error = math.hypot(
            path[0][0] - x0,
            path[0][1] - y0,
        )
        end_position_error = math.hypot(
            path[-1][0] - x1,
            path[-1][1] - y1,
        )
        start_yaw_error = abs(
            angle_difference(path[0][2], yaw0)
        )
        end_yaw_error = abs(
            angle_difference(path[-1][2], yaw1)
        )

        if (
            start_position_error > 1e-5
            or end_position_error > 1e-5
            or start_yaw_error > math.radians(0.01)
            or end_yaw_error > math.radians(0.01)
        ):
            return None

        # Snap only after the interpolation check has passed, to remove tiny
        # floating-point error while keeping the endpoint invariant explicit.
        path[0] = (x0, y0, wrap_angle(yaw0))
        path[-1] = (x1, y1, wrap_angle(yaw1))

        length = self._path_length(path)
        deviation = self._max_chord_deviation(
            path,
            (x0, y0),
            (x1, y1),
        )

        radii = []
        for geometry in (geometry_1, geometry_2_reverse):
            if not geometry['straight']:
                radii.append(abs(geometry['radius']))
        min_radius = min(radii) if radii else float('inf')

        return {
            'path': path,
            'd1': d1,
            'd2': d2,
            'length': length,
            'deviation': deviation,
            'min_radius': min_radius,
            'sweep_1': forward_sweep_1,
            'sweep_2': forward_sweep_2,
        }

    def _build_biarc_path(self, start_pose, goal_pose):
        """Search the exact G1 biarc family and choose a compact member."""
        equal_d = self._equal_biarc_distance(
            start_pose,
            goal_pose,
        )
        if equal_d is None:
            self.get_logger().warn(
                'Biarc family is degenerate; falling back to direct SMC.'
            )
            return [tuple(start_pose), tuple(goal_pose)]

        log_min = math.log(self.biarc_d1_factor_min)
        log_max = math.log(self.biarc_d1_factor_max)
        factors = [
            math.exp(
                log_min
                + (log_max - log_min)
                * index
                / (self.biarc_d1_factor_samples - 1)
            )
            for index in range(self.biarc_d1_factor_samples)
        ]
        factors.append(1.0)

        candidates = []
        chord = math.hypot(
            goal_pose[0] - start_pose[0],
            goal_pose[1] - start_pose[1],
        )

        for factor in factors:
            candidate = self._build_biarc_candidate(
                start_pose,
                goal_pose,
                equal_d * factor,
            )
            if candidate is None:
                continue

            # Primary objective: short path. Secondary objective: avoid a large
            # outward bow. Very tight circles are also penalized because the
            # SMC must track the reference with finite angular velocity.
            radius_penalty = 0.0
            if (
                math.isfinite(candidate['min_radius'])
                and candidate['min_radius'] < 0.30
            ):
                radius_penalty = 2.0 * (
                    0.30 - candidate['min_radius']
                )

            sweep_penalty = 0.15 * (
                abs(candidate['sweep_1'])
                + abs(candidate['sweep_2'])
            )

            candidate['score'] = (
                candidate['length']
                + 0.75 * candidate['deviation']
                + radius_penalty
                + sweep_penalty
            )

            if chord > 1e-6 and candidate['length'] > 3.0 * chord:
                candidate['score'] += 5.0 * (
                    candidate['length'] - 3.0 * chord
                )

            candidates.append(candidate)

        if not candidates:
            self.get_logger().warn(
                'Biarc search found no compact forward G1 solution within '
                f'{math.degrees(self.biarc_max_arc_sweep):.1f} deg per arc; '
                'falling back to direct SMC.'
            )
            return [tuple(start_pose), tuple(goal_pose)]

        best = min(candidates, key=lambda item: item['score'])

        self.get_logger().info(
            'Biarc selected: '
            f'd1={best["d1"]:.3f} m, d2={best["d2"]:.3f} m, '
            f'length={best["length"]:.3f} m, '
            f'max_deviation={best["deviation"]:.3f} m, '
            f'min_radius={best["min_radius"]:.3f} m, '
            f'forward_sweeps=({math.degrees(best["sweep_1"]):+.1f}, '
            f'{math.degrees(best["sweep_2"]):+.1f}) deg.'
        )

        return best['path']

    def _publish_goal_pose(self, goal_pose):
        if goal_pose is None:
            return

        msg = PoseStamped()
        msg.header.frame_id = self.odom_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(goal_pose[0])
        msg.pose.position.y = float(goal_pose[1])
        msg.pose.position.z = 0.03

        q = quaternion_from_euler(0.0, 0.0, goal_pose[2])
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])

        self.goal_pose_pub.publish(msg)

    def _publish_reference_path(self):
        if not self.reference_path:
            return

        msg = Path()
        msg.header.frame_id = self.odom_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        for x, y, yaw in self.reference_path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.02
            q = quaternion_from_euler(0.0, 0.0, yaw)
            pose.pose.orientation.x = float(q[0])
            pose.pose.orientation.y = float(q[1])
            pose.pose.orientation.z = float(q[2])
            pose.pose.orientation.w = float(q[3])
            msg.poses.append(pose)

        self.reference_path_pub.publish(msg)

    def _biarc_reference(self, robot):
        if not self.reference_path:
            return None

        start = max(0, self.path_progress_index - 3)
        closest_index = start
        closest_distance = float('inf')

        for index in range(start, len(self.reference_path)):
            px, py, _ = self.reference_path[index]
            distance = math.hypot(
                px - robot[0],
                py - robot[1],
            )
            if distance < closest_distance:
                closest_distance = distance
                closest_index = index

        self.path_progress_index = max(
            self.path_progress_index,
            closest_index,
        )

        lookahead_index = self.path_progress_index
        accumulated = 0.0
        while lookahead_index + 1 < len(self.reference_path):
            p0 = self.reference_path[lookahead_index]
            p1 = self.reference_path[lookahead_index + 1]
            accumulated += math.hypot(
                p1[0] - p0[0],
                p1[1] - p0[1],
            )
            lookahead_index += 1
            if accumulated >= self.biarc_lookahead:
                break

        return self.reference_path[lookahead_index]

    def _base_planar_range_to_tag(self, tag_id):
        try:
            tf_base_tag = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.observed_tag_prefix + str(tag_id),
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return float('nan')

        p = tf_base_tag.transform.translation
        return math.hypot(float(p.x), float(p.y))

    def _enter_stable(self, label, robot, goal_pose, tag_id):
        _, _, rho, _, e_y, e_theta, s = self._smc_command(
            robot,
            goal_pose,
        )
        base_range = self._base_planar_range_to_tag(tag_id)
        self.get_logger().info(
            f'{label}: '
            f'rho={rho:.4f} m, e_y={e_y:+.4f} m, '
            f'e_theta={math.degrees(e_theta):+.2f} deg, '
            f's={s:+.4f}, base_range={base_range:.3f} m.'
        )
        with self.lock:
            self.phase = 'stable'
            self.stable_since = None

    def control_loop(self):
        self.process_pending_detection()

        with self.lock:
            if not self.active:
                return

            goal_handle = self.current_goal_handle
            deadline = self.action_deadline
            phase = self.phase
            observe_started = self.observe_started
            last_seen = self.last_tag_seen
            goal_pose = None if self.goal_pose is None else list(self.goal_pose)
            tag_id = self.tracked_tag_id
            distance = self.last_tag_distance
            bearing = self.last_tag_bearing
            face_angle = self.last_face_angle

        if goal_handle is None:
            return

        now = time.monotonic()
        recent = (
            last_seen is not None
            and now - last_seen <= self.tag_lost_timeout
        )

        feedback = ApproachTag.Feedback()
        feedback.tag_id = int(tag_id)
        feedback.distance = float(
            distance if math.isfinite(distance) else -1.0
        )
        feedback.bearing = float(
            bearing if math.isfinite(bearing) else 0.0
        )
        feedback.phase = phase
        goal_handle.publish_feedback(feedback)

        if goal_handle.is_cancel_requested:
            self.finish_approach(
                success=False,
                canceled=True,
                message='Approach canceled.',
            )
            return

        if deadline is not None and now >= deadline:
            self.finish_approach(
                success=False,
                message=f'Approach timed out in phase {phase}.',
            )
            return

        if phase == 'search':
            self.stable_since = None

            robot = self.get_robot_pose()
            if robot is not None:
                yaw = robot[2]
                if self.search_last_yaw is None:
                    self.search_last_yaw = yaw
                else:
                    step = abs(angle_difference(yaw, self.search_last_yaw))
                    if step <= math.pi / 2.0:
                        self.search_accumulated_yaw += step
                    self.search_last_yaw = yaw

                if self.search_accumulated_yaw >= self.search_rotation_rad:
                    self.finish_approach(
                        success=False,
                        message=(
                            'No acceptable AprilTag found after a full '
                            f'{self.search_accumulated_yaw:.3f} rad search rotation.'
                        ),
                    )
                    return

            self.publish_cmd(0.0, self.search_angular_velocity)
            return

        if phase == 'observe':
            self.stable_since = None
            self.publish_cmd(0.0, 0.0)

            if observe_started is None:
                return

            selection_duration = (
                self.selection_settle_time
                + self.selection_window
            )
            if now - observe_started >= selection_duration:
                if not self.select_and_lock_tag():
                    with self.lock:
                        self.phase = 'search'
                        self.observe_started = None
                        self.selection_samples = {}
                    self.get_logger().warn(
                        'Stationary selection did not collect enough usable '
                        'samples; returning to search.'
                    )
            return

        if goal_pose is None or tag_id < 0:
            with self.lock:
                self.phase = 'search'
                self.observe_started = None
                self.selection_samples = {}
            self.publish_cmd(0.0, self.search_angular_velocity)
            return

        robot = self.get_robot_pose()
        if robot is None:
            self.publish_cmd(0.0, 0.0)
            return

        # RViz must show the exact goal used by this controller, in the same
        # odom frame as the generated reference path.
        self._publish_goal_pose(goal_pose)

        gx, gy, gyaw = goal_pose

        # --------------------------------------------------------
        # Strategy 2: original main-branch direct-goal controller.
        # --------------------------------------------------------
        if phase == 'main_approach':
            dx = gx - robot[0]
            dy = gy - robot[1]
            rho = math.hypot(dx, dy)
            heading_error = angle_difference(
                math.atan2(dy, dx),
                robot[2],
            )

            if rho > self.position_tolerance:
                self.stable_since = None
                angular = clamp(
                    self.main_k_heading * heading_error,
                    -self.max_angular_velocity,
                    self.max_angular_velocity,
                )
                if abs(heading_error) > self.main_drive_heading_limit:
                    linear = 0.0
                else:
                    linear = clamp(
                        self.main_k_position * rho,
                        0.0,
                        self.max_linear_velocity,
                    )
                    linear *= max(0.20, math.cos(heading_error))
                self.publish_cmd(linear, angular)
                return

            with self.lock:
                self.phase = 'main_final_align'
            phase = 'main_final_align'

        if phase == 'main_final_align':
            yaw_error = angle_difference(gyaw, robot[2])
            if abs(yaw_error) > self.yaw_tolerance:
                self.stable_since = None
                angular = clamp(
                    self.main_k_final_yaw * yaw_error,
                    -self.max_angular_velocity,
                    self.max_angular_velocity,
                )
                self.publish_cmd(0.0, angular)
                return

            self._enter_stable(
                'Main-branch pose reached',
                robot,
                goal_pose,
                tag_id,
            )
            phase = 'stable'

        # --------------------------------------------------------
        # Strategy 3: biarc tangent path -> moving SMC reference.
        # --------------------------------------------------------
        if phase == 'biarc_smc':
            if not self.reference_path:
                self.reference_path = self._build_biarc_path(
                    robot,
                    goal_pose,
                )
                self.path_progress_index = 0
                self._publish_reference_path()

                endpoint = self.reference_path[-1]
                endpoint_position_error = math.hypot(
                    endpoint[0] - goal_pose[0],
                    endpoint[1] - goal_pose[1],
                )
                endpoint_yaw_error = abs(
                    angle_difference(endpoint[2], goal_pose[2])
                )

                self.get_logger().info(
                    f'Biarc path generated: {len(self.reference_path)} poses; '
                    f'endpoint_error={endpoint_position_error:.6f} m, '
                    f'endpoint_yaw_error='
                    f'{math.degrees(endpoint_yaw_error):.6f} deg.'
                )

            final_goal = self.reference_path[-1]
            goal_rho = math.hypot(
                final_goal[0] - robot[0],
                final_goal[1] - robot[1],
            )
            goal_yaw_error = angle_difference(
                final_goal[2],
                robot[2],
            )

            if (
                goal_rho <= self.position_tolerance
                and abs(goal_yaw_error) <= self.yaw_tolerance
            ):
                self._enter_stable(
                    'Biarc-SMC pose reached',
                    robot,
                    final_goal,
                    tag_id,
                )
                phase = 'stable'
            else:
                reference = self._biarc_reference(robot)
                if reference is None:
                    self.publish_cmd(0.0, 0.0)
                    return

                linear, angular, _, _, _, _, _ = self._smc_command(
                    robot,
                    reference,
                )
                self.publish_cmd(linear, angular)
                return

        # --------------------------------------------------------
        # Strategy 4: perpendicular normal-ray capture -> SMC.
        # --------------------------------------------------------
        if phase in ('ray_turn', 'ray_cross', 'ray_face'):
            dx = robot[0] - gx
            dy = robot[1] - gy
            lateral = (
                -math.sin(gyaw) * dx
                + math.cos(gyaw) * dy
            )

            if phase == 'ray_turn':
                if abs(lateral) <= self.ray_position_tolerance:
                    with self.lock:
                        self.phase = 'ray_face'
                    phase = 'ray_face'
                else:
                    if self.ray_heading is None:
                        self.ray_heading = wrap_angle(
                            gyaw
                            - math.copysign(math.pi / 2.0, lateral)
                        )

                    heading_error = angle_difference(
                        self.ray_heading,
                        robot[2],
                    )
                    if abs(heading_error) > self.ray_heading_tolerance:
                        self.publish_cmd(
                            0.0,
                            clamp(
                                self.ray_heading_gain * heading_error,
                                -self.max_angular_velocity,
                                self.max_angular_velocity,
                            ),
                        )
                        return

                    with self.lock:
                        self.phase = 'ray_cross'
                    phase = 'ray_cross'

            if phase == 'ray_cross':
                if abs(lateral) <= self.ray_position_tolerance:
                    self.publish_cmd(0.0, 0.0)
                    with self.lock:
                        self.phase = 'ray_face'
                    phase = 'ray_face'
                else:
                    heading_error = angle_difference(
                        self.ray_heading,
                        robot[2],
                    )
                    linear = min(
                        self.ray_cross_speed,
                        max(0.08, 0.8 * abs(lateral)),
                    )
                    angular = clamp(
                        self.ray_heading_gain * heading_error,
                        -self.max_angular_velocity,
                        self.max_angular_velocity,
                    )
                    self.publish_cmd(linear, angular)
                    return

            if phase == 'ray_face':
                heading_error = angle_difference(
                    gyaw,
                    robot[2],
                )
                if abs(heading_error) > self.ray_heading_tolerance:
                    self.publish_cmd(
                        0.0,
                        clamp(
                            self.ray_heading_gain * heading_error,
                            -self.max_angular_velocity,
                            self.max_angular_velocity,
                        ),
                    )
                    return

                self.ray_heading = None
                with self.lock:
                    self.phase = 'smc_pose'
                phase = 'smc_pose'

        # --------------------------------------------------------
        # Strategy 1 and final stage of strategy 4: direct pose SMC.
        # --------------------------------------------------------
        if phase == 'smc_pose':
            linear, angular, rho, _, e_y, e_theta, s = self._smc_command(
                robot,
                goal_pose,
            )

            if (
                rho > self.position_tolerance
                or abs(e_theta) > self.yaw_tolerance
            ):
                self.stable_since = None
                self.publish_cmd(linear, angular)
                return

            base_range = self._base_planar_range_to_tag(tag_id)
            self.get_logger().info(
                'Tag SMC pose reached: '
                f'rho={rho:.4f} m, e_y={e_y:+.4f} m, '
                f'e_theta={math.degrees(e_theta):+.2f} deg, '
                f's={s:+.4f}, base_range={base_range:.3f} m.'
            )
            with self.lock:
                self.phase = 'stable'
                self.stable_since = None
            phase = 'stable'

        if phase == 'stable':
            self.publish_cmd(0.0, 0.0)

            # /relocalize will immediately need this same locked tag.
            if not recent:
                self.stable_since = None
                return

            if self.stable_since is None:
                self.stable_since = now
                return

            if now - self.stable_since >= self.stable_time:
                face_text = (
                    f'{math.degrees(face_angle):.1f} deg'
                    if math.isfinite(face_angle)
                    else 'unknown'
                )
                self.finish_approach(
                    success=True,
                    message=(
                        'Reached requested tag observation pose with locked '
                        f'tag {tag_id}; strategy={self.control_strategy}; '
                        f'final face angle={face_text}.'
                    ),
                )

    # ============================================================
    # Action completion / command output
    # ============================================================

    def finish_approach(self, success, message, canceled=False):
        self.stop_robot()

        with self.lock:
            if not self.active:
                return

            goal_handle = self.current_goal_handle
            future = self.action_future
            tag_id = self.tracked_tag_id
            distance = self.last_tag_distance
            bearing = self.last_tag_bearing

            self.active = False
            self.phase = 'idle'
            self.preferred_tag_id = -1
            self.goal_pose = None
            self.tracked_tag_id = -1
            self.last_tag_seen = None
            self.last_tag_distance = float('nan')
            self.last_tag_bearing = float('nan')
            self.last_face_angle = float('nan')
            self.stable_since = None
            self.reference_path = []
            self.path_progress_index = 0
            self.ray_heading = None
            self.search_last_yaw = None
            self.search_accumulated_yaw = 0.0

            self.observe_started = None
            self.selection_samples = {}
            self.pending_detection = None
            self.pending_detection_received = None

            self.current_goal_handle = None
            self.action_future = None
            self.action_deadline = None

        self.control_timer.cancel()

        result = ApproachTag.Result()
        result.success = bool(success)
        result.tag_id = int(tag_id)
        result.final_distance = float(
            distance if math.isfinite(distance) else -1.0
        )
        result.final_bearing = float(
            bearing if math.isfinite(bearing) else 0.0
        )
        result.message = message

        if goal_handle is not None:
            if canceled:
                goal_handle.canceled()
            elif success:
                goal_handle.succeed()
            else:
                goal_handle.abort()

        if future is not None and not future.done():
            future.set_result(result)

    def publish_cmd(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = TagApproachController()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
