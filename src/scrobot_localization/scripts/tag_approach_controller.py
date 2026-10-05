#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TwistStamped
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
    Stationary multi-frame tag selection + camera-relative SMC approach.

    Flow:
      search -> stop/observe -> lock best-facing tag
      -> SMC to camera-relative observation pose -> stable

    Once a tag is locked, the controller keeps driving toward the odom-frame
    goal even if the camera briefly loses the tag. The locked tag may refine
    that goal when new observations arrive, but other tags cannot steal it.
    """

    def __init__(self):
        super().__init__('tag_approach_controller')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('camera_frame', 'camera_color_frame')
        self.declare_parameter('detections_topic', '/apriltag/detections')
        self.declare_parameter('observed_tag_prefix', 'observed_tag_')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_relocalization')

        self.declare_parameter('min_decision_margin', 10.0)
        self.declare_parameter('max_detection_distance', 10.0)
        self.declare_parameter('default_target_distance', 0.80)
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

        # Shared nonlinear SMC pose law. The controlled point is a virtual
        # point on the robot centerline at the camera's forward X offset.
        # The camera's small lateral Y offset is compensated in the goal.
        self.declare_parameter('smc_reference_speed', 0.50)
        self.declare_parameter('smc_lambda', 2.00)
        self.declare_parameter('smc_ks', 1.60)
        self.declare_parameter('smc_eta', 0.50)
        self.declare_parameter('smc_phi', 0.08)
        self.declare_parameter('smc_krho', 0.80)
        self.declare_parameter('heading_stop_deg', 70.0)

        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance_deg', 5.0)
        self.declare_parameter('stable_time', 0.25)
        self.declare_parameter('goal_filter_alpha', 0.30)
        self.declare_parameter('control_rate', 25.0)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
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
            'selection, best-facing tag lock, camera-relative SMC pose control.'
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
            f'ApproachTag SMC started: preferred_tag={self.preferred_tag_id}, '
            f'target_distance={target_distance:.2f} m'
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

        # Desired observation pose is defined at the COLOR CAMERA, not at
        # base_footprint. target_distance is therefore camera-relative.
        T_mount_camera_goal = xyz_rpy_to_matrix(
            [target_distance, 0.0, 0.0],
            [0.0, 0.0, math.pi],
        )
        T_odom_camera_goal = T_odom_mount @ T_mount_camera_goal

        camera_goal_x = float(T_odom_camera_goal[0, 3])
        camera_goal_y = float(T_odom_camera_goal[1, 3])
        q_goal = quaternion_from_matrix(T_odom_camera_goal)
        _, _, gyaw = euler_from_quaternion(q_goal)
        gyaw = wrap_angle(gyaw)

        try:
            tf_base_camera = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None, 'tf'

        camera_dx = float(tf_base_camera.transform.translation.x)
        camera_dy = float(tf_base_camera.transform.translation.y)

        # SMC controls a virtual centerline point c metres ahead of the base.
        # Compensate the real camera's lateral offset so convergence of the
        # virtual point places the actual camera at the requested pose.
        control_offset_c = camera_dx
        gx = camera_goal_x + camera_dy * math.sin(gyaw)
        gy = camera_goal_y - camera_dy * math.cos(gyaw)

        return {
            'tag_id': tag_id,
            'margin': float(detection.decision_margin),
            'distance': distance,
            'bearing': bearing,
            'face_angle': face_angle,
            'goal_x': gx,
            'goal_y': gy,
            'goal_yaw': gyaw,
            'control_offset_c': control_offset_c,
            'camera_goal_x': camera_goal_x,
            'camera_goal_y': camera_goal_y,
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
            if self.goal_pose is None:
                self.goal_pose = [
                    candidate['goal_x'],
                    candidate['goal_y'],
                    candidate['goal_yaw'],
                    candidate['control_offset_c'],
                ]
            else:
                a = self.goal_filter_alpha
                old_x, old_y, old_yaw, old_c = self.goal_pose
                self.goal_pose = [
                    old_x + a * (candidate['goal_x'] - old_x),
                    old_y + a * (candidate['goal_y'] - old_y),
                    wrap_angle(
                        old_yaw
                        + a * angle_difference(candidate['goal_yaw'], old_yaw)
                    ),
                    old_c + a * (candidate['control_offset_c'] - old_c),
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
                'control_offset_c': median(
                    [sample['control_offset_c'] for sample in samples]
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
                best['control_offset_c'],
            ]
            self.last_tag_seen = time.monotonic()
            self.last_tag_distance = best['distance']
            self.last_tag_bearing = best['bearing']
            self.last_face_angle = best['face_angle']
            self.phase = 'smc_pose'
            self.selection_samples = {}

        self.get_logger().info(
            f'Locked tag {best["tag_id"]}: '
            f'samples={best["count"]}, '
            f'face_angle={math.degrees(best["face_angle"]):.1f} deg, '
            f'distance={best["distance"]:.2f} m, '
            f'margin={best["margin"]:.1f}. '
            f'SMC to {self.target_distance:.2f} m camera-relative observation pose.'
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
                    # Ignore impossible discontinuities rather than counting a TF reset
                    # as physical rotation.
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

            selection_duration = self.selection_settle_time + self.selection_window
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

        x, y, yaw = robot
        gx, gy, gyaw, control_offset_c = goal_pose

        # Pose of the virtual SMC control point on the robot centerline.
        control_x = x + control_offset_c * math.cos(yaw)
        control_y = y + control_offset_c * math.sin(yaw)

        dx = gx - control_x
        dy = gy - control_y
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

        denominator = 1.0 + self.smc_lambda * control_offset_c
        angular = (
            self.smc_lambda
            * self.smc_reference_speed
            * math.sin(e_theta)
            + self.smc_ks * s
            + self.smc_eta * sat
        ) / denominator
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

        if phase == 'smc_pose':
            if (
                rho > self.position_tolerance
                or abs(e_theta) > self.yaw_tolerance
            ):
                self.stable_since = None
                self.publish_cmd(linear, angular)
                return

            camera_range = float('nan')
            try:
                tf_camera_tag = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    self.observed_tag_prefix + str(tag_id),
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
                p = tf_camera_tag.transform.translation
                camera_range = math.hypot(float(p.x), float(p.y))
            except TransformException:
                pass

            self.get_logger().info(
                'Tag SMC pose reached: '
                f'rho={rho:.4f} m, e_y={e_y:+.4f} m, '
                f'e_theta={math.degrees(e_theta):+.2f} deg, '
                f's={s:+.4f}, '
                f'camera_range={camera_range:.3f} m.'
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
                        f'tag {tag_id}; final face angle={face_text}.'
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
