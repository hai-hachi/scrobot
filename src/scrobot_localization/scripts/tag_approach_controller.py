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
    def __init__(self):
        super().__init__('tag_approach_controller')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('detections_topic', '/apriltag/detections')
        self.declare_parameter('observed_tag_prefix', 'observed_tag_')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_relocalization')
        self.declare_parameter('min_decision_margin', 10.0)
        self.declare_parameter('max_detection_distance', 6.0)
        self.declare_parameter('default_target_distance', 1.70)
        self.declare_parameter('default_timeout', 20.0)
        self.declare_parameter('tag_lost_timeout', 0.50)
        self.declare_parameter('search_angular_velocity', 0.25)
        self.declare_parameter('max_linear_velocity', 0.25)
        self.declare_parameter('max_angular_velocity', 0.60)
        self.declare_parameter('k_position', 0.80)
        self.declare_parameter('k_heading', 1.80)
        self.declare_parameter('k_final_yaw', 1.80)
        self.declare_parameter('drive_heading_limit_deg', 35.0)
        self.declare_parameter('position_tolerance', 0.08)
        self.declare_parameter('yaw_tolerance_deg', 5.0)
        self.declare_parameter('stable_time', 0.40)
        self.declare_parameter('goal_filter_alpha', 0.30)
        self.declare_parameter('control_rate', 20.0)

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
        self.search_angular_velocity = float(
            self.get_parameter('search_angular_velocity').value
        )
        self.max_linear_velocity = float(
            self.get_parameter('max_linear_velocity').value
        )
        self.max_angular_velocity = float(
            self.get_parameter('max_angular_velocity').value
        )
        self.k_position = float(self.get_parameter('k_position').value)
        self.k_heading = float(self.get_parameter('k_heading').value)
        self.k_final_yaw = float(self.get_parameter('k_final_yaw').value)
        self.drive_heading_limit = math.radians(
            float(self.get_parameter('drive_heading_limit_deg').value)
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

        # Same user-validated mount -> apriltag_ros PnP transform as the global localizer.
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
        self.preferred_tag_id = -1
        self.target_distance = self.default_target_distance
        self.goal_pose = None
        self.tracked_tag_id = -1
        self.last_tag_seen = None
        self.last_tag_distance = float('nan')
        self.last_tag_bearing = float('nan')
        self.stable_since = None

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
        # No reason to wake at 20 Hz while no approach action is active.
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
            'Tag approach controller V2 started: non-blocking action, '
            'single-threaded executor, active-only control timer.'
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
            self.preferred_tag_id = int(goal_handle.request.preferred_tag_id)
            self.target_distance = target_distance
            self.goal_pose = None
            self.tracked_tag_id = -1
            self.last_tag_seen = None
            self.last_tag_distance = float('nan')
            self.last_tag_bearing = float('nan')
            self.stable_since = None

            self.current_goal_handle = goal_handle
            self.action_future = future
            self.action_deadline = time.monotonic() + timeout

        self.control_timer.reset()

        self.get_logger().info(
            f'ApproachTag started: tag={self.preferred_tag_id}, '
            f'target_distance={target_distance:.2f} m'
        )

        result = await future
        return result

    def detection_callback(self, msg):
        with self.lock:
            if not self.active:
                return
            preferred = self.preferred_tag_id
            target_distance = self.target_distance

        stamp = Time.from_msg(msg.header.stamp)
        candidates = []

        for detection in msg.detections:
            tag_id = int(detection.id)
            if tag_id not in [0, 1, 2, 3] or detection.hamming != 0:
                continue
            if preferred >= 0 and tag_id != preferred:
                continue

            margin = float(detection.decision_margin)
            if margin < self.min_decision_margin:
                continue

            observed_tag_frame = self.observed_tag_prefix + str(tag_id)

            if not self.tf_buffer.can_transform(
                self.odom_frame,
                observed_tag_frame,
                stamp,
                timeout=Duration(seconds=0.0),
            ):
                continue
            if not self.tf_buffer.can_transform(
                self.base_frame,
                observed_tag_frame,
                stamp,
                timeout=Duration(seconds=0.0),
            ):
                continue

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
                continue

            p = tf_base_tag.transform.translation
            distance = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
            bearing = math.atan2(p.y, p.x)
            if distance > self.max_detection_distance:
                continue

            score = margin / max(distance * distance, 0.25)
            candidates.append(
                (
                    score,
                    tag_id,
                    distance,
                    bearing,
                    transform_to_matrix(tf_odom_tag.transform),
                )
            )

        if not candidates:
            return

        _, tag_id, distance, bearing, T_odom_april = max(
            candidates,
            key=lambda item: item[0],
        )

        # Reconstruct the intuitive physical tag mount frame in odom.
        T_odom_mount = T_odom_april @ inverse_matrix(self.T_mount_apriltag)

        # Desired base pose: target_distance in front of the visible tag face,
        # with base +X facing back toward the tag.
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

        with self.lock:
            if self.goal_pose is None:
                self.goal_pose = [gx, gy, gyaw]
            else:
                a = self.goal_filter_alpha
                old_x, old_y, old_yaw = self.goal_pose
                self.goal_pose = [
                    old_x + a * (gx - old_x),
                    old_y + a * (gy - old_y),
                    wrap_angle(
                        old_yaw + a * angle_difference(gyaw, old_yaw)
                    ),
                ]

            self.tracked_tag_id = tag_id
            self.last_tag_seen = time.monotonic()
            self.last_tag_distance = distance
            self.last_tag_bearing = bearing

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
        with self.lock:
            if not self.active:
                return

            goal_handle = self.current_goal_handle
            deadline = self.action_deadline
            last_seen = self.last_tag_seen
            goal_pose = None if self.goal_pose is None else list(self.goal_pose)
            tag_id = self.tracked_tag_id
            distance = self.last_tag_distance
            bearing = self.last_tag_bearing

        if goal_handle is None:
            return

        recent = (
            last_seen is not None
            and time.monotonic() - last_seen <= self.tag_lost_timeout
        )

        feedback = ApproachTag.Feedback()
        feedback.tag_id = int(tag_id)
        feedback.distance = float(
            distance if math.isfinite(distance) else -1.0
        )
        feedback.bearing = float(
            bearing if math.isfinite(bearing) else 0.0
        )
        feedback.phase = (
            'approach' if recent and goal_pose is not None else 'search'
        )
        goal_handle.publish_feedback(feedback)

        if goal_handle.is_cancel_requested:
            self.finish_approach(
                success=False,
                canceled=True,
                message='Approach canceled.',
            )
            return

        if deadline is not None and time.monotonic() >= deadline:
            self.finish_approach(
                success=False,
                message='Approach timed out.',
            )
            return

        if not recent or goal_pose is None:
            self.stable_since = None
            self.publish_cmd(0.0, self.search_angular_velocity)
            return

        robot = self.get_robot_pose()
        if robot is None:
            self.publish_cmd(0.0, 0.0)
            return

        x, y, yaw = robot
        gx, gy, gyaw = goal_pose
        dx = gx - x
        dy = gy - y
        rho = math.hypot(dx, dy)
        heading_error = angle_difference(math.atan2(dy, dx), yaw)
        final_yaw_error = angle_difference(gyaw, yaw)

        if rho > self.position_tolerance:
            self.stable_since = None
            angular = clamp(
                self.k_heading * heading_error,
                -self.max_angular_velocity,
                self.max_angular_velocity,
            )
            if abs(heading_error) > self.drive_heading_limit:
                linear = 0.0
            else:
                linear = clamp(
                    self.k_position * rho,
                    0.0,
                    self.max_linear_velocity,
                )
                linear *= max(0.20, math.cos(heading_error))
            self.publish_cmd(linear, angular)
            return

        if abs(final_yaw_error) > self.yaw_tolerance:
            self.stable_since = None
            angular = clamp(
                self.k_final_yaw * final_yaw_error,
                -self.max_angular_velocity,
                self.max_angular_velocity,
            )
            self.publish_cmd(0.0, angular)
            return

        self.publish_cmd(0.0, 0.0)
        if self.stable_since is None:
            self.stable_since = time.monotonic()
        elif time.monotonic() - self.stable_since >= self.stable_time:
            self.finish_approach(
                success=True,
                message='Reached requested tag observation pose.',
            )

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
            self.preferred_tag_id = -1
            self.goal_pose = None
            self.current_goal_handle = None
            self.action_future = None
            self.action_deadline = None
            self.stable_since = None

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
