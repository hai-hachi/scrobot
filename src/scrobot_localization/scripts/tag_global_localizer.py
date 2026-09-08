#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from scrobot_interfaces.action import Relocalize
from std_msgs.msg import Int32MultiArray
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformException, TransformListener
from tf_transformations import concatenate_matrices, euler_from_quaternion, inverse_matrix, quaternion_from_euler, quaternion_from_matrix, quaternion_matrix, translation_from_matrix, translation_matrix


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_difference(target, source):
    return wrap_angle(target - source)


def clamp(value, low, high):
    return max(low, min(high, value))


def transform_to_matrix(transform):
    translation = [transform.translation.x, transform.translation.y, transform.translation.z]
    quaternion = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
    return concatenate_matrices(translation_matrix(translation), quaternion_matrix(quaternion))


def xyz_rpy_to_matrix(xyz, rpy):
    quaternion = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    return concatenate_matrices(translation_matrix(xyz), quaternion_matrix(quaternion))


def matrix_to_transform(matrix):
    return translation_from_matrix(matrix), quaternion_from_matrix(matrix)


def make_transform(parent, child, matrix, stamp):
    translation, quaternion = matrix_to_transform(matrix)
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = float(translation[0])
    transform.transform.translation.y = float(translation[1])
    transform.transform.translation.z = float(translation[2])
    transform.transform.rotation.x = float(quaternion[0])
    transform.transform.rotation.y = float(quaternion[1])
    transform.transform.rotation.z = float(quaternion[2])
    transform.transform.rotation.w = float(quaternion[3])
    return transform


class TagGlobalLocalizer(Node):
    def __init__(self):
        super().__init__('tag_global_localizer')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('detections_topic', '/apriltag/detections')
        self.declare_parameter('observed_tag_prefix', 'observed_tag_')
        self.declare_parameter('mount_frame_prefix', 'tag_mount_')
        self.declare_parameter('known_tag_prefix', 'court_tag_')

        self.declare_parameter('pole_x', 0.0)
        self.declare_parameter('left_pole_y', 3.05)
        self.declare_parameter('right_pole_y', -3.05)
        self.declare_parameter('tag_height', 0.120)
        self.declare_parameter('tag_mount_radius', 0.030)
        self.declare_parameter('inward_angle_deg', 45.0)
        self.declare_parameter('tag_edge_size', 0.100)
        self.declare_parameter('active_grid_cells', 6)
        self.declare_parameter('quiet_border_cells', 1)
        self.declare_parameter('texture_pixels', 1024)

        self.declare_parameter('min_decision_margin', 20.0)
        self.declare_parameter('max_tag_distance', 6.0)
        self.declare_parameter('relocalization_max_distance', 2.0)
        self.declare_parameter('max_view_angle_deg', 35.0)
        self.declare_parameter('max_position_disagreement', 0.15)
        self.declare_parameter('max_yaw_disagreement_deg', 6.0)

        self.declare_parameter('default_sample_count', 15)
        self.declare_parameter('default_timeout', 5.0)
        self.declare_parameter('batch_outlier_position', 0.10)
        self.declare_parameter('batch_outlier_yaw_deg', 4.0)
        self.declare_parameter('max_batch_position_std', 0.04)
        self.declare_parameter('max_batch_yaw_std_deg', 1.5)
        self.declare_parameter('minimum_batch_keep_ratio', 0.70)

        self.declare_parameter('stationary_linear_threshold', 0.02)
        self.declare_parameter('stationary_angular_threshold', 0.03)
        self.declare_parameter('stationary_settle_time', 0.40)
        self.declare_parameter('processing_rate', 100.0)
        self.declare_parameter('publish_rate', 30.0)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.detections_topic = str(self.get_parameter('detections_topic').value)
        self.observed_tag_prefix = str(self.get_parameter('observed_tag_prefix').value)
        self.mount_frame_prefix = str(self.get_parameter('mount_frame_prefix').value)
        self.known_tag_prefix = str(self.get_parameter('known_tag_prefix').value)

        self.pole_x = float(self.get_parameter('pole_x').value)
        self.left_pole_y = float(self.get_parameter('left_pole_y').value)
        self.right_pole_y = float(self.get_parameter('right_pole_y').value)
        self.tag_height = float(self.get_parameter('tag_height').value)
        self.tag_mount_radius = float(self.get_parameter('tag_mount_radius').value)
        self.inward_angle_deg = float(self.get_parameter('inward_angle_deg').value)

        self.min_decision_margin = float(self.get_parameter('min_decision_margin').value)
        self.max_tag_distance = float(self.get_parameter('max_tag_distance').value)
        self.relocalization_max_distance = float(self.get_parameter('relocalization_max_distance').value)
        self.max_view_angle = math.radians(float(self.get_parameter('max_view_angle_deg').value))
        self.max_position_disagreement = float(self.get_parameter('max_position_disagreement').value)
        self.max_yaw_disagreement = math.radians(float(self.get_parameter('max_yaw_disagreement_deg').value))

        self.default_sample_count = int(self.get_parameter('default_sample_count').value)
        self.default_timeout = float(self.get_parameter('default_timeout').value)
        self.batch_outlier_position = float(self.get_parameter('batch_outlier_position').value)
        self.batch_outlier_yaw = math.radians(float(self.get_parameter('batch_outlier_yaw_deg').value))
        self.max_batch_position_std = float(self.get_parameter('max_batch_position_std').value)
        self.max_batch_yaw_std = math.radians(float(self.get_parameter('max_batch_yaw_std_deg').value))
        self.minimum_batch_keep_ratio = float(self.get_parameter('minimum_batch_keep_ratio').value)

        self.stationary_linear_threshold = float(self.get_parameter('stationary_linear_threshold').value)
        self.stationary_angular_threshold = float(self.get_parameter('stationary_angular_threshold').value)
        self.stationary_settle_time = float(self.get_parameter('stationary_settle_time').value)
        processing_rate = float(self.get_parameter('processing_rate').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.tag_ids = [0, 1, 2, 3]
        self.mount_map = {}
        self.tag_map = {}

        # This is the transform you experimentally validated in your TF setup.
        self.T_mount_apriltag = xyz_rpy_to_matrix([0.0, 0.0, 0.0], [-math.pi / 2.0, 0.0, -math.pi / 2.0])

        headings = self.compute_tag_headings(self.inward_angle_deg)
        for tag_id in self.tag_ids:
            heading = headings[tag_id]
            pole_y = self.left_pole_y if tag_id in (0, 2) else self.right_pole_y
            x = self.pole_x + self.tag_mount_radius * math.cos(heading)
            y = pole_y + self.tag_mount_radius * math.sin(heading)
            z = self.tag_height
            T_map_mount = xyz_rpy_to_matrix([x, y, z], [0.0, 0.0, heading])
            self.mount_map[tag_id] = T_map_mount
            self.tag_map[tag_id] = T_map_mount @ self.T_mount_apriltag

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.pending_detection = None
        self.map_to_odom_state = None
        self.stationary_since = None

        self.lock = threading.Lock()
        self.acquisition_active = False
        self.acquisition_preferred_tag = -1
        self.acquisition_samples = []
        self.last_visible_tag_ids = []
        self.last_best_distance = float('nan')
        self.last_best_margin = float('nan')

        self.raw_pose_pub = self.create_publisher(PoseStamped, '/global_localization/tag_pose_raw', 10)
        self.committed_pose_pub = self.create_publisher(PoseStamped, '/global_localization/tag_pose_committed', 10)
        self.active_tags_pub = self.create_publisher(Int32MultiArray, '/global_localization/active_tags', 10)

        self.detection_sub = self.create_subscription(AprilTagDetectionArray, self.detections_topic, self.detection_callback, 10, callback_group=self.cb_group)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20, callback_group=self.cb_group)
        self.processing_timer = self.create_timer(1.0 / processing_rate, self.process_pending_detection, callback_group=self.cb_group)
        self.broadcast_timer = self.create_timer(1.0 / publish_rate, self.broadcast_map_to_odom, callback_group=self.cb_group)

        self.action_server = ActionServer(self, Relocalize, '/relocalize', execute_callback=self.execute_relocalize, goal_callback=self.goal_callback, cancel_callback=self.cancel_callback, callback_group=self.cb_group)

        self.publish_known_tags()
        self.get_logger().info('Tag global localizer V4 started: map->odom changes only after a successful /relocalize action.')

    @staticmethod
    def compute_tag_headings(inward_angle_deg):
        a = math.radians(inward_angle_deg)
        return {0: -a, 1: +a, 2: -math.pi + a, 3: +math.pi - a}

    def publish_known_tags(self):
        transforms = []
        now = self.get_clock().now().to_msg()
        for tag_id in self.tag_ids:
            mount_frame = f'{self.mount_frame_prefix}{tag_id}'
            known_tag_frame = f'{self.known_tag_prefix}{tag_id}'
            transforms.append(make_transform(self.map_frame, mount_frame, self.mount_map[tag_id], now))
            transforms.append(make_transform(mount_frame, known_tag_frame, self.T_mount_apriltag, now))
        self.static_tf_broadcaster.sendTransform(transforms)

    def odom_callback(self, msg):
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        wz = float(msg.twist.twist.angular.z)
        stationary = math.hypot(vx, vy) <= self.stationary_linear_threshold and abs(wz) <= self.stationary_angular_threshold
        if stationary:
            if self.stationary_since is None:
                self.stationary_since = time.monotonic()
        else:
            self.stationary_since = None

    def is_stationary(self):
        return self.stationary_since is not None and time.monotonic() - self.stationary_since >= self.stationary_settle_time

    def detection_callback(self, msg):
        if msg.detections:
            self.pending_detection = msg

    def compute_view_angle(self, T_camera_tag):
        p = T_camera_tag[:3, 3]
        norm = math.sqrt(float(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]))
        if norm < 1e-6:
            return math.pi
        z_axis = T_camera_tag[:3, 2]
        to_camera = [-float(p[0]) / norm, -float(p[1]) / norm, -float(p[2]) / norm]
        dot = clamp(float(z_axis[0]) * to_camera[0] + float(z_axis[1]) * to_camera[1] + float(z_axis[2]) * to_camera[2], -1.0, 1.0)
        return math.acos(dot)

    def process_pending_detection(self):
        if self.pending_detection is None:
            return

        msg = self.pending_detection
        stamp = Time.from_msg(msg.header.stamp)
        camera_frame = msg.header.frame_id

        if not self.tf_buffer.can_transform(camera_frame, self.base_frame, stamp, timeout=Duration(seconds=0.0)):
            return
        if not self.tf_buffer.can_transform(self.odom_frame, self.base_frame, stamp, timeout=Duration(seconds=0.0)):
            return

        try:
            tf_camera_base = self.tf_buffer.lookup_transform(camera_frame, self.base_frame, stamp, timeout=Duration(seconds=0.0))
            tf_odom_base = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, stamp, timeout=Duration(seconds=0.0))
        except TransformException:
            return

        T_camera_base = transform_to_matrix(tf_camera_base.transform)
        T_odom_base = transform_to_matrix(tf_odom_base.transform)

        with self.lock:
            acquisition_active = self.acquisition_active
            preferred_tag = self.acquisition_preferred_tag

        candidates = []
        visible_ids = []

        for detection in msg.detections:
            tag_id = int(detection.id)
            if tag_id not in self.tag_ids or detection.hamming != 0:
                continue

            margin = float(detection.decision_margin)
            if margin < self.min_decision_margin:
                continue
            if acquisition_active and preferred_tag >= 0 and tag_id != preferred_tag:
                continue

            observed_tag_frame = self.observed_tag_prefix + str(tag_id)
            if not self.tf_buffer.can_transform(camera_frame, observed_tag_frame, stamp, timeout=Duration(seconds=0.0)):
                continue

            try:
                tf_camera_tag = self.tf_buffer.lookup_transform(camera_frame, observed_tag_frame, stamp, timeout=Duration(seconds=0.0))
            except TransformException:
                continue

            t = tf_camera_tag.transform.translation
            distance = math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z)
            if distance > self.max_tag_distance:
                continue

            T_camera_tag = transform_to_matrix(tf_camera_tag.transform)
            view_angle = self.compute_view_angle(T_camera_tag)
            visible_ids.append(tag_id)

            if acquisition_active:
                if distance > self.relocalization_max_distance or view_angle > self.max_view_angle:
                    continue

            T_map_camera = self.tag_map[tag_id] @ inverse_matrix(T_camera_tag)
            T_map_base = T_map_camera @ T_camera_base
            T_map_odom = T_map_base @ inverse_matrix(T_odom_base)
            translation, quaternion = matrix_to_transform(T_map_odom)
            _, _, yaw = euler_from_quaternion(quaternion)

            effective_margin = max(margin - self.min_decision_margin + 1.0, 1.0)
            effective_distance = max(distance, 0.25)
            weight = effective_margin / (effective_distance * effective_distance)
            candidates.append({'id': tag_id, 'x': float(translation[0]), 'y': float(translation[1]), 'yaw': wrap_angle(yaw), 'distance': distance, 'margin': margin, 'weight': weight})

        self.pending_detection = None

        if not candidates:
            with self.lock:
                self.last_visible_tag_ids = visible_ids
            return

        reference = max(candidates, key=lambda c: c['weight'])
        accepted = []
        for candidate in candidates:
            position_disagreement = math.hypot(candidate['x'] - reference['x'], candidate['y'] - reference['y'])
            yaw_disagreement = abs(angle_difference(candidate['yaw'], reference['yaw']))
            if position_disagreement <= self.max_position_disagreement and yaw_disagreement <= self.max_yaw_disagreement:
                accepted.append(candidate)

        if not accepted:
            return

        weight_sum = sum(c['weight'] for c in accepted)
        fused_x = sum(c['weight'] * c['x'] for c in accepted) / weight_sum
        fused_y = sum(c['weight'] * c['y'] for c in accepted) / weight_sum
        fused_yaw = math.atan2(sum(c['weight'] * math.sin(c['yaw']) for c in accepted), sum(c['weight'] * math.cos(c['yaw']) for c in accepted))
        raw_state = [fused_x, fused_y, fused_yaw]

        raw_map_to_base = self.state_to_matrix(raw_state) @ T_odom_base
        self.publish_pose(self.raw_pose_pub, raw_map_to_base, stamp)

        active_tags = Int32MultiArray()
        active_tags.data = [c['id'] for c in accepted]
        self.active_tags_pub.publish(active_tags)

        best = max(accepted, key=lambda c: c['weight'])
        with self.lock:
            self.last_visible_tag_ids = [c['id'] for c in accepted]
            self.last_best_distance = float(best['distance'])
            self.last_best_margin = float(best['margin'])
            if self.acquisition_active and self.is_stationary():
                self.acquisition_samples.append({'state': raw_state, 'tag_ids': [c['id'] for c in accepted]})
                if len(self.acquisition_samples) > 100:
                    self.acquisition_samples = self.acquisition_samples[-100:]

    def goal_callback(self, goal_request):
        with self.lock:
            if self.acquisition_active:
                return GoalResponse.REJECT
        if goal_request.preferred_tag_id not in [-1, 0, 1, 2, 3]:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_relocalize(self, goal_handle):
        sample_count = int(goal_handle.request.sample_count) if goal_handle.request.sample_count > 0 else self.default_sample_count
        timeout = float(goal_handle.request.timeout_sec) if goal_handle.request.timeout_sec > 0.0 else self.default_timeout

        with self.lock:
            self.acquisition_active = True
            self.acquisition_preferred_tag = int(goal_handle.request.preferred_tag_id)
            self.acquisition_samples = []

        self.get_logger().info(f'Relocalization acquisition started: tag={self.acquisition_preferred_tag}, samples={sample_count}, timeout={timeout:.1f}s')
        start = time.monotonic()
        last_quality_message = 'waiting for stable samples'

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.finish_acquisition()
                goal_handle.canceled()
                result = Relocalize.Result()
                result.success = False
                result.message = 'Relocalization canceled.'
                return result

            with self.lock:
                samples = list(self.acquisition_samples)
                visible_ids = list(self.last_visible_tag_ids)
                best_distance = self.last_best_distance
                best_margin = self.last_best_margin

            feedback = Relocalize.Feedback()
            feedback.samples_collected = len(samples)
            feedback.stationary = self.is_stationary()
            feedback.visible_tag_ids = visible_ids
            feedback.best_tag_distance = float(best_distance if math.isfinite(best_distance) else -1.0)
            feedback.best_decision_margin = float(best_margin if math.isfinite(best_margin) else -1.0)
            goal_handle.publish_feedback(feedback)

            if len(samples) >= sample_count:
                quality = self.evaluate_batch(samples[-sample_count:])
                last_quality_message = quality['message']
                if quality['success']:
                    old_state = self.map_to_odom_state
                    new_state = quality['state']
                    self.map_to_odom_state = new_state
                    delta = [0.0, 0.0, 0.0] if old_state is None else [new_state[0] - old_state[0], new_state[1] - old_state[1], angle_difference(new_state[2], old_state[2])]

                    try:
                        tf_odom_base = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time(), timeout=Duration(seconds=0.1))
                        committed_map_to_base = self.state_to_matrix(new_state) @ transform_to_matrix(tf_odom_base.transform)
                        self.publish_pose(self.committed_pose_pub, committed_map_to_base, self.get_clock().now())
                    except TransformException:
                        pass

                    self.finish_acquisition()
                    goal_handle.succeed()
                    result = Relocalize.Result()
                    result.success = True
                    result.used_tag_ids = quality['used_tag_ids']
                    result.map_to_odom_x = float(new_state[0])
                    result.map_to_odom_y = float(new_state[1])
                    result.map_to_odom_yaw = float(new_state[2])
                    result.delta_x = float(delta[0])
                    result.delta_y = float(delta[1])
                    result.delta_yaw = float(delta[2])
                    result.std_x = float(quality['std_x'])
                    result.std_y = float(quality['std_y'])
                    result.std_yaw = float(quality['std_yaw'])
                    result.message = quality['message']
                    self.get_logger().info(f'Relocalization committed: map->odom=({new_state[0]:.3f}, {new_state[1]:.3f}, {math.degrees(new_state[2]):.2f} deg)')
                    return result

            if time.monotonic() - start >= timeout:
                self.finish_acquisition()
                goal_handle.abort()
                result = Relocalize.Result()
                result.success = False
                result.message = f'Relocalization timed out: {last_quality_message}.'
                return result

            time.sleep(0.05)

        self.finish_acquisition()
        result = Relocalize.Result()
        result.success = False
        result.message = 'ROS shutdown.'
        return result

    def finish_acquisition(self):
        with self.lock:
            self.acquisition_active = False
            self.acquisition_preferred_tag = -1
            self.acquisition_samples = []

    def evaluate_batch(self, batch):
        states = [sample['state'] for sample in batch]
        median_x = sorted(s[0] for s in states)[len(states) // 2]
        median_y = sorted(s[1] for s in states)[len(states) // 2]
        initial_yaw = math.atan2(sum(math.sin(s[2]) for s in states), sum(math.cos(s[2]) for s in states))

        kept = []
        for sample in batch:
            state = sample['state']
            if math.hypot(state[0] - median_x, state[1] - median_y) <= self.batch_outlier_position and abs(angle_difference(state[2], initial_yaw)) <= self.batch_outlier_yaw:
                kept.append(sample)

        minimum_keep = max(3, math.ceil(len(batch) * self.minimum_batch_keep_ratio))
        if len(kept) < minimum_keep:
            return {'success': False, 'message': f'only {len(kept)}/{len(batch)} samples survived outlier rejection'}

        xs = [s['state'][0] for s in kept]
        ys = [s['state'][1] for s in kept]
        yaws = [s['state'][2] for s in kept]
        mean_x = sum(xs) / len(xs)
        mean_y = sum(ys) / len(ys)
        mean_yaw = math.atan2(sum(math.sin(y) for y in yaws), sum(math.cos(y) for y in yaws))
        std_x = math.sqrt(sum((x - mean_x) ** 2 for x in xs) / len(xs))
        std_y = math.sqrt(sum((y - mean_y) ** 2 for y in ys) / len(ys))
        std_yaw = math.sqrt(sum(angle_difference(y, mean_yaw) ** 2 for y in yaws) / len(yaws))

        if max(std_x, std_y) > self.max_batch_position_std:
            return {'success': False, 'message': f'position batch std too high: sx={std_x:.3f}, sy={std_y:.3f}'}
        if std_yaw > self.max_batch_yaw_std:
            return {'success': False, 'message': f'yaw batch std too high: {math.degrees(std_yaw):.2f} deg'}

        used_tag_ids = sorted(set(tag_id for sample in kept for tag_id in sample['tag_ids']))
        return {'success': True, 'state': [mean_x, mean_y, mean_yaw], 'std_x': std_x, 'std_y': std_y, 'std_yaw': std_yaw, 'used_tag_ids': used_tag_ids, 'message': f'accepted {len(kept)}/{len(batch)} samples'}

    @staticmethod
    def state_to_matrix(state):
        x, y, yaw = state
        quaternion = quaternion_from_euler(0.0, 0.0, yaw)
        return concatenate_matrices(translation_matrix([x, y, 0.0]), quaternion_matrix(quaternion))

    def publish_pose(self, publisher, matrix, stamp):
        translation, quaternion = matrix_to_transform(matrix)
        pose = PoseStamped()
        pose.header.stamp = stamp.to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = float(translation[0])
        pose.pose.position.y = float(translation[1])
        pose.pose.position.z = float(translation[2])
        pose.pose.orientation.x = float(quaternion[0])
        pose.pose.orientation.y = float(quaternion[1])
        pose.pose.orientation.z = float(quaternion[2])
        pose.pose.orientation.w = float(quaternion[3])
        publisher.publish(pose)

    def broadcast_map_to_odom(self):
        if self.map_to_odom_state is None:
            return
        x, y, yaw = self.map_to_odom_state
        quaternion = quaternion_from_euler(0.0, 0.0, yaw)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = TagGlobalLocalizer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
