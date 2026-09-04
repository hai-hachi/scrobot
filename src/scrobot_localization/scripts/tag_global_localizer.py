#!/usr/bin/env python3

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Int32MultiArray

from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)

from tf_transformations import (
    concatenate_matrices,
    euler_from_quaternion,
    inverse_matrix,
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_from_matrix,
    translation_matrix,
)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_difference(target, source):
    return wrap_angle(target - source)


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


def matrix_to_transform(matrix):
    return translation_from_matrix(matrix), quaternion_from_matrix(matrix)


class TagGlobalLocalizer(Node):

    def __init__(self):
        super().__init__('tag_global_localizer')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('detections_topic', '/apriltag/detections')
        self.declare_parameter('observed_tag_prefix', 'observed_tag_')

        self.declare_parameter('min_decision_margin', 20.0)
        self.declare_parameter('max_tag_distance', 6.0)
        self.declare_parameter('max_position_disagreement', 0.10)
        self.declare_parameter('max_yaw_disagreement_deg', 5.0)

        self.declare_parameter('filter_tau', 0.25)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('processing_rate', 100.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.observed_tag_prefix = self.get_parameter('observed_tag_prefix').value

        self.min_decision_margin = float(
            self.get_parameter('min_decision_margin').value
        )
        self.max_tag_distance = float(
            self.get_parameter('max_tag_distance').value
        )
        self.max_position_disagreement = float(
            self.get_parameter('max_position_disagreement').value
        )
        self.max_yaw_disagreement = math.radians(
            float(self.get_parameter('max_yaw_disagreement_deg').value)
        )
        self.filter_tau = float(self.get_parameter('filter_tau').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        processing_rate = float(self.get_parameter('processing_rate').value)

        self.tag_ids = [0, 1, 2, 3]
        self.tag_map = {}

        for tag_id in self.tag_ids:
            xyz_name = f'tag_{tag_id}.xyz'
            rpy_name = f'tag_{tag_id}.rpy'

            self.declare_parameter(xyz_name, [0.0, 0.0, 0.0])
            self.declare_parameter(rpy_name, [0.0, 0.0, 0.0])

            xyz = list(self.get_parameter(xyz_name).value)
            rpy = list(self.get_parameter(rpy_name).value)
            self.tag_map[tag_id] = xyz_rpy_to_matrix(xyz, rpy)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.pending_detection = None
        self.filtered_map_to_odom = None  # [x, y, yaw]
        self.last_measurement_stamp = None

        self.raw_pose_pub = self.create_publisher(
            PoseStamped,
            '/global_localization/tag_pose_raw',
            10,
        )
        self.filtered_pose_pub = self.create_publisher(
            PoseStamped,
            '/global_localization/tag_pose_filtered',
            10,
        )
        self.active_tags_pub = self.create_publisher(
            Int32MultiArray,
            '/global_localization/active_tags',
            10,
        )

        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self.detection_callback,
            10,
        )

        self.processing_timer = self.create_timer(
            1.0 / processing_rate,
            self.process_pending_detection,
        )
        self.broadcast_timer = self.create_timer(
            1.0 / publish_rate,
            self.broadcast_map_to_odom,
        )

        self.publish_known_tags()

        self.get_logger().info(
            'Tag global localizer V2 started: multi-tag fusion + gating + filtering'
        )

    def publish_known_tags(self):
        transforms = []
        now = self.get_clock().now().to_msg()

        for tag_id in self.tag_ids:
            translation, quaternion = matrix_to_transform(self.tag_map[tag_id])

            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = self.map_frame
            transform.child_frame_id = f'court_tag_{tag_id}'

            transform.transform.translation.x = float(translation[0])
            transform.transform.translation.y = float(translation[1])
            transform.transform.translation.z = float(translation[2])

            transform.transform.rotation.x = float(quaternion[0])
            transform.transform.rotation.y = float(quaternion[1])
            transform.transform.rotation.z = float(quaternion[2])
            transform.transform.rotation.w = float(quaternion[3])

            transforms.append(transform)

        self.static_tf_broadcaster.sendTransform(transforms)

    def detection_callback(self, msg):
        # Do not query TF here. apriltag_ros publishes the detection
        # message before its corresponding tag TFs.
        if msg.detections:
            self.pending_detection = msg

    def process_pending_detection(self):
        if self.pending_detection is None:
            return

        msg = self.pending_detection
        stamp = Time.from_msg(msg.header.stamp)
        camera_frame = msg.header.frame_id

        if not self.tf_buffer.can_transform(
            camera_frame,
            self.base_frame,
            stamp,
            timeout=Duration(seconds=0.0),
        ):
            return

        if not self.tf_buffer.can_transform(
            self.odom_frame,
            self.base_frame,
            stamp,
            timeout=Duration(seconds=0.0),
        ):
            return

        try:
            tf_camera_base = self.tf_buffer.lookup_transform(
                camera_frame,
                self.base_frame,
                stamp,
                timeout=Duration(seconds=0.0),
            )
            tf_odom_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                stamp,
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            return

        T_camera_base = transform_to_matrix(tf_camera_base.transform)
        T_odom_base = transform_to_matrix(tf_odom_base.transform)

        candidates = []

        for detection in msg.detections:
            tag_id = int(detection.id)

            if tag_id not in self.tag_ids:
                continue
            if detection.hamming != 0:
                continue

            decision_margin = float(detection.decision_margin)
            if decision_margin < self.min_decision_margin:
                continue

            observed_tag_frame = self.observed_tag_prefix + str(tag_id)

            if not self.tf_buffer.can_transform(
                camera_frame,
                observed_tag_frame,
                stamp,
                timeout=Duration(seconds=0.0),
            ):
                continue

            try:
                tf_camera_tag = self.tf_buffer.lookup_transform(
                    camera_frame,
                    observed_tag_frame,
                    stamp,
                    timeout=Duration(seconds=0.0),
                )
            except TransformException:
                continue

            t = tf_camera_tag.transform.translation
            tag_distance = math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z)

            if tag_distance > self.max_tag_distance:
                continue

            T_camera_tag = transform_to_matrix(tf_camera_tag.transform)
            T_map_tag = self.tag_map[tag_id]

            T_map_camera = T_map_tag @ inverse_matrix(T_camera_tag)
            T_map_base = T_map_camera @ T_camera_base
            T_map_odom = T_map_base @ inverse_matrix(T_odom_base)

            translation, quaternion = matrix_to_transform(T_map_odom)
            _, _, yaw = euler_from_quaternion(quaternion)

            effective_margin = max(
                decision_margin - self.min_decision_margin + 1.0,
                1.0,
            )
            effective_distance = max(tag_distance, 0.25)

            weight = effective_margin / (effective_distance * effective_distance)

            candidates.append({
                'id': tag_id,
                'x': float(translation[0]),
                'y': float(translation[1]),
                'yaw': wrap_angle(yaw),
                'distance': tag_distance,
                'margin': decision_margin,
                'weight': weight,
            })

        if not candidates:
            return

        # This detection message was successfully processed.
        self.pending_detection = None

        # Use the strongest candidate as the consistency reference.
        reference = max(candidates, key=lambda candidate: candidate['weight'])

        accepted = []

        for candidate in candidates:
            position_disagreement = math.hypot(
                candidate['x'] - reference['x'],
                candidate['y'] - reference['y'],
            )
            yaw_disagreement = abs(
                angle_difference(candidate['yaw'], reference['yaw'])
            )

            if position_disagreement > self.max_position_disagreement:
                continue
            if yaw_disagreement > self.max_yaw_disagreement:
                continue

            accepted.append(candidate)

        if not accepted:
            return

        weight_sum = sum(candidate['weight'] for candidate in accepted)

        fused_x = sum(
            candidate['weight'] * candidate['x']
            for candidate in accepted
        ) / weight_sum

        fused_y = sum(
            candidate['weight'] * candidate['y']
            for candidate in accepted
        ) / weight_sum

        sin_sum = sum(
            candidate['weight'] * math.sin(candidate['yaw'])
            for candidate in accepted
        )
        cos_sum = sum(
            candidate['weight'] * math.cos(candidate['yaw'])
            for candidate in accepted
        )
        fused_yaw = math.atan2(sin_sum, cos_sum)

        raw_state = [fused_x, fused_y, fused_yaw]

        raw_map_to_odom = self.state_to_matrix(raw_state)
        raw_map_to_base = raw_map_to_odom @ T_odom_base
        self.publish_pose(self.raw_pose_pub, raw_map_to_base, stamp)

        # First absolute fix should be immediate. Only subsequent
        # corrections are filtered.
        if self.filtered_map_to_odom is None:
            self.filtered_map_to_odom = raw_state
        else:
            if self.last_measurement_stamp is None:
                dt = 0.0
            else:
                dt = (
                    stamp.nanoseconds - self.last_measurement_stamp.nanoseconds
                ) * 1e-9

            if self.filter_tau <= 0.0:
                alpha = 1.0
            elif dt <= 0.0:
                alpha = 0.0
            else:
                alpha = 1.0 - math.exp(-dt / self.filter_tau)

            old_x, old_y, old_yaw = self.filtered_map_to_odom

            filtered_x = old_x + alpha * (fused_x - old_x)
            filtered_y = old_y + alpha * (fused_y - old_y)
            filtered_yaw = wrap_angle(
                old_yaw + alpha * angle_difference(fused_yaw, old_yaw)
            )

            self.filtered_map_to_odom = [
                filtered_x,
                filtered_y,
                filtered_yaw,
            ]

        self.last_measurement_stamp = stamp

        filtered_map_to_odom_matrix = self.state_to_matrix(
            self.filtered_map_to_odom
        )
        filtered_map_to_base = filtered_map_to_odom_matrix @ T_odom_base
        self.publish_pose(
            self.filtered_pose_pub,
            filtered_map_to_base,
            stamp,
        )

        active_tags = Int32MultiArray()
        active_tags.data = [candidate['id'] for candidate in accepted]
        self.active_tags_pub.publish(active_tags)

    def state_to_matrix(self, state):
        x, y, yaw = state
        quaternion = quaternion_from_euler(0.0, 0.0, yaw)
        return concatenate_matrices(
            translation_matrix([x, y, 0.0]),
            quaternion_matrix(quaternion),
        )

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
        if self.filtered_map_to_odom is None:
            return

        x, y, yaw = self.filtered_map_to_odom
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

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()