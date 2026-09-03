#!/usr/bin/env python3

import math

import numpy as np

import rclpy

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from apriltag_msgs.msg import AprilTagDetectionArray

from geometry_msgs.msg import (
    PoseStamped,
    TransformStamped,
)

from std_msgs.msg import Int32

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


# ==============================================================
# Transform helpers
# ==============================================================


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

    quaternion = quaternion_from_euler(
        rpy[0],
        rpy[1],
        rpy[2],
    )

    return concatenate_matrices(
        translation_matrix(xyz),
        quaternion_matrix(quaternion),
    )


def matrix_to_transform(matrix):

    translation = translation_from_matrix(
        matrix
    )

    quaternion = quaternion_from_matrix(
        matrix
    )

    return translation, quaternion


# ==============================================================
# Global localizer
# ==============================================================


class TagGlobalLocalizer(Node):

    def __init__(self):

        super().__init__(
            'tag_global_localizer'
        )

        # ======================================================
        # Parameters
        # ======================================================

        self.declare_parameter(
            'map_frame',
            'map'
        )

        self.declare_parameter(
            'odom_frame',
            'odom'
        )

        self.declare_parameter(
            'base_frame',
            'base_footprint'
        )

        self.declare_parameter(
            'detections_topic',
            '/apriltag/detections'
        )

        self.declare_parameter(
            'observed_tag_prefix',
            'observed_tag_'
        )

        self.declare_parameter(
            'min_decision_margin',
            20.0
        )

        self.declare_parameter(
            'max_tag_distance',
            3.0
        )

        self.declare_parameter(
            'publish_rate',
            30.0
        )

        self.map_frame = (
            self.get_parameter(
                'map_frame'
            ).value
        )

        self.odom_frame = (
            self.get_parameter(
                'odom_frame'
            ).value
        )

        self.base_frame = (
            self.get_parameter(
                'base_frame'
            ).value
        )

        self.detections_topic = (
            self.get_parameter(
                'detections_topic'
            ).value
        )

        self.observed_tag_prefix = (
            self.get_parameter(
                'observed_tag_prefix'
            ).value
        )

        self.min_decision_margin = (
            self.get_parameter(
                'min_decision_margin'
            ).value
        )

        self.max_tag_distance = (
            self.get_parameter(
                'max_tag_distance'
            ).value
        )

        publish_rate = (
            self.get_parameter(
                'publish_rate'
            ).value
        )

        # ======================================================
        # Court landmark parameters
        # ======================================================

        self.tag_ids = [0, 1, 2, 3]

        self.tag_map = {}

        for tag_id in self.tag_ids:

            xyz_name = (
                f'tag_{tag_id}.xyz'
            )

            rpy_name = (
                f'tag_{tag_id}.rpy'
            )

            self.declare_parameter(
                xyz_name,
                [0.0, 0.0, 0.0]
            )

            self.declare_parameter(
                rpy_name,
                [0.0, 0.0, 0.0]
            )

            xyz = list(
                self.get_parameter(
                    xyz_name
                ).value
            )

            rpy = list(
                self.get_parameter(
                    rpy_name
                ).value
            )

            self.tag_map[tag_id] = (
                xyz_rpy_to_matrix(
                    xyz,
                    rpy
                )
            )

        # ======================================================
        # TF
        # ======================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.tf_broadcaster = (
            TransformBroadcaster(
                self
            )
        )

        self.static_tf_broadcaster = (
            StaticTransformBroadcaster(
                self
            )
        )

        # Last accepted map -> odom transform.
        #
        # None means:
        #
        # global localization has not happened yet.
        #
        self.latest_map_to_odom = None

        # ======================================================
        # Debug publishers
        # ======================================================

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/global_localization/tag_pose',
            10
        )

        self.active_tag_pub = (
            self.create_publisher(
                Int32,
                '/global_localization/active_tag',
                10
            )
        )

        # ======================================================
        # Detection subscriber
        # ======================================================

        self.detection_sub = (
            self.create_subscription(
                AprilTagDetectionArray,
                self.detections_topic,
                self.detection_callback,
                10
            )
        )

        # ======================================================
        # TF publishing timer
        # ======================================================

        self.broadcast_timer = (
            self.create_timer(
                1.0 / publish_rate,
                self.broadcast_map_to_odom
            )
        )

        # Publish known court landmarks once.
        self.publish_known_tags()

        self.get_logger().info(
            'Tag global localizer started'
        )

        self.get_logger().info(
            'Waiting for a valid AprilTag '
            'before publishing map -> odom'
        )

        self.pending_detection = None

        self.pending_timer = self.create_timer(
            0.02,  # 50 Hz checking
            self.process_pending_detection
        )

    # ==========================================================
    # Publish known landmark TFs
    # ==========================================================

    def publish_known_tags(self):

        transforms = []

        now = (
            self.get_clock()
            .now()
            .to_msg()
        )

        for tag_id in self.tag_ids:

            matrix = self.tag_map[tag_id]

            translation, quaternion = (
                matrix_to_transform(
                    matrix
                )
            )

            tf = TransformStamped()

            tf.header.stamp = now
            tf.header.frame_id = (
                self.map_frame
            )

            tf.child_frame_id = (
                f'court_tag_{tag_id}'
            )

            tf.transform.translation.x = (
                float(translation[0])
            )

            tf.transform.translation.y = (
                float(translation[1])
            )

            tf.transform.translation.z = (
                float(translation[2])
            )

            tf.transform.rotation.x = (
                float(quaternion[0])
            )

            tf.transform.rotation.y = (
                float(quaternion[1])
            )

            tf.transform.rotation.z = (
                float(quaternion[2])
            )

            tf.transform.rotation.w = (
                float(quaternion[3])
            )

            transforms.append(tf)

        self.static_tf_broadcaster.sendTransform(
            transforms
        )

    # ==========================================================
    # AprilTag callback
    # ==========================================================

    def detection_callback(self, msg):

        if not msg.detections:
            return

        candidates = []

        for detection in msg.detections:

            if detection.id not in self.tag_ids:
                continue

            if detection.hamming != 0:
                continue

            if (
                detection.decision_margin
                <
                self.min_decision_margin
            ):
                continue

            candidates.append(detection)

        if not candidates:
            return

        # strongest detection first
        candidates.sort(
            key=lambda d: d.decision_margin,
            reverse=True
        )

        detection = candidates[0]

        # IMPORTANT:
        # Do not lookup TF here.
        #
        # apriltag_ros publishes the detection
        # before publishing the corresponding TF.
        self.pending_detection = {
            'tag_id': int(detection.id),
            'decision_margin': float(
                detection.decision_margin
            ),
            'camera_frame': msg.header.frame_id,
            'stamp': Time.from_msg(
                msg.header.stamp
            ),
        }

    def process_pending_detection(self):

        if self.pending_detection is None:
            return

        data = self.pending_detection

        tag_id = data['tag_id']

        observed_tag_frame = (
            self.observed_tag_prefix
            +
            str(tag_id)
        )

        stamp = data['stamp']
        camera_frame = data['camera_frame']

        # ==========================================================
        # Non-blocking TF availability checks
        # ==========================================================

        if not self.tf_buffer.can_transform(
            camera_frame,
            observed_tag_frame,
            stamp,
            timeout=Duration(seconds=0.0)
        ):
            return

        if not self.tf_buffer.can_transform(
            camera_frame,
            self.base_frame,
            stamp,
            timeout=Duration(seconds=0.0)
        ):
            return

        if not self.tf_buffer.can_transform(
            self.odom_frame,
            self.base_frame,
            stamp,
            timeout=Duration(seconds=0.0)
        ):
            return

        # All transforms for the image timestamp
        # are now actually in the TF buffer.

        success = self.process_tag(
            tag_id,
            data['decision_margin'],
            camera_frame,
            stamp
        )

        if success:
            self.pending_detection = None

    # ==========================================================
    # Process one tag
    # ==========================================================

    def process_tag(
        self,
        tag_id,
        decision_margin,
        camera_frame,
        measurement_time
    ):

        observed_tag_frame = (
            self.observed_tag_prefix
            +
            str(tag_id)
        )

        try:

            # ==================================================
            # camera -> observed tag
            # ==================================================

            tf_camera_tag = (
                self.tf_buffer.lookup_transform(
                    camera_frame,
                    observed_tag_frame,
                    measurement_time,
                    timeout=Duration(
                        seconds=0.0
                    )
                )
            )

            # ==================================================
            # camera -> base
            # ==================================================

            tf_camera_base = (
                self.tf_buffer.lookup_transform(
                    camera_frame,
                    self.base_frame,
                    measurement_time,
                    timeout=Duration(
                        seconds=0.0
                    )
                )
            )

            # ==================================================
            # odom -> base
            # ==================================================

            tf_odom_base = (
                self.tf_buffer.lookup_transform(
                    self.odom_frame,
                    self.base_frame,
                    measurement_time,
                    timeout=Duration(
                        seconds=0.0
                    )
                )
            )

        except TransformException as exception:

            self.get_logger().debug(
                f'TF lookup failed for '
                f'tag {tag_id}: '
                f'{exception}'
            )

            return False

        # ======================================================
        # Reject excessively distant tiny tags
        # ======================================================

        t = tf_camera_tag.transform.translation

        tag_distance = math.sqrt(
            t.x * t.x
            +
            t.y * t.y
            +
            t.z * t.z
        )

        if (
            tag_distance
            >
            self.max_tag_distance
        ):

            self.get_logger().debug(
                f'Rejected tag {tag_id}: '
                f'{tag_distance:.2f} m > '
                f'{self.max_tag_distance:.2f} m'
            )

            return False

        # ======================================================
        # Convert transforms to homogeneous matrices
        # ======================================================

        # ^camera T_tag
        T_camera_tag = transform_to_matrix(
            tf_camera_tag.transform
        )

        # ^camera T_base
        T_camera_base = transform_to_matrix(
            tf_camera_base.transform
        )

        # ^odom T_base
        T_odom_base = transform_to_matrix(
            tf_odom_base.transform
        )

        # ^map T_tag
        T_map_tag = self.tag_map[
            tag_id
        ]

        # ======================================================
        # Calculate map -> camera
        #
        # ^map T_camera
        #
        # =
        #
        # ^map T_tag
        #
        # *
        #
        # inverse(^camera T_tag)
        # ======================================================

        T_map_camera = (
            T_map_tag
            @
            inverse_matrix(
                T_camera_tag
            )
        )

        # ======================================================
        # Calculate map -> base
        #
        # ^map T_base
        #
        # =
        #
        # ^map T_camera
        #
        # *
        #
        # ^camera T_base
        # ======================================================

        T_map_base = (
            T_map_camera
            @
            T_camera_base
        )

        # ======================================================
        # Calculate map -> odom
        #
        # ^map T_odom
        #
        # =
        #
        # ^map T_base
        #
        # *
        #
        # inverse(^odom T_base)
        # ======================================================

        T_map_odom = (
            T_map_base
            @
            inverse_matrix(
                T_odom_base
            )
        )

        # ======================================================
        # Project map -> odom into 2D
        #
        # Our robot_localization EKF uses two_d_mode.
        #
        # Therefore:
        #
        # z     = 0
        # roll  = 0
        # pitch = 0
        #
        # only x, y, yaw remain.
        # ======================================================

        translation, quaternion = (
            matrix_to_transform(
                T_map_odom
            )
        )

        _, _, yaw = euler_from_quaternion(
            quaternion
        )

        quaternion_2d = quaternion_from_euler(
            0.0,
            0.0,
            yaw
        )

        map_to_odom = TransformStamped()

        map_to_odom.header.frame_id = (
            self.map_frame
        )

        map_to_odom.child_frame_id = (
            self.odom_frame
        )

        map_to_odom.transform.translation.x = (
            float(translation[0])
        )

        map_to_odom.transform.translation.y = (
            float(translation[1])
        )

        map_to_odom.transform.translation.z = 0.0

        map_to_odom.transform.rotation.x = (
            float(quaternion_2d[0])
        )

        map_to_odom.transform.rotation.y = (
            float(quaternion_2d[1])
        )

        map_to_odom.transform.rotation.z = (
            float(quaternion_2d[2])
        )

        map_to_odom.transform.rotation.w = (
            float(quaternion_2d[3])
        )

        self.latest_map_to_odom = (
            map_to_odom
        )

        # ======================================================
        # Publish debug global robot pose
        # ======================================================

        self.publish_robot_pose(
            T_map_base,
            measurement_time
        )

        active_tag_msg = Int32()

        active_tag_msg.data = int(
            tag_id
        )

        self.active_tag_pub.publish(
            active_tag_msg
        )

        self.get_logger().info(
            f'Global correction from '
            f'tag {tag_id}: '
            f'd={tag_distance:.2f} m, '
            f'margin={decision_margin:.1f}, '
            f'map->odom='
            f'({translation[0]:.3f}, '
            f'{translation[1]:.3f}, '
            f'{math.degrees(yaw):.1f} deg)'
        )

        return True

    # ==========================================================
    # Debug pose
    # ==========================================================

    def publish_robot_pose(
        self,
        T_map_base,
        stamp
    ):

        translation, quaternion = (
            matrix_to_transform(
                T_map_base
            )
        )

        pose = PoseStamped()

        pose.header.stamp = (
            stamp.to_msg()
        )

        pose.header.frame_id = (
            self.map_frame
        )

        pose.pose.position.x = float(
            translation[0]
        )

        pose.pose.position.y = float(
            translation[1]
        )

        pose.pose.position.z = float(
            translation[2]
        )

        pose.pose.orientation.x = float(
            quaternion[0]
        )

        pose.pose.orientation.y = float(
            quaternion[1]
        )

        pose.pose.orientation.z = float(
            quaternion[2]
        )

        pose.pose.orientation.w = float(
            quaternion[3]
        )

        self.pose_pub.publish(
            pose
        )

    # ==========================================================
    # Continuous map -> odom broadcaster
    # ==========================================================

    def broadcast_map_to_odom(self):

        if self.latest_map_to_odom is None:
            return

        # map -> odom itself remains fixed between global
        # corrections, but TF must continue to be published
        # with a current timestamp.

        self.latest_map_to_odom.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        self.tf_broadcaster.sendTransform(
            self.latest_map_to_odom
        )


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