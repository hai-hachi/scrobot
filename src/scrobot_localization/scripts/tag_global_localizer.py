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

        # ======================================================
        # Frames / topics
        # ======================================================
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('detections_topic', '/apriltag/detections')
        self.declare_parameter('observed_tag_prefix', 'observed_tag_')
        self.declare_parameter('mount_frame_prefix', 'tag_mount_')
        self.declare_parameter('known_tag_prefix', 'court_tag_')

        # ======================================================
        # Physical court / tag geometry
        # ======================================================
        self.declare_parameter('pole_x', 0.0)
        self.declare_parameter('left_pole_y', 3.05)
        self.declare_parameter('right_pole_y', -3.05)
        self.declare_parameter('tag_height', 0.150)
        self.declare_parameter('tag_mount_radius', 0.075)
        self.declare_parameter('inward_angle_deg', 20.0)

        # These are also consumed by the Gazebo generator. They are
        # declared here so this same YAML can be the single source.
        self.declare_parameter('tag_edge_size', 0.100)
        self.declare_parameter('active_grid_cells', 6)
        self.declare_parameter('quiet_border_cells', 1)
        self.declare_parameter('texture_pixels', 1024)

        # ======================================================
        # Observation gating
        # ======================================================
        self.declare_parameter('min_decision_margin', 20.0)
        self.declare_parameter('max_tag_distance', 6.0)
        self.declare_parameter('max_position_disagreement', 0.15)
        self.declare_parameter('max_yaw_disagreement_deg', 6.0)

        # ======================================================
        # Filtering / timing
        # ======================================================
        self.declare_parameter('filter_tau', 0.25)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('processing_rate', 100.0)

        # ======================================================
        # Read parameters
        # ======================================================
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.observed_tag_prefix = self.get_parameter('observed_tag_prefix').value
        self.mount_frame_prefix = self.get_parameter('mount_frame_prefix').value
        self.known_tag_prefix = self.get_parameter('known_tag_prefix').value

        self.pole_x = float(self.get_parameter('pole_x').value)
        self.left_pole_y = float(self.get_parameter('left_pole_y').value)
        self.right_pole_y = float(self.get_parameter('right_pole_y').value)
        self.tag_height = float(self.get_parameter('tag_height').value)
        self.tag_mount_radius = float(self.get_parameter('tag_mount_radius').value)
        self.inward_angle_deg = float(self.get_parameter('inward_angle_deg').value)

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

        # ======================================================
        # Tag geometry
        # ======================================================
        self.tag_ids = [0, 1, 2, 3]

        # Human-intuitive physical mount transforms:
        #   tag_mount +X = visible-face outward normal
        #   tag_mount +Y = left
        #   tag_mount +Z = up
        self.mount_map = {}

        # Known frames matching christianrauch/apriltag_ros PnP output.
        self.tag_map = {}

        # For pose_estimation_method=pnp, the tag frame used by
        # christianrauch/apriltag_ros is:
        #   +X = right on tag image
        #   +Y = up on tag image
        #   +Z = out of tag toward observer/camera
        #
        # For our REP-103 mount frame viewed from +X_mount:
        #   image right = +Y_mount
        #   image up    = +Z_mount
        #   tag outward = +X_mount
        #
        # Therefore:
        #   X_april = +Y_mount
        #   Y_april = +Z_mount
        #   Z_april = +X_mount
        #
        # Fixed mount -> detector-tag transform:
        # RPY = [+90 deg, 0 deg, +90 deg]
        self.T_mount_apriltag = xyz_rpy_to_matrix(
            [0.0, 0.0, 0.0],
            [-math.pi / 2.0, 0.0, -math.pi / 2.0],
        )

        headings = self.compute_tag_headings(self.inward_angle_deg)

        for tag_id in self.tag_ids:
            heading = headings[tag_id]
            pole_y = self.left_pole_y if tag_id in (0, 2) else self.right_pole_y

            x = self.pole_x + self.tag_mount_radius * math.cos(heading)
            y = pole_y + self.tag_mount_radius * math.sin(heading)
            z = self.tag_height

            T_map_mount = xyz_rpy_to_matrix(
                [x, y, z],
                [0.0, 0.0, heading],
            )

            T_map_apriltag = T_map_mount @ self.T_mount_apriltag

            self.mount_map[tag_id] = T_map_mount
            self.tag_map[tag_id] = T_map_apriltag

            self.get_logger().info(
                f'tag {tag_id}: mount xyz=({x:.4f}, {y:.4f}, {z:.4f}), '
                f'heading={math.degrees(heading):.1f} deg'
            )

        # ======================================================
        # TF
        # ======================================================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # ======================================================
        # State
        # ======================================================
        self.pending_detection = None
        self.filtered_map_to_odom = None  # [x, y, yaw]
        self.last_measurement_stamp = None

        # ======================================================
        # Debug publishers
        # ======================================================
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

        # ======================================================
        # Detection subscriber
        # ======================================================
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self.detection_callback,
            10,
        )

        # ======================================================
        # Timers
        # ======================================================
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
            'Tag global localizer V3 started: '
            'REP-103 tag mounts -> fixed AprilTag PnP frames -> multi-tag fusion'
        )

    @staticmethod
    def compute_tag_headings(inward_angle_deg):
        """Return physical mount yaw for all four tags.

        Court convention:
          +X = longitudinal side A
          +Y = left side

        IDs:
          0 = left pole,  +X half
          1 = right pole, +X half
          2 = left pole,  -X half
          3 = right pole, -X half
        """
        a = math.radians(inward_angle_deg)
        return {
            0: -a,
            1: +a,
            2: -math.pi + a,
            3: +math.pi - a,
        }

    def publish_known_tags(self):
        """Publish intuitive mount frames plus detector-compatible tag frames."""
        transforms = []
        now = self.get_clock().now().to_msg()

        for tag_id in self.tag_ids:
            mount_frame = f'{self.mount_frame_prefix}{tag_id}'
            known_tag_frame = f'{self.known_tag_prefix}{tag_id}'

            # map -> tag_mount_N
            transforms.append(
                make_transform(
                    self.map_frame,
                    mount_frame,
                    self.mount_map[tag_id],
                    now,
                )
            )

            # tag_mount_N -> court_tag_N
            # Same fixed conversion for every tag.
            transforms.append(
                make_transform(
                    mount_frame,
                    known_tag_frame,
                    self.T_mount_apriltag,
                    now,
                )
            )

        self.static_tf_broadcaster.sendTransform(transforms)

    def detection_callback(self, msg):
        # apriltag_ros publishes detections before the matching tag TFs.
        # Cache the message and process it from the timer after TF arrives.
        if msg.detections:
            self.pending_detection = msg

    def process_pending_detection(self):
        if self.pending_detection is None:
            return

        msg = self.pending_detection
        stamp = Time.from_msg(msg.header.stamp)
        camera_frame = msg.header.frame_id

        # The following historical transforms must exist at the image stamp.
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

            # Known tag pose + measured tag pose -> absolute camera pose.
            T_map_camera = T_map_tag @ inverse_matrix(T_camera_tag)
            T_map_base = T_map_camera @ T_camera_base

            # Preserve smooth local odometry; correct only map -> odom.
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

        # This detection message was successfully consumed.
        self.pending_detection = None

        # ======================================================
        # Multi-tag consistency gating
        # ======================================================
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

        # ======================================================
        # Weighted SE(2) fusion
        # ======================================================
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

        # Raw tag-derived robot pose for debugging.
        raw_map_to_odom = self.state_to_matrix(raw_state)
        raw_map_to_base = raw_map_to_odom @ T_odom_base
        self.publish_pose(self.raw_pose_pub, raw_map_to_base, stamp)

        # ======================================================
        # Temporal filter
        # ======================================================
        if self.filtered_map_to_odom is None:
            # First valid absolute fix initializes immediately.
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

    @staticmethod
    def state_to_matrix(state):
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
