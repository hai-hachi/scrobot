#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose


def quat_normalize(q):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    inv = 1.0 / norm
    return (x * inv, y * inv, z * inv, w * inv)


def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


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
    vx, vy, vz = v
    vq = (vx, vy, vz, 0.0)
    rotated = quat_multiply(quat_multiply(qn, vq), quat_conjugate(qn))
    return (rotated[0], rotated[1], rotated[2])


def vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def compose_pose(parent_t, parent_q, child_t, child_q):
    """Compose world_T_parent * parent_T_child -> world_T_child."""
    out_t = vec_add(parent_t, quat_rotate(parent_q, child_t))
    out_q = quat_normalize(quat_multiply(parent_q, child_q))
    return out_t, out_q


def transform_point_inverse(frame_t, frame_q, point_world):
    """Transform a world point into a frame whose pose is world_T_frame."""
    relative = vec_sub(point_world, frame_t)
    return quat_rotate(quat_conjugate(quat_normalize(frame_q)), relative)


def transform_to_tuple(transform):
    t = transform.translation
    q = transform.rotation
    return (
        (float(t.x), float(t.y), float(t.z)),
        quat_normalize((float(q.x), float(q.y), float(q.z), float(q.w))),
    )


def pose_to_tuple(pose):
    p = pose.position
    q = pose.orientation
    return (
        (float(p.x), float(p.y), float(p.z)),
        quat_normalize((float(q.x), float(q.y), float(q.z), float(q.w))),
    )


class FakeShuttleDetector(Node):
    """Camera-limited shuttle detector driven by Gazebo ground truth.

    The node publishes the same Detection3DArray contract planned for the real
    RGB-D pipeline. Gazebo truth is used only inside this simulation adapter.
    Visibility is decided in the color optical frame using CameraInfo, then the
    accepted 3D measurement is expressed in the depth optical frame.
    """

    def __init__(self):
        super().__init__('fake_shuttle_detector')

        self.declare_parameter('update_rate', 15.0)
        self.declare_parameter('min_range', 0.20)
        self.declare_parameter('max_range', 3.00)
        self.declare_parameter('class_id', 'shuttle')
        self.declare_parameter('bbox_size_x', 0.08)
        self.declare_parameter('bbox_size_y', 0.08)
        self.declare_parameter('bbox_size_z', 0.10)

        self.declare_parameter(
            'shuttle_ground_truth_topic',
            '/evaluation/shuttle_ground_truth',
        )
        self.declare_parameter(
            'ground_truth_odom_topic',
            '/evaluation/ground_truth_odom',
        )
        self.declare_parameter(
            'camera_info_topic',
            '/camera/camera/color/camera_info',
        )
        self.declare_parameter(
            'output_topic',
            '/perception/shuttle_detections_3d',
        )

        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('color_frame', 'camera_color_optical_frame')
        self.declare_parameter('depth_frame', 'camera_depth_optical_frame')

        self.update_rate = float(self.get_parameter('update_rate').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.class_id = str(self.get_parameter('class_id').value)
        self.bbox_size = (
            float(self.get_parameter('bbox_size_x').value),
            float(self.get_parameter('bbox_size_y').value),
            float(self.get_parameter('bbox_size_z').value),
        )

        self.shuttle_gt_topic = str(
            self.get_parameter('shuttle_ground_truth_topic').value
        )
        self.gt_odom_topic = str(
            self.get_parameter('ground_truth_odom_topic').value
        )
        self.camera_info_topic = str(
            self.get_parameter('camera_info_topic').value
        )
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.color_frame = str(self.get_parameter('color_frame').value)
        self.depth_frame = str(self.get_parameter('depth_frame').value)

        if self.update_rate <= 0.0:
            raise ValueError('update_rate must be > 0')
        if self.min_range < 0.0:
            raise ValueError('min_range must be >= 0')
        if self.max_range <= self.min_range:
            raise ValueError('max_range must be greater than min_range')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._camera_info = None
        self._robot_pose_world = None
        self._robot_stamp = None
        self._shuttle_positions_world = []
        self._received_shuttle_gt = False

        self._base_to_color = None
        self._base_to_depth = None

        self._reported_ready = False
        self._waiting_log_counter = 0

        self.create_subscription(
            PoseArray,
            self.shuttle_gt_topic,
            self._shuttle_ground_truth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.gt_odom_topic,
            self._ground_truth_odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )

        self.publisher = self.create_publisher(
            Detection3DArray,
            self.output_topic,
            qos_profile_sensor_data,
        )

        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self._timer_callback,
        )

        self.get_logger().info(
            'Fake shuttle detector started: '
            f'{self.update_rate:.1f} Hz, '
            f'range={self.min_range:.2f}-{self.max_range:.2f} m, '
            f'input={self.shuttle_gt_topic}, '
            f'output={self.output_topic}'
        )

    def _shuttle_ground_truth_callback(self, msg):
        self._received_shuttle_gt = True
        self._shuttle_positions_world = [
            (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            )
            for pose in msg.poses
        ]

    def _ground_truth_odom_callback(self, msg):
        self._robot_pose_world = pose_to_tuple(msg.pose.pose)
        self._robot_stamp = msg.header.stamp

    def _camera_info_callback(self, msg):
        if msg.width <= 0 or msg.height <= 0:
            return
        if len(msg.k) < 9 or msg.k[0] <= 0.0 or msg.k[4] <= 0.0:
            return
        self._camera_info = msg

    def _lookup_static_camera_transforms(self):
        if self._base_to_color is None:
            try:
                stamped = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.color_frame,
                    Time(),
                )
                self._base_to_color = transform_to_tuple(stamped.transform)
            except TransformException:
                return False

        if self._base_to_depth is None:
            try:
                stamped = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.depth_frame,
                    Time(),
                )
                self._base_to_depth = transform_to_tuple(stamped.transform)
            except TransformException:
                return False

        return True

    def _inputs_ready(self):
        return (
            self._camera_info is not None
            and self._robot_pose_world is not None
            and self._received_shuttle_gt
            and self._lookup_static_camera_transforms()
        )

    def _maybe_log_waiting(self):
        self._waiting_log_counter += 1
        interval = max(1, int(round(self.update_rate * 5.0)))
        if self._waiting_log_counter % interval != 0:
            return

        missing = []
        if self._camera_info is None:
            missing.append('CameraInfo')
        if self._robot_pose_world is None:
            missing.append('ground-truth odometry')
        if not self._received_shuttle_gt:
            missing.append('shuttle ground truth')
        if self._base_to_color is None or self._base_to_depth is None:
            missing.append('base->camera TF')

        self.get_logger().warn('Waiting for: ' + ', '.join(missing))

    def _visible_in_color_camera(self, point_color):
        x, y, z = point_color
        if z <= 0.0:
            return False

        distance = math.sqrt(x * x + y * y + z * z)
        if distance < self.min_range or distance > self.max_range:
            return False

        info = self._camera_info
        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])

        u = fx * x / z + cx
        v = fy * y / z + cy

        return 0.0 <= u < float(info.width) and 0.0 <= v < float(info.height)

    def _make_detection(self, point_depth, stamp):
        detection = Detection3D()
        detection.header.stamp = stamp
        detection.header.frame_id = self.depth_frame
        detection.id = ''

        x, y, z = point_depth
        detection.bbox.center.position.x = x
        detection.bbox.center.position.y = y
        detection.bbox.center.position.z = z
        detection.bbox.center.orientation.w = 1.0
        detection.bbox.size.x = self.bbox_size[0]
        detection.bbox.size.y = self.bbox_size[1]
        detection.bbox.size.z = self.bbox_size[2]

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = self.class_id
        hypothesis.hypothesis.score = 1.0
        hypothesis.pose.pose.position.x = x
        hypothesis.pose.pose.position.y = y
        hypothesis.pose.pose.position.z = z
        hypothesis.pose.pose.orientation.w = 1.0
        detection.results.append(hypothesis)

        return detection

    def _measurement_stamp(self):
        # Geometry is computed from the latest Gazebo ground-truth odometry, so
        # stamp the synthetic detection with that same simulation timestamp.
        # This avoids mixing UNIX wall time with Gazebo /clock / TF history.
        if self._robot_stamp is not None:
            if self._robot_stamp.sec != 0 or self._robot_stamp.nanosec != 0:
                return self._robot_stamp
        return self.get_clock().now().to_msg()

    def _timer_callback(self):
        if not self._inputs_ready():
            self._maybe_log_waiting()
            return

        if not self._reported_ready:
            self._reported_ready = True
            self.get_logger().info(
                'Inputs ready; publishing camera-limited shuttle detections.'
            )

        world_to_base_t, world_to_base_q = self._robot_pose_world
        base_to_color_t, base_to_color_q = self._base_to_color
        base_to_depth_t, base_to_depth_q = self._base_to_depth

        world_to_color_t, world_to_color_q = compose_pose(
            world_to_base_t,
            world_to_base_q,
            base_to_color_t,
            base_to_color_q,
        )
        world_to_depth_t, world_to_depth_q = compose_pose(
            world_to_base_t,
            world_to_base_q,
            base_to_depth_t,
            base_to_depth_q,
        )

        stamp = self._measurement_stamp()
        output = Detection3DArray()
        output.header.stamp = stamp
        output.header.frame_id = self.depth_frame

        for point_world in self._shuttle_positions_world:
            point_color = transform_point_inverse(
                world_to_color_t,
                world_to_color_q,
                point_world,
            )

            if not self._visible_in_color_camera(point_color):
                continue

            point_depth = transform_point_inverse(
                world_to_depth_t,
                world_to_depth_q,
                point_world,
            )
            output.detections.append(
                self._make_detection(point_depth, stamp)
            )

        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = FakeShuttleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
