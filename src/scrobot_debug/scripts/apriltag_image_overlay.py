#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import quaternion_matrix


class AprilTagImageOverlay(Node):
    """Presentation-only RGB overlay for AprilTag pose visualization."""

    def __init__(self):
        super().__init__('apriltag_image_overlay')

        self.declare_parameter(
            'image_topic',
            '/camera/camera/color/image_raw',
        )
        self.declare_parameter(
            'camera_info_topic',
            '/camera/camera/color/camera_info',
        )
        self.declare_parameter(
            'output_topic',
            '/debug/apriltag_overlay/image',
        )
        self.declare_parameter(
            'camera_frame',
            'camera_color_optical_frame',
        )
        self.declare_parameter('tag_frame_prefix', 'observed_tag_')
        self.declare_parameter('tag_ids', [0, 1, 2, 3])
        self.declare_parameter('tag_size', 0.100)
        self.declare_parameter('axis_length', 0.080)
        self.declare_parameter('tf_timeout', 0.01)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(
            self.get_parameter('camera_info_topic').value
        )
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.tag_frame_prefix = str(
            self.get_parameter('tag_frame_prefix').value
        )
        self.tag_ids = [
            int(v) for v in self.get_parameter('tag_ids').value
        ]
        self.tag_size = float(self.get_parameter('tag_size').value)
        self.axis_length = float(
            self.get_parameter('axis_length').value
        )
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)

        self.bridge = CvBridge()
        self.camera_info = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        self.publisher = self.create_publisher(
            Image,
            self.output_topic,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.image_topic,
            self._image_cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'AprilTag presentation overlay: {self.image_topic} -> '
            f'{self.output_topic}'
        )

    def _camera_info_cb(self, msg):
        if (
            msg.width > 0
            and msg.height > 0
            and len(msg.k) >= 9
            and msg.k[0] > 0.0
            and msg.k[4] > 0.0
        ):
            self.camera_info = msg

    @staticmethod
    def _transform_matrix(transform):
        q = transform.rotation
        matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
        matrix[0, 3] = float(transform.translation.x)
        matrix[1, 3] = float(transform.translation.y)
        matrix[2, 3] = float(transform.translation.z)
        return matrix

    def _project(self, point):
        if self.camera_info is None:
            return None
        x, y, z = [float(v) for v in point[:3]]
        if z <= 1e-4:
            return None

        k = self.camera_info.k
        u = float(k[0]) * x / z + float(k[2])
        v = float(k[4]) * y / z + float(k[5])
        return int(round(u)), int(round(v))

    def _lookup_tag(self, tag_id):
        try:
            return self.tf_buffer.lookup_transform(
                self.camera_frame,
                f'{self.tag_frame_prefix}{tag_id}',
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException:
            return None

    def _draw_tag(self, image, tag_id, transform):
        matrix = self._transform_matrix(transform)
        half = 0.5 * self.tag_size

        local = {
            'origin': np.array([0.0, 0.0, 0.0, 1.0]),
            'x': np.array([self.axis_length, 0.0, 0.0, 1.0]),
            'y': np.array([0.0, self.axis_length, 0.0, 1.0]),
            'z': np.array([0.0, 0.0, self.axis_length, 1.0]),
            'c0': np.array([-half, -half, 0.0, 1.0]),
            'c1': np.array([+half, -half, 0.0, 1.0]),
            'c2': np.array([+half, +half, 0.0, 1.0]),
            'c3': np.array([-half, +half, 0.0, 1.0]),
        }
        cam = {key: matrix @ value for key, value in local.items()}
        pix = {key: self._project(value) for key, value in cam.items()}

        origin = pix['origin']
        if origin is None:
            return

        corners = [pix[f'c{i}'] for i in range(4)]
        if all(point is not None for point in corners):
            polygon = np.array(corners, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(
                image,
                [polygon],
                True,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        # Standard presentation convention: X red, Y green, Z blue.
        for key, color in (
            ('x', (0, 0, 255)),
            ('y', (0, 255, 0)),
            ('z', (255, 0, 0)),
        ):
            endpoint = pix[key]
            if endpoint is not None:
                cv2.arrowedLine(
                    image,
                    origin,
                    endpoint,
                    color,
                    3,
                    cv2.LINE_AA,
                    tipLength=0.20,
                )

        distance = math.sqrt(
            cam['origin'][0] ** 2
            + cam['origin'][1] ** 2
            + cam['origin'][2] ** 2
        )
        label = f'Tag {tag_id}  {distance:.2f} m'
        x0 = max(8, origin[0] + 12)
        y0 = max(28, origin[1] - 12)
        cv2.putText(
            image,
            label,
            (x0, y0),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def _image_cb(self, msg):
        if self.camera_info is None:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8',
            )
        except Exception as exc:
            self.get_logger().warning(
                f'RGB conversion failed: {exc}'
            )
            return

        annotated = image.copy()
        for tag_id in self.tag_ids:
            transform = self._lookup_tag(tag_id)
            if transform is not None:
                self._draw_tag(annotated, tag_id, transform)

        output = self.bridge.cv2_to_imgmsg(
            annotated,
            encoding='bgr8',
        )
        output.header = msg.header
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagImageOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
