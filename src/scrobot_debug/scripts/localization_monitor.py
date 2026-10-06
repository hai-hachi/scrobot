#!/usr/bin/env python3

import math
import time

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformException, TransformListener


class LocalizationMonitor(Node):
    """Report AprilTag recognition range, image centering, and localization TF health."""

    def __init__(self):
        super().__init__('localization_monitor')

        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('detections_topic', '/apriltag/detections')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('observed_tag_prefix', 'observed_tag_')
        self.declare_parameter('report_rate', 1.0)
        self.declare_parameter('detection_stale_timeout', 1.0)

        self.camera_info = None
        self.latest_detections = []
        self.last_detection_monotonic = None
        self.max_seen_range = {}

        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.observed_tag_prefix = str(self.get_parameter('observed_tag_prefix').value)
        self.detection_stale_timeout = max(
            0.1, float(self.get_parameter('detection_stale_timeout').value)
        )

        detection_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self.camera_info_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            AprilTagDetectionArray,
            str(self.get_parameter('detections_topic').value),
            self.detections_cb,
            detection_qos,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        rate = max(0.2, float(self.get_parameter('report_rate').value))
        self.create_timer(1.0 / rate, self.report)

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def detections_cb(self, msg):
        self.latest_detections = list(msg.detections)
        self.last_detection_monotonic = time.monotonic()

    def tag_distance(self, tag_id):
        frame = self.observed_tag_prefix + str(tag_id)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.camera_frame,
                frame,
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return None

        t = tf.transform.translation
        return math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z)

    def tf_ok(self, parent, child):
        try:
            self.tf_buffer.lookup_transform(
                parent,
                child,
                Time(),
                timeout=Duration(seconds=0.02),
            )
            return True
        except TransformException:
            return False

    def report(self):
        local_tf = self.tf_ok('odom', 'base_footprint')
        global_tf = self.tf_ok('map', 'odom')

        if self.camera_info is None:
            self.get_logger().info(
                f'TF odom->base={"OK" if local_tf else "WAIT"} '
                f'map->odom={"OK" if global_tf else "WAIT"} | waiting for CameraInfo'
            )
            return

        info = self.camera_info
        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])

        stale = (
            self.last_detection_monotonic is None
            or time.monotonic() - self.last_detection_monotonic > self.detection_stale_timeout
        )
        detections = [] if stale else self.latest_detections

        prefix = (
            f'TF odom->base={"OK" if local_tf else "WAIT"} '
            f'map->odom={"OK" if global_tf else "WAIT"} | '
            f'camera={info.width}x{info.height} '
            f'fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}'
        )

        if not detections:
            self.get_logger().info(prefix + ' | TAGS=none')
            return

        entries = []
        for det in detections:
            tag_id = int(det.id)
            u = float(det.centre.x)
            v = float(det.centre.y)
            du = u - cx
            dv = v - cy
            h_angle = math.degrees(math.atan2(du, fx)) if fx > 0.0 else float('nan')
            v_angle = math.degrees(math.atan2(dv, fy)) if fy > 0.0 else float('nan')
            distance = self.tag_distance(tag_id)

            if distance is not None:
                previous = self.max_seen_range.get(tag_id, 0.0)
                self.max_seen_range[tag_id] = max(previous, distance)
                range_text = (
                    f'r={distance:.3f}m max={self.max_seen_range[tag_id]:.3f}m'
                )
            else:
                range_text = 'r=TF_WAIT'

            entries.append(
                f'id={tag_id} margin={float(det.decision_margin):.1f} '
                f'{range_text} center=({u:.1f},{v:.1f}) '
                f'dpx=({du:+.1f},{dv:+.1f}) '
                f'deg=({h_angle:+.2f},{v_angle:+.2f})'
            )

        self.get_logger().info(prefix + ' | ' + ' ; '.join(entries))


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
