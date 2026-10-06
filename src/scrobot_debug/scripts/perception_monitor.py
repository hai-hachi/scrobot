#!/usr/bin/env python3

import time

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from vision_msgs.msg import Detection3DArray


class PerceptionMonitor(Node):
    def __init__(self):
        super().__init__('perception_monitor')
        self.declare_parameter('report_rate', 1.0)
        self.declare_parameter('stale_timeout', 2.0)
        self.report_rate = float(self.get_parameter('report_rate').value)
        self.stale_timeout = float(self.get_parameter('stale_timeout').value)
        self.last = {key: None for key in [
            'rgb', 'color_info', 'depth_info', 'aligned_depth',
            'scan', 'apriltag', 'yolo3d'
        ]}
        self.scan_valid = 0
        self.scan_total = 0
        self.tag_count = 0
        self.shuttle_count = 0
        self.color_info = None
        self.depth_info = None

        self.create_subscription(Image, '/camera/camera/color/image_raw', lambda msg: self._touch('rgb'), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self._color_info_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera/depth/camera_info', self._depth_info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', lambda msg: self._touch('aligned_depth'), qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/camera/camera/depth/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(AprilTagDetectionArray, '/apriltag/detections', self._apriltag_cb, qos_profile_sensor_data)
        self.create_subscription(Detection3DArray, '/perception/shuttle_detections_3d', self._yolo_cb, qos_profile_sensor_data)

        self.timer = self.create_timer(1.0 / self.report_rate, self._report)
        self.get_logger().info('Perception monitor started.')

    @staticmethod
    def _now():
        return time.perf_counter()

    def _touch(self, key):
        self.last[key] = self._now()

    def _color_info_cb(self, msg):
        self._touch('color_info')
        self.color_info = msg

    def _depth_info_cb(self, msg):
        self._touch('depth_info')
        self.depth_info = msg

    def _scan_cb(self, msg):
        self._touch('scan')
        self.scan_total = len(msg.ranges)
        self.scan_valid = sum(1 for value in msg.ranges if value == value and msg.range_min <= value <= msg.range_max)

    def _apriltag_cb(self, msg):
        self._touch('apriltag')
        self.tag_count = len(msg.detections)

    def _yolo_cb(self, msg):
        self._touch('yolo3d')
        self.shuttle_count = len(msg.detections)

    def _state(self, key):
        stamp = self.last[key]
        if stamp is None:
            return 'WAIT'
        age = self._now() - stamp
        return 'OK' if age <= self.stale_timeout else f'STALE({age:.1f}s)'

    @staticmethod
    def _info_text(info):
        if info is None or len(info.k) < 9:
            return 'none'
        return f'{info.width}x{info.height} fx={info.k[0]:.2f} fy={info.k[4]:.2f} cx={info.k[2]:.2f} cy={info.k[5]:.2f}'

    def _report(self):
        node_names = set(self.get_node_names())
        yolo_node = 'OK' if 'yolo_shuttle_detector' in node_names else 'MISSING'
        apriltag_node = 'OK' if 'apriltag' in node_names else 'MISSING'
        scan_node = 'OK' if 'depth_pointcloud_to_scan' in node_names else 'MISSING'
        filter_node = 'OK' if 'depth_scan_self_filter' in node_names else 'MISSING'

        self.get_logger().info(
            '[PERCEPTION] '
            f'NODES[yolo={yolo_node},tag={apriltag_node},scan={scan_node},filter={filter_node}] | '
            f"RGB={self._state('rgb')} | ALIGN={self._state('aligned_depth')} | "
            f"SCAN={self._state('scan')} valid={self.scan_valid}/{self.scan_total} | "
            f"TAG={self._state('apriltag')} n={self.tag_count} | "
            f"YOLO3D={self._state('yolo3d')} n={self.shuttle_count}"
        )
        if self.color_info is not None:
            self.get_logger().info('[CAMERA INFO] color ' + self._info_text(self.color_info))
        if self.depth_info is not None:
            self.get_logger().info('[CAMERA INFO] depth ' + self._info_text(self.depth_info))


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionMonitor()
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
