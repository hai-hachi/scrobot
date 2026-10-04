#!/usr/bin/env python3

import csv
import hashlib
import math
import os
import struct
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x/n, y/n, z/n, w/n)


def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )


def quat_rotate(q, v):
    qn = quat_normalize(q)
    vx, vy, vz = v
    rq = quat_multiply(
        quat_multiply(qn, (vx, vy, vz, 0.0)),
        quat_conjugate(qn),
    )
    return np.array([rq[0], rq[1], rq[2]], dtype=np.float64)


def pose_tuple(pose):
    p = pose.position
    q = pose.orientation
    return (
        np.array([float(p.x), float(p.y), float(p.z)], dtype=np.float64),
        quat_normalize((float(q.x), float(q.y), float(q.z), float(q.w))),
    )


def transform_tuple(transform):
    t = transform.translation
    q = transform.rotation
    return (
        np.array([float(t.x), float(t.y), float(t.z)], dtype=np.float64),
        quat_normalize((float(q.x), float(q.y), float(q.z), float(q.w))),
    )


def compose(parent_t, parent_q, child_t, child_q):
    return (
        parent_t + quat_rotate(parent_q, child_t),
        quat_normalize(quat_multiply(parent_q, child_q)),
    )


def world_to_frame(frame_t, frame_q, point_world):
    return quat_rotate(quat_conjugate(frame_q), point_world - frame_t)


def load_stl_vertices(path):
    path = Path(path)
    raw = path.read_bytes()

    # Binary STL: 80-byte header + uint32 triangle count + 50 bytes/triangle.
    if len(raw) >= 84:
        tri_count = struct.unpack_from('<I', raw, 80)[0]
        if 84 + tri_count * 50 == len(raw):
            vertices = []
            offset = 84
            for _ in range(tri_count):
                offset += 12  # normal
                for _ in range(3):
                    vertices.append(struct.unpack_from('<fff', raw, offset))
                    offset += 12
                offset += 2
            return np.asarray(vertices, dtype=np.float64)

    # ASCII fallback.
    vertices = []
    for line in raw.decode('utf-8', errors='ignore').splitlines():
        parts = line.strip().split()
        if len(parts) == 4 and parts[0].lower() == 'vertex':
            vertices.append(tuple(float(x) for x in parts[1:]))
    if not vertices:
        raise RuntimeError(f'Could not parse STL vertices: {path}')
    return np.asarray(vertices, dtype=np.float64)


def image_to_bgr(msg):
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise RuntimeError(f'Unsupported image encoding: {msg.encoding}')
    channels = 3
    row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = row[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img.copy()


def yaw_from_quat(q):
    x, y, z, w = q
    return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))


class YoloDatasetCapture(Node):
    def __init__(self):
        super().__init__('yolo_dataset_capture')

        self.declare_parameter('output_dir', str(Path.home() / 'Desktop' / 'yoloshuttle' / 'dataset' / 'gazebo_scrobot'))
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('shuttle_ground_truth_topic', '/evaluation/shuttle_ground_truth')
        self.declare_parameter('ground_truth_odom_topic', '/evaluation/ground_truth_odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('capture_rate', 2.0)
        self.declare_parameter('min_focus_range', 0.50)
        self.declare_parameter('max_focus_range', 1.68)
        self.declare_parameter('negative_keep_probability', 0.20)
        self.declare_parameter('min_box_pixels', 3.0)
        self.declare_parameter('jpeg_quality', 95)
        self.declare_parameter('position_group_m', 0.50)
        self.declare_parameter('yaw_group_deg', 30.0)
        self.declare_parameter('session_name', '')

        self.output_dir = Path(str(self.get_parameter('output_dir').value)).expanduser().resolve()
        self.capture_rate = max(0.1, float(self.get_parameter('capture_rate').value))
        self.min_focus_range = float(self.get_parameter('min_focus_range').value)
        self.max_focus_range = float(self.get_parameter('max_focus_range').value)
        self.negative_keep_probability = min(1.0, max(0.0, float(self.get_parameter('negative_keep_probability').value)))
        self.min_box_pixels = max(1.0, float(self.get_parameter('min_box_pixels').value))
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.position_group_m = max(0.05, float(self.get_parameter('position_group_m').value))
        self.yaw_group_rad = math.radians(max(1.0, float(self.get_parameter('yaw_group_deg').value)))
        requested_session = str(self.get_parameter('session_name').value).strip()
        self.session_name = requested_session or time.strftime('s%Y%m%d_%H%M%S')

        sim_share = Path(get_package_share_directory('scrobot_simulation'))
        self.mesh_vertices = load_stl_vertices(sim_share / 'models' / 'shuttle' / 'meshes' / 'shuttle.STL')
        self.get_logger().info(f'Loaded {len(self.mesh_vertices)} shuttle STL vertices.')

        self.camera_info = None
        self.latest_image = None
        self.robot_pose_world = None
        self.shuttle_poses_world = []
        self.received_shuttle_gt = False
        self.base_to_camera = None
        self.frame_index = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._prepare_output()

        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self.image_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self.camera_info_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter('shuttle_ground_truth_topic').value),
            self.shuttle_gt_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('ground_truth_odom_topic').value),
            self.odom_cb,
            qos_profile_sensor_data,
        )

        self.create_timer(1.0 / self.capture_rate, self.capture)
        self.get_logger().info(f'Writing synthetic YOLO dataset to {self.output_dir}')

    def _prepare_output(self):
        for split in ('train', 'val', 'test'):
            (self.output_dir / 'images' / split).mkdir(parents=True, exist_ok=True)
            (self.output_dir / 'labels' / split).mkdir(parents=True, exist_ok=True)

        data_yaml = self.output_dir / 'data.yaml'
        data_yaml.write_text(
            f'path: {self.output_dir}\n'
            'train: images/train\n'
            'val: images/val\n'
            'test: images/test\n\n'
            'names:\n'
            '  0: Shuttlecock\n'
        )

        self.metadata_path = self.output_dir / 'metadata.csv'
        if not self.metadata_path.exists():
            with self.metadata_path.open('w', newline='') as f:
                csv.writer(f).writerow([
                    'frame', 'session', 'split', 'robot_x', 'robot_y', 'robot_yaw_rad',
                    'boxes', 'focus_boxes', 'group_key'
                ])

    def image_cb(self, msg):
        self.latest_image = msg

    def camera_info_cb(self, msg):
        if msg.width > 0 and msg.height > 0 and len(msg.k) >= 9:
            self.camera_info = msg

    def shuttle_gt_cb(self, msg):
        self.received_shuttle_gt = True
        self.shuttle_poses_world = [pose_tuple(p) for p in msg.poses]

    def odom_cb(self, msg):
        self.robot_pose_world = pose_tuple(msg.pose.pose)

    def _lookup_camera(self):
        if self.base_to_camera is not None:
            return True
        try:
            tf = self.tf_buffer.lookup_transform(
                str(self.get_parameter('base_frame').value),
                str(self.get_parameter('camera_frame').value),
                Time(),
            )
        except TransformException:
            return False
        self.base_to_camera = transform_tuple(tf.transform)
        return True

    def _split_for_pose(self):
        robot_t, robot_q = self.robot_pose_world
        yaw = yaw_from_quat(robot_q)
        gx = int(round(robot_t[0] / self.position_group_m))
        gy = int(round(robot_t[1] / self.position_group_m))
        gyaw = int(round(yaw / self.yaw_group_rad))
        key = f'{gx}:{gy}:{gyaw}'
        bucket = int(hashlib.sha1(key.encode()).hexdigest()[:8], 16) % 100
        if bucket < 70:
            split = 'train'
        elif bucket < 85:
            split = 'val'
        else:
            split = 'test'
        return split, key, yaw

    def _project_shuttle(self, shuttle_t, shuttle_q, camera_t, camera_q):
        info = self.camera_info
        fx, fy, cx, cy = float(info.k[0]), float(info.k[4]), float(info.k[2]), float(info.k[5])

        uv = []
        center_cam = world_to_frame(camera_t, camera_q, shuttle_t)
        center_range = float(np.linalg.norm(center_cam))

        for v in self.mesh_vertices:
            point_world = shuttle_t + quat_rotate(shuttle_q, v)
            point_cam = world_to_frame(camera_t, camera_q, point_world)
            x, y, z = point_cam
            if z <= 1e-4:
                continue
            u = fx * x / z + cx
            vv = fy * y / z + cy
            uv.append((u, vv))

        if len(uv) < 3:
            return None

        arr = np.asarray(uv, dtype=np.float64)
        u0, v0 = np.min(arr[:, 0]), np.min(arr[:, 1])
        u1, v1 = np.max(arr[:, 0]), np.max(arr[:, 1])

        if u1 < 0 or v1 < 0 or u0 >= info.width or v0 >= info.height:
            return None

        u0 = max(0.0, min(float(info.width - 1), u0))
        u1 = max(0.0, min(float(info.width - 1), u1))
        v0 = max(0.0, min(float(info.height - 1), v0))
        v1 = max(0.0, min(float(info.height - 1), v1))

        bw = u1 - u0
        bh = v1 - v0
        if bw < self.min_box_pixels or bh < self.min_box_pixels:
            return None

        xc = (u0 + u1) * 0.5 / info.width
        yc = (v0 + v1) * 0.5 / info.height
        nw = bw / info.width
        nh = bh / info.height

        return (xc, yc, nw, nh, center_range)

    def capture(self):
        if (
            self.latest_image is None
            or self.camera_info is None
            or self.robot_pose_world is None
            or not self.received_shuttle_gt
            or not self._lookup_camera()
        ):
            return

        robot_t, robot_q = self.robot_pose_world
        base_cam_t, base_cam_q = self.base_to_camera
        camera_t, camera_q = compose(robot_t, robot_q, base_cam_t, base_cam_q)

        boxes = []
        focus_boxes = 0
        for shuttle_t, shuttle_q in self.shuttle_poses_world:
            projected = self._project_shuttle(shuttle_t, shuttle_q, camera_t, camera_q)
            if projected is None:
                continue
            xc, yc, w, h, distance = projected
            boxes.append((xc, yc, w, h, distance))
            if self.min_focus_range <= distance <= self.max_focus_range:
                focus_boxes += 1

        # Positive frames are always useful. Negative frames are deterministically
        # subsampled so repeated empty views do not dominate the dataset.
        if not boxes:
            digest = int(hashlib.sha1(str(self.frame_index).encode()).hexdigest()[:8], 16)
            if (digest % 10000) / 10000.0 >= self.negative_keep_probability:
                self.frame_index += 1
                return

        # Prefer frames containing at least one shuttle in the operating range.
        # Out-of-focus positive frames are still retained because unlabeled visible
        # shuttles would be harmful training data.
        split, group_key, yaw = self._split_for_pose()
        stem = f'{self.session_name}_gazebo_{self.frame_index:07d}'
        img_path = self.output_dir / 'images' / split / f'{stem}.jpg'
        label_path = self.output_dir / 'labels' / split / f'{stem}.txt'

        try:
            bgr = image_to_bgr(self.latest_image)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return

        ok = cv2.imwrite(
            str(img_path),
            bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            self.get_logger().error(f'Failed to write {img_path}')
            return

        with label_path.open('w') as f:
            for xc, yc, w, h, _ in boxes:
                f.write(f'0 {xc:.8f} {yc:.8f} {w:.8f} {h:.8f}\n')

        robot_t, _ = self.robot_pose_world
        with self.metadata_path.open('a', newline='') as f:
            csv.writer(f).writerow([
                stem, self.session_name, split,
                f'{robot_t[0]:.6f}', f'{robot_t[1]:.6f}', f'{yaw:.6f}',
                len(boxes), focus_boxes, group_key,
            ])

        if self.frame_index % 20 == 0:
            self.get_logger().info(
                f'saved={stem} split={split} boxes={len(boxes)} '
                f'focus={focus_boxes} output={self.output_dir}'
            )

        self.frame_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = YoloDatasetCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
