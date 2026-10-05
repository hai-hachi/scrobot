#!/usr/bin/env python3

import csv
import hashlib
import json
import math
import os
import random
import struct
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


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


def quat_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return quat_normalize((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def world_to_frame(frame_t, frame_q, point_world):
    return quat_rotate(quat_conjugate(frame_q), point_world - frame_t)


def quat_to_rotmat(q):
    x, y, z, w = quat_normalize(q)
    return np.array([
        [1.0 - 2.0*(y*y + z*z), 2.0*(x*y - z*w), 2.0*(x*z + y*w)],
        [2.0*(x*y + z*w), 1.0 - 2.0*(x*x + z*z), 2.0*(y*z - x*w)],
        [2.0*(x*z - y*w), 2.0*(y*z + x*w), 1.0 - 2.0*(x*x + y*y)],
    ], dtype=np.float64)


def load_stl_vertices(path):
    path = Path(path)
    raw = path.read_bytes()

    if len(raw) >= 84:
        tri_count = struct.unpack_from('<I', raw, 80)[0]
        if 84 + tri_count * 50 == len(raw):
            vertices = []
            offset = 84
            for _ in range(tri_count):
                offset += 12
                for _ in range(3):
                    vertices.append(struct.unpack_from('<fff', raw, offset))
                    offset += 12
                offset += 2
            return np.asarray(vertices, dtype=np.float64)

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

    row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = row[:, :msg.width * 3].reshape(msg.height, msg.width, 3)

    if msg.encoding == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img.copy()


class FastYoloDatasetCapture(Node):
    def __init__(self):
        super().__init__('yolo_dataset_fast_capture')

        self.declare_parameter(
            'output_dir',
            str(Path.home() / 'Desktop' / 'yoloshuttle' / 'dataset' / 'gazebo_scrobot'),
        )
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('status_topic', '/debug/yolo_dataset_status')

        self.declare_parameter('world_name', 'yolo_dataset')
        self.declare_parameter('camera_entity', 'yolo_camera_rig')
        self.declare_parameter('camera_trigger_topic', '/yolo/camera/color/trigger')
        self.declare_parameter('target_images', 1200)
        self.declare_parameter('shuttle_count', 40)
        self.declare_parameter('spawn_workers', 8)

        self.declare_parameter('random_seed', 42)
        self.declare_parameter('positive_pose_fraction', 0.85)
        self.declare_parameter('negative_keep_probability', 0.35)
        self.declare_parameter('min_focus_range', 0.50)
        self.declare_parameter('max_focus_range', 1.68)
        self.declare_parameter('yaw_jitter_deg', 25.0)

        self.declare_parameter('camera_height', 0.28683059)
        self.declare_parameter('camera_pitch_deg', 15.0)
        self.declare_parameter('shuttle_z', 0.050)

        self.declare_parameter('court_length', 13.40)
        self.declare_parameter('court_width', 6.10)
        self.declare_parameter('court_margin_x', 0.45)
        self.declare_parameter('court_margin_y', 0.35)
        self.declare_parameter('net_exclusion_x', 0.45)

        self.declare_parameter('min_box_pixels', 3.0)
        self.declare_parameter('jpeg_quality', 95)
        self.declare_parameter('position_group_m', 0.50)
        self.declare_parameter('yaw_group_deg', 30.0)
        self.declare_parameter('session_name', '')

        self.output_dir = Path(
            str(self.get_parameter('output_dir').value)
        ).expanduser().resolve()

        self.world_name = str(self.get_parameter('world_name').value)
        self.camera_entity = str(self.get_parameter('camera_entity').value)
        self.camera_trigger_topic = str(self.get_parameter('camera_trigger_topic').value)
        self.target_images = max(1, int(self.get_parameter('target_images').value))
        self.shuttle_count = max(1, int(self.get_parameter('shuttle_count').value))
        self.spawn_workers = max(1, int(self.get_parameter('spawn_workers').value))

        self.rng = random.Random(int(self.get_parameter('random_seed').value))
        self.positive_pose_fraction = min(
            1.0, max(0.0, float(self.get_parameter('positive_pose_fraction').value))
        )
        self.negative_keep_probability = min(
            1.0, max(0.0, float(self.get_parameter('negative_keep_probability').value))
        )
        self.min_focus_range = float(self.get_parameter('min_focus_range').value)
        self.max_focus_range = float(self.get_parameter('max_focus_range').value)
        self.yaw_jitter_rad = math.radians(
            max(0.0, float(self.get_parameter('yaw_jitter_deg').value))
        )

        self.camera_height = float(self.get_parameter('camera_height').value)
        self.camera_pitch = math.radians(
            float(self.get_parameter('camera_pitch_deg').value)
        )
        self.shuttle_z = float(self.get_parameter('shuttle_z').value)

        self.court_length = float(self.get_parameter('court_length').value)
        self.court_width = float(self.get_parameter('court_width').value)
        self.court_margin_x = max(
            0.0, float(self.get_parameter('court_margin_x').value)
        )
        self.court_margin_y = max(
            0.0, float(self.get_parameter('court_margin_y').value)
        )
        self.net_exclusion_x = max(
            0.0, float(self.get_parameter('net_exclusion_x').value)
        )

        self.min_box_pixels = max(
            1.0, float(self.get_parameter('min_box_pixels').value)
        )
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.position_group_m = max(
            0.05, float(self.get_parameter('position_group_m').value)
        )
        self.yaw_group_rad = math.radians(
            max(1.0, float(self.get_parameter('yaw_group_deg').value))
        )

        requested_session = str(self.get_parameter('session_name').value).strip()
        self.session_name = requested_session or time.strftime('s%Y%m%d_%H%M%S')

        sim_share = Path(get_package_share_directory('scrobot_simulation'))
        debug_share = Path(get_package_share_directory('scrobot_debug'))

        # Render the detailed shuttle.STL in Gazebo, but label from the accepted
        # low-poly octagonal envelope. Projecting the detailed mesh meant
        # 156,846 vertex records per shuttle, i.e. >6.2 million Python vertex
        # projections per image with 40 shuttles.
        label_vertices = load_stl_vertices(
            sim_share / 'models' / 'shuttle' / 'meshes'
            / 'shuttle_collision_octagonal.stl'
        )
        self.label_mesh_vertices = np.unique(label_vertices, axis=0)
        self.get_logger().info(
            f'Label envelope vertices: {len(self.label_mesh_vertices)} '
            '(detailed visual mesh remains unchanged in Gazebo)'
        )
        self.static_shuttle_sdf = (
            debug_share / 'models' / 'yolo_shuttle_visual' / 'model.sdf'
        )

        self.latest_image = None
        self.camera_info = None
        self.shuttles = []
        self.scene_spawned = False

        self.camera_position = np.array(
            [2.0, 0.0, self.camera_height], dtype=np.float64
        )
        self.camera_yaw = 0.0
        self.camera_color_q = quat_from_rpy(0.0, self.camera_pitch, 0.0)

        # camera_color_frame -> camera_color_optical_frame
        self.color_to_optical_q = quat_from_rpy(-math.pi / 2.0, 0.0, -math.pi / 2.0)

        self.waiting_for_fresh_frame = False
        self.teleport_stamp_ns = 0
        self.frame_index = 0
        self.saved_images = 0
        self.pose_attempts = 0
        self.pose_kind = 'waiting'
        self.finished = False
        self.state = 'WAITING_FOR_CAMERA'
        self.initial_trigger_sent = False
        self.finalized = False

        self.last_saved_split = ''
        self.last_saved_boxes = 0
        self.last_saved_focus_boxes = 0
        self.last_reported_state = None
        self.last_timings = {
            'teleport_ms': 0.0,
            'trigger_ms': 0.0,
            'wait_rgb_ms': 0.0,
            'label_ms': 0.0,
            'write_ms': 0.0,
        }
        self.wait_rgb_wall_start = None

        self._prepare_output()

        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )

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

        # Wall-clock timer intentionally used through normal ROS timer. The world
        # runs with real_time_factor=0 (as fast as possible), so this loop is not
        # gated by a deliberate simulation settle interval.
        self.create_timer(0.01, self.capture_step)

        self.get_logger().info(
            f'FAST YOLO dataset capture target={self.target_images}, '
            f'shuttles={self.shuttle_count}, output={self.output_dir}'
        )

    def _prepare_output(self):
        for split in ('train', 'val', 'test'):
            (self.output_dir / 'images' / split).mkdir(parents=True, exist_ok=True)
            (self.output_dir / 'labels' / split).mkdir(parents=True, exist_ok=True)

        (self.output_dir / 'data.yaml').write_text(
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
                    'frame',
                    'session',
                    'split',
                    'camera_x',
                    'camera_y',
                    'camera_yaw_rad',
                    'boxes',
                    'focus_boxes',
                    'group_key',
                ])

    def image_cb(self, msg):
        self.latest_image = msg

    def camera_info_cb(self, msg):
        if msg.width > 0 and msg.height > 0 and len(msg.k) >= 9:
            self.camera_info = msg

    @staticmethod
    def _image_stamp_ns(msg):
        return (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )

    def _court_limits(self):
        return (
            self.court_length * 0.5 - self.court_margin_x,
            self.court_width * 0.5 - self.court_margin_y,
        )

    def _camera_pose_safe(self, x, y):
        half_l, half_w = self._court_limits()
        if not (-half_l <= x <= half_l and -half_w <= y <= half_w):
            return False
        return abs(x) >= self.net_exclusion_x

    def _sample_shuttle_layout(self):
        half_l = self.court_length * 0.5 - 0.15
        half_w = self.court_width * 0.5 - 0.15

        cluster_centers = [
            (
                self.rng.uniform(-half_l, half_l),
                self.rng.uniform(-half_w, half_w),
            )
            for _ in range(6)
        ]

        specs = []
        for index in range(self.shuttle_count):
            if self.rng.random() < 0.35:
                cx, cy = self.rng.choice(cluster_centers)
                x = max(-half_l, min(half_l, self.rng.gauss(cx, 0.30)))
                y = max(-half_w, min(half_w, self.rng.gauss(cy, 0.30)))
            else:
                x = self.rng.uniform(-half_l, half_l)
                y = self.rng.uniform(-half_w, half_w)

            yaw = self.rng.uniform(-math.pi, math.pi)
            pitch = math.pi / 2.0
            q = quat_from_rpy(0.0, pitch, yaw)
            shuttle_t = np.array([x, y, self.shuttle_z], dtype=np.float64)
            shuttle_r = quat_to_rotmat(q)
            world_vertices = (
                (shuttle_r @ self.label_mesh_vertices.T).T + shuttle_t
            )
            specs.append({
                'name': f'yolo_shuttle_{index:03d}',
                't': shuttle_t,
                'q': q,
                'world_vertices': world_vertices,
                'roll': 0.0,
                'pitch': pitch,
                'yaw': yaw,
            })

        return specs

    def _spawn_one(self, spec):
        t = spec['t']
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', self.world_name,
            '-name', spec['name'],
            '-file', str(self.static_shuttle_sdf),
            '-x', f'{t[0]:.9f}',
            '-y', f'{t[1]:.9f}',
            '-z', f'{t[2]:.9f}',
            '-R', f"{spec['roll']:.9f}",
            '-P', f"{spec['pitch']:.9f}",
            '-Y', f"{spec['yaw']:.9f}",
        ]

        result = subprocess.run(
            cmd,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"{spec['name']} spawn failed: {result.stdout.strip()}"
            )

    def _spawn_scene(self):
        self._set_state('SPAWNING_STATIC_SHUTTLES')
        specs = self._sample_shuttle_layout()

        failures = []
        workers = min(self.spawn_workers, len(specs))
        with ThreadPoolExecutor(max_workers=workers) as executor:
            futures = {
                executor.submit(self._spawn_one, spec): spec
                for spec in specs
            }
            for future in as_completed(futures):
                spec = futures[future]
                try:
                    future.result()
                except Exception as exc:
                    failures.append(str(exc))

        if failures:
            for failure in failures[:5]:
                self.get_logger().error(failure)
            raise RuntimeError(
                f'{len(failures)} static shuttle spawns failed.'
            )

        self.shuttles = specs
        self.scene_spawned = True
        self._set_state('STATIC_SCENE_READY')
        self.get_logger().info(
            f'Spawned {len(self.shuttles)} static visual-only shuttles.'
        )

    def _sample_random_camera_pose(self):
        half_l, half_w = self._court_limits()
        for _ in range(64):
            x = self.rng.uniform(-half_l, half_l)
            y = self.rng.uniform(-half_w, half_w)
            if self._camera_pose_safe(x, y):
                return (
                    np.array([x, y, self.camera_height], dtype=np.float64),
                    self.rng.uniform(-math.pi, math.pi),
                    'random',
                )
        return (
            np.array([2.0, 0.0, self.camera_height], dtype=np.float64),
            self.rng.uniform(-math.pi, math.pi),
            'random',
        )

    def _sample_target_camera_pose(self):
        if not self.shuttles:
            return None

        dz = self.camera_height - self.shuttle_z

        for _ in range(96):
            shuttle = self.rng.choice(self.shuttles)
            desired_range = self.rng.uniform(
                self.min_focus_range,
                self.max_focus_range,
            )

            horizontal = math.sqrt(
                max(0.05 * 0.05, desired_range * desired_range - dz * dz)
            )
            bearing = self.rng.uniform(-math.pi, math.pi)
            direction = np.array(
                [math.cos(bearing), math.sin(bearing)],
                dtype=np.float64,
            )

            camera_xy = shuttle['t'][:2] - horizontal * direction
            yaw = bearing + self.rng.uniform(
                -self.yaw_jitter_rad,
                self.yaw_jitter_rad,
            )

            if self._camera_pose_safe(camera_xy[0], camera_xy[1]):
                return (
                    np.array(
                        [camera_xy[0], camera_xy[1], self.camera_height],
                        dtype=np.float64,
                    ),
                    yaw,
                    'target',
                )

        return None

    def _trigger_camera(self):
        t0 = time.perf_counter()
        cmd = [
            'gz', 'topic',
            '-t', self.camera_trigger_topic,
            '-m', 'gz.msgs.Boolean',
            '-p', 'data: true',
        ]
        result = subprocess.run(
            cmd,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self.last_timings['trigger_ms'] = (time.perf_counter() - t0) * 1000.0
        if result.returncode != 0:
            self._set_state('CAMERA_TRIGGER_FAILED')
            self.get_logger().error(
                'Camera trigger failed: ' + result.stdout.strip()
            )
            return False
        return True

    def _teleport_camera(self, position, yaw):
        t0 = time.perf_counter()
        q = quat_from_rpy(0.0, self.camera_pitch, yaw)

        request = (
            f'name: "{self.camera_entity}", '
            f'position: {{x: {position[0]:.9f}, y: {position[1]:.9f}, z: {position[2]:.9f}}}, '
            f'orientation: {{x: {q[0]:.12f}, y: {q[1]:.12f}, '
            f'z: {q[2]:.12f}, w: {q[3]:.12f}}}'
        )

        cmd = [
            'gz', 'service',
            '-s', f'/world/{self.world_name}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000',
            '--req', request,
        ]

        result = subprocess.run(
            cmd,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        self.last_timings['teleport_ms'] = (time.perf_counter() - t0) * 1000.0
        if result.returncode != 0:
            self._set_state('CAMERA_TELEPORT_FAILED')
            self.get_logger().error(
                'Camera set_pose failed: ' + result.stdout.strip()
            )
            return False

        self.camera_position = position
        self.camera_yaw = yaw
        self.camera_color_q = q
        self.teleport_stamp_ns = self.get_clock().now().nanoseconds

        if not self._trigger_camera():
            return False

        self.waiting_for_fresh_frame = True
        self.wait_rgb_wall_start = time.perf_counter()
        self._set_state('WAITING_FOR_TRIGGERED_RGB')
        return True

    def _next_view(self):
        self.pose_attempts += 1

        if self.rng.random() < self.positive_pose_fraction:
            sampled = self._sample_target_camera_pose()
        else:
            sampled = None

        if sampled is None:
            sampled = self._sample_random_camera_pose()

        position, yaw, kind = sampled
        self.pose_kind = kind
        self._set_state('TELEPORTING_RGB_RIG')
        self._teleport_camera(position, yaw)

    def _camera_optical_pose(self):
        optical_q = quat_normalize(
            quat_multiply(self.camera_color_q, self.color_to_optical_q)
        )
        return self.camera_position, optical_q

    def _project_shuttle(self, shuttle, camera_t, world_to_camera_r):
        info = self.camera_info
        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])

        center_delta = shuttle['t'] - camera_t
        center_cam = world_to_camera_r @ center_delta
        center_range = float(np.linalg.norm(center_cam))

        # Fast center/FOV rejection before touching the envelope vertices.
        if center_cam[2] <= 0.02:
            return None

        points_cam = (
            world_to_camera_r
            @ (shuttle['world_vertices'] - camera_t).T
        ).T
        points_cam = points_cam[points_cam[:, 2] > 1e-4]

        if len(points_cam) < 3:
            return None

        u = fx * points_cam[:, 0] / points_cam[:, 2] + cx
        v = fy * points_cam[:, 1] / points_cam[:, 2] + cy

        u0 = float(np.min(u))
        u1 = float(np.max(u))
        v0 = float(np.min(v))
        v1 = float(np.max(v))

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

        return (
            (u0 + u1) * 0.5 / info.width,
            (v0 + v1) * 0.5 / info.height,
            bw / info.width,
            bh / info.height,
            center_range,
        )

    def _split_for_camera_pose(self):
        x = float(self.camera_position[0])
        y = float(self.camera_position[1])
        yaw = self.camera_yaw

        gx = int(round(x / self.position_group_m))
        gy = int(round(y / self.position_group_m))
        gyaw = int(round(yaw / self.yaw_group_rad))
        key = f'{gx}:{gy}:{gyaw}'

        bucket = int(
            hashlib.sha1(key.encode()).hexdigest()[:8],
            16,
        ) % 100

        if bucket < 70:
            split = 'train'
        elif bucket < 85:
            split = 'val'
        else:
            split = 'test'

        return split, key

    def _save_current_frame(self):
        label_t0 = time.perf_counter()
        camera_t, camera_q = self._camera_optical_pose()
        world_to_camera_r = quat_to_rotmat(camera_q).T

        boxes = []
        focus_boxes = 0
        for shuttle in self.shuttles:
            projected = self._project_shuttle(
                shuttle,
                camera_t,
                world_to_camera_r,
            )
            if projected is None:
                continue

            boxes.append(projected)
            if self.min_focus_range <= projected[4] <= self.max_focus_range:
                focus_boxes += 1

        self.last_timings['label_ms'] = (
            time.perf_counter() - label_t0
        ) * 1000.0

        if self.pose_kind == 'target' and focus_boxes <= 0:
            self.waiting_for_fresh_frame = False
            self._set_state('TARGET_VIEW_REJECTED')
            return

        if not boxes and self.rng.random() >= self.negative_keep_probability:
            self.waiting_for_fresh_frame = False
            self._set_state('NEGATIVE_VIEW_SKIPPED')
            return

        write_t0 = time.perf_counter()
        split, group_key = self._split_for_camera_pose()
        stem = f'{self.session_name}_gazebo_{self.frame_index:07d}'

        img_path = self.output_dir / 'images' / split / f'{stem}.jpg'
        label_path = self.output_dir / 'labels' / split / f'{stem}.txt'

        try:
            bgr = image_to_bgr(self.latest_image)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            self.waiting_for_fresh_frame = False
            return

        if not cv2.imwrite(
            str(img_path),
            bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        ):
            self.get_logger().error(f'Failed to write {img_path}')
            self.waiting_for_fresh_frame = False
            return

        with label_path.open('w') as f:
            for xc, yc, w, h, _ in boxes:
                f.write(f'0 {xc:.8f} {yc:.8f} {w:.8f} {h:.8f}\n')

        with self.metadata_path.open('a', newline='') as f:
            csv.writer(f).writerow([
                stem,
                self.session_name,
                split,
                f'{self.camera_position[0]:.6f}',
                f'{self.camera_position[1]:.6f}',
                f'{self.camera_yaw:.6f}',
                len(boxes),
                focus_boxes,
                group_key,
            ])

        self.last_timings['write_ms'] = (
            time.perf_counter() - write_t0
        ) * 1000.0
        if self.wait_rgb_wall_start is not None:
            self.last_timings['wait_rgb_ms'] = (
                time.perf_counter() - self.wait_rgb_wall_start
            ) * 1000.0
            self.wait_rgb_wall_start = None

        self.frame_index += 1
        self.saved_images += 1
        self.last_saved_split = split
        self.last_saved_boxes = len(boxes)
        self.last_saved_focus_boxes = focus_boxes
        self.waiting_for_fresh_frame = False
        self._set_state('FRAME_SAVED')

        self.get_logger().info(
            'FRAME '
            f'{self.saved_images}/{self.target_images} '
            f'[{100.0*self.saved_images/max(1,self.target_images):.1f}%] '
            f'split={split} boxes={len(boxes)} focus={focus_boxes} | '
            f"set_pose={self.last_timings['teleport_ms']:.0f}ms "
            f"trigger={self.last_timings['trigger_ms']:.0f}ms "
            f"wait_rgb={self.last_timings['wait_rgb_ms']:.0f}ms "
            f"label={self.last_timings['label_ms']:.1f}ms "
            f"write={self.last_timings['write_ms']:.1f}ms"
        )
        self.publish_status(force=True)

        if self.saved_images >= self.target_images:
            self.finished = True
            self._set_state('COMPLETE')
            self.get_logger().info(
                f'Fast dataset complete: {self.saved_images} images in {self.output_dir}'
            )
            self.finalize_partial_dataset(reason='complete')
            self.create_timer(0.10, lambda: rclpy.shutdown())

    def _set_state(self, state):
        if state == self.state:
            return
        self.state = state
        self.publish_status(force=True)

    def capture_step(self):
        if self.finished:
            return

        if self.latest_image is None or self.camera_info is None:
            self._set_state('WAITING_FOR_CAMERA')
            if not self.initial_trigger_sent:
                self.initial_trigger_sent = self._trigger_camera()
            return

        if not self.scene_spawned:
            try:
                self._spawn_scene()
            except Exception as exc:
                self._set_state('STATIC_SCENE_SPAWN_FAILED')
                self.get_logger().error(str(exc))
            return

        if not self.waiting_for_fresh_frame:
            self._next_view()
            return

        if self._image_stamp_ns(self.latest_image) <= self.teleport_stamp_ns:
            self._set_state('WAITING_FOR_TRIGGERED_RGB')
            return

        self._set_state('SAVING_FRAME')
        self._save_current_frame()

    def finalize_partial_dataset(self, reason='stopped'):
        if self.finalized:
            return

        self.finalized = True

        counts = {}
        total = 0
        for split in ('train', 'val', 'test'):
            image_dir = self.output_dir / 'images' / split
            label_dir = self.output_dir / 'labels' / split

            image_count = len(list(image_dir.glob('*.jpg'))) if image_dir.exists() else 0
            label_count = len(list(label_dir.glob('*.txt'))) if label_dir.exists() else 0

            counts[split] = {
                'images': image_count,
                'labels': label_count,
            }
            total += image_count

        usable = (
            counts['train']['images'] > 0
            and counts['val']['images'] > 0
        )

        summary = {
            'status': reason,
            'session': self.session_name,
            'saved_this_run': self.saved_images,
            'dataset_total_images': total,
            'target_images': self.target_images,
            'counts': counts,
            'training_ready': usable,
            'output_dir': str(self.output_dir),
        }

        (self.output_dir / 'dataset_summary.json').write_text(
            json.dumps(summary, indent=2, sort_keys=True) + '\n'
        )

        self.get_logger().info(
            'DATASET_FINALIZED '
            f"reason={reason} "
            f"saved_this_run={self.saved_images} "
            f"total={total} "
            f"train={counts['train']['images']} "
            f"val={counts['val']['images']} "
            f"test={counts['test']['images']} "
            f"training_ready={usable}"
        )

    def publish_status(self, force=False):
        if not force and self.state == self.last_reported_state:
            return

        self.last_reported_state = self.state
        progress = 100.0 * self.saved_images / max(1, self.target_images)

        line = (
            f'{self.state} | '
            f'{self.saved_images}/{self.target_images} ({progress:.1f}%) | '
            f'attempt={self.pose_attempts} view={self.pose_kind} | '
            f'last={self.last_saved_split or "-"} '
            f'boxes={self.last_saved_boxes} focus={self.last_saved_focus_boxes}'
        )

        msg = String()
        msg.data = line
        self.status_pub.publish(msg)

        # Console only on state changes / saved frames. No sim-time heartbeat,
        # so unlimited simulation time cannot flood repeated identical logs.
        self.get_logger().info(f'DATASET {line}')


def main(args=None):
    rclpy.init(args=args)
    node = FastYoloDatasetCapture()
    stop_reason = 'stopped'
    try:
        rclpy.spin(node)
        if node.finished:
            stop_reason = 'complete'
    except KeyboardInterrupt:
        stop_reason = 'interrupted'
    finally:
        try:
            node.finalize_partial_dataset(reason=stop_reason)
        except Exception as exc:
            node.get_logger().error(f'Failed to finalize partial dataset: {exc}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
