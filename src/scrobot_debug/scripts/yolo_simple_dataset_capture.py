#!/usr/bin/env python3

import csv
import hashlib
import math
import random
import shutil
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
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray


class SimpleYoloDatasetCapture(Node):
    def __init__(self):
        super().__init__('yolo_simple_dataset_capture')

        self.declare_parameter(
            'output_dir',
            str(Path.home() / 'Desktop' / 'yoloshuttle' / 'dataset' / 'gazebo_scrobot_simple'),
        )
        self.declare_parameter('world_name', 'yolo_dataset_native')
        self.declare_parameter('camera_entity', 'yolo_bbox_camera_rig')
        self.declare_parameter('target_images', 1200)
        self.declare_parameter('shuttle_count', 40)
        self.declare_parameter('spawn_workers', 8)
        self.declare_parameter('random_seed', 42)
        self.declare_parameter('settle_time_s', 2.0)
        self.declare_parameter('camera_wait_s', 0.10)
        self.declare_parameter('capture_timeout_s', 1.50)
        self.declare_parameter('failure_cooldown_s', 0.20)
        self.declare_parameter('camera_height', 0.28683059)
        self.declare_parameter('camera_pitch_deg', 15.0)
        self.declare_parameter('min_focus_range', 0.50)
        self.declare_parameter('max_focus_range', 1.68)
        self.declare_parameter('court_length', 13.40)
        self.declare_parameter('court_width', 6.10)
        self.declare_parameter('jpeg_quality', 95)

        self.output_dir = Path(
            str(self.get_parameter('output_dir').value)
        ).expanduser().resolve()
        self.world_name = str(self.get_parameter('world_name').value)
        self.camera_entity = str(self.get_parameter('camera_entity').value)
        self.target_images = max(1, int(self.get_parameter('target_images').value))
        self.shuttle_count = max(1, int(self.get_parameter('shuttle_count').value))
        self.spawn_workers = max(1, int(self.get_parameter('spawn_workers').value))
        self.rng = random.Random(int(self.get_parameter('random_seed').value))
        self.settle_time_s = max(0.0, float(self.get_parameter('settle_time_s').value))
        self.camera_wait_s = max(0.0, float(self.get_parameter('camera_wait_s').value))
        self.capture_timeout_s = max(
            0.2, float(self.get_parameter('capture_timeout_s').value)
        )
        self.failure_cooldown_s = max(
            0.0, float(self.get_parameter('failure_cooldown_s').value)
        )
        self.camera_height = float(self.get_parameter('camera_height').value)
        self.camera_pitch = math.radians(
            float(self.get_parameter('camera_pitch_deg').value)
        )
        self.min_focus_range = float(self.get_parameter('min_focus_range').value)
        self.max_focus_range = float(self.get_parameter('max_focus_range').value)
        self.court_length = float(self.get_parameter('court_length').value)
        self.court_width = float(self.get_parameter('court_width').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        debug_share = Path(get_package_share_directory('scrobot_debug'))
        self.shuttle_sdf = (
            debug_share / 'models' / 'yolo_shuttle_dynamic_labeled' / 'model.sdf'
        )

        self._prepare_output()

        self.trigger_pub = self.create_publisher(
            Bool, '/yolo/capture/trigger', 10
        )
        self.create_subscription(
            Image, '/yolo/rgb', self._image_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Detection2DArray,
            '/yolo/bbox',
            self._boxes_cb,
            qos_profile_sensor_data,
        )

        self.rgb_by_stamp = {}
        self.boxes_by_stamp = {}
        self.rgb_wall_by_stamp = {}
        self.boxes_wall_by_stamp = {}

        self.shuttles = []
        self.spawned = False
        self.frozen = False
        self.settle_started = None

        self.saved = 0
        self.attempts = 0
        self.failed = 0
        self.duplicate_skips = 0
        self.accepted_hashes = set()

        self.state = 'INIT'
        self.pending_pose = None
        self.state_started = time.perf_counter()
        self.trigger_wall = None
        self.finished = False

        self.create_timer(0.01, self._step)

    def _prepare_output(self):
        if self.output_dir.exists():
            shutil.rmtree(self.output_dir)

        for split in ('train', 'val', 'test'):
            (self.output_dir / 'images' / split).mkdir(parents=True, exist_ok=True)
            (self.output_dir / 'labels' / split).mkdir(parents=True, exist_ok=True)

        (self.output_dir / 'data.yaml').write_text(
            f'path: {self.output_dir}\n'
            'train: images/train\n'
            'val: images/val\n'
            'test: images/test\n\n'
            'names:\n  0: Shuttlecock\n'
        )

        self.metadata_path = self.output_dir / 'metadata.csv'
        with self.metadata_path.open('w', newline='') as f:
            csv.writer(f).writerow([
                'frame', 'split', 'attempt',
                'camera_x', 'camera_y', 'camera_yaw_rad',
                'stamp_ns', 'boxes', 'image_sha1',
            ])

    @staticmethod
    def _stamp_ns(header):
        return (
            int(header.stamp.sec) * 1_000_000_000
            + int(header.stamp.nanosec)
        )

    def _image_cb(self, msg):
        stamp = self._stamp_ns(msg.header)
        self.rgb_by_stamp[stamp] = msg
        self.rgb_wall_by_stamp[stamp] = time.perf_counter()
        self._trim_buffers()

    def _boxes_cb(self, msg):
        stamp = self._stamp_ns(msg.header)
        self.boxes_by_stamp[stamp] = msg
        self.boxes_wall_by_stamp[stamp] = time.perf_counter()
        self._trim_buffers()

    def _trim_buffers(self):
        keys = sorted(
            set(self.rgb_by_stamp) | set(self.boxes_by_stamp)
        )
        for stamp in keys[:-8]:
            self.rgb_by_stamp.pop(stamp, None)
            self.boxes_by_stamp.pop(stamp, None)
            self.rgb_wall_by_stamp.pop(stamp, None)
            self.boxes_wall_by_stamp.pop(stamp, None)

    def _clear_buffers(self):
        self.rgb_by_stamp.clear()
        self.boxes_by_stamp.clear()
        self.rgb_wall_by_stamp.clear()
        self.boxes_wall_by_stamp.clear()

    def _set_state(self, state):
        if state == self.state:
            return
        self.state = state
        self.state_started = time.perf_counter()
        self.get_logger().info(
            f'DATASET {state} | saved={self.saved}/{self.target_images} '
            f'attempts={self.attempts} failed={self.failed}'
        )

    @staticmethod
    def _run(cmd, timeout=None):
        try:
            return subprocess.run(
                cmd,
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired as exc:
            return subprocess.CompletedProcess(
                cmd, 124, stdout=(exc.stdout or '') + '\nTIMEOUT'
            )

    def _spawn_scene(self):
        self._set_state('SPAWNING_SHUTTLES')
        half_l = self.court_length * 0.5 - 0.15
        half_w = self.court_width * 0.5 - 0.15

        specs = []
        for i in range(self.shuttle_count):
            x = self.rng.uniform(-half_l, half_l)
            y = self.rng.uniform(-half_w, half_w)
            yaw = self.rng.uniform(-math.pi, math.pi)
            specs.append((i, x, y, yaw))

        def spawn_one(spec):
            i, x, y, yaw = spec
            return spec, self._run([
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', self.world_name,
                '-name', f'yolo_shuttle_{i:03d}',
                '-file', str(self.shuttle_sdf),
                '-x', f'{x:.9f}',
                '-y', f'{y:.9f}',
                '-z', '0.050',
                '-R', '0',
                '-P', f'{math.pi / 2.0:.9f}',
                '-Y', f'{yaw:.9f}',
            ], timeout=10.0)

        errors = []
        workers = min(self.spawn_workers, len(specs))
        with ThreadPoolExecutor(max_workers=workers) as pool:
            futures = [pool.submit(spawn_one, spec) for spec in specs]
            for future in as_completed(futures):
                spec, result = future.result()
                if result.returncode != 0:
                    errors.append((spec[0], result.stdout.strip()))

        if errors:
            raise RuntimeError(
                f'{len(errors)} shuttle spawn(s) failed; first={errors[0]}'
            )

        self.shuttles = [(x, y) for _, x, y, _ in specs]
        self.spawned = True
        self.settle_started = time.perf_counter()
        self._set_state('SETTLING_SHUTTLES')

    def _freeze_shuttles(self):
        result = self._run([
            'gz', 'service',
            '-s', '/yolo/freeze_shuttles',
            '--reqtype', 'gz.msgs.Boolean',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000',
            '--req', 'data: true',
        ], timeout=3.0)

        if result.returncode != 0 or 'data: true' not in result.stdout.lower():
            raise RuntimeError(
                'freeze service failed: ' + result.stdout.strip()
            )

        self.frozen = True
        self._clear_buffers()
        self._set_state('SHUTTLES_FROZEN')

    def _sample_camera_pose(self):
        sx, sy = self.rng.choice(self.shuttles)
        desired = self.rng.uniform(
            self.min_focus_range, self.max_focus_range
        )
        dz = self.camera_height - 0.050
        horizontal = math.sqrt(
            max(0.05 * 0.05, desired * desired - dz * dz)
        )
        bearing = self.rng.uniform(-math.pi, math.pi)

        x = sx - horizontal * math.cos(bearing)
        y = sy - horizontal * math.sin(bearing)

        half_l = self.court_length * 0.5 - 0.45
        half_w = self.court_width * 0.5 - 0.35
        x = max(-half_l, min(half_l, x))
        y = max(-half_w, min(half_w, y))

        yaw = bearing + self.rng.uniform(
            math.radians(-25.0), math.radians(25.0)
        )
        return x, y, yaw

    @staticmethod
    def _quat_from_rpy(roll, pitch, yaw):
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _move_camera(self):
        self.attempts += 1
        x, y, yaw = self._sample_camera_pose()
        self.pending_pose = (x, y, yaw)
        q = self._quat_from_rpy(0.0, self.camera_pitch, yaw)

        request = (
            f'name: "{self.camera_entity}", '
            f'position: {{x: {x:.9f}, y: {y:.9f}, z: {self.camera_height:.9f}}}, '
            f'orientation: {{x: {q[0]:.12f}, y: {q[1]:.12f}, '
            f'z: {q[2]:.12f}, w: {q[3]:.12f}}}'
        )

        result = self._run([
            'gz', 'service',
            '-s', f'/world/{self.world_name}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', request,
        ], timeout=2.0)

        if result.returncode != 0 or 'data: true' not in result.stdout.lower():
            self.failed += 1
            self.get_logger().warning(
                f'camera pose failed; skip attempt {self.attempts}'
            )
            self.pending_pose = None
            self._set_state('COOLDOWN')
            return

        self._clear_buffers()
        self._set_state('CAMERA_MOVED')

    def _trigger(self):
        msg = Bool()
        msg.data = True
        self.trigger_wall = time.perf_counter()
        self.trigger_pub.publish(msg)
        self._set_state('WAITING_FOR_PAIR')

    def _find_pair(self):
        if self.trigger_wall is None:
            return None

        common = sorted(
            set(self.rgb_by_stamp).intersection(self.boxes_by_stamp),
            reverse=True,
        )
        for stamp in common:
            if self.rgb_wall_by_stamp.get(stamp, 0.0) < self.trigger_wall:
                continue
            if self.boxes_wall_by_stamp.get(stamp, 0.0) < self.trigger_wall:
                continue
            return stamp, self.rgb_by_stamp[stamp], self.boxes_by_stamp[stamp]
        return None

    @staticmethod
    def _image_to_bgr(msg):
        if msg.encoding not in ('rgb8', 'bgr8'):
            raise RuntimeError(f'unsupported image encoding {msg.encoding}')

        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.step
        )
        img = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img.copy()

    @staticmethod
    def _split_for_index(idx):
        cycle = (
            'train', 'train', 'val', 'train', 'test',
            'train', 'train', 'train', 'val', 'train',
            'test', 'train', 'train', 'train', 'val',
            'train', 'test', 'train', 'train', 'train',
        )
        return cycle[idx % len(cycle)]

    def _save_pair(self, stamp, image_msg, detections_msg):
        bgr = self._image_to_bgr(image_msg)
        image_hash = hashlib.sha1(bgr.tobytes()).hexdigest()

        if image_hash in self.accepted_hashes:
            self.duplicate_skips += 1
            self.failed += 1
            self.get_logger().warning(
                f'duplicate render; skip attempt {self.attempts}'
            )
            return False

        rows = []
        for detection in detections_msg.detections:
            if not detection.results:
                continue
            if detection.results[0].hypothesis.class_id != '1':
                continue

            xc = float(detection.bbox.center.position.x) / float(image_msg.width)
            yc = float(detection.bbox.center.position.y) / float(image_msg.height)
            w = float(detection.bbox.size_x) / float(image_msg.width)
            h = float(detection.bbox.size_y) / float(image_msg.height)

            if w <= 0.0 or h <= 0.0:
                continue
            rows.append((xc, yc, w, h))

        split = self._split_for_index(self.saved)
        stem = f'simple_{self.saved:07d}'
        image_path = self.output_dir / 'images' / split / f'{stem}.jpg'
        label_path = self.output_dir / 'labels' / split / f'{stem}.txt'

        ok = cv2.imwrite(
            str(image_path),
            bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            raise RuntimeError(f'failed to write {image_path}')

        with label_path.open('w') as f:
            for xc, yc, w, h in rows:
                f.write(f'0 {xc:.8f} {yc:.8f} {w:.8f} {h:.8f}\n')

        x, y, yaw = self.pending_pose
        with self.metadata_path.open('a', newline='') as f:
            csv.writer(f).writerow([
                stem, split, self.attempts,
                f'{x:.9f}', f'{y:.9f}', f'{yaw:.9f}',
                stamp, len(rows), image_hash,
            ])

        self.accepted_hashes.add(image_hash)
        self.saved += 1

        self.get_logger().info(
            f'FRAME {self.saved}/{self.target_images} '
            f'attempt={self.attempts} boxes={len(rows)} split={split}'
        )
        return True

    def _step(self):
        if self.finished:
            return

        try:
            if not self.spawned:
                self._spawn_scene()
                return

            if not self.frozen:
                if (
                    self.settle_started is not None
                    and time.perf_counter() - self.settle_started
                    >= self.settle_time_s
                ):
                    self._freeze_shuttles()
                return

            if self.saved >= self.target_images:
                self.finished = True
                self._set_state('COMPLETE')
                self.get_logger().info(
                    f'COMPLETE saved={self.saved} attempts={self.attempts} '
                    f'failed={self.failed} duplicates={self.duplicate_skips}'
                )
                rclpy.shutdown()
                return

            if self.state in ('SHUTTLES_FROZEN', 'FRAME_DONE', 'COOLDOWN'):
                if (
                    self.state == 'COOLDOWN'
                    and time.perf_counter() - self.state_started
                    < self.failure_cooldown_s
                ):
                    return
                self._clear_buffers()
                self._move_camera()
                return

            if self.state == 'CAMERA_MOVED':
                if time.perf_counter() - self.state_started < self.camera_wait_s:
                    return
                self._trigger()
                return

            if self.state == 'WAITING_FOR_PAIR':
                pair = self._find_pair()
                if pair is not None:
                    stamp, image_msg, boxes_msg = pair
                    self._save_pair(stamp, image_msg, boxes_msg)
                    self.pending_pose = None
                    self.trigger_wall = None
                    self._clear_buffers()
                    self._set_state('FRAME_DONE')
                    return

                if (
                    time.perf_counter() - self.state_started
                    >= self.capture_timeout_s
                ):
                    self.failed += 1
                    self.get_logger().warning(
                        f'render timeout; skip attempt {self.attempts}'
                    )
                    self.pending_pose = None
                    self.trigger_wall = None
                    self._clear_buffers()
                    self._set_state('COOLDOWN')
                    return

        except Exception as exc:
            self.get_logger().error(f'Fatal dataset error: {exc}')
            self.finished = True
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleYoloDatasetCapture()
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
