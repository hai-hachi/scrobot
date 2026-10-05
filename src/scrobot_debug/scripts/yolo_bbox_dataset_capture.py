#!/usr/bin/env python3

import csv
import hashlib
import json
import math
import os
import random
import shutil
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


class NativeBBoxDatasetCapture(Node):
    def __init__(self):
        super().__init__('yolo_bbox_dataset_capture')

        self.declare_parameter(
            'output_dir',
            str(Path.home() / 'Desktop' / 'yoloshuttle' / 'dataset' / 'gazebo_scrobot_native'),
        )
        self.declare_parameter('raw_dir', '/tmp/scrobot_yolo_bbox_raw')
        self.declare_parameter('world_name', 'yolo_dataset_native')
        self.declare_parameter('camera_entity', 'yolo_bbox_camera_rig')
        self.declare_parameter('trigger_topic', '/yolo/bbox/trigger')
        self.declare_parameter('target_images', 20)
        self.declare_parameter('shuttle_count', 40)
        self.declare_parameter('random_seed', 42)
        self.declare_parameter('camera_height', 0.28683059)
        self.declare_parameter('settle_time_s', 2.0)
        self.declare_parameter('camera_apply_wait_s', 0.10)
        self.declare_parameter('spawn_workers', 8)
        self.declare_parameter('sample_timeout_s', 5.0)
        self.declare_parameter('max_camera_sync_retries', 2)
        self.declare_parameter('pose_retry_wait_s', 0.50)
        self.declare_parameter('camera_pitch_deg', 15.0)
        self.declare_parameter('min_focus_range', 0.50)
        self.declare_parameter('max_focus_range', 1.68)
        self.declare_parameter('court_length', 13.40)
        self.declare_parameter('court_width', 6.10)
        self.declare_parameter('status_topic', '/debug/yolo_dataset_status')

        self.output_dir = Path(str(self.get_parameter('output_dir').value)).expanduser().resolve()
        self.raw_dir = Path(str(self.get_parameter('raw_dir').value))
        self.world_name = str(self.get_parameter('world_name').value)
        self.camera_entity = str(self.get_parameter('camera_entity').value)
        self.trigger_topic = str(self.get_parameter('trigger_topic').value)
        self.target_images = max(1, int(self.get_parameter('target_images').value))
        self.shuttle_count = max(1, int(self.get_parameter('shuttle_count').value))
        self.rng = random.Random(int(self.get_parameter('random_seed').value))
        self.camera_height = float(self.get_parameter('camera_height').value)
        self.settle_time_s = max(0.0, float(self.get_parameter('settle_time_s').value))
        self.camera_apply_wait_s = max(0.0, float(self.get_parameter('camera_apply_wait_s').value))
        self.spawn_workers = max(1, int(self.get_parameter('spawn_workers').value))
        self.sample_timeout_s = max(1.0, float(self.get_parameter('sample_timeout_s').value))
        self.max_camera_sync_retries = max(1, int(self.get_parameter('max_camera_sync_retries').value))
        self.pose_retry_wait_s = max(0.10, float(self.get_parameter('pose_retry_wait_s').value))
        self.camera_pitch = math.radians(float(self.get_parameter('camera_pitch_deg').value))
        self.min_focus_range = float(self.get_parameter('min_focus_range').value)
        self.max_focus_range = float(self.get_parameter('max_focus_range').value)
        self.court_length = float(self.get_parameter('court_length').value)
        self.court_width = float(self.get_parameter('court_width').value)

        debug_share = Path(get_package_share_directory('scrobot_debug'))
        self.shuttle_sdf = (
            debug_share / 'models' / 'yolo_shuttle_dynamic_labeled' / 'model.sdf'
        )

        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10
        )

        self.shuttles = []
        self.spawned = False
        self.saved = 0
        self.attempts = 0
        self.state = 'INIT'
        self.finished = False
        self.settle_started_wall = None
        self.scene_frozen = False

        # Sequential camera / triggered-render state machine.
        self.capture_phase = 'idle'
        self.pending_pose = None
        self.phase_started_wall = None
        self.capture_raw_index = None
        self.next_raw_index = 0
        self.accepted_hashes = set()
        self.last_accepted_hash = None
        self.camera_sync_retries = 0
        self.pose_command_failures = 0

        self._prepare_dirs()
        self.create_timer(0.05, self.step)

    def _prepare_dirs(self):
        # Raw directory is cleared by the launch file before Gazebo starts so
        # BoundingBoxCameraSensor initializes its save counter at exactly zero.
        (self.raw_dir / 'images').mkdir(parents=True, exist_ok=True)
        (self.raw_dir / 'boxes').mkdir(parents=True, exist_ok=True)

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
                'frame', 'split',
                'requested_camera_x', 'requested_camera_y',
                'requested_camera_yaw_rad',
                'raw_index', 'image_sha1', 'boxes',
                'scene_frozen',
            ])

    def _set_state(self, s):
        if s == self.state:
            return
        self.state = s
        msg = String()
        msg.data = f'{s} | {self.saved}/{self.target_images} | attempts={self.attempts}'
        self.status_pub.publish(msg)
        self.get_logger().info(f'DATASET {msg.data}')

    def _run(self, cmd):
        return subprocess.run(
            cmd, check=False, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True
        )

    def _spawn_scene(self):
        self._set_state('SPAWNING_SHUTTLES')
        half_l = self.court_length * 0.5 - 0.15
        half_w = self.court_width * 0.5 - 0.15

        specs = []
        for i in range(self.shuttle_count):
            x = self.rng.uniform(-half_l, half_l)
            y = self.rng.uniform(-half_w, half_w)

            # Match production simulation convention exactly.
            roll = 0.0
            pitch = math.pi / 2.0
            yaw = self.rng.uniform(-math.pi, math.pi)
            specs.append((i, x, y, roll, pitch, yaw))

        def spawn_one(spec):
            i, x, y, roll, pitch, yaw = spec
            cmd = [
                'ros2','run','ros_gz_sim','create',
                '-world',self.world_name,
                '-name',f'yolo_shuttle_{i:03d}',
                '-file',str(self.shuttle_sdf),
                '-x',f'{x:.9f}','-y',f'{y:.9f}','-z','0.050',
                '-R',f'{roll:.9f}','-P',f'{pitch:.9f}','-Y',f'{yaw:.9f}'
            ]
            result = self._run(cmd)
            return spec, result

        errors = []
        workers = min(self.spawn_workers, self.shuttle_count)
        with ThreadPoolExecutor(max_workers=workers) as pool:
            futures = [pool.submit(spawn_one, spec) for spec in specs]
            for future in as_completed(futures):
                spec, result = future.result()
                if result.returncode != 0:
                    errors.append((spec[0], result.stdout.strip()))

        if errors:
            detail = '; '.join(
                f'{idx}: {msg}' for idx, msg in errors[:5]
            )
            raise RuntimeError(
                f'{len(errors)} shuttle spawn(s) failed: {detail}'
            )

        # Keep target sampling deterministic, independent of worker completion.
        self.shuttles = [(x, y) for _, x, y, _, _, _ in specs]

        self.spawned = True
        self.settle_started_wall = time.perf_counter()
        self._set_state('SETTLING_SHUTTLES')

    @staticmethod
    def _image_sha1(path):
        h = hashlib.sha1()
        with path.open('rb') as f:
            for chunk in iter(lambda: f.read(1024 * 1024), b''):
                h.update(chunk)
        return h.hexdigest()

    def _trigger_raw_sample(self):
        idx = self.next_raw_index
        r = self._run([
            'gz','topic','-t',self.trigger_topic,
            '-m','gz.msgs.Boolean','-p','data: true'
        ])
        if r.returncode != 0:
            raise RuntimeError(f'camera trigger failed: {r.stdout.strip()}')
        self.next_raw_index += 1
        return idx

    def _raw_pair_ready(self, idx):
        img, box = self._raw_pair_paths(idx)
        return img.exists() and box.exists()

    def _sample_camera_pose(self):
        sx, sy = self.rng.choice(self.shuttles)
        desired = self.rng.uniform(self.min_focus_range, self.max_focus_range)
        dz = self.camera_height - 0.050
        horizontal = math.sqrt(max(0.05*0.05, desired*desired - dz*dz))
        bearing = self.rng.uniform(-math.pi, math.pi)
        x = sx - horizontal * math.cos(bearing)
        y = sy - horizontal * math.sin(bearing)

        half_l = self.court_length * 0.5 - 0.45
        half_w = self.court_width * 0.5 - 0.35
        x = max(-half_l, min(half_l, x))
        y = max(-half_w, min(half_w, y))

        yaw = bearing + self.rng.uniform(math.radians(-25), math.radians(25))
        return x, y, yaw

    def _quat_from_rpy(self, roll, pitch, yaw):
        cr, sr = math.cos(roll/2), math.sin(roll/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        cy, sy = math.cos(yaw/2), math.sin(yaw/2)
        return (
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy,
            cr*cp*cy + sr*sp*sy,
        )

    def _command_next_camera_pose(self, reuse_pose=False):
        if not reuse_pose or self.pending_pose is None:
            self.attempts += 1
            self.pending_pose = self._sample_camera_pose()
            self.camera_sync_retries = 0
            self.pose_command_failures = 0
            self.capture_raw_index = None

        x, y, yaw = self.pending_pose
        q = self._quat_from_rpy(0.0, self.camera_pitch, yaw)

        req = (
            f'name: "{self.camera_entity}", '
            f'position: {{x: {x:.9f}, y: {y:.9f}, z: {self.camera_height:.9f}}}, '
            f'orientation: {{x: {q[0]:.12f}, y: {q[1]:.12f}, '
            f'z: {q[2]:.12f}, w: {q[3]:.12f}}}'
        )
        r = self._run([
            'gz','service',
            '-s',f'/world/{self.world_name}/set_pose',
            '--reqtype','gz.msgs.Pose',
            '--reptype','gz.msgs.Boolean',
            '--timeout','5000',
            '--req',req
        ])

        if r.returncode != 0 or 'data: true' not in r.stdout.lower():
            self.pose_command_failures += 1
            if self.pose_command_failures > self.max_camera_sync_retries:
                self.get_logger().warning(
                    'set_pose kept failing; abandoning this camera pose and '
                    'continuing with a new one.'
                )
                self._reset_camera_capture()
                self._set_state('SET_POSE_SKIP_VIEW')
                return False

            self.get_logger().warning(
                f'set_pose did not confirm ({self.pose_command_failures}/'
                f'{self.max_camera_sync_retries}): {r.stdout.strip() or "timeout"}'
            )
            self.capture_phase = 'wait_pose_retry'
            self.phase_started_wall = time.perf_counter()
            self._set_state('WAITING_TO_RETRY_SET_POSE')
            return False

        self.pose_command_failures = 0

        # Match the old fast generator: the world keeps running, so UserCommands
        # and Ogre naturally consume the new static-camera pose on subsequent
        # iterations. Only a short wall-clock guard is needed before trigger.
        self.capture_phase = 'wait_camera_apply'
        self.phase_started_wall = time.perf_counter()
        self._set_state('WAITING_FOR_CAMERA_APPLY')
        return True

    def _raw_pair_paths(self, idx):
        stem = f'{idx:07d}'
        return (
            self.raw_dir / 'images' / f'image_{stem}.png',
            self.raw_dir / 'boxes' / f'boxes_{stem}.csv',
        )

    def _split_for_index(self, idx):
        # Interleave splits instead of putting the first 14 samples in train.
        # The repeating 20-sample cycle is still 70/15/15, but even short
        # smoke tests contain validation and test frames.
        cycle = (
            'train', 'train', 'val', 'train', 'test',
            'train', 'train', 'train', 'val', 'train',
            'test', 'train', 'train', 'train', 'val',
            'train', 'test', 'train', 'train', 'train',
        )
        return cycle[idx % len(cycle)]

    def _convert_raw_sample(self, raw_index, image_hash):
        img, box = self._raw_pair_paths(raw_index)
        split = self._split_for_index(self.saved)
        stem = f'native_{self.saved:07d}'
        out_img = self.output_dir / 'images' / split / f'{stem}.png'
        out_lbl = self.output_dir / 'labels' / split / f'{stem}.txt'
        shutil.copy2(img, out_img)

        rows = []
        with box.open(newline='') as f:
            for row in csv.DictReader(f):
                if int(row['label']) != 1:
                    continue
                xc = float(row['x_center']) / 1280.0
                yc = float(row['y_center']) / 720.0
                w = float(row['width']) / 1280.0
                h = float(row['height']) / 720.0
                rows.append((xc,yc,w,h))

        with out_lbl.open('w') as f:
            for xc,yc,w,h in rows:
                f.write(f'0 {xc:.8f} {yc:.8f} {w:.8f} {h:.8f}\n')

        x, y, yaw = self.pending_pose
        with self.metadata_path.open('a', newline='') as f:
            csv.writer(f).writerow([
                stem, split,
                f'{x:.9f}', f'{y:.9f}', f'{yaw:.9f}',
                raw_index, image_hash, len(rows), 'settled_auto_disabled',
            ])

        self.saved += 1
        self.accepted_hashes.add(image_hash)
        self.last_accepted_hash = image_hash
        self.get_logger().info(
            f'FRAME {self.saved}/{self.target_images} split={split} '
            f'boxes={len(rows)} raw=image_{raw_index:07d}.png '
            f'hash={image_hash[:10]}'
        )

        self._reset_camera_capture()
        self._set_state('FRAME_SAVED')

    def _reset_camera_capture(self):
        self.pending_pose = None
        self.capture_raw_index = None
        self.camera_sync_retries = 0
        self.pose_command_failures = 0
        self.capture_phase = 'idle'
        self.phase_started_wall = None

    def _retry_render_sync(self, reason):
        self.camera_sync_retries += 1

        if self.camera_sync_retries > self.max_camera_sync_retries:
            self.get_logger().warning(
                f'{reason}; skipping camera pose after '
                f'{self.max_camera_sync_retries} quick retries.'
            )
            self._reset_camera_capture()
            self._set_state('CAMERA_SYNC_SKIP_POSE')
            return

        self.get_logger().warning(
            f'{reason}; renderer is one frame behind, retrying trigger '
            f'({self.camera_sync_retries}/{self.max_camera_sync_retries}).'
        )
        self.capture_phase = 'wait_camera_apply'
        self.phase_started_wall = time.perf_counter()
        self._set_state('WAITING_FOR_CAMERA_APPLY')

    def step(self):
        if self.finished:
            return

        try:
            # Phase 1: initialize the scene once.
            if not self.spawned:
                self._spawn_scene()
                return

            # Dynamic shuttles settle first; no dataset image can be taken yet.
            if self.settle_started_wall is not None:
                elapsed = time.perf_counter() - self.settle_started_wall
                if elapsed < self.settle_time_s:
                    self._set_state('SETTLING_SHUTTLES')
                    return

                self.settle_started_wall = None
                self.scene_frozen = True
                self._set_state('SCENE_SETTLED')
                self.get_logger().info(
                    'Shuttle settling complete. World remains running so camera '
                    'teleports propagate normally; resting shuttle bodies use '
                    'allow_auto_disable=true and stay asleep unless disturbed.'
                )
                return

            if not self.scene_frozen:
                return

            if self.saved >= self.target_images:
                self.finished = True
                self._set_state('COMPLETE')
                self.get_logger().info(
                    f'Native bbox dataset complete: {self.output_dir}'
                )
                rclpy.shutdown()
                return

            # Phase 2: one camera pose at a time.
            if self.capture_phase == 'idle':
                self._command_next_camera_pose()
                return

            if self.capture_phase == 'wait_pose_retry':
                if (
                    time.perf_counter() - self.phase_started_wall
                    < self.pose_retry_wait_s
                ):
                    return
                self._command_next_camera_pose(reuse_pose=True)
                return

            if self.capture_phase == 'wait_camera_apply':
                if (
                    time.perf_counter() - self.phase_started_wall
                    < self.camera_apply_wait_s
                ):
                    return

                # One native bbox render should now correspond to the updated
                # camera pose because paused simulation steps were forced above.
                self.capture_raw_index = self._trigger_raw_sample()
                self.capture_phase = 'wait_capture'
                self.phase_started_wall = time.perf_counter()
                self._set_state('WAITING_FOR_NATIVE_SAMPLE')
                return

            if self.capture_phase == 'wait_capture':
                if not self._raw_pair_ready(self.capture_raw_index):
                    if (
                        time.perf_counter() - self.phase_started_wall
                        > self.sample_timeout_s
                    ):
                        self._retry_render_sync('native sample timed out')
                    return

                capture_img, _ = self._raw_pair_paths(self.capture_raw_index)
                capture_hash = self._image_sha1(capture_img)

                # Exact duplicate means the renderer still has the previous
                # viewpoint. Advance controlled simulation steps and try again.
                if capture_hash in self.accepted_hashes:
                    self._retry_render_sync(
                        'renderer returned a previously accepted camera view'
                    )
                    return

                self._convert_raw_sample(
                    self.capture_raw_index,
                    capture_hash,
                )
                return

        except Exception as exc:
            # A coding / filesystem error is still fatal. Normal camera
            # transport and synchronization failures are handled above.
            self.get_logger().error(f'Fatal dataset error: {exc}')
            self.finished = True
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = NativeBBoxDatasetCapture()
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
