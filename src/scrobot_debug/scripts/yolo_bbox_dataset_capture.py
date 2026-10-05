#!/usr/bin/env python3

import csv
import json
import math
import os
import random
import shutil
import subprocess
import time
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
        self.camera_pitch = math.radians(float(self.get_parameter('camera_pitch_deg').value))
        self.min_focus_range = float(self.get_parameter('min_focus_range').value)
        self.max_focus_range = float(self.get_parameter('max_focus_range').value)
        self.court_length = float(self.get_parameter('court_length').value)
        self.court_width = float(self.get_parameter('court_width').value)

        debug_share = Path(get_package_share_directory('scrobot_debug'))
        self.shuttle_sdf = debug_share / 'models' / 'yolo_shuttle_visual' / 'model.sdf'

        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10
        )

        self.shuttles = []
        self.spawned = False
        self.saved = 0
        self.attempts = 0
        self.state = 'INIT'
        self.pending_raw_index = None
        self.pending_pose = None
        self.pending_started_wall = None
        self.finished = False

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

        for i in range(self.shuttle_count):
            x = self.rng.uniform(-half_l, half_l)
            y = self.rng.uniform(-half_w, half_w)
            yaw = self.rng.uniform(-math.pi, math.pi)
            cmd = [
                'ros2','run','ros_gz_sim','create',
                '-world',self.world_name,
                '-name',f'yolo_shuttle_{i:03d}',
                '-file',str(self.shuttle_sdf),
                '-x',f'{x:.9f}','-y',f'{y:.9f}','-z','0.050',
                '-R','0','-P',f'{math.pi/2:.9f}','-Y',f'{yaw:.9f}'
            ]
            r = self._run(cmd)
            if r.returncode != 0:
                raise RuntimeError(r.stdout)
            self.shuttles.append((x,y))

        self.spawned = True
        self._set_state('SCENE_READY')

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

    def _move_and_trigger(self):
        self.attempts += 1
        x,y,yaw = self._sample_camera_pose()
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
            '--timeout','2000',
            '--req',req
        ])
        if r.returncode != 0 or 'data: true' not in r.stdout.lower():
            self.get_logger().warning(f'set_pose failed: {r.stdout.strip()}')
            return

        before = set((self.raw_dir / 'images').glob('image_*.png'))
        r = self._run([
            'gz','topic','-t',self.trigger_topic,
            '-m','gz.msgs.Boolean','-p','data: true'
        ])
        if r.returncode != 0:
            self.get_logger().warning(f'trigger failed: {r.stdout.strip()}')
            return

        self.pending_raw_index = len(before)
        self.pending_pose = (x,y,yaw)
        self.pending_started_wall = time.perf_counter()
        self._set_state('WAITING_FOR_NATIVE_SAMPLE')

    def _raw_pair_paths(self, idx):
        stem = f'{idx:07d}'
        return (
            self.raw_dir / 'images' / f'image_{stem}.png',
            self.raw_dir / 'boxes' / f'boxes_{stem}.csv',
        )

    def _split_for_index(self, idx):
        v = idx % 20
        if v < 14:
            return 'train'
        if v < 17:
            return 'val'
        return 'test'

    def _convert_pending(self):
        img, box = self._raw_pair_paths(self.pending_raw_index)
        if not img.exists() or not box.exists():
            return False

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

        self.saved += 1
        self.get_logger().info(
            f'FRAME {self.saved}/{self.target_images} split={split} boxes={len(rows)} '
            f'raw={img.name}'
        )
        self.pending_raw_index = None
        self.pending_pose = None
        self.pending_started_wall = None
        self._set_state('FRAME_SAVED')
        return True

    def step(self):
        if self.finished:
            return
        try:
            if not self.spawned:
                self._spawn_scene()
                return

            if self.pending_raw_index is not None:
                if self._convert_pending():
                    return
                if (
                    self.pending_started_wall is not None
                    and time.perf_counter() - self.pending_started_wall > 5.0
                ):
                    self.get_logger().warning(
                        f'Native sample {self.pending_raw_index} not written '
                        'within 5.0 s; retrying a new viewpoint.'
                    )
                    self.pending_raw_index = None
                    self.pending_pose = None
                    self.pending_started_wall = None
                    self._set_state('NATIVE_SAMPLE_TIMEOUT')
                return

            if self.saved >= self.target_images:
                self.finished = True
                self._set_state('COMPLETE')
                self.get_logger().info(f'Native bbox dataset complete: {self.output_dir}')
                rclpy.shutdown()
                return

            self._move_and_trigger()
        except Exception as exc:
            self.get_logger().error(str(exc))
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
