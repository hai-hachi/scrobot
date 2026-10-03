#!/usr/bin/env python3

import math
import struct
import time

import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def rotate_vector(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    return (
        (1.0 - 2.0 * (y * y + z * z)) * vx
        + 2.0 * (x * y - z * w) * vy
        + 2.0 * (x * z + y * w) * vz,
        2.0 * (x * y + z * w) * vx
        + (1.0 - 2.0 * (x * x + z * z)) * vy
        + 2.0 * (y * z - x * w) * vz,
        2.0 * (x * z - y * w) * vx
        + 2.0 * (y * z + x * w) * vy
        + (1.0 - 2.0 * (x * x + y * y)) * vz,
    )


def transform_point(transform, p):
    q = transform.rotation
    rotated = rotate_vector(q, p)
    t = transform.translation
    return (
        rotated[0] + t.x,
        rotated[1] + t.y,
        rotated[2] + t.z,
    )


class CameraFrameRangeMonitor(Node):
    """Validate camera TF, intrinsics, stream rate, FOV and shuttle range."""

    def __init__(self):
        super().__init__('camera_frame_range_monitor')

        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter(
            'color_frame', 'camera_color_optical_frame'
        )
        self.declare_parameter(
            'depth_frame', 'camera_depth_optical_frame'
        )
        self.declare_parameter(
            'event_topic', '/debug/camera_test'
        )
        self.declare_parameter('center_offset_z', 0.045)
        self.declare_parameter('report_rate', 1.0)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.color_frame = str(self.get_parameter('color_frame').value)
        self.depth_frame = str(self.get_parameter('depth_frame').value)
        self.event_topic = str(self.get_parameter('event_topic').value)
        self.center_offset_z = float(
            self.get_parameter('center_offset_z').value
        )
        self.report_rate = max(
            0.2, float(self.get_parameter('report_rate').value)
        )

        self.publisher = self.create_publisher(
            String, self.event_topic, 50
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False
        )

        self.color_info = None
        self.depth_info = None
        self.color_image = None
        self.depth_image = None
        self.robot_pose = None
        self.shuttles = []

        self.color_count = 0
        self.depth_count = 0
        self.last_rate_wall = time.monotonic()
        self.last_color_count = 0
        self.last_depth_count = 0
        self.color_rate = 0.0
        self.depth_rate = 0.0

        self.start_wall = time.monotonic()
        self.tf_reported = False
        self.color_info_reported = False
        self.depth_info_reported = False
        self.missing_warned = set()

        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self._color_info_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            '/camera/camera/depth/camera_info',
            self._depth_info_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self._color_image_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            '/camera/camera/depth/image_raw',
            self._depth_image_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            '/evaluation/ground_truth_odom',
            self._robot_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseArray,
            '/evaluation/shuttle_ground_truth',
            self._shuttle_cb,
            qos_profile_sensor_data,
        )

        self.create_timer(
            1.0 / self.report_rate, self._report
        )

        self._emit(
            'READY waiting for camera TF, camera_info, images and '
            'optional shuttle ground truth'
        )

    def _emit(self, text):
        msg = String()
        msg.data = str(text)
        self.publisher.publish(msg)

    def _color_info_cb(self, msg):
        self.color_info = msg

    def _depth_info_cb(self, msg):
        self.depth_info = msg

    def _color_image_cb(self, msg):
        self.color_image = msg
        self.color_count += 1

    def _depth_image_cb(self, msg):
        self.depth_image = msg
        self.depth_count += 1

    def _robot_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_pose = (
            float(p.x),
            float(p.y),
            float(p.z),
            yaw_from_quaternion(q),
        )

    def _shuttle_cb(self, msg):
        self.shuttles = list(msg.poses)

    def _update_rates(self):
        now = time.monotonic()
        dt = now - self.last_rate_wall
        if dt < 0.8:
            return

        self.color_rate = (
            self.color_count - self.last_color_count
        ) / dt
        self.depth_rate = (
            self.depth_count - self.last_depth_count
        ) / dt

        self.last_color_count = self.color_count
        self.last_depth_count = self.depth_count
        self.last_rate_wall = now

    @staticmethod
    def _info_text(label, info):
        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])

        hfov = math.degrees(
            2.0 * math.atan2(float(info.width), 2.0 * fx)
        ) if fx > 0.0 else float('nan')
        vfov = math.degrees(
            2.0 * math.atan2(float(info.height), 2.0 * fy)
        ) if fy > 0.0 else float('nan')

        return (
            f'{label}_INFO frame={info.header.frame_id} '
            f'size={info.width}x{info.height} '
            f'fx={fx:.4f} fy={fy:.4f} '
            f'cx={cx:.4f} cy={cy:.4f} '
            f'HFOV={hfov:.3f}deg VFOV={vfov:.3f}deg'
        )

    def _report_tf(self):
        if self.tf_reported:
            return

        try:
            color_tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.color_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            depth_tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.depth_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            return

        for label, tf in (
            ('COLOR_TF', color_tf),
            ('DEPTH_TF', depth_tf),
        ):
            t = tf.transform.translation
            q = tf.transform.rotation
            forward = rotate_vector(q, (0.0, 0.0, 1.0))
            down_deg = math.degrees(
                math.atan2(
                    -forward[2],
                    math.hypot(forward[0], forward[1]),
                )
            )
            yaw_deg = math.degrees(
                math.atan2(forward[1], forward[0])
            )
            self._emit(
                f'{label} {self.base_frame}->{tf.child_frame_id} '
                f'xyz=({t.x:+.5f},{t.y:+.5f},{t.z:+.5f})m '
                f'optical_forward_base='
                f'({forward[0]:+.5f},{forward[1]:+.5f},'
                f'{forward[2]:+.5f}) '
                f'down={down_deg:+.3f}deg yaw={yaw_deg:+.3f}deg'
            )

        self.tf_reported = True

    def _warn_missing(self):
        if time.monotonic() - self.start_wall < 6.0:
            return

        checks = (
            ('color_camera_info', self.color_info is not None),
            ('depth_camera_info', self.depth_info is not None),
            ('color_image', self.color_image is not None),
            ('depth_image', self.depth_image is not None),
        )
        for name, present in checks:
            if not present and name not in self.missing_warned:
                self._emit(f'WARN missing={name}')
                self.missing_warned.add(name)

    def _shuttle_center_world(self, pose):
        offset = rotate_vector(
            pose.orientation,
            (0.0, 0.0, self.center_offset_z),
        )
        return (
            float(pose.position.x) + offset[0],
            float(pose.position.y) + offset[1],
            float(pose.position.z) + offset[2],
        )

    def _world_to_base(self, p):
        rx, ry, rz, yaw = self.robot_pose
        dx = p[0] - rx
        dy = p[1] - ry
        c = math.cos(yaw)
        s = math.sin(yaw)
        return (
            c * dx + s * dy,
            -s * dx + c * dy,
            p[2] - rz,
        )

    def _base_to_optical(self, p, frame):
        tf = self.tf_buffer.lookup_transform(
            frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=0.02),
        )
        return transform_point(tf.transform, p)

    @staticmethod
    def _project(p, info):
        x, y, z = p
        if z <= 0.0:
            return None

        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])

        u = fx * x / z + cx
        v = fy * y / z + cy
        inside = (
            0.0 <= u < float(info.width)
            and 0.0 <= v < float(info.height)
        )
        return u, v, z, inside

    def _sample_depth(self, u, v, radius=4):
        image = self.depth_image
        if image is None:
            return None

        if image.encoding not in ('32FC1', '32FC'):
            return None

        values = []
        ui = int(round(u))
        vi = int(round(v))

        x0 = max(0, ui - radius)
        x1 = min(int(image.width) - 1, ui + radius)
        y0 = max(0, vi - radius)
        y1 = min(int(image.height) - 1, vi + radius)

        byte_order = '>' if image.is_bigendian else '<'

        for yy in range(y0, y1 + 1):
            row = yy * int(image.step)
            for xx in range(x0, x1 + 1):
                offset = row + xx * 4
                if offset + 4 > len(image.data):
                    continue
                value = struct.unpack_from(
                    byte_order + 'f', image.data, offset
                )[0]
                if math.isfinite(value) and value > 0.0:
                    values.append(float(value))

        if not values:
            return None

        values.sort()
        return values[len(values) // 2]

    def _report_target(self):
        if (
            self.robot_pose is None
            or not self.shuttles
            or self.color_info is None
            or self.depth_info is None
        ):
            return

        pose = self.shuttles[0]
        center_world = self._shuttle_center_world(pose)
        center_base = self._world_to_base(center_world)

        try:
            color_point = self._base_to_optical(
                center_base, self.color_frame
            )
            depth_point = self._base_to_optical(
                center_base, self.depth_frame
            )
        except Exception:
            return

        color_proj = self._project(
            color_point, self.color_info
        )
        depth_proj = self._project(
            depth_point, self.depth_info
        )

        color_text = 'color=BEHIND'
        if color_proj is not None:
            cu, cv, cz, cin = color_proj
            color_text = (
                f'color_z={cz:.3f}m '
                f'uv=({cu:.1f},{cv:.1f}) '
                f'in={"YES" if cin else "NO"}'
            )

        depth_text = 'depth=BEHIND'
        if depth_proj is not None:
            du, dv, dz, din = depth_proj
            sample = (
                self._sample_depth(du, dv)
                if din else None
            )
            sample_text = (
                '?'
                if sample is None
                else f'{sample:.3f}m'
            )
            error_text = ''
            if sample is not None:
                error_text = f' err={sample - dz:+.3f}m'
            depth_text = (
                f'depth_z={dz:.3f}m '
                f'uv=({du:.1f},{dv:.1f}) '
                f'in={"YES" if din else "NO"} '
                f'sample={sample_text}{error_text}'
            )

        self._emit(
            f'TARGET center_base='
            f'({center_base[0]:+.3f},'
            f'{center_base[1]:+.3f},'
            f'{center_base[2]:+.3f})m '
            f'{color_text} | {depth_text}'
        )

    def _report(self):
        self._update_rates()
        self._report_tf()
        self._warn_missing()

        if (
            self.color_info is not None
            and not self.color_info_reported
        ):
            self._emit(
                self._info_text('COLOR', self.color_info)
            )
            self.color_info_reported = True

        if (
            self.depth_info is not None
            and not self.depth_info_reported
        ):
            self._emit(
                self._info_text('DEPTH', self.depth_info)
            )
            self.depth_info_reported = True

        color_frame = (
            '?'
            if self.color_image is None
            else self.color_image.header.frame_id
        )
        depth_frame = (
            '?'
            if self.depth_image is None
            else self.depth_image.header.frame_id
        )
        self._emit(
            f'STREAM color={self.color_rate:.2f}Hz '
            f'frame={color_frame} '
            f'depth={self.depth_rate:.2f}Hz '
            f'frame={depth_frame}'
        )

        self._report_target()


def main(args=None):
    rclpy.init(args=args)
    node = CameraFrameRangeMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
