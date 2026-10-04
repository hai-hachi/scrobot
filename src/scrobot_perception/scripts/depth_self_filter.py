#!/usr/bin/env python3

import math
import struct

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
import tf2_ros


class DepthSelfFilter(Node):
    """Remove points that fall inside the robot body before LaserScan collapse."""

    def __init__(self):
        super().__init__('depth_self_filter')

        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('input_topic', '/camera/camera/depth/points')
        self.declare_parameter('output_topic', '/camera/camera/depth/points_filtered')
        self.declare_parameter('transform_tolerance', 0.05)

        # Tight robot-body mask. The current false obstacle is near
        # x=0.316, y=-0.218, which is on the physical front-right body edge.
        self.declare_parameter('self_min_x', -0.45)
        self.declare_parameter('self_max_x', 0.34)
        self.declare_parameter('self_min_y', -0.24)
        self.declare_parameter('self_max_y', 0.24)
        self.declare_parameter('self_min_z', -0.05)
        self.declare_parameter('self_max_z', 0.75)

        self.target_frame = self.get_parameter('target_frame').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.transform_tolerance = float(self.get_parameter('transform_tolerance').value)

        self.bounds = (
            float(self.get_parameter('self_min_x').value),
            float(self.get_parameter('self_max_x').value),
            float(self.get_parameter('self_min_y').value),
            float(self.get_parameter('self_max_y').value),
            float(self.get_parameter('self_min_z').value),
            float(self.get_parameter('self_max_z').value),
        )

        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PointCloud2, output_topic, sensor_qos)
        self.create_subscription(PointCloud2, input_topic, self.cloud_cb, sensor_qos)

        self.last_tf_warn_sec = 0.0
        self.last_stats_sec = 0.0
        self.get_logger().info(
            'Filtering %s -> %s using body box x=[%.3f, %.3f], y=[%.3f, %.3f], z=[%.3f, %.3f] in %s'
            % (input_topic, output_topic, *self.bounds, self.target_frame)
        )

    @staticmethod
    def field_offset(fields, name):
        for field in fields:
            if field.name == name and field.datatype == PointField.FLOAT32:
                return field.offset
        return None

    @staticmethod
    def rotate(q, x, y, z):
        qx, qy, qz, qw = q
        # v' = v + 2*qw*(q.xyz x v) + 2*(q.xyz x (q.xyz x v))
        tx = 2.0 * (qy * z - qz * y)
        ty = 2.0 * (qz * x - qx * z)
        tz = 2.0 * (qx * y - qy * x)
        rx = x + qw * tx + (qy * tz - qz * ty)
        ry = y + qw * ty + (qz * tx - qx * tz)
        rz = z + qw * tz + (qx * ty - qy * tx)
        return rx, ry, rz

    def inside_self(self, x, y, z):
        min_x, max_x, min_y, max_y, min_z, max_z = self.bounds
        return min_x <= x <= max_x and min_y <= y <= max_y and min_z <= z <= max_z

    def passthrough(self, msg, reason):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_tf_warn_sec > 2.0:
            self.get_logger().warn(f'Passing raw depth cloud through: {reason}')
            self.last_tf_warn_sec = now
        self.pub.publish(msg)

    def cloud_cb(self, msg):
        x_offset = self.field_offset(msg.fields, 'x')
        y_offset = self.field_offset(msg.fields, 'y')
        z_offset = self.field_offset(msg.fields, 'z')
        if x_offset is None or y_offset is None or z_offset is None:
            self.passthrough(msg, 'PointCloud2 does not contain float32 x/y/z fields')
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=self.transform_tolerance),
            )
        except Exception as exc:
            self.passthrough(msg, f'TF {msg.header.frame_id} -> {self.target_frame} unavailable: {exc}')
            return

        t = transform.transform.translation
        r = transform.transform.rotation
        q = (r.x, r.y, r.z, r.w)
        translation = (t.x, t.y, t.z)

        endian = '>' if msg.is_bigendian else '<'
        data = memoryview(msg.data)
        kept = bytearray()
        removed = 0
        total = 0

        for offset in range(0, len(data), msg.point_step):
            point = data[offset:offset + msg.point_step]
            if len(point) < msg.point_step:
                continue

            x = struct.unpack_from(endian + 'f', point, x_offset)[0]
            y = struct.unpack_from(endian + 'f', point, y_offset)[0]
            z = struct.unpack_from(endian + 'f', point, z_offset)[0]
            total += 1

            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                bx, by, bz = self.rotate(q, x, y, z)
                bx += translation[0]
                by += translation[1]
                bz += translation[2]
                if self.inside_self(bx, by, bz):
                    removed += 1
                    continue

            kept.extend(point)

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = len(kept) // msg.point_step
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = len(kept)
        out.data = bytes(kept)
        out.is_dense = False
        self.pub.publish(out)

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_stats_sec > 2.0:
            self.get_logger().info(
                f'depth_self_filter removed {removed}/{total} points inside robot body'
            )
            self.last_stats_sec = now


def main(args=None):
    rclpy.init(args=args)
    node = DepthSelfFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
