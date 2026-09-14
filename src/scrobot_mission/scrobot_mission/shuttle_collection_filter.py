#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray


def detection_position(detection):
    if detection.results:
        p = detection.results[0].pose.pose.position
    else:
        p = detection.bbox.center.position
    return float(p.x), float(p.y), float(p.z)


class ShuttleCollectionFilter(Node):
    """Publish only shuttles that are eligible for local collection.

    Eligibility is intentionally mission-specific and separate from perception:
      * target must be within max_target_range of the robot;
      * target must stay outside the exclusion radius around either net pole.

    The output detections keep their original frame and measurement so the
    local-collect controller can still freeze the raw 3D point in odom.
    """

    def __init__(self):
        super().__init__('shuttle_collection_filter')

        self.declare_parameter(
            'input_topic', '/perception/shuttle_detections_3d'
        )
        self.declare_parameter(
            'output_topic', '/perception/collectable_shuttle_detections_3d'
        )
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('tf_timeout', 0.05)
        self.declare_parameter('max_target_range', 2.0)
        self.declare_parameter('pole_x', 0.0)
        self.declare_parameter('pole_y_positions', [3.05, -3.05])
        self.declare_parameter('pole_exclusion_radius', 0.60)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.max_target_range = float(
            self.get_parameter('max_target_range').value
        )
        self.pole_x = float(self.get_parameter('pole_x').value)
        self.pole_y_positions = [
            float(v) for v in self.get_parameter('pole_y_positions').value
        ]
        self.pole_exclusion_radius = float(
            self.get_parameter('pole_exclusion_radius').value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(
            Detection3DArray,
            self.output_topic,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Detection3DArray,
            self.input_topic,
            self._detections_cb,
            qos_profile_sensor_data,
        )

        self.last_summary = None
        self.get_logger().info(
            'Shuttle collection filter started: '
            f'range <= {self.max_target_range:.2f} m, '
            f'pole exclusion = {self.pole_exclusion_radius:.2f} m.'
        )

    def _lookup(self, target, source):
        try:
            return self.tf_buffer.lookup_transform(
                target,
                source,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException:
            return None

    @staticmethod
    def _transform_point(transform, source_frame, xyz):
        point = PointStamped()
        point.header.frame_id = source_frame
        point.point.x = float(xyz[0])
        point.point.y = float(xyz[1])
        point.point.z = float(xyz[2])
        out = do_transform_point(point, transform)
        return float(out.point.x), float(out.point.y), float(out.point.z)

    def _near_pole(self, point_map):
        x, y, _ = point_map
        for pole_y in self.pole_y_positions:
            if math.hypot(x - self.pole_x, y - pole_y) <= self.pole_exclusion_radius:
                return True
        return False

    def _detections_cb(self, msg):
        output = Detection3DArray()
        output.header = msg.header

        source_frame = msg.header.frame_id.strip()
        if not source_frame and msg.detections:
            source_frame = msg.detections[0].header.frame_id.strip()

        if not source_frame:
            self.publisher.publish(output)
            return

        if source_frame == self.base_frame:
            base_tf = None
        else:
            base_tf = self._lookup(self.base_frame, source_frame)

        if source_frame == self.map_frame:
            map_tf = None
        else:
            map_tf = self._lookup(self.map_frame, source_frame)

        far_count = 0
        pole_count = 0
        tf_count = 0

        if base_tf is None and source_frame != self.base_frame:
            tf_count = len(msg.detections)
        elif map_tf is None and source_frame != self.map_frame:
            tf_count = len(msg.detections)
        else:
            for detection in msg.detections:
                point = detection_position(detection)

                if base_tf is None:
                    point_base = point
                else:
                    point_base = self._transform_point(base_tf, source_frame, point)

                robot_range = math.hypot(point_base[0], point_base[1])
                if robot_range > self.max_target_range:
                    far_count += 1
                    continue

                if map_tf is None:
                    point_map = point
                else:
                    point_map = self._transform_point(map_tf, source_frame, point)

                if self._near_pole(point_map):
                    pole_count += 1
                    continue

                output.detections.append(detection)

        self.publisher.publish(output)

        summary = (
            len(msg.detections),
            len(output.detections),
            far_count,
            pole_count,
            tf_count,
        )
        if summary != self.last_summary:
            self.last_summary = summary
            self.get_logger().info(
                'Collection candidates: '
                f'raw={summary[0]}, eligible={summary[1]}, '
                f'far={summary[2]}, near_pole={summary[3]}, tf_drop={summary[4]}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ShuttleCollectionFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
