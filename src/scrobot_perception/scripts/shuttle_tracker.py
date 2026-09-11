#!/usr/bin/env python3

import math
from dataclasses import dataclass

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose


def quat_normalize(q):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    inv = 1.0 / norm
    return (x * inv, y * inv, z * inv, w * inv)


def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q, v):
    qn = quat_normalize(q)
    vx, vy, vz = v
    vq = (vx, vy, vz, 0.0)
    rotated = quat_multiply(quat_multiply(qn, vq), quat_conjugate(qn))
    return (rotated[0], rotated[1], rotated[2])


def transform_point(transform, point):
    t = transform.translation
    q = transform.rotation
    rotated = quat_rotate(
        (float(q.x), float(q.y), float(q.z), float(q.w)),
        point,
    )
    return (
        rotated[0] + float(t.x),
        rotated[1] + float(t.y),
        rotated[2] + float(t.z),
    )


@dataclass
class Track:
    track_id: int
    x: float
    y: float
    z: float
    last_seen_ns: int
    class_id: str = 'shuttle'
    score: float = 1.0
    bbox_x: float = 0.08
    bbox_y: float = 0.08
    bbox_z: float = 0.10
    hit_count: int = 1


class ShuttleTracker(Node):
    """Associate camera-frame shuttle detections into persistent map-frame tracks."""

    def __init__(self):
        super().__init__('shuttle_tracker')

        self.declare_parameter('input_topic', '/perception/shuttle_detections_3d')
        self.declare_parameter('output_topic', '/perception/tracked_shuttles')
        self.declare_parameter('tracking_frame', 'map')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('association_distance', 0.30)
        self.declare_parameter('position_alpha', 0.50)
        self.declare_parameter('stale_timeout', 1.50)
        self.declare_parameter('tf_timeout', 0.05)
        self.declare_parameter('fallback_to_latest_tf', True)
        self.declare_parameter('default_class_id', 'shuttle')

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.tracking_frame = str(self.get_parameter('tracking_frame').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.association_distance = float(self.get_parameter('association_distance').value)
        self.position_alpha = float(self.get_parameter('position_alpha').value)
        self.stale_timeout = float(self.get_parameter('stale_timeout').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.fallback_to_latest_tf = bool(self.get_parameter('fallback_to_latest_tf').value)
        self.default_class_id = str(self.get_parameter('default_class_id').value)

        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate must be > 0')
        if self.association_distance <= 0.0:
            raise ValueError('association_distance must be > 0')
        if not 0.0 < self.position_alpha <= 1.0:
            raise ValueError('position_alpha must be in (0, 1]')
        if self.stale_timeout <= 0.0:
            raise ValueError('stale_timeout must be > 0')
        if self.tf_timeout < 0.0:
            raise ValueError('tf_timeout must be >= 0')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tracks = {}
        self.next_track_id = 1
        self.last_tf_warning_ns = 0
        self.last_tf_fallback_log_ns = 0

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            Detection3DArray,
            self.input_topic,
            self._detections_callback,
            qos_profile_sensor_data,
        )
        self.publisher = self.create_publisher(
            Detection3DArray,
            self.output_topic,
            output_qos,
        )
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_tracks)

        self.get_logger().info(
            'Shuttle tracker started: '
            f'frame={self.tracking_frame}, gate={self.association_distance:.2f} m, '
            f'stale={self.stale_timeout:.2f} s, output={self.output_topic}'
        )

    def _measurement_time(self, msg):
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return Time()
        return Time.from_msg(msg.header.stamp)

    def _warn_tf_throttled(self, source_frame, error):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_tf_warning_ns < int(2.0e9):
            return
        self.last_tf_warning_ns = now_ns
        self.get_logger().warn(
            f'Cannot transform {source_frame} -> {self.tracking_frame}: {error}'
        )

    def _log_tf_fallback_throttled(self, source_frame):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_tf_fallback_log_ns < int(5.0e9):
            return
        self.last_tf_fallback_log_ns = now_ns
        self.get_logger().warn(
            f'Exact-time TF unavailable for {source_frame} -> {self.tracking_frame}; '
            'using latest transform. Check that all simulation nodes use /clock.'
        )

    @staticmethod
    def _extract_measurement(detection):
        if detection.results:
            hypothesis = detection.results[0]
            p = hypothesis.pose.pose.position
            class_id = hypothesis.hypothesis.class_id or 'shuttle'
            score = float(hypothesis.hypothesis.score)
        else:
            p = detection.bbox.center.position
            class_id = 'shuttle'
            score = 1.0

        bbox = detection.bbox.size
        return (
            (float(p.x), float(p.y), float(p.z)),
            class_id,
            score,
            float(bbox.x),
            float(bbox.y),
            float(bbox.z),
        )

    def _lookup_transform(self, source_frame, msg):
        try:
            return self.tf_buffer.lookup_transform(
                self.tracking_frame,
                source_frame,
                self._measurement_time(msg),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException as exact_exc:
            if not self.fallback_to_latest_tf:
                raise exact_exc

            try:
                latest = self.tf_buffer.lookup_transform(
                    self.tracking_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout),
                )
                self._log_tf_fallback_throttled(source_frame)
                return latest.transform
            except TransformException:
                raise exact_exc

    def _transform_measurements(self, msg):
        source_frame = msg.header.frame_id.strip()
        if not source_frame and msg.detections:
            source_frame = msg.detections[0].header.frame_id.strip()
        if not source_frame:
            self._warn_tf_throttled('<empty frame>', 'detection frame is empty')
            return []

        if source_frame == self.tracking_frame:
            transform = None
        else:
            try:
                transform = self._lookup_transform(source_frame, msg)
            except TransformException as exc:
                self._warn_tf_throttled(source_frame, exc)
                return []

        measurements = []
        for detection in msg.detections:
            point, class_id, score, bx, by, bz = self._extract_measurement(detection)
            if transform is not None:
                point = transform_point(transform, point)
            measurements.append((point, class_id, score, bx, by, bz))

        return measurements

    def _detections_callback(self, msg):
        measurements = self._transform_measurements(msg)
        now_ns = self.get_clock().now().nanoseconds

        if not measurements:
            self._remove_stale_tracks(now_ns)
            return

        candidates = []
        for track_id, track in self.tracks.items():
            for measurement_index, measurement in enumerate(measurements):
                point = measurement[0]
                dx = point[0] - track.x
                dy = point[1] - track.y
                dz = point[2] - track.z
                distance = math.sqrt(dx * dx + dy * dy + dz * dz)
                if distance <= self.association_distance:
                    candidates.append((distance, track_id, measurement_index))

        candidates.sort(key=lambda item: item[0])
        assigned_tracks = set()
        assigned_measurements = set()

        for _, track_id, measurement_index in candidates:
            if track_id in assigned_tracks or measurement_index in assigned_measurements:
                continue
            self._update_track(self.tracks[track_id], measurements[measurement_index], now_ns)
            assigned_tracks.add(track_id)
            assigned_measurements.add(measurement_index)

        for measurement_index, measurement in enumerate(measurements):
            if measurement_index not in assigned_measurements:
                self._create_track(measurement, now_ns)

        self._remove_stale_tracks(now_ns)

    def _update_track(self, track, measurement, now_ns):
        point, class_id, score, bx, by, bz = measurement
        alpha = self.position_alpha
        beta = 1.0 - alpha
        track.x = beta * track.x + alpha * point[0]
        track.y = beta * track.y + alpha * point[1]
        track.z = beta * track.z + alpha * point[2]
        track.last_seen_ns = now_ns
        track.class_id = class_id or self.default_class_id
        track.score = score
        track.bbox_x = bx
        track.bbox_y = by
        track.bbox_z = bz
        track.hit_count += 1

    def _create_track(self, measurement, now_ns):
        point, class_id, score, bx, by, bz = measurement
        track_id = self.next_track_id
        self.next_track_id += 1
        self.tracks[track_id] = Track(
            track_id=track_id,
            x=point[0], y=point[1], z=point[2],
            last_seen_ns=now_ns,
            class_id=class_id or self.default_class_id,
            score=score,
            bbox_x=bx, bbox_y=by, bbox_z=bz,
        )
        self.get_logger().info(
            f'Created shuttle track {track_id} at '
            f'({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f})'
        )

    def _remove_stale_tracks(self, now_ns):
        timeout_ns = int(self.stale_timeout * 1.0e9)
        stale_ids = [
            track_id for track_id, track in self.tracks.items()
            if now_ns - track.last_seen_ns > timeout_ns
        ]
        for track_id in stale_ids:
            del self.tracks[track_id]
            self.get_logger().info(f'Removed stale shuttle track {track_id}')

    def _track_to_detection(self, track, stamp):
        detection = Detection3D()
        detection.header.stamp = stamp
        detection.header.frame_id = self.tracking_frame
        detection.id = str(track.track_id)
        detection.bbox.center.position.x = track.x
        detection.bbox.center.position.y = track.y
        detection.bbox.center.position.z = track.z
        detection.bbox.center.orientation.w = 1.0
        detection.bbox.size.x = track.bbox_x
        detection.bbox.size.y = track.bbox_y
        detection.bbox.size.z = track.bbox_z

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = track.class_id
        hypothesis.hypothesis.score = track.score
        hypothesis.pose.pose.position.x = track.x
        hypothesis.pose.pose.position.y = track.y
        hypothesis.pose.pose.position.z = track.z
        hypothesis.pose.pose.orientation.w = 1.0
        detection.results.append(hypothesis)
        return detection

    def _publish_tracks(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        self._remove_stale_tracks(now_ns)

        msg = Detection3DArray()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.tracking_frame
        for track_id in sorted(self.tracks):
            msg.detections.append(self._track_to_detection(self.tracks[track_id], msg.header.stamp))
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ShuttleTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
