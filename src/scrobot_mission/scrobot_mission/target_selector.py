#!/usr/bin/env python3

import copy
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
    cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
    return math.atan2(siny_cosp, cosy_cosp)


def detection_position(detection):
    if detection.results:
        p = detection.results[0].pose.pose.position
    else:
        p = detection.bbox.center.position
    return float(p.x), float(p.y), float(p.z)


class ShuttleTargetSelector(Node):
    """Generate a staging pose for the shuttle ID requested by patrol_manager."""

    def __init__(self):
        super().__init__('shuttle_target_selector')

        self.declare_parameter('tracked_topic', '/perception/tracked_shuttles')
        self.declare_parameter('target_request_topic', '/mission/target_request_id')
        self.declare_parameter('selected_topic', '/mission/selected_shuttle')
        self.declare_parameter('selected_id_topic', '/mission/selected_shuttle_id')
        self.declare_parameter('staging_pose_topic', '/mission/shuttle_staging_pose')
        self.declare_parameter(
            'selection_enabled_topic',
            '/mission/target_selection_enabled',
        )
        self.declare_parameter(
            'collection_complete_topic',
            '/mission/collection_complete',
        )
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('staging_distance', 0.75)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('tf_timeout', 0.05)
        self.declare_parameter('enabled_at_start', True)

        self.tracked_topic = str(self.get_parameter('tracked_topic').value)
        self.target_request_topic = str(
            self.get_parameter('target_request_topic').value
        )
        self.selected_topic = str(self.get_parameter('selected_topic').value)
        self.selected_id_topic = str(self.get_parameter('selected_id_topic').value)
        self.staging_pose_topic = str(self.get_parameter('staging_pose_topic').value)
        self.selection_enabled_topic = str(
            self.get_parameter('selection_enabled_topic').value
        )
        self.collection_complete_topic = str(
            self.get_parameter('collection_complete_topic').value
        )
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.staging_distance = float(self.get_parameter('staging_distance').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.selection_enabled = bool(
            self.get_parameter('enabled_at_start').value
        )

        if self.staging_distance < 0.0:
            raise ValueError('staging_distance must be >= 0')
        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate must be > 0')
        if self.tf_timeout < 0.0:
            raise ValueError('tf_timeout must be >= 0')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_tracks = {}
        self.requested_id = ''
        self.selected_id = ''
        self.selected_detection = None
        self.staging_pose = None
        self.last_tf_warning_ns = 0
        self.completed_ids = set()

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        control_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            Detection3DArray,
            self.tracked_topic,
            self._tracks_callback,
            reliable_qos,
        )
        self.create_subscription(
            String,
            self.target_request_topic,
            self._target_request_callback,
            control_qos,
        )
        self.create_subscription(
            Bool,
            self.selection_enabled_topic,
            self._selection_enabled_callback,
            control_qos,
        )
        self.create_subscription(
            String,
            self.collection_complete_topic,
            self._collection_complete_callback,
            reliable_qos,
        )

        self.selected_pub = self.create_publisher(
            Detection3D,
            self.selected_topic,
            reliable_qos,
        )
        self.selected_id_pub = self.create_publisher(
            String,
            self.selected_id_topic,
            reliable_qos,
        )
        self.staging_pub = self.create_publisher(
            PoseStamped,
            self.staging_pose_topic,
            reliable_qos,
        )

        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish)

        self.get_logger().info(
            'Shuttle target staging node started: '
            f'staging_distance={self.staging_distance:.2f} m, '
            f'enabled={self.selection_enabled}, input={self.tracked_topic}'
        )

    def _collection_complete_callback(self, msg):
        track_id = msg.data.strip()
        if not track_id:
            return
        self.completed_ids.add(track_id)
        if track_id == self.requested_id:
            self.requested_id = ''
        if track_id == self.selected_id:
            self._clear_target('collection complete')

    def _target_request_callback(self, msg):
        track_id = msg.data.strip()
        if track_id in self.completed_ids:
            return
        if track_id == self.requested_id:
            self._try_select_requested()
            return

        self.requested_id = track_id
        self._clear_target('new target request')
        if track_id:
            self.get_logger().info(f'Requested shuttle target {track_id}.')
            self._try_select_requested()

    def _selection_enabled_callback(self, msg):
        enabled = bool(msg.data)
        if enabled == self.selection_enabled:
            if enabled:
                self._try_select_requested()
            return

        self.selection_enabled = enabled
        self.get_logger().info(
            f'Target staging {"enabled" if enabled else "disabled"}.'
        )
        if not enabled:
            self._clear_target('selection disabled')
        else:
            self._try_select_requested()

    def _warn_tf_throttled(self, error):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_tf_warning_ns < int(2.0e9):
            return
        self.last_tf_warning_ns = now_ns
        self.get_logger().warn(
            f'Cannot get robot pose {self.map_frame} -> {self.base_frame}: {error}'
        )

    def _robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as exc:
            self._warn_tf_throttled(exc)
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        return float(t.x), float(t.y), yaw_from_quaternion(q)

    def _make_staging_pose(self, robot_pose, detection):
        rx, ry, robot_yaw = robot_pose
        tx, ty, _ = detection_position(detection)
        dx = tx - rx
        dy = ty - ry
        distance = math.hypot(dx, dy)

        if distance > 1e-6:
            ux = dx / distance
            uy = dy / distance
            travel = max(0.0, distance - self.staging_distance)
            sx = rx + travel * ux
            sy = ry + travel * uy
            yaw = math.atan2(ty - sy, tx - sx)
        else:
            sx = rx
            sy = ry
            yaw = robot_yaw

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = sx
        pose.pose.position.y = sy
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        return pose

    def _set_target(self, track_id, detection, robot_pose):
        self.selected_id = track_id
        self.selected_detection = copy.deepcopy(detection)
        self.staging_pose = self._make_staging_pose(robot_pose, detection)
        tx, ty, _ = detection_position(detection)
        sx = self.staging_pose.pose.position.x
        sy = self.staging_pose.pose.position.y
        self.get_logger().info(
            f'Staging requested shuttle {track_id}: target=({tx:.2f}, {ty:.2f}), '
            f'staging=({sx:.2f}, {sy:.2f})'
        )

    def _clear_target(self, reason='target unavailable'):
        if self.selected_id:
            self.get_logger().info(
                f'Cleared shuttle target {self.selected_id}: {reason}'
            )
        self.selected_id = ''
        self.selected_detection = None
        self.staging_pose = None

    def _try_select_requested(self):
        if not self.selection_enabled or not self.requested_id:
            return
        if self.requested_id in self.completed_ids:
            return

        detection = self.latest_tracks.get(self.requested_id)
        if detection is None:
            return

        if self.selected_id == self.requested_id and self.staging_pose is not None:
            self.selected_detection = copy.deepcopy(detection)
            return

        robot_pose = self._robot_pose()
        if robot_pose is None:
            return

        self._set_target(self.requested_id, detection, robot_pose)

    def _tracks_callback(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f'Ignoring tracked shuttle array in frame {msg.header.frame_id}; '
                f'expected {self.map_frame}'
            )
            return

        self.latest_tracks = {
            detection.id: detection
            for detection in msg.detections
            if detection.id and detection.id not in self.completed_ids
        }

        self._try_select_requested()

    def _publish(self):
        id_msg = String()
        id_msg.data = self.selected_id if self.selection_enabled else ''
        self.selected_id_pub.publish(id_msg)
        if not self.selection_enabled:
            return
        if not self.selected_id:
            return
        if self.selected_detection is None or self.staging_pose is None:
            return

        self.selected_pub.publish(copy.deepcopy(self.selected_detection))
        staging = copy.deepcopy(self.staging_pose)
        staging.header.stamp = self.get_clock().now().to_msg()
        self.staging_pub.publish(staging)


def main(args=None):
    rclpy.init(args=args)
    node = ShuttleTargetSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
