#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseArray, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection3DArray


class TelemetryMonitor(Node):
    """Single terminal-friendly view of SC Robot runtime telemetry.

    This node is intentionally debug-only. Production packages publish their
    normal state/topics and logs; scrobot_debug observes those interfaces and
    consolidates them without adding debug behavior to mission/control code.
    """

    LEVEL_NAMES = {
        10: 'DEBUG',
        20: 'INFO',
        30: 'WARN',
        40: 'ERROR',
        50: 'FATAL',
    }

    def __init__(self):
        super().__init__('telemetry_monitor')

        self.declare_parameter('output_topic', '/debug/telemetry')
        self.declare_parameter('summary_rate', 1.0)
        self.declare_parameter('relay_rosout', True)
        self.declare_parameter('min_rosout_level', 20)
        self.declare_parameter(
            'rosout_nodes',
            [
                'manual_mode_manager',
                'sweep_mission_manager',
                'local_collect_controller',
                'shuttle_collection_filter',
                'fake_shuttle_detector',
                'shuttle_tracker',
                'tag_global_localizer',
                'tag_approach_controller',
                'controller_server',
                'collision_monitor',
            ],
        )

        self.output_topic = str(self.get_parameter('output_topic').value)
        self.summary_rate = max(
            0.0, float(self.get_parameter('summary_rate').value)
        )
        self.relay_rosout = bool(self.get_parameter('relay_rosout').value)
        self.min_rosout_level = int(
            self.get_parameter('min_rosout_level').value
        )
        self.rosout_nodes = {
            str(name).lstrip('/')
            for name in self.get_parameter('rosout_nodes').value
        }

        reliable_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        rosout_qos = QoSProfile(
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(
            String, self.output_topic, reliable_qos
        )

        self.create_subscription(
            Bool, '/control/manual_mode', self._control_mode_cb, state_qos
        )
        self.create_subscription(
            String, '/mission/state', self._mission_state_cb, state_qos
        )
        self.create_subscription(
            String,
            '/mission/local_collect_phase',
            self._local_collect_phase_cb,
            reliable_qos,
        )
        self.create_subscription(
            Detection3DArray,
            '/perception/shuttle_detections_3d',
            self._raw_detections_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Detection3DArray,
            '/perception/collectable_shuttle_detections_3d',
            self._eligible_detections_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseArray,
            '/evaluation/shuttle_ground_truth',
            self._ground_truth_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseArray,
            '/evaluation/shuttle_collected',
            self._collected_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self._odom_cb,
            reliable_qos,
        )
        self.create_subscription(
            Odometry,
            '/evaluation/ground_truth_odom',
            self._ground_truth_odom_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped,
            '/cmd_vel_auto',
            self._auto_cmd_cb,
            reliable_qos,
        )
        self.create_subscription(
            TwistStamped,
            '/cmd_vel_selected',
            self._selected_cmd_cb,
            reliable_qos,
        )
        self.create_subscription(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            self._drive_cmd_cb,
            reliable_qos,
        )
        self.create_subscription(
            String,
            '/debug/shuttle_physics',
            self._shuttle_physics_cb,
            reliable_qos,
        )
        self.create_subscription(
            String,
            '/debug/collection_test',
            self._collection_test_cb,
            reliable_qos,
        )
        self.create_subscription(
            String,
            '/debug/camera_test',
            self._camera_test_cb,
            reliable_qos,
        )

        if self.relay_rosout:
            self.create_subscription(
                Log, '/rosout', self._rosout_cb, rosout_qos
            )

        self.control_mode = 'UNKNOWN'
        self.mission_state = 'UNKNOWN'
        self.local_collect_phase = 'UNKNOWN'
        self.raw_count = None
        self.eligible_count = None
        self.gt_shuttle_count = None
        self.collected_count = 0
        self.odom = None
        self.gt_odom = None
        self.auto_cmd = None
        self.selected_cmd = None
        self.drive_cmd = None

        if self.summary_rate > 0.0:
            self.create_timer(
                1.0 / self.summary_rate,
                self._publish_summary,
            )

        self.get_logger().info(
            f'Telemetry monitor ready; consolidated topic={self.output_topic}'
        )

    def _stamp_text(self):
        now_ns = self.get_clock().now().nanoseconds
        return f'{now_ns / 1.0e9:10.3f}'

    def _emit(self, source, text):
        line = f'{self._stamp_text()} [{source}] {text}'
        msg = String()
        msg.data = line
        self.publisher.publish(msg)
        print(line, flush=True)

    @staticmethod
    def _yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _control_mode_cb(self, msg):
        mode = 'MANUAL' if bool(msg.data) else 'AUTO'
        if mode == self.control_mode:
            return
        previous = self.control_mode
        self.control_mode = mode
        self._emit('CONTROL', f'mode {previous} -> {mode}')

    def _mission_state_cb(self, msg):
        state = str(msg.data)
        if state == self.mission_state:
            return
        previous = self.mission_state
        self.mission_state = state
        self._emit('MISSION', f'state {previous} -> {state}')

    def _local_collect_phase_cb(self, msg):
        phase = str(msg.data)
        if phase == self.local_collect_phase:
            return
        previous = self.local_collect_phase
        self.local_collect_phase = phase
        self._emit('LOCAL', f'phase {previous} -> {phase}')

    def _raw_detections_cb(self, msg):
        count = len(msg.detections)
        if count != self.raw_count:
            self._emit('PERCEPTION', f'raw_visible={count}')
            self.raw_count = count

    def _eligible_detections_cb(self, msg):
        count = len(msg.detections)
        if count != self.eligible_count:
            self._emit('PERCEPTION', f'collectable_visible={count}')
            self.eligible_count = count

    def _ground_truth_cb(self, msg):
        count = len(msg.poses)
        if count != self.gt_shuttle_count:
            self._emit('SIM', f'ground_truth_shuttles={count}')
            self.gt_shuttle_count = count

    def _collected_cb(self, msg):
        if not msg.poses:
            return
        self.collected_count += len(msg.poses)
        self._emit(
            'SIM',
            f'collection_event={len(msg.poses)} total_collected={self.collected_count}',
        )

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom = (float(p.x), float(p.y), self._yaw(q))

    def _ground_truth_odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.gt_odom = (float(p.x), float(p.y), self._yaw(q))

    @staticmethod
    def _twist_pair(msg):
        return (
            float(msg.twist.linear.x),
            float(msg.twist.angular.z),
        )

    def _auto_cmd_cb(self, msg):
        self.auto_cmd = self._twist_pair(msg)

    def _selected_cmd_cb(self, msg):
        self.selected_cmd = self._twist_pair(msg)

    def _drive_cmd_cb(self, msg):
        self.drive_cmd = self._twist_pair(msg)

    def _shuttle_physics_cb(self, msg):
        self._emit('SHUTTLE_PHYS', str(msg.data))

    def _collection_test_cb(self, msg):
        self._emit('COLLECTION_TEST', str(msg.data))

    def _camera_test_cb(self, msg):
        self._emit('CAMERA_TEST', str(msg.data))

    def _rosout_cb(self, msg):
        logger_name = str(msg.name).lstrip('/')
        if logger_name not in self.rosout_nodes:
            return
        if int(msg.level) < self.min_rosout_level:
            return
        level = self.LEVEL_NAMES.get(int(msg.level), str(int(msg.level)))
        self._emit(f'LOG:{logger_name}', f'{level}: {msg.msg}')

    def _publish_summary(self):
        raw = '?' if self.raw_count is None else str(self.raw_count)
        eligible = '?' if self.eligible_count is None else str(self.eligible_count)
        gt = '?' if self.gt_shuttle_count is None else str(self.gt_shuttle_count)

        pose_text = 'pose=?'
        if self.odom is not None:
            pose_text = (
                f'pose=({self.odom[0]:+.2f},{self.odom[1]:+.2f},'
                f'{math.degrees(self.odom[2]):+.1f}deg)'
            )

        error_text = ''
        if self.odom is not None and self.gt_odom is not None:
            pos_err = math.hypot(
                self.odom[0] - self.gt_odom[0],
                self.odom[1] - self.gt_odom[1],
            )
            yaw_err = math.atan2(
                math.sin(self.odom[2] - self.gt_odom[2]),
                math.cos(self.odom[2] - self.gt_odom[2]),
            )
            error_text = (
                f' err=({pos_err:.3f}m,'
                f'{math.degrees(yaw_err):+.2f}deg)'
            )

        cmd_text = ''
        if self.drive_cmd is not None:
            cmd_text = (
                f' drive=({self.drive_cmd[0]:+.2f}m/s,'
                f'{self.drive_cmd[1]:+.2f}rad/s)'
            )
        elif self.selected_cmd is not None:
            cmd_text = (
                f' selected=({self.selected_cmd[0]:+.2f}m/s,'
                f'{self.selected_cmd[1]:+.2f}rad/s)'
            )
        elif self.auto_cmd is not None:
            cmd_text = (
                f' auto=({self.auto_cmd[0]:+.2f}m/s,'
                f'{self.auto_cmd[1]:+.2f}rad/s)'
            )

        self._emit(
            'SUMMARY',
            f'mode={self.control_mode} mission={self.mission_state} '
            f'local={self.local_collect_phase} raw={raw} eligible={eligible} '
            f'gt={gt} collected={self.collected_count} '
            f'{pose_text}{error_text}{cmd_text}',
        )


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
