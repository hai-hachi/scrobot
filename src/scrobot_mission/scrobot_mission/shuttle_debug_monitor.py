#!/usr/bin/env python3

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray


class ShuttleDebugMonitor(Node):
    """Aggregate shuttle-perception / local-collect debug information.

    The node combines event summaries from the shuttle detection topics with
    selected /rosout messages from the nodes involved in collection. The goal
    is to have one terminal-friendly topic for controller tuning without
    changing the mission/controller logic itself.
    """

    LEVEL_NAMES = {
        10: 'DEBUG',
        20: 'INFO',
        30: 'WARN',
        40: 'ERROR',
        50: 'FATAL',
    }

    def __init__(self):
        super().__init__('shuttle_debug_monitor')

        self.declare_parameter('output_topic', '/debug/shuttle_collection')
        self.declare_parameter(
            'raw_detection_topic', '/perception/shuttle_detections_3d'
        )
        self.declare_parameter('tracked_topic', '/perception/tracked_shuttles')
        self.declare_parameter(
            'visible_tracked_topic', '/perception/visible_tracked_shuttles'
        )
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter(
            'local_collect_phase_topic', '/mission/local_collect_phase'
        )
        self.declare_parameter('summary_rate', 1.0)
        self.declare_parameter('relay_rosout', True)
        self.declare_parameter('min_rosout_level', 20)
        self.declare_parameter(
            'rosout_nodes',
            [
                'fake_shuttle_detector',
                'shuttle_tracker',
                'local_collect_controller',
                'sweep_mission_manager',
            ],
        )

        self.output_topic = str(self.get_parameter('output_topic').value)
        self.raw_detection_topic = str(
            self.get_parameter('raw_detection_topic').value
        )
        self.tracked_topic = str(self.get_parameter('tracked_topic').value)
        self.visible_tracked_topic = str(
            self.get_parameter('visible_tracked_topic').value
        )
        self.mission_state_topic = str(
            self.get_parameter('mission_state_topic').value
        )
        self.local_collect_phase_topic = str(
            self.get_parameter('local_collect_phase_topic').value
        )
        self.summary_rate = float(self.get_parameter('summary_rate').value)
        self.relay_rosout = bool(self.get_parameter('relay_rosout').value)
        self.min_rosout_level = int(self.get_parameter('min_rosout_level').value)
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

        self.publisher = self.create_publisher(String, self.output_topic, reliable_qos)

        self.create_subscription(
            Detection3DArray,
            self.raw_detection_topic,
            self._raw_detections_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Detection3DArray,
            self.tracked_topic,
            self._tracked_cb,
            reliable_qos,
        )
        self.create_subscription(
            Detection3DArray,
            self.visible_tracked_topic,
            self._visible_tracked_cb,
            reliable_qos,
        )
        self.create_subscription(
            String,
            self.mission_state_topic,
            self._mission_state_cb,
            state_qos,
        )
        self.create_subscription(
            String,
            self.local_collect_phase_topic,
            self._local_collect_phase_cb,
            reliable_qos,
        )

        if self.relay_rosout:
            self.create_subscription(Log, '/rosout', self._rosout_cb, rosout_qos)

        self.raw_count = None
        self.confirmed_ids = None
        self.visible_ids = None
        self.mission_state = 'UNKNOWN'
        self.local_collect_phase = 'UNKNOWN'

        if self.summary_rate > 0.0:
            self.create_timer(1.0 / self.summary_rate, self._publish_summary)

        self.get_logger().info(
            f'Shuttle debug monitor publishing consolidated events on {self.output_topic}'
        )

    def _stamp_text(self):
        now_ns = self.get_clock().now().nanoseconds
        return f'{now_ns / 1.0e9:10.3f}'

    def _emit(self, source, text):
        msg = String()
        msg.data = f'{self._stamp_text()} [{source}] {text}'
        self.publisher.publish(msg)

    @staticmethod
    def _ids(msg):
        ids = []
        for index, detection in enumerate(msg.detections):
            track_id = str(detection.id).strip()
            ids.append(track_id if track_id else f'?{index}')
        return tuple(sorted(ids))

    def _raw_detections_cb(self, msg):
        count = len(msg.detections)
        previous = self.raw_count
        self.raw_count = count

        if previous is None:
            self._emit('PERCEPTION', f'raw_visible={count}')
            return
        if count == previous:
            return

        delta = count - previous
        if delta > 0:
            detail = f'+{delta} entered/detected'
        else:
            detail = f'{-delta} left/lost from raw FOV'
        self._emit(
            'PERCEPTION',
            f'raw_visible {previous} -> {count} ({detail})',
        )

    def _tracked_cb(self, msg):
        ids = self._ids(msg)
        previous = self.confirmed_ids
        self.confirmed_ids = ids

        if previous is None:
            self._emit('TRACKER', f'confirmed={len(ids)} ids={list(ids)}')
            return
        if ids == previous:
            return

        old = set(previous)
        new = set(ids)
        added = sorted(new - old)
        removed = sorted(old - new)
        self._emit(
            'TRACKER',
            f'confirmed={len(ids)} ids={list(ids)} '
            f'added={added} removed={removed}',
        )

    def _visible_tracked_cb(self, msg):
        ids = self._ids(msg)
        previous = self.visible_ids
        self.visible_ids = ids

        if previous is None:
            self._emit('TRACKER', f'visible_tracks={len(ids)} ids={list(ids)}')
            return
        if ids == previous:
            return

        old = set(previous)
        new = set(ids)
        entered = sorted(new - old)
        lost = sorted(old - new)
        self._emit(
            'TRACKER',
            f'visible_tracks={len(ids)} ids={list(ids)} '
            f'entered={entered} lost_from_FOV={lost}',
        )

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
        self._emit('LOCAL_COLLECT', f'phase {previous} -> {phase}')

    def _rosout_cb(self, msg):
        logger_name = str(msg.name).lstrip('/')
        if logger_name not in self.rosout_nodes:
            return
        if int(msg.level) < self.min_rosout_level:
            return

        level = self.LEVEL_NAMES.get(int(msg.level), str(int(msg.level)))
        self._emit(
            f'LOG:{logger_name}',
            f'{level}: {msg.msg}',
        )

    def _publish_summary(self):
        raw = '?' if self.raw_count is None else str(self.raw_count)
        confirmed = '?' if self.confirmed_ids is None else str(len(self.confirmed_ids))
        visible = '?' if self.visible_ids is None else str(len(self.visible_ids))
        self._emit(
            'SUMMARY',
            f'mission={self.mission_state} '
            f'local_collect={self.local_collect_phase} '
            f'raw_visible={raw} confirmed={confirmed} visible_tracks={visible}',
        )


def main(args=None):
    rclpy.init(args=args)
    node = ShuttleDebugMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
