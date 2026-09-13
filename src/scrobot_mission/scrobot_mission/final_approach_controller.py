#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseArray, TwistStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from scrobot_interfaces.action import CollectShuttle
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray


def quat_normalize(q):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    inv = 1.0 / norm
    return x * inv, y * inv, z * inv, w * inv


def quat_conjugate(q):
    x, y, z, w = q
    return -x, -y, -z, w


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
    out = quat_multiply(
        quat_multiply(qn, (v[0], v[1], v[2], 0.0)),
        quat_conjugate(qn),
    )
    return out[0], out[1], out[2]


def quat_to_yaw(q):
    x, y, z, w = quat_normalize(q)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def transform_point(transform, point):
    q = transform.rotation
    t = transform.translation
    rotated = quat_rotate(
        (float(q.x), float(q.y), float(q.z), float(q.w)), point
    )
    return (
        rotated[0] + float(t.x),
        rotated[1] + float(t.y),
        rotated[2] + float(t.z),
    )


def detection_position(detection):
    if detection.results:
        p = detection.results[0].pose.pose.position
    else:
        p = detection.bbox.center.position
    return float(p.x), float(p.y), float(p.z)


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def point_distance(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2
        + (a[1] - b[1]) ** 2
        + (a[2] - b[2]) ** 2
    )


class FinalApproachController(Node):
    """Best-effort ALIGN then fixed straight DRIVE collection maneuver.

    Camera / tracker data are used once to define the shuttle-group geometry.
    After ALIGN finishes, DRIVE deliberately ignores later perception changes.
    Gazebo collection events are evaluation only: they never stop or fail the
    maneuver. The action succeeds when the commanded maneuver is complete.
    """

    def __init__(self):
        super().__init__('final_approach_controller')

        self.declare_parameter('tracked_topic', '/perception/tracked_shuttles')
        self.declare_parameter('raw_detection_topic', '/perception/shuttle_detections_3d')
        self.declare_parameter('action_name', '/collect_shuttle')
        self.declare_parameter('simulation_collection_topic', '/evaluation/shuttle_collected')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_approach')
        self.declare_parameter('collection_phase_topic', '/mission/collection_phase')
        self.declare_parameter('collection_outcome_topic', '/mission/collection_outcome')

        self.declare_parameter('tracking_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('control_rate', 25.0)
        self.declare_parameter('tf_timeout', 0.05)

        # Initial local target snapshot.
        self.declare_parameter('acquisition_timeout', 0.75)
        self.declare_parameter('startup_association_distance', 1.00)

        # ALIGN: rotate toward the center of the locked group, but never wait
        # forever for perfect alignment.
        self.declare_parameter('align_kp', 2.40)
        self.declare_parameter('align_tolerance', 0.0523598776)  # 3 deg
        self.declare_parameter('align_timeout', 2.0)
        self.declare_parameter('min_align_speed', 0.15)
        self.declare_parameter('max_align_speed', 1.20)

        # DRIVE: one fixed straight pass. The distance is calculated after
        # ALIGN from the furthest locked shuttle projected onto the achieved
        # robot heading.
        self.declare_parameter('pickup_offset_x', 0.165)
        self.declare_parameter('overrun_margin', 0.10)
        self.declare_parameter('minimum_drive_distance', 0.10)
        self.declare_parameter('drive_speed', 0.20)
        self.declare_parameter('drive_timeout', 20.0)

        self.declare_parameter('collection_event_match_distance', 0.50)

        self.tracked_topic = str(self.get_parameter('tracked_topic').value)
        self.raw_detection_topic = str(self.get_parameter('raw_detection_topic').value)
        self.action_name = str(self.get_parameter('action_name').value)
        self.simulation_collection_topic = str(
            self.get_parameter('simulation_collection_topic').value
        )
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.collection_phase_topic = str(
            self.get_parameter('collection_phase_topic').value
        )
        self.collection_outcome_topic = str(
            self.get_parameter('collection_outcome_topic').value
        )

        self.tracking_frame = str(self.get_parameter('tracking_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)

        self.acquisition_timeout = float(self.get_parameter('acquisition_timeout').value)
        self.startup_association_distance = float(
            self.get_parameter('startup_association_distance').value
        )
        self.align_kp = float(self.get_parameter('align_kp').value)
        self.align_tolerance = float(self.get_parameter('align_tolerance').value)
        self.align_timeout = float(self.get_parameter('align_timeout').value)
        self.min_align_speed = float(self.get_parameter('min_align_speed').value)
        self.max_align_speed = float(self.get_parameter('max_align_speed').value)
        self.pickup_offset_x = float(self.get_parameter('pickup_offset_x').value)
        self.overrun_margin = float(self.get_parameter('overrun_margin').value)
        self.minimum_drive_distance = float(
            self.get_parameter('minimum_drive_distance').value
        )
        self.drive_speed = float(self.get_parameter('drive_speed').value)
        self.drive_timeout = float(self.get_parameter('drive_timeout').value)
        self.collection_event_match_distance = float(
            self.get_parameter('collection_event_match_distance').value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.callback_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()

        self.tracks = {}
        self.raw_detections = []
        self.raw_frame = ''
        self.active_ids = []
        self.collected_ids = set()
        self.current_phase = ''
        self.current_outcome = ''

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            Detection3DArray,
            self.tracked_topic,
            self._tracks_callback,
            reliable_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Detection3DArray,
            self.raw_detection_topic,
            self._raw_detections_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            PoseArray,
            self.simulation_collection_topic,
            self._simulation_collection_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped, self.cmd_vel_topic, reliable_qos
        )
        self.phase_pub = self.create_publisher(
            String, self.collection_phase_topic, latched_qos
        )
        self.outcome_pub = self.create_publisher(
            String, self.collection_outcome_topic, latched_qos
        )

        self.action_server = ActionServer(
            self,
            CollectShuttle,
            self.action_name,
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )

        self._set_phase('IDLE')
        self._set_outcome('UNKNOWN')
        self.get_logger().info(
            'CollectShuttle ready: deterministic ALIGN -> DRIVE; '
            f'collector_x={self.pickup_offset_x:.3f} m, '
            f'overrun={self.overrun_margin:.3f} m, '
            f'drive_speed={self.drive_speed:.2f} m/s.'
        )

    def _set_phase(self, phase):
        if phase == self.current_phase:
            return
        self.current_phase = phase
        msg = String()
        msg.data = phase
        self.phase_pub.publish(msg)
        self.get_logger().info(f'Collection phase -> {phase}')

    def _set_outcome(self, outcome):
        if outcome == self.current_outcome:
            return
        self.current_outcome = outcome
        msg = String()
        msg.data = outcome
        self.outcome_pub.publish(msg)
        self.get_logger().info(f'Collection outcome -> {outcome}')

    def _tracks_callback(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.tracking_frame:
            return
        with self.lock:
            self.tracks = {d.id: d for d in msg.detections if d.id}

    def _raw_detections_callback(self, msg):
        with self.lock:
            self.raw_frame = msg.header.frame_id
            self.raw_detections = [detection_position(d) for d in msg.detections]

    def _goal_callback(self, goal_request):
        ids = [item.strip() for item in goal_request.shuttle_ids if item.strip()]
        if not ids:
            return GoalResponse.REJECT
        with self.lock:
            if self.active_ids:
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _simulation_collection_callback(self, msg):
        """Record intended IDs collected during the pass; never alter control."""
        if not msg.poses:
            return

        with self.lock:
            active_ids = list(self.active_ids)
            tracks = {track_id: self.tracks.get(track_id) for track_id in active_ids}
            already = set(self.collected_ids)

        newly_collected = []
        for track_id, detection in tracks.items():
            if detection is None or track_id in already:
                continue
            tx, ty, tz = detection_position(detection)
            best = min(
                math.sqrt(
                    (float(p.position.x) - tx) ** 2
                    + (float(p.position.y) - ty) ** 2
                    + (float(p.position.z) - tz) ** 2
                )
                for p in msg.poses
            )
            if best <= self.collection_event_match_distance:
                newly_collected.append(track_id)

        if newly_collected:
            with self.lock:
                self.collected_ids.update(newly_collected)
            self.get_logger().info(
                'Observed physical collection: ' + ', '.join(newly_collected)
            )

    def _publish_cmd(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def _publish_stop(self):
        self._publish_cmd(0.0, 0.0)

    def _lookup_transform(self, target, source):
        try:
            return self.tf_buffer.lookup_transform(
                target,
                source,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException:
            return None

    def _transform_point_to_base(self, source_frame, point):
        if not source_frame:
            return None
        if source_frame == self.base_frame:
            return point
        tf = self._lookup_transform(self.base_frame, source_frame)
        if tf is None:
            return None
        return transform_point(tf, point)

    def _base_pose_in_odom(self):
        tf = self._lookup_transform(self.odom_frame, self.base_frame)
        if tf is None:
            return None
        q = tf.rotation
        yaw = quat_to_yaw((float(q.x), float(q.y), float(q.z), float(q.w)))
        return float(tf.translation.x), float(tf.translation.y), yaw

    def _expected_group_local(self):
        with self.lock:
            active_ids = list(self.active_ids)
            tracks = {track_id: self.tracks.get(track_id) for track_id in active_ids}

        points = []
        for track_id in active_ids:
            detection = tracks.get(track_id)
            if detection is None:
                continue
            local = self._transform_point_to_base(
                self.tracking_frame,
                detection_position(detection),
            )
            if local is not None:
                points.append(local)
        return points

    def _raw_points_local(self):
        with self.lock:
            raw_points = list(self.raw_detections)
            raw_frame = self.raw_frame
        points = []
        for point in raw_points:
            local = self._transform_point_to_base(raw_frame, point)
            if local is not None and local[0] > -0.05:
                points.append(local)
        return points

    def _snapshot_group_local(self):
        """Take one local geometry snapshot; no later perception is required."""
        expected = self._expected_group_local()
        raw = self._raw_points_local()
        desired_count = max(1, len(self.active_ids))

        if raw and expected:
            remaining = list(raw)
            matched = []
            for expected_point in expected:
                if not remaining:
                    break
                index = min(
                    range(len(remaining)),
                    key=lambda i: point_distance(remaining[i], expected_point),
                )
                if point_distance(remaining[index], expected_point) <= self.startup_association_distance:
                    matched.append(remaining.pop(index))
            if matched:
                return matched

        if raw:
            # Primary/group selection already happened before this action. If
            # map association is poor, use the nearest visible local points.
            raw.sort(key=lambda p: math.hypot(p[0], p[1]))
            return raw[:desired_count]

        # Final fallback: use the locked persistent tracks transformed once.
        return expected

    @staticmethod
    def _group_heading(points):
        ys = [p[1] for p in points]
        center_y = 0.5 * (min(ys) + max(ys))
        center_x = sum(p[0] for p in points) / float(len(points))
        return math.atan2(center_y, max(center_x, 1e-6))

    def _align(self, points, goal_handle, feedback):
        """Rotate toward the group center as well as possible within timeout."""
        desired_delta = self._group_heading(points)
        start_pose = self._base_pose_in_odom()
        start_time = time.monotonic()

        if abs(desired_delta) <= self.align_tolerance:
            self._publish_stop()
            return 0.0

        # Preferred closed-loop local odometry alignment.
        if start_pose is not None:
            target_yaw = wrap_angle(start_pose[2] + desired_delta)
            while rclpy.ok() and time.monotonic() - start_time < self.align_timeout:
                if goal_handle.is_cancel_requested:
                    return None
                pose = self._base_pose_in_odom()
                if pose is None:
                    break
                error = wrap_angle(target_yaw - pose[2])
                feedback.distance_to_target = 0.0
                goal_handle.publish_feedback(feedback)
                if abs(error) <= self.align_tolerance:
                    self._publish_stop()
                    return wrap_angle(pose[2] - start_pose[2])

                speed = clamp(
                    abs(self.align_kp * error),
                    self.min_align_speed,
                    self.max_align_speed,
                )
                self._publish_cmd(0.0, math.copysign(speed, error))
                time.sleep(1.0 / max(self.control_rate, 1.0))

            self._publish_stop()
            pose = self._base_pose_in_odom()
            if pose is not None:
                achieved = wrap_angle(pose[2] - start_pose[2])
                self.get_logger().warn(
                    f'ALIGN ended at timeout/best effort: desired={math.degrees(desired_delta):.1f} deg, '
                    f'achieved={math.degrees(achieved):.1f} deg.'
                )
                return achieved

        # Odom TF unavailable: bounded open-loop best effort, never block.
        speed = clamp(
            abs(self.align_kp * desired_delta),
            self.min_align_speed,
            self.max_align_speed,
        )
        duration = min(self.align_timeout, abs(desired_delta) / max(speed, 1e-6))
        end = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end:
            if goal_handle.is_cancel_requested:
                return None
            self._publish_cmd(0.0, math.copysign(speed, desired_delta))
            time.sleep(1.0 / max(self.control_rate, 1.0))
        self._publish_stop()
        self.get_logger().warn('ALIGN used bounded open-loop fallback because odom TF was unavailable.')
        return desired_delta

    def _drive_distance_for_group(self, points, achieved_rotation):
        c = math.cos(achieved_rotation)
        s = math.sin(achieved_rotation)
        projected_forward = [c * p[0] + s * p[1] for p in points]
        furthest_x = max(projected_forward)
        distance = furthest_x - self.pickup_offset_x + self.overrun_margin
        return max(self.minimum_drive_distance, distance), furthest_x

    def _drive_straight(self, distance, goal_handle, feedback):
        start_pose = self._base_pose_in_odom()
        start_time = time.monotonic()
        allowed_time = max(
            self.drive_timeout,
            distance / max(self.drive_speed, 1e-6) + 2.0,
        )
        period = 1.0 / max(self.control_rate, 1.0)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return False, 0.0, 'canceled'

            elapsed = time.monotonic() - start_time
            travelled = elapsed * self.drive_speed
            if start_pose is not None:
                pose = self._base_pose_in_odom()
                if pose is not None:
                    travelled = math.hypot(
                        pose[0] - start_pose[0],
                        pose[1] - start_pose[1],
                    )

            feedback.distance_to_target = float(max(0.0, distance - travelled))
            goal_handle.publish_feedback(feedback)

            if travelled >= distance:
                self._publish_stop()
                return True, travelled, 'distance reached'

            if elapsed >= allowed_time:
                self._publish_stop()
                self.get_logger().warn(
                    f'DRIVE best-effort timeout: requested={distance:.3f} m, '
                    f'travelled={travelled:.3f} m.'
                )
                return True, travelled, 'best-effort timeout'

            # Deliberately straight. Ignore camera, tracker and collection
            # events until the planned pass distance is complete.
            self._publish_cmd(self.drive_speed, 0.0)
            time.sleep(period)

        return True, 0.0, 'ROS shutdown'

    def _final_outcome(self, intended_ids):
        with self.lock:
            collected = sorted(self.collected_ids)
        count = len(collected)
        if count <= 0:
            outcome = 'MISSED'
        elif count >= len(intended_ids):
            outcome = 'COLLECTED'
        else:
            outcome = 'PARTIAL'
        return outcome, collected

    def _clear_active(self):
        with self.lock:
            self.active_ids = []
            self.collected_ids = set()

    def _execute(self, goal_handle):
        intended_ids = []
        for item in goal_handle.request.shuttle_ids:
            shuttle_id = item.strip()
            if shuttle_id and shuttle_id not in intended_ids:
                intended_ids.append(shuttle_id)

        result = CollectShuttle.Result()
        feedback = CollectShuttle.Feedback()

        with self.lock:
            self.active_ids = list(intended_ids)
            self.collected_ids = set()

        self._set_outcome('UNKNOWN')
        self._set_phase('ALIGN')
        self.get_logger().info(
            f'Collection pass for {intended_ids}: ALIGN once, then fixed straight DRIVE.'
        )

        try:
            # Acquire one local geometry snapshot. Once obtained, perception is
            # intentionally ignored for the remainder of the maneuver.
            points = []
            acquisition_start = time.monotonic()
            while rclpy.ok() and time.monotonic() - acquisition_start < self.acquisition_timeout:
                if goal_handle.is_cancel_requested:
                    self._publish_stop()
                    self._set_phase('CANCELED')
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Collection maneuver canceled.'
                    result.collected_ids = sorted(self.collected_ids)
                    return result
                points = self._snapshot_group_local()
                if points:
                    break
                self._publish_stop()
                time.sleep(1.0 / max(self.control_rate, 1.0))

            if not points:
                # Nothing usable to drive toward. This is an observed miss, not
                # a mission failure: complete the action and move on.
                self._publish_stop()
                outcome, collected = self._final_outcome(intended_ids)
                self._set_outcome(outcome)
                self._set_phase('DONE')
                goal_handle.succeed()
                result.success = True
                result.collected_ids = collected
                result.message = 'No local target geometry available; maneuver skipped and mission may continue.'
                self.get_logger().warn(result.message)
                return result

            heading = self._group_heading(points)
            self.get_logger().info(
                f'Locked geometry: {len(points)} point(s), '
                f'group heading={math.degrees(heading):.1f} deg. '
                'Later perception changes will be ignored.'
            )

            achieved_rotation = self._align(points, goal_handle, feedback)
            if achieved_rotation is None:
                self._publish_stop()
                self._set_phase('CANCELED')
                goal_handle.canceled()
                result.success = False
                result.message = 'Collection maneuver canceled during ALIGN.'
                result.collected_ids = sorted(self.collected_ids)
                return result

            drive_distance, furthest_x = self._drive_distance_for_group(
                points, achieved_rotation
            )
            self.get_logger().info(
                f'DRIVE plan: furthest projected shuttle x={furthest_x:.3f} m, '
                f'collector_x={self.pickup_offset_x:.3f} m, '
                f'overrun={self.overrun_margin:.3f} m -> '
                f'drive {drive_distance:.3f} m straight.'
            )

            self._set_phase('DRIVE')
            completed, travelled, reason = self._drive_straight(
                drive_distance, goal_handle, feedback
            )
            if not completed:
                self._publish_stop()
                self._set_phase('CANCELED')
                goal_handle.canceled()
                result.success = False
                result.message = 'Collection maneuver canceled during DRIVE.'
                result.collected_ids = sorted(self.collected_ids)
                return result

            outcome, collected = self._final_outcome(intended_ids)
            self._set_outcome(outcome)
            self._set_phase('DONE')

            # Success means the robot completed its best-effort maneuver. The
            # simulation-only collection outcome is reported separately.
            goal_handle.succeed()
            result.success = True
            result.collected_ids = collected
            result.message = (
                f'Collection maneuver complete ({reason}); travelled={travelled:.3f} m; '
                f'outcome={outcome}; intended={intended_ids}; collected={collected}.'
            )
            self.get_logger().info(result.message)
            return result
        finally:
            self._publish_stop()
            self._clear_active()

    def destroy_node(self):
        self._publish_stop()
        self.action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FinalApproachController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
