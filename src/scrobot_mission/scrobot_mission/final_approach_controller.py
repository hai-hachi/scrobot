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


def point_distance(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2
        + (a[1] - b[1]) ** 2
        + (a[2] - b[2]) ** 2
    )


class FinalApproachController(Node):
    """Camera-local collection servo with a bounded blind-zone commit phase."""

    def __init__(self):
        super().__init__('final_approach_controller')

        self.declare_parameter('tracked_topic', '/perception/tracked_shuttles')
        self.declare_parameter('raw_detection_topic', '/perception/shuttle_detections_3d')
        self.declare_parameter('action_name', '/collect_shuttle')
        self.declare_parameter('simulation_collection_topic', '/evaluation/shuttle_collected')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_approach')
        self.declare_parameter('collection_phase_topic', '/mission/collection_phase')
        self.declare_parameter('tracking_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('control_rate', 25.0)
        self.declare_parameter('timeout', 20.0)
        self.declare_parameter('tf_timeout', 0.05)

        self.declare_parameter('linear_kp', 1.10)
        self.declare_parameter('angular_kp', 2.40)
        self.declare_parameter('min_linear_speed', 0.05)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_angular_speed', 1.20)
        self.declare_parameter('heading_slowdown_angle', 0.35)
        self.declare_parameter('rotate_only_angle', 0.15)
        self.declare_parameter('pickup_offset_x', 0.165)

        self.declare_parameter('startup_association_distance', 1.00)
        self.declare_parameter('camera_association_distance', 0.50)
        self.declare_parameter('camera_lost_grace_time', 0.25)
        self.declare_parameter('commit_speed', 0.12)
        self.declare_parameter('commit_max_distance', 0.45)
        self.declare_parameter('commit_timeout', 5.0)
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
        self.tracking_frame = str(self.get_parameter('tracking_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.linear_kp = float(self.get_parameter('linear_kp').value)
        self.angular_kp = float(self.get_parameter('angular_kp').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.heading_slowdown_angle = float(
            self.get_parameter('heading_slowdown_angle').value
        )
        self.rotate_only_angle = float(self.get_parameter('rotate_only_angle').value)
        self.pickup_offset_x = float(self.get_parameter('pickup_offset_x').value)
        self.startup_association_distance = float(
            self.get_parameter('startup_association_distance').value
        )
        self.camera_association_distance = float(
            self.get_parameter('camera_association_distance').value
        )
        self.camera_lost_grace_time = float(
            self.get_parameter('camera_lost_grace_time').value
        )
        self.commit_speed = float(self.get_parameter('commit_speed').value)
        self.commit_max_distance = float(
            self.get_parameter('commit_max_distance').value
        )
        self.commit_timeout = float(self.get_parameter('commit_timeout').value)
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
        self.primary_id = ''
        self.collected_ids = set()
        self.current_phase = ''

        self.camera_locked = False
        self.last_group_points = []
        self.last_local_target = None
        self.last_local_target_time = 0.0
        self.camera_loss_start_time = None
        self.last_wait_log_time = 0.0

        self.commit_active = False
        self.commit_start_time = None
        self.commit_start_xy = None

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        phase_qos = QoSProfile(
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
            String, self.collection_phase_topic, phase_qos
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
        self.get_logger().info(
            f'CollectShuttle camera-local servo ready on {self.action_name}; '
            f'phase={self.collection_phase_topic}, raw={self.raw_detection_topic}, '
            f'collector_x={self.pickup_offset_x:.3f} m, '
            f'commit={self.commit_max_distance:.2f} m @ {self.commit_speed:.2f} m/s.'
        )

    def _set_phase(self, phase):
        if phase == self.current_phase:
            return
        self.current_phase = phase
        msg = String()
        msg.data = phase
        self.phase_pub.publish(msg)
        self.get_logger().info(f'Collection phase -> {phase}')

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
        if not msg.poses:
            return

        with self.lock:
            active_ids = list(self.active_ids)
            tracks = {track_id: self.tracks.get(track_id) for track_id in active_ids}
            already_collected = set(self.collected_ids)

        newly_collected = []
        for track_id, detection in tracks.items():
            if detection is None or track_id in already_collected:
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
                'Physical collection matched: ' + ', '.join(newly_collected)
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

    def _transform_point_to_base(self, source_frame, point):
        if not source_frame:
            return None
        if source_frame == self.base_frame:
            return point
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException:
            return None
        return transform_point(tf, point)

    def _base_xy_in_odom(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException:
            return None
        return float(tf.translation.x), float(tf.translation.y)

    def _raw_points_local(self):
        with self.lock:
            raw_points = list(self.raw_detections)
            raw_frame = self.raw_frame

        output = []
        for point in raw_points:
            local = self._transform_point_to_base(raw_frame, point)
            if local is not None and local[0] > -0.05:
                output.append(local)
        return output

    def _expected_targets_local(self):
        with self.lock:
            active_ids = list(self.active_ids)
            tracks = {track_id: self.tracks.get(track_id) for track_id in active_ids}
            collected = set(self.collected_ids)

        output = []
        for track_id in active_ids:
            if track_id in collected:
                continue
            detection = tracks.get(track_id)
            if detection is None:
                continue
            local = self._transform_point_to_base(
                self.tracking_frame,
                detection_position(detection),
            )
            if local is not None:
                output.append(local)
        return output

    @staticmethod
    def _center(points):
        if not points:
            return None
        count = float(len(points))
        return (
            sum(p[0] for p in points) / count,
            sum(p[1] for p in points) / count,
            sum(p[2] for p in points) / count,
        )

    @staticmethod
    def _corridor_target(points):
        if not points:
            return None
        count = float(len(points))
        mean_x = sum(p[0] for p in points) / count
        mean_z = sum(p[2] for p in points) / count
        ys = [p[1] for p in points]
        center_y = 0.5 * (min(ys) + max(ys))
        return mean_x, center_y, mean_z

    def _initial_camera_lock(self, raw_local):
        expected = self._expected_targets_local()
        desired_count = max(1, len(self.active_ids) - len(self.collected_ids))
        if not raw_local:
            return None

        matched = []
        remaining = list(raw_local)
        for expected_point in expected:
            if not remaining:
                break
            best_index = min(
                range(len(remaining)),
                key=lambda i: point_distance(remaining[i], expected_point),
            )
            best_distance = point_distance(remaining[best_index], expected_point)
            if best_distance <= self.startup_association_distance:
                matched.append(remaining.pop(best_index))

        if matched:
            center = self._center(matched)
            while remaining and len(matched) < desired_count:
                best_index = min(
                    range(len(remaining)),
                    key=lambda i: point_distance(remaining[i], center),
                )
                candidate = remaining[best_index]
                if point_distance(candidate, center) > self.startup_association_distance:
                    break
                matched.append(remaining.pop(best_index))
                center = self._center(matched)

            self.camera_locked = True
            self.last_group_points = matched
            target = self._corridor_target(matched)
            self.get_logger().info(
                f'Camera-local target acquired: {len(matched)} point(s), '
                f'corridor x={target[0]:.3f}, y={target[1]:.3f} m.'
            )
            return target

        nearest = min(raw_local, key=lambda p: math.hypot(p[0], p[1]))
        selected = [nearest]
        others = [p for p in raw_local if p is not nearest]
        while others and len(selected) < desired_count:
            center = self._center(selected)
            best_index = min(
                range(len(others)),
                key=lambda i: point_distance(others[i], center),
            )
            candidate = others[best_index]
            if abs(candidate[1] - center[1]) > 0.30:
                break
            selected.append(others.pop(best_index))

        self.camera_locked = True
        self.last_group_points = selected
        target = self._corridor_target(selected)
        self.get_logger().warn(
            'Map-to-camera startup association failed; using visible raw '
            f'camera target directly at x={target[0]:.3f}, y={target[1]:.3f} m.'
        )
        return target

    def _track_camera_group(self, raw_local):
        if not raw_local or not self.last_group_points:
            return None

        desired_count = max(1, len(self.active_ids) - len(self.collected_ids))
        previous = list(self.last_group_points)
        remaining = list(raw_local)
        matched = []

        for previous_point in previous:
            if not remaining or len(matched) >= desired_count:
                break
            best_index = min(
                range(len(remaining)),
                key=lambda i: point_distance(remaining[i], previous_point),
            )
            best_distance = point_distance(remaining[best_index], previous_point)
            if best_distance <= self.camera_association_distance:
                matched.append(remaining.pop(best_index))

        if not matched:
            return None

        self.last_group_points = matched
        return self._corridor_target(matched)

    def _camera_group_target_local(self):
        raw_local = self._raw_points_local()
        if not raw_local:
            return None
        if not self.camera_locked:
            return self._initial_camera_lock(raw_local)
        return self._track_camera_group(raw_local)

    def _log_waiting_for_camera(self, now):
        if now - self.last_wait_log_time < 1.0:
            return
        self.last_wait_log_time = now
        with self.lock:
            raw_count = len(self.raw_detections)
            raw_frame = self.raw_frame
            track_count = len(self.tracks)
        self.get_logger().warn(
            'COLLECTING but no usable local camera target: '
            f'raw_count={raw_count}, raw_frame="{raw_frame}", '
            f'tracks={track_count}, camera_locked={self.camera_locked}, '
            f'commit_active={self.commit_active}.'
        )

    def _start_commit(self, now):
        self.commit_active = True
        self.commit_start_time = now
        self.commit_start_xy = self._base_xy_in_odom()
        self._set_phase('COMMIT')
        self.get_logger().info(
            'Camera target entered near blind zone; starting straight COMMIT: '
            f'{self.commit_max_distance:.2f} m max at {self.commit_speed:.2f} m/s.'
        )

    def _commit_distance(self, now):
        if self.commit_start_xy is not None:
            current = self._base_xy_in_odom()
            if current is not None:
                return math.hypot(
                    current[0] - self.commit_start_xy[0],
                    current[1] - self.commit_start_xy[1],
                )
        return max(0.0, now - self.commit_start_time) * self.commit_speed

    def _reset_action_state(self):
        with self.lock:
            self.active_ids = []
            self.primary_id = ''
            self.collected_ids = set()
            self.camera_locked = False
            self.last_group_points = []
            self.last_local_target = None
            self.last_local_target_time = 0.0
            self.camera_loss_start_time = None
            self.commit_active = False
            self.commit_start_time = None
            self.commit_start_xy = None

    def _execute(self, goal_handle):
        ids = []
        for item in goal_handle.request.shuttle_ids:
            shuttle_id = item.strip()
            if shuttle_id and shuttle_id not in ids:
                ids.append(shuttle_id)

        primary_id = ids[0]
        result = CollectShuttle.Result()
        feedback = CollectShuttle.Feedback()

        with self.lock:
            self.active_ids = ids
            self.primary_id = primary_id
            self.collected_ids = set()
            self.camera_locked = False
            self.last_group_points = []
            self.last_local_target = None
            self.last_local_target_time = 0.0
            self.camera_loss_start_time = None
            self.last_wait_log_time = 0.0
            self.commit_active = False
            self.commit_start_time = None
            self.commit_start_xy = None

        self._set_phase('ALIGN')
        self.get_logger().info(
            f'Collecting shuttle group {ids}; primary={primary_id}. '
            'Phases: ALIGN -> visual DRIVE -> odometry-bounded COMMIT.'
        )
        start = time.monotonic()
        period = 1.0 / max(self.control_rate, 1.0)

        try:
            while rclpy.ok():
                now = time.monotonic()

                if goal_handle.is_cancel_requested:
                    self._publish_stop()
                    self._set_phase('CANCELED')
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Collection canceled.'
                    result.collected_ids = sorted(self.collected_ids)
                    return result

                if now - start > self.timeout:
                    self._publish_stop()
                    self._set_phase('FAILED')
                    goal_handle.abort()
                    result.success = False
                    result.message = f'Collection timed out after {self.timeout:.1f} s.'
                    result.collected_ids = sorted(self.collected_ids)
                    self.get_logger().error(result.message)
                    return result

                with self.lock:
                    collected = set(self.collected_ids)

                if primary_id in collected:
                    self._publish_stop()
                    self._set_phase('SUCCESS')
                    goal_handle.succeed()
                    result.success = True
                    result.collected_ids = sorted(collected)
                    result.message = (
                        f'Primary shuttle {primary_id} collected; '
                        f'{len(collected)} shuttle(s) collected in pass.'
                    )
                    self.get_logger().info(result.message)
                    return result

                if self.commit_active:
                    elapsed = now - self.commit_start_time
                    travelled = self._commit_distance(now)
                    feedback.distance_to_target = float(
                        max(0.0, self.commit_max_distance - travelled)
                    )
                    goal_handle.publish_feedback(feedback)

                    if elapsed >= self.commit_timeout or travelled >= self.commit_max_distance:
                        self._publish_stop()
                        self._set_phase('FAILED')
                        goal_handle.abort()
                        result.success = False
                        result.collected_ids = sorted(collected)
                        result.message = (
                            'Blind-zone commit ended without physical collection: '
                            f'travelled={travelled:.3f} m, elapsed={elapsed:.2f} s.'
                        )
                        self.get_logger().error(result.message)
                        return result

                    self._publish_cmd(self.commit_speed, 0.0)
                    time.sleep(period)
                    continue

                local = self._camera_group_target_local()

                if local is not None:
                    self.last_local_target = local
                    self.last_local_target_time = now
                    self.camera_loss_start_time = None
                elif self.camera_locked and self.last_local_target is not None:
                    if self.camera_loss_start_time is None:
                        self.camera_loss_start_time = now

                    lost_for = now - self.camera_loss_start_time
                    if lost_for >= self.camera_lost_grace_time:
                        self._start_commit(now)
                        self._publish_cmd(self.commit_speed, 0.0)
                        time.sleep(period)
                        continue

                    local = self.last_local_target
                else:
                    self._publish_stop()
                    self._set_phase('ALIGN')
                    self._log_waiting_for_camera(now)
                    feedback.distance_to_target = float('nan')
                    goal_handle.publish_feedback(feedback)
                    time.sleep(period)
                    continue

                x, y, _ = local
                error_x = x - self.pickup_offset_x
                error_y = y
                distance = math.hypot(error_x, error_y)
                feedback.distance_to_target = float(distance)
                goal_handle.publish_feedback(feedback)

                heading = math.atan2(error_y, max(error_x, 1e-6))
                angular = clamp(
                    self.angular_kp * heading,
                    -self.max_angular_speed,
                    self.max_angular_speed,
                )

                if abs(heading) >= self.rotate_only_angle or error_x <= 0.0:
                    self._set_phase('ALIGN')
                    linear = 0.0
                else:
                    self._set_phase('DRIVE')
                    linear = clamp(
                        self.linear_kp * max(error_x, 0.0),
                        self.min_linear_speed,
                        self.max_linear_speed,
                    )
                    if abs(heading) >= self.heading_slowdown_angle:
                        linear *= 0.35
                    if self.camera_loss_start_time is not None:
                        linear = min(linear, self.commit_speed)

                self._publish_cmd(linear, angular)
                time.sleep(period)
        finally:
            self._publish_stop()
            self._reset_action_state()

        self._set_phase('FAILED')
        result.success = False
        result.message = 'Collection stopped because ROS shut down.'
        result.collected_ids = []
        return result

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
