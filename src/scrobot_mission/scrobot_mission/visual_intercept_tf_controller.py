#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import TransformException
from scrobot_interfaces.action import CollectShuttle

from scrobot_mission.visual_intercept_controller import (
    VisualInterceptController,
    clamp,
    detection_position,
    transform_point,
)


class VisualInterceptTfController(VisualInterceptController):
    """Camera-guided intercept with TF-based blind collector overrun.

    Once the target has been seen, losing it from the camera is expected near
    the collector. The last observed point is kept in the tracking frame and
    continuously transformed into collector_link. The maneuver finishes only
    after that virtual shuttle point has moved slightly behind the collector
    centerline, guaranteeing geometric overrun instead of relying on a timer.
    """

    def __init__(self):
        super().__init__()
        self.declare_parameter('collector_frame', 'collector_link')
        self.declare_parameter('collector_overrun_distance', 0.05)
        self.declare_parameter('collector_lateral_tolerance', 0.13)
        self.declare_parameter('blind_tf_timeout', 3.0)
        self.declare_parameter('acquisition_scan_speed', 0.18)

        self.collector_frame = str(self.get_parameter('collector_frame').value)
        self.collector_overrun_distance = float(
            self.get_parameter('collector_overrun_distance').value
        )
        self.collector_lateral_tolerance = float(
            self.get_parameter('collector_lateral_tolerance').value
        )
        self.blind_tf_timeout = float(
            self.get_parameter('blind_tf_timeout').value
        )
        self.acquisition_scan_speed = float(
            self.get_parameter('acquisition_scan_speed').value
        )

        self.get_logger().info(
            'TF overrun enabled: camera loss near pickup is expected; '
            'collector_link is driven past the last observed shuttle pose.'
        )

    def _target_tracking_point(self, track_id):
        with self.lock:
            detection = self.visible.get(track_id)
        if detection is None:
            return None

        point = detection_position(detection)
        source = detection.header.frame_id or self.tracking_frame
        if source == self.tracking_frame:
            return point

        try:
            tf = self.tf_buffer.lookup_transform(
                self.tracking_frame,
                source,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException:
            return None
        return transform_point(tf, point)

    def _tracking_point_in_frame(self, point, target_frame):
        if target_frame == self.tracking_frame:
            return point
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame,
                self.tracking_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException:
            return None
        return transform_point(tf, point)

    def _execute(self, goal_handle):
        track_id = next(x.strip() for x in goal_handle.request.shuttle_ids if x.strip())
        result = CollectShuttle.Result()
        feedback = CollectShuttle.Feedback()

        with self.lock:
            self.active = True

        try:
            # Search at the Nav2 staging pose instead of immediately declaring
            # the target missed if it is just outside the current camera view.
            acquire_deadline = time.monotonic() + self.acquisition_timeout
            last_tracking_point = None
            scan_sign = 1.0
            while rclpy.ok() and time.monotonic() < acquire_deadline:
                if goal_handle.is_cancel_requested:
                    self._stop()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Visual intercept canceled while acquiring target.'
                    return result

                last_tracking_point = self._target_tracking_point(track_id)
                if last_tracking_point is not None:
                    break

                self._publish_cmd(0.0, scan_sign * self.acquisition_scan_speed)
                # Reverse search direction halfway through acquisition.
                remaining = acquire_deadline - time.monotonic()
                if remaining < 0.5 * self.acquisition_timeout:
                    scan_sign = -1.0
                time.sleep(1.0 / max(self.control_rate, 1.0))

            self._stop()
            if last_tracking_point is None:
                goal_handle.succeed()
                result.success = False
                result.message = f'Shuttle {track_id} not visible after staging scan.'
                return result

            self.get_logger().info(
                f'Visual intercept acquired shuttle {track_id}; live camera steering active.'
            )

            started = time.monotonic()
            last_seen = started
            blind_started = None
            period = 1.0 / max(self.control_rate, 1.0)

            while rclpy.ok() and time.monotonic() - started < self.intercept_timeout:
                if goal_handle.is_cancel_requested:
                    self._stop()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Visual intercept canceled.'
                    return result

                live_tracking = self._target_tracking_point(track_id)
                if live_tracking is not None:
                    last_tracking_point = live_tracking
                    last_seen = time.monotonic()
                    blind_started = None

                target_base = self._tracking_point_in_frame(
                    last_tracking_point,
                    self.base_frame,
                )
                target_collector = self._tracking_point_in_frame(
                    last_tracking_point,
                    self.collector_frame,
                )

                if target_base is None or target_collector is None:
                    self._stop()
                    time.sleep(period)
                    continue

                bx, by, _ = target_base
                cx, cy, _ = target_collector
                feedback.distance_to_target = float(math.hypot(cx, cy))
                goal_handle.publish_feedback(feedback)

                # The target is guaranteed to have passed beneath the collector
                # once it is slightly behind collector_link in X and still
                # inside the collector's lateral corridor.
                if (
                    cx <= -self.collector_overrun_distance
                    and abs(cy) <= self.collector_lateral_tolerance
                ):
                    self._stop()
                    goal_handle.succeed()
                    result.success = True
                    result.collected_ids = [track_id]
                    result.message = (
                        f'Collector TF overrun complete for shuttle {track_id}: '
                        f'collector-relative=({cx:.3f}, {cy:.3f}) m.'
                    )
                    self.get_logger().info(result.message)
                    return result

                if live_tracking is None:
                    # Camera loss is expected when the shuttle moves below the
                    # optical field. Continue from the last map-frame point.
                    if blind_started is None:
                        blind_started = time.monotonic()
                        self.get_logger().info(
                            f'Shuttle {track_id} left camera; continuing by TF to collector_link.'
                        )
                    if time.monotonic() - blind_started > self.blind_tf_timeout:
                        self._stop()
                        goal_handle.succeed()
                        result.success = False
                        result.message = (
                            f'TF overrun timed out for shuttle {track_id}; '
                            f'collector-relative=({cx:.3f}, {cy:.3f}) m.'
                        )
                        return result

                # Use the live / last-known local target to keep the centerline
                # aimed at the shuttle. Once vision is gone, speed is capped low.
                heading = math.atan2(by, max(bx, 1e-4))
                angular = clamp(
                    self.angular_kp * heading,
                    -self.max_angular_speed,
                    self.max_angular_speed,
                )
                forward_error = max(0.0, bx - self.pickup_offset_x)
                linear = clamp(
                    self.linear_kp * forward_error,
                    self.min_linear_speed,
                    self.max_linear_speed,
                )
                if live_tracking is None:
                    linear = min(linear, 0.14)
                if abs(heading) > self.heading_slowdown_angle:
                    linear *= 0.45

                self._publish_cmd(linear, angular)
                time.sleep(period)

            self._stop()
            goal_handle.succeed()
            result.success = False
            result.message = f'Visual/TF intercept timed out for shuttle {track_id}.'
            return result
        finally:
            self._stop()
            with self.lock:
                self.active = False


def main(args=None):
    rclpy.init(args=args)
    node = VisualInterceptTfController()
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
