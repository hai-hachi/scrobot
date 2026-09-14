#!/usr/bin/env python3

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformException

from scrobot_mission.patrol_sweep_path import nearest_path_index
from scrobot_mission.sweep_mission_manager import MissionState, SweepMissionManager


class RuntimeSweepMissionManager(SweepMissionManager):
    """Runtime sweep manager with forgiving join and opportunistic relocalization.

    The base implementation retains the fixed-stop relocalization strategy so it
    can be restored later. This wrapper temporarily defaults to opportunistic
    relocalization: while SWEEPING, a valid visible AprilTag candidate inside a
    configurable range cancels FollowPath, runs /relocalize while stationary,
    then resumes the same sweep progress.

    JOIN_SWEEP is also intentionally less strict than NavigateToPose's exact
    terminal convergence. Once the robot is close enough to the first sweep
    point, the join goal is canceled and FollowPath takes over. This prevents a
    differential-drive robot from circling a single precise start pose.
    """

    def __init__(self):
        super().__init__()

        # Join handoff: NavigateToPose only needs to get us near the sweep.
        self.declare_parameter('join_acceptance_distance', 0.40)

        # Keep both strategies available. Current branch default is opportunistic.
        self.declare_parameter('relocalization_mode', 'opportunistic')
        self.declare_parameter(
            'active_tags_topic', '/global_localization/active_tags'
        )
        self.declare_parameter('observed_tag_prefix', 'observed_tag_')
        self.declare_parameter('opportunistic_max_tag_distance', 3.50)
        self.declare_parameter('opportunistic_min_travel_distance', 1.00)
        self.declare_parameter('opportunistic_retry_cooldown', 2.00)

        self.join_acceptance_distance = float(
            self.get_parameter('join_acceptance_distance').value
        )
        self.relocalization_mode = str(
            self.get_parameter('relocalization_mode').value
        ).strip().lower()
        self.active_tags_topic = str(
            self.get_parameter('active_tags_topic').value
        )
        self.observed_tag_prefix = str(
            self.get_parameter('observed_tag_prefix').value
        )
        self.opportunistic_max_tag_distance = float(
            self.get_parameter('opportunistic_max_tag_distance').value
        )
        self.opportunistic_min_travel_distance = float(
            self.get_parameter('opportunistic_min_travel_distance').value
        )
        self.opportunistic_retry_cooldown = float(
            self.get_parameter('opportunistic_retry_cooldown').value
        )

        if self.relocalization_mode not in ('opportunistic', 'fixed_stops'):
            self.get_logger().warn(
                f'Unknown relocalization_mode={self.relocalization_mode!r}; '
                'using opportunistic.'
            )
            self.relocalization_mode = 'opportunistic'

        self.opportunistic_tag_id = -1
        self.last_opportunistic_attempt = -1e9

        self.create_subscription(
            Int32MultiArray,
            self.active_tags_topic,
            self._active_tags_cb,
            10,
        )

        self.get_logger().info(
            'Runtime sweep overrides active: '
            f'join_acceptance={self.join_acceptance_distance:.2f} m, '
            f'relocalization_mode={self.relocalization_mode}, '
            f'opportunistic_range={self.opportunistic_max_tag_distance:.2f} m.'
        )

    # ------------------------------------------------------------------
    # Forgiving join-to-sweep handoff
    # ------------------------------------------------------------------

    def _maybe_accept_join(self):
        if (
            self.state != MissionState.JOIN_SWEEP
            or self.navigate_goal_handle is None
            or not self.sweep_points
        ):
            return

        pose_info = self._robot_pose()
        if pose_info is None:
            return
        pose, _ = pose_info
        start = self.sweep_points[0]
        distance = math.hypot(
            pose.position.x - start.x,
            pose.position.y - start.y,
        )
        if distance > self.join_acceptance_distance:
            return

        self.get_logger().info(
            f'Join accepted at {distance:.2f} m from sweep start; '
            'handing control to FollowPath instead of converging on one exact pose.'
        )
        self._cancel_navigation('join_accepted')

    def _navigation_result(self, future, purpose):
        wrapped = future.result()
        self.navigate_goal_handle = None
        reason = self.nav_cancel_reason
        self.nav_cancel_reason = ''

        if wrapped.status == GoalStatus.STATUS_CANCELED:
            if reason == 'diversion':
                self._start_local_collect()
                return
            if reason == 'join_accepted' and purpose == 'join':
                self.current_path_index = 0
                self._start_sweep_follow()
                return
            self._fail(
                f'Navigation canceled unexpectedly ({purpose}), reason={reason!r}.'
            )
            return

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._fail(f'Navigation failed ({purpose}), status={wrapped.status}.')
            return

        if purpose == 'join':
            self.current_path_index = 0
            self._start_sweep_follow()
        elif purpose == 'return':
            self.current_path_index = self.checkpoint_index
            self.checkpoint_pose = None
            self.diversion_pending = False
            self._start_sweep_follow()

    # ------------------------------------------------------------------
    # Opportunistic relocalization
    # ------------------------------------------------------------------

    def _start_sweep_follow(self):
        if self.relocalization_mode != 'opportunistic':
            super()._start_sweep_follow()
            return

        self._set_state(MissionState.SWEEPING)
        self.diversion_pending = False
        self.active_relocalization_stop = None
        self._send_follow(None, 'sweep_end', None)

    def _tag_distance_from_robot(self, tag_id):
        frame = f'{self.observed_tag_prefix}{tag_id}'
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException:
            return None

        t = tf.translation
        return math.sqrt(float(t.x * t.x + t.y * t.y + t.z * t.z))

    def _active_tags_cb(self, msg):
        if self.relocalization_mode != 'opportunistic':
            return
        if (
            self.state != MissionState.SWEEPING
            or self.diversion_pending
            or self.follow_goal_handle is None
            or self.relocalize_goal_handle is not None
            or self.opportunistic_tag_id >= 0
        ):
            return

        # Prevent immediate repeated corrections while the same tag remains in
        # view. After a successful correction, odom travel resets to zero.
        if self.distance_since_relocalize < self.opportunistic_min_travel_distance:
            return
        if (
            time.monotonic() - self.last_opportunistic_attempt
            < self.opportunistic_retry_cooldown
        ):
            return

        candidates = []
        for raw_id in msg.data:
            tag_id = int(raw_id)
            if tag_id not in (0, 1, 2, 3):
                continue
            distance = self._tag_distance_from_robot(tag_id)
            if distance is None or distance > self.opportunistic_max_tag_distance:
                continue
            candidates.append((distance, tag_id))

        if not candidates:
            return

        distance, tag_id = min(candidates)

        # Save progress immediately before canceling the sweep. The map->odom
        # correction may move the robot pose in map, but coverage progress must
        # remain where the sweep was actually interrupted.
        pose_info = self._robot_pose()
        if pose_info is not None:
            pose, _ = pose_info
            self.current_path_index = nearest_path_index(
                self.sweep_points,
                pose.position.x,
                pose.position.y,
                self.current_path_index,
            )

        self.opportunistic_tag_id = tag_id
        self.last_opportunistic_attempt = time.monotonic()
        self.get_logger().info(
            f'Good tag candidate {tag_id} visible at {distance:.2f} m; '
            f'pausing sweep at path index {self.current_path_index} for /relocalize.'
        )
        self._cancel_follow('opportunistic_relocalization')

    def _follow_result(self, future):
        wrapped = future.result()
        self.follow_goal_handle = None
        reason = self.follow_cancel_reason
        self.follow_cancel_reason = ''

        if wrapped.status == GoalStatus.STATUS_CANCELED:
            if reason == 'diversion':
                self._start_local_collect()
                return
            if reason == 'opportunistic_relocalization':
                tag_id = self.opportunistic_tag_id
                if tag_id < 0:
                    self._start_sweep_follow()
                    return
                self._set_state(MissionState.RELOCALIZING)
                self._send_relocalize(tag_id, initial=False)
                return
            if reason == 'reschedule_relocalization':
                # Retain compatibility if fixed-stop mode is selected again.
                stop = self.active_relocalization_stop
                if stop is not None:
                    self._send_follow(stop.path_index, 'relocalize_stop', stop)
                return
            self._fail('FollowPath canceled unexpectedly.')
            return

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._fail(f'FollowPath failed, status={wrapped.status}.')
            return

        if self.follow_purpose == 'relocalize_stop':
            if self.active_relocalization_stop is not None:
                self.current_path_index = self.active_relocalization_stop.path_index
            self._begin_relocalization_stop()
        else:
            self.current_path_index = max(0, len(self.sweep_points) - 1)
            self._set_state(MissionState.COMPLETE)
            self.get_logger().info('Sweep complete.')

    def _relocalize_goal_response(self, future, initial):
        self.relocalize_goal_handle = future.result()
        if self.relocalize_goal_handle is None or not self.relocalize_goal_handle.accepted:
            if not initial and self.relocalization_mode == 'opportunistic':
                self.get_logger().warn(
                    'Opportunistic /relocalize goal was rejected; resuming sweep.'
                )
                self.relocalize_goal_handle = None
                self.opportunistic_tag_id = -1
                self._start_sweep_follow()
                return
            self._fail('Relocalize goal rejected.')
            return
        result_future = self.relocalize_goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._relocalize_result(f, initial)
        )

    def _relocalize_result(self, future, initial):
        if initial or self.relocalization_mode != 'opportunistic':
            super()._relocalize_result(future, initial)
            return

        wrapped = future.result()
        self.relocalize_goal_handle = None
        tag_id = self.opportunistic_tag_id
        self.opportunistic_tag_id = -1

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            self.get_logger().warn(
                f'Opportunistic relocalization with tag {tag_id} did not pass the '
                f'localizer quality checks: {wrapped.result.message}. Resuming sweep.'
            )
            self._start_sweep_follow()
            return

        self._reset_relocalization_distance()
        used = list(wrapped.result.used_tag_ids)
        self.get_logger().info(
            f'Opportunistic relocalization complete; requested tag={tag_id}, '
            f'used={used}, delta=({wrapped.result.delta_x:.3f}, '
            f'{wrapped.result.delta_y:.3f}, '
            f'{math.degrees(wrapped.result.delta_yaw):.2f} deg). '
            f'Resuming sweep at path index {self.current_path_index}.'
        )
        self._start_sweep_follow()

    # ------------------------------------------------------------------
    # Runtime tick
    # ------------------------------------------------------------------

    def _tick(self):
        if self.state == MissionState.JOIN_SWEEP:
            self._maybe_accept_join()
            return

        if self.state != MissionState.SWEEPING:
            return

        self._update_sweep_progress()
        if self.relocalization_mode == 'fixed_stops':
            self._maybe_reschedule_for_relocalization()


def main(args=None):
    rclpy.init(args=args)
    node = RuntimeSweepMissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
