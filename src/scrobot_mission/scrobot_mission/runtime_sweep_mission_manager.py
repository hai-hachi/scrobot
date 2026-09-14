#!/usr/bin/env python3

import copy
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from scrobot_mission.patrol_sweep_path import nearest_path_index
from scrobot_mission.sweep_mission_manager import MissionState, SweepMissionManager


class RuntimeSweepMissionManager(SweepMissionManager):
    """Sweep runtime wrapper focused on path-following tests.

    The base class owns the real mission behavior, including deterministic
    fixed-stop relocalization. This wrapper adds:

      * a forgiving NavigateToPose -> FollowPath handoff at sweep start;
      * an option to disable all shuttle-triggered diversions;
      * a test goal interrupt that leaves the sweep, visits a pose, returns to
        the exact departure checkpoint, then resumes the sweep;
      * ABORT / RESTART_SWEEP runtime commands for repeatable controller tests.

    Fixed-stop relocalization remains the active/default strategy.
    """

    def __init__(self):
        super().__init__()

        self.declare_parameter('join_acceptance_distance', 0.40)
        self.declare_parameter('enable_shuttle_interrupts', False)
        self.declare_parameter('enable_test_controls', True)
        self.declare_parameter('test_command_topic', '/mission/test_command')
        self.declare_parameter('test_goal_topic', '/mission/test_goal')
        self.declare_parameter('accept_rviz_goal_topic', True)
        self.declare_parameter('rviz_goal_topic', '/goal_pose')

        self.join_acceptance_distance = float(
            self.get_parameter('join_acceptance_distance').value
        )
        self.enable_shuttle_interrupts = bool(
            self.get_parameter('enable_shuttle_interrupts').value
        )
        self.enable_test_controls = bool(
            self.get_parameter('enable_test_controls').value
        )
        self.test_command_topic = str(
            self.get_parameter('test_command_topic').value
        )
        self.test_goal_topic = str(
            self.get_parameter('test_goal_topic').value
        )
        self.accept_rviz_goal_topic = bool(
            self.get_parameter('accept_rviz_goal_topic').value
        )
        self.rviz_goal_topic = str(
            self.get_parameter('rviz_goal_topic').value
        )

        self.test_goal_pose = None
        self.test_active = False
        self.test_phase = 'IDLE'
        self.restart_pending = False
        self.abort_pending = False
        self.control_cancel_reason = ''

        self.test_phase_pub = self.create_publisher(
            String, '/mission/test_phase', 10
        )
        self.create_subscription(
            String,
            self.test_command_topic,
            self._test_command_cb,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.test_goal_topic,
            self._test_goal_cb,
            10,
        )
        if self.accept_rviz_goal_topic and self.rviz_goal_topic != self.test_goal_topic:
            self.create_subscription(
                PoseStamped,
                self.rviz_goal_topic,
                self._test_goal_cb,
                10,
            )

        self._publish_test_phase('IDLE')
        self.get_logger().info(
            'Sweep test runtime ready: '
            f'fixed-stop relocalization, join_acceptance={self.join_acceptance_distance:.2f} m, '
            f'shuttle_interrupts={self.enable_shuttle_interrupts}, '
            f'commands={self.test_command_topic}, goals={self.test_goal_topic}'
            + (f' + {self.rviz_goal_topic}' if self.accept_rviz_goal_topic else '')
        )

    # ------------------------------------------------------------------
    # Shuttle test isolation
    # ------------------------------------------------------------------

    def _detections_cb(self, msg):
        if not self.enable_shuttle_interrupts:
            return
        super()._detections_cb(msg)

    # ------------------------------------------------------------------
    # Test status helpers
    # ------------------------------------------------------------------

    def _publish_test_phase(self, phase):
        self.test_phase = str(phase)
        msg = String()
        msg.data = self.test_phase
        self.test_phase_pub.publish(msg)

    def _clear_test_diversion(self):
        self.test_goal_pose = None
        self.test_active = False
        self.checkpoint_pose = None
        self.checkpoint_index = 0
        self.diversion_pending = False
        self._publish_test_phase('IDLE')

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
            'handing control directly to FollowPath.'
        )
        self._cancel_navigation('join_accepted')

    # ------------------------------------------------------------------
    # Mid-sweep NavigateToPose excursion test
    # ------------------------------------------------------------------

    def _test_goal_cb(self, msg):
        if not self.enable_test_controls:
            return
        if self.test_active:
            self.get_logger().warn('Ignoring test goal: a test excursion is already active.')
            return
        if self.state != MissionState.SWEEPING or self.follow_goal_handle is None:
            self.get_logger().warn(
                f'Ignoring test goal while mission state is {self.state.name}; '
                'send the goal while SWEEPING.'
            )
            return
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            self.get_logger().warn(
                f'Ignoring test goal in frame {msg.header.frame_id!r}; '
                f'expected {self.frame_id!r}.'
            )
            return

        pose_info = self._robot_pose()
        if pose_info is None:
            self.get_logger().warn('Cannot save sweep checkpoint: map robot pose unavailable.')
            return

        pose, _ = pose_info
        self.checkpoint_pose = copy.deepcopy(pose)
        self.checkpoint_index = nearest_path_index(
            self.sweep_points,
            pose.position.x,
            pose.position.y,
            self.current_path_index,
        )
        self.current_path_index = self.checkpoint_index
        self.test_goal_pose = copy.deepcopy(msg.pose)
        self.test_active = True
        self.diversion_pending = True
        self._publish_test_phase('LEAVING_SWEEP')

        self.get_logger().info(
            f'Test excursion requested: saved checkpoint index={self.checkpoint_index}, '
            f'x={pose.position.x:.2f}, y={pose.position.y:.2f}; '
            f'goal=({self.test_goal_pose.position.x:.2f}, '
            f'{self.test_goal_pose.position.y:.2f}).'
        )
        self._cancel_follow('test_goal')

    def _start_test_outbound(self):
        if self.test_goal_pose is None or self.checkpoint_pose is None:
            self._fail('Test excursion lost its goal or checkpoint.')
            return
        # Reuse RETURN_TO_SWEEP mission state because the base enum intentionally
        # stays mission-focused. /mission/test_phase identifies outbound/return.
        self._set_state(MissionState.RETURN_TO_SWEEP)
        self._publish_test_phase('GO_TO_TEST_GOAL')
        self._send_navigation(self.test_goal_pose, 'test_goal')

    def _start_test_return(self):
        if self.checkpoint_pose is None:
            self._fail('Test excursion has no saved sweep checkpoint.')
            return
        self._publish_test_phase('RETURN_TO_CHECKPOINT')
        self._send_navigation(self.checkpoint_pose, 'test_return')

    def _finish_test_return(self):
        self.current_path_index = self.checkpoint_index
        self.get_logger().info(
            f'Returned to test checkpoint index={self.checkpoint_index}; resuming FollowPath.'
        )
        self.test_active = False
        self.test_goal_pose = None
        self.checkpoint_pose = None
        self.diversion_pending = False
        self._publish_test_phase('RESUME_SWEEP')
        self._start_sweep_follow()
        self._publish_test_phase('IDLE')

    # ------------------------------------------------------------------
    # Runtime commands
    # ------------------------------------------------------------------

    def _test_command_cb(self, msg):
        if not self.enable_test_controls:
            return
        command = str(msg.data).strip().upper()
        if command in ('RESTART', 'RESTART_SWEEP', 'ABORT_RESTART'):
            self._request_restart()
        elif command == 'ABORT':
            self._request_abort()
        else:
            self.get_logger().warn(
                f'Unknown test command {command!r}. '
                'Use RESTART_SWEEP, ABORT_RESTART, or ABORT.'
            )

    def _request_restart(self):
        if not self.sweep_points:
            self.get_logger().warn(
                'Cannot restart sweep yet: sweep geometry has not been generated.'
            )
            return

        self.get_logger().info('RESTART_SWEEP requested: canceling current motion and restarting at path index 0.')
        self.restart_pending = True
        self.abort_pending = False
        self.control_cancel_reason = 'restart_sweep'
        self.test_active = False
        self.test_goal_pose = None
        self.checkpoint_pose = None
        self.diversion_pending = False
        self.active_relocalization_stop = None
        self._publish_test_phase('RESTARTING')

        if self.follow_goal_handle is not None:
            self._cancel_follow('restart_sweep')
            return
        if self.navigate_goal_handle is not None:
            self._cancel_navigation('restart_sweep')
            return
        if self.spin_goal_handle is not None:
            self.spin_goal_handle.cancel_goal_async()
            return
        if self.relocalize_goal_handle is not None:
            self.relocalize_goal_handle.cancel_goal_async()
            return
        self._execute_restart()

    def _request_abort(self):
        self.get_logger().info('ABORT requested: canceling current mission motion.')
        self.abort_pending = True
        self.restart_pending = False
        self.control_cancel_reason = 'abort'
        self.test_active = False
        self.test_goal_pose = None
        self.diversion_pending = False
        self._publish_test_phase('ABORTING')

        if self.follow_goal_handle is not None:
            self._cancel_follow('abort')
            return
        if self.navigate_goal_handle is not None:
            self._cancel_navigation('abort')
            return
        if self.spin_goal_handle is not None:
            self.spin_goal_handle.cancel_goal_async()
            return
        if self.relocalize_goal_handle is not None:
            self.relocalize_goal_handle.cancel_goal_async()
            return
        self._finish_abort()

    def _execute_restart(self):
        self.restart_pending = False
        self.abort_pending = False
        self.control_cancel_reason = ''
        self.current_path_index = 0
        self.checkpoint_index = 0
        self.checkpoint_pose = None
        self.diversion_pending = False
        self.active_relocalization_stop = None
        self._reset_relocalization_distance()
        self._publish_test_phase('JOINING_SWEEP_START')
        self._join_sweep()

    def _finish_abort(self):
        self.restart_pending = False
        self.abort_pending = False
        self.control_cancel_reason = ''
        self.checkpoint_pose = None
        self.diversion_pending = False
        self.active_relocalization_stop = None
        self._set_state(MissionState.IDLE)
        self._publish_test_phase('ABORTED')

    def _finish_control_cancel(self):
        if self.restart_pending:
            self._execute_restart()
        elif self.abort_pending:
            self._finish_abort()

    # ------------------------------------------------------------------
    # Action result overrides for test controls
    # ------------------------------------------------------------------

    def _follow_result(self, future):
        wrapped = future.result()
        reason = self.follow_cancel_reason

        if wrapped.status == GoalStatus.STATUS_CANCELED and reason in (
            'test_goal', 'restart_sweep', 'abort'
        ):
            self.follow_goal_handle = None
            self.follow_cancel_reason = ''
            if reason == 'test_goal':
                self._start_test_outbound()
            else:
                self._finish_control_cancel()
            return

        super()._follow_result(future)

    def _navigation_result(self, future, purpose):
        wrapped = future.result()
        self.navigate_goal_handle = None
        reason = self.nav_cancel_reason
        self.nav_cancel_reason = ''

        if wrapped.status == GoalStatus.STATUS_CANCELED:
            if reason == 'join_accepted' and purpose == 'join':
                self.current_path_index = 0
                self._publish_test_phase('IDLE')
                self._start_sweep_follow()
                return
            if reason == 'diversion':
                self._start_local_collect()
                return
            if reason in ('restart_sweep', 'abort'):
                self._finish_control_cancel()
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
            self._publish_test_phase('IDLE')
            self._start_sweep_follow()
        elif purpose == 'return':
            self.current_path_index = self.checkpoint_index
            self.checkpoint_pose = None
            self.diversion_pending = False
            self._start_sweep_follow()
        elif purpose == 'test_goal':
            self.get_logger().info('Reached test goal; returning to saved sweep checkpoint.')
            self._start_test_return()
        elif purpose == 'test_return':
            self._finish_test_return()
        else:
            self._fail(f'Unknown navigation purpose {purpose!r}.')

    def _spin_result(self, future, purpose):
        wrapped = future.result()
        if (
            wrapped.status == GoalStatus.STATUS_CANCELED
            and self.control_cancel_reason in ('restart_sweep', 'abort')
        ):
            self.spin_goal_handle = None
            self._finish_control_cancel()
            return
        super()._spin_result(future, purpose)

    def _relocalize_result(self, future, initial):
        wrapped = future.result()
        if (
            wrapped.status == GoalStatus.STATUS_CANCELED
            and self.control_cancel_reason in ('restart_sweep', 'abort')
        ):
            self.relocalize_goal_handle = None
            self._finish_control_cancel()
            return
        super()._relocalize_result(future, initial)

    # ------------------------------------------------------------------
    # Runtime tick
    # ------------------------------------------------------------------

    def _tick(self):
        if self.state == MissionState.JOIN_SWEEP:
            self._maybe_accept_join()
            return

        # During a NavigateToPose test excursion, do not project the off-sweep
        # robot pose forward along the sweep path. The hard checkpoint owns
        # coverage progress until the return completes.
        if self.test_active:
            return

        if self.state == MissionState.SWEEPING:
            self._update_sweep_progress()
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
