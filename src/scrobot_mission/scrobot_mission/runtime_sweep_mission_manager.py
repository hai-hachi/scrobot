#!/usr/bin/env python3

import copy
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from scrobot_mission.patrol_sweep_path import nearest_path_index
from scrobot_mission.sweep_mission_manager import MissionState, SweepMissionManager


class RuntimeSweepMissionManager(SweepMissionManager):
    """Sweep runtime wrapper focused on path-following tests.

    The base class owns the real mission behavior. This wrapper adds:

      * a forgiving NavigateToPose -> FollowPath handoff at sweep start;
      * an option to disable all shuttle-triggered diversions;
      * a test goal interrupt that leaves the sweep, visits a pose, returns to
        the exact departure XY checkpoint, aligns to the sweep tangent heading,
        then resumes the sweep;
      * unconditional fixed-stop relocalization at every generated station;
      * ABORT / RESTART_SWEEP runtime commands for repeatable controller tests;
      * manual-mode pause/resume that cancels active autonomous actions and
        continues from the saved mission context when AUTO is restored.

    There is intentionally no traveled-distance threshold for fixed-stop
    relocalization in this runtime manager. Every fixed station is visited once
    in path order on each sweep run.
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
        self.declare_parameter('manual_mode_topic', '/control/manual_mode')
        self.declare_parameter('manual_resume_return_distance', 0.15)
        self.declare_parameter('manual_resume_return_yaw_deg', 10.0)
        self.declare_parameter('tag_recovery_max_attempts', 3)

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
        self.manual_mode_topic = str(
            self.get_parameter('manual_mode_topic').value
        )
        self.manual_resume_return_distance = max(
            0.0,
            float(self.get_parameter('manual_resume_return_distance').value),
        )
        self.manual_resume_return_yaw = math.radians(
            max(
                0.0,
                float(self.get_parameter('manual_resume_return_yaw_deg').value),
            )
        )
        self.tag_recovery_max_attempts = max(
            1, int(self.get_parameter('tag_recovery_max_attempts').value)
        )

        self.last_good_localization_pose = None
        self.tag_recovery_attempts = 0

        self.test_goal_pose = None
        self.test_active = False
        self.test_phase = 'IDLE'
        self.restart_pending = False
        self.abort_pending = False
        self.control_cancel_reason = ''

        # Manual override state. The control package owns AUTO/MANUAL mode;
        # the mission manager only reacts by pausing/resuming autonomous actions.
        self.manual_paused = False
        self.manual_resume_state = MissionState.IDLE
        self.manual_nav_pose = None
        self.manual_nav_purpose = ''
        self.manual_spin_target_yaw = None
        self.manual_spin_purpose = ''
        self.manual_relocalize_tag_id = None
        self.manual_relocalize_initial = False
        self.manual_checkpoint_pose = None
        self.manual_checkpoint_index = 0

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

        manual_state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            self.manual_mode_topic,
            self._manual_mode_cb,
            manual_state_qos,
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
            'fixed-stop relocalization at every station, '
            f'join_acceptance={self.join_acceptance_distance:.2f} m, '
            f'shuttle_interrupts={self.enable_shuttle_interrupts}, '
            f'commands={self.test_command_topic}, goals={self.test_goal_topic}'
            + (f' + {self.rviz_goal_topic}' if self.accept_rviz_goal_topic else '')
        )

    # ------------------------------------------------------------------
    # Manual override integration
    # ------------------------------------------------------------------

    def _manual_mode_cb(self, msg):
        if bool(msg.data):
            self._request_manual_pause()
        else:
            self._resume_from_manual()

    def _request_manual_pause(self):
        if self.manual_paused:
            return

        self.manual_paused = True
        self.manual_resume_state = self.state
        self.control_cancel_reason = 'manual_pause'

        # Save where autonomous control was interrupted. For phases that assume
        # the robot is at a specific place (sweep, tag stop, local collection),
        # AUTO first returns here after the operator has driven elsewhere.
        self.manual_checkpoint_pose = None
        pose_info = self._robot_pose()
        if pose_info is not None:
            pose, _ = pose_info
            self.manual_checkpoint_pose = copy.deepcopy(pose)

        if self.state == MissionState.SWEEPING:
            self._update_sweep_progress()
            self.manual_checkpoint_index = self.current_path_index
            if self.manual_checkpoint_pose is not None and self.sweep_points:
                self.manual_checkpoint_pose = self._checkpoint_with_sweep_heading(
                    self.manual_checkpoint_pose,
                    self.manual_checkpoint_index,
                )

        self.get_logger().info(
            f'MANUAL requested while mission state={self.state.name}; '
            'canceling autonomous action and preserving mission context.'
        )

        if self.follow_goal_handle is not None:
            self._cancel_follow('manual_pause')
            return
        if self.navigate_goal_handle is not None:
            self._cancel_navigation('manual_pause')
            return
        if self.spin_goal_handle is not None:
            self.spin_goal_handle.cancel_goal_async()
            return
        if self.relocalize_goal_handle is not None:
            self.relocalize_goal_handle.cancel_goal_async()
            return
        if self.collect_goal_handle is not None:
            self.collect_goal_handle.cancel_goal_async()
            return
        if self.approach_goal_handle is not None:
            self.approach_goal_handle.cancel_goal_async()
            return

        self._manual_pause_complete()

    def _manual_pause_complete(self):
        self.control_cancel_reason = ''
        self._set_state(MissionState.PAUSED)
        self.get_logger().info(
            'Autonomous mission paused. Manual command path owns the drive base.'
        )

    def _resume_from_manual(self):
        if not self.manual_paused:
            return

        resume_state = self.manual_resume_state
        self.manual_paused = False
        self.control_cancel_reason = ''

        self.get_logger().info(
            f'AUTO requested; resuming mission from {resume_state.name}.'
        )

        if resume_state in (
            MissionState.IDLE,
            MissionState.COMPLETE,
            MissionState.ERROR,
        ):
            self._set_state(resume_state)
            return

        if resume_state == MissionState.INITIAL_TAG_APPROACH:
            self._set_state(MissionState.INITIAL_TAG_APPROACH)
            self._send_initial_approach()
            return

        if resume_state == MissionState.INITIAL_RELOCALIZATION:
            self._set_state(MissionState.INITIAL_RELOCALIZATION)
            tag_id = (
                self.start_tag_id
                if self.manual_relocalize_tag_id is None
                else self.manual_relocalize_tag_id
            )
            self._send_relocalize(tag_id, initial=True)
            return

        if resume_state == MissionState.STARTING_NAV2:
            self._start_nav2()
            return

        if resume_state == MissionState.JOIN_SWEEP:
            self._join_sweep()
            return

        if resume_state in (
            MissionState.SWEEPING,
            MissionState.TURN_TO_TAG,
            MissionState.RELOCALIZING,
            MissionState.RESTORE_SWEEP_HEADING,
            MissionState.LOCAL_COLLECT,
        ) and self.manual_checkpoint_pose is not None:
            if self._manual_checkpoint_requires_return():
                self.get_logger().info(
                    'Returning to the manual-interrupt checkpoint before '
                    f'resuming {resume_state.name}.'
                )
                self._set_state(MissionState.RETURN_TO_SWEEP)
                self._send_navigation(
                    copy.deepcopy(self.manual_checkpoint_pose),
                    'manual_resume_checkpoint',
                )
                return

            self.get_logger().info(
                'Manual displacement stayed inside the resume tolerance; '
                f'resuming {resume_state.name} without a Nav2 return.'
            )

        if self._resume_saved_manual_phase():
            return

        if resume_state in (
            MissionState.RETURN_TO_SWEEP,
            MissionState.TAG_RECOVERY_RETURN,
        ):
            if self.manual_nav_pose is None or not self.manual_nav_purpose:
                self._fail(
                    'Cannot resume paused navigation: target pose/purpose was not preserved.'
                )
                return
            self._set_state(resume_state)
            self._send_navigation(
                copy.deepcopy(self.manual_nav_pose),
                self.manual_nav_purpose,
            )
            return

        self._fail(f'Unsupported manual resume state: {resume_state.name}')

    @staticmethod
    def _yaw_from_pose(pose):
        q = pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def _angle_error(a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def _manual_checkpoint_requires_return(self):
        if self.manual_checkpoint_pose is None:
            return False

        pose_info = self._robot_pose()
        if pose_info is None:
            # Be conservative if TF is temporarily unavailable.
            return True

        current_pose, current_yaw = pose_info
        checkpoint = self.manual_checkpoint_pose

        dx = current_pose.pose.position.x - checkpoint.pose.position.x
        dy = current_pose.pose.position.y - checkpoint.pose.position.y
        distance = math.hypot(dx, dy)

        checkpoint_yaw = self._yaw_from_pose(checkpoint)
        yaw_error = abs(self._angle_error(current_yaw, checkpoint_yaw))

        self.get_logger().info(
            'Manual resume displacement: '
            f'distance={distance:.3f} m, '
            f'yaw={math.degrees(yaw_error):.1f} deg '
            f'(return thresholds '
            f'{self.manual_resume_return_distance:.3f} m / '
            f'{math.degrees(self.manual_resume_return_yaw):.1f} deg).'
        )

        return (
            distance > self.manual_resume_return_distance
            or yaw_error > self.manual_resume_return_yaw
        )

    def _resume_saved_manual_phase(self):
        resume_state = self.manual_resume_state

        if resume_state == MissionState.SWEEPING:
            self.current_path_index = self.manual_checkpoint_index
            self._start_sweep_follow()
            return True

        if resume_state in (
            MissionState.TURN_TO_TAG,
            MissionState.RESTORE_SWEEP_HEADING,
        ):
            if self.manual_spin_target_yaw is None:
                self._fail('Cannot resume paused turn: target yaw was not preserved.')
                return True
            self._set_state(resume_state)
            self._send_spin_to_absolute(
                self.manual_spin_target_yaw,
                self.manual_spin_purpose,
            )
            return True

        if resume_state == MissionState.RELOCALIZING:
            if self.manual_relocalize_tag_id is None:
                self._fail('Cannot resume relocalization: tag ID was not preserved.')
                return True
            self._set_state(MissionState.RELOCALIZING)
            self._send_relocalize(
                self.manual_relocalize_tag_id,
                initial=self.manual_relocalize_initial,
            )
            return True

        if resume_state == MissionState.LOCAL_COLLECT:
            self._start_local_collect()
            return True

        return False

    # Save enough context to recreate cancellable actions after MANUAL mode.

    def _send_navigation(self, pose, purpose):
        self.manual_nav_pose = copy.deepcopy(pose)
        self.manual_nav_purpose = str(purpose)
        super()._send_navigation(pose, purpose)

    def _send_spin_to_absolute(self, target_yaw, purpose):
        self.manual_spin_target_yaw = float(target_yaw)
        self.manual_spin_purpose = str(purpose)
        super()._send_spin_to_absolute(target_yaw, purpose)

    def _send_relocalize(self, tag_id, initial):
        self.manual_relocalize_tag_id = int(tag_id)
        self.manual_relocalize_initial = bool(initial)
        super()._send_relocalize(tag_id, initial)

    # Keep a late action goal from escaping into autonomous motion if MANUAL
    # was selected while the goal request was still in flight.

    def _approach_goal_response(self, future):
        super()._approach_goal_response(future)
        if self.manual_paused and self.approach_goal_handle is not None:
            self.control_cancel_reason = 'manual_pause'
            self.approach_goal_handle.cancel_goal_async()

    def _navigation_goal_response(self, future, purpose):
        super()._navigation_goal_response(future, purpose)
        if self.manual_paused and self.navigate_goal_handle is not None:
            self.control_cancel_reason = 'manual_pause'
            self._cancel_navigation('manual_pause')

    def _follow_goal_response(self, future):
        super()._follow_goal_response(future)
        if self.manual_paused and self.follow_goal_handle is not None:
            self.control_cancel_reason = 'manual_pause'
            self._cancel_follow('manual_pause')

    def _spin_goal_response(self, future, purpose):
        super()._spin_goal_response(future, purpose)
        if self.manual_paused and self.spin_goal_handle is not None:
            self.control_cancel_reason = 'manual_pause'
            self.spin_goal_handle.cancel_goal_async()

    def _relocalize_goal_response(self, future, initial):
        super()._relocalize_goal_response(future, initial)
        if self.manual_paused and self.relocalize_goal_handle is not None:
            self.control_cancel_reason = 'manual_pause'
            self.relocalize_goal_handle.cancel_goal_async()

    def _collect_goal_response(self, future):
        super()._collect_goal_response(future)
        if self.manual_paused and self.collect_goal_handle is not None:
            self.control_cancel_reason = 'manual_pause'
            self.collect_goal_handle.cancel_goal_async()

    def _start_once(self):
        # If the operator selected MANUAL before mission autostart, leave the
        # base start timer alive. It will start normally after AUTO is restored.
        if self.manual_paused:
            return
        super()._start_once()

    def _nav2_started(self, future):
        if not self.manual_paused:
            super()._nav2_started(future)
            return

        response = future.result()
        if response is None or not response.success:
            self._fail('Nav2 startup failed while entering MANUAL mode.')
            return
        self._manual_pause_complete()

    # ------------------------------------------------------------------
    # Checkpoint heading helper
    # ------------------------------------------------------------------

    def _checkpoint_with_sweep_heading(self, pose, path_index):
        """Keep the departure XY but replace yaw with the sweep tangent yaw."""
        if not self.sweep_points:
            return copy.deepcopy(pose)
        index = max(0, min(int(path_index), len(self.sweep_points) - 1))
        sweep_yaw = self.sweep_points[index].yaw
        return self._pose_xy_yaw(
            pose.position.x,
            pose.position.y,
            sweep_yaw,
        )

    # ------------------------------------------------------------------
    # Shuttle test isolation
    # ------------------------------------------------------------------

    def _detections_cb(self, msg):
        if not self.enable_shuttle_interrupts:
            return

        # Let the base manager create the hard checkpoint and begin cancellation.
        super()._detections_cb(msg)

        # A return checkpoint is positional, but its required final orientation
        # is the sweep tangent. This makes NavigateToPose stop translating once
        # XY is accepted and rotate in place to the direction FollowPath expects.
        if self.checkpoint_pose is not None and self.sweep_points:
            self.checkpoint_pose = self._checkpoint_with_sweep_heading(
                self.checkpoint_pose,
                self.checkpoint_index,
            )

    # ------------------------------------------------------------------
    # Fixed-stop relocalization: always visit the next station
    # ------------------------------------------------------------------

    def _odom_cb(self, msg):
        """Track distance only for diagnostics; it no longer gates relocalization."""
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self.last_odom_xy is not None:
            step = math.hypot(x - self.last_odom_xy[0], y - self.last_odom_xy[1])
            if step < 1.0:
                self.distance_since_relocalize += step
        self.last_odom_xy = (x, y)

    def _next_due_stop(self):
        """Return the next fixed station regardless of traveled distance."""
        for stop in self.relocalization_stops:
            if stop.path_index > self.current_path_index + 1:
                return stop
        return None

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
        self.checkpoint_index = nearest_path_index(
            self.sweep_points,
            pose.position.x,
            pose.position.y,
            self.current_path_index,
        )
        self.current_path_index = self.checkpoint_index
        self.checkpoint_pose = self._checkpoint_with_sweep_heading(
            pose,
            self.checkpoint_index,
        )
        self.test_goal_pose = copy.deepcopy(msg.pose)
        self.test_active = True
        self.diversion_pending = True
        self._publish_test_phase('LEAVING_SWEEP')

        sweep_yaw = self.sweep_points[self.checkpoint_index].yaw
        self.get_logger().info(
            f'Test excursion requested: saved checkpoint index={self.checkpoint_index}, '
            f'x={pose.position.x:.2f}, y={pose.position.y:.2f}, '
            f'return_yaw={math.degrees(sweep_yaw):.1f} deg; '
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
        self.get_logger().info(
            'Returning to checkpoint XY; goal yaw is the local sweep tangent. '
            'RPP rotate-to-heading will align before NavigateToPose succeeds.'
        )
        self._send_navigation(self.checkpoint_pose, 'test_return')

    def _finish_test_return(self):
        self.current_path_index = self.checkpoint_index
        self.get_logger().info(
            f'Returned and aligned to test checkpoint index={self.checkpoint_index}; '
            'resuming FollowPath.'
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

        self.get_logger().info(
            'RESTART_SWEEP requested: canceling current motion and restarting at path index 0.'
        )
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

        if self.manual_paused and wrapped.status != GoalStatus.STATUS_CANCELED:
            self.follow_goal_handle = None
            self.follow_cancel_reason = ''
            self._manual_pause_complete()
            return

        if wrapped.status == GoalStatus.STATUS_CANCELED and reason in (
            'test_goal', 'restart_sweep', 'abort', 'manual_pause'
        ):
            self.follow_goal_handle = None
            self.follow_cancel_reason = ''
            if reason == 'test_goal':
                self._start_test_outbound()
            elif reason == 'manual_pause':
                self._manual_pause_complete()
            else:
                self._finish_control_cancel()
            return

        super()._follow_result(future)

    def _navigation_result(self, future, purpose):
        wrapped = future.result()
        self.navigate_goal_handle = None
        reason = self.nav_cancel_reason
        self.nav_cancel_reason = ''

        if self.manual_paused and wrapped.status != GoalStatus.STATUS_CANCELED:
            self._manual_pause_complete()
            return

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
            if reason == 'manual_pause':
                self._manual_pause_complete()
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
        elif purpose == 'tag_recovery_return':
            self.get_logger().info(
                'Reached last successful localization pose; retrying AprilTag search.'
            )
            self._set_state(MissionState.INITIAL_TAG_APPROACH)
            self._send_initial_approach()
        elif purpose == 'manual_resume_checkpoint':
            self.get_logger().info(
                'Manual-interrupt checkpoint reached; resuming saved autonomous phase.'
            )
            if not self._resume_saved_manual_phase():
                self._fail(
                    f'No resume handler for {self.manual_resume_state.name}.'
                )
        else:
            self._fail(f'Unknown navigation purpose {purpose!r}.')

    def _spin_result(self, future, purpose):
        wrapped = future.result()
        if self.manual_paused and wrapped.status != GoalStatus.STATUS_CANCELED:
            self.spin_goal_handle = None
            self._manual_pause_complete()
            return
        if (
            wrapped.status == GoalStatus.STATUS_CANCELED
            and self.control_cancel_reason in ('restart_sweep', 'abort', 'manual_pause')
        ):
            self.spin_goal_handle = None
            if self.control_cancel_reason == 'manual_pause':
                self._manual_pause_complete()
            else:
                self._finish_control_cancel()
            return
        super()._spin_result(future, purpose)

    def _relocalize_result(self, future, initial):
        wrapped = future.result()
        if self.manual_paused and wrapped.status != GoalStatus.STATUS_CANCELED:
            self.relocalize_goal_handle = None
            self._manual_pause_complete()
            return
        if (
            wrapped.status == GoalStatus.STATUS_CANCELED
            and self.control_cancel_reason in ('restart_sweep', 'abort', 'manual_pause')
        ):
            self.relocalize_goal_handle = None
            if self.control_cancel_reason == 'manual_pause':
                self._manual_pause_complete()
            else:
                self._finish_control_cancel()
            return

        if (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped.result.success
        ):
            pose_info = self._robot_pose()
            if pose_info is not None:
                pose, _ = pose_info
                self.last_good_localization_pose = copy.deepcopy(pose)
            self.tag_recovery_attempts = 0

        super()._relocalize_result(future, initial)

    def _approach_result(self, future):
        wrapped = future.result()

        if self.manual_paused and wrapped.status != GoalStatus.STATUS_CANCELED:
            self.approach_goal_handle = None
            self._manual_pause_complete()
            return
        if (
            wrapped.status == GoalStatus.STATUS_CANCELED
            and self.control_cancel_reason == 'manual_pause'
        ):
            self.approach_goal_handle = None
            self._manual_pause_complete()
            return

        if (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped.result.success
        ):
            self.tag_recovery_attempts = 0
            super()._approach_result(future)
            return

        # Recovery is only for the explicit no-tag result produced after the
        # measured full-rotation search. Approach/alignment failures remain real
        # mission failures and are handled by the base mission manager.
        if 'No acceptable AprilTag found after a full' not in wrapped.result.message:
            super()._approach_result(future)
            return

        self.approach_goal_handle = None
        self.tag_recovery_attempts += 1

        if self.tag_recovery_attempts >= self.tag_recovery_max_attempts:
            self._fail(
                'Initial AprilTag acquisition failed after '
                f'{self.tag_recovery_attempts} full-search attempt(s): '
                f'{wrapped.result.message}'
            )
            return

        if self.last_good_localization_pose is not None:
            self.get_logger().warn(
                'AprilTag search completed one full rotation without a usable tag. '
                'Returning to the last successful localization pose before retrying '
                f'(attempt {self.tag_recovery_attempts + 1}/'
                f'{self.tag_recovery_max_attempts}).'
            )
            self._set_state(MissionState.TAG_RECOVERY_RETURN)
            self._send_navigation(
                copy.deepcopy(self.last_good_localization_pose),
                'tag_recovery_return',
            )
            return

        self.get_logger().warn(
            'AprilTag search completed one full rotation and no previous '
            'localization pose exists. Retrying in place '
            f'(attempt {self.tag_recovery_attempts + 1}/'
            f'{self.tag_recovery_max_attempts}).'
        )
        self._set_state(MissionState.INITIAL_TAG_APPROACH)
        self._send_initial_approach()

    def _collect_result(self, future):
        wrapped = future.result()
        if self.manual_paused and wrapped.status != GoalStatus.STATUS_CANCELED:
            self.collect_goal_handle = None
            self._manual_pause_complete()
            return
        if (
            wrapped.status == GoalStatus.STATUS_CANCELED
            and self.control_cancel_reason == 'manual_pause'
        ):
            self.collect_goal_handle = None
            self._manual_pause_complete()
            return
        super()._collect_result(future)

    # ------------------------------------------------------------------
    # Runtime tick
    # ------------------------------------------------------------------

    def _tick(self):
        if self.manual_paused:
            return

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
            # No distance-trigger rescheduling: _start_sweep_follow() always
            # targets the next fixed relocalization station directly.


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
