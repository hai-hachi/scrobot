"""Offline regression tests for the read-only health classifier."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from health_rules import ERROR, OK, WARN, evaluate, primary_finding  # noqa: E402


def good(**changes):
    obs = {
        "ready": True,
        "expect_clock": True,
        "expect_mission": True,
        "expect_perception": True,
        "expect_localization": True,
        "expect_scan": True,
        "clock_age": 0.05,
        "robot_description_seen": True,
        "rgb_age": 0.12,
        "depth_age": 0.13,
        "scan_age": 0.10,
        "odom_age": 0.04,
        "yolo_age": 0.12,
        "tf_local": True,
        "tf_global": True,
        "mission_state": "SWEEPING",
        "state_age": 120.0,   # Long sweeps are expected.
        "control_mode": "AUTO",
        "failure_reason": "",
        "collision_action": "DO_NOTHING",
        "collision_polygon": "",
        "blocked_for": 0.0,
        "auto_blocked_for": 0.0,
        "stall_for": 0.0,
        "no_command_for": 0.0,
    }
    obs.update(changes)
    return obs


def component(findings, name):
    return next(f for f in findings if f.component == name)


class HealthRulesTest(unittest.TestCase):
    def test_healthy_long_sweep(self):
        findings = evaluate(good())
        self.assertTrue(all(f.level == OK for f in findings))
        self.assertEqual(primary_finding(findings).code, "OK")

    def test_startup_grace_is_not_fault(self):
        findings = evaluate(good(
            ready=False, clock_age=None, robot_description_seen=False,
            rgb_age=None, depth_age=None, scan_age=None, odom_age=None,
            tf_local=False, tf_global=False, mission_state="UNKNOWN",
        ))
        self.assertEqual(primary_finding(findings).level, OK)

    def test_clock_pause_uses_wall_time(self):
        primary = primary_finding(evaluate(good(
            clock_age=8.0, scan_age=8.0, odom_age=8.0
        )))
        self.assertEqual(primary.code, "CLOCK_STALLED")
        self.assertEqual(primary.level, ERROR)

    def test_missing_robot_description_is_root_cause(self):
        findings = evaluate(good(
            robot_description_seen=False, rgb_age=None, depth_age=None
        ))
        self.assertEqual(primary_finding(findings).code, "ROBOT_DESCRIPTION_MISSING")

    def test_mission_error_exposes_original_reason(self):
        reason = "Nav2 lifecycle service unavailable."
        findings = evaluate(good(
            mission_state="ERROR", failure_reason=reason, state_age=1.0
        ))
        mission = component(findings, "Mission")
        self.assertEqual(mission.code, "MISSION_ERROR")
        self.assertEqual(mission.message, reason)
        self.assertEqual(mission.level, ERROR)

    def test_collision_stop_not_misclassified_as_motor_failure(self):
        findings = evaluate(good(
            collision_action="STOP", collision_polygon="stop_zone",
            blocked_for=3.2, stall_for=0.0,
        ))
        gate = component(findings, "Command gate")
        self.assertEqual(gate.code, "COLLISION_STOP")
        self.assertIn("stop_zone", gate.message)
        self.assertEqual(primary_finding(findings).code, "COLLISION_STOP")

    def test_unknown_downstream_suppression(self):
        gate = component(evaluate(good(blocked_for=1.2)), "Command gate")
        self.assertEqual(gate.code, "FINAL_COMMAND_SUPPRESSED")
        self.assertEqual(gate.level, WARN)

    def test_drive_command_without_motion(self):
        movement = component(evaluate(good(stall_for=5.0)), "Drive progress")
        self.assertEqual(movement.code, "MOTION_STALLED")

    def test_mux_suppression_and_manual_exemption(self):
        auto = component(evaluate(good(auto_blocked_for=2.0)), "Command gate")
        manual = component(evaluate(good(
            control_mode="MANUAL", auto_blocked_for=2.0,
            no_command_for=30.0,
        )), "Command gate")
        self.assertEqual(auto.code, "AUTO_COMMAND_SUPPRESSED")
        self.assertEqual(manual.code, "OK")

    def test_no_command_during_sweep(self):
        movement = component(evaluate(good(no_command_for=18.0)), "Drive progress")
        self.assertEqual(movement.code, "MISSION_NO_COMMAND")

    def test_no_command_during_relocalization_is_expected(self):
        movement = component(evaluate(good(
            mission_state="RELOCALIZING", state_age=4.0,
            no_command_for=20.0,
        )), "Drive progress")
        self.assertEqual(movement.code, "OK")

    def test_initial_global_tf_unavailable_is_normal(self):
        status = component(evaluate(good(
            mission_state="INITIAL_TAG_APPROACH", state_age=10.0,
            tf_global=False,
        )), "TF")
        self.assertEqual(status.level, OK)

    def test_missing_global_tf_when_sweeping(self):
        status = component(evaluate(good(tf_global=False)), "TF")
        self.assertEqual(status.code, "GLOBAL_TF_MISSING")
        self.assertEqual(status.level, ERROR)

    def test_minimal_gazebo_mode_disables_optional_subsystems(self):
        findings = evaluate(good(
            expect_mission=False, expect_perception=False,
            expect_localization=False, expect_scan=False,
            mission_state="UNKNOWN", yolo_age=None,
            odom_age=None, scan_age=None,
            tf_local=False, tf_global=False,
        ))
        self.assertEqual(primary_finding(findings).level, OK)

    def test_hardware_mode_no_clock_required(self):
        findings = evaluate(good(expect_clock=False, clock_age=None))
        self.assertEqual(primary_finding(findings).level, OK)

    def test_stale_yolo(self):
        status = component(evaluate(good(yolo_age=7.0)), "YOLO")
        self.assertEqual(status.code, "YOLO_STALE")
        self.assertEqual(status.level, WARN)

    def test_long_nontravel_phase_times_out(self):
        status = component(evaluate(good(
            mission_state="INITIAL_RELOCALIZATION",
            state_age=60.0,
        )), "Mission")
        self.assertEqual(status.code, "MISSION_STATE_TIMEOUT")


if __name__ == "__main__":
    unittest.main()
