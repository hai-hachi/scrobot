#!/usr/bin/env python3
"""Offline contract tests for SCROBOT wheel-speed calibration settings."""
import math
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[3]
WHEEL_RADIUS = 0.050
WHEEL_SEPARATION = 0.420
MAX_WHEEL_M_S = 0.50


def params(path, name):
    document = yaml.safe_load((ROOT / path).read_text(encoding="utf-8"))
    return document[name]["ros__parameters"]


def wheel_speeds(v, yaw_rate):
    offset = WHEEL_SEPARATION * yaw_rate / 2.0
    return (v - offset, v + offset)


class DriveLimitsTests(unittest.TestCase):
    def test_max_wheel_speed_and_equivalent_rpm(self):
        rpm = MAX_WHEEL_M_S / (2.0 * math.pi * WHEEL_RADIUS) * 60.0
        self.assertAlmostEqual(rpm, 95.4929659, places=5)
        hw_urdf = (ROOT / "src/scrobot_description/urdf/ros2_control_real.xacro").read_text()
        self.assertIn('<param name="max_wheel_rpm">95.0</param>', hw_urdf)

    def test_smoother_and_drive_controller_body_caps_match(self):
        smooth = params("src/scrobot_control/config/command_pipeline.yaml", "velocity_smoother")
        self.assertEqual(smooth["smoothing_frequency"], 25.0)
        self.assertEqual(smooth["feedback"], "OPEN_LOOP")
        self.assertEqual(smooth["max_velocity"], [0.5, 0.0, 2.3])
        self.assertEqual(smooth["min_velocity"], [-0.5, 0.0, -2.3])
        self.assertEqual(smooth["max_accel"], [1.0, 0.0, 1.0])
        self.assertEqual(smooth["max_decel"], [-1.4, 0.0, -1.8])
        for config in (
            "src/scrobot_control/config/controllers.yaml",
            "src/scrobot_debug/config/controllers_control_stack_check.yaml",
        ):
            controller = params(config, "diff_drive_controller")
            for prefix, max_speed in (("linear.x", 0.5), ("angular.z", 2.3)):
                self.assertTrue(controller[prefix + ".has_velocity_limits"])
                self.assertAlmostEqual(controller[prefix + ".max_velocity"], max_speed)
                self.assertAlmostEqual(controller[prefix + ".min_velocity"], -max_speed)

    def test_nav2_baseline_for_half_meter_curves(self):
        config = params("src/scrobot_navigation/config/nav2_params.yaml", "controller_server")
        rpp = config["FollowPath"]
        cruise = rpp["desired_linear_vel"]
        radius = rpp["regulated_linear_scaling_min_radius"]
        self.assertEqual(config["controller_frequency"], 25.0)
        self.assertLessEqual(cruise * (1 + WHEEL_SEPARATION / (2 * radius)),
                             MAX_WHEEL_M_S)
        self.assertEqual(cruise, 0.35)
        effective_lookahead = round(cruise * rpp["lookahead_time"], 6)
        self.assertLessEqual(rpp["min_lookahead_dist"], effective_lookahead)
        self.assertGreaterEqual(rpp["max_lookahead_dist"], effective_lookahead)

    def test_smc_maxima_respect_combined_wheel_limit(self):
        for config in (
            "src/scrobot_collection/config/local_collect.yaml",
            "src/scrobot_mission/config/local_collect.yaml",
        ):
            smc = params(config, "local_collect_controller")
            v = smc["smc_max_linear_speed"]
            w = smc["smc_max_angular_speed"]
            self.assertEqual(v, 0.35)
            self.assertEqual(w, 0.70)
            self.assertEqual(smc["smc_reference_speed"], v)
            for sign_v in (-1, 1):
                for sign_w in (-1, 1):
                    speeds = wheel_speeds(sign_v * v, sign_w * w)
                    self.assertLessEqual(max(map(abs, speeds)), MAX_WHEEL_M_S)

    def test_independent_body_limits_are_not_full_wheel_envelope(self):
        # Prevent future documentation from presenting these independent
        # config caps as a wheel-space safety guarantee.
        speeds = wheel_speeds(0.50, 2.30)
        self.assertGreater(max(map(abs, speeds)), MAX_WHEEL_M_S)


if __name__ == "__main__":
    unittest.main()
