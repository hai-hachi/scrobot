"""Headless tests; no ROS installation or Gazebo required."""
import math
import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core import (
    DEFAULTS, WheelFilter, cap_wheels, compare_rmse, first_order_reference,
    profile_targets, reference_csv, response_metrics, rpm_to_twist,
    twist_to_rpm, validate_settings,
)


class MobilityCoreTests(unittest.TestCase):
    def test_90_rpm_matches_loaded_wheel_geometry(self):
        v, w = rpm_to_twist(90, 90)
        self.assertAlmostEqual(v, 0.471238898, places=7)
        self.assertEqual(w, 0)
        self.assertAlmostEqual(twist_to_rpm(v, w)[0], 90, places=7)
        self.assertLessEqual(abs(v), 0.50)

    def test_floor_spin_and_single_wheel(self):
        v, w = rpm_to_twist(-90, 90)
        self.assertAlmostEqual(v, 0, places=7)
        self.assertAlmostEqual(w, 2.24399475256, places=7)
        self.assertEqual(profile_targets("left", 90), (90.0, 0))
        self.assertEqual(profile_targets("right", 90), (0, 90.0))
        self.assertEqual(profile_targets("reverse", 90), (-90.0, -90.0))

    def test_wheel_envelope_preserves_curvature(self):
        left, right = cap_wheels(120, -60)
        self.assertEqual((left, right), (95.0, -47.5))

    def test_first_order_settles_near_point_two_seconds(self):
        cfg = validate_settings({**DEFAULTS, "model": "first_order"})
        filt = WheelFilter()
        values = []
        for n in range(1, 31):
            y, _ = filt.update((90, 0), 0.01, cfg)
            values.append(y)
        self.assertAlmostEqual(values[19], 90*(1-math.exp(-4)), places=6)
        self.assertAlmostEqual(first_order_reference(.20, 90), 90*.98, places=5)
        self.assertLess(values[0], 90)
        self.assertGreater(values[19], 88)
        filt.reset()
        self.assertEqual((filt.left, filt.right), (0, 0))

    def test_baseline_command_is_immediate(self):
        filt = WheelFilter()
        self.assertEqual(filt.update((90, -90), .01, DEFAULTS), (90, -90))

    def test_measured_settling_metrics(self):
        samples = [
            {"t": t/100, "actual_left_rpm": first_order_reference(t/100, 90),
             "actual_right_rpm": 0}
            for t in range(151)
        ]
        m = response_metrics(samples, "left", 90, 1.5)
        self.assertIsNotNone(m)
        self.assertAlmostEqual(m["settling_s"], .20, delta=.015)
        self.assertAlmostEqual(m["tau_63_s"], .05, delta=.015)
        self.assertIsNone(response_metrics(samples, "right", 0, 1.5))
        bad = [{"t": x["t"], "actual_left_rpm": x["actual_left_rpm"]*.90}
               for x in samples]
        self.assertIsNone(response_metrics(bad, "left", 90, 1.5)["settling_s"])

    def test_csv_import_and_rmse(self):
        reference = reference_csv("time_s,left_actual_rpm,right_actual_rpm\n"
                                  "0,0,0\n0.1,30,20\n0.2,60,40\n")
        samples = [{"t": x["t"], "actual_left_rpm": x["actual_left_rpm"],
                    "actual_right_rpm": x["actual_right_rpm"]} for x in reference]
        self.assertEqual(compare_rmse(samples, reference, "left"), 0)
        self.assertEqual(compare_rmse(samples, reference, "right"), 0)
        self.assertEqual(len(reference_csv(
            "time_s,left_rpm\n0,0\n0.1,40\n0.2,80\n")), 3)

    def test_reject_corrupt_csv_and_config(self):
        for bad in ("time_s,left_actual_rpm\n0,0\n0,4\n1,9",
                    "time_s,left_actual_rpm\n0,0\n1,nan\n2,2"):
            with self.assertRaises(ValueError):
                reference_csv(bad)
        for bad in (
            {"model": "magic"},
            {"drive_contact_mu": -1},
            {"tau_left": 0},
            {"unknown": 123},
            {"wheel_collision_width": 0.2},
        ):
            with self.assertRaises(ValueError):
                validate_settings(bad)


if __name__ == "__main__":
    unittest.main()
