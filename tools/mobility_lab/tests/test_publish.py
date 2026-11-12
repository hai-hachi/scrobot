"""Offline publication safety and simulation motor-response contract tests."""
from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools/mobility_lab"))

from core import DEFAULTS
from publish import (
    SIM_CONFIG, build_text, calibration_document, publish_saved_calibration,
    published_status, read_production_config,
)


def sim_motor():
    path = ROOT / "src/scrobot_simulation/scripts/sim_motor_response.py"
    spec = importlib.util.spec_from_file_location("sim_motor_response", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class PublicationTests(unittest.TestCase):
    def test_baseline_matches_published_defaults(self):
        self.assertEqual(read_production_config(SIM_CONFIG), calibration_document(DEFAULTS))
        self.assertEqual(read_production_config(SIM_CONFIG)["motor"]["model"], "baseline")

    def test_atomic_publish_and_previous_file_backup(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            settings = root / "settings.json"
            production = root / "mobility_calibration.yaml"
            backups = root / "backups"
            old = SIM_CONFIG.read_text(encoding="utf8")
            production.write_text(old, encoding="utf8")
            altered = dict(DEFAULTS)
            altered.update({
                "drive_contact_mu": 3.5,
                "caster_contact_mu": 0.12,
                "wheel_joint_damping": 0.018,
                "drive_slip1": 0.001,
                "drive_slip2": 0.004,
                "model": "first_order",
                "tau_left": 0.061,
                "tau_right": 0.055,
                "gain_left": 0.98,
                "gain_right": 1.01,
            })
            settings.write_text(json.dumps(altered), encoding="utf8")
            self.assertFalse(published_status(altered, production)["matches_saved"])
            result = publish_saved_calibration(settings, production, backups)
            self.assertTrue(result["changed"])
            self.assertEqual(Path(result["backup"]).read_text(), old)
            self.assertEqual(read_production_config(production),
                             calibration_document(altered))
            self.assertTrue(published_status(altered, production)["matches_saved"])
            second = publish_saved_calibration(settings, production, backups)
            self.assertFalse(second["changed"])
            self.assertIsNone(second["backup"])
            self.assertEqual(len(list(backups.iterdir())), 1)

    def test_invalid_saved_settings_cannot_modify_main_sim(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            settings = root / "settings.json"
            production = root / "mobility_calibration.yaml"
            backups = root / "backups"
            original = SIM_CONFIG.read_text(encoding="utf8")
            production.write_text(original, encoding="utf8")
            for invalid in (
                {"tau_left": -0.2}, {"wheel_collision_width": 0.7},
                {"model": "not_a_motor"}, {"drive_slip1": float("nan")},
            ):
                config = {**DEFAULTS, **invalid}
                settings.write_text(json.dumps(config), encoding="utf8")
                with self.assertRaises((ValueError, TypeError)):
                    publish_saved_calibration(settings, production, backups)
                self.assertEqual(production.read_text(), original)
            self.assertFalse(backups.exists())

    def test_main_sim_only_routing_contract(self):
        sim = (ROOT / "src/scrobot_bringup/launch/sim.launch.py").read_text()
        real = (ROOT / "src/scrobot_bringup/launch/robot.launch.py").read_text()
        stack = (ROOT / "src/scrobot_control/launch/control_stack.launch.py").read_text()
        physics = (ROOT / "src/scrobot_simulation/launch/simulation.launch.py").read_text()
        assert "motor_model_enabled" in sim
        assert "/cmd_vel_motor_target" in sim
        assert "collision_monitor_output" in sim
        assert "collision_monitor_output" in stack
        assert "'/diff_drive_controller/cmd_vel'" in stack
        assert "mobility_calibration.yaml" in physics
        assert "motor_model_enabled" not in real
        assert "/cmd_vel_motor_target" not in real

    def test_sim_motor_filter_and_speed_envelope(self):
        mod = sim_motor()
        v, w = mod.wheel_rpm_to_twist(-90, +90)
        l, r = mod.twist_to_wheel_rpm(v, w)
        self.assertAlmostEqual(l, -90, places=7)
        self.assertAlmostEqual(r, 90, places=7)
        self.assertEqual(mod.limit_wheels(100, -50), (95.0, -47.5))
        value = 0.0
        for i in range(20):
            value = mod.filter_update(value, 90.0, 0.01, 0.05, 1.0)
        self.assertAlmostEqual(value, 90.0 * (1 - math.exp(-4.0)), places=6)
        with self.assertRaises(ValueError):
            mod.filter_update(0.0, 90.0, 0.01, 0.0, 1.0)


if __name__ == "__main__":
    unittest.main()
