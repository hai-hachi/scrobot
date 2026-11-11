"""Prevent Lighting Lab preset saves from overwriting prior experiments."""
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

from server import Lab


class LightingPresetIsolationTests(unittest.TestCase):
    def test_duplicate_preset_name_never_replaces_existing_result(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "presets"
            lab = object.__new__(Lab)
            lab.source = Path(__file__).resolve().parents[2] / (
                "src/scrobot_simulation/worlds/badminton_court.sdf"
            )
            lab.settings = {"lights": {"rows": 2, "cols": 3}}
            lab.adapter = SimpleNamespace(
                status=lambda: {
                    "sim": {}, "feeds": {}, "gpu": None,
                    "load_1m": 0.0, "cpu_cores": 1,
                },
                last_action="",
            )
            with patch("server.PRESSTS", target):
                saved = lab.save_preset({"name": "my_test"})
                original = Path(saved).read_bytes()
                with self.assertRaisesRegex(ValueError, "already exists"):
                    lab.save_preset({"name": "my_test"})
                self.assertEqual(Path(saved).read_bytes(), original)
                self.assertEqual(len(list(target.glob("*.json"))), 1)


if __name__ == "__main__":
    unittest.main()
