"""Saved Mobility YAML is the production handoff, not transient JSON."""
from pathlib import Path
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/"tools"))
sys.path.insert(0,str(ROOT/"tools/mobility_lab"))
from lab_configs import read, save, path_for
from core import DEFAULTS, validate_settings
from publish import publish_saved_calibration, SIM_CONFIG, read_production_config

class MobilityYamlTests(unittest.TestCase):
    def test_mobility_publishes_explicitly_saved_yaml_only(self):
        settings=validate_settings({**DEFAULTS,
            "model":"first_order","drive_contact_mu":4.75,"tau_left":0.065})
        with tempfile.TemporaryDirectory() as td:
            folder=Path(td)
            yaml_path=folder/"mobility_lab.yaml"
            prod=folder/"mobility_calibration.yaml"
            prod.write_bytes(SIM_CONFIG.read_bytes())
            baseline=prod.read_bytes()
            self.assertIsNone(read("mobility",yaml_path))
            save("mobility",settings,yaml_path,validator=validate_settings)
            self.assertEqual(read("mobility",yaml_path),settings)
            self.assertEqual(prod.read_bytes(),baseline)
            result=publish_saved_calibration(yaml_path,prod,folder/"backups")
            self.assertEqual(result["changed"],True)
            self.assertEqual(
                read_production_config(prod)["motor"]["tau_left"],0.065)
            self.assertEqual(Path(result["backup"]).read_bytes(),baseline)
            self.assertEqual(path_for("mobility").name,"mobility_lab.yaml")

    def test_saved_document_rejects_mobility_schema_mismatch(self):
        with tempfile.TemporaryDirectory() as td:
            target=Path(td)/"bad.yaml"
            save("camera",{"color":"640x480"},target)
            with self.assertRaisesRegex(ValueError,"not a supported mobility"):
                read("mobility",target)
            with self.assertRaises(ValueError):
                save("mobility",{**DEFAULTS,"drive_slip1":-4},target,
                     validator=validate_settings)
if __name__=="__main__":
    unittest.main()
