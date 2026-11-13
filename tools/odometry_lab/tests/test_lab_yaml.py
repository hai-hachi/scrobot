"""Independent saved Odometry YAML and shared Mobility source contract."""
from pathlib import Path
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/"tools"))
sys.path.insert(0,str(ROOT/"tools/odometry_lab"))
sys.path.insert(0,str(ROOT/"src/scrobot_simulation/launch"))
from lab_configs import read, save, path_for
from core import DEFAULTS, validate, RATE_NAMES
from odometry_rate_config import validated_rates

class OdometryYamlTests(unittest.TestCase):
    def test_saved_odometry_yaml_contains_validated_100hz_rates(self):
        config=validate({**DEFAULTS,"imu_hz":80.0,"ekf_hz":90.0})
        with tempfile.TemporaryDirectory() as td:
            odom=Path(td)/"odometry.yaml"
            mobility=Path(td)/"mobility.yaml"
            save("odometry",config,odom,validator=validate)
            self.assertEqual(read("odometry",odom),config)
            rates={key:config[key] for key in RATE_NAMES}
            self.assertEqual(validated_rates(rates)["imu_hz"],80.0)
            # Only the owning lab may overwrite its YAML.
            save("mobility",{"note":"shared source"},mobility)
            self.assertEqual(read("mobility",mobility)["note"],"shared source")
            self.assertEqual(read("odometry",odom),config)
            self.assertEqual(path_for("odometry").name,"odometry_lab.yaml")

    def test_publish_uses_lab_yaml_not_interactive_json(self):
        code=(ROOT/"tools/odometry_lab/server.py").read_text()
        self.assertIn('read_lab_yaml("odometry")',code)
        self.assertIn('save_lab_yaml("odometry"',code)
        self.assertIn('"/api/config/save"',code)
        self.assertIn('read_lab_yaml("mobility")',code)
        ui=(ROOT/"tools/odometry_lab/index.html").read_text()
        self.assertIn('Save Odometry Lab YAML',ui)
if __name__=="__main__":
    unittest.main()
