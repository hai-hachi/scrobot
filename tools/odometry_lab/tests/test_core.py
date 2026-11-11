"""Offline Odometry Lab mathematics and simulation rate wiring regressions."""
import math
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

LAB=Path(__file__).resolve().parents[1]
REPO=LAB.parents[1]
sys.path.insert(0,str(LAB))
sys.path.insert(0,str(REPO/"src/scrobot_simulation/launch"))

from core import (
    DEFAULTS, StreamMeter, analyse, duration, interpolate_pose,
    pose_metrics, quaternion_yaw, relative, segments, sequence_command,
    validate, wrap,
)
from odometry_rate_config import (
    DEFAULT_RATES, load_rates, prepare_configurations, validated_rates,
)


class CoreTests(unittest.TestCase):
    def test_valid_100_hz_defaults_and_safety(self):
        s=validate(DEFAULTS)
        self.assertTrue(all(s[k]==100 for k in DEFAULT_RATES))
        for pair in (
            {"wheel_odom_hz":200},
            {"imu_hz":0},
            {"ekf_hz":float("nan")},
            {"linear_m_s":0.40,"angular_rad_s":1.8},
            {"profile":"sideways"},
        ):
            with self.assertRaises(ValueError):
                validate(pair)
        self.assertEqual(duration({**DEFAULTS,"profile":"stationary"}),30.0)
        self.assertEqual(sequence_command(DEFAULTS,-1.0),(0.0,0.0,False))

    def test_trajectory_segments_are_within_wheel_limit(self):
        for name in ("stationary","straight","reverse","spin90","spin180",
                     "spin360","square","figure8"):
            cfg=validate({**DEFAULTS,"profile":name})
            for seconds,v,yaw in segments(cfg):
                self.assertGreater(seconds,0)
                self.assertLessEqual(abs(v)+.21*abs(yaw),.5+1e-10)
            self.assertTrue(sequence_command(cfg,duration(cfg)+.01)[2])

    def test_yaw_math_and_normalized_pose_accuracy(self):
        self.assertAlmostEqual(wrap(math.radians(370)),math.radians(10))
        self.assertAlmostEqual(quaternion_yaw((0,0,0,1)),0)
        truth=[]
        wheel=[]
        for i in range(101):
            t=i*.01
            truth.append({"t":t,"x":3+t,"y":7.0,"yaw":1.1})
            wheel.append({"t":t,"x":10+t,"y":2.0,"yaw":1.1})
        m=pose_metrics(wheel,truth)
        self.assertIsNotNone(m)
        self.assertEqual(m["samples"],101)
        # World translations differ but both relative robot paths coincide.
        self.assertAlmostEqual(m["position_rmse_m"],0,delta=1e-6)
        self.assertAlmostEqual(m["yaw_rmse_deg"],0,delta=1e-6)

        fixed=[{"t":x["t"],"x":x["x"]+5,
                "y":x["y"]-2,"yaw":x["yaw"]} for x in truth]
        m=pose_metrics(fixed,truth)
        self.assertEqual(m["position_rmse_m"],0)
        self.assertEqual(m["yaw_rmse_deg"],0)

        offset=[{**x,"x":x["x"]+(i*.002)} for i,x in enumerate(truth)]
        m=pose_metrics(offset,truth)
        self.assertGreater(m["position_rmse_m"],.05)

    def test_interpolation_and_stream_metrics(self):
        rows=[{"t":0.0,"x":0,"y":0,"yaw":math.radians(175)},
              {"t":1.0,"x":1,"y":2,"yaw":math.radians(-175)}]
        interp=interpolate_pose(rows,.5)
        self.assertAlmostEqual(interp["x"],.5)
        self.assertAlmostEqual(abs(interp["yaw"]),math.pi)
        self.assertIsNone(interpolate_pose(rows,-.2))
        meter=StreamMeter()
        for i in range(100):
            meter.add(i*.01,"base_link")
        stats=meter.status()
        self.assertAlmostEqual(stats["hz"],100)
        self.assertAlmostEqual(stats["jitter_ms"],0)
        self.assertEqual(stats["frame_id"],"base_link")
        meter.add(.99,"base_link")
        self.assertEqual(meter.status()["stamp_nonmonotonic"],1)

    def test_generated_controller_and_ekf_rates_without_touching_source(self):
        controllers=REPO/"src/scrobot_control/config/controllers.yaml"
        ekf=REPO/"src/scrobot_localization/config/ekf.yaml"
        source_odom=controllers.read_text()
        source_ekf=ekf.read_text()
        published=REPO/"src/scrobot_simulation/config/odometry_calibration.yaml"
        self.assertEqual(load_rates(published),DEFAULT_RATES)
        with tempfile.TemporaryDirectory() as td:
            rates={**DEFAULT_RATES,"wheel_odom_hz":80,"ekf_hz":120}
            produced=prepare_configurations(rates,controllers,ekf,td)
            c=yaml.safe_load(Path(produced["controller_params"]).read_text())
            e=yaml.safe_load(Path(produced["ekf_params"]).read_text())
            self.assertEqual(c["controller_manager"]["ros__parameters"]["update_rate"],100)
            self.assertEqual(c["diff_drive_controller"]["ros__parameters"]["publish_rate"],80)
            self.assertEqual(e["ekf_filter_node"]["ros__parameters"]["frequency"],120)
        self.assertEqual(controllers.read_text(),source_odom)
        self.assertEqual(ekf.read_text(),source_ekf)
        with self.assertRaises(ValueError):
            validated_rates({**DEFAULT_RATES,"wheel_odom_hz":150})

    def test_main_sim_only_rate_wiring(self):
        sim=(REPO/"src/scrobot_simulation/launch/simulation.launch.py").read_text()
        bringup=(REPO/"src/scrobot_bringup/launch/sim.launch.py").read_text()
        real=(REPO/"src/scrobot_bringup/launch/robot.launch.py").read_text()
        local=(REPO/"src/scrobot_localization/launch/localization.launch.py").read_text()
        sensors=(REPO/"src/scrobot_simulation/urdf/sensors_gazebo.xacro").read_text()
        self.assertIn("odometry_calibration.yaml",sim)
        self.assertIn("prepare_configurations",sim)
        self.assertIn("generated_odometry['ekf_params']",bringup)
        self.assertIn("ekf_params",local)
        self.assertNotIn("odometry_calibration.yaml",real)
        self.assertNotIn("generated_odometry",real)
        self.assertIn("imu_rate_hz",sensors)
        self.assertIn("magnetometer_rate_hz",sensors)


if __name__=="__main__":
    unittest.main()
