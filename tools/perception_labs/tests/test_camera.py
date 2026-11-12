"""Offline regression tests for SCROBOT Camera Lab publishing + metrics."""
from __future__ import annotations
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from xml.etree import ElementTree as ET

ROOT=Path(__file__).resolve().parents[3]
LAB=ROOT/"tools/perception_labs"
sys.path.insert(0,str(LAB))
from camera_config import CONFIG, document, publish_benchmark, read_config, status
from core import Meter, validate
from resources import ResourceMeter, process_counters, descendants


def benchmark(config, valid=True):
    c=validate({**config,"mode":"camera"})
    w,h=map(int,c["color"].split("x"))
    dw,dh=map(int,c["depth"].split("x"))
    return {"mode":"camera","phase":"streaming","settings":c,
            "monitor":{"streams":{
                "rgb":{"samples":100 if valid else 2,"resolution":[w,h],
                       "sim_fps":c["color_fps"]},
                "depth":{"samples":100,"resolution":[dw,dh],
                         "sim_fps":c["depth_fps"]},
            }}}


class CameraPublishingTests(unittest.TestCase):
    def test_published_baseline_is_unchanged(self):
        raw=read_config(CONFIG)
        self.assertEqual(raw["color"],{"width":1280,"height":720,"fps":15})
        self.assertEqual(raw["depth"],
                         {"width":848,"height":480,"fps":15,"enabled":True})
        self.assertEqual(document({"color":"1280x720","depth":"848x480",
                                  "color_fps":15,"depth_fps":15,
                                  "depth_enabled":True}),raw)

    def test_publish_only_measured_camera_snapshot_and_backup(self):
        with tempfile.TemporaryDirectory() as td:
            dest=Path(td)/"camera_calibration.yaml"
            prior=CONFIG.read_bytes()
            dest.write_bytes(prior)
            backups=Path(td)/"backups"
            cfg={"color":"640x480","depth":"424x240","color_fps":30,
                 "depth_fps":10,"depth_enabled":True}
            record=benchmark(cfg)
            with self.assertRaisesRegex(ValueError,"30 RGB"):
                publish_benchmark(benchmark(cfg,valid=False),dest,backups)
            self.assertEqual(dest.read_bytes(),prior)
            record["monitor"]["streams"]["depth"]["sim_fps"]=5
            with self.assertRaisesRegex(ValueError,"Depth stream below"):
                publish_benchmark(record,dest,backups)
            self.assertEqual(dest.read_bytes(),prior)
            record["monitor"]["streams"]["depth"]["sim_fps"]=10
            result=publish_benchmark(record,dest,backups)
            self.assertTrue(result["changed"])
            self.assertEqual(Path(result["backup"]).read_bytes(),prior)
            self.assertEqual(read_config(dest),document(cfg))
            self.assertEqual(status(cfg,dest)["matches_lab"],True)
            self.assertFalse(publish_benchmark(record,dest,backups)["changed"])
            self.assertEqual(len(list(backups.iterdir())),1)

    def test_production_launch_and_xacro_camera_parameter_chain(self):
        launch=(ROOT/"src/scrobot_simulation/launch/simulation.launch.py").read_text()
        spawn=(ROOT/"src/scrobot_simulation/launch/spawn_robot.launch.py").read_text()
        wrapper=(ROOT/"src/scrobot_simulation/urdf/scrobot_sim.urdf.xacro").read_text()
        sensor=(ROOT/"src/scrobot_simulation/urdf/sensors_gazebo.xacro").read_text()
        for key in ("color_width","color_height","color_fps",
                    "depth_width","depth_height","depth_fps"):
            self.assertIn(key,launch)
            self.assertIn(key,spawn)
            self.assertIn(key,wrapper)
            self.assertIn(key,sensor)
        self.assertIn("camera_calibration.yaml",launch)
        self.assertIn("camera['depth']['enabled']",launch)
        for p in ("scrobot_sim.urdf.xacro","sensors_gazebo.xacro"):
            ET.parse(ROOT/"src/scrobot_simulation/urdf"/p)
        self.assertNotIn("camera_calibration.yaml",
                          (ROOT/"src/scrobot_bringup/launch/robot.launch.py").read_text())

    def test_camera_ros_payload_bytes_measured_from_received_images(self):
        m=Meter()
        now=time.monotonic()
        for i in range(40):
            m.stamps.append(i/15)
            m.wall.append(now+i/30)
            m.payload.append((now+i/30,1_000_000))
            m.n+=1
            m.total_bytes+=1_000_000
            m.payload_sum+=1_000_000
        result=m.status()
        self.assertEqual(result["payload_total_mb"],40)
        self.assertAlmostEqual(result["payload_mean_mb"],1)
        self.assertAlmostEqual(result["payload_mb_s"],30,delta=.1)
        self.assertAlmostEqual(result["effective_rtf"],.5,delta=.01)

    def test_linux_proc_cpu_sampling_is_noninvasive(self):
        cpu,rss=process_counters(os.getpid())
        self.assertGreaterEqual(cpu,0)
        self.assertGreater(rss,0)
        self.assertIn(os.getpid(),descendants(os.getpid()))
        class SelfProcess:
            pid=os.getpid()
            def poll(self):
                return None
        meter=ResourceMeter()
        self.assertIsNone(meter.snapshot({"server":SelfProcess()})
                          ["server"]["cpu_pct_one_core"])
        time.sleep(.12)
        second=meter.snapshot({"server":SelfProcess()})["server"]
        self.assertGreaterEqual(second["cpu_pct_one_core"],0)
        self.assertGreater(second["rss_mb"],0)

    def test_camera_ui_and_server_publish_contract(self):
        ui=(LAB/"index.html").read_text()
        server=(LAB/"server.py").read_text()
        for token in ('id="rgbMBs"','id="depthMBs"','id="resourceRows"',
                      'id="publishCamera"','id="publishCameraStatus"'):
            self.assertIn(token,ui)
        self.assertIn("/api/camera/publish",server)
        self.assertIn("saved_camera_benchmark",server)
        self.assertIn("publish_benchmark(record)",server)


if __name__=="__main__":
    unittest.main()
