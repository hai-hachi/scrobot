"""Headless Camera Lab contracts; no ROS, Gazebo, CUDA or display required."""
from __future__ import annotations
from pathlib import Path
from unittest import TestCase, mock
from xml.etree import ElementTree as ET
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[3]
LAB=ROOT/"tools/camera_lab"
sys.path.insert(0,str(LAB))
import modes
import server


class CameraLabTests(TestCase):
    def test_d435i_usb3_catalog_matches_selected_mode_rules(self):
        self.assertEqual(len(modes.COLOR),9)
        self.assertEqual(len(modes.DEPTH),6)
        self.assertEqual(modes.COLOR["1920x1080"],(6,15,30))
        self.assertEqual(modes.COLOR["320x180"],(6,30,60))
        self.assertEqual(modes.DEPTH["848x480"],(6,15,30,60,90))
        self.assertEqual(modes.DEFAULT,{"color":"1280x720","color_fps":15,
                                        "depth":"848x480","depth_fps":15,
                                        "depth_enabled":True})
        for bad in (
            {**modes.DEFAULT,"color_fps":60},
            {**modes.DEFAULT,"depth_fps":120},
            {**modes.DEFAULT,"color":"1024x768"},
            {**modes.DEFAULT,"depth_enabled":"false"},
            {**modes.DEFAULT,"extra":True},
        ):
            with self.assertRaises(ValueError):
                modes.validate(bad)

    def test_expected_uncompressed_ros_bytes_not_usb_bytes(self):
        predicted=modes.expected_mb_s(modes.DEFAULT)
        self.assertAlmostEqual(predicted["rgb"],41.472)
        self.assertAlmostEqual(predicted["depth"],24.422,places=3)
        self.assertAlmostEqual(predicted["total"],65.894,places=3)
        off={**modes.DEFAULT,"depth_enabled":False}
        self.assertEqual(modes.expected_mb_s(off)["depth"],0)

    def test_minimal_sensor_world_and_scene(self):
        with tempfile.TemporaryDirectory() as tmp:
            output=Path(tmp)/"world.sdf"
            modes.world(modes.DEFAULT,output)
            root=ET.parse(output).getroot()
            sensors=root.findall(".//sensor")
            self.assertEqual(len(sensors),2)
            rgb,depth=sensors
            self.assertEqual(rgb.get("name"),"color")
            self.assertEqual(depth.get("name"),"depth")
            self.assertEqual(rgb.findtext("update_rate"),"15")
            self.assertEqual(depth.findtext("camera/image/width"),"848")
            self.assertEqual(rgb.findtext("camera/image/height"),"720")
            self.assertIsNotNone(root.find(".//plugin[@filename='gz-sim-sensors-system']"))
            self.assertEqual(len(root.findall(".//model")),3)
            modes.world({**modes.DEFAULT,"depth_enabled":False},output)
            sensors=ET.parse(output).getroot().findall(".//sensor")
            self.assertEqual(len(sensors),1)
            modes.world({"color":"1920x1080","color_fps":30,
                         "depth":"424x240","depth_fps":90,
                         "depth_enabled":True},output)
            sensors=ET.parse(output).getroot().findall(".//sensor")
            self.assertEqual(sensors[0].findtext("camera/image/width"),"1920")
            self.assertEqual(sensors[0].findtext("update_rate"),"30")
            self.assertEqual(sensors[1].findtext("update_rate"),"90")

    def test_process_cpu_snapshot(self):
        cpu,rss=server.usage(os.getpid())
        self.assertGreaterEqual(cpu,0)
        self.assertGreaterEqual(rss,0)
        self.assertIn(os.getpid(),server.proc_tree(os.getpid()))

    def test_server_uses_only_camera_and_image_bridge(self):
        lab=server.Lab()
        try:
            args=[]
            with mock.patch.object(lab,"launch",side_effect=lambda name,cmd:args.append((name,cmd))):
                lab.start(dict(modes.DEFAULT))
            self.assertEqual([name for name,_ in args],["gazebo","image_bridge","monitor"])
            self.assertEqual(args[0][1][:5],["gz","sim","-r","-s","-v"])
            self.assertIn("ros_gz_image",args[1][1])
            self.assertNotIn("apriltag",str(args).lower())
            self.assertNotIn("yolo",str(args).lower())
            self.assertNotIn("perception_labs",str(args))
        finally:
            lab.stop()

    def test_yaml_publication_only_when_live_samples_and_safe_wall_rate(self):
        lab=server.Lab()
        lab.phase="streaming"
        record={
            "color":{"frames":40,"resolution":[1280,720],
                     "wall_fps":15,"sim_fps":15},
            "depth":{"frames":40,"resolution":[848,480],
                     "wall_fps":15,"sim_fps":15},
        }
        try:
            with tempfile.TemporaryDirectory() as tmp:
                production=Path(tmp)/"camera_calibration.yaml"
                production.write_text("old camera calibration")
                with mock.patch.object(server,"PUBLISHED",production),\
                     mock.patch.object(server,"RESULTS",Path(tmp)),\
                     mock.patch.object(lab,"check_measurement",return_value=record):
                    result=lab.publish()
                    self.assertTrue(result["changed"])
                    self.assertEqual(Path(result["backup"]).read_text(),"old camera calibration")
                    self.assertIn("schema_version: 1",production.read_text())
                    self.assertIn("width: 1280",production.read_text())
                    self.assertIn("enabled: true",production.read_text())
                    record["color"]["wall_fps"]=5
                    with self.assertRaisesRegex(ValueError,"wall FPS"):
                        lab.publish()
        finally:
            lab.stop()

    def test_static_browser_and_shell(self):
        html=(LAB/"index.html").read_text()
        for control in ("color","depth","color_fps","depth_fps",
                        "depth_enabled","start","stop","save","publish",
                        "rgbRate","depthRate","totalRate","processes"):
            self.assertIn(f'id="{control}"',html)
        self.assertNotIn("Perception Labs v3",html)
        subprocess.run(["bash","-n",str(LAB/"start.sh")],check=True)
        if shutil.which("node"):
            script=html.split("<script>",1)[1].split("</script>",1)[0]
            with tempfile.TemporaryDirectory() as tmp:
                path=Path(tmp)/"lab.js"
                path.write_text(script)
                subprocess.run(["node","--check",str(path)],check=True)


if __name__=="__main__":
    unittest.main()
