"""Offline contracts for standalone SCROBOT YOLO Lab (no ROS/Gazebo required)."""
from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
LAB = ROOT / "tools/yolo_lab"
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tools/camera_lab"))
sys.path.insert(0, str(LAB))

import lab_configs
import modes
import server
import world
from lighting_source import calibration
from yolo_settings import DEFAULT, validate


class YoloLabTests(unittest.TestCase):
    def test_own_yaml_and_shared_camera_yaml_do_not_publish_to_production(self):
        self.assertEqual(lab_configs.path_for("yolo").name, "yolo_lab.yaml")
        self.assertEqual(lab_configs.path_for("camera").name, "camera_lab.yaml")
        produced = [
            ROOT/"src/scrobot_simulation/worlds/badminton_court.sdf",
            ROOT/"src/scrobot_simulation/config/camera_calibration.yaml",
            ROOT/"src/scrobot_perception/config/yolo_shuttle_detector.yaml",
        ]
        before = {p: p.read_bytes() for p in produced}
        with tempfile.TemporaryDirectory() as td:
            a = Path(td)/"camera.yaml"
            b = Path(td)/"yolo.yaml"
            lab_configs.save("camera", modes.DEFAULT, a, modes.validate)
            lab_configs.save("yolo", DEFAULT, b, validate)
            self.assertEqual(lab_configs.read("camera", a), modes.DEFAULT)
            self.assertEqual(lab_configs.read("yolo", b), validate(DEFAULT))
            with self.assertRaisesRegex(ValueError, "not a supported"):
                lab_configs.read("yolo", a)
        for file, original in before.items():
            self.assertEqual(file.read_bytes(), original)

    def test_yolo_config_safety(self):
        self.assertEqual(validate(DEFAULT), {**DEFAULT,
            "robot_x": 2.0, "robot_y": 0.0, "robot_yaw": 0.0,
            "positive_fraction": .85, "confidence": .1})
        for change in (
            {"shuttle_count": 0},
            {"dataset_images": 10},
            {"batch": -1},
            {"imgsz": 650},
            {"confidence": float("nan")},
            {"seed": True},
            {"unknown": "unsafe"},
        ):
            with self.assertRaises(ValueError):
                validate({**DEFAULT, **change})

    def test_real_shuttle_court_uses_saved_lighting_and_camera_mode(self):
        lights = calibration()
        with tempfile.TemporaryDirectory() as td:
            path = Path(td)/"world.sdf"
            output, manifest = world.build(
                {**modes.DEFAULT, "color": "1920x1080", "color_fps": 30,
                 "depth_enabled": False},
                {**DEFAULT, "shuttle_count": 7, "seed": 77},
                path, lighting=lights)
            self.assertEqual(output, str(path))
            scene = ET.parse(path).getroot().find("world")
            self.assertEqual(scene.get("name"), "yolo_lab")
            self.assertEqual(len(scene.findall("light")),
                             lights["lights"]["rows"] * lights["lights"]["cols"])
            self.assertEqual(sum(x.findtext("cast_shadows")=="true"
                                 for x in scene.findall("light")),
                             lights["lights"]["shadow_count"])
            static = {m.get("name") for m in scene.findall("model")}
            for expected in ("court_floor", "court_markings",
                             "net_assembly", "simulation_safety_boundary",
                             "camera_rig"):
                self.assertIn(expected, static)
            self.assertNotIn("target", static)
            self.assertNotIn("court_apriltags", static)
            # ElementTree XPath has no starts-with predicate.
            shuttles = [m for m in scene.findall("model")
                        if m.get("name", "").startswith("yolo_shuttle_")]
            self.assertEqual(len(shuttles), 7)
            for s in shuttles:
                self.assertEqual(s.findtext("static"), "true")
                self.assertEqual(s.findtext(
                    ".//visual/geometry/mesh/uri"),
                    "model://shuttle/meshes/shuttle.STL")
                self.assertIsNone(s.find(".//collision"))
            sensor = scene.find(
                "./model[@name='camera_rig']/link/sensor[@name='color']")
            self.assertEqual(sensor.findtext("camera/image/width"),"1920")
            self.assertEqual(sensor.findtext("update_rate"),"30")
            self.assertEqual(len(scene.findall(".//sensor")),1)
            floor = scene.find("./model[@name='court_floor']")
            box = floor.find(
                ".//collision[@name='floor_collision']/geometry/box/size")
            dx, dy, _ = map(float, box.text.split())
            self.assertAlmostEqual(dx, 15.4+2*lights["venue"]["clearance_x"])
            self.assertAlmostEqual(dy, 7.2+2*lights["venue"]["clearance_y"])
            self.assertEqual(manifest["dataset_labels_created"],False)
            self.assertEqual(len(manifest["shuttles"]),7)
            self.assertEqual(json.loads(
                (Path(td)/"scene_manifest.json").read_text())["yolo"]["seed"],77)

    def test_seed_reproducibility_and_independent_camera_fps(self):
        lights = calibration()
        with tempfile.TemporaryDirectory() as td:
            first, one = world.build(modes.DEFAULT, DEFAULT, Path(td)/"a/world.sdf",
                                     lighting=lights)
            second, two = world.build(modes.DEFAULT, DEFAULT, Path(td)/"b/world.sdf",
                                      lighting=lights)
            self.assertEqual(one["shuttles"],two["shuttles"])
            self.assertEqual(one["camera"],modes.DEFAULT)
            self.assertEqual(two["yolo"],validate(DEFAULT))

    def test_run_uses_saved_yaml_only_and_raw_meter_not_apriltags(self):
        camera = {**modes.DEFAULT, "color":"1920x1080", "color_fps":30,
                  "depth_enabled":False}
        yolo = {**DEFAULT, "seed":111, "shuttle_count":3}
        def source(name):
            return {"camera":camera,"yolo":yolo}.get(name)
        calls = []
        with tempfile.TemporaryDirectory() as td:
            with (mock.patch.object(server,"OUTPUT",Path(td)),
                  mock.patch.object(server,"read_lab",side_effect=source)):
                lab = server.Lab()
                with mock.patch.object(
                    lab, "launch", side_effect=lambda name,cmd:calls.append((name,cmd))):
                    result = lab.start()
                self.assertEqual([name for name,_ in calls],
                    ["gazebo","image_bridge","meter","preview"])
                self.assertIn("gz",calls[0][1])
                self.assertEqual(calls[0][1][:3],["gz","sim","-r"])
                self.assertNotIn("apriltag",str(calls).lower())
                self.assertNotIn("yolo_shuttle_detector",str(calls).lower())
                self.assertEqual(result["camera"],camera)
                self.assertEqual(result["shuttles"],3)
                self.assertEqual(lab.environment()["ROS_DOMAIN_ID"],"83")
                self.assertTrue(str(Path(td)/"world.sdf") in calls[0][1])
                lab.stop()
            with (mock.patch.object(server,"OUTPUT",Path(td)),
                  mock.patch.object(server,"read_lab",
                                    side_effect=lambda _:None)):
                lab=server.Lab()
                with self.assertRaisesRegex(ValueError,"Save Camera Lab YAML"):
                    lab.start()

    def test_ui_tabs_and_browser_js_are_valid(self):
        page = (LAB/"index.html").read_text()
        for key in ("tabCamera","tabYolo","color","color_fps",
                    "depth","depth_fps","depth_enabled","saveCamera",
                    "shuttle_count","saveYolo","startScene",
                    "rgbWall","rgbSim","preview","toggleGui","lightingInfo"):
            self.assertIn(f'id="{key}"',page)
        self.assertIn('"/api/config/camera/save"',page)
        self.assertIn('"/api/config/yolo/save"',page)
        self.assertIn('"/api/scene/start"',page)
        self.assertIn("dataset_labels_created", (LAB/"world.py").read_text())
        self.assertNotIn('id="trainButton"',page)
        subprocess.run(["bash","-n",str(LAB/"start.sh")],check=True)
        if shutil.which("node"):
            script = page.split("<script>",1)[1].split("</script>",1)[0]
            with tempfile.TemporaryDirectory() as td:
                path = Path(td)/"ui.js"
                path.write_text(script)
                subprocess.run(["node","--check",str(path)],check=True)


if __name__=="__main__":
    unittest.main()
