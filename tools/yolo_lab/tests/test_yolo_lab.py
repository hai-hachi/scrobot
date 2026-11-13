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
                self.assertEqual(s.findtext("static"), "false")
                self.assertEqual(s.findtext(
                    ".//visual/geometry/mesh/uri"),
                    "model://shuttle/meshes/shuttle.STL")
                self.assertIsNotNone(s.find(".//collision"))
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

    def test_spawn_modes_weights_and_static_freeze(self):
        from distribution import sample
        from settle import parse_poses
        for mode in ("single", "random", "cluster", "mixed"):
            cfg = validate({**DEFAULT, "spawn_mode": mode, "shuttle_count": 13})
            points = sample(cfg)
            self.assertEqual(len(points), 1 if mode == "single" else 13)
            self.assertEqual(points, sample(cfg))
        cfg = validate({**DEFAULT, "spawn_mode": "cluster", "shuttle_count": 7})
        with tempfile.TemporaryDirectory() as td:
            placements = {f"yolo_shuttle_{i:03d}":(0.0,0.0,0.05,0.0,0.7071,0.0,0.7071)
                          for i in range(7)}
            _, manifest = world.build(modes.DEFAULT, cfg, Path(td)/"static.sdf",
                                      lighting=calibration(), settled_poses=placements)
            self.assertTrue(manifest["settled_static"])
            doc = ET.parse(Path(td)/"static.sdf")
            self.assertEqual(len(doc.findall(".//model[@static='true']")), 0)
            shuttles = [m for m in doc.findall(".//model")
                        if m.get("name", "").startswith("yolo_shuttle_")]
            self.assertEqual(len(shuttles), 7)
            self.assertTrue(all(m.findtext("static") == "true" for m in shuttles))
        pose_payload = ('pose { name: "yolo_shuttle_000" '
                        'position { x: 1.5 y: 2 z: 0.08 } '
                        'orientation { w: 1 } }')
        self.assertAlmostEqual(
            parse_poses(pose_payload, {"yolo_shuttle_000"})["yolo_shuttle_000"][0],
            1.5)

    def test_spawn_geometry_is_sourced_from_production_yaml(self):
        import yaml
        from distribution import geometry, sample
        source = ROOT/"src/scrobot_simulation/config/shuttle_spawn.yaml"
        documented = yaml.safe_load(source.read_text(encoding="utf-8"))
        g = geometry()
        self.assertEqual(g["court"]["length"], documented["court"]["length"])
        self.assertEqual(g["court"]["width"], documented["court"]["width"])
        self.assertEqual(g["court"]["margin"], documented["court"]["margin"])
        self.assertEqual(g["cluster"]["sigma_x"], documented["cluster"]["sigma_x"])
        self.assertEqual(g["cluster"]["sigma_y"], documented["cluster"]["sigma_y"])
        self.assertEqual(g["cluster"]["line_x"],
                         list(documented["cluster"]["line_x"].values()))
        self.assertEqual(g["single"]["pitch"], documented["single"]["pitch"])
        self.assertEqual(g["spawn"]["height"], documented["spawn"]["height"])
        self.assertEqual(DEFAULT["cluster_weights"], documented["cluster"]["weights"])
        # Old saved lab copies of dimensions are accepted, but ignored.
        migrated = validate({**DEFAULT, "single_x": 5.0,
                             "cluster_sigma_y": 3.0,
                             "cluster_line_x": [0]*7})
        self.assertNotIn("single_x", migrated)
        self.assertNotIn("cluster_line_x", migrated)
        self.assertNotIn("cluster_sigma_y", migrated)
        self.assertEqual(sample({**migrated, "spawn_mode": "single"})[0][:2],
                         (documented["single"]["x"], documented["single"]["y"]))
        with tempfile.TemporaryDirectory() as td:
            custom = dict(documented)
            custom["single"] = {**custom["single"], "x": 1.2, "y": -0.5}
            custom["court"] = {**custom["court"], "length": 14.0}
            path = Path(td)/"spawn.yaml"
            path.write_text(yaml.safe_dump(custom))
            custom_geometry = geometry(path)
            self.assertEqual(custom_geometry["court"]["length"], 14.0)
            self.assertEqual(sample({**migrated,"spawn_mode":"single"},
                                    custom_geometry)[0][:2], (1.2,-0.5))

    def test_scene_layout_groups_only_editable_spawn_choices(self):
        page = (LAB/"index.html").read_text()
        start = page.index('<section class="card" id="shuttleSpawnCard">')
        robot = page.index('<h2>Robot placement</h2>')
        preview = page.index('<h2>Live simulated robot RGB</h2>')
        next_tab = page.index('<div id="datasetPane" hidden>')
        self.assertLess(start, robot)
        self.assertLess(robot, preview)
        self.assertLess(preview, next_tab)
        self.assertNotIn('<h2>Dataset, training and inference defaults</h2>', page)
        self.assertIn('id="scenePosePreview"', page)
        self.assertIn('id="spawnSourceInfo"', page)
        self.assertNotIn('id="cluster_sigma_x"', page)
        self.assertNotIn('id="cluster_line_x"', page)
        self.assertNotIn('id="single_x"', page)
        self.assertIn("renderSpawnGeometry(s.spawn_geometry)", page)

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

    def test_tunable_settings_live_in_respective_tabs(self):
        page=(LAB/"index.html").read_text(encoding="utf-8")
        scene=page.split('<div id="yoloPane" hidden>',1)[1].split(
            '<div id="datasetPane" hidden>',1)[0]
        dataset=page.split('<div id="datasetPane" hidden>',1)[1].split(
            '<div id="trainPane" hidden>',1)[0]
        train=page.split('<div id="trainPane" hidden>',1)[1].split(
            '<div id="validatePane" hidden>',1)[0]
        validation=page.split('<div id="validatePane" hidden>',1)[1].split(
            '<div id="livePane" hidden>',1)[0]
        live=page.split('<div id="livePane" hidden>',1)[1]
        for setting in ("dataset_images","positive_fraction"):
            self.assertIn(f'id="{setting}"',dataset)
            self.assertNotIn(f'id="{setting}"',scene)
        for setting in ("epochs","batch","imgsz"):
            self.assertIn(f'id="{setting}"',train)
            self.assertNotIn(f'id="{setting}"',scene)
        for setting in ("validation_imgsz","validation_batch"):
            self.assertIn(f'id="{setting}"',validation)
        for setting in ("confidence","live_imgsz"):
            self.assertIn(f'id="{setting}"',live)
        for button in ("saveDatasetSettings","saveTrainingSettings",
                       "saveValidationSettings","saveLiveSettings",
                       "renameDataset","deleteDataset","datasetDeleteConfirm"):
            self.assertIn(f'id="{button}"',page)
        self.assertNotIn("Dataset, training and inference defaults",page)
        for model in ("best.pt","yolo11n_shuttle.pt","yolo11s-ball.pt"):
            self.assertIn(f'value="{model}"',train)
        for route in ("/api/dataset/delete","/api/dataset/rename"):
            self.assertIn(route,page)

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
