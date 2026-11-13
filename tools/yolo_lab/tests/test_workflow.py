"""Headless integration contracts for migrated YOLO Lab workflow."""
from __future__ import annotations
import ast
import json
import pathlib
import sys
import tempfile
import unittest
from unittest import mock
from xml.etree import ElementTree as ET

ROOT=pathlib.Path(__file__).resolve().parents[3]
LAB=ROOT/"tools/yolo_lab"
sys.path[:0]=[str(ROOT/"tools"),str(ROOT/"tools/camera_lab"),
              str(LAB),str(LAB/"scripts")]
from modes import DEFAULT as CAMERA_DEFAULT
from yolo_settings import DEFAULT as YOLO_DEFAULT
from lighting_source import calibration
from dataset_world import build_dataset
import workflow
import inspect_dataset
from import_baselines import BLOBS, sha_git_blob, import_checkpoint


class FakeLab:
    def __init__(self):
        self.processes={}
        self.events=[]
        self.phase="idle"
        self.camera=None
        self.yolo=None
    def stop(self):
        self.events.append(("stop",[]))
        self.processes.clear()
        self.phase="stopped"
    def launch(self,name,args):
        self.events.append((name,list(args)))
    def _terminate(self,name):
        self.events.append(("terminate",name))
        self.processes.pop(name,None)
    def start(self):
        self.events.append(("start",[]))
        self.phase="starting"
        return {"started":True}


class WorkflowTests(unittest.TestCase):
    def test_triggered_camera_dataset_world_reuses_all_calibration(self):
        settings={**CAMERA_DEFAULT,"color":"1920x1080","color_fps":30,
                  "depth_enabled":False}
        lighting=calibration()
        with tempfile.TemporaryDirectory() as td:
            path=pathlib.Path(td)/"world.sdf"
            built=build_dataset(settings,YOLO_DEFAULT,path,lighting=lighting)
            self.assertEqual(built,str(path))
            scene=ET.parse(path).getroot().find("world")
            self.assertEqual(scene.get("name"),"yolo_dataset")
            self.assertEqual(len(scene.findall("light")),
                             lighting["lights"]["rows"]*lighting["lights"]["cols"])
            self.assertIsNone(scene.find("./model[@name='camera_rig']"))
            models=[x.get("name") for x in scene.findall("model")]
            self.assertNotIn("yolo_shuttle_000",models)
            self.assertIn("court_floor",models)
            self.assertIn("net_assembly",models)
            rig=scene.find("./model[@name='yolo_camera_rig']")
            self.assertIsNotNone(rig)
            sensor=rig.find("./link/sensor")
            self.assertEqual(sensor.findtext("update_rate"),"30")
            self.assertEqual(sensor.findtext("camera/triggered"),"true")
            self.assertEqual(sensor.findtext("camera/trigger_topic"),
                             "/yolo/camera/color/trigger")
            self.assertEqual(sensor.findtext("topic"),"/yolo/camera/color")
            self.assertEqual(sensor.findtext("camera/image/width"),"1920")
            self.assertEqual(sensor.findtext("camera/image/height"),"1080")
            self.assertEqual(sensor.findtext("camera/clip/far"),"20.0")
            self.assertEqual(len(scene.findall(".//sensor")),1)

    def test_dataset_start_launches_production_mesh_label_collector_without_ros_gui(self):
        fake=FakeLab()
        with tempfile.TemporaryDirectory() as td:
            place=pathlib.Path(td)
            w=workflow.Workflow(fake)
            with (mock.patch.object(workflow,"DATAROOT",place/"datasets"),
                  mock.patch.object(workflow.Workflow,"lab_output",
                                    new_callable=mock.PropertyMock,return_value=place),
                  mock.patch.object(workflow,"read_lab",
                      side_effect=lambda lab: CAMERA_DEFAULT if lab=="camera" else YOLO_DEFAULT),
                  mock.patch.object(workflow,"lighting_calibration",
                      return_value=calibration())):
                result=w.start_dataset()
                self.assertTrue(result["started"])
                self.assertEqual(result["images_target"],1200)
                self.assertEqual([n for n,_ in fake.events[:6]],
                    ["stop","gazebo","image_bridge","info_bridge","capture"])
                image_bridge=fake.events[2][1]
                self.assertIn("/camera/camera/color/image_raw"," ".join(image_bridge))
                capture=fake.events[-1][1]
                self.assertIn("dataset_capture.py", " ".join(capture))
                self.assertIn("target_images:=1200"," ".join(capture))
                self.assertIn("random_seed:=42"," ".join(capture))
                self.assertIn("status_json:="," ".join(capture))
                self.assertTrue((place/"dataset_world.sdf").exists())
                self.assertTrue((place/"datasets"/result["dataset"]).is_dir())

    def test_early_finish_preserves_capture_and_is_idempotent(self):
        fake = FakeLab()
        w = workflow.Workflow(fake)
        with tempfile.TemporaryDirectory() as td:
            directory = pathlib.Path(td)
            w.capture_directory = directory
            w.dataset = directory.name
            fake.processes["capture"] = mock.Mock(poll=lambda: None)
            response = w.cancel_dataset()
            self.assertTrue(response["stop_requested"])
            marker = directory/".finish_early.request"
            self.assertTrue(marker.is_file())
            self.assertEqual(w.stage, "finishing_early")
            self.assertNotIn(("terminate", "capture"), fake.events)
            self.assertTrue(w.stop_task("capture")["stop_requested"])
            fake.processes["capture"].poll = lambda: 0
            with self.assertRaisesRegex(ValueError, "No active capture"):
                w.cancel_dataset()

    def test_latest_label_preview_separate_from_inspector_previews(self):
        script = LAB/"scripts/dataset_capture.py"
        tree = ast.parse(script.read_text(),filename=str(script))
        names = [node.name for node in tree.body if isinstance(node,ast.FunctionDef)]
        self.assertIn("write_labeled_preview", names)
        source = script.read_text()
        self.assertIn('directory / "capture_preview"', source)
        self.assertIn('write_labeled_preview(bgr, boxes, self.output_dir, split, stem,',source)
        self.assertIn('self.finalize_partial_dataset(reason="early_stop")',source)
        self.assertIn('rclpy.try_shutdown()',source)
        self.assertIn('"last_timings_ms"',source)
        page=(LAB/"index.html").read_text()
        self.assertIn('id="capturePreview"',page)
        self.assertIn('id="startInspect"',page)
        self.assertNotIn('id="tabLabels"',page)
        self.assertNotIn('id="labelsPane"',page)
        self.assertIn('"/api/capture-preview?seq="',page)

    def test_model_validation_and_training_require_inspection_pass(self):
        fake=FakeLab()
        with tempfile.TemporaryDirectory() as td:
            root=pathlib.Path(td)
            data=root/"datasets"/"dataset_20261113T000000000000Z"
            data.mkdir(parents=True)
            (data/"data.yaml").write_text("names: {0: Shuttlecock}\n")
            (data/"dataset_summary.json").write_text(
                json.dumps({"training_ready":True}))
            w=workflow.Workflow(fake)
            with (mock.patch.object(workflow,"DATAROOT",root/"datasets"),
                  mock.patch.object(workflow.Workflow,"lab_output",
                       new_callable=mock.PropertyMock,return_value=root),
                  mock.patch.object(w,"model_path",return_value=root/"best.pt"),
                  mock.patch.object(workflow,"read_lab",return_value=YOLO_DEFAULT)):
                with self.assertRaisesRegex(ValueError,"Label Inspector"):
                    w.start_train("models/pretrained/best.pt","cpu")
                (data/"inspection.json").write_text(json.dumps({"passed":True}))
                trained=w.start_train("models/pretrained/best.pt","cpu")
                self.assertTrue(trained["started"])
                self.assertEqual(fake.events[-1][0],"train")
                argv=fake.events[-1][1]
                self.assertIn("train_finetune.py"," ".join(argv))
                self.assertIn(str(data/"data.yaml"),argv)
                self.assertIn("--artifacts-dir",argv)
                with self.assertRaisesRegex(ValueError,"already running"):
                    # Fake launch doesn't create a process; test bound via a fake.
                    fake.processes["train"]=mock.Mock(poll=lambda:None)
                    w.start_validate("models/pretrained/best.pt")
                fake.processes.clear()
                tested=w.start_validate("models/pretrained/best.pt",split="test",device="cpu")
                self.assertTrue(tested["started"])
                self.assertIn("validate_model.py"," ".join(fake.events[-1][1]))
                self.assertIn("--metrics-json",fake.events[-1][1])

    def test_original_validator_wrapper_produces_inspection_and_previews(self):
        with tempfile.TemporaryDirectory() as td:
            root=pathlib.Path(td)
            (root/"data.yaml").write_text("names: {0: Shuttlecock}\n")
            for split in ("train","val","test"):
                (root/"images"/split).mkdir(parents=True)
                (root/"labels"/split).mkdir(parents=True)
                (root/"images"/split/"a.jpg").write_bytes(b"JPEG")
                (root/"labels"/split/"a.txt").write_text(
                    "0 0.4 0.5 0.1 0.15\n" if split!="test" else "")
            (root/"preview").mkdir()
            (root/"preview"/"01_train_a.jpg").write_bytes(b"preview")
            done=mock.Mock(returncode=0,stdout="CHECK RESULT: STRUCTURAL PASS",
                            stderr="")
            with mock.patch.object(inspect_dataset.subprocess,"run",return_value=done):
                report=inspect_dataset.inspect(root,3)
            self.assertTrue(report["passed"])
            self.assertEqual(report["labels"],2)
            self.assertEqual(report["negative_images"],1)
            self.assertEqual(len(report["previews"]),1)
            self.assertTrue((root/"inspection.json").exists())

    def test_migrated_scripts_are_parseable_and_model_hashes_are_pinned(self):
        for p in (LAB/"scripts").glob("*.py"):
            ast.parse(p.read_text(),filename=str(p))
        self.assertIn("best.pt",BLOBS)
        self.assertEqual(len(BLOBS["best.pt"][1]),40)
        self.assertEqual(sha_git_blob(b""),"e69de29bb2d1d6434b8b29ae775ad8c2e48c5391")
        with self.assertRaises(ValueError):
            import_checkpoint("not-whitelisted.pt")


if __name__=="__main__":
    unittest.main()
