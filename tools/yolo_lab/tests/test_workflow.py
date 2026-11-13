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
        self.assertRegex(source, r"""['"]last_timings_ms['"]\s*:\s*dict\(self\.last_timings\)""")
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

    def test_dataset_rename_preserves_internal_paths_and_inspection(self):
        import yaml
        fake = FakeLab()
        with tempfile.TemporaryDirectory() as td:
            root = pathlib.Path(td)
            datasets = root/"datasets"
            name = "dataset_20261114T100000123456Z"
            folder = datasets/name
            (folder/"images/train").mkdir(parents=True)
            (folder/"labels/train").mkdir(parents=True)
            (folder/"images/train/a.jpg").write_bytes(b"image")
            (folder/"labels/train/a.txt").write_text("0 0.5 0.5 0.1 0.1\\n")
            (folder/"data.yaml").write_text(
                yaml.safe_dump({"path":str(folder),"train":"images/train",
                                "val":"images/val","names":{0:"Shuttlecock"}}))
            (folder/"dataset_summary.json").write_text(
                json.dumps({"output_dir":str(folder),"training_ready":True}))
            (folder/"inspection.json").write_text(
                json.dumps({"dataset":str(folder),"passed":True}))
            w = workflow.Workflow(fake)
            w.dataset = name
            with (mock.patch.object(workflow,"DATAROOT",datasets),
                  mock.patch.object(workflow.Workflow,"lab_output",
                                    new_callable=mock.PropertyMock,return_value=root)):
                renamed = w.rename_dataset(name,"  720p Mixed Sample ")
                newname = "dataset_720p_Mixed_Sample"
                self.assertEqual(renamed["dataset"],newname)
                self.assertTrue(newname.startswith("dataset_"))
                self.assertEqual(w.rename_dataset(newname,"dataset_720p_Mixed_Sample")["renamed"],False)
                self.assertFalse(folder.exists())
                self.assertEqual(w.dataset,newname)
                newfolder=datasets/newname
                self.assertEqual(yaml.safe_load(
                    (newfolder/"data.yaml").read_text())["path"],str(newfolder))
                self.assertEqual(json.loads(
                    (newfolder/"dataset_summary.json").read_text())["output_dir"],
                    str(newfolder))
                self.assertEqual(json.loads(
                    (newfolder/"inspection.json").read_text())["dataset"],str(newfolder))
                self.assertTrue((newfolder/"images/train/a.jpg").is_file())
                self.assertEqual(w.datasets(),[newname])
                self.assertEqual(w.choose_dataset(newname)["dataset"],newname)
                with self.assertRaisesRegex(ValueError,"exact dataset identifier"):
                    w.delete_dataset(newname, "wrong")
                self.assertTrue(newfolder.exists())
                result=w.delete_dataset(newname,newname)
                self.assertEqual(result["deleted"],newname)
                self.assertFalse(newfolder.exists())

    def test_dataset_manager_rejects_traversal_symlinks_and_busy_tasks(self):
        fake=FakeLab()
        w=workflow.Workflow(fake)
        with tempfile.TemporaryDirectory() as td:
            root=pathlib.Path(td)
            dataset_dir=root/"datasets"
            name="dataset_20261114T120000123456Z"
            (dataset_dir/name).mkdir(parents=True)
            (dataset_dir/name/"data.yaml").write_text("path: .\\n")
            outside=root/"outside"
            outside.mkdir()
            symlink=dataset_dir/"dataset_20261114T120001123456Z"
            symlink.symlink_to(outside, target_is_directory=True)
            with mock.patch.object(workflow,"DATAROOT",dataset_dir):
                with self.assertRaisesRegex(ValueError,"Invalid dataset identifier"):
                    w.rename_dataset("../outside","bad")
                with self.assertRaisesRegex(ValueError,"managed YOLO dataset"):
                    w.delete_dataset(symlink.name,symlink.name)
                with self.assertRaisesRegex(ValueError,"Name must"):
                    w.rename_dataset(name,"unsafe/../name")
                with self.assertRaisesRegex(ValueError,"exact dataset identifier"):
                    w.delete_dataset(name,"DELETE")
                fake.processes["train"]=mock.Mock(poll=lambda:None)
                with self.assertRaisesRegex(ValueError,"already running"):
                    w.delete_dataset(name,name)
                fake.processes.clear()
                self.assertTrue((dataset_dir/name).is_dir())
                self.assertTrue(outside.is_dir())

    def test_separate_validation_and_live_model_labels(self):
        self.assertIn("YOLO11n", workflow.BASELINE_LABELS["yolo11n_shuttle.pt"])
        self.assertIn("ball", workflow.BASELINE_LABELS["yolo11s-ball.pt"])
        self.assertIn("unverified", workflow.BASELINE_LABELS["best.pt"])
        settings = {**YOLO_DEFAULT,"validation_imgsz":768,
                    "validation_batch":2,"live_imgsz":512}
        from yolo_settings import validate as checked
        self.assertEqual(checked(settings)["validation_imgsz"],768)
        fake=FakeLab()
        w=workflow.Workflow(fake)
        with tempfile.TemporaryDirectory() as td:
            root=pathlib.Path(td)
            folder=root/"datasets"/"dataset_20261114T120000123456Z"
            folder.mkdir(parents=True)
            (folder/"data.yaml").write_text("path: .\\n")
            (folder/"dataset_summary.json").write_text(
                json.dumps({"training_ready":True,"counts":{"val":{"images":2}}}))
            (folder/"inspection.json").write_text(json.dumps({"passed":True}))
            w.dataset=folder.name
            with (mock.patch.object(workflow,"DATAROOT",root/"datasets"),
                  mock.patch.object(workflow.Workflow,"lab_output",
                      new_callable=mock.PropertyMock,return_value=root),
                  mock.patch.object(workflow,"read_lab",return_value=settings),
                  mock.patch.object(w,"model_path",return_value=root/"baseline.pt")):
                w.start_validate("models/pretrained/baseline.pt",split="val")
                cmd=fake.events[-1][1]
                self.assertEqual(cmd[cmd.index("--imgsz")+1],"768")
                self.assertEqual(cmd[cmd.index("--batch")+1],"2")

    def test_simple_date_index_dataset_name_preserves_legacy(self):
        from datetime import datetime
        from zoneinfo import ZoneInfo
        local_day=datetime.now(ZoneInfo("Asia/Ho_Chi_Minh"))
        months=("Jan","Feb","Mar","Apr","May","Jun","Jul","Aug",
                "Sep","Oct","Nov","Dec")
        prefix=f"dataset_{months[local_day.month-1]}{local_day.day:02d}"
        with tempfile.TemporaryDirectory() as td:
            root=pathlib.Path(td)
            for suffix in ("01","02"):
                folder=root/(prefix+"_"+suffix)
                folder.mkdir()
                (folder/"data.yaml").write_text("path: .\\n")
            w=workflow.Workflow(FakeLab())
            with mock.patch.object(workflow,"DATAROOT",root):
                self.assertEqual(w._next_dataset_name(),prefix+"_03")
                self.assertIn(prefix+"_01",w.datasets())
                self.assertTrue(workflow.DATASET_ID.fullmatch(
                    "dataset_20261113T000000000000Z"))

    def test_protected_pretrains_and_managed_artifacts(self):
        with tempfile.TemporaryDirectory() as td:
            root=pathlib.Path(td)
            protected=root/"models/pretrains"
            custom=root/"models/trained"
            artifacts=root/"artifacts/models"
            for folder in (protected,custom,artifacts):
                folder.mkdir(parents=True,exist_ok=True)
            original=protected/"my_renamed_original.pt"
            original.write_bytes(b"protected checkpoint")
            trained=custom/"candidate.pt"
            trained.write_bytes(b"custom model")
            artifact=artifacts/"lab_1.pt"
            artifact.write_bytes(b"artifact checkpoint")
            pointer=root/"artifacts/latest_model.txt"
            pointer.write_text(str(artifact.resolve())+"\n")
            with (mock.patch.object(workflow,"HERE",root),
                  mock.patch.object(workflow,"MODELS",root/"models/pretrained"),
                  mock.patch.object(workflow,"PRETRAINS",protected),
                  mock.patch.object(workflow,"TRAINED",custom),
                  mock.patch.object(workflow,"ARTIFACTS",artifacts),
                  mock.patch.object(workflow,"RUNS",root/"runs/detect")):
                w=workflow.Workflow(FakeLab())
                found={m["id"]:m for m in w.models()}
                self.assertFalse(found["models/pretrains/my_renamed_original.pt"]["editable"])
                self.assertTrue(found["models/trained/candidate.pt"]["editable"])
                with self.assertRaisesRegex(ValueError,"Only managed"):
                    w.rename_model("models/pretrains/my_renamed_original.pt","bad")
                with self.assertRaisesRegex(ValueError,"Only managed"):
                    w.delete_models(["models/pretrains/my_renamed_original.pt"],True)
                renamed=w.rename_model("artifacts/models/lab_1.pt","shuttle_v2")
                self.assertEqual(renamed["model"],"artifacts/models/lab_shuttle_v2.pt")
                self.assertEqual(pointer.read_text().strip(),str(
                    (artifacts/"lab_shuttle_v2.pt").resolve()))
                w.delete_models(["artifacts/models/lab_shuttle_v2.pt",
                                 "models/trained/candidate.pt"],True)
                self.assertFalse(pointer.exists())
                self.assertTrue(original.is_file())

    def test_compare_models_use_same_split_without_concurrent_gpu_validation(self):
        fake=FakeLab()
        with tempfile.TemporaryDirectory() as td:
            root=pathlib.Path(td)
            folder=root/"datasets"/"dataset_Nov14_01"
            folder.mkdir(parents=True)
            (folder/"data.yaml").write_text("path: .\\n")
            (folder/"dataset_summary.json").write_text(
                json.dumps({"training_ready":True,"counts":{"test":{"images":2}}}))
            (folder/"inspection.json").write_text(json.dumps({"passed":True}))
            w=workflow.Workflow(fake)
            w.dataset=folder.name
            config={**YOLO_DEFAULT,"validation_imgsz":768,
                    "validation_batch":2,"learning_rate":0.0002}
            with (mock.patch.object(workflow,"DATAROOT",root/"datasets"),
                  mock.patch.object(workflow.Workflow,"lab_output",
                      new_callable=mock.PropertyMock,return_value=root),
                  mock.patch.object(workflow,"read_lab",return_value=config),
                  mock.patch.object(w,"model_path",
                      side_effect=lambda value:root/"weights"/pathlib.Path(value).name),
                  mock.patch.object(workflow.threading,"Thread") as fake_thread):
                models=["models/pretrains/A.pt","artifacts/models/B.pt"]
                result=w.start_compare_validation(models,"test","cpu")
                self.assertTrue(result["sequential"])
                self.assertEqual(fake_thread.call_count,1)
                commands=fake_thread.call_args.kwargs["args"][1]
                self.assertEqual([slot for slot,_ in commands],["a","b"])
                self.assertNotEqual(commands[0][1][-1],commands[1][1][-1])
                for _,cmd in commands:
                    self.assertEqual(cmd[cmd.index("--imgsz")+1],"768")
                    self.assertEqual(cmd[cmd.index("--batch")+1],"2")
                    self.assertEqual(cmd[cmd.index("--split")+1],"test")
                result=w.start_compare_live(models)
                self.assertTrue(result["started"])
                self.assertTrue(result["waiting_for_static_scene"])
                self.assertEqual(w.comparison["state"],"waiting_for_static_scene")
                # This worker only starts inference once the Gazebo scene
                # has fully settled and survived its static-world reload.
                import threading
                fake.lock = threading.RLock()
                fake.manifest = {"settled_static":True}
                fake.processes["gazebo"] = mock.Mock(poll=lambda:None)
                with mock.patch.object(workflow.time,"sleep",return_value=None):
                    w._wait_and_launch_live_pair(
                        w.compare_generation,
                        [root/"weights"/"A.pt",root/"weights"/"B.pt"],config)
                self.assertEqual(w.comparison["state"],"running")
                self.assertIn("live_a",[event[0] for event in fake.events])
                self.assertIn("live_b",[event[0] for event in fake.events])
                self.assertTrue(w.stop_comparison()["stopped"])

    def test_model_run_names_keep_lab_prefix_and_increment(self):
        from datetime import datetime
        from zoneinfo import ZoneInfo
        today=datetime.now(ZoneInfo("Asia/Ho_Chi_Minh"))
        months=("Jan","Feb","Mar","Apr","May","Jun","Jul",
                "Aug","Sep","Oct","Nov","Dec")
        prefix=f"lab_{months[today.month-1]}{today.day:02d}"
        with tempfile.TemporaryDirectory() as td:
            root=pathlib.Path(td)
            runs=root/"runs"
            artifacts=root/"artifacts"
            (runs/(prefix+"_01")).mkdir(parents=True)
            artifacts.mkdir()
            (artifacts/(prefix+"_02.pt")).write_bytes(b"checkpoint")
            with (mock.patch.object(workflow,"RUNS",runs),
                  mock.patch.object(workflow,"ARTIFACTS",artifacts)):
                self.assertEqual(workflow.Workflow(FakeLab())._next_model_run_name(),
                                 prefix+"_03")

    def test_train_flags_and_validation_plot_arguments(self):
        fake=FakeLab()
        with tempfile.TemporaryDirectory() as td:
            root=pathlib.Path(td)
            dataset=root/"datasets/dataset_Nov14_01"
            dataset.mkdir(parents=True)
            (dataset/"data.yaml").write_text("path: .\\n")
            (dataset/"dataset_summary.json").write_text(
                json.dumps({"training_ready":True,
                            "counts":{"test":{"images":2}}}))
            (dataset/"inspection.json").write_text(json.dumps({"passed":True}))
            cfg={**YOLO_DEFAULT,"optimizer":"SGD","learning_rate":0.0002,
                 "patience":11,"mosaic":0.4,"rect":True}
            w=workflow.Workflow(fake)
            w.dataset=dataset.name
            with (mock.patch.object(workflow,"DATAROOT",root/"datasets"),
                  mock.patch.object(workflow,"RUNS",root/"runs"),
                  mock.patch.object(workflow,"ARTIFACTS",root/"artifacts"),
                  mock.patch.object(workflow.Workflow,"lab_output",
                      new_callable=mock.PropertyMock,return_value=root),
                  mock.patch.object(workflow,"read_lab",return_value=cfg),
                  mock.patch.object(w,"model_path",return_value=root/"model.pt")):
                w.start_train("models/pretrains/custom.pt","cpu")
                cmd=fake.events[-1][1]
                for key,expected in (
                    ("--optimizer","SGD"),("--lr0","0.0002"),
                    ("--patience","11"),("--mosaic","0.4")):
                    self.assertEqual(cmd[cmd.index(key)+1],expected)
                self.assertIn("--rect",cmd)
                self.assertTrue(w.run_name.startswith("lab_"))
                fake.events.clear()
                w.start_validate("models/pretrains/custom.pt","test","cpu")
                self.assertIn("--metrics-json",fake.events[-1][1])
                # Sequential comparison produces real plot directory args.
                with mock.patch.object(workflow.threading,"Thread") as fake_thread:
                    w.start_compare_validation(
                        ["models/pretrains/A.pt","models/pretrains/B.pt"],
                        "test","cpu")
                    commands=fake_thread.call_args.kwargs["args"][1]
                    for slot,cmd in commands:
                        self.assertEqual(
                            cmd[cmd.index("--plots-dir")+1],
                            str(root/"compare"/("plots_"+slot)))
                        self.assertIn("--metrics-json",cmd)

    def test_trainer_and_validator_enable_real_ultralytics_plots(self):
        trainer=(LAB/"scripts/train_finetune.py").read_text()
        validator=(LAB/"scripts/validate_model.py").read_text()
        self.assertIn("mosaic=args.mosaic",trainer)
        self.assertIn("rect=args.rect",trainer)
        self.assertIn('"--plots-dir"',validator)
        self.assertIn('"plots": True',validator)
        for expected in ("confusion_matrix.png","confusion_matrix_normalized.png",
                         "BoxPR_curve.png","BoxF1_curve.png",
                         "BoxP_curve.png","BoxR_curve.png"):
            self.assertIn(expected,validator)
        server=(LAB/"server.py").read_text()
        self.assertIn('route == "/api/compare-plot"',server)
        self.assertIn('kind not in plot_files',server)

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
