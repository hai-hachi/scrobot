"""YOLO Lab stage controller. No dataset/model task runs in the HTTP thread."""
from __future__ import annotations

from datetime import datetime, timezone
from zoneinfo import ZoneInfo
import json
import os
from pathlib import Path
import re
import shutil
import signal
import sys
import threading
import time

import yaml

from dataset_world import build_dataset
from lab_configs import read as read_lab
from modes import validate as validate_camera
from yolo_settings import validate as validate_yolo
from lighting_source import calibration as lighting_calibration
from log_text import tail as clean_tail

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SCRIPTS = HERE / "scripts"
MODELS = HERE / "models/pretrained"
PRETRAINS = HERE / "models/pretrains"  # User-renamed immutable originals
TRAINED = HERE / "models/trained"
ARTIFACTS = HERE / "artifacts/models"
RUNS = HERE / "runs/detect"
DATAROOT = ROOT / "evaluation_results/yolo_lab/datasets"
DATASET_ID = re.compile(
    r"^(?:dataset_[0-9]{8}T[0-9]{12}Z|[A-Z][a-z]{2}(?:0[1-9]|[12][0-9]|3[01])_[0-9]{2,3})(?:__[a-z0-9][a-z0-9_-]{0,39})?$"
)
DATASET_SLUG = re.compile(r"^[a-z0-9][a-z0-9_-]{0,39}$")
MODEL_CHOICE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,128}$")
BASELINE_LABELS = {
    "best.pt": "Legacy YOLO shuttle baseline (best.pt; architecture unverified)",
    "yolo11n_shuttle.pt": "YOLO11n Nano - shuttlecock detector",
    "yolo11s-ball.pt": "YOLO11s Small - ball detector (not shuttle-specific)",
}



def _timestamp():
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")


def _read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return {}


def _tail(path, max_bytes=3500):
    path = Path(path)
    if not path.is_file():
        return ""
    try:
        with path.open("rb") as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell()-max_bytes))
            return clean_tail(path, max_bytes)
    except OSError:
        return ""


class Workflow:
    def __init__(self, lab):
        self.lab = lab
        self.dataset = None
        self.stage = "idle"
        self.selected_model = None
        self.run_name = None
        self.capture_directory = None
        self.comparison = None
        self.compare_generation = 0

    def datasets(self):
        if not DATAROOT.exists():
            return []
        return [p.name for p in sorted(
            (p for p in DATAROOT.iterdir()
             if p.is_dir() and not p.is_symlink() and DATASET_ID.fullmatch(p.name)
             and (p / "data.yaml").is_file()),
            key=lambda p: p.stat().st_mtime, reverse=True)][:100]

    def _next_dataset_name(self):
        # Use workstation location's calendar date; never depend on UTC day.
        prefix = datetime.now(ZoneInfo("Asia/Ho_Chi_Minh")).strftime("%b%d")
        occupied = {p.name.split("__", 1)[0]
                    for p in DATAROOT.iterdir()} if DATAROOT.is_dir() else set()
        for number in range(1, 1000):
            candidate = f"{prefix}_{number:02d}"
            if candidate not in occupied:
                return candidate
        raise ValueError("Too many datasets generated on this calendar day")

    def choose_dataset(self, name):
        if not isinstance(name, str) or name not in self.datasets():
            raise ValueError("Choose a generated dataset shown in the Dataset tab")
        self.dataset = name
        return {"dataset": name}

    def _managed_dataset(self, name):
        """Reject traversal, symlinked datasets, and unindexed folders."""
        if not isinstance(name, str) or not DATASET_ID.fullmatch(name):
            raise ValueError("Invalid dataset identifier")
        root = DATAROOT.resolve()
        target = DATAROOT / name
        if (name not in self.datasets() or target.is_symlink()
                or not target.is_dir() or target.resolve().parent != root):
            raise ValueError("Dataset is not a managed YOLO dataset")
        return target

    @staticmethod
    def _write_atomic(path, raw):
        import tempfile
        path = Path(path)
        if path.is_symlink():
            raise ValueError("Dataset metadata must not be a symlink")
        temp = None
        try:
            with tempfile.NamedTemporaryFile(dir=path.parent, prefix=".metadata_",
                                             delete=False) as stream:
                temp = Path(stream.name)
                stream.write(raw)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp, path)
            temp = None
        finally:
            if temp is not None:
                temp.unlink(missing_ok=True)

    def rename_dataset(self, name, new_name):
        """Rename a saved dataset directory, keeping its timestamp and links."""
        self._ensure_idle()
        old = self._managed_dataset(name)
        if not isinstance(new_name, str):
            raise ValueError("Enter a dataset name")
        slug = re.sub(r"[-_]+", "_",
                      re.sub(r"\s+", "_", new_name.strip().lower()))
        if not DATASET_SLUG.fullmatch(slug):
            raise ValueError("Name must use 1-40 lowercase letters, numbers, _ or -")
        base = name.split("__", 1)[0]
        proposed = base + "__" + slug
        new = DATAROOT / proposed
        if new == old:
            return {"renamed": False, "dataset": name}
        if new.exists() or new.is_symlink():
            raise ValueError("A dataset with this name already exists")
        old_path, new_path = str(old), str(new)

        # Read and validate metadata before moving anything. These files are
        # maintained by this lab and must not point through symlinks.
        replacements = {}
        yaml_path = old / "data.yaml"
        if yaml_path.is_symlink():
            raise ValueError("Dataset data.yaml must not be a symlink")
        data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError("Invalid dataset data.yaml")
        data["path"] = new_path
        replacements["data.yaml"] = yaml.safe_dump(
            data, sort_keys=False).encode("utf-8")
        for filename, field in (("dataset_summary.json", "output_dir"),
                                ("inspection.json", "dataset")):
            source = old / filename
            if source.exists():
                if source.is_symlink():
                    raise ValueError("Dataset metadata must not be a symlink")
                doc = json.loads(source.read_text(encoding="utf-8"))
                if not isinstance(doc, dict):
                    raise ValueError("Invalid dataset metadata: " + filename)
                doc[field] = new_path
                replacements[filename] = (
                    json.dumps(doc, indent=2) + "\n").encode("utf-8")
        originals = {name: (old / name).read_bytes() for name in replacements}
        old.rename(new)
        try:
            for filename, payload in replacements.items():
                self._write_atomic(new / filename, payload)
        except Exception:
            for filename, raw in originals.items():
                self._write_atomic(new / filename, raw)
            new.rename(old)
            raise

        if self.dataset == name:
            self.dataset = proposed
        if self.capture_directory == old:
            self.capture_directory = new
        # Capture progress is a disposable process-status cache, not the
        # authoritative saved dataset summary.
        progress_path = self.lab_output / "capture_progress.json"
        progress = _read_json(progress_path)
        if progress.get("output_dir") == old_path:
            progress["output_dir"] = new_path
            self._write_atomic(progress_path, (
                json.dumps(progress) + "\n").encode("utf-8"))
        return {"renamed": True, "previous": name, "dataset": proposed,
                "directory": new_path}

    def delete_datasets(self, names, confirmation):
        """Validate the complete selection before deleting any dataset."""
        self._ensure_idle()
        if confirmation is not True or not isinstance(names, list) or (
                not 1 <= len(names) <= 100) or len(set(map(str, names))) != len(names):
            raise ValueError("Select datasets and confirm deletion")
        targets = [self._managed_dataset(n) for n in names]
        for target in targets:
            # Recheck the target is still managed at the time of deletion.
            if target.is_symlink() or target.resolve().parent != DATAROOT.resolve():
                raise ValueError("Unsafe dataset folder")
        for target in targets:
            shutil.rmtree(target)
        if self.dataset in names:
            self.dataset = None
        if self.capture_directory in targets:
            self.capture_directory = None
        self.current_dataset()
        return {"deleted": names, "selected_dataset": self.dataset}

    def delete_dataset(self, name, confirmation):
        """Permanently remove a selected generated dataset, never a parent."""
        self._ensure_idle()
        target = self._managed_dataset(name)
        if confirmation != name:
            raise ValueError("Type the exact dataset identifier to confirm deletion")
        shutil.rmtree(target)
        if self.dataset == name:
            self.dataset = None
        if self.capture_directory == target:
            self.capture_directory = None
        self.current_dataset()
        return {"deleted": name, "selected_dataset": self.dataset}

    def current_dataset(self):
        choices = self.datasets()
        if self.dataset not in choices:
            self.dataset = choices[0] if choices else None
        return DATAROOT / self.dataset if self.dataset else None

    def _managed_model(self, choice):
        """Only user-trained/copied models can be changed; pretrains are immutable."""
        if not isinstance(choice, str) or choice not in {
                m["id"] for m in self.models() if m["editable"]}:
            raise ValueError("Only managed trained/artifact models may be changed")
        target = HERE / choice
        if (target.is_symlink() or not target.is_file()
                or target.resolve().parent not in
                (ARTIFACTS.resolve(), TRAINED.resolve())):
            raise ValueError("Cannot modify original or unmanaged models")
        return target

    def rename_model(self, choice, new_name):
        self._ensure_idle()
        target = self._managed_model(choice)
        if not isinstance(new_name, str):
            raise ValueError("Enter a new model name")
        stem = new_name.strip().removesuffix(".pt")
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,79}", stem):
            raise ValueError("Use 1-80 letters, digits, underscore or hyphen")
        destination = target.with_name(stem + ".pt")
        if destination == target:
            return {"renamed":False,"model":choice}
        if destination.exists() or destination.is_symlink():
            raise ValueError("A model with that name already exists")
        target.rename(destination)
        if self.selected_model == choice:
            self.selected_model = str(destination.relative_to(HERE))
        pointer = HERE / "artifacts/latest_model.txt"
        if pointer.is_file() and not pointer.is_symlink():
            if pointer.read_text(encoding="utf-8").strip() == str(target.resolve()):
                pointer.write_text(str(destination.resolve()) + "\n", encoding="utf-8")
        return {"renamed":True, "previous":choice,
                "model":str(destination.relative_to(HERE))}

    def delete_models(self, choices, confirmation):
        self._ensure_idle()
        if confirmation is not True or not isinstance(choices,list) or (
                not 1 <= len(choices) <= 100) or len(set(map(str,choices))) != len(choices):
            raise ValueError("Select models and confirm deletion")
        targets = [self._managed_model(c) for c in choices]
        if self.selected_model in choices:
            self.selected_model = None
        pointer = HERE / "artifacts/latest_model.txt"
        pointed = pointer.read_text(encoding="utf-8").strip() if pointer.is_file() and not pointer.is_symlink() else ""
        for target in targets:
            target.unlink()
        if pointed in [str(p.resolve()) for p in targets]:
            pointer.unlink(missing_ok=True)
        return {"deleted":choices}

    def models(self):
        files = []
        for directory in (PRETRAINS, MODELS, TRAINED, ARTIFACTS):
            if directory.is_dir():
                files.extend(sorted(directory.glob("*.pt")))
        if RUNS.is_dir():
            files.extend(sorted(RUNS.glob("*/weights/best.pt")))
        return [{"id": str(p.relative_to(HERE)),
                 "name": (BASELINE_LABELS.get(p.name, p.stem)
                          if p.parent in (MODELS, PRETRAINS)
                          else p.stem),
                 "group": ("Original pretrains (protected)" if p.parent in (PRETRAINS, MODELS)
                           else "Managed trained models" if p.parent in (TRAINED, ARTIFACTS)
                           else "Training-run weights (read-only)"),
                 "editable": p.parent in (TRAINED, ARTIFACTS),
                 "bytes": p.stat().st_size}
                for p in files if p.is_file() and not p.is_symlink()
                and HERE.resolve() in p.resolve().parents]

    def model_path(self, choice):
        if not isinstance(choice, str):
            raise ValueError("Select a model from the model picker")
        available = {entry["id"] for entry in self.models()}
        if choice not in available:
            raise ValueError("Model not found. Put pretrained .pt files into models/pretrains or train a model")
        file = (HERE / choice).resolve()
        if file.suffix != ".pt" or HERE.resolve() not in file.parents:
            raise ValueError("Invalid YOLO model selection")
        return file

    def _ensure_idle(self):
        for name in ("capture", "inspect", "train", "validate", "validate_a", "validate_b", "import", "live_a", "live_b", "live"):
            p = self.lab.processes.get(name)
            if p is not None and p.poll() is None:
                raise ValueError(f"{name} is already running; finish or stop it first")

    def start_dataset(self):
        self._ensure_idle()
        selected_camera = read_lab("camera")
        selected_yolo = read_lab("yolo")
        if selected_camera is None or selected_yolo is None:
            raise ValueError("Save Camera Lab YAML and YOLO Lab YAML first")
        camera = validate_camera(selected_camera)
        yolo = validate_yolo(selected_yolo)
        self.lab.stop()
        run_id = self._next_dataset_name()
        directory = DATAROOT / run_id
        directory.mkdir(parents=True, exist_ok=False)
        self.capture_directory = directory
        self.dataset = run_id
        self.stage = "dataset"
        (self.lab_output / "capture_progress.json").unlink(missing_ok=True)
        try:
            generated = build_dataset(
                camera, yolo, self.lab_output / "dataset_world.sdf",
                lighting=lighting_calibration())
            self.lab.camera = camera
            self.lab.yolo = yolo
            self.lab.phase = "generating_dataset"
            self.lab.launch("gazebo", [
                "gz", "sim", "-r", "-s", "-v", "3", generated])
            self.lab.launch("image_bridge", [
                "ros2", "run", "ros_gz_image", "image_bridge",
                "/yolo/camera/color", "--ros-args",
                "-p", "qos:=sensor_data",
                "-r", "/yolo/camera/color:=/camera/camera/color/image_raw"])
            self.lab.launch("info_bridge", [
                "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                "--ros-args", "-p",
                f"config_file:={HERE/'assets/yolo_dataset_bridge.yaml'}"])
            capture = [
                sys.executable, str(SCRIPTS / "dataset_capture.py"),
                "--ros-args", "-p", "use_sim_time:=true",
            ]
            args = {
                "output_dir":str(directory),
                "target_images":int(yolo["dataset_images"]),
                "shuttle_count":int(yolo["shuttle_count"]),
                "positive_pose_fraction":float(yolo["positive_fraction"]),
                "random_seed":int(yolo["seed"]),
                "status_json":str(self.lab_output / "capture_progress.json"),
            }
            for key, value in args.items():
                capture.extend(["-p", f"{key}:={value}"])
            self.lab.launch("capture", capture)
        except Exception:
            self.lab.stop()
            self.stage = "failed"
            raise
        return {"started": True, "dataset": run_id,
                "output_dir": str(directory), "images_target": yolo["dataset_images"],
                "camera": camera, "lighting_source": "saved lighting_lab.yaml"}

    @property
    def lab_output(self):
        return ROOT / "evaluation_results/yolo_lab"

    def _early_finish_fallback(self, process, directory):
        """SIGINT fallback if Gazebo/sim-time prevents the collector timer firing."""
        time.sleep(15)
        if (self.lab.processes.get("capture") is not process
                or process.poll() is not None
                or not (directory / ".finish_early.request").exists()
                or (directory / "dataset_summary.json").exists()):
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            pass

    def cancel_dataset(self):
        """Request a clean early finish, preserving complete image/label pairs."""
        process = self.lab.processes.get("capture")
        if process is None or process.poll() is not None:
            raise ValueError("No active capture to finish")
        directory = self.capture_directory
        if directory is None or not directory.is_dir():
            raise ValueError("Active capture directory is missing")
        marker = directory / ".finish_early.request"
        if not marker.exists():
            pending = directory / ".finish_early.request.tmp"
            pending.write_text("Finish after the current image-label transaction\\n",
                               encoding="utf-8")
            pending.replace(marker)
            # The normal path finishes inside capture_step; the fallback is
            # only for a frozen sim clock or a blocked ROS callback.
            threading.Thread(
                target=self._early_finish_fallback, args=(process, directory),
                daemon=True, name="YoloCaptureFinishFallback").start()
        self.stage = "finishing_early"
        return {"stop_requested": True, "dataset": directory.name,
                "message": "Finishing current image/label pair and finalizing partial dataset"}

    def start_inspect(self, sample_count=24):
        self._ensure_idle()
        if type(sample_count) is not int or not 1<=sample_count<=100:
            raise ValueError("Label preview count must be from 1 to 100")
        directory = self.current_dataset()
        if directory is None:
            raise ValueError("Generate or select a dataset first")
        summary = _read_json(directory / "dataset_summary.json")
        if not summary or not summary.get("training_ready"):
            raise ValueError("Dataset generation must finish with nonempty train and val splits")
        # Regenerating previews invalidates previous inspector approval.
        (directory / "inspection.json").unlink(missing_ok=True)
        self.lab._terminate("inspect")
        self.lab.launch("inspect", [
            sys.executable, str(SCRIPTS / "inspect_dataset.py"),
            "--dataset", str(directory), "--samples", str(sample_count)])
        self.stage = "inspect"
        return {"started": True, "dataset": self.dataset, "samples": sample_count}

    def _approved_dataset(self):
        directory = self.current_dataset()
        if directory is None:
            raise ValueError("Select a dataset first")
        inspection = _read_json(directory / "inspection.json")
        if inspection.get("passed") is not True:
            raise ValueError("Run Label Inspector successfully before using this dataset")
        summary = _read_json(directory / "dataset_summary.json")
        if summary.get("training_ready") is not True:
            raise ValueError("Dataset train/val splits are empty or collection unfinished")
        return directory

    def start_train(self, model_choice, device="0"):
        self._ensure_idle()
        dataset = self._approved_dataset()
        model = self.model_path(model_choice)
        if not isinstance(device,str) or device not in ("0", "cpu"):
            raise ValueError("Training device must be 0 (CUDA) or cpu")
        yolo = validate_yolo(read_lab("yolo") or {})
        self.lab.stop()  # avoid Gazebo taking GPU capacity during training
        self.run_name = "lab_" + _timestamp()
        self.selected_model = model_choice
        cmd = [self.training_python, str(SCRIPTS / "train_finetune.py"),
               "--model", str(model), "--data", str(dataset / "data.yaml"),
               "--device", device, "--epochs", str(yolo["epochs"]),
               "--batch", str(yolo["batch"]), "--imgsz", str(yolo["imgsz"]),
               "--optimizer", "AdamW", "--lr0", str(yolo["learning_rate"]),
               "--patience", "8",
               "--name", self.run_name,
               "--project", str(RUNS),
               "--artifacts-dir", str(ARTIFACTS)]
        self.lab.launch("train", cmd)
        self.lab.phase = "training"
        self.stage = "train"
        return {"started": True, "run": self.run_name,
                "dataset": self.dataset, "model": str(model)}

    def stop_task(self, name):
        if name not in ("capture", "inspect", "train", "validate", "live", "import"):
            raise ValueError("Unsupported task to stop")
        if name == "capture":
            return self.cancel_dataset()
        self.lab._terminate(name)
        if name == "live":
            for item in ("live.jpg","live_status.json"):
                (self.lab_output / "live" / item).unlink(missing_ok=True)
        self.stage = name + "_stopped"
        return {"stopped": True, "task": name}

    def start_validate(self, model_choice, split="test", device="0"):
        self._ensure_idle()
        dataset = self._approved_dataset()
        model = self.model_path(model_choice)
        if split not in ("val","test") or device not in ("0","cpu"):
            raise ValueError("Invalid dataset split or device")
        counts = _read_json(dataset / "dataset_summary.json").get("counts", {})
        if counts and (counts.get(split) or {}).get("images", 0) < 1:
            raise ValueError(
                f"No {split} images exist in this dataset. Capture more varied views "
                "or choose another split.")
        self.lab.stop()
        report = self.lab_output / "validation_metrics.json"
        report.unlink(missing_ok=True)
        yolo = validate_yolo(read_lab("yolo") or {})
        self.lab.launch("validate", [
            self.training_python, str(SCRIPTS / "validate_model.py"),
            str(model), "--data", str(dataset / "data.yaml"),
            "--device",device, "--imgsz",str(yolo["validation_imgsz"]),
            "--batch",str(yolo["validation_batch"]), "--split",split,
            "--metrics-json",str(report)])
        self.lab.phase = "validating"
        self.stage = "validate"
        self.selected_model = model_choice
        return {"started": True, "model": str(model), "split": split}

    def _validate_pair_worker(self, generation, commands):
        """Evaluate A then B sequentially to keep GPU use bounded."""
        try:
            for slot, cmd in commands:
                with self.lab.lock:
                    if generation != self.compare_generation or self.lab.phase != "comparing_models":
                        if generation == self.compare_generation and self.comparison:
                            self.comparison["state"] = "stopped"
                        return
                    proc = self.lab.launch("validate_" + slot, cmd)
                    self.comparison["state"] = "running_" + slot
                code = proc.wait()
                with self.lab.lock:
                    if generation != self.compare_generation:
                        return
                    self.lab._terminate("validate_" + slot)
                    if code != 0:
                        self.comparison["state"] = "failed_" + slot
                        self.comparison["error"] = (
                            "Model " + slot.upper() + " evaluation exited " + str(code))
                        return
                    self.comparison["state"] = "completed_" + slot
            with self.lab.lock:
                if generation == self.compare_generation:
                    self.comparison["state"] = "complete"
        except Exception as exc:
            with self.lab.lock:
                if generation == self.compare_generation and self.comparison:
                    self.comparison["state"] = "failed"
                    self.comparison["error"] = str(exc)

    def start_compare_validation(self, choices, split="test", device="0"):
        self._ensure_idle()
        if not isinstance(choices, list) or len(choices) != 2 or choices[0] == choices[1]:
            raise ValueError("Pick two different models for comparison")
        dataset = self._approved_dataset()
        models = [self.model_path(choice) for choice in choices]
        if split not in ("test", "val") or device not in ("0", "cpu"):
            raise ValueError("Invalid comparison split or device")
        counts = _read_json(dataset / "dataset_summary.json").get("counts", {})
        if counts and (counts.get(split) or {}).get("images", 0) < 1:
            raise ValueError("Selected dataset split has no images")
        self.lab.stop()
        self.lab.phase = "comparing_models"
        self.compare_generation += 1
        generation = self.compare_generation
        settings = validate_yolo(read_lab("yolo") or {})
        output = self.lab_output / "compare"
        output.mkdir(parents=True, exist_ok=True)
        commands = []
        for slot, model in zip(("a", "b"), models):
            path = output / ("validation_" + slot + ".json")
            path.unlink(missing_ok=True)
            commands.append((slot, [
                self.training_python, str(SCRIPTS / "validate_model.py"),
                str(model), "--data", str(dataset / "data.yaml"),
                "--device", device,
                "--imgsz", str(settings["validation_imgsz"]),
                "--batch", str(settings["validation_batch"]),
                "--split", split, "--metrics-json", str(path)
            ]))
        self.comparison = {"mode":"validation", "state":"starting",
                           "models":choices, "split":split, "dataset":str(dataset),
                           "error":None}
        self.stage = "compare_validation"
        threading.Thread(target=self._validate_pair_worker,
                         args=(generation,commands),
                         daemon=True,name="YoloCompareValidation").start()
        return {"started":True, "models":choices, "split":split, "sequential":True}

    def _wait_and_launch_live_pair(self, generation, models, settings):
        """Wait for static-scene reload so it cannot kill live detector nodes."""
        try:
            for _ in range(120):
                time.sleep(0.5)
                with self.lab.lock:
                    if generation != self.compare_generation:
                        return
                    if self.lab.phase in ("stopped","failed"):
                        self.comparison.update(state="failed",error="Gazebo stopped")
                        return
                    gazebo = self.lab.processes.get("gazebo")
                    ready = (getattr(self.lab,"manifest",None) is not None
                             and self.lab.manifest.get("settled_static") is True
                             and gazebo is not None and gazebo.poll() is None)
                    if not ready:
                        continue
                    try:
                        for slot, model in zip(("a","b"), models):
                            folder = self.lab_output / "compare" / ("live_"+slot)
                            folder.mkdir(parents=True,exist_ok=True)
                            for file in ("live.jpg","live_status.json"):
                                (folder / file).unlink(missing_ok=True)
                            self.lab.launch("live_"+slot, [
                                sys.executable,str(SCRIPTS/"live_detection.py"),
                                "--model",str(model),"--output",str(folder),
                                "--confidence",str(settings["confidence"]),
                                "--imgsz",str(settings["live_imgsz"]),
                                "--max-rate","8"])
                    except Exception:
                        for slot in ("a","b"):
                            self.lab._terminate("live_"+slot)
                        raise
                    self.comparison["state"] = "running"
                    return
            with self.lab.lock:
                if generation == self.compare_generation:
                    self.comparison.update(state="failed",error="Static scene not ready")
        except Exception as exc:
            with self.lab.lock:
                if generation == self.compare_generation:
                    self.comparison.update(state="failed",error=str(exc))

    def start_compare_live(self, choices):
        self._ensure_idle()
        if not isinstance(choices,list) or len(choices)!=2 or choices[0]==choices[1]:
            raise ValueError("Pick two different models for comparison")
        models=[self.model_path(choice) for choice in choices]
        gazebo=self.lab.processes.get("gazebo")
        if gazebo is None or gazebo.poll() is not None or self.lab.phase=="generating_dataset":
            self.lab.start()
        for name in ("live","live_a","live_b"):
            self.lab._terminate(name)
        self.compare_generation+=1
        generation=self.compare_generation
        settings=validate_yolo(read_lab("yolo") or {})
        self.comparison={"mode":"live","state":"waiting_for_static_scene",
                         "models":choices,"error":None}
        self.stage="compare_live"
        threading.Thread(target=self._wait_and_launch_live_pair,
                         args=(generation,models,settings),daemon=True,
                         name="YoloCompareLive").start()
        return {"started":True,"models":choices,"waiting_for_static_scene":True,
                "max_rate_per_model":8}

    def stop_comparison(self):
        self.compare_generation += 1
        for name in ("validate_a","validate_b","live_a","live_b"):
            self.lab._terminate(name)
        if self.comparison:
            self.comparison["state"]="stopped"
        self.stage="comparison_stopped"
        return {"stopped":True}

    def start_live(self, model_choice):
        self._ensure_idle()
        model = self.model_path(model_choice)
        current = self.lab.processes.get("gazebo")
        # Triggered dataset camera does not stream continuously. Restart with
        # the saved inspection camera if the current scene is a dataset world.
        if current is None or current.poll() is not None or self.lab.phase == "generating_dataset":
            self.lab.start()
        self.lab._terminate("live")
        live_dir = self.lab_output / "live"
        live_dir.mkdir(parents=True, exist_ok=True)
        for name in ("live_status.json", "live.jpg"):
            (live_dir / name).unlink(missing_ok=True)
        settings = validate_yolo(read_lab("yolo") or {})
        self.lab.launch("live", [
            sys.executable, str(SCRIPTS / "live_detection.py"),
            "--model",str(model), "--output",str(live_dir),
            "--confidence",str(settings["confidence"]),
            "--imgsz",str(settings["live_imgsz"]), "--max-rate","15"])
        self.stage = "live"
        self.selected_model = model_choice
        return {"started": True,"model":str(model)}

    def import_model(self, name):
        self._ensure_idle()
        if name not in ("best.pt","yolo11n_shuttle.pt","yolo11s-ball.pt"):
            raise ValueError("Choose one of the three original checkpoints")
        self.lab._terminate("import")
        self.lab.launch("import", [
            sys.executable, str(SCRIPTS / "import_baselines.py"),name])
        self.stage = "import"
        return {"started": True,"model":name}

    @property
    def training_python(self):
        path=os.environ.get("SCROBOT_YOLO_PYTHON",sys.executable)
        if not Path(path).is_file():
            raise ValueError("SCROBOT_YOLO_PYTHON must point to an existing Python executable")
        return path

    def progress(self):
        directory=self.current_dataset()
        status = _read_json(self.lab_output / "capture_progress.json")
        # This file describes only the most recent collector process.
        # When inspecting an older dataset, never mix its summary with
        # the live progress of a different dataset.
        if directory is None or status.get("output_dir") != str(directory):
            status = {}
        summary = _read_json(directory/"dataset_summary.json") if directory else {}
        inspection = _read_json(directory/"inspection.json") if directory else {}
        capture_preview = _read_json(directory/"capture_preview"/"latest.json") if directory else {}
        live = _read_json(self.lab_output/"live/live_status.json")
        metrics = _read_json(self.lab_output/"validation_metrics.json")
        comparison = dict(self.comparison) if self.comparison else {}
        cmp_root = self.lab_output / "compare"
        comparison["validation"] = {
            slot:_read_json(cmp_root/("validation_"+slot+".json"))
            for slot in ("a","b")
        }
        comparison["live"] = {
            slot:_read_json(cmp_root/("live_"+slot)/"live_status.json")
            for slot in ("a","b")
        }
        tasks={}
        for name in ("capture","inspect","train","validate","live","import","validate_a","validate_b","live_a","live_b"):
            proc=self.lab.processes.get(name)
            tasks[name]= {
                "running":proc is not None and proc.poll() is None,
                "exit_code":proc.poll() if proc is not None else None,
            }
        if tasks["capture"]["running"] is False and tasks["capture"]["exit_code"] == 0 and summary:
            status["state"]="COMPLETE"
        training={}
        if self.run_name:
            run=RUNS/self.run_name
            csv=run/"results.csv"
            if csv.is_file():
                import csv as csvlib
                with csv.open(newline="") as f:
                    rows=list(csvlib.DictReader(f))
                if rows:
                    training={"last_epoch":rows[-1].get("epoch"),
                              "last_metrics":rows[-1],
                              "total_epochs":len(rows),
                              "run":str(run)}
        return {
            "stage":self.stage,"datasets":self.datasets(),
            "selected_dataset":self.dataset,
            "dataset_progress":status,
            "dataset_summary":summary,
            "capture_preview":capture_preview,
            "inspection":inspection,
            "previews":inspection.get("previews",[]),
            "models":self.models(),
            "selected_model":self.selected_model,
            "tasks":tasks,
            "training":training,"validation":metrics,
            "live":live,
            "comparison":comparison,
            "run":self.run_name,
            "task_logs": {
                n: _tail(self.lab_output/f"{n}.log")
                for n in ("capture","inspect","train","validate","live","import","validate_a","validate_b","live_a","live_b")
            },
        }
