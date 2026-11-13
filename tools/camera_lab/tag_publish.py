"""Explicit AprilTags Lab -> production tag geometry publication.

These two YAML files are authoritative for the built Gazebo model,
localization geometry, and apriltag_ros size. The existing SCROBOT build
regenerates court_apriltags from court_landmarks.yaml, so there is no
second mesh copy to hand-edit. RGB pixel noise stays lab-only.
"""
from __future__ import annotations

from datetime import datetime, timezone
import os
from pathlib import Path
import re
import tempfile

import yaml

from tag_math import mount_radius, validate
from tags_scene import LANDMARKS, ROOT

DETECTOR = ROOT / "src/scrobot_perception/config/perception.yaml"
BACKUPS = ROOT / "evaluation_results/camera_lab/tag_published_backups"


def _replace_scalar(content, key, replacement):
    pattern = re.compile(r"(?m)^([ \t]*" + re.escape(key) + r":\s*)([^\r\n#]+)([ \t]*(?:#.*)?)$")
    def replace(match):
        return match.group(1) + replacement + match.group(3)
    updated, count = pattern.subn(replace, content)
    if count != 1:
        raise ValueError(f"Expected exactly one production {key} key, got {count}")
    return updated


def prepare(tag, landmarks_text, detector_text):
    cfg = validate(tag)
    radius = mount_radius(cfg["size"])
    landmarks = yaml.safe_load(landmarks_text)
    detector = yaml.safe_load(detector_text)
    params = landmarks["tag_global_localizer"]["ros__parameters"]
    apr = detector["apriltag"]["ros__parameters"]
    if (apr["family"] != "16h5" or apr["tag"]["ids"] != [0, 1, 2, 3]
            or len(apr["tag"]["sizes"]) != 4
            or params["active_grid_cells"] != 6
            or params["quiet_border_cells"] != 1):
        raise ValueError("Production AprilTag IDs/family/texture layout changed")
    edge = f'{cfg["size"]:.3f}'
    height = f'{cfg["height"]:.3f}'
    landmarks_out = landmarks_text
    for field, value in (
        ("tag_edge_size", edge),
        ("tag_height", height),
        ("tag_mount_radius", f"{radius:.6f}"),
    ):
        landmarks_out = _replace_scalar(landmarks_out, field, value)
    detector_out = _replace_scalar(detector_text, "size", edge)
    # Keep the human-readable description accurate for non-100 mm tags.
    detector_out = detector_out.replace(
        "# Final court fiducials: AprilTag 16h5, IDs 0-3, 100 mm active edge.",
        "# Final court fiducials: AprilTag 16h5, IDs 0-3; size calibrated below."
    )
    detector_out = _replace_scalar(
        detector_out, "sizes", "[" + ", ".join([edge] * 4) + "]")
    # Explicitly ensure producer and consumer share the exact same size.
    known = yaml.safe_load(landmarks_out)["tag_global_localizer"]["ros__parameters"]
    det = yaml.safe_load(detector_out)["apriltag"]["ros__parameters"]
    if not (
        abs(known["tag_edge_size"] - cfg["size"]) < 1e-9
        and abs(known["tag_height"] - cfg["height"]) < 1e-9
        and abs(known["tag_mount_radius"] - radius) < 1e-9
        and abs(det["size"] - cfg["size"]) < 1e-9
        and all(abs(x - cfg["size"]) < 1e-9 for x in det["tag"]["sizes"])
    ):
        raise ValueError("Publication did not produce consistent tag sizes")
    return landmarks_out, detector_out


def _atomic_replace(destination, content):
    dest = Path(destination)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            "w", encoding="utf-8", newline="\n", dir=dest.parent,
            prefix=".tag_publish_", suffix=".tmp", delete=False,
        ) as output:
            temporary = Path(output.name)
            output.write(content)
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary, dest.stat().st_mode & 0o777)
        os.replace(temporary, dest)
        temporary = None
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def publish(tag, landmark_path=LANDMARKS, detector_path=DETECTOR,
            backups_dir=BACKUPS):
    """Write both producer and detector values with byte-exact backups.

    A failed second write attempts to restore the first file from its
    backed-up copy. Never touch the physical camera or AprilTag noise.
    """
    landmark_path, detector_path = Path(landmark_path), Path(detector_path)
    original_landmarks = landmark_path.read_bytes()
    original_detector = detector_path.read_bytes()
    revised = prepare(tag, original_landmarks.decode("utf-8"),
                      original_detector.decode("utf-8"))
    old = (original_landmarks, original_detector)
    updated = tuple(v.encode("utf-8") for v in revised)
    if old == updated:
        return {"changed": False, "paths": [str(landmark_path), str(detector_path)],
                "backup": None}
    backups_dir = Path(backups_dir)
    backup_dir = backups_dir / datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%S%fZ")
    backup_dir.mkdir(parents=True, exist_ok=False)
    for name, contents in (
        ("court_landmarks.yaml", original_landmarks),
        ("perception.yaml", original_detector),
    ):
        backup_file = backup_dir / name
        with backup_file.open("xb") as output:
            output.write(contents)
            output.flush()
            os.fsync(output.fileno())
    targets = (landmark_path, detector_path)
    changed = []
    try:
        for dest, new_data, prior in zip(targets, revised, old):
            if dest.read_bytes() != prior:
                raise RuntimeError(
                    f"Production {dest.name} changed during publication; "
                    "aborting rather than overwriting concurrent edits"
                )
            if dest.read_bytes() != new_data.encode("utf-8"):
                _atomic_replace(dest, new_data)
                changed.append(dest)
        rechecked = prepare(tag, landmark_path.read_text(),
                            detector_path.read_text())
        if rechecked != (landmark_path.read_text(), detector_path.read_text()):
            raise RuntimeError("Published AprilTag YAML failed verification")
    except Exception:
        for dest in changed:
            backup_name = ("court_landmarks.yaml"
                           if dest == landmark_path else "perception.yaml")
            _atomic_replace(dest, (backup_dir / backup_name).read_text())
        raise
    return {
        "changed": True,
        "paths": [str(p) for p in targets],
        "backup": str(backup_dir),
        "tag_edge_size_m": validate(tag)["size"],
        "tag_height_m": validate(tag)["height"],
        "tag_mount_radius_m": mount_radius(validate(tag)["size"]),
        "noise_published": False,
        "rebuild_required": True,
    }


def status(tag, landmark_path=LANDMARKS, detector_path=DETECTOR):
    try:
        a = Path(landmark_path).read_text()
        b = Path(detector_path).read_text()
        revised = prepare(tag, a, b)
        return {"matches_lab": (a, b) == revised,
                "paths": [str(landmark_path), str(detector_path)],
                "error": None}
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        return {"matches_lab": False, "paths": [
            str(landmark_path), str(detector_path)], "error": str(exc)}
