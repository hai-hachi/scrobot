"""Validated Camera Lab -> SIMULATION D435i configuration publication.

No production source changes except a single dedicated YAML, never the
physical RealSense driver or its hardware launch. Writes are atomic and
the preceding YAML is backed up byte-for-byte.
"""
from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile
import hashlib
import yaml

from core import COLOR, DEPTH, FPS, validate

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "src/scrobot_simulation/config/camera_calibration.yaml"
OUTPUT = ROOT / "evaluation_results/perception_labs"
BACKUPS = OUTPUT / "camera_published_backups"


def document(config):
    cfg = validate({**config, "mode": "camera"})
    cw, ch = COLOR[cfg["color"]]
    dw, dh = DEPTH[cfg["depth"]]
    return {
        "schema_version": 1,
        "color": {"width": cw, "height": ch, "fps": cfg["color_fps"]},
        "depth": {
            "width": dw, "height": dh, "fps": cfg["depth_fps"],
            "enabled": cfg["depth_enabled"],
        },
    }


def read_config(path=CONFIG):
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or set(raw) != {"schema_version", "color", "depth"}:
        raise ValueError("Unexpected camera calibration keys")
    if raw["schema_version"] != 1:
        raise ValueError("Unsupported camera calibration schema")
    for stream, options in (("color", COLOR), ("depth", DEPTH)):
        fields = raw.get(stream)
        if not isinstance(fields, dict) or set(fields) != (
            {"width", "height", "fps", "enabled"} if stream == "depth"
            else {"width", "height", "fps"}
        ):
            raise ValueError(f"Malformed {stream} camera calibration")
        if (type(fields["width"]) is not int or
                type(fields["height"]) is not int or
                (fields["width"], fields["height"]) not in options.values() or
                type(fields["fps"]) is not int or fields["fps"] not in FPS):
            raise ValueError(f"Unsupported {stream} dimensions or frame rate")
        if stream == "depth" and type(fields["enabled"]) is not bool:
            raise ValueError("Depth enabled must be boolean")
    return raw


def status(settings, path=CONFIG):
    try:
        actual = read_config(path)
        return {
            "path": str(path),
            "matches_lab": actual == document(settings),
            "published": actual,
            "error": None,
        }
    except (OSError, ValueError, yaml.YAMLError) as exc:
        return {"path": str(path), "matches_lab": False,
                "published": None, "error": str(exc)}


def publish_benchmark(record, path=CONFIG, backup_dir=BACKUPS):
    """Publish a completed, explicit, full-court camera benchmark.

    The benchmark is a persisted JSON snapshot, not unsaved browser sliders.
    The camera must have produced actual frames of the commanded dimensions
    and enough timestamps to calculate the observed sim frame rate.
    """
    if not isinstance(record, dict) or record.get("mode") != "camera":
        raise ValueError("Only a saved Camera Lab benchmark can be published")
    if record.get("phase") != "streaming":
        raise ValueError("Benchmark was not recorded during streaming")
    cfg = validate(record.get("settings", {}))
    streams = (record.get("monitor") or {}).get("streams") or {}
    rgb = streams.get("rgb") or {}
    if rgb.get("samples", 0) < 30:
        raise ValueError("Collect at least 30 RGB frames before publishing")
    if rgb.get("resolution") != list(COLOR[cfg["color"]]):
        raise ValueError("Measured RGB resolution does not match candidate settings")
    if not rgb.get("sim_fps") or rgb["sim_fps"] < cfg["color_fps"] * 0.8:
        raise ValueError("RGB stream is below 80% of the requested SIM frame rate")
    if cfg["depth_enabled"]:
        depth = streams.get("depth") or {}
        if depth.get("samples", 0) < 30:
            raise ValueError("Collect at least 30 depth frames before publishing")
        if depth.get("resolution") != list(DEPTH[cfg["depth"]]):
            raise ValueError("Measured depth resolution does not match candidate")
        if (not depth.get("sim_fps") or
                depth["sim_fps"] < cfg["depth_fps"] * 0.8):
            raise ValueError("Depth stream below 80% of requested SIM FPS")
    path = Path(path)
    backup_dir = Path(backup_dir)
    old_doc = read_config(path)
    new_doc = document(cfg)
    if old_doc == new_doc:
        return {"changed": False, "path": str(path), "backup": None}
    previous = path.read_bytes()
    backup_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    backup = backup_dir / (timestamp + "_" +
                           hashlib.sha256(previous).hexdigest()[:12] + ".yaml")
    with backup.open("xb") as f:
        f.write(previous)
        f.flush()
        os.fsync(f.fileno())
    payload = (
        "# Published by SCROBOT Camera Lab after verifying a saved benchmark.\n"
        "# SIMULATION ONLY: physical D435i driver remains unchanged.\n"
        + yaml.safe_dump(new_doc, sort_keys=False)
    )
    temp = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8",
                                         dir=path.parent, delete=False,
                                         prefix=".camera_cal_", suffix=".tmp") as f:
            temp = Path(f.name)
            f.write(payload)
            f.flush()
            os.fsync(f.fileno())
        os.chmod(temp, path.stat().st_mode & 0o777)
        os.replace(temp, path)
        temp = None
    finally:
        if temp is not None:
            temp.unlink(missing_ok=True)
    if read_config(path) != new_doc:
        raise RuntimeError("Post-write camera calibration verification failed")
    return {
        "changed": True, "path": str(path), "backup": str(backup),
        "configuration": new_doc,
        "message": "Rebuild/synchronize the installed config and restart Gazebo",
    }
