#!/usr/bin/env python3
"""Atomically publish saved Mobility Lab parameters to NORMAL SIMULATION ONLY.

Never edits the real robot URDF, controller YAML, Lighting Lab, or mission.
The normal Gazebo bringup reads exactly one simulation config YAML.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import stat
import tempfile

import yaml

from core import DEFAULTS, validate_settings

REPO = Path(__file__).resolve().parents[2]
SIM_CONFIG = REPO / "src/scrobot_simulation/config/mobility_calibration.yaml"
LAB_SETTINGS = Path.home() / "scrobot_ws/evaluation_results/mobility_lab/settings.json"
BACKUPS = Path.home() / "scrobot_ws/evaluation_results/mobility_lab/published_backups"

PHYSICS = (
    "drive_contact_mu", "caster_contact_mu", "wheel_joint_damping",
    "wheel_collision_width", "drive_slip1", "drive_slip2",
)
MOTOR = ("model", "tau_left", "tau_right", "gain_left", "gain_right")


def calibration_document(settings):
    settings = validate_settings(settings)
    return {
        "schema_version": 1,
        "physics": {k: settings[k] for k in PHYSICS},
        "motor": {k: settings[k] for k in MOTOR},
    }


def read_production_config(path=SIM_CONFIG):
    if not path.is_file():
        raise FileNotFoundError(f"Missing simulation calibration YAML: {path}")
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(doc, dict) or doc.get("schema_version") != 1:
        raise ValueError("Unexpected simulation mobility calibration schema")
    if set(doc) != {"schema_version", "physics", "motor"}:
        raise ValueError("Unexpected keys in simulation mobility calibration")
    if (not isinstance(doc["physics"], dict)
            or not isinstance(doc["motor"], dict)
            or set(doc["physics"]) != set(PHYSICS)
            or set(doc["motor"]) != set(MOTOR)):
        raise ValueError("Simulation mobility calibration keys are incomplete")
    candidate = {
        **DEFAULTS, **doc["physics"], **doc["motor"],
    }
    cleaned = calibration_document(candidate)
    # Enforce lossless, unambiguous translation from valid source YAML.
    if cleaned != doc:
        raise ValueError("Simulation mobility calibration contains invalid values")
    return doc


def build_text(settings):
    doc = calibration_document(settings)
    return (
        "# Simulation-only SCROBOT mobility calibration.\n"
        "# Published from Mobility Lab SAVED settings; do not edit real robot.\n"
        "# Apply after restarting the normal Gazebo simulation.\n"
        "# No motor PID is added to the physical robot.\n"
        + yaml.safe_dump(doc, sort_keys=False, default_flow_style=False)
    )


def _atomic_write(path, text):
    tmp = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent,
            prefix=".mobility_calibration_", suffix=".tmp", delete=False,
        ) as f:
            tmp = Path(f.name)
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.chmod(tmp, stat.S_IMODE(path.stat().st_mode))
        os.replace(tmp, path)
        tmp = None
    finally:
        if tmp is not None:
            tmp.unlink(missing_ok=True)


def publish_saved_calibration(settings_path=LAB_SETTINGS, dest_path=SIM_CONFIG,
                              backup_dir=BACKUPS):
    """Publish only persisted, validated lab settings, with a byte-exact backup."""
    settings_path, dest_path = Path(settings_path), Path(dest_path)
    backup_dir = Path(backup_dir)
    if not settings_path.is_file():
        raise ValueError("Save Mobility Lab settings before publishing")
    raw = json.loads(settings_path.read_text(encoding="utf-8"))
    clean = validate_settings(raw)
    read_production_config(dest_path)  # Strict target-file verification.
    previous = dest_path.read_text(encoding="utf-8")
    updated = build_text(clean)
    digest = hashlib.sha256(updated.encode("utf-8")).hexdigest()[:16]
    if previous == updated or read_production_config(dest_path) == calibration_document(clean):
        return {
            "changed": False, "path": str(dest_path),
            "backup": None, "sha256_prefix": digest,
            "motor_model": clean["model"],
            "message": "Simulation calibration already matches saved lab settings",
        }
    backup_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    backup = backup_dir / (
        timestamp + "_" + hashlib.sha256(previous.encode("utf-8")).hexdigest()[:12]
        + ".yaml"
    )
    # Exclusive creation avoids silently overwriting a prior backup.
    with backup.open("x", encoding="utf-8") as f:
        f.write(previous)
        f.flush()
        os.fsync(f.fileno())
    _atomic_write(dest_path, updated)
    if read_production_config(dest_path) != calibration_document(clean):
        raise RuntimeError("Published file failed post-write verification")
    return {
        "changed": True, "path": str(dest_path),
        "backup": str(backup), "sha256_prefix": digest,
        "motor_model": clean["model"],
        "message": "Published simulation calibration. Restart and rebuild as needed.",
    }


def published_status(settings, config_path=SIM_CONFIG):
    target = calibration_document(settings)
    try:
        current = read_production_config(config_path)
        return {
            "matches_saved": target == current,
            "config_path": str(config_path),
            "published_motor_model": current["motor"]["model"],
            "error": None,
        }
    except (OSError, ValueError, yaml.YAMLError) as exc:
        return {"matches_saved": False, "config_path": str(config_path),
                "published_motor_model": None, "error": str(exc)}


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--settings", type=Path, default=LAB_SETTINGS)
    parser.add_argument("--config", type=Path, default=SIM_CONFIG)
    args = parser.parse_args()
    print(json.dumps(publish_saved_calibration(
        args.settings, args.config
    ), indent=2))
