"""Shared, versioned *lab* YAML handoff, separate from production config.

Each active lab owns one saved YAML file under tools/configs/labs/.
Readers can consume other labs' saved values without copying or publishing
production files. Producers must validate settings before saving. Readers must
validate again with their own source-domain validators.
"""
from __future__ import annotations

from pathlib import Path
import os
import tempfile

import yaml

ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = ROOT / "tools/configs/labs"
LEGACY_CONFIG_DIR = ROOT / "src/scrobot_simulation/config"
FILES = {
    "lighting": CONFIG_DIR / "lighting_lab.yaml",  # legacy top-level schema
    "camera": CONFIG_DIR / "camera_lab.yaml",
    "apriltags": CONFIG_DIR / "apriltags_lab.yaml",
    "mobility": CONFIG_DIR / "mobility_lab.yaml",
    "odometry": CONFIG_DIR / "odometry_lab.yaml",
    "yolo": CONFIG_DIR / "yolo_lab.yaml",
}


def path_for(lab):
    if lab not in FILES:
        raise ValueError(f"Unknown lab: {lab}")
    return FILES[lab]


def _safe_yaml(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Expected a nonempty YAML mapping: {path}")
    return data


def read(lab, path=None):
    """Read another lab's saved YAML; return None if it has never been saved.

    Lighting Lab predates this registry and has a strict legacy top-level
    lights/colors/venue schema. Do not force a silent migration.
    """
    src = Path(path) if path is not None else path_for(lab)
    if not src.is_file() and path is None:
        # For older checkouts where labs were saved beside production files.
        legacy = LEGACY_CONFIG_DIR / (lab + "_lab.yaml")
        if legacy.is_file():
            src = legacy
    if not src.is_file():
        return None
    data = _safe_yaml(src)
    if lab == "lighting":
        if set(data) != {"lights", "colors", "venue"}:
            raise ValueError("Unexpected Lighting Lab YAML sections")
        return data
    if set(data) != {"schema_version", "lab", "settings"}:
        raise ValueError(f"Unexpected {lab} Lab YAML fields in {src}")
    if data["schema_version"] != 1 or data["lab"] != lab:
        raise ValueError(f"{src} is not a supported {lab} Lab YAML")
    if not isinstance(data["settings"], dict):
        raise ValueError(f"Invalid settings in {src}")
    return data["settings"]


def save(lab, settings, path=None, validator=None):
    """Validate, atomically save the requested lab's settings, and return path.

    A caller cannot write another lab's YAML without specifying that lab;
    the settings are always checked by the owning lab validator.
    """
    if lab == "lighting":
        raise ValueError("Use Lighting Lab's calibrated YAML writer")
    if not isinstance(settings, dict):
        raise ValueError("Settings must be a mapping")
    clean = validator(settings) if validator else settings
    if not isinstance(clean, dict):
        raise ValueError("Lab validator must return a settings mapping")
    dest = Path(path) if path is not None else path_for(lab)
    dest.parent.mkdir(parents=True, exist_ok=True)
    doc = {"schema_version": 1, "lab": lab, "settings": clean}
    serialized = (
        f"# Saved {lab.capitalize()} Lab settings, not main simulation params.\n"
        "# Use the separate Publish button to transfer to production.\n"
        + yaml.safe_dump(doc, sort_keys=False, allow_unicode=True)
    )
    temp = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=dest.parent,
            prefix=f".{lab}_lab_", suffix=".yaml.tmp", delete=False
        ) as out:
            temp = Path(out.name)
            out.write(serialized)
            out.flush()
            os.fsync(out.fileno())
        if dest.exists():
            os.chmod(temp, dest.stat().st_mode & 0o777)
        os.replace(temp, dest)
        temp = None
    finally:
        if temp is not None:
            temp.unlink(missing_ok=True)
    return dest


def matches(lab, settings, validator=None, path=None):
    saved = read(lab, path)
    if saved is None:
        return False
    return (validator(saved) if validator else saved) == (
        validator(settings) if validator else settings
    )


def lighting_for_consumer():
    """Validated Lighting Lab YAML, or fail rather than silently use a sun.

    Every lab can call read("lighting"), but rendering consumers should
    validate with the Lighting Lab's canonical checked_lights/etc.
    """
    source = read("lighting")
    if source is None:
        raise FileNotFoundError(
            f"Lighting Lab YAML is missing: {path_for('lighting')}. "
            "Save it in Lighting Lab before launching a dependent lab."
        )
    return source
