"""Read the *saved* Lighting/Camera/AprilTags/Mobility/Odometry Lab YAMLs.

The Nav2 Lab is a consumer. It never writes upstream lab records or published
simulation YAMLs. Each run captures a validated snapshot and source hashes.
"""
from __future__ import annotations

from copy import deepcopy
from hashlib import sha256
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[3]
TOOLS = ROOT / "tools"
for directory in (TOOLS, TOOLS / "lighting_lab", TOOLS / "camera_lab"):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

from shared import lab_configs  # noqa: E402
from config import validated_calibration as validate_lighting  # noqa: E402
from modes import validate as validate_camera  # noqa: E402
from tag_math import validate as validate_apriltags  # noqa: E402

MOBILITY_KEYS = (
    "drive_contact_mu", "caster_contact_mu", "wheel_joint_damping",
    "wheel_collision_width", "drive_slip1", "drive_slip2",
)
RATE_KEYS = (
    "wheel_odom_hz", "imu_hz", "magnetometer_hz",
    "ground_truth_hz", "ekf_hz",
)


def source_path(lab):
    """Actual YAML path, including registry's historical fallback locations."""
    preferred = lab_configs.path_for(lab)
    if preferred.is_file():
        return preferred
    legacy = lab_configs.LEGACY_CONFIG_DIR / (lab + "_lab.yaml")
    return legacy if legacy.is_file() else None


def _origin(lab, path, kind):
    return {
        "lab": lab, "source": kind, "path": str(path),
        "sha256": sha256(Path(path).read_bytes()).hexdigest(),
    }


def _validate_mobility(saved):
    # Reuse the owning lab's original validator, not independently invented bounds.
    import importlib.util
    source = TOOLS / "mobility_lab" / "core.py"
    spec = importlib.util.spec_from_file_location("_nav2_mobility_owner", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.validate_settings(saved)


def _validate_odometry(saved):
    import importlib.util
    source = TOOLS / "odometry_lab" / "core.py"
    spec = importlib.util.spec_from_file_location("_nav2_odometry_owner", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.validate(saved)


def load_inputs():
    """Load saved values on *this* machine; explicit provenance for each source.

    Lighting, Camera and AprilTags are required for faithful planner visuals.
    Mobility and Odometry can fall back to published simulation calibration
    where no saved lab YAML exists; such fallback is clearly reported.
    """
    result = {}
    provenance = {}
    warnings = []
    for lab, validator in (
        ("lighting", validate_lighting),
        ("camera", validate_camera),
        ("apriltags", validate_apriltags),
    ):
        saved = lab_configs.read(lab)
        path = source_path(lab)
        if saved is None or path is None:
            raise FileNotFoundError(
                f"Missing saved {lab} lab YAML. Save it first using the owning lab "
                f"({lab_configs.path_for(lab)}). Production settings are not "
                "a silent substitute."
            )
        result[lab] = validator(saved)
        provenance[lab] = _origin(lab, path, "saved_lab_yaml")

    if not result["camera"]["depth_enabled"]:
        raise ValueError(
            "Camera Lab saved depth_enabled=false. Enable and Save the depth "
            "camera in Camera Lab before testing depth-based Nav2 planning."
        )

    saved_mobility = lab_configs.read("mobility")
    mobility_path = source_path("mobility")
    if saved_mobility is not None:
        validated = _validate_mobility(saved_mobility)
        result["mobility"] = {
            "physics": {key: validated[key] for key in MOBILITY_KEYS},
            "motor": {key: validated[key] for key in
                      ("model", "tau_left", "tau_right", "gain_left", "gain_right")},
        }
        provenance["mobility"] = _origin("mobility", mobility_path, "saved_lab_yaml")
    else:
        published = ROOT / "src/scrobot_simulation/config/mobility_calibration.yaml"
        data = yaml.safe_load(published.read_text())
        if not isinstance(data, dict) or data.get("schema_version") != 1:
            raise ValueError("Invalid published Mobility Lab fallback calibration")
        physics = data.get("physics", {})
        motor = data.get("motor", {})
        _validate_mobility({
            "profile": "forward", "target_rpm": 90, "pre_s": .3, "hold_s": 1.5,
            "post_s": .65, **physics, **motor,
        })
        result["mobility"] = {"physics": {k: physics[k] for k in MOBILITY_KEYS},
                              "motor": dict(motor)}
        provenance["mobility"] = _origin("mobility", published, "published_fallback")
        warnings.append("Mobility Lab has no saved YAML here; using explicitly "
                        "labeled published simulation calibration instead.")

    saved_odometry = lab_configs.read("odometry")
    odometry_path = source_path("odometry")
    if saved_odometry is not None:
        validated = _validate_odometry(saved_odometry)
        result["odometry"] = {k: float(validated[k]) for k in RATE_KEYS}
        provenance["odometry"] = _origin("odometry", odometry_path, "saved_lab_yaml")
    else:
        published = ROOT / "src/scrobot_simulation/config/odometry_calibration.yaml"
        data = yaml.safe_load(published.read_text())
        if not isinstance(data, dict) or data.get("schema_version") != 1:
            raise ValueError("Invalid published Odometry Lab fallback calibration")
        sys.path.insert(0, str(ROOT / "src/scrobot_simulation/launch"))
        from odometry_rate_config import validated_rates
        result["odometry"] = validated_rates(data["rates"])
        provenance["odometry"] = _origin("odometry", published, "published_fallback")
        warnings.append("Odometry Lab has no saved YAML here; using explicitly "
                        "labeled published simulation calibration instead.")

    result["provenance"] = provenance
    result["warnings"] = warnings
    return result


def inputs_fingerprint(loaded):
    return {name: info["sha256"]
            for name, info in loaded["provenance"].items()}


def build_world(inputs, output_dir):
    """Saved Lighting + AprilTags geometry, original court and Gazebo systems."""
    import copy
    import xml.etree.ElementTree as ET
    from lighting_source import apply_court_world
    from tags_scene import generate_tags

    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    tag_model, _tag_params = generate_tags(inputs["apriltags"], directory)
    world_source = ROOT / "src/scrobot_simulation/worlds/badminton_court.sdf"
    document = ET.parse(world_source)
    world = document.getroot().find("world")
    if world is None or world.get("name") != "badminton_court":
        raise ValueError("Unexpected source badminton court SDF")
    apply_court_world(world, inputs["lighting"])
    # Replace only the generated AprilTag include in this lab copy.
    # The tag texture mesh references model://court_apriltags/meshes/* and
    # GZ_SIM_RESOURCE_PATH is prefixed with the generated tag_models/ directory.
    for element in list(world.findall("include")):
        if (element.findtext("uri") or "").strip() == "model://court_apriltags":
            world.remove(element)
    model = ET.parse(tag_model / "model.sdf").getroot().find("model")
    if model is None:
        raise RuntimeError("Canonical AprilTags generator produced no model")
    world.append(copy.deepcopy(model))
    target = directory / "nav2_saved_calibration_court.sdf"
    ET.indent(document, space="  ")
    document.write(target, encoding="utf-8", xml_declaration=True)
    return {
        "world_path": str(target),
        "landmarks_path": str(directory / "tag_landmarks.yaml"),
        "tag_resource_dir": str(tag_model.parent),
    }


def generate_rate_profiles(inputs, output_dir):
    """Use production simulation helper to create lab-only controller/EKF YAML."""
    sys.path.insert(0, str(ROOT / "src/scrobot_simulation/launch"))
    from odometry_rate_config import prepare_configurations
    return prepare_configurations(
        inputs["odometry"],
        ROOT / "src/scrobot_control/config/controllers.yaml",
        ROOT / "src/scrobot_localization/config/ekf.yaml",
        directory=Path(output_dir) / "rates",
    )


def saved_summary(loaded):
    camera = loaded["camera"]
    return {
        "sources": deepcopy(loaded["provenance"]),
        "warnings": list(loaded["warnings"]),
        "camera": {
            "color": camera["color"], "color_fps": camera["color_fps"],
            "depth": camera["depth"], "depth_fps": camera["depth_fps"],
            "depth_enabled": camera["depth_enabled"],
        },
        "lighting": {
            "light_rows": loaded["lighting"]["lights"]["rows"],
            "light_cols": loaded["lighting"]["lights"]["cols"],
            "shadow_count": loaded["lighting"]["lights"]["shadow_count"],
            "venue": deepcopy(loaded["lighting"]["venue"]),
        },
        "apriltags": {
            "size_m": loaded["apriltags"]["size"],
            "height_m": loaded["apriltags"]["height"],
        },
        "mobility": deepcopy(loaded["mobility"]),
        "odometry": deepcopy(loaded["odometry"]),
        "note": "YOLO dataset/training settings are not used by depth-only path planning.",
    }
