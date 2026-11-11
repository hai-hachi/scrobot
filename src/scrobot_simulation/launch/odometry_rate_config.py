"""Simulation-only sensor / wheel odometry / EKF frequency configuration.

This module is installed with scrobot_simulation/launch and can be imported
from main simulation or independent odometry_lab launch descriptions.
It NEVER edits the physical-robot controller or EKF source YAML.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

import yaml

RATE_KEYS = (
    "wheel_odom_hz", "imu_hz", "magnetometer_hz",
    "ground_truth_hz", "ekf_hz",
)
DEFAULT_RATES = {name: 100.0 for name in RATE_KEYS}


def validated_rates(raw):
    if not isinstance(raw, dict) or set(raw) != set(RATE_KEYS):
        raise ValueError("Odometry rates require exactly " + ", ".join(RATE_KEYS))
    rates = {}
    for key in RATE_KEYS:
        value = raw[key]
        if isinstance(value, bool) or not isinstance(value, (float, int)):
            raise ValueError(f"{key} must be numeric")
        value = float(value)
        if not math.isfinite(value) or not 10.0 <= value <= 200.0:
            raise ValueError(f"{key} must be between 10 and 200 Hz")
        rates[key] = value
    return rates


def load_rates(path):
    doc = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(doc, dict) or doc.get("schema_version") != 1:
        raise ValueError(f"Invalid odometry calibration schema: {path}")
    return validated_rates(doc.get("rates"))


def prepare_configurations(rates, controller_base, ekf_base,
                           directory=None):
    """Return generated, versioned YAML paths without changing source files.

    Configs are content-addressed so concurrent ROS launch processes with
    different rates cannot overwrite one another.
    """
    rates = validated_rates(rates)
    controller = yaml.safe_load(Path(controller_base).read_text(encoding="utf-8"))
    ekf = yaml.safe_load(Path(ekf_base).read_text(encoding="utf-8"))
    if not isinstance(controller, dict) or not isinstance(ekf, dict):
        raise ValueError("Invalid baseline ROS controller or EKF YAML")
    if int(controller["controller_manager"]["ros__parameters"]["update_rate"]) < max(
        rates["wheel_odom_hz"], 100.0
    ):
        # A publisher cannot produce fresh encoder odometry faster than the
        # controller update loop. Keep controller_manager 100 Hz by default;
        # raising it is explicitly necessary for faster wheel publication.
        controller["controller_manager"]["ros__parameters"]["update_rate"] = max(
            100, int(math.ceil(rates["wheel_odom_hz"]))
        )
    controller["diff_drive_controller"]["ros__parameters"]["publish_rate"] = rates["wheel_odom_hz"]
    ekf["ekf_filter_node"]["ros__parameters"]["frequency"] = rates["ekf_hz"]
    output = Path(directory) if directory is not None else (
        Path.home() / ".ros/scrobot_odometry_lab/rate_profiles"
    )
    output.mkdir(parents=True, exist_ok=True)
    def store(label, doc):
        payload = yaml.safe_dump(doc, sort_keys=False)
        digest = hashlib.sha256(payload.encode()).hexdigest()[:16]
        target = output / f"{label}_{digest}.yaml"
        if not target.exists():
            with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=output,
                prefix="." + label + "_", delete=False
            ) as f:
                temporary = Path(f.name)
                f.write(payload)
                f.flush()
                os.fsync(f.fileno())
            try:
                os.replace(temporary, target)
            finally:
                temporary.unlink(missing_ok=True)
        return str(target)
    return {
        "controller_params": store("controllers", controller),
        "ekf_params": store("ekf", ekf),
    }
