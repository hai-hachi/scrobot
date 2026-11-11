"""Real, floor-loaded STM32 wheel PID validation reference for Mobility Lab.

The left/right logs were recorded in separate experiments ten seconds apart.
Their timestamps are aligned to EACH motor's own command edge, never treated
as simultaneous chassis motion measurements.
"""
from __future__ import annotations

import json
from pathlib import Path

from core import compare_rmse, reference_csv

REFERENCE_DIR = Path(__file__).resolve().parent / "data/floor_loaded_20260922"
STEP = REFERENCE_DIR / "reference_step_90rpm.csv"
STOP = REFERENCE_DIR / "reference_stop_90rpm.csv"
MANIFEST = REFERENCE_DIR / "manifest.json"


def bundled_validation():
    """Return two 100-Hz aligned response phases plus provenance metadata."""
    step = reference_csv(STEP.read_text(encoding="utf-8"))
    stop = reference_csv(STOP.read_text(encoding="utf-8"))
    metadata = json.loads(MANIFEST.read_text(encoding="utf-8"))
    if len(step) != 501 or len(stop) != 101:
        raise ValueError("Bundled validation data has unexpected length")
    if abs(step[0]["t"]) > 1e-9 or abs(stop[0]["t"]) > 1e-9:
        raise ValueError("Bundled phase references must begin at their command edges")
    return step, stop, metadata


def active_real_wheel(settings):
    """Only actual 90-RPM isolated single-wheel tests have matching real data."""
    if not isinstance(settings, dict):
        return None
    if abs(float(settings.get("target_rpm", 0)) - 90.0) > 1e-6:
        return None
    profile = settings.get("profile")
    return profile if profile in ("left", "right") else None


def phase_aligned_reference(settings, step, stop=None):
    """Transform real 5-s step + stop into the lab's chosen shorter hold time.

    Output uses the exact same 't' and actual_*_rpm fields as recorded sim.
    The original uploaded logs remain untouched. We do NOT extrapolate their
    5-s HOLD into a multiwheel or reverse drivetrain validation.
    """
    wheel = active_real_wheel(settings)
    if wheel is None:
        return []
    hold = float(settings["hold_s"])
    post = float(settings["post_s"])
    field = "actual_" + wheel + "_rpm"
    output = []
    for sample in step:
        if 0.0 <= sample["t"] < hold and sample.get(field) is not None:
            output.append({"t": sample["t"], field: sample[field]})
    if stop:
        for sample in stop:
            if 0.0 <= sample["t"] <= post and sample.get(field) is not None:
                output.append({"t": round(hold + sample["t"], 6),
                               field: sample[field]})
    return output


def compare_validation(samples, settings, step, stop=None):
    """Only score measured real-equivalent wheel and command amplitude."""
    ref = phase_aligned_reference(settings, step, stop)
    wheel = active_real_wheel(settings)
    if not wheel or len(ref) < 2:
        return {"left": None, "right": None}, ref
    return {side: compare_rmse(samples, ref, side) if side == wheel else None
            for side in ("left", "right")}, ref


def status_label(settings, source):
    wheel = active_real_wheel(settings)
    if not wheel:
        return ("No directly comparable real run: bundled recordings are "
                "independent +90 RPM LEFT-ONLY and RIGHT-ONLY tests.")
    if source == "builtin":
        return (f"Real floor-loaded {wheel.upper()} 90 RPM (2026-09-22); "
                "step and braking edges aligned separately; not a simultaneous two-wheel test.")
    return f"Uploaded {wheel.upper()} 90 RPM step-only reference."
