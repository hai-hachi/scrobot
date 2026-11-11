"""Pure, ROS-free wheel-test math and calibration helpers for Mobility Lab."""
from __future__ import annotations

import csv
import io
import math
from typing import Iterable

RADIUS = 0.050
SEPARATION = 0.420
RPM_LIMIT = 95.0
DEFAULTS = {
    "profile": "left",
    "target_rpm": 90.0,
    "pre_s": 0.30,
    "hold_s": 1.50,
    "post_s": 0.65,
    "model": "baseline",
    "tau_left": 0.050,
    "tau_right": 0.050,
    "gain_left": 1.0,
    "gain_right": 1.0,
    "drive_contact_mu": 5.0,
    "caster_contact_mu": 0.05,
    "wheel_joint_damping": 0.0,
    "drive_slip1": 0.0,
    "drive_slip2": 0.0,
    "wheel_collision_width": 0.001,
}
BOUNDS = {
    "target_rpm": (5.0, RPM_LIMIT),
    "pre_s": (0.1, 1.0),
    "hold_s": (0.5, 5.0),
    "post_s": (0.4, 3.0),
    "tau_left": (0.005, 0.5),
    "tau_right": (0.005, 0.5),
    "gain_left": (0.8, 1.1),
    "gain_right": (0.8, 1.1),
    "drive_contact_mu": (0.05, 10.0),
    "caster_contact_mu": (0.0, 2.0),
    "wheel_joint_damping": (0.0, 0.5),
    "drive_slip1": (0.0, 0.02),
    "drive_slip2": (0.0, 0.02),
    "wheel_collision_width": (0.001, 0.03),
}
PROFILES = ("left", "right", "forward", "reverse", "spin")


def validate_settings(raw):
    if not isinstance(raw, dict):
        raise ValueError("Expected settings object")
    unknown = set(raw) - set(DEFAULTS)
    if unknown:
        raise ValueError("Unknown settings: " + ", ".join(sorted(unknown)))
    result = dict(DEFAULTS)
    result.update(raw)
    if result["profile"] not in PROFILES:
        raise ValueError("Unknown test profile")
    if result["model"] not in ("baseline", "first_order"):
        raise ValueError("Unknown motor model")
    for name, (low, high) in BOUNDS.items():
        if isinstance(result[name], bool):
            raise ValueError(name + " must be a number")
        try:
            value = float(result[name])
        except (TypeError, ValueError) as exc:
            raise ValueError(name + " must be numeric") from exc
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"{name} outside [{low}, {high}]")
        result[name] = value
    return result


def profile_targets(profile, rpm):
    rpm = min(max(0.0, float(rpm)), RPM_LIMIT)
    return {
        "left": (rpm, 0.0),
        "right": (0.0, rpm),
        "forward": (rpm, rpm),
        "reverse": (-rpm, -rpm),
        "spin": (-rpm, rpm),
    }[profile]


def rpm_to_twist(left_rpm, right_rpm):
    left = float(left_rpm) * 2 * math.pi * RADIUS / 60.0
    right = float(right_rpm) * 2 * math.pi * RADIUS / 60.0
    return (left + right) / 2.0, (right - left) / SEPARATION


def twist_to_rpm(v, yaw):
    left = (v - SEPARATION * yaw / 2) * 60.0 / (2 * math.pi * RADIUS)
    right = (v + SEPARATION * yaw / 2) * 60.0 / (2 * math.pi * RADIUS)
    return left, right


def cap_wheels(left_rpm, right_rpm, limit=RPM_LIMIT):
    peak = max(abs(left_rpm), abs(right_rpm))
    factor = min(1.0, limit / peak) if peak > 0 else 1.0
    return left_rpm * factor, right_rpm * factor


class WheelFilter:
    """Command-side surrogate of two first-order closed-loop RPM plants.

    The model acts BEFORE the default Gazebo velocity interface. It is not a
    torque actuator and does not make RPM load-dependent through friction.
    """

    def __init__(self):
        self.left = 0.0
        self.right = 0.0

    def reset(self):
        self.left = self.right = 0.0

    def update(self, targets, dt, config):
        if config["model"] == "baseline":
            self.left, self.right = cap_wheels(*targets)
        else:
            dt = max(0.0, min(float(dt), 0.1))
            a = -math.expm1(-dt / config["tau_left"])
            b = -math.expm1(-dt / config["tau_right"])
            self.left += a * (targets[0] * config["gain_left"] - self.left)
            self.right += b * (targets[1] * config["gain_right"] - self.right)
            self.left, self.right = cap_wheels(self.left, self.right)
        return self.left, self.right


def first_order_reference(t, command, settling_s=0.20):
    """Ideal 2%-band response; exact tau = settling_s / -ln(.02)."""
    tau = float(settling_s) / -math.log(0.02)
    if t < 0:
        return 0.0
    return command * -math.expm1(-t / tau)


def response_metrics(samples, side, command, hold_s):
    """Step-up rise/settle metrics using simulation timestamps, not wall time.

    Settling time is the earliest sample that stays in the 2% command band
    through the remainder of the commanded HOLD interval. Missing data/poor
    steady-state accuracy returns null rather than a false PASS.
    """
    if abs(command) < 1e-9:
        return None
    key = "actual_" + side + "_rpm"
    data = [(float(x["t"]), float(x[key])) for x in samples
            if 0 <= x["t"] <= hold_s and x.get(key) is not None]
    if len(data) < 8:
        return None
    initial = float(command)
    normalized = [(t, y / initial) for t, y in data]
    def crossing(frac):
        for t, y in normalized:
            if y >= frac:
                return t
        return None
    t10, t90, t63 = crossing(0.1), crossing(0.9), crossing(1 - 1/math.e)
    overshoot = max(0.0, (max(y for _, y in normalized) - 1.0) * 100.0)
    settling = None
    for i, (t, _) in enumerate(normalized):
        if all(abs(y - 1.0) <= 0.02 for _, y in normalized[i:]):
            settling = t
            break
    tail = [y for t, y in normalized if t >= hold_s * 0.80]
    final = sum(tail) / len(tail) if tail else normalized[-1][1]
    return {
        "settling_s": round(settling, 4) if settling is not None else None,
        "rise_10_90_s": round(t90 - t10, 4) if t10 is not None and t90 is not None else None,
        "tau_63_s": round(t63, 4) if t63 is not None else None,
        "overshoot_pct": round(overshoot, 3),
        "final_rpm": round(initial * final, 3),
        "steady_error_pct": round(100 * (1 - final), 3),
        "target_rpm": round(initial, 3),
    }


def reference_csv(text):
    """Import exported MATLAB CSV with time_s and one/two wheel RPM columns."""
    if not isinstance(text, str) or len(text) > 4_000_000:
        raise ValueError("Reference CSV must be text under 4 MB")
    reader = csv.DictReader(io.StringIO(text.lstrip("\ufeff")))
    if not reader.fieldnames:
        raise ValueError("CSV has no header")
    canonical = {s.strip().lower(): s for s in reader.fieldnames if s}
    def column(*candidates):
        return next((canonical[k] for k in candidates if k in canonical), None)
    time_col = column("time_s", "time", "t", "seconds")
    left_col = column("left_actual_rpm", "left_rpm", "rpm_left", "wl_rpm")
    right_col = column("right_actual_rpm", "right_rpm", "rpm_right", "wr_rpm")
    if not time_col or not (left_col or right_col):
        raise ValueError("Need time_s and left_actual_rpm and/or right_actual_rpm")
    rows = []
    for record in reader:
        try:
            t = float(record[time_col])
            left = float(record[left_col]) if left_col and record.get(left_col) else None
            right = float(record[right_col]) if right_col and record.get(right_col) else None
        except (ValueError, TypeError) as exc:
            raise ValueError("Non-numeric reference CSV sample") from exc
        if not all(math.isfinite(v) for v in (t, left, right) if v is not None):
            raise ValueError("Non-finite reference samples")
        rows.append({"t": t, "actual_left_rpm": left, "actual_right_rpm": right})
        if len(rows) > 150_000:
            raise ValueError("Too many reference samples")
    if len(rows) < 3 or any(rows[i]["t"] <= rows[i-1]["t"] for i in range(1, len(rows))):
        raise ValueError("Reference must contain >=3 strictly time-increasing samples")
    return rows


def compare_rmse(samples, reference, side):
    """Linear-interpolated RPM RMSE over overlapping time range."""
    key = "actual_" + side + "_rpm"
    ref = [(r["t"], r[key]) for r in reference if r.get(key) is not None]
    if len(ref) < 2:
        return None
    errors = []
    idx = 0
    for sample in samples:
        t, measured = sample["t"], sample.get(key)
        if measured is None or t < ref[0][0] or t > ref[-1][0]:
            continue
        while idx + 1 < len(ref) - 1 and ref[idx + 1][0] < t:
            idx += 1
        t0, y0 = ref[idx]
        t1, y1 = ref[idx + 1]
        expected = y0 + (y1 - y0) * (t - t0) / (t1 - t0)
        errors.append((measured - expected)**2)
    return round(math.sqrt(sum(errors) / len(errors)), 4) if errors else None
