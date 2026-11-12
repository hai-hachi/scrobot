"""AprilTag detector repeatability diagnostics, no ROS runtime dependencies.

These are measurement consistency statistics, NOT absolute tag-to-camera
position/rotation accuracy. Absolute accuracy additionally requires measured
camera extrinsics, known tag frame convention and Gazebo pose verification.
"""
from __future__ import annotations

from collections import defaultdict, deque
import math
import statistics


def quaternion_angular_distance(a, b):
    """Quaternion-sign-invariant 3D angular difference in degrees."""
    dot = sum(float(x) * float(y) for x, y in zip(a, b))
    na = math.sqrt(sum(float(x) ** 2 for x in a))
    nb = math.sqrt(sum(float(x) ** 2 for x in b))
    if na == 0 or nb == 0:
        raise ValueError("Invalid zero quaternion")
    return math.degrees(2 * math.acos(min(1.0, abs(dot / (na * nb)))))


class TagQuality:
    def __init__(self, window=120):
        self.window = window
        self.message_count = 0
        self.counts = defaultdict(int)
        self.poses = defaultdict(lambda: deque(maxlen=self.window))
        self.margins = defaultdict(lambda: deque(maxlen=self.window))
        self.hammings = defaultdict(lambda: deque(maxlen=self.window))

    def add(self, detections):
        """List of dicts id/optional margin, hamming, x/y/z, quaternion."""
        self.message_count += 1
        for d in detections:
            tag = int(d["id"])
            self.counts[tag] += 1
            if d.get("margin") is not None:
                self.margins[tag].append(float(d["margin"]))
            if d.get("hamming") is not None:
                self.hammings[tag].append(int(d["hamming"]))
            if "position" in d and "quaternion" in d:
                point = tuple(map(float, d["position"]))
                q = tuple(map(float, d["quaternion"]))
                if len(point) == 3 and len(q) == 4:
                    if all(math.isfinite(v) for v in (*point, *q)):
                        self.poses[tag].append((point, q))

    def status(self):
        result = []
        for tag in sorted(self.counts):
            m = self.margins[tag]
            ps = list(self.poses[tag])
            item = {
                "id": tag,
                "detections": self.counts[tag],
                "detector_frame_fraction": round(
                    self.counts[tag] / max(1, self.message_count), 3),
                "mean_margin": round(statistics.fmean(m), 2) if m else None,
                "max_hamming": max(self.hammings[tag]) if self.hammings[tag] else None,
                "pose_samples": len(ps),
                "translation_std_m": None,
                "rotation_rms_deg": None,
                "range_mean_m": None,
            }
            if len(ps) >= 3:
                xyz = list(zip(*(p for p, _ in ps)))
                item["translation_std_m"] = round(
                    math.sqrt(sum(statistics.pvariance(axis) for axis in xyz)), 5)
                item["range_mean_m"] = round(
                    statistics.fmean(math.sqrt(sum(v*v for v in p)) for p,_ in ps), 3)
                q_ref = ps[0][1]
                item["rotation_rms_deg"] = round(
                    math.sqrt(statistics.fmean(
                        quaternion_angular_distance(q_ref, q)**2
                        for _,q in ps)), 3)
            result.append(item)
        return result
