"""Deterministic SCROBOT shuttle layouts, matching the production spawn modes.

Coordinate convention: side A is +X, left is +Y. The weights below
match src/scrobot_simulation/config/shuttle_spawn.yaml.
"""
from __future__ import annotations

import math
import random

ROWS = (
    ("A singles long", 6.70),
    ("A doubles long", 5.94),
    ("A short", 1.98),
    ("Net", 0.00),
    ("B short", -1.98),
    ("B doubles long", -5.94),
    ("B singles long", -6.70),
)
DEFAULT_WEIGHTS = (
    (1.0, 0.0), (0.0, 1.0), (0.2, 0.8), (1.0, 0.3),
    (0.7, 0.3), (0.0, 1.0), (1.0, 0.0),
)
COURT_LENGTH, COURT_WIDTH, MARGIN = 13.40, 6.10, 0.15


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _sample_xy(rng, density):
    hx = COURT_LENGTH / 2 - MARGIN
    hy = COURT_WIDTH / 2 - MARGIN
    if density == "uniform":
        return rng.uniform(-hx, hx), rng.uniform(-hy, hy)
    if density == "center":
        return (_clamp(rng.gauss(0, COURT_LENGTH/5), -hx, hx),
                _clamp(rng.gauss(0, COURT_WIDTH/5), -hy, hy))
    if density == "net":
        return (_clamp(rng.gauss(0, COURT_LENGTH*0.08), -hx, hx),
                rng.uniform(-hy, hy))
    raise ValueError("Unknown random density: " + str(density))


def _truncated(rng, mean, sigma, lo, hi):
    if sigma == 0:
        return _clamp(mean, lo, hi)
    for _ in range(64):
        sample = rng.gauss(mean, sigma)
        if lo <= sample <= hi:
            return sample
    return _clamp(mean, lo, hi)


def _cluster(rng, count, cfg):
    weights = cfg["cluster_weights"]
    total = sum(sum(row) for row in weights)
    allocations = []
    for line_index, row in enumerate(weights):
        for side_index, weight in enumerate(row):
            exact = count * weight / total
            allocations.append([line_index, side_index, int(exact),
                                exact - math.floor(exact), rng.random()])
    remainder = count - sum(row[2] for row in allocations)
    for item in sorted(allocations, key=lambda x: (-x[3], x[4]))[:remainder]:
        item[2] += 1
    points = []
    for i, side, n, _, _ in allocations:
        if not n:
            continue
        mx = cfg["cluster_line_x"][i]
        my = cfg["cluster_left_y"] if side == 0 else cfg["cluster_right_y"]
        ylo, yhi = ((0, COURT_WIDTH/2) if side == 0 else (-COURT_WIDTH/2, 0))
        for _ in range(n):
            points.append((
                _truncated(rng, mx, cfg["cluster_sigma_x"],
                           -COURT_LENGTH/2, COURT_LENGTH/2),
                _truncated(rng, my, cfg["cluster_sigma_y"], ylo, yhi),
                ROWS[i][0] + (" left" if side == 0 else " right"),
            ))
    rng.shuffle(points)
    return points


def sample(cfg):
    """Return deterministic (x, y, zone) for the scene, never modify cfg."""
    rng = random.Random(cfg["seed"])
    mode = cfg["spawn_mode"]
    count = cfg["shuttle_count"]
    if mode == "single":
        return [(cfg["single_x"], cfg["single_y"], "single")]
    if mode == "random":
        return [(*_sample_xy(rng, cfg["random_density"]), "random")
                for _ in range(count)]
    if mode == "cluster":
        return _cluster(rng, count, cfg)
    if mode == "mixed":
        n_random = int(round(count * cfg["random_fraction"]))
        points = [(*_sample_xy(rng, cfg["random_density"]), "random")
                  for _ in range(n_random)]
        points += _cluster(rng, count - n_random, cfg) if count > n_random else []
        rng.shuffle(points)
        return points
    raise ValueError("Unknown spawn mode: " + str(mode))
