"""YOLO inspection shuttle distribution using the production spawn YAML geometry.

Shuttle mode, count, seed, density, mixed fraction and cluster weights come from
YOLO Lab settings. Court dimensions, line positions, cluster spread, side
centers, single pose and spawn height ALWAYS come from shuttle_spawn.yaml.
"""
from __future__ import annotations

import math
from pathlib import Path
import random

import yaml

SOURCE = (Path(__file__).resolve().parents[2] /
          "src/scrobot_simulation/config/shuttle_spawn.yaml")
ROW_KEYS = (
    "side_a_singles_long", "side_a_doubles_long", "side_a_short", "net",
    "side_b_short", "side_b_doubles_long", "side_b_singles_long",
)
ROW_LABELS = (
    "A singles long", "A doubles long", "A short", "Net",
    "B short", "B doubles long", "B singles long",
)


def geometry(path=SOURCE):
    """Load and check dimensions from the authoritative simulation config."""
    source = Path(path)
    data = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Invalid shuttle_spawn.yaml root")
    try:
        court = data["court"]
        cluster = data["cluster"]
        single = data["single"]
        spawn = data["spawn"]
        length, width, margin = (float(court[k]) for k in
                                 ("length", "width", "margin"))
        line_x = [float(cluster["line_x"][key]) for key in ROW_KEYS]
        sigma_x = float(cluster["sigma_x"])
        sigma_y = float(cluster["sigma_y"])
        left_y = float(cluster["left_center_y"])
        right_y = float(cluster["right_center_y"])
        height = float(spawn["height"])
        single_pose = {k: float(single[k]) for k in
                       ("x", "y", "z", "roll", "pitch", "yaw")}
        weights = [[float(v) for v in row] for row in cluster["weights"]]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Invalid shuttle_spawn.yaml dimensions or weights") from exc
    values = [length, width, margin, *line_x, sigma_x, sigma_y,
              left_y, right_y, height, *single_pose.values(),
              *(v for row in weights for v in row)]
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Non-finite shuttle_spawn.yaml value")
    if not (length > 0 and width > 0 and 0 <= margin < min(length, width)/2
            and sigma_x >= 0 and sigma_y >= 0 and height >= 0
            and -width/2 <= right_y <= 0 <= left_y <= width/2
            and all(abs(x) <= length/2 for x in line_x)
            and len(weights) == 7 and all(len(row) == 2 for row in weights)
            and all(v >= 0 for row in weights for v in row)
            and sum(map(sum, weights)) > 0):
        raise ValueError("Out-of-range shuttle_spawn.yaml geometry or weights")
    return {
        "path": str(source),
        "court": {"length":length, "width":width, "margin":margin},
        "cluster": {"line_x":line_x, "sigma_x":sigma_x, "sigma_y":sigma_y,
                    "left_center_y":left_y, "right_center_y":right_y,
                    "weights": weights},
        "single": single_pose,
        "spawn": {"height":height},
    }


def _clamp(value, low, high):
    return max(low, min(high, value))


def _random_xy(rng, density, dims):
    length, width, margin = (dims["court"][key]
                             for key in ("length", "width", "margin"))
    hx = length/2 - margin
    hy = width/2 - margin
    if density == "uniform":
        return rng.uniform(-hx, hx), rng.uniform(-hy, hy)
    if density == "center":
        return (_clamp(rng.gauss(0, length/5), -hx, hx),
                _clamp(rng.gauss(0, width/5), -hy, hy))
    if density == "net":
        return (_clamp(rng.gauss(0, length*0.08), -hx, hx),
                rng.uniform(-hy, hy))
    raise ValueError("Unknown random density: " + str(density))


def _truncated(rng, mean, sigma, lower, upper):
    if sigma == 0:
        return _clamp(mean, lower, upper)
    for _ in range(64):
        value = rng.gauss(mean, sigma)
        if lower <= value <= upper:
            return value
    return _clamp(mean, lower, upper)


def _cluster(rng, count, cfg, dims):
    if count == 0:
        return []
    weights = cfg["cluster_weights"]
    total = sum(map(sum, weights))
    allocations = []
    for row_index, row in enumerate(weights):
        for side_index, weight in enumerate(row):
            exact = count*weight/total
            allocations.append([row_index, side_index, int(math.floor(exact)),
                                exact-math.floor(exact), rng.random()])
    remaining = count - sum(item[2] for item in allocations)
    for item in sorted(allocations, key=lambda p: (-p[3], p[4]))[:remaining]:
        item[2] += 1
    cluster = dims["cluster"]
    half_l = dims["court"]["length"]/2
    half_w = dims["court"]["width"]/2
    points = []
    for index, side, allocated, _, _ in allocations:
        x_mean = cluster["line_x"][index]
        y_mean = (cluster["left_center_y"] if side == 0
                  else cluster["right_center_y"])
        ylow, yhigh = (0, half_w) if side == 0 else (-half_w, 0)
        for _ in range(allocated):
            points.append((
                _truncated(rng, x_mean, cluster["sigma_x"], -half_l, half_l),
                _truncated(rng, y_mean, cluster["sigma_y"], ylow, yhigh),
                ROW_LABELS[index] + (" left" if side == 0 else " right"),
            ))
    rng.shuffle(points)
    return points


def sample(cfg, dims=None):
    """Return deterministic (x, y, zone) placements using authoritative geometry."""
    dims = geometry() if dims is None else dims
    rng = random.Random(cfg["seed"])
    mode, count = cfg["spawn_mode"], cfg["shuttle_count"]
    if mode == "single":
        return [(dims["single"]["x"], dims["single"]["y"], "single")]
    if mode == "random":
        return [(*_random_xy(rng, cfg["random_density"], dims), "random")
                for _ in range(count)]
    if mode == "cluster":
        return _cluster(rng, count, cfg, dims)
    if mode == "mixed":
        n_random = int(round(count*cfg["random_fraction"]))
        points = [(*_random_xy(rng, cfg["random_density"], dims), "random")
                  for _ in range(n_random)]
        points += _cluster(rng, count-n_random, cfg, dims)
        rng.shuffle(points)
        return points
    raise ValueError("Unknown spawn mode: " + str(mode))
