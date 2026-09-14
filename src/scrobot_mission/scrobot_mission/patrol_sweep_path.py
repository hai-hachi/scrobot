import math
from dataclasses import dataclass


@dataclass(frozen=True)
class SweepPoint:
    x: float
    y: float
    yaw: float
    distance: float
    lane_index: int
    is_lane: bool


@dataclass(frozen=True)
class RelocalizationStop:
    tag_id: int
    path_index: int
    path_distance: float
    x: float
    y: float
    sweep_yaw: float
    face_tag_yaw: float


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def tag_geometry(pole_x=0.0, left_pole_y=3.05, right_pole_y=-3.05,
                 tag_mount_radius=0.075, inward_angle_deg=45.0):
    """Return map-frame (x, y, inward_yaw) for the four court tags."""
    a = math.radians(inward_angle_deg)
    headings = {0: -a, 1: +a, 2: -math.pi + a, 3: +math.pi - a}
    result = {}
    for tag_id, heading in headings.items():
        pole_y = left_pole_y if tag_id in (0, 2) else right_pole_y
        result[tag_id] = (
            pole_x + tag_mount_radius * math.cos(heading),
            pole_y + tag_mount_radius * math.sin(heading),
            heading,
        )
    return result


def start_sides_for_tag(tag_id):
    """Return (start_from_positive_x, start_from_positive_y)."""
    if tag_id not in (0, 1, 2, 3):
        return False, False
    return tag_id in (0, 1), tag_id in (0, 2)


def _append_raw(raw, x, y, lane_index, is_lane):
    """Append one XY path sample while suppressing exact duplicates."""
    x = float(x)
    y = float(y)
    if raw:
        px, py, _, _ = raw[-1]
        if math.hypot(x - px, y - py) <= 1e-9:
            raw[-1] = (x, y, int(lane_index), bool(is_lane))
            return
    raw.append((x, y, int(lane_index), bool(is_lane)))


def _finalize_points(raw):
    """Convert sampled XY geometry to SweepPoint with tangent yaw and arc length."""
    if not raw:
        return []

    points = []
    distance = 0.0
    for i, (x, y, lane_index, is_lane) in enumerate(raw):
        if i > 0:
            px, py, _, _ = raw[i - 1]
            distance += math.hypot(x - px, y - py)

        if len(raw) == 1:
            yaw = 0.0
        elif i == 0:
            nx, ny, _, _ = raw[1]
            yaw = math.atan2(ny - y, nx - x)
        elif i == len(raw) - 1:
            px, py, _, _ = raw[i - 1]
            yaw = math.atan2(y - py, x - px)
        else:
            px, py, _, _ = raw[i - 1]
            nx, ny, _, _ = raw[i + 1]
            yaw = math.atan2(ny - py, nx - px)

        points.append(SweepPoint(
            x=x,
            y=y,
            yaw=wrap_angle(yaw),
            distance=distance,
            lane_index=lane_index,
            is_lane=is_lane,
        ))
    return points


def generate_sweep_path(court_length=13.40, court_width=6.10, passes=4,
                        sweep_extension=1.0, turn_samples=18,
                        path_resolution=0.20, start_tag_id=0):
    """Generate a camera-coverage serpentine sweep.

    ``sweep_extension`` is the deliberate *initial* outside entry distance. For
    the current camera proxy it is 1.0 m = 0.5 m camera-forward offset + 0.5 m
    requested outside coverage margin.

    Only the very first sweep entry is outside the court. Every U-turn is a true
    semicircle fully contained inside the court markings, and the final sweep
    endpoint is also inside the markings. The U-turn radius is half one lane
    spacing, so its tangent points are inset by that radius from the X boundary.

    The returned pose yaw is computed from the sampled path tangent instead of
    hand-written turn formulas. This avoids discontinuous / incorrect headings
    at U-turns and gives RPP a smoother path to follow.
    """
    passes = max(2, int(passes))
    turn_samples = max(6, int(turn_samples))
    path_resolution = max(0.05, float(path_resolution))
    sweep_extension = max(0.0, float(sweep_extension))

    x_half = 0.5 * float(court_length)
    y_half = 0.5 * float(court_width)
    lane_spacing = float(court_width) / passes
    turn_radius = 0.5 * lane_spacing

    # Tangency points are inset by one turn radius, making the complete
    # semicircle touch, but never cross, the court end marking.
    x_left_turn = -x_half + turn_radius
    x_right_turn = x_half - turn_radius

    # The first entry is intentionally outside. With the current 0.5 m camera
    # forward proxy and 0.5 m outside coverage margin this is +/-7.70 m.
    x_left_entry = -x_half - sweep_extension
    x_right_entry = x_half + sweep_extension

    lane_ys = [-y_half + (i + 0.5) * lane_spacing for i in range(passes)]
    start_from_right, start_from_positive_y = start_sides_for_tag(int(start_tag_id))
    if start_from_positive_y:
        lane_ys.reverse()

    raw = []
    current_from_right = bool(start_from_right)

    for lane_idx, y in enumerate(lane_ys):
        if lane_idx == 0:
            x_start = x_right_entry if current_from_right else x_left_entry
        else:
            x_start = x_right_turn if current_from_right else x_left_turn
        x_end = x_left_turn if current_from_right else x_right_turn

        lane_length = abs(x_end - x_start)
        samples = max(2, int(math.ceil(lane_length / path_resolution)) + 1)
        for j in range(samples):
            ratio = j / float(samples - 1)
            x = x_start + ratio * (x_end - x_start)
            _append_raw(raw, x, y, lane_idx, True)

        if lane_idx == passes - 1:
            break

        next_y = lane_ys[lane_idx + 1]
        center_y = 0.5 * (y + next_y)
        moving_to_higher_y = next_y > y
        start_theta = -math.pi / 2.0 if moving_to_higher_y else math.pi / 2.0
        theta_sign = 1.0 if moving_to_higher_y else -1.0

        # The circle center is at the straight-lane tangent X. Right-side turns
        # bulge +X; left-side turns bulge -X. Because the tangent is inset by r,
        # the outermost point lands exactly on the court end marking.
        center_x = x_end
        for j in range(1, turn_samples + 1):
            theta = start_theta + theta_sign * math.pi * j / turn_samples
            if x_end > 0.0:
                x = center_x + turn_radius * math.cos(theta)
            else:
                x = center_x - turn_radius * math.cos(theta)
            yy = center_y + turn_radius * math.sin(theta)
            _append_raw(raw, x, yy, lane_idx, False)

        current_from_right = not current_from_right

    return _finalize_points(raw)


def _ray_lane_intersection(tag_x, tag_y, tag_yaw, lane_y):
    dy = math.sin(tag_yaw)
    if abs(dy) < 1e-9:
        return None
    t = (lane_y - tag_y) / dy
    if t <= 0.0:
        return None
    return tag_x + t * math.cos(tag_yaw), lane_y, t


def choose_fixed_relocalization_stops(points, court_length=13.40,
                                      pole_x=0.0, left_pole_y=3.05,
                                      right_pole_y=-3.05,
                                      tag_mount_radius=0.075,
                                      inward_angle_deg=45.0,
                                      max_tag_distance=2.0):
    """Choose one deterministic fixed stop per tag.

    Each candidate is the intersection of the tag inward normal and one straight
    sweep pass. Passes facing more than 90 degrees away from the tag are rejected.
    Among the valid intersections, the closest-to-tag intersection is selected.
    """
    tags = tag_geometry(pole_x, left_pole_y, right_pole_y,
                        tag_mount_radius, inward_angle_deg)

    # Determine each lane direction from its actual straight samples. This is
    # independent of the smoothed tangent yaw at the U-turn boundary samples.
    lane_meta = {}
    lane_indices_by_id = {}
    for i, p in enumerate(points):
        if p.is_lane:
            lane_indices_by_id.setdefault(p.lane_index, []).append(i)
    for lane_idx, indices in lane_indices_by_id.items():
        if len(indices) < 2:
            continue
        first = points[indices[0]]
        last = points[indices[-1]]
        sweep_yaw = math.atan2(last.y - first.y, last.x - first.x)
        lane_meta[lane_idx] = (first.y, sweep_yaw, indices)

    x_limit = 0.5 * float(court_length) + 1e-6
    stops = []
    for tag_id in range(4):
        tx, ty, tag_yaw = tags[tag_id]
        candidates = []
        for lane_idx, (lane_y, sweep_yaw, lane_indices) in lane_meta.items():
            hit = _ray_lane_intersection(tx, ty, tag_yaw, lane_y)
            if hit is None:
                continue
            x, y, ray_distance = hit
            if abs(x) > x_limit or ray_distance > float(max_tag_distance):
                continue

            face_tag_yaw = math.atan2(ty - y, tx - x)
            if abs(wrap_angle(face_tag_yaw - sweep_yaw)) > math.pi / 2.0:
                continue

            path_index = min(
                lane_indices,
                key=lambda i: math.hypot(points[i].x - x, points[i].y - y),
            )
            path_distance = points[path_index].distance
            candidates.append((ray_distance, lane_idx, path_index, path_distance,
                               x, y, sweep_yaw, face_tag_yaw))

        if not candidates:
            continue
        best = min(candidates, key=lambda c: (c[0], c[1]))
        _, _, path_index, path_distance, x, y, sweep_yaw, face_tag_yaw = best
        stops.append(RelocalizationStop(
            tag_id=tag_id,
            path_index=path_index,
            path_distance=path_distance,
            x=x,
            y=y,
            sweep_yaw=sweep_yaw,
            face_tag_yaw=face_tag_yaw,
        ))

    stops.sort(key=lambda s: s.path_index)
    return stops


def nearest_path_index(points, x, y, start_index=0):
    if not points:
        return 0
    start_index = max(0, min(int(start_index), len(points) - 1))
    return min(
        range(start_index, len(points)),
        key=lambda i: math.hypot(points[i].x - x, points[i].y - y),
    )
