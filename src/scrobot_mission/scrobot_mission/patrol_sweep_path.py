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
    """Return map-frame (x, y, inward_yaw) for the four court tags.

    This mirrors tag_global_localizer.compute_tag_headings().
    """
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
    """Return (start_from_positive_x, start_from_positive_y).

    The tag heading identifies the court quadrant it faces. Tags 0/1 face the
    +X half and tags 2/3 face the -X half; tags 0/2 are on +Y and tags 1/3 on
    -Y. This makes initial sweep entry deterministic after relocalization.
    """
    if tag_id not in (0, 1, 2, 3):
        return False, False
    return tag_id in (0, 1), tag_id in (0, 2)


def _append_point(points, x, y, yaw, lane_index, is_lane):
    if points:
        prev = points[-1]
        distance = prev.distance + math.hypot(x - prev.x, y - prev.y)
    else:
        distance = 0.0
    points.append(SweepPoint(float(x), float(y), wrap_angle(yaw), distance,
                             int(lane_index), bool(is_lane)))


def generate_sweep_path(court_length=13.40, court_width=6.10, passes=4,
                        sweep_extension=1.0, turn_samples=12, start_tag_id=0):
    """Generate a continuous four-pass serpentine sweep.

    Straight passes run along court X. Their first/last endpoints extend beyond
    the court by sweep_extension so the accumulated forward camera footprint can
    cover the end boundaries. U-turns are sampled half-circles outside the X
    boundary to avoid point turns.
    """
    passes = max(2, int(passes))
    turn_samples = max(3, int(turn_samples))
    x_half = 0.5 * float(court_length)
    y_half = 0.5 * float(court_width)
    x_left = -x_half - float(sweep_extension)
    x_right = x_half + float(sweep_extension)

    # Equal-width coverage strips: lane center is at the center of each strip.
    lane_spacing = float(court_width) / passes
    lane_ys = [-y_half + (i + 0.5) * lane_spacing for i in range(passes)]

    start_pos_x, start_pos_y = start_sides_for_tag(int(start_tag_id))
    if start_pos_y:
        lane_ys.reverse()

    points = []
    current_from_right = bool(start_pos_x)

    for lane_idx, y in enumerate(lane_ys):
        x_start = x_right if current_from_right else x_left
        x_end = x_left if current_from_right else x_right
        yaw = math.pi if current_from_right else 0.0
        _append_point(points, x_start, y, yaw, lane_idx, True)
        _append_point(points, x_end, y, yaw, lane_idx, True)

        if lane_idx == passes - 1:
            break

        next_y = lane_ys[lane_idx + 1]
        center_y = 0.5 * (y + next_y)
        radius = 0.5 * abs(next_y - y)
        side_x = x_end

        # Semicircle tangent to both straight lanes. Parameterization depends on
        # whether the robot reached the left or right side of the court.
        if x_end > 0.0:
            # At right edge: heading +X, bulge further +X, finish heading -X.
            start_angle = -math.pi / 2.0 if next_y > y else math.pi / 2.0
            sign = 1.0 if next_y > y else -1.0
            for j in range(1, turn_samples + 1):
                theta = start_angle + sign * math.pi * j / turn_samples
                x = side_x + radius * math.cos(theta)
                yy = center_y + radius * math.sin(theta)
                tangent_yaw = theta + sign * math.pi / 2.0
                _append_point(points, x, yy, tangent_yaw, lane_idx, False)
        else:
            # At left edge: heading -X, bulge further -X, finish heading +X.
            start_angle = math.pi / 2.0 if next_y > y else -math.pi / 2.0
            sign = 1.0 if next_y > y else -1.0
            for j in range(1, turn_samples + 1):
                theta = start_angle + sign * math.pi * j / turn_samples
                x = side_x - radius * math.cos(theta)
                yy = center_y + radius * math.sin(theta)
                tangent_yaw = math.pi - (theta + sign * math.pi / 2.0)
                _append_point(points, x, yy, tangent_yaw, lane_idx, False)

        current_from_right = not current_from_right

    return points


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
    """Choose exactly one deterministic sweep stop per tag.

    Candidate stops are intersections of a tag inward normal with a straight
    sweep pass. Intersections outside the court-length sweep span or beyond the
    allowed tag range are rejected. For each tag, prefer a pass whose travel
    direction faces generally toward the tag (|bearing| <= 90 deg), then choose
    the closest valid intersection. No runtime rotation cost is used.
    """
    tags = tag_geometry(pole_x, left_pole_y, right_pole_y,
                        tag_mount_radius, inward_angle_deg)

    # Lane metadata from the straight endpoints. Each lane is represented by
    # the first lane point encountered in traversal order.
    lane_meta = {}
    for idx, p in enumerate(points):
        if p.is_lane and p.lane_index not in lane_meta:
            lane_meta[p.lane_index] = (p.y, p.yaw)

    x_limit = 0.5 * float(court_length) + 1e-6
    stops = []
    for tag_id in range(4):
        tx, ty, tag_yaw = tags[tag_id]
        candidates = []
        for lane_idx, (lane_y, sweep_yaw) in lane_meta.items():
            hit = _ray_lane_intersection(tx, ty, tag_yaw, lane_y)
            if hit is None:
                continue
            x, y, ray_distance = hit
            if abs(x) > x_limit or ray_distance > float(max_tag_distance):
                continue

            face_tag_yaw = math.atan2(ty - y, tx - x)
            bearing_from_sweep = abs(wrap_angle(face_tag_yaw - sweep_yaw))
            if bearing_from_sweep > math.pi / 2.0:
                continue

            # Locate the closest sampled path point on this straight lane.
            lane_indices = [
                i for i, p in enumerate(points)
                if p.is_lane and p.lane_index == lane_idx
            ]
            if not lane_indices:
                continue
            path_index = min(
                lane_indices,
                key=lambda i: math.hypot(points[i].x - x, points[i].y - y),
            )
            # We store the exact analytic stop position while using path_index
            # only as a traversal-order marker.
            path_distance = points[path_index].distance + abs(x - points[path_index].x)
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

    stops.sort(key=lambda s: s.path_distance)
    return stops


def nearest_path_index(points, x, y, start_index=0):
    if not points:
        return 0
    start_index = max(0, min(int(start_index), len(points) - 1))
    return min(
        range(start_index, len(points)),
        key=lambda i: math.hypot(points[i].x - x, points[i].y - y),
    )
