#!/usr/bin/env python3

import math


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _linspace(a, b, count):
    if count <= 1:
        return [0.5 * (a + b)]
    step = (b - a) / float(count - 1)
    return [a + i * step for i in range(count)]


def _append_unique(points, point, eps=1e-9):
    x, y = point
    if points:
        px, py = points[-1]
        if abs(px - x) <= eps and abs(py - y) <= eps:
            return
    points.append((float(x), float(y)))


def _sample_line(x0, y0, x1, y1, spacing):
    length = math.hypot(x1 - x0, y1 - y0)
    segments = max(1, int(math.ceil(length / spacing)))
    return [
        (
            x0 + (x1 - x0) * i / float(segments),
            y0 + (y1 - y0) * i / float(segments),
        )
        for i in range(segments + 1)
    ]


def _sample_semicircle(cx, cy, radius, start_angle, end_angle, spacing):
    arc_length = abs(end_angle - start_angle) * radius
    segments = max(4, int(math.ceil(arc_length / spacing)))
    return [
        (
            cx + radius * math.cos(start_angle + (end_angle - start_angle) * i / float(segments)),
            cy + radius * math.sin(start_angle + (end_angle - start_angle) * i / float(segments)),
        )
        for i in range(segments + 1)
    ]


def orient_polyline(points):
    """Return (x, y, yaw) using centered tangents where possible."""
    if not points:
        return []

    result = []
    for i, (x, y) in enumerate(points):
        if len(points) == 1:
            yaw = 0.0
        elif i == 0:
            nx, ny = points[1]
            yaw = math.atan2(ny - y, nx - x)
        elif i == len(points) - 1:
            px, py = points[i - 1]
            yaw = math.atan2(y - py, x - px)
        else:
            px, py = points[i - 1]
            nx, ny = points[i + 1]
            yaw = math.atan2(ny - py, nx - px)
        result.append((float(x), float(y), float(yaw)))
    return result


def reverse_oriented_polyline(route):
    """Reverse an (x, y, yaw) route and recompute tangent headings."""
    return orient_polyline([(x, y) for x, y, _ in reversed(route)])


def generate_snake_sweep(
    court_length,
    court_width,
    lane_spacing,
    waypoint_spacing,
    margin_x=0.45,
    margin_y=0.45,
):
    """Generate a C1-like lawnmower path with tangent semicircle U-turns.

    Straight survey lanes are joined by semicircles whose diameter is exactly
    the actual lane spacing. The straight lane endpoints are moved inward by
    one turn radius, so the semicircle bulge remains inside the requested court
    margins. This removes the old 90-deg corner + vertical connector + 90-deg
    corner pattern and gives a differential-drive robot a continuous-curvature
    direction change that RPP can track without stop/rotate behavior.
    """
    if court_length <= 0.0 or court_width <= 0.0:
        raise ValueError('Court dimensions must be positive.')
    if lane_spacing <= 0.0 or waypoint_spacing <= 0.0:
        raise ValueError('Sweep spacing must be positive.')

    x_min = -0.5 * court_length + margin_x
    x_max = 0.5 * court_length - margin_x
    y_min = -0.5 * court_width + margin_y
    y_max = 0.5 * court_width - margin_y
    if x_min >= x_max or y_min >= y_max:
        raise ValueError('Sweep margins leave no usable court area.')

    usable_width = y_max - y_min
    lane_intervals = max(1, int(math.ceil(usable_width / lane_spacing)))
    lane_count = lane_intervals + 1
    lane_ys = _linspace(y_min, y_max, lane_count)
    actual_lane_spacing = usable_width / float(lane_intervals)

    turn_radius = 0.5 * actual_lane_spacing
    left_lane_x = x_min + turn_radius
    right_lane_x = x_max - turn_radius
    if left_lane_x >= right_lane_x:
        raise ValueError(
            'Court is too short for semicircle connectors at this lane spacing.'
        )

    points = []

    for lane_index, y in enumerate(lane_ys):
        moving_right = lane_index % 2 == 0
        start_x = left_lane_x if moving_right else right_lane_x
        end_x = right_lane_x if moving_right else left_lane_x

        line = _sample_line(start_x, y, end_x, y, waypoint_spacing)
        for point in line:
            _append_unique(points, point)

        if lane_index >= lane_count - 1:
            continue

        next_y = lane_ys[lane_index + 1]
        cy = 0.5 * (y + next_y)

        if moving_right:
            # Start tangent points +X; end tangent points -X.
            arc = _sample_semicircle(
                right_lane_x,
                cy,
                turn_radius,
                -0.5 * math.pi,
                0.5 * math.pi,
                waypoint_spacing,
            )
        else:
            # Start tangent points -X; end tangent points +X.
            arc = _sample_semicircle(
                left_lane_x,
                cy,
                turn_radius,
                1.5 * math.pi,
                0.5 * math.pi,
                waypoint_spacing,
            )

        for point in arc:
            _append_unique(points, point)

    route = orient_polyline(points)
    return route, {
        'lane_count': lane_count,
        'actual_lane_spacing': actual_lane_spacing,
        'turn_radius': turn_radius,
        'waypoint_count': len(route),
        'x_min': x_min,
        'x_max': x_max,
        'y_min': y_min,
        'y_max': y_max,
        'straight_x_min': left_lane_x,
        'straight_x_max': right_lane_x,
    }


def route_length(start_xy, order, positions):
    if not order:
        return 0.0
    px, py = start_xy
    total = 0.0
    for key in order:
        x, y = positions[key][:2]
        total += math.hypot(x - px, y - py)
        px, py = x, y
    return total


def nearest_neighbor_order(start_xy, positions):
    """Open-route nearest-neighbor seed; no return to the start."""
    remaining = set(positions.keys())
    order = []
    px, py = start_xy

    while remaining:
        next_key = min(
            remaining,
            key=lambda key: math.hypot(
                positions[key][0] - px,
                positions[key][1] - py,
            ),
        )
        order.append(next_key)
        px, py = positions[next_key][:2]
        remaining.remove(next_key)

    return order


def two_opt_open(start_xy, order, positions, max_passes=100):
    """2-opt improvement for an open route beginning at fixed start_xy."""
    best = list(order)
    if len(best) < 3:
        return best

    best_cost = route_length(start_xy, best, positions)
    eps = 1e-9

    for _ in range(max(1, int(max_passes))):
        improved = False
        n = len(best)
        for i in range(n - 1):
            for j in range(i + 1, n):
                if i == 0 and j == n - 1 and n <= 3:
                    continue
                candidate = best[:i] + list(reversed(best[i:j + 1])) + best[j + 1:]
                cost = route_length(start_xy, candidate, positions)
                if cost + eps < best_cost:
                    best = candidate
                    best_cost = cost
                    improved = True
                    break
            if improved:
                break
        if not improved:
            break

    return best


def plan_collection_order(start_xy, positions, max_two_opt_passes=100):
    seed = nearest_neighbor_order(start_xy, positions)
    optimized = two_opt_open(
        start_xy,
        seed,
        positions,
        max_passes=max_two_opt_passes,
    )
    return optimized, {
        'nearest_neighbor_length': route_length(start_xy, seed, positions),
        'optimized_length': route_length(start_xy, optimized, positions),
    }
