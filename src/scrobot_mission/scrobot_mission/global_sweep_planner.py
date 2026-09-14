#!/usr/bin/env python3

import math


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _linspace(a, b, count):
    if count <= 1:
        return [0.5 * (a + b)]
    step = (b - a) / float(count - 1)
    return [a + i * step for i in range(count)]


def orient_polyline(points):
    """Return (x, y, yaw) for a 2-D polyline."""
    if not points:
        return []

    result = []
    last_yaw = 0.0
    for i, (x, y) in enumerate(points):
        if len(points) == 1:
            yaw = last_yaw
        elif i < len(points) - 1:
            nx, ny = points[i + 1]
            yaw = math.atan2(ny - y, nx - x)
        else:
            px, py = points[i - 1]
            yaw = math.atan2(y - py, x - px)
        last_yaw = yaw
        result.append((float(x), float(y), float(yaw)))
    return result


def reverse_oriented_polyline(route):
    """Reverse an (x, y, yaw) route and recompute headings."""
    return orient_polyline([(x, y) for x, y, _ in reversed(route)])


def generate_snake_sweep(
    court_length,
    court_width,
    lane_spacing,
    waypoint_spacing,
    margin_x=0.45,
    margin_y=0.45,
):
    """Generate a dense lawnmower path covering the usable court rectangle.

    Lane count is chosen so the actual lateral separation never exceeds
    lane_spacing. Dense points along each lane discourage Nav2 from cutting
    corners between only two distant lane endpoints.
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

    usable_length = x_max - x_min
    segment_count = max(1, int(math.ceil(usable_length / waypoint_spacing)))
    lane_xs = _linspace(x_min, x_max, segment_count + 1)

    points = []
    for lane_index, y in enumerate(lane_ys):
        xs = lane_xs if lane_index % 2 == 0 else list(reversed(lane_xs))
        for x in xs:
            # Avoid duplicating the exact connector point if geometry ever
            # produces one. Normally y changes, so both lane endpoints remain.
            if points and abs(points[-1][0] - x) < 1e-9 and abs(points[-1][1] - y) < 1e-9:
                continue
            points.append((x, y))

    route = orient_polyline(points)
    actual_lane_spacing = usable_width / float(lane_intervals)
    return route, {
        'lane_count': lane_count,
        'actual_lane_spacing': actual_lane_spacing,
        'waypoint_count': len(route),
        'x_min': x_min,
        'x_max': x_max,
        'y_min': y_min,
        'y_max': y_max,
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
