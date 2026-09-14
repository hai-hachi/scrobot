import math


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def find_minimum_grid(
    court_length,
    court_width,
    effective_range,
    max_grid_size=20,
):
    """Return the smallest rectangular grid that covers the court.

    Each patrol point is placed at a cell center. Coverage is accepted when the
    farthest cell corner is no farther than effective_range from its center.
    """
    best = None

    for nx in range(1, max_grid_size + 1):
        for ny in range(1, max_grid_size + 1):
            dx = court_length / nx
            dy = court_width / ny
            worst_case_distance = 0.5 * math.hypot(dx, dy)

            if worst_case_distance > effective_range:
                continue

            candidate = (
                nx * ny,
                worst_case_distance,
                nx,
                ny,
                dx,
                dy,
            )

            if best is None or candidate[:2] < best[:2]:
                best = candidate

    if best is None:
        raise RuntimeError('Could not generate a patrol grid.')

    _, worst_case_distance, nx, ny, dx, dy = best
    return nx, ny, dx, dy, worst_case_distance


def generate_patrol_points(court_length, court_width, nx, ny):
    """Generate a serpentine list of (x, y, yaw) patrol points."""
    dx = court_length / nx
    dy = court_width / ny
    x_min = -court_length / 2.0
    y_min = -court_width / 2.0

    points = []

    for row in range(ny):
        y = y_min + dy / 2.0 + row * dy
        xs = [
            x_min + dx / 2.0 + col * dx
            for col in range(nx)
        ]

        if row % 2 == 1:
            xs.reverse()

        yaw = 0.0 if row % 2 == 0 else math.pi
        for x in xs:
            points.append((x, y, yaw))

    return points


def generate_tag_watch_poses(
    target_distance=1.70,
    pole_x=0.0,
    left_pole_y=3.05,
    right_pole_y=-3.05,
    tag_mount_radius=0.075,
    inward_angle_deg=45.0,
):
    """Return the four known initial relocalization watch poses.

    Each item is ``tag_id: (x, y, yaw)``. The geometry matches the court tag
    generator: two inward-facing tags on each pole. ``yaw`` is the robot yaw at
    the normal-incidence watch pose, i.e. facing back toward the tag.
    """
    a = math.radians(inward_angle_deg)
    mounts = {
        0: (pole_x, left_pole_y, -a),
        1: (pole_x, right_pole_y, +a),
        2: (pole_x, left_pole_y, -math.pi + a),
        3: (pole_x, right_pole_y, math.pi - a),
    }

    watches = {}
    for tag_id, (pole_cx, pole_cy, mount_yaw) in mounts.items():
        tag_x = pole_cx + tag_mount_radius * math.cos(mount_yaw)
        tag_y = pole_cy + tag_mount_radius * math.sin(mount_yaw)
        watch_x = tag_x + target_distance * math.cos(mount_yaw)
        watch_y = tag_y + target_distance * math.sin(mount_yaw)
        watch_yaw = wrap_angle(mount_yaw + math.pi)
        watches[tag_id] = (watch_x, watch_y, watch_yaw)

    return watches


def _route_indices(count, start_index, direction):
    if direction > 0:
        return [
            (start_index + step) % count
            for step in range(count)
        ]
    return [
        (start_index - step) % count
        for step in range(count)
    ]


def _segment_heading(point_a, point_b):
    return math.atan2(point_b[1] - point_a[1], point_b[0] - point_a[0])


def _orient_route(points):
    """Set every route point yaw to the direction of the next patrol leg."""
    if not points:
        return []
    if len(points) == 1:
        return list(points)

    oriented = []
    for i, (x, y, _yaw) in enumerate(points):
        if i < len(points) - 1:
            yaw = _segment_heading(points[i], points[i + 1])
        else:
            yaw = _segment_heading(points[i - 1], points[i])
        oriented.append((x, y, yaw))
    return oriented


def choose_tag_aligned_patrol_route(
    patrol_points,
    watch_pose,
    heading_weight_m_per_rad=1.50,
):
    """Choose the best patrol entry and traversal direction for one tag.

    Pure nearest-point selection is intentionally *not* used. The robot exits
    initial relocalization with a known position and yaw, so the entry score is

        distance_to_point + heading_weight * required_turn.

    This prefers a slightly farther patrol point when it lies naturally in
    front of the robot. After choosing the entry point, forward vs reverse
    traversal is selected by whichever first patrol leg best matches the
    incoming heading.

    Returns ``(route, metadata)`` where ``route`` is a reordered/oriented list
    of ``(x, y, yaw)`` tuples.
    """
    if not patrol_points:
        return [], {
            'start_index': -1,
            'direction': 1,
            'entry_distance': float('inf'),
            'entry_turn': float('inf'),
        }

    wx, wy, wyaw = watch_pose
    best = None

    for index, point in enumerate(patrol_points):
        dx = point[0] - wx
        dy = point[1] - wy
        distance = math.hypot(dx, dy)
        heading = math.atan2(dy, dx) if distance > 1e-9 else wyaw
        turn = abs(wrap_angle(heading - wyaw))
        score = distance + heading_weight_m_per_rad * turn
        candidate = (score, distance, turn, index, heading)
        if best is None or candidate < best:
            best = candidate

    _score, entry_distance, entry_turn, start_index, incoming_heading = best
    count = len(patrol_points)

    if count == 1:
        route = _orient_route([patrol_points[start_index]])
        return route, {
            'start_index': start_index,
            'direction': 1,
            'entry_distance': entry_distance,
            'entry_turn': entry_turn,
        }

    forward_indices = _route_indices(count, start_index, +1)
    reverse_indices = _route_indices(count, start_index, -1)

    forward_heading = _segment_heading(
        patrol_points[forward_indices[0]], patrol_points[forward_indices[1]]
    )
    reverse_heading = _segment_heading(
        patrol_points[reverse_indices[0]], patrol_points[reverse_indices[1]]
    )
    forward_turn = abs(wrap_angle(forward_heading - incoming_heading))
    reverse_turn = abs(wrap_angle(reverse_heading - incoming_heading))

    if reverse_turn < forward_turn:
        direction = -1
        indices = reverse_indices
        first_leg_turn = reverse_turn
    else:
        direction = +1
        indices = forward_indices
        first_leg_turn = forward_turn

    route = _orient_route([patrol_points[i] for i in indices])
    return route, {
        'start_index': start_index,
        'direction': direction,
        'entry_distance': entry_distance,
        'entry_turn': entry_turn,
        'first_leg_turn': first_leg_turn,
    }


def precompute_tag_patrol_routes(
    patrol_points,
    target_distance=1.70,
    heading_weight_m_per_rad=1.50,
    pole_x=0.0,
    left_pole_y=3.05,
    right_pole_y=-3.05,
    tag_mount_radius=0.075,
    inward_angle_deg=45.0,
):
    """Precompute one optimized patrol route for each known AprilTag ID."""
    watches = generate_tag_watch_poses(
        target_distance=target_distance,
        pole_x=pole_x,
        left_pole_y=left_pole_y,
        right_pole_y=right_pole_y,
        tag_mount_radius=tag_mount_radius,
        inward_angle_deg=inward_angle_deg,
    )

    routes = {}
    for tag_id, watch_pose in watches.items():
        route, metadata = choose_tag_aligned_patrol_route(
            patrol_points,
            watch_pose,
            heading_weight_m_per_rad=heading_weight_m_per_rad,
        )
        routes[tag_id] = {
            'watch_pose': watch_pose,
            'route': route,
            **metadata,
        }
    return routes
