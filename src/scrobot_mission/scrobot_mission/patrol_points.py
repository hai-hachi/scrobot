import math


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
