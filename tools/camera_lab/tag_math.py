"""Standalone AprilTags Lab geometry and ground-truth comparisons.

The simulation tags have planar +X normals and 90 degree separation on
each pole. The detector uses camera optical frame (+Z forward, +X right).
The camera transform matches the RealSense mount on scrobot_description:
  screw xyz=(0.110, 0, 0.2275), pitch=+15 degrees
  screw -> link=(0.0106, 0.0175, 0.0125)
  link -> color=(0, 0.015, 0).
"""
from __future__ import annotations
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

# Use the Lighting Lab's CALIBRATED venue clearance and avoid positioning
# beyond the floor already published in the court SDF.
WS = Path(__file__).resolve().parents[2]
LIGHTING_CONFIG = WS / "src/scrobot_simulation/config/lighting_lab.yaml"
PUBLISHED_COURT = WS / "src/scrobot_simulation/worlds/badminton_court.sdf"
COURT_LENGTH, COURT_WIDTH = 15.4, 7.2
ROBOT_EDGE_MARGIN = 0.30  # keep camera proxy inside the outer venue boundary


def venue_bounds(lighting_path=LIGHTING_CONFIG, court_path=PUBLISHED_COURT):
    settings = yaml.safe_load(Path(lighting_path).read_text(encoding="utf-8"))
    venue = settings["venue"]
    cx, cy = float(venue["clearance_x"]), float(venue["clearance_y"])
    if not all(math.isfinite(v) and v >= 0 for v in (cx, cy)):
        raise ValueError("Lighting Lab venue clearances must be finite and nonnegative")
    configured_x = COURT_LENGTH + 2 * cx
    configured_y = COURT_WIDTH + 2 * cy
    published = ET.parse(court_path).getroot().find(
        './/model[@name="court_floor"]/link/collision/geometry/box/size'
    )
    if published is None or not published.text:
        raise ValueError("Cannot find published court floor bounds")
    dimensions = [float(x) for x in published.text.split()]
    floor_x = min(configured_x, dimensions[0])
    floor_y = min(configured_y, dimensions[1])
    if floor_x / 2 <= ROBOT_EDGE_MARGIN or floor_y / 2 <= ROBOT_EDGE_MARGIN:
        raise ValueError("Venue floor too small for robot placement")
    return {
        "clearance_x": cx, "clearance_y": cy,
        "floor_x": floor_x, "floor_y": floor_y,
        "limit_x": round(floor_x / 2 - ROBOT_EDGE_MARGIN, 5),
        "limit_y": round(floor_y / 2 - ROBOT_EDGE_MARGIN, 5),
        "lighting_config_matches_published_floor": (
            abs(configured_x - dimensions[0]) < 0.001 and
            abs(configured_y - dimensions[1]) < 0.001
        ),
    }


VENUE = venue_bounds()

POLE_RADIUS = 0.025
PLATE_QUIET_BORDER_FACTOR = 8 / 6
INWARD_ANGLE_DEG = 45.0
POLES = {0: (0.0, +3.05), 1: (0.0, -3.05),
         2: (0.0, +3.05), 3: (0.0, -3.05)}
CAMERA_OFFSET = (0.1206, 0.0325, 0.2400)
CAMERA_PITCH = math.radians(15.0)
MIN_DISTANCE, MAX_DISTANCE = 0.4, 12.0

TAG_DEFAULT = {
    "size": 0.10, "height": 0.15, "noise_sigma": 0.0,
    "tag_id": 0, "robot_x": 1.47, "robot_y": 1.58, "robot_yaw": 135.0,
}


def validate(tag):
    if not isinstance(tag, dict) or set(tag) != set(TAG_DEFAULT):
        raise ValueError("Unexpected AprilTag settings keys")
    result = {}
    for key, low, high in (
        ("size", 0.04, 0.25), ("height", 0.12, 1.50),
        ("noise_sigma", 0.0, 50.0),
        ("robot_x", -VENUE["limit_x"], VENUE["limit_x"]),
        ("robot_y", -VENUE["limit_y"], VENUE["limit_y"]),
        ("robot_yaw", -180.0, 180.0),
    ):
        value = tag[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(key + " must be numeric")
        value = float(value)
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(key + f" must be in [{low}, {high}]")
        result[key] = value
    if result["height"] < result["size"] * PLATE_QUIET_BORDER_FACTOR / 2 + 0.008:
        raise ValueError("Tag height must clear the court floor by 8 mm at the selected plate size")
    if type(tag["tag_id"]) is not int or tag["tag_id"] not in POLES:
        raise ValueError("Tag ID must be 0..3")
    result["tag_id"] = tag["tag_id"]
    return result


def heading(tag_id):
    a = math.radians(INWARD_ANGLE_DEG)
    return {0: -a, 1: a, 2: -math.pi + a, 3: math.pi - a}[tag_id]


def mount_radius(size, clearance=0.010):
    """Minimum plate-center offset to avoid intersection on same pole.

    Two surface planes with angle Δ=90° and equal normal offsets R intersect
    at a tangential distance R*tan(Δ/2) from each center. Keep that greater
    than half of the full textured plate width (active edge + quiet border),
    with a positive margin; also clear the actual 25 mm pole radius.
    """
    plate = size * PLATE_QUIET_BORDER_FACTOR
    delta = math.radians(90.0)
    return round(max(
        (plate / 2 + clearance) / math.tan(delta / 2),
        POLE_RADIUS + 0.006), 6)


def plate_separation_margin(size, radius):
    """Positive gap along the projected tangential intersection direction."""
    return radius * math.tan(math.pi / 4) - size * PLATE_QUIET_BORDER_FACTOR / 2


def tag_pose(tag_id, config):
    px, py = POLES[tag_id]
    a = heading(tag_id)
    r = mount_radius(config["size"])
    return {
        "x": px + r * math.cos(a),
        "y": py + r * math.sin(a),
        "z": config["height"], "yaw": a,
    }


def head_on_pose(tag_id, distance, config, heading_offset_deg=0.0):
    """Robot base pose, where distance is horizontal base-to-tag distance."""
    if not MIN_DISTANCE <= distance <= MAX_DISTANCE:
        raise ValueError("Head-on test distance outside [0.4, 12.0] m")
    t = tag_pose(tag_id, config)
    a = t["yaw"]
    x = t["x"] + distance * math.cos(a)
    y = t["y"] + distance * math.sin(a)
    yaw = (math.degrees(a) + 180 + heading_offset_deg + 180) % 360 - 180
    if not (-VENUE["limit_x"] <= x <= VENUE["limit_x"] and
            -VENUE["limit_y"] <= y <= VENUE["limit_y"]):
        raise ValueError(
            f"Robot pose ({x:.2f}, {y:.2f}) m would exceed the Lighting Lab "
            f"venue limits (+/-{VENUE['limit_x']:.2f} X, "
            f"+/-{VENUE['limit_y']:.2f} Y) m. "
            "Reduce sweep distance or publish a larger venue."
        )
    return {"robot_x": round(x, 5), "robot_y": round(y, 5),
            "robot_yaw": round(yaw, 5)}


def optical_vector(robot_x, robot_y, robot_yaw, world_vector):
    """World displacement -> camera optical frame with pitched robot mount."""
    theta = math.radians(robot_yaw)
    ct, st = math.cos(theta), math.sin(theta)
    wx, wy, wz = world_vector
    # inverse base yaw
    bx, by, bz = ct * wx + st * wy, -st * wx + ct * wy, wz
    # inverse camera pitch
    cp, sp = math.cos(CAMERA_PITCH), math.sin(CAMERA_PITCH)
    cx, cy, cz = cp * bx - sp * bz, by, sp * bx + cp * bz
    # camera frame X-forward/Y-left/Z-up -> optical X-right/Y-down/Z-forward
    return (-cy, -cz, cx)


def expected_translation(config):
    tag = tag_pose(config["tag_id"], config)
    theta = math.radians(config["robot_yaw"])
    ct, st = math.cos(theta), math.sin(theta)
    dx, dy, dz = CAMERA_OFFSET
    camera_x = config["robot_x"] + ct * dx - st * dy
    camera_y = config["robot_y"] + st * dx + ct * dy
    camera_z = dz
    return optical_vector(
        config["robot_x"], config["robot_y"], config["robot_yaw"],
        (tag["x"]-camera_x, tag["y"]-camera_y, tag["z"]-camera_z)
    )


def expected_normal(config):
    a = heading(config["tag_id"])
    return optical_vector(config["robot_x"], config["robot_y"],
                          config["robot_yaw"], (math.cos(a), math.sin(a), 0))


def rotate_quaternion(q, v):
    x, y, z, w = q
    # q v q^-1 using double cross product
    tx = 2 * (y*v[2] - z*v[1])
    ty = 2 * (z*v[0] - x*v[2])
    tz = 2 * (x*v[1] - y*v[0])
    return (v[0] + w*tx + y*tz - z*ty,
            v[1] + w*ty + z*tx - x*tz,
            v[2] + w*tz + x*ty - y*tx)


def optical_frame_matches(tf_parent, image_frame):
    """Only compare TF translation directly when its frame matches the image.

    Gazebo Harmonic's ros_gz_image bridge can publish 'color_optical_frame'
    even if the sensor SDF requests 'camera_color_optical_frame'.
    We must use the ACTUAL incoming image frame, not a hard-coded alias.
    """
    a = str(tf_parent or "").lstrip("/")
    b = str(image_frame or "").lstrip("/")
    return bool(a and b and a == b and a.endswith("_optical_frame"))


def comparison(config, position, quaternion=None):
    """Pose error vs known Gazebo geometry at the commanded static base pose.

    Translation is in camera optical axes. Normal-axis comparison uses
    unsigned plate normal (not full roll/pitch/yaw from tag texture rotation).
    """
    truth = expected_translation(config)
    if not all(math.isfinite(v) for v in position):
        return None
    delta = [float(position[i])-truth[i] for i in range(3)]
    err = math.sqrt(sum(v*v for v in delta))
    measured_range = math.sqrt(sum(v*v for v in position))
    true_range = math.sqrt(sum(v*v for v in truth))
    bearing = math.degrees(math.atan2(position[0],position[2])) if position[2] else 90.0
    true_bearing = math.degrees(math.atan2(truth[0],truth[2])) if truth[2] else 90.0
    diff_bearing = (bearing-true_bearing+180)%360-180
    result = {
        "position_error_m": err,
        "range_error_m": measured_range-true_range,
        "bearing_error_deg": diff_bearing,
        "true_range_m": true_range,
        "measured_range_m": measured_range,
    }
    if quaternion is not None:
        normal = expected_normal(config)
        estimated = rotate_quaternion(quaternion, (0, 0, 1))
        dot = sum(normal[i]*estimated[i] for i in range(3))
        dot = max(-1.0, min(1.0, dot))
        result["normal_axis_error_deg"] = math.degrees(math.acos(abs(dot)))
    return result


def sweep_values(start, end, step, limit=45):
    for v in (start, end, step):
        if not isinstance(v, (int, float)) or not math.isfinite(float(v)):
            raise ValueError("Sweep values must be finite")
    if step <= 0 or end < start or start == end:
        raise ValueError("Sweep needs an increasing range and positive step")
    count = int(math.floor((end-start)/step + 1e-9))+1
    if count > limit or count < 2:
        raise ValueError(f"Sweep must have 2..{limit} points")
    values = [round(start + i*step, 5) for i in range(count)]
    if end - values[-1] > step*0.01 and len(values) < limit:
        values.append(round(end, 5))
    return values
