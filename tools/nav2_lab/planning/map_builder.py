"""Generate a free-space court map from SCROBOT's actual SDF and landmarks.

Only the net poles and AprilTag mount plates are permanently occupied. Court
lines and the net are VISUAL ONLY. The floor supports motion but is free space.
The map frame is odom and requires spawning the Gazebo robot at world (0,0,0).
"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def _pose_xy(text):
    parts = (text or "0 0 0 0 0 0").split()
    return float(parts[0]), float(parts[1])


def court_geometry(world_path, landmarks_path):
    world = ET.parse(world_path).getroot()
    floor = world.find(".//model[@name='court_floor']/link/collision[@name='floor_collision']/geometry/box/size")
    if floor is None:
        raise ValueError("SCROBOT floor collision box not found in world SDF")
    width, height, _ = map(float, floor.text.split())
    poles = []
    for collision in world.findall(".//model[@name='net_assembly']/link/collision"):
        cylinder = collision.find("geometry/cylinder/radius")
        if cylinder is not None:
            x, y = _pose_xy(collision.findtext("pose"))
            poles.append({"x": x, "y": y, "radius": float(cylinder.text),
                          "name": collision.get("name")})
    if len(poles) != 2:
        raise ValueError("Expected exactly 2 pole collisions in net_assembly")
    tags = yaml.safe_load(Path(landmarks_path).read_text())
    l = tags["tag_global_localizer"]["ros__parameters"]
    r = float(l["tag_mount_radius"])
    angle = math.radians(float(l["inward_angle_deg"]))
    headings = (-angle, angle, -math.pi+angle, math.pi-angle)
    # The 100-mm active tag has a quiet border: 8 / 6 full/active cell ratio.
    plate_edge = float(l["tag_edge_size"]) * (
        int(l["active_grid_cells"]) + 2*int(l["quiet_border_cells"])) / int(l["active_grid_cells"])
    mounts = []
    for tag_id, heading in enumerate(headings):
        y_pole = float(l["left_pole_y"] if tag_id in (0, 2) else l["right_pole_y"])
        mounts.append({
            "id": tag_id, "x": float(l["pole_x"]) + r*math.cos(heading),
            "y": y_pole + r*math.sin(heading), "heading": heading,
            # The tag has no Gazebo physics collision in the existing generator;
            # treat its physical mounting plate as a conservative 2-D obstacle.
            "radius": plate_edge/2 + .015,
        })
    return {"floor_width_m": width, "floor_height_m": height,
            "poles": poles, "tag_mounts": mounts}


def occupancy(width, height, resolution, circles):
    nx = math.ceil(width/resolution)
    ny = math.ceil(height/resolution)
    origin_x = -width/2
    origin_y = -height/2
    pixels = bytearray([254]) * (nx*ny)
    for c in circles:
        x, y, radius = c["x"], c["y"], c["radius"]
        ix_min = max(0, int(math.floor((x-radius-origin_x)/resolution)))
        ix_max = min(nx-1, int(math.floor((x+radius-origin_x)/resolution)))
        iy_min = max(0, int(math.floor((y-radius-origin_y)/resolution)))
        iy_max = min(ny-1, int(math.floor((y+radius-origin_y)/resolution)))
        for iy in range(iy_min, iy_max+1):
            cy = origin_y+(iy+.5)*resolution
            for ix in range(ix_min, ix_max+1):
                cx = origin_x+(ix+.5)*resolution
                if math.hypot(cx-x,cy-y) <= radius + math.sqrt(2)*resolution/2:
                    pixels[(ny-1-iy)*nx+ix] = 0  # PGM rows top to bottom
    return nx, ny, pixels


def write_court_map(world_path, landmarks_path, output_dir, resolution=0.05):
    if not 0.02 <= resolution <= 0.20:
        raise ValueError("Map resolution must be within [0.02,0.20] m")
    geometry = court_geometry(world_path, landmarks_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    width, height, pixels = occupancy(
        geometry["floor_width_m"], geometry["floor_height_m"], resolution,
        geometry["poles"] + geometry["tag_mounts"])
    image = output_dir/"court_static.pgm"
    image.write_bytes(f"P5\n{width} {height}\n255\n".encode() + pixels)
    desc = {
        "image": image.name, "mode": "trinary", "resolution": resolution,
        "origin": [-geometry["floor_width_m"]/2, -geometry["floor_height_m"]/2, 0.0],
        "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.25,
    }
    (output_dir/"court_static.yaml").write_text(yaml.safe_dump(desc,sort_keys=False))
    (output_dir/"geometry.json").write_text(json.dumps(geometry,indent=2))
    return output_dir/"court_static.yaml", geometry


if __name__ == "__main__":
    import argparse
    p=argparse.ArgumentParser()
    p.add_argument("--world",required=True,type=Path)
    p.add_argument("--landmarks",required=True,type=Path)
    p.add_argument("--output",required=True,type=Path)
    p.add_argument("--resolution",default=.05,type=float)
    a=p.parse_args()
    location,g=write_court_map(a.world,a.landmarks,a.output,a.resolution)
    print(location)
    print(f"Loaded {len(g['poles'])} poles, {len(g['tag_mounts'])} tag mounts")
