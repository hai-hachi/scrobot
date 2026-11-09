"""Gazebo GUI-only RGB light coordinate frames and floor guides.

Uses Gazebo Harmonic MarkerManager's supported CYLINDER primitive, not the
less-portable AXIS marker geometry. Markers are strictly GUI-visible and
never become physics entities or robot-camera visuals.
"""
from __future__ import annotations

import math

from scene import centered_positions

MARKER_NAMESPACE = "scrobot_lighting_lab_frames"
GEOMETRY_KEYS = frozenset(("rows", "cols", "reach_x", "reach_y", "height"))
AXIS_LENGTH = 0.45
FLOOR_Z = 0.08
AXIS_RGB = {
    "x": (1.0, 0.14, 0.10),
    "y": (0.14, 1.0, 0.20),
    "z": (0.20, 0.44, 1.0),
    "stem": (1.0, 0.84, 0.10),
}


def extent(cfg):
    """Actual coordinates of the light grid, including single-row cases."""
    positions = list(centered_positions(cfg))
    return {
        "rows": cfg["rows"], "cols": cfg["cols"], "count": len(positions),
        "min_x": min(p[1] for p in positions),
        "max_x": max(p[1] for p in positions),
        "min_y": min(p[2] for p in positions),
        "max_y": max(p[2] for p in positions),
        "height": cfg["height"],
        "y_reach_active": cfg["rows"] > 1,
        "x_reach_active": cfg["cols"] > 1,
    }


def marker_instructions(cfg, visible=True):
    """One RGB-axis triad and vertical floor-projection for every light."""
    commands = [{"action": "delete_all", "namespace": MARKER_NAMESPACE}]
    if not visible:
        return commands
    for index, (name, x, y, z) in enumerate(centered_positions(cfg)):
        for offset, axis in enumerate(("x", "y", "z"), 1):
            center = {
                "x": (x + AXIS_LENGTH / 2, y, z),
                "y": (x, y + AXIS_LENGTH / 2, z),
                "z": (x, y, z + AXIS_LENGTH / 2),
            }[axis]
            commands.append({
                "action": "add", "kind": axis, "namespace": MARKER_NAMESPACE,
                "id": 4 * index + offset, "name": name + "_axis_" + axis,
                "center": center, "length": AXIS_LENGTH,
            })
        commands.append({
            "action": "add", "kind": "stem", "namespace": MARKER_NAMESPACE,
            "id": 4 * index + 4, "name": name + "_floor_projection",
            "center": (x, y, (z + FLOOR_Z) / 2), "length": z - FLOOR_Z,
        })
    return commands


def create_marker_batch(cfg, visible, marker_v_type, marker_type):
    """Encode a single Marker_V request; no Gazebo runtime needed by tests."""
    batch = marker_v_type()
    quarter_turn = math.sqrt(0.5)
    for instruction in marker_instructions(cfg, visible):
        marker = batch.marker.add()
        marker.ns = instruction["namespace"]
        marker.action = (
            marker_type.DELETE_ALL if instruction["action"] == "delete_all"
            else marker_type.ADD_MODIFY
        )
        if instruction["action"] == "delete_all":
            continue
        marker.id = instruction["id"]
        marker.visibility = marker_type.GUI
        marker.lifetime.sec = 0
        marker.lifetime.nsec = 0
        marker.type = marker_type.CYLINDER
        cx, cy, cz = instruction["center"]
        marker.pose.position.x = cx
        marker.pose.position.y = cy
        marker.pose.position.z = cz
        marker.pose.orientation.w = 1.0
        if instruction["kind"] == "x":
            marker.pose.orientation.y = quarter_turn
            marker.pose.orientation.w = quarter_turn
        elif instruction["kind"] == "y":
            marker.pose.orientation.x = -quarter_turn
            marker.pose.orientation.w = quarter_turn
        marker.scale.x = marker.scale.y = (
            0.018 if instruction["kind"] == "stem" else 0.028
        )
        marker.scale.z = instruction["length"]
        marker.material.lighting = False
        red, green, blue = AXIS_RGB[instruction["kind"]]
        marker.material.diffuse.r = red
        marker.material.diffuse.g = green
        marker.material.diffuse.b = blue
        marker.material.diffuse.a = 1.0
        marker.material.emissive.r = red
        marker.material.emissive.g = green
        marker.material.emissive.b = blue
        marker.material.emissive.a = 1.0
    return batch
