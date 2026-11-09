"""Gazebo GUI-only light position / coordinate-frame overlay.

MarkerManager renders in the GUI scene rather than adding SDF models, physics
entities, or visual geometry to the robot camera streams. No calibration SDF
or production world is changed by showing/hiding these frames.
"""
from __future__ import annotations

from scene import centered_positions

MARKER_NAMESPACE = "scrobot_lighting_lab_frames"
GEOMETRY_KEYS = frozenset(("rows", "cols", "reach_x", "reach_y", "height"))


def extent(cfg):
    """Actual world-coordinate light array bounds, including single-row case."""
    positions = list(centered_positions(cfg))
    return {
        "rows": cfg["rows"],
        "cols": cfg["cols"],
        "count": len(positions),
        "min_x": min(p[1] for p in positions),
        "max_x": max(p[1] for p in positions),
        "min_y": min(p[2] for p in positions),
        "max_y": max(p[2] for p in positions),
        "height": cfg["height"],
        "y_reach_active": cfg["rows"] > 1,
        "x_reach_active": cfg["cols"] > 1,
    }


def marker_instructions(cfg, visible=True):
    """Plain descriptions, independent of Gazebo bindings, for offline tests.

    One XYZ axis and one vertical floor-projection guide per light. The
    deletion in the same batch removes stale frames when the grid shrinks.
    """
    commands = [{"action": "delete_all", "namespace": MARKER_NAMESPACE}]
    if not visible:
        return commands
    for index, (name, x, y, z) in enumerate(centered_positions(cfg)):
        commands.append({
            "action": "add", "kind": "axis", "namespace": MARKER_NAMESPACE,
            "id": 2 * index + 1, "name": name,
            "xyz": (x, y, z),
        })
        commands.append({
            "action": "add", "kind": "stem", "namespace": MARKER_NAMESPACE,
            "id": 2 * index + 2, "name": name + "_floor_projection",
            "start": (x, y, z), "end": (x, y, 0.08),
        })
    return commands


def create_marker_batch(cfg, visible, marker_v_type, marker_type):
    """Fill gz.msgs.Marker_V using only documented Gazebo Harmonic fields."""
    batch = marker_v_type()
    for instruction in marker_instructions(cfg, visible):
        marker = batch.marker.add()
        marker.ns = instruction["namespace"]
        marker.action = (marker_type.DELETE_ALL if instruction["action"] == "delete_all"
                         else marker_type.ADD_MODIFY)
        if instruction["action"] == "delete_all":
            continue
        marker.id = instruction["id"]
        marker.lifetime.sec = 0
        marker.lifetime.nsec = 0
        marker.material.lighting = False
        if instruction["kind"] == "axis":
            marker.type = marker_type.AXIS
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = instruction["xyz"]
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.42
        else:
            marker.type = marker_type.LINE_LIST
            marker.scale.x = marker.scale.y = marker.scale.z = 0.025
            for xyz in (instruction["start"], instruction["end"]):
                pt = marker.point.add()
                pt.x, pt.y, pt.z = xyz
            marker.material.diffuse.r = 1.0
            marker.material.diffuse.g = 0.85
            marker.material.diffuse.b = 0.12
            marker.material.diffuse.a = 1.0
            marker.material.emissive.r = 1.0
            marker.material.emissive.g = 0.75
            marker.material.emissive.b = 0.05
            marker.material.emissive.a = 1.0
    return batch
