"""Read-only robot URDF/Xacro and shuttle SDF geometry audit.

URDF is expanded with the existing xacro executable when available. If the
workspace was not sourced, report the error rather than pretending that a raw
Xacro tree is the active robot model. Poses are local link/joint origins.
"""
from __future__ import annotations

import os
import struct
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

from tools.mesh_lab.settings import ROOT

ROBOT_XACRO = ROOT / "src/scrobot_simulation/urdf/scrobot_sim.urdf.xacro"
SHUTTLE_SDF = ROOT / "src/scrobot_simulation/models/shuttle/model.sdf"


def mesh_path(uri: str):
    prefixes = (
        ("package://scrobot_description/", ROOT / "src/scrobot_description"),
        ("package://scrobot_simulation/", ROOT / "src/scrobot_simulation"),
        ("model://shuttle/", ROOT / "src/scrobot_simulation/models/shuttle"),
    )
    for prefix, directory in prefixes:
        if uri.startswith(prefix):
            relative = uri[len(prefix):]
            path = (directory / relative).resolve()
            if path.is_relative_to(directory.resolve()):
                return path
    return None


def triangle_count(path):
    """Read binary STL face count cheaply, or stream ASCII facets."""
    if not path or not path.is_file():
        return None
    if path.suffix.lower() != ".stl":
        return None
    size = path.stat().st_size
    with path.open("rb") as stream:
        head = stream.read(84)
        if len(head) == 84:
            count = struct.unpack_from("<I", head, 80)[0]
            if 84 + 50 * count == size:
                return count
    count = 0
    with path.open("rb") as stream:
        for line in stream:
            if line.lstrip().lower().startswith(b"facet normal"):
                count += 1
    return count or None


def describe_geometry(element, sdf=False):
    geometry = element.find("geometry")
    if geometry is None:
        return {"kind": "none"}
    mesh = geometry.find("mesh")
    if mesh is not None:
        uri = mesh.findtext("uri") if sdf else mesh.attrib.get("filename")
        uri = uri or ""
        file = mesh_path(uri)
        return {
            "kind": "mesh", "uri": uri,
            "scale": mesh.findtext("scale") if sdf else mesh.attrib.get("scale", "1 1 1"),
            "exists": bool(file and file.is_file()),
            "triangles": triangle_count(file),
            "bytes": file.stat().st_size if file and file.is_file() else None,
        }
    for kind in ("box", "cylinder", "sphere", "capsule", "ellipsoid"):
        geom = geometry.find(kind)
        if geom is not None:
            if sdf:
                dimensions = {x.tag: (x.text or "").strip() for x in geom}
            else:
                dimensions = dict(geom.attrib)
            return {"kind": kind, "dimensions": dimensions}
    return {"kind": "other"}


def describe_link(link, sdf=False):
    parts = {"visuals": [], "collisions": []}
    for tag, key in (("visual", "visuals"), ("collision", "collisions")):
        for element in link.findall(tag):
            pose = (element.findtext("pose") or "0 0 0 0 0 0") if sdf else None
            origin = element.find("origin") if not sdf else None
            if origin is not None:
                pose = f'{origin.get("xyz", "0 0 0")} | {origin.get("rpy", "0 0 0")}'
            item = {
                "name": element.get("name") or tag,
                "pose": pose or "0 0 0 | 0 0 0",
                **describe_geometry(element, sdf=sdf),
            }
            parts[key].append(item)
    return {"name": link.get("name"), **parts}


def robot_audit(visual_mode="full"):
    if visual_mode not in ("full", "simple", "none"):
        raise ValueError("Invalid robot visual mode")
    try:
        process = subprocess.run(
            ["xacro", str(ROBOT_XACRO), f"robot_visual_mode:={visual_mode}",
             "enable_depth_camera:=false", "enable_magnetometer:=false"],
            capture_output=True, text=True, check=False, timeout=20,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"links": [], "joints": [], "error": f"Cannot expand Xacro: {exc}"}
    if process.returncode != 0:
        return {"links": [], "joints": [],
                "error": "Xacro failed (source ROS 2 and workspace): " + process.stderr[-600:]}
    try:
        root = ET.fromstring(process.stdout)
    except ET.ParseError as exc:
        return {"links": [], "joints": [], "error": f"Invalid expanded URDF: {exc}"}
    joints = []
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        joints.append({
            "name": joint.get("name"), "type": joint.get("type"),
            "parent": joint.find("parent").get("link") if joint.find("parent") is not None else None,
            "child": joint.find("child").get("link") if joint.find("child") is not None else None,
            "xyz": origin.get("xyz", "0 0 0") if origin is not None else "0 0 0",
            "rpy": origin.get("rpy", "0 0 0") if origin is not None else "0 0 0",
        })
    return {
        "links": [describe_link(link) for link in root.findall("link")],
        "joints": joints, "error": None,
    }


def shuttle_audit():
    root = ET.parse(SHUTTLE_SDF).getroot()
    model = root.find("model")
    return {
        "model": model.get("name"),
        "links": [describe_link(link, sdf=True) for link in model.findall("link")],
        "joints": [],
    }


def audit(visual_mode="full"):
    return {
        "reference": "All poses are local to the link/joint; not world poses.",
        "robot": robot_audit(visual_mode),
        "shuttle": shuttle_audit(),
    }
