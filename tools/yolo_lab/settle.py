"""Capture Gazebo shuttle poses after gravity settling, for static reloading."""
from __future__ import annotations

import re
import subprocess


def _section(source, label):
    match = re.search(r"\\b" + re.escape(label) + r"\\s*\\{", source)
    if not match:
        return None
    pos, level = match.end(), 1
    for i in range(pos, len(source)):
        if source[i] == "{":
            level += 1
        elif source[i] == "}":
            level -= 1
            if level == 0:
                return source[pos:i]
    return None


def _number(text, field, default):
    match = re.search(r"\\b" + field + r"\\s*:\\s*([-+0-9.eE]+)", text)
    return float(match.group(1)) if match else default


def parse_poses(text, names):
    """Read model-level pose entries, not nested link or visual poses."""
    poses = {}
    cursor = 0
    while True:
        found = re.search(r"\\bpose\\s*\\{", text[cursor:])
        if not found:
            break
        start = cursor + found.start()
        body = _section(text[start:], "pose")
        if body is None:
            break
        cursor = start + found.end() + len(body) + 1
        name_match = re.search(r'\\bname\\s*:\\s*"([^"]+)"', body)
        if not name_match or name_match.group(1) not in names:
            continue
        p = _section(body, "position")
        q = _section(body, "orientation")
        if p is None or q is None:
            continue
        pose = (
            _number(p, "x", 0), _number(p, "y", 0), _number(p, "z", 0),
            _number(q, "x", 0), _number(q, "y", 0), _number(q, "z", 0),
            _number(q, "w", 1),
        )
        poses[name_match.group(1)] = pose
    return poses


def snapshot(names, env):
    """Read one fresh pose/info packet, fail closed if any model is missing."""
    cmd = ["gz", "topic", "-e", "-t", "/world/yolo_lab/pose/info", "-n", "1"]
    try:
        p = subprocess.run(cmd, env=env, capture_output=True, text=True,
                           timeout=12, check=False)
        output = p.stdout
        if p.returncode:
            raise RuntimeError(p.stderr.strip() or "Gazebo pose topic failed")
    except subprocess.TimeoutExpired as exc:
        # Some gz transport versions do not implement -n; timeout is a real
        # failure, not a reason to claim that the shuttles were frozen.
        raise RuntimeError("Timed out reading settled Gazebo poses") from exc
    found = parse_poses(output, set(names))
    missing = set(names) - found.keys()
    if missing:
        raise RuntimeError("Missing settled Gazebo poses: " +
                           ", ".join(sorted(missing)[:8]))
    return found
