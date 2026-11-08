#!/usr/bin/env python3
"""Publish saved Lighting Lab calibration into the normal Gazebo world SDF.

Unlike build_world() this keeps the production scene: shuttle collection,
plugins, court landmarks, robot spawn, physics, and original camera stack.
Only world-level lights, four material groups and venue floor/walls change.
The net mesh and tape remain visible but never cast shadows.
"""
from __future__ import annotations

import argparse
import copy
import os
from pathlib import Path
import re
import stat
import tempfile
import xml.etree.ElementTree as ET

from config import read_lab_config, validated_calibration
from scene import (
    WORLD, VISUAL_GROUPS, centered_positions, configure_model,
    configure_venue, make_light,
)

REPO = Path(__file__).resolve().parents[2]
PRODUCTION_WORLD = REPO / "src/scrobot_simulation/worlds/badminton_court.sdf"
LAB_CONFIG = REPO / "src/scrobot_simulation/config/lighting_lab.yaml"

# Only world-level light tags (four-space indentation) may be replaced.
LIGHT_BLOCK = re.compile(r"(?ms)^    <light\\b[^>]*>.*?^    </light>\\n")
NET_NO_SHADOW = ("net_visual", "net_top_tape")


def _named_block(text, tag, name):
    pattern = re.compile(
        r'(?ms)^(?P<indent>[ ]*)<' + re.escape(tag) +
        r' name="' + re.escape(name) + r'">.*?^[ ]*</' + tag + r'>'
    )
    matches = list(pattern.finditer(text))
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one {tag} named {name}, found {len(matches)}")
    return matches[0]


def _replace_tag_inside(text, block_tag, name, child_tag, value):
    matched = _named_block(text, block_tag, name)
    section = matched.group()
    child = re.compile(
        r'<' + re.escape(child_tag) + r'>(.*?)</' + child_tag + r'>',
        re.DOTALL,
    )
    new_section, count = child.subn(
        lambda _: f"<{child_tag}>{value}</{child_tag}>", section, count=1
    )
    if count != 1:
        raise ValueError(f"Missing {child_tag} inside {block_tag} {name}")
    return text[:matched.start()] + new_section + text[matched.end():]


def _format_xml(element, indent_spaces=10):
    element = copy.deepcopy(element)
    ET.indent(element, space="  ")
    payload = ET.tostring(element, encoding="unicode")
    return "\n".join((" " * indent_spaces) + line for line in payload.splitlines())


def _replace_material(text, visual_name, material):
    matched = _named_block(text, "visual", visual_name)
    section = matched.group()
    pattern = re.compile(r"(?m)^([ ]*)<material>.*?</material>", re.DOTALL)
    matches = list(pattern.finditer(section))
    if len(matches) != 1:
        raise ValueError(f"Expected one material on visual {visual_name}")
    original = matches[0]
    indentation = len(original.group(1))
    updated = _format_xml(material, indentation)
    section = section[:original.start()] + updated + section[original.end():]
    return text[:matched.start()] + section + text[matched.end():]


def _ensure_no_shadow(text, visual_name):
    matched = _named_block(text, "visual", visual_name)
    section = matched.group()
    tag = re.compile(r"<cast_shadows>[^<]*</cast_shadows>")
    if tag.search(section):
        section = tag.sub("<cast_shadows>false</cast_shadows>", section)
    else:
        closing = section.rfind("</visual>")
        section = (
            section[:closing] + "  <cast_shadows>false</cast_shadows>\n"
            + section[closing:]
        )
    return text[:matched.start()] + section + text[matched.end():]


def _replace_world_lights(text, lights, count):
    matches = list(LIGHT_BLOCK.finditer(text))
    if len(matches) != count or not matches:
        raise ValueError(
            f"Expected {count} world-level light blocks, found {len(matches)}"
        )
    between = text[matches[0].start():matches[-1].end()]
    for match in reversed(matches):
        start = match.start() - matches[0].start()
        end = match.end() - matches[0].start()
        between = between[:start] + between[end:]
    if between.strip():
        raise ValueError("World light blocks are not contiguous")
    rendered = []
    for name, x, y, z in centered_positions(lights):
        xml = make_light(name.replace("lab_light_", "court_light_"),
                         x, y, z, lights)
        rendered.append(_format_xml(xml, 4))
    return text[:matches[0].start()] + "\n\n".join(rendered) + "\n" + text[matches[-1].end():]


def render_production_world(source_text, calibration):
    """Return SDF with ONLY selected scene lighting/material/venue changes."""
    clean = validated_calibration(calibration)
    root = ET.fromstring(source_text)
    world = root.find("world")
    if world is None or world.get("name") != WORLD:
        raise ValueError("Expected SCROBOT production badminton court")
    manager = world.find("./plugin[@filename='libshuttle_manager_system.so']")
    if manager is None or manager.findtext("enable_collection") != "true":
        raise ValueError("Refusing to publish a non-production world")
    if world.find("./model[@name='lab_free_camera']") is not None:
        raise ValueError("Refusing to publish a Lighting Lab derivative")
    existing_lights = world.findall("light")
    if not existing_lights:
        raise ValueError("Production world has no lights to update")

    # Calculate target floor and wall geometry using the exact same pure
    # scene functions as the Lighting Lab (no second set of floor formulas).
    configure_model(world, clean["colors"])
    configure_venue(world, clean["venue"])

    result = _replace_world_lights(source_text, clean["lights"], len(existing_lights))
    for group, members in VISUAL_GROUPS.items():
        for model_name, visual_name in members:
            target = next(
                model for model in world.findall("model")
                if model.get("name") == model_name
            )
            target_visual = next(
                visual for visual in target.findall(".//visual")
                if visual.get("name") == visual_name
            )
            result = _replace_material(result, visual_name, target_visual.find("material"))

    for name in ("floor_collision", "venue_floor_visual"):
        target = next(item for item in world.iter() if item.get("name") == name)
        tag = "collision" if name == "floor_collision" else "visual"
        result = _replace_tag_inside(
            result, tag, name, "size", target.findtext("./geometry/box/size")
        )
    boundary = next(
        model for model in world.findall("model")
        if model.get("name") == "simulation_safety_boundary"
    )
    for axis in ("x", "y"):
        for sign in ("positive", "negative"):
            for suffix in ("collision", "visual"):
                name = f"wall_{axis}_{sign}_{suffix}"
                target = next(item for item in boundary.iter() if item.get("name") == name)
                result = _replace_tag_inside(result, suffix, name, "pose", target.findtext("pose"))
                result = _replace_tag_inside(
                    result, suffix, name, "size", target.findtext("./geometry/box/size")
                )
    for name in NET_NO_SHADOW:
        result = _ensure_no_shadow(result, name)

    # Ensure the resulting scene is valid XML and retains mission behavior.
    published = ET.fromstring(result).find("world")
    expected = ET.fromstring(source_text).find("world")
    for tag in ("physics", "spherical_coordinates"):
        if ET.tostring(published.find(tag)) != ET.tostring(expected.find(tag)):
            raise ValueError(f"Publishing changed production {tag}")
    if [ET.tostring(p) for p in published.findall("plugin")] != [
        ET.tostring(p) for p in expected.findall("plugin")
    ]:
        raise ValueError("Publishing changed production plugins")
    for name in ("court_markings", "net_assembly"):
        current = published.find(f"./model[@name='{name}']")
        original = expected.find(f"./model[@name='{name}']")
        if name == "court_markings" and ET.tostring(current) != ET.tostring(original):
            raise ValueError("Publishing changed regulation court markings")
        if name == "net_assembly":
            for collision in original.findall(".//collision"):
                other = next(c for c in current.findall(".//collision")
                             if c.get("name") == collision.get("name"))
                if ET.tostring(collision) != ET.tostring(other):
                    raise ValueError("Publishing changed net/post collisions")
    return result


def write_production_world(world_path, calibration):
    """Atomically publish and return True if the file content changed."""
    before = world_path.read_text(encoding="utf-8")
    after = render_production_world(before, calibration)
    if after == before:
        return False
    tmp_name = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=world_path.parent,
            prefix=".scrobot_world_", suffix=".sdf.tmp", delete=False,
        ) as temp:
            tmp_name = temp.name
            temp.write(after)
            temp.flush()
            os.fsync(temp.fileno())
        os.chmod(tmp_name, stat.S_IMODE(world_path.stat().st_mode))
        os.replace(tmp_name, world_path)
        tmp_name = None
    finally:
        if tmp_name is not None:
            Path(tmp_name).unlink(missing_ok=True)
    return True


def publish_saved_calibration(config_path=LAB_CONFIG, world_path=PRODUCTION_WORLD):
    """Only the SAVED YAML is published, not unsaved dashboard slider values."""
    calibration = read_lab_config(config_path)
    if not calibration:
        raise ValueError(f"Missing Lighting Lab YAML: {config_path}")
    changed = write_production_world(world_path, calibration)
    return changed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=LAB_CONFIG)
    parser.add_argument("--world", type=Path, default=PRODUCTION_WORLD)
    parser.add_argument("--check", action="store_true",
                        help="Validate and preview; do not write production SDF")
    args = parser.parse_args()
    if args.check:
        config = read_lab_config(args.config)
        current = args.world.read_text(encoding="utf-8")
        target = render_production_world(current, config)
        print(f"Production world valid; changes needed: {target != current}")
        return
    changed = publish_saved_calibration(args.config, args.world)
    print(f"Published Lighting Lab YAML to {args.world} (changed={changed})")


if __name__ == "__main__":
    main()
