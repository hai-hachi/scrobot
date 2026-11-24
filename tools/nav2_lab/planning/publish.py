"""Explicit, scoped Nav2 planner publish from lab to production YAML.

Only the top-level planner_server section is rewritten. All other production
YAML bytes and comments remain unchanged. Keep GridBased as the default BT
planner ID for existing NavigateToPose trees, regardless of algorithm.
"""
from __future__ import annotations

from copy import deepcopy
from hashlib import sha256
import os
from pathlib import Path
import re
import stat
import tempfile
from datetime import datetime, timezone

import yaml

from planning.core import PLANNERS, LATTICE_RESOLUTION_M, planner_profiles


_SECTION = re.compile(r"(?m)^planner_server:[ \t]*\n")
_TOP_LEVEL = re.compile(r"(?m)^[A-Za-z_][A-Za-z_0-9-]*:[ \t]*(?:#.*)?$")


def _section_span(text):
    matches=list(_SECTION.finditer(text))
    if len(matches)!=1:
        raise ValueError("Expected exactly one production planner_server YAML section")
    start=matches[0].start()
    following=_TOP_LEVEL.search(text,matches[0].end())
    end=following.start() if following else len(text)
    return start,end


def _read(path):
    target=Path(path)
    text=target.read_text(encoding="utf-8")
    document=yaml.safe_load(text)
    if not isinstance(document,dict):
        raise ValueError("Production Nav2 YAML must be a mapping")
    raw=document.get("planner_server")
    if not isinstance(raw,dict) or not isinstance(raw.get("ros__parameters"),dict):
        raise ValueError("Production planner_server.ros__parameters not found")
    _section_span(text)
    return text,document


def _selection(enabled,default):
    if not isinstance(enabled,list) or not enabled:
        raise ValueError("Select at least one planner")
    if any(not isinstance(name,str) or name not in PLANNERS for name in enabled):
        raise ValueError("Unknown planner selection")
    if len(enabled)!=len(set(enabled)):
        raise ValueError("Repeated planner selection")
    if not isinstance(default,str) or default not in enabled:
        raise ValueError("Default planner must be one of the selected planners")
    return [name for name in PLANNERS if name in enabled],default


def _infer_alias(params):
    alias=params.get("GridBased")
    if not isinstance(alias,dict):
        return None
    plugin=alias.get("plugin")
    if plugin==PLANNERS["SmacLattice"]:
        return "SmacLattice"
    if plugin==PLANNERS["ThetaStar"]:
        return "ThetaStar"
    if plugin==PLANNERS["NavFnAStar"]:
        return "NavFnAStar" if alias.get("use_astar") is True else None
    return None


def production_status(path):
    text,document=_read(path)
    params=document["planner_server"]["ros__parameters"]
    enabled=params.get("planner_plugins") or []
    if not isinstance(enabled,list):
        raise ValueError("Invalid production planner_plugins type")
    known=[name for name in PLANNERS if name in enabled]
    default=_infer_alias(params)
    if default and default not in known:
        known.insert(0,default)
    return {
        "path":str(Path(path)),
        "sha256":sha256(text.encode("utf-8")).hexdigest(),
        "selected":known,
        "default":default,
        "plugin_ids":list(enabled),
        "other_plugin_ids":[name for name in enabled
                            if name not in PLANNERS and name!="GridBased"],
        "restart_required":False,
        "note":"Production NavigateToPose defaults to GridBased; its algorithm is the selected default. Other selected planner IDs remain addressable.",
    }


def render_publish(original,enabled,default):
    selected,default=_selection(enabled,default)
    text,document=(
        (original,yaml.safe_load(original)) if isinstance(original,str)
        else _read(original)
    )
    if not isinstance(document,dict) or not isinstance(document.get("planner_server"),dict):
        raise ValueError("Production planner_server missing")
    old_params=document["planner_server"].get("ros__parameters")
    if not isinstance(old_params,dict):
        raise ValueError("Production planner ROS parameters missing")
    if "SmacLattice" in selected:
        resolution = document.get("global_costmap", {}).get("global_costmap", {}).get("ros__parameters", {}).get("resolution")
        if resolution != LATTICE_RESOLUTION_M:
            raise ValueError("Production global costmap must use 0.05 m resolution to publish Smac State Lattice")
    start,end=_section_span(text)
    params=deepcopy(old_params)
    old_ids=params.get("planner_plugins",[])
    if not isinstance(old_ids,list):
        raise ValueError("Existing planner_plugins must be a list")
    for name in set(old_ids)|set(PLANNERS)|{"GridBased", "NavFnDijkstra", "Smac2D"}:
        if isinstance(name,str):
            params.pop(name,None)
    profiles=planner_profiles()
    params["planner_plugins"]=["GridBased",*selected]
    # Backwards-compatible BT default. All enabled planner IDs are also
    # registered for explicit ComputePathToPose planner_id requests.
    params["GridBased"]=deepcopy(profiles[default])
    for name in selected:
        params[name]=deepcopy(profiles[name])
    rewritten=yaml.safe_dump(
        {"planner_server":{"ros__parameters":params}},
        sort_keys=False,allow_unicode=True)
    candidate=text[:start]+rewritten+"\n"+text[end:]
    check=yaml.safe_load(candidate)
    if not isinstance(check,dict):
        raise ValueError("Generated production Nav2 YAML is invalid")
    before=deepcopy(document)
    after=deepcopy(check)
    before.pop("planner_server",None)
    after.pop("planner_server",None)
    if before!=after:
        raise RuntimeError("Publishing would change non-planner production settings")
    # Whitespace and comments outside the planner section stay byte-identical.
    if not candidate.startswith(text[:start]) or not candidate.endswith(text[end:]):
        raise RuntimeError("Unexpected change outside planner_server section")
    return candidate


def publish(path,enabled,default,expected_sha256,backup_root):
    _selection(enabled,default)
    text,_=_read(path)
    actual=sha256(text.encode("utf-8")).hexdigest()
    if not isinstance(expected_sha256,str) or expected_sha256!=actual:
        raise ValueError("Production Nav2 config changed since preview; reload before publishing")
    candidate=render_publish(text,enabled,default)
    if candidate==text:
        result=production_status(path)
        result.update({"changed":False,"restart_required":False,"backup_path":None})
        return result
    target=Path(path)
    backup_dir=Path(backup_root)
    backup_dir.mkdir(parents=True,exist_ok=True)
    timestamp=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    backup=backup_dir/("nav2_params_before_planners_"+timestamp+"_"+actual[:8]+".yaml")
    # Never overwrite an existing backup; it is the user's rollback source.
    with backup.open("x",encoding="utf-8") as dest:
        dest.write(text)
    tmp_path=None
    try:
        with tempfile.NamedTemporaryFile(
            dir=target.parent,prefix=".nav2_planner_publish_",
            suffix=".tmp",mode="w",encoding="utf-8",delete=False
        ) as out:
            tmp_path=Path(out.name)
            out.write(candidate)
            out.flush()
            os.fsync(out.fileno())
        os.chmod(tmp_path,stat.S_IMODE(target.stat().st_mode))
        # Protect against a concurrent external editor between preview and write.
        if sha256(target.read_bytes()).hexdigest()!=actual:
            raise ValueError("Production config changed during publish; retry")
        os.replace(tmp_path,target)
        tmp_path=None
    finally:
        if tmp_path is not None:
            tmp_path.unlink(missing_ok=True)
    result=production_status(target)
    result.update({
        "changed":True,
        "restart_required":True,
        "backup_path":str(backup),
        "note":"Published only planner_server. Rebuild/source scrobot_navigation and restart the main Nav2 stack; live running nodes are unchanged.",
    })
    return result
