#!/usr/bin/env python3
"""Expose repository-maintained developer tools at ~/scrobot_ws/tools.

The Git repository is checked out at ~/scrobot_ws/src. Git cannot check out
files into its parent, so a persistent symlink avoids copying the tools and
allows the tools/lighting_lab/server.py command to run at workspace root.

Run once after checkout; the symlink persists across reboots and git pulls.
"""
from __future__ import annotations

import argparse
from pathlib import Path


def install_workspace_tools(repo_root: Path, workspace: Path) -> Path:
    source = (repo_root / "tools").resolve(strict=True)
    workspace = workspace.expanduser().resolve(strict=True)
    target = workspace / "tools"
    if workspace == repo_root.resolve():
        raise ValueError("Workspace must be the parent of the repository checkout")
    if target.is_symlink():
        if target.resolve() != source:
            raise ValueError(f"{target} already links to a different tools directory")
        return target
    if target.exists():
        if not target.is_dir():
            raise ValueError(f"{target} exists and is not a directory")
        # A developer may already have personal scripts in a real workspace
        # tools directory; preserve all of those and link missing repo tools.
        for entry in source.iterdir():
            if entry.name.startswith(".") or entry.name == "__pycache__":
                continue
            link = target / entry.name
            if link.is_symlink():
                if link.resolve() != entry.resolve():
                    raise ValueError(f"{link} links to a different file")
            elif link.exists():
                if link.resolve() != entry.resolve():
                    raise ValueError(
                        f"{link} already exists; not overwriting user files"
                    )
            else:
                link.symlink_to(entry, target_is_directory=entry.is_dir())
        return target
    target.symlink_to(source, target_is_directory=True)
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    repo_root = Path(__file__).resolve().parents[1]
    parser.add_argument(
        "--workspace", type=Path, default=repo_root.parent,
        help="Workspace root (default: parent of this repository checkout)"
    )
    args = parser.parse_args()
    target = install_workspace_tools(repo_root, args.workspace)
    print(f"Workspace tools ready: {target} -> {target.resolve()}")
    print(f"Run: cd {target.parent} && python3 tools/lighting_lab/server.py")


if __name__ == "__main__":
    main()
