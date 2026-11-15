"""Safe deletion of Planner Lab's timestamped experiment directories.

Never follow symlinks, traverse the result root, touch the active _planning_active
runtime directory, or delete another lab's files. File names are server-chosen.
"""
import re
import shutil
from pathlib import Path

RUN_NAME=re.compile(r"[0-9]{8}_[0-9]{6}_planning_[0-9a-f]{5}\Z")


def safe_run(root,name):
    if not isinstance(name,str) or not RUN_NAME.fullmatch(name):
        raise ValueError("Invalid Planner experiment name")
    base=Path(root).expanduser().resolve()
    child=base/name
    if child.is_symlink() or child.resolve().parent != base:
        raise ValueError("Planner experiment path is unsafe")
    if not child.is_dir() or not (child/"summary.json").is_file():
        raise FileNotFoundError("Planner experiment no longer exists")
    return child


def list_runs(root):
    base=Path(root).expanduser().resolve()
    if not base.exists():return []
    return sorted(
        [candidate for candidate in base.iterdir()
         if candidate.is_dir() and not candidate.is_symlink()
         and RUN_NAME.fullmatch(candidate.name)
         and (candidate/"summary.json").is_file()],
        key=lambda x:x.name,reverse=True
    )


def delete_run(root,name):
    target=safe_run(root,name)
    shutil.rmtree(target)
    return name


def delete_all(root):
    names=[]
    for item in list_runs(root):
        names.append(delete_run(root,item.name))
    return names
