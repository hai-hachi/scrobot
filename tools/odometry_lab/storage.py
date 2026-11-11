"""Protect results from collision with other SCROBOT labs.

Odometry Lab can use an alternate output disk, but never a parent or child
of Lighting/Mobility results, nor a path inside/above the source checkout.
Run names are single directory components and reservations are exclusive.
"""
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import re

REPO = Path(__file__).resolve().parents[2]
WORKSPACE_RESULTS = Path.home() / "scrobot_ws/evaluation_results"
OTHER_LAB_RESULTS = (
    WORKSPACE_RESULTS / "lighting_lab",
    WORKSPACE_RESULTS / "mobility_lab",
)
RUN_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,79}\Z")


def _overlaps(a: Path, b: Path) -> bool:
    return a == b or a in b.parents or b in a.parents


def validated_result_root(raw) -> Path:
    """Resolve user-supplied root; fail closed on cross-lab/source overlap."""
    if not isinstance(raw, (str, Path)) or not str(raw).strip():
        raise ValueError("Odometry run_root must be a nonempty path")
    root = Path(raw).expanduser().resolve()
    protected = (REPO, *(p.resolve() for p in OTHER_LAB_RESULTS))
    if any(_overlaps(root, target) for target in protected):
        raise ValueError(
            "Odometry run_root overlaps the source checkout or another "
            f"lab's results: {root}"
        )
    return root


def reserve_run_dir(config, name=None):
    """Allocate a unique output folder atomically; never reuse an old trial."""
    root = validated_result_root(config["run_root"])
    root.mkdir(parents=True, exist_ok=True)
    base = datetime.now().strftime("odom_%Y%m%d_%H%M%S") if name is None else name
    if not RUN_NAME.fullmatch(base) or base in (".", ".."):
        raise ValueError("Run name must be one safe directory component")
    for number in range(10000):
        path = root / (base if number == 0 else f"{base}_{number:02d}")
        try:
            path.mkdir()
            return path
        except FileExistsError:
            continue
    raise RuntimeError("Too many Odometry Lab runs with the same name")
