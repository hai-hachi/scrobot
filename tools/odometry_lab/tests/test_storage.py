"""Storage isolation contracts: never overwrite another lab or an old run."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import storage


def test_atomic_run_reservation_preserves_existing_measurements(tmp_path):
    config = {"run_root": str(tmp_path / "odometry")}
    first = storage.reserve_run_dir(config, "repeatable_trial")
    existing = first / "summary.json"
    existing.write_text('{"saved": true}')
    second = storage.reserve_run_dir(config, "repeatable_trial")
    assert second != first
    assert second.name == "repeatable_trial_01"
    assert existing.read_text() == '{"saved": true}'
    assert second.exists()


@pytest.mark.parametrize("name", ["../mobility_lab", "../../lighting_lab",
                                  "/tmp/escape", "..", "a/b", "", "."])
def test_bad_run_names_rejected(tmp_path, name):
    with pytest.raises(ValueError, match="Run name"):
        storage.reserve_run_dir({"run_root": str(tmp_path / "odom")}, name)


def test_reject_overlapping_other_labs_and_checkout(monkeypatch, tmp_path):
    lighting = tmp_path / "evaluation_results" / "lighting_lab"
    mobility = tmp_path / "evaluation_results" / "mobility_lab"
    odometry = tmp_path / "evaluation_results" / "odometry_lab"
    monkeypatch.setattr(storage, "OTHER_LAB_RESULTS", (lighting, mobility))
    for root in (lighting, lighting / "some_run", mobility,
                 mobility / "recordings", lighting.parent):
        with pytest.raises(ValueError, match="overlaps"):
            storage.validated_result_root(root)
    assert storage.validated_result_root(odometry) == odometry.resolve()
    with pytest.raises(ValueError, match="overlaps"):
        storage.validated_result_root(storage.REPO / "tools/odometry_lab")


def test_workspace_root_tools_layout_accepts_default_odometry_results(monkeypatch, tmp_path):
    """Regression: ~/scrobot_ws/tools is not always inside the git checkout.

    In that layout parents[2] of storage.py is the entire workspace. Results
    under ~/scrobot_ws/evaluation_results/odometry_lab must remain writable.
    """
    workspace = tmp_path / "scrobot_ws"
    results = workspace / "evaluation_results"
    odometry = results / "odometry_lab"
    lighting = results / "lighting_lab"
    mobility = results / "mobility_lab"
    monkeypatch.setattr(storage, "REPO", workspace)
    monkeypatch.setattr(storage, "WORKSPACE_RESULTS", results)
    monkeypatch.setattr(storage, "OTHER_LAB_RESULTS", (lighting, mobility))

    assert storage.validated_result_root(odometry) == odometry.resolve()
    assert storage.validated_result_root(odometry / "runs") == (odometry / "runs").resolve()

    new_run = storage.reserve_run_dir({"run_root": str(odometry)}, "web_first")
    snapshot = new_run / "summary.json"
    snapshot.write_text('{"finished": true}')
    next_run = storage.reserve_run_dir({"run_root": str(odometry)}, "web_first")
    assert next_run != new_run
    assert snapshot.read_text() == '{"finished": true}'

    for forbidden in (lighting, lighting / "presets", mobility,
                      mobility / "recordings", results, workspace / "tools"):
        with pytest.raises(ValueError, match="overlaps"):
            storage.validated_result_root(forbidden)


def test_symlink_to_other_lab_results_cannot_bypass_guard(monkeypatch, tmp_path):
    results = tmp_path / "evaluation_results"
    lighting = results / "lighting_lab"
    mobility = results / "mobility_lab"
    lighting.mkdir(parents=True)
    alias = tmp_path / "alias_to_lighting"
    alias.symlink_to(lighting, target_is_directory=True)
    monkeypatch.setattr(storage, "OTHER_LAB_RESULTS", (lighting, mobility))
    with pytest.raises(ValueError, match="overlaps"):
        storage.validated_result_root(alias / "presets")
