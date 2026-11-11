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
