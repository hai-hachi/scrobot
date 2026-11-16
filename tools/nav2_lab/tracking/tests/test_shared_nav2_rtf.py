"""Planner and Tracker share a saved Gazebo cap, with per-process applied state."""
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

HERE = Path(__file__).resolve().parents[1]
TOOLS = HERE.parents[1]
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(HERE.parent))

from shared import sim_speed
from planning.web_server import Handler


class LiveProcess:
    def poll(self):
        return None


def test_same_nav2_cap_and_independent_restart_warning(tmp_path, monkeypatch):
    monkeypatch.setattr(sim_speed, "FILE", tmp_path / "speed.yaml")
    sim_speed.save("nav2", .25)
    planner = SimpleNamespace(
        stack=LiveProcess(), loaded_config={"simulation": {"max_rtf": .25}})
    tracker = SimpleNamespace(stack=LiveProcess(), applied_rtf=.50)
    handle = Handler.__new__(Handler)
    monkeypatch.setattr(Handler, "service", planner)
    monkeypatch.setattr(Handler, "tracking", tracker)

    response = handle.shared_speed_status()
    assert response["max_rtf"] == .25
    assert response["applied"] == {"planner": .25, "tracker": .50}
    assert response["restart_required"] == {"planner": False, "tracker": True}

    saved = handle.save_shared_speed({"max_rtf": .50})
    assert saved["max_rtf"] == .50
    assert sim_speed.get("nav2") == sim_speed.get("tracking") == .50
    # Save doesn't restart Gazebo. Only Planner needs a restart now.
    assert saved["applied"] == {"planner": .25, "tracker": .50}
    assert saved["restart_required"] == {"planner": True, "tracker": False}

    tracker.stack = None
    stopped = handle.shared_speed_status()
    assert stopped["applied"]["tracker"] is None
    assert stopped["restart_required"]["tracker"] is False

    with pytest.raises(ValueError):
        handle.save_shared_speed({"max_rtf": .25, "other_key": True})
    with pytest.raises(ValueError):
        handle.save_shared_speed({"max_rtf": 1.5})


def test_both_nav2_tabs_use_exactly_the_same_speed_control():
    planner = (HERE.parent / "planning/web/index.html").read_text()
    tracker = (HERE / "web/index.html").read_text()
    shared = (HERE.parent / "shared/rtf_control.js").read_text()
    server = (HERE.parent / "planning/web_server.py").read_text()
    tracker_app = (HERE / "web/app.js").read_text()
    tracker_service = (HERE / "service.py").read_text()
    assert planner.count('src="/lab-rtf.js"') == 1
    assert tracker.count('src="/lab-rtf.js"') == 1
    assert planner.count('id="lab-rtf-slider"') == 0
    assert tracker.count('id="lab-rtf-slider"') == 0
    assert shared.count('id="lab-rtf-slider"') == 1
    assert 'container.insertBefore(section, container.firstChild)' in shared
    assert 'Save cap for both tabs' in shared
    assert "'/api/sim-speed'" in shared
    assert '"planner"' in server and '"tracker"' in server
    assert 'if path=="/lab-rtf.js":' in server
    assert "speed_get(\"nav2\")" in tracker_service
    assert "speed_save(\"nav2\",value)" in tracker_service
    assert "save-speed" not in tracker_app
    assert "max-rtf-text" not in tracker_app
    assert "max-rtf" not in tracker
