"""Static developer-lab UI conventions (no Gazebo/ROS installation required).

Run from the repository root:
    python3 -m unittest tools.test_lab_ui_consistency -v
"""
from __future__ import annotations

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LABS = ["lighting_lab", "camera_lab", "mobility_lab", "odometry_lab",
        "yolo_lab", "mesh_lab", "nav2_lab", "performance_lab"]
HTML = {
    "lighting_lab": ROOT / "tools/lighting_lab/index.html",
    "camera_lab": ROOT / "tools/camera_lab/index.html",
    "mobility_lab": ROOT / "tools/mobility_lab/index.html",
    "odometry_lab": ROOT / "tools/odometry_lab/index.html",
    "yolo_lab": ROOT / "tools/yolo_lab/index.html",
    "mesh_lab": ROOT / "tools/mesh_lab/index.html",
    "nav2_lab": ROOT / "tools/nav2_lab/planning/web/index.html",
    "performance_lab": ROOT / "tools/performance_lab/web/index.html",
}
SERVERS = {
    "nav2_lab": ROOT / "tools/nav2_lab/planning/web_server.py",
    **{name: ROOT / f"tools/{name}/server.py" for name in LABS if name != "nav2_lab"},
}


class LabInterfaceTests(unittest.TestCase):
    def test_server_entrypoints(self):
        for lab in LABS:
            with self.subTest(lab=lab):
                self.assertTrue((ROOT / f"tools/{lab}/server.py").is_file())
                source = SERVERS[lab].read_text(encoding="utf-8")
                self.assertIn("open_new_tab", source)
                self.assertIn("--no-browser", source)

    def test_runtime_log_panels_at_end(self):
        for lab, path in HTML.items():
            with self.subTest(lab=lab):
                html = path.read_text(encoding="utf-8")
                self.assertRegex(html, r"(?i)runtime logs|process diagnostics")
                self.assertIn("<details", html)
                ids = re.findall(r'\bid="([^"]+)"', html)
                self.assertEqual(len(ids), len(set(ids)), f"Duplicate HTML IDs in {lab}")

    def test_headless_baseline(self):
        launch = ROOT / "src/scrobot_simulation/launch"
        gazebo = (launch / "gazebo.launch.py").read_text()
        simulation = (launch / "simulation.launch.py").read_text()
        self.assertIn("IfElseSubstitution(headless", gazebo)
        for source in (gazebo, simulation):
            self.assertIn("default_value='false'", source)
            self.assertIn("'headless'", source)
        for name in ("mobility_lab.launch.py", "odometry_lab.launch.py"):
            self.assertIn("headless", (launch / name).read_text())
        # Planner Lab invokes gz -s directly rather than passing a
        # headless argument into gazebo.launch.py.
        nav2 = (ROOT / "tools/nav2_lab/planning/launch/planning.launch.py").read_text()
        self.assertIn('"gz_args": ["-r -s -v 3 "', nav2)
        performance = (ROOT / "tools/performance_lab/server.py").read_text()
        self.assertIn('"use_sim_time:=true","headless:=true"', performance)

    def test_optional_viewer_routes(self):
        for lab in ("lighting_lab", "mobility_lab", "odometry_lab", "mesh_lab"):
            with self.subTest(lab=lab):
                server = SERVERS[lab].read_text()
                self.assertIn("/api/gazebo-gui/open", server)
                self.assertIn("/api/gazebo-gui/close", server)
                self.assertIn("gazebo_gui", server)
        nav = SERVERS["nav2_lab"].read_text()
        for route in ("/api/gazebo-gui/open", "/api/gazebo-gui/close",
                      "/api/rviz/open", "/api/rviz/close"):
            self.assertIn(route, nav)
        perf = SERVERS["performance_lab"].read_text()
        for route in ("/api/gazebo/open", "/api/gazebo/close",
                      "/api/rviz/open", "/api/rviz/close"):
            self.assertIn(route, perf)

    def test_publish_adjacent_to_yaml_save(self):
        for lab in ("mobility_lab", "odometry_lab"):
            html = HTML[lab].read_text()
            self.assertRegex(html, r'id="apply">Save .* Lab YAML</button><button id="publish"')


if __name__ == "__main__":
    unittest.main()
