#!/usr/bin/env python3
"""Fast, ROS-independent contracts for SCROBOT's simulation runtime profile.

Run: python3 tools/test_sim_runtime_contract.py
This checks source wiring and defaults only. It does not benchmark Gazebo and
does not replace full mission/perception regressions on a ROS 2 Jazzy PC.
"""

from __future__ import annotations

import ast
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def load(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def launch_default(py_file: str, name: str) -> str:
    tree = ast.parse(load(py_file), filename=py_file)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name):
            continue
        if node.func.id != "DeclareLaunchArgument" or not node.args:
            continue
        if not isinstance(node.args[0], ast.Constant) or node.args[0].value != name:
            continue
        for keyword in node.keywords:
            if keyword.arg == "default_value" and isinstance(keyword.value, ast.Constant):
                return keyword.value.value
    raise AssertionError(f"{py_file}: missing string default for launch arg {name!r}")


class SimulationPerformanceContractTest(unittest.TestCase):
    def setUp(self) -> None:
        self.full = "src/scrobot_debug/launch/full_mission_check.launch.py"
        self.sim = "src/scrobot_simulation/launch/simulation.launch.py"
        self.gz = "src/scrobot_simulation/launch/gazebo.launch.py"

    def test_full_mission_starts_without_duplicate_gpu_viewers(self):
        self.assertEqual(launch_default(self.full, "gz_headless"), "true")
        self.assertEqual(launch_default(self.full, "launch_rviz"), "false")
        self.assertEqual(launch_default(self.full, "publish_debug_image"), "false")
        self.assertEqual(launch_default(self.full, "launch_telemetry"), "false")

    def test_camera_and_perception_fidelity_remain_default(self):
        self.assertEqual(launch_default(self.full, "yolo_max_detection_rate"), "15.0")
        self.assertEqual(
            launch_default("src/scrobot_perception/launch/perception.launch.py",
                           "max_detection_rate"), "15.0",
        )
        self.assertIn("'max_detection_rate': yolo_max_detection_rate", load(self.full))
        self.assertIn("'max_detection_rate': max_detection_rate",
                      load("src/scrobot_perception/launch/perception.launch.py"))

    def test_minimal_simulation_is_still_interactive(self):
        self.assertEqual(launch_default(self.sim, "gz_headless"), "false")
        self.assertEqual(launch_default(self.gz, "gz_headless"), "false")
        self.assertIn("'gz_headless': gz_headless", load(self.sim))
        self.assertIn("'gz_headless': gz_headless", load(self.full))

    def test_headless_launch_has_gui_escape_hatch(self):
        source = load(self.gz)
        self.assertIn("condition=IfCondition(gz_headless)", source)
        self.assertIn("condition=UnlessCondition(gz_headless)", source)
        self.assertIn("'-r -s -v '", source)
        self.assertIn("'-r -v '", source)

    def test_image_bridge_only_connects_when_subscribed(self):
        source = load(self.gz)
        self.assertIn("'lazy': True", source)
        self.assertIn("'qos': 'sensor_data'", source)

    def test_camera_optics_unchanged_and_visual_frustums_disabled(self):
        source = load("src/scrobot_simulation/urdf/sensors_gazebo.xacro")
        root = ET.fromstring(source)
        sensors = [
            sensor for sensor in root.iter("sensor")
            if sensor.attrib.get("type") in ("camera", "depth_camera")
        ]
        # Sensors occur inside xacro:macro but remain XML sensor elements.
        self.assertEqual(len(sensors), 2)
        for sensor in sensors:
            self.assertEqual(sensor.findtext("visualize"), "false")
        rgb = next(s for s in sensors if s.attrib["type"] == "camera")
        depth = next(s for s in sensors if s.attrib["type"] == "depth_camera")
        self.assertEqual(rgb.findtext("camera/horizontal_fov"), "1.117010720035761")
        self.assertEqual(depth.findtext("camera/horizontal_fov"), "1.5795850624055896")
        self.assertEqual(rgb.findtext("camera/image/width"), "1280")
        self.assertEqual(depth.findtext("camera/image/width"), "848")

    def test_yolo_only_decodes_depth_after_detection(self):
        source = load("src/scrobot_perception/scripts/yolo_shuttle_detector.py")
        self.assertLess(source.index("self.model.predict("),
                        source.index("self._depth_to_meters(depth_msg)"))
        self.assertIn("if len(boxes) > 0:", source)

    def test_clock_health_requires_time_progress(self):
        source = load("src/scrobot_diagnostics/scripts/health_monitor.py")
        self.assertIn("def _clock_cb(self, msg):", source)
        self.assertIn("stamp_ns != self.last_clock_stamp_ns", source)

    def test_python_launch_syntax(self):
        for path in (
            self.gz, self.sim, self.full,
            "src/scrobot_perception/launch/perception.launch.py",
            "src/scrobot_perception/scripts/yolo_shuttle_detector.py",
            "src/scrobot_diagnostics/scripts/health_monitor.py",
        ):
            with self.subTest(path=path):
                ast.parse(load(path), filename=path)


if __name__ == "__main__":
    unittest.main()
