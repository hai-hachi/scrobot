#!/usr/bin/env python3
"""Offline architecture/compatibility contracts.

No ROS/Gazebo/PyYAML dependency. Run with:
    python3 tools/test_package_architecture.py -v

These checks protect the packaging and entrypoints, NOT dynamic ROS behavior.
"""
from __future__ import annotations

import ast
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"


def source(path):
    return (ROOT / path).read_text(encoding="utf-8")


def launch_default(path, name):
    tree = ast.parse(source(path), filename=path)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
            continue
        if node.func.id != "DeclareLaunchArgument" or not node.args:
            continue
        if not isinstance(node.args[0], ast.Constant) or node.args[0].value != name:
            continue
        for kw in node.keywords:
            if kw.arg == "default_value" and isinstance(kw.value, ast.Constant):
                return kw.value.value
        return "<dynamic>"
    raise AssertionError(f"No launch argument {name!r} in {path}")


class ArchitectureContracts(unittest.TestCase):
    def test_target_packages_exist(self):
        names = {
            ET.parse(p).getroot().findtext("name")
            for p in SRC.glob("scrobot_*/package.xml")
        }
        required = {
            "scrobot_interfaces", "scrobot_description", "scrobot_control",
            "scrobot_hardware", "scrobot_simulation", "scrobot_localization",
            "scrobot_perception", "scrobot_navigation", "scrobot_collection",
            "scrobot_mission", "scrobot_bringup", "scrobot_evaluation",
            "scrobot_debug", "scrobot_scenarios",
        }
        self.assertEqual(names, required)

    def test_internal_package_dependencies_are_acyclic(self):
        packages = {}
        for path in SRC.glob("scrobot_*/package.xml"):
            root = ET.parse(path).getroot()
            name = root.findtext("name")
            packages[name] = {
                el.text for el in root
                if el.tag in {"depend", "exec_depend", "build_depend", "buildtool_depend"}
                and el.text and el.text.startswith("scrobot_")
            }
        for name, deps in packages.items():
            self.assertFalse(deps - packages.keys(), f"{name} has nonexistent deps")
        visiting, visited = set(), set()

        def visit(name):
            if name in visiting:
                self.fail(f"Cyclic package dependency at {name}")
            if name in visited:
                return
            visiting.add(name)
            for dep in packages[name]:
                visit(dep)
            visiting.remove(name)
            visited.add(name)

        for name in packages:
            visit(name)

    def test_normal_sim_excludes_test_runtime_processes(self):
        path = "src/scrobot_bringup/launch/sim.launch.py"
        text = source(path)
        for forbidden in (
            "scrobot_evaluation", "scrobot_debug", "rviz.launch.py",
            "telemetry.launch.py", "collection_session_eval.launch.py",
        ):
            self.assertNotIn(forbidden, text)
        for mandatory in (
            "scrobot_simulation", "scrobot_control", "scrobot_localization",
            "scrobot_perception", "scrobot_mission", "spawn_shuttles.launch.py",
        ):
            self.assertIn(mandatory, text)
        self.assertEqual(launch_default(path, "publish_debug_image"), "false")
        self.assertEqual(launch_default(path, "use_magnetometer"), "true")
        self.assertEqual(launch_default(path, "shuttle_count"), "50")
        self.assertEqual(launch_default(path, "shuttle_seed"), "20261003")
        self.assertEqual(launch_default(path, "mission_delay"), "15.0")

    def test_original_regression_defaults_preserved(self):
        path = "src/scrobot_debug/launch/full_mission_check.launch.py"
        required = {
            "publish_debug_image": "true",
            "launch_rviz": "true",
            "run_evaluation": "true",
            "use_magnetometer": "true",
            "shuttle_mode": "mixed",
            "shuttle_count": "50",
            "shuttle_seed": "20261003",
            "robot_x": "2.0", "robot_y": "3.05",
            "robot_z": "0.003", "robot_yaw": "3.14159",
            "stack_delay": "4.0", "shuttle_delay": "5.0",
            "evaluator_delay": "6.0", "rviz_delay": "6.0",
            "mission_delay": "15.0",
        }
        for arg, value in required.items():
            with self.subTest(arg=arg):
                self.assertEqual(launch_default(path, arg), value)
        text = source(path)
        self.assertIn("'scrobot_bringup', 'sim.launch.py'", text)
        self.assertIn("collection_session_eval.launch.py", text)
        self.assertIn("telemetry.launch.py", text)
        self.assertIn("rviz.launch.py", text)

    def test_collection_is_single_implementation_with_legacy_shims(self):
        cmake = source("src/scrobot_collection/CMakeLists.txt")
        self.assertIn("ament_python_install_package", cmake)
        mission = source("src/scrobot_mission/launch/sweep_mission.launch.py")
        self.assertIn("get_package_share_directory('scrobot_collection')", mission)
        self.assertIn("collection.launch.py", mission)
        config = source("src/scrobot_collection/config/local_collect.yaml")
        self.assertEqual(config, source("src/scrobot_mission/config/local_collect.yaml"))
        for n, class_name in (
            ("local_collect_controller", "LocalCollectController"),
            ("shuttle_collection_filter", "ShuttleCollectionFilter"),
        ):
            impl = source(f"src/scrobot_collection/scrobot_collection/{n}.py")
            legacy = source(f"src/scrobot_mission/scrobot_mission/{n}.py")
            self.assertIn(f"class {class_name}", impl)
            self.assertIn(f"from scrobot_collection.{n} import *", legacy)
            self.assertIn(f"RENAME {n}", cmake)
            self.assertIn(f"executable='{n}'", source(
                "src/scrobot_collection/launch/collection.launch.py"
            ))
        self.assertIn("scrobot_collection", source("src/scrobot_mission/package.xml"))

    def test_tag_approach_belongs_to_navigation(self):
        impl = source("src/scrobot_navigation/scrobot_navigation/tag_approach_controller.py")
        legacy = source("src/scrobot_localization/scripts/tag_approach_controller.py")
        self.assertIn("class TagApproachController", impl)
        self.assertIn("from scrobot_navigation.tag_approach_controller import *", legacy)
        self.assertIn("ament_python_install_package", source(
            "src/scrobot_navigation/CMakeLists.txt"
        ))
        launch = source("src/scrobot_localization/launch/global_localization.launch.py")
        self.assertIn("package='scrobot_navigation'", launch)
        self.assertIn("executable='tag_approach_controller.py'", launch)

    def test_scenarios_is_opt_in_and_legacy_compatible(self):
        launches = sorted((SRC / "scrobot_scenarios/launch").glob("*.launch.py"))
        self.assertGreaterEqual(len(launches), 20)
        self.assertIn(
            "scrobot_debug",
            source("src/scrobot_scenarios/launch/full_mission_check.launch.py"),
        )
        for path in launches:
            self.assertTrue((SRC / "scrobot_debug/launch" / path.name).exists())
            self.assertIn("IncludeLaunchDescription", path.read_text())
        self.assertNotIn("scrobot_scenarios", source("src/scrobot_bringup/package.xml"))

    def test_python3_syntax(self):
        scripts = sorted(list(SRC.rglob("*.py")) + list((ROOT / "tools").rglob("*.py")))
        for path in scripts:
            with self.subTest(path=str(path.relative_to(ROOT))):
                ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


if __name__ == "__main__":
    unittest.main()
