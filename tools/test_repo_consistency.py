#!/usr/bin/env python3
"""Cross-package, no-ROS regression tests for integration conflict prevention.

Individual Lighting, Mobility and Odometry Lab suites test their own behavior;
this suite checks shared topic names, launch boundaries and web ports.
"""
from __future__ import annotations

import re
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def contents(relative):
    return (ROOT / relative).read_text(encoding="utf-8")


class RepositoryConsistencyTests(unittest.TestCase):
    def test_three_browser_labs_use_separate_ports(self):
        sources = {
            "lighting": ("tools/lighting_lab/server.py", r'parser\.add_argument\("--port", type=int, default=(\d+)\)'),
            "mobility": ("tools/mobility_lab/server.py", r'parser\.add_argument\("--port", type=int, default=(\d+)\)'),
            "odometry": ("tools/odometry_lab/web_server.py", r"ap\.add_argument\('--port', type=int, default=(\d+)\)"),
        }
        ports = {}
        for name, (path, pattern) in sources.items():
            m = re.search(pattern, contents(path))
            self.assertIsNotNone(m, path)
            ports[name] = int(m.group(1))
        self.assertEqual(ports, {"lighting": 8765, "mobility": 8766, "odometry": 8767})
        root_readme = contents("README.md")
        for port in ports.values():
            self.assertIn(f"127.0.0.1:{port}", root_readme)

    def test_ground_truth_bridge_is_shared_consistently(self):
        bridge = contents("src/scrobot_simulation/config/ros_gz_bridge.yaml")
        self.assertIn("ros_topic_name: /evaluation/ground_truth_odom", bridge)
        mobility = contents("tools/mobility_lab/ros_adapter.py")
        odometry = contents("tools/odometry_lab/config/lab.yaml")
        self.assertIn('"/evaluation/ground_truth_odom"', mobility)
        self.assertIn("/evaluation/ground_truth_odom", odometry)
        # Never replace estimator data with known-perfect Gazebo ground truth.
        nav2 = contents("src/scrobot_navigation/config/nav2_params.yaml")
        self.assertNotIn("odom_topic: /evaluation/ground_truth_odom", nav2)

    def test_published_motor_model_is_simulation_only(self):
        sim = contents("src/scrobot_bringup/launch/sim.launch.py")
        hardware = contents("src/scrobot_bringup/launch/robot.launch.py")
        stack = contents("src/scrobot_control/launch/control_stack.launch.py")
        self.assertIn("motor_model_enabled", sim)
        self.assertIn("sim_motor_response", sim)
        self.assertIn("/cmd_vel_motor_target", sim)
        self.assertIn("collision_monitor_output", stack)
        self.assertNotIn("motor_model_enabled", hardware)
        self.assertNotIn("/cmd_vel_motor_target", hardware)
        self.assertNotIn("mobility_calibration.yaml", hardware)
        self.assertIn("'/diff_drive_controller/cmd_vel'", stack)

    def test_mobility_and_lighting_modify_distinct_configurations(self):
        mobility = contents("tools/mobility_lab/publish.py")
        lighting = contents("tools/lighting_lab/publish.py")
        self.assertIn("config/mobility_calibration.yaml", mobility)
        self.assertIn("worlds/badminton_court.sdf", lighting)
        self.assertNotIn("worlds/badminton_court.sdf", mobility)
        self.assertNotIn("mobility_calibration.yaml", lighting)
        self.assertNotIn("scrobot_system.cpp", mobility)


    def test_lab_result_isolation_and_no_unintended_overwrites(self):
        lighting = contents("tools/lighting_lab/server.py")
        mobility = contents("tools/mobility_lab/server.py")
        odometry = contents("tools/odometry_lab/storage.py")
        web = contents("tools/odometry_lab/web_server.py")
        replay = contents("tools/odometry_lab/replay.py")

        # All three labs save under independent per-lab result trees.
        self.assertIn("evaluation_results/lighting_lab/presets", lighting)
        self.assertIn("evaluation_results/mobility_lab", mobility)
        self.assertIn('"lighting_lab"', odometry)
        self.assertIn('"mobility_lab"', odometry)
        self.assertIn("reserve_run_dir", odometry)
        self.assertIn("validated_result_root", web)
        self.assertIn("validated_result_root", replay)

        # Only Lighting/Mobility explicit publishing edits production.
        self.assertNotIn("mobility_calibration.yaml", web)
        self.assertNotIn("badminton_court.sdf", web)
        self.assertIn('output.open("x"', lighting)
        self.assertIn("run_log.open('x')", web)

    def test_labs_use_separate_ros_gazebo_domains(self):
        lighting = contents("tools/lighting_lab/server.py")
        mobility = contents("tools/mobility_lab/server.py")
        odometry = contents("tools/odometry_lab/web_server.py")
        self.assertIn('default=76', lighting)
        self.assertIn('default=77', mobility)
        for source in (lighting, mobility):
            self.assertIn('ROS_DOMAIN_ID', source)
            self.assertIn('GZ_PARTITION', source)
        # Odometry should attach to the normal ROS domain; it never
        # launches a second gazebo or claims a private motor interface.
        self.assertIn("simulated_topics_present", odometry)
        self.assertNotIn('GZ_PARTITION', odometry)

    def test_odometry_motion_requires_single_sim_clock_and_control_stack(self):
        sim = contents("src/scrobot_bringup/launch/sim.launch.py")
        self.assertIn("condition=IfCondition(launch_mission)", sim)
        self.assertIn("'launch_mission', default_value='true'", sim)

        preflight = contents("tools/odometry_lab/web_server.py")
        for requirement in (
            "topics['wheel_odom']", "topics['ekf_odom']", "topics['joint_states']",
            "count != 1", "'/control/set_manual_mode'",
        ):
            self.assertIn(requirement, preflight)

        mobility = contents("tools/mobility_lab/server.py")
        self.assertIn('default=77', mobility)
        self.assertIn('GZ_PARTITION', mobility)
        lighting = contents("tools/lighting_lab/server.py")
        self.assertIn('configure_warp_safe_network()', lighting)

    def test_production_scene_lights_have_no_duplicate_sdf_children(self):
        root = ET.fromstring(contents("src/scrobot_simulation/worlds/badminton_court.sdf"))
        world = root.find("world")
        self.assertIsNotNone(world)
        lights = world.findall("light")
        self.assertGreater(len(lights), 0)
        names = [light.get("name") for light in lights]
        self.assertEqual(len(set(names)), len(names))
        for light in lights:
            for tag in ("pose", "diffuse", "specular", "intensity", "direction",
                        "attenuation", "cast_shadows"):
                self.assertEqual(len(light.findall(tag)), 1, f"{light.get('name')}.{tag}")
            if light.get("type") == "spot":
                self.assertEqual(len(light.findall("spot")), 1, light.get("name"))
        self.assertLessEqual(
            sum(light.findtext("cast_shadows") == "true" for light in lights), 2
        )


if __name__ == "__main__":
    unittest.main()
