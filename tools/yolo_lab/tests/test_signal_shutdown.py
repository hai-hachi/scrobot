"""Unit checks for graceful ROS shutdown without requiring ROS/Gazebo.

Execute the real main() function AST from each ROS node against lightweight
rclpy stubs. This catches regressions where KeyboardInterrupt leaks or
shutdown is called twice after the context is already stopped.
"""
from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = (
    ("tools/yolo_lab/preview.py", "Preview", ["preview.py", "preview.jpg"]),
    ("tools/camera_lab/monitor.py", "CameraMeter",
     ["monitor.py", "meter.json", "false"]),
    ("tools/camera_lab/tag_monitor.py", "TagMonitor",
     ["tag_monitor.py", "results"]),
)


class FakeExternalShutdown(Exception):
    pass


class ShutdownTests(unittest.TestCase):
    def _run(self, script, node_name, argv, spin_error):
        path = ROOT / script
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        functions = [item for item in tree.body
                     if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
                     and item.name == "main"]
        self.assertEqual(len(functions), 1)
        calls = {"init": 0, "spin": 0, "destroy": 0, "try_shutdown": 0}

        class StubNode:
            def __init__(self, *args):
                self.args = args

            def destroy_node(self):
                calls["destroy"] += 1

        class StubRclpy:
            def init(self):
                calls["init"] += 1

            def spin(self, node):
                self_node.assertIsInstance(node, StubNode)
                calls["spin"] += 1
                if spin_error is not None:
                    raise spin_error()

            def try_shutdown(self):
                calls["try_shutdown"] += 1

        self_node = self
        namespace = {
            "sys": SimpleNamespace(argv=argv),
            "rclpy": StubRclpy(),
            "ExternalShutdownException": FakeExternalShutdown,
            node_name: StubNode,
        }
        function_module = ast.fix_missing_locations(
            ast.Module(body=functions, type_ignores=[]))
        exec(compile(function_module, str(path), "exec"), namespace)
        if spin_error is RuntimeError:
            with self.assertRaises(RuntimeError):
                namespace["main"]()
        else:
            namespace["main"]()

        self.assertEqual(calls["init"], 1, script)
        self.assertEqual(calls["spin"], 1, script)
        self.assertEqual(calls["destroy"], 1, script)
        self.assertEqual(calls["try_shutdown"], 1, script)

    def test_sigint_shutdown_is_clean_in_all_three_nodes(self):
        for script, node_name, argv in SCRIPTS:
            for kind in (KeyboardInterrupt, FakeExternalShutdown, None,
                         RuntimeError):
                with self.subTest(script=script, kind=str(kind)):
                    self._run(script, node_name, argv, kind)


if __name__ == "__main__":
    unittest.main()
