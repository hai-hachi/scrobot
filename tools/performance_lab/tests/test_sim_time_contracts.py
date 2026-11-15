"""Regression contracts for ROS-time pacing (no ROS runtime required)."""
import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def calls(path, class_name, method_name):
    source = (ROOT / path).read_text()
    module = ast.parse(source)
    cls = next(x for x in module.body if isinstance(x, ast.ClassDef) and x.name == class_name)
    method = next(x for x in cls.body if isinstance(x, ast.FunctionDef) and x.name == method_name)
    return source, {ast.unparse(x.func) for x in ast.walk(method) if isinstance(x, ast.Call)}


def test_local_collect_sim_time_control():
    path = "src/scrobot_collection/scrobot_collection/local_collect_controller.py"
    source, methods = calls(path, "LocalCollectController", "_drive_precollect_pose")
    assert "self._now_seconds" in methods
    assert "self._sleep_sim" in methods
    assert "time.monotonic()" not in source
    assert "time.sleep(period)" not in source
    assert "self.raw_stamp_sim_s" in source


def test_yolo_source_stamped_throttle():
    source, methods = calls("src/scrobot_perception/scripts/yolo_shuttle_detector.py", "YoloShuttleDetector", "_image_pair_callback")
    assert "color_msg.header.stamp" in source
    assert "self._last_inference_stamp" in source
    assert "self._last_inference_wall" not in source
    assert "time.perf_counter" in methods  # measures compute time only


def test_command_timing_observer_topics():
    source = (ROOT / "tools/performance_lab/monitor.py").read_text()
    for topic in ("/cmd_vel_nav", "/cmd_vel_approach", "/cmd_vel_auto",
                  "/cmd_vel_smoothed", "/cmd_vel_motor_target",
                  "/diff_drive_controller/cmd_vel"):
        assert topic in source
    assert "serialized_header_stamp(msg)" in source
