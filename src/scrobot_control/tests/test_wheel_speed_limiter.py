"""Pure kinematic regression checks; requires only Python stdlib."""
import ast
import math
from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / "scripts" / "wheel_speed_limiter.py"
TREE = ast.parse(SOURCE.read_text())
FUNCTION = next(node for node in TREE.body
                if isinstance(node, ast.FunctionDef) and
                node.name == "constrain_twist")
ns = {"math": math}
exec(compile(ast.Module(body=[FUNCTION], type_ignores=[]), str(SOURCE), "exec"), ns)
limit = ns["constrain_twist"]
V_MAX = 2 * math.pi * 0.05 * 100 / 60


def feasible(v, omega):
    return max(abs(v - 0.21 * omega), abs(v + 0.21 * omega)) <= V_MAX + 1e-10


def test_all_motion_quadrants():
    for v in (-3.0, -0.7, -0.3, 0.0, 0.3, 0.7, 3.0):
        for w in (-7.0, -3.0, -0.5, 0.0, 0.5, 3.0, 7.0):
            vv, ww = limit(v, w)
            assert feasible(vv, ww)
            if v != 0.0 and w != 0.0:
                assert math.isclose(vv / v, ww / w, abs_tol=1e-10)


def test_no_change_within_envelope():
    assert limit(0.3, 0.4) == (0.3, 0.4)
    assert limit(0.0, 0.0) == (0.0, 0.0)


def test_nonfinite_fails_closed():
    assert limit(float("nan"), 1.0) == (0.0, 0.0)
    assert limit(0.0, float("inf")) == (0.0, 0.0)
    assert limit(1.0, 1.0, max_wheel_rpm=0) == (0.0, 0.0)
