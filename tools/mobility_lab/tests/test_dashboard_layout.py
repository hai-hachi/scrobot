"""Static layout contracts for Mobility Lab wheel plots.

Runs without ROS or a browser. Matching width, height, axes and the
shared draw function prevent the old narrow-left/full-width-right mismatch.
"""
from html.parser import HTMLParser
from pathlib import Path
import re
import unittest


DASHBOARD = Path(__file__).resolve().parents[1] / "index.html"


class CanvasInspector(HTMLParser):
    def __init__(self):
        super().__init__()
        self.stack = []
        self.canvases = {}

    def handle_starttag(self, tag, attrs):
        props = dict(attrs)
        parent = next((a for t, a in reversed(self.stack) if t == "div"), {})
        if tag == "canvas":
            key = props.get("id")
            self.canvases[key] = {
                "attributes": props,
                "parent_class": parent.get("class"),
                "hierarchy": tuple(
                    a.get("class", "") for t, a in self.stack if t == "div"
                ),
            }
        # All tags in dashboard use standard paired tags except void tags.
        if tag not in {"meta", "input", "hr", "br", "img", "link"}:
            self.stack.append((tag, props))

    def handle_endtag(self, tag):
        for i in range(len(self.stack) - 1, -1, -1):
            if self.stack[i][0] == tag:
                del self.stack[i:]
                return


class MobilityDashboardLayoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = DASHBOARD.read_text(encoding="utf-8")
        cls.inspector = CanvasInspector()
        cls.inspector.feed(cls.html)

    def test_wheel_charts_have_identical_dimensions_and_containers(self):
        left = self.inspector.canvases["leftChart"]
        right = self.inspector.canvases["rightChart"]
        self.assertEqual(left["attributes"]["class"], "chart wheel-chart")
        self.assertEqual(left["attributes"]["class"], right["attributes"]["class"])
        self.assertEqual(left["parent_class"], "wheel-plot")
        self.assertEqual(left["parent_class"], right["parent_class"])
        self.assertEqual(left["hierarchy"], right["hierarchy"])
        self.assertIn(".wheel-chart{height:240px}", self.html)
        self.assertIn(".chart{width:100%;", self.html)
        self.assertNotIn("chart short", self.html)
        self.assertNotIn(".chart.short", self.html)

    def test_ground_truth_is_separate_from_both_rpm_plots(self):
        path = self.inspector.canvases["pathChart"]
        self.assertEqual(path["attributes"]["class"], "chart path-chart")
        self.assertIn("trajectory-section", path["hierarchy"])
        self.assertNotIn("wheel-plot-stack", path["hierarchy"])
        self.assertNotIn("trajectory-section",
                         self.inspector.canvases["leftChart"]["hierarchy"])
        self.assertNotIn("trajectory-section",
                         self.inspector.canvases["rightChart"]["hierarchy"])

    def test_both_wheels_use_same_chart_renderer_and_axes(self):
        self.assertIn('chart("leftChart",data,"left")', self.html)
        self.assertIn('chart("rightChart",data,"right")', self.html)
        self.assertEqual(len(re.findall(r"function chart\(", self.html)), 1)
        self.assertIn("const xMin=-0.08,xMax=", self.html)
        self.assertIn("const yMin=-yRange,yMax=yRange;", self.html)
        self.assertIn("Math.round(rect.width*dpr)", self.html)
        self.assertIn("Math.round(rect.height*dpr)", self.html)


if __name__ == "__main__":
    unittest.main()
