"""Camera-fixed AprilTag quality metrics are repeatable but not absolute pose truth."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tag_quality import TagQuality, quaternion_angular_distance


class TagQualityTests(unittest.TestCase):
    def test_quaternion_sign_invariance(self):
        a=(0.0,0.0,0.0,1.0)
        b=tuple(-x for x in a)
        self.assertAlmostEqual(quaternion_angular_distance(a,b),0,places=6)
        half=math.pi/4
        c=(0,0,math.sin(half),math.cos(half))
        self.assertAlmostEqual(quaternion_angular_distance(a,c),90,places=6)

    def test_consistent_pose_has_zero_jitter(self):
        quality=TagQuality()
        for i in range(20):
            quality.add([{"id":0,"margin":35.5,"hamming":0,
                          "position":(1.2,0.1,0.2),
                          "quaternion":(0,0,0,1)}])
        tag=quality.status()[0]
        self.assertEqual(tag["detections"],20)
        self.assertEqual(tag["pose_samples"],20)
        self.assertEqual(tag["translation_std_m"],0.0)
        self.assertEqual(tag["rotation_rms_deg"],0.0)
        self.assertEqual(tag["detector_frame_fraction"],1)
        self.assertAlmostEqual(tag["mean_margin"],35.5)
        self.assertAlmostEqual(tag["range_mean_m"],math.sqrt(1.49),places=3)

    def test_detection_fraction_and_pose_unavailable(self):
        quality=TagQuality()
        for _ in range(6):
            quality.add([])
        for _ in range(4):
            quality.add([{"id":3,"margin":11.1}])
        tag=quality.status()[0]
        self.assertAlmostEqual(tag["detector_frame_fraction"],0.4)
        self.assertIsNone(tag["translation_std_m"])
        self.assertIsNone(tag["rotation_rms_deg"])


if __name__=="__main__":
    unittest.main()
