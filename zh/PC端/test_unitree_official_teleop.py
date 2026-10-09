import unittest

import numpy as np

from robot_config import JointAngles
from unitree_official_teleop import (
    OFFICIAL_G1_23_PROFILE,
    OfficialWeightedMovingFilter,
    UnitreeOfficialImuIkAdapter,
)


class OfficialWeightedMovingFilterTest(unittest.TestCase):

    def test_uses_official_newest_first_weights(self):
        output_filter = OfficialWeightedMovingFilter((0.4, 0.3, 0.2, 0.1))
        result = None
        for value in (0.0, 1.0, 2.0, 3.0):
            result = output_filter.apply(JointAngles(
                value, 0.0, 0.0, 0.0, 0.0, side="right"))
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.shoulder_pitch, 2.0, places=6)

    def test_angle_wrap_does_not_average_through_zero(self):
        output_filter = OfficialWeightedMovingFilter((0.6, 0.4))
        output_filter.apply(JointAngles(
            np.radians(179.0), 0.0, 0.0, 0.0, 0.0, side="right"))
        result = output_filter.apply(JointAngles(
            np.radians(-179.0), 0.0, 0.0, 0.0, 0.0, side="right"))
        self.assertGreater(abs(np.degrees(result.shoulder_pitch)), 175.0)


class UnitreeOfficialImuIkAdapterTest(unittest.TestCase):

    def test_reference_profile_matches_unitree_g1_23(self):
        self.assertEqual(OFFICIAL_G1_23_PROFILE.translation_weight, 50.0)
        self.assertEqual(OFFICIAL_G1_23_PROFILE.rotation_weight, 0.5)
        self.assertEqual(OFFICIAL_G1_23_PROFILE.posture_weight, 0.02)
        self.assertEqual(OFFICIAL_G1_23_PROFILE.smooth_weight, 0.1)
        self.assertEqual(
            OFFICIAL_G1_23_PROFILE.filter_weights, (0.4, 0.3, 0.2, 0.1))

    def test_natural_down_pose_uses_official_adapter_without_outward_twist(self):
        adapter = UnitreeOfficialImuIkAdapter(enable_filter=True)
        down = np.array([0.0, 0.0, -1.0])
        palm_inward = np.array([0.0, -1.0, 0.0])
        solved = JointAngles(
            0.0, 0.0, 0.0, 0.0, 0.0, side="right")
        for _ in range(15):
            solved, diagnostics = adapter.solve_pose(
                down, down, palm_inward, side="right", seed=solved)

        self.assertTrue(diagnostics["official_adapter"])
        self.assertTrue(diagnostics["official_6d_pose"])
        self.assertTrue(diagnostics["official_output_filter"])
        self.assertLess(abs(np.degrees(solved.shoulder_yaw)), 15.0)
        self.assertLess(diagnostics["max_error_deg"], 10.0)
        self.assertLess(diagnostics["palm_error_deg"], 10.0)


if __name__ == "__main__":
    unittest.main()
