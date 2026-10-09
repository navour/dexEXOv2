import unittest

import numpy as np

import arm_solver
from arm_solver import (
    JointAngles,
    clamp_joint_angles,
    direction_to_joint_angles,
    evaluate_direction_mapping_deg,
    forward_kinematics_left_arm_full,
    forward_kinematics_right_arm_full,
    solve_direction_ik,
)


def _targets_from_angles(angles):
    fk = (forward_kinematics_left_arm_full(angles)
          if angles.side == "left" else
          forward_kinematics_right_arm_full(angles))
    upper_urdf = fk["elbow"] - fk["shoulder_pitch"]
    forearm_urdf = fk["wrist"] - fk["elbow"]
    upper_urdf /= np.linalg.norm(upper_urdf)
    forearm_urdf /= np.linalg.norm(forearm_urdf)
    # URDF → chest 与 chest → URDF 都是翻转 Y。
    upper_chest = np.array([upper_urdf[0], -upper_urdf[1], upper_urdf[2]])
    forearm_chest = np.array([
        forearm_urdf[0], -forearm_urdf[1], forearm_urdf[2]])
    return upper_chest, forearm_chest


class DirectionIkTest(unittest.TestCase):

    def test_clamp_preserves_wrist_roll(self):
        angles = JointAngles(0.2, -0.1, 0.3, 0.7, 1.1, side="right")
        clamped = clamp_joint_angles(angles)
        self.assertAlmostEqual(clamped.wrist_roll, 1.1)

    def test_right_official_fk_target_is_reproduced(self):
        expected = JointAngles(0.45, -0.35, 0.55, 0.85, 0.6, side="right")
        upper, forearm = _targets_from_angles(expected)
        solved, diagnostics = solve_direction_ik(
            upper, forearm, wrist_roll=expected.wrist_roll,
            side="right", seed=JointAngles(0., 0., 0., 0., 0., side="right"))
        self.assertLess(diagnostics["max_error_deg"], 1.0)
        self.assertAlmostEqual(solved.wrist_roll, expected.wrist_roll)

    def test_left_official_fk_target_is_reproduced(self):
        expected = JointAngles(-0.35, 0.30, -0.45, 0.75, -0.5, side="left")
        upper, forearm = _targets_from_angles(expected)
        solved, diagnostics = solve_direction_ik(
            upper, forearm, wrist_roll=expected.wrist_roll,
            side="left", seed=JointAngles(0., 0., 0., 0., 0., side="left"))
        self.assertLess(diagnostics["max_error_deg"], 1.0)
        self.assertAlmostEqual(solved.wrist_roll, expected.wrist_roll)

    def test_optimizer_is_not_worse_than_analytic_mapping(self):
        expected = JointAngles(0.8, -0.7, 0.9, 1.1, 0.0, side="right")
        upper, forearm = _targets_from_angles(expected)
        analytic = direction_to_joint_angles(
            upper, forearm, side="right",
            seed=JointAngles(0., 0., 0., 0., 0., side="right"))
        analytic_error = evaluate_direction_mapping_deg(
            upper, forearm, analytic)["max_error_deg"]
        _, diagnostics = solve_direction_ik(
            upper, forearm, side="right", seed=analytic)
        self.assertLessEqual(
            diagnostics["max_error_deg"], analytic_error + 0.05)

    def test_natural_down_pose_avoids_outward_shoulder_twist(self):
        down = np.array([0., 0., -1.])
        solved, diagnostics = solve_direction_ik(
            down, down, side="right",
            seed=JointAngles(0., 0., 0., 0., 0., side="right"))

        self.assertTrue(diagnostics["posture_assisted"])
        self.assertLess(abs(np.degrees(solved.shoulder_yaw)), 15.0)
        self.assertLess(diagnostics["max_error_deg"], 10.0)

    def test_down_pose_recovers_from_previous_outward_branch(self):
        down = np.array([0., 0., -1.])
        outward_seed = JointAngles(
            np.radians(1.0), np.radians(19.0), np.radians(-84.0),
            np.radians(65.0), 0.0, side="right")
        solved, _ = solve_direction_ik(
            down, down, side="right", seed=outward_seed)

        self.assertLess(abs(np.degrees(solved.shoulder_yaw)), 20.0)

    def test_straight_arm_raise_keeps_natural_continuous_branch(self):
        seed = JointAngles(0., 0., 0., 0., 0., side="right")
        previous = None

        for angle in np.linspace(0.0, np.pi / 2.0, 19):
            direction = np.array([
                np.sin(angle), 0.0, -np.cos(angle),
            ])
            solved, diagnostics = solve_direction_ik(
                direction, direction, side="right", seed=seed)

            self.assertLess(abs(np.degrees(solved.shoulder_yaw)), 15.0)
            self.assertLess(diagnostics["max_error_deg"], 10.0)
            if previous is not None:
                joint_step_deg = np.max(np.abs(np.degrees(
                    solved.as_array()[:4] - previous.as_array()[:4])))
                self.assertLess(joint_step_deg, 8.0)
            previous = solved
            seed = solved


if __name__ == "__main__":
    unittest.main()
