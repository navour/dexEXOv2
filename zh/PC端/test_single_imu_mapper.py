#!/usr/bin/env python3
"""单右小臂 IMU 诊断映射测试。"""

import math
import unittest

import numpy as np

from single_imu_mapper import RightArmTwoImuMapper, SingleForearmMapper


def quat_x(angle: float) -> np.ndarray:
    return np.array([math.cos(angle / 2.0), math.sin(angle / 2.0), 0.0, 0.0])


def quat_y(angle: float) -> np.ndarray:
    return np.array([math.cos(angle / 2.0), 0.0, math.sin(angle / 2.0), 0.0])


def quat_z(angle: float) -> np.ndarray:
    return np.array([math.cos(angle / 2.0), 0.0, 0.0, math.sin(angle / 2.0)])


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


class SingleForearmMapperTests(unittest.TestCase):
    def test_zero_pose(self) -> None:
        mapper = SingleForearmMapper()
        mapper.calibrate(np.array([1.0, 0.0, 0.0, 0.0]))
        elbow, wrist = mapper.compute(np.array([1.0, 0.0, 0.0, 0.0]))
        self.assertAlmostEqual(elbow, 0.0)
        self.assertAlmostEqual(wrist, 0.0)

    def test_elbow_and_wrist_decomposition(self) -> None:
        mapper = SingleForearmMapper()
        mapper.calibrate(np.array([1.0, 0.0, 0.0, 0.0]))
        expected_elbow = 0.7
        expected_wrist = -0.4
        current = quat_mul(quat_y(expected_elbow), quat_x(expected_wrist))
        elbow, wrist = mapper.compute(current)
        self.assertAlmostEqual(elbow, expected_elbow, places=7)
        self.assertAlmostEqual(wrist, expected_wrist, places=7)

    def test_configurable_signs(self) -> None:
        mapper = SingleForearmMapper(elbow_sign=-1.0, wrist_sign=-1.0)
        mapper.calibrate(np.array([1.0, 0.0, 0.0, 0.0]))
        elbow, wrist = mapper.compute(quat_mul(quat_y(0.3), quat_x(0.2)))
        self.assertAlmostEqual(elbow, -0.3)
        self.assertAlmostEqual(wrist, -0.2)


class RightArmTwoImuMapperTests(unittest.TestCase):
    def test_shoulder_elbow_and_wrist_decomposition(self) -> None:
        mapper = RightArmTwoImuMapper()
        identity = np.array([1.0, 0.0, 0.0, 0.0])
        mapper.calibrate(identity, identity)

        expected = (0.35, -0.25, 0.2, 0.75, -0.3)
        shoulder = quat_mul(
            quat_mul(quat_y(expected[0]), quat_x(expected[1])),
            quat_z(expected[2]),
        )
        forearm = quat_mul(
            quat_mul(shoulder, quat_y(expected[3])),
            quat_x(expected[4]),
        )
        actual = mapper.compute(shoulder, forearm)
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value, places=7)

    def test_common_non_identity_zero_pose_is_cancelled(self) -> None:
        mapper = RightArmTwoImuMapper()
        base = quat_z(0.8)
        mapper.calibrate(base, base)
        shoulder_delta = quat_y(0.4)
        upper = quat_mul(base, shoulder_delta)
        forearm = quat_mul(upper, quat_y(0.6))
        sp, sr, sy, elbow, wrist = mapper.compute(upper, forearm)
        self.assertAlmostEqual(sp, 0.4, places=7)
        self.assertAlmostEqual(sr, 0.0, places=7)
        self.assertAlmostEqual(sy, 0.0, places=7)
        self.assertAlmostEqual(elbow, 0.6, places=7)
        self.assertAlmostEqual(wrist, 0.0, places=7)


if __name__ == "__main__":
    unittest.main()
