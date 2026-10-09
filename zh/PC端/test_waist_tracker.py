import unittest

import numpy as np

from waist_tracker import (
    WaistTracker, waist_yaw_from_quaternion, wrap_to_pi,
)


def _axis_quaternion(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    half = float(angle) * 0.5
    return np.concatenate(([np.cos(half)], axis * np.sin(half)))


class WaistYawExtractionTest(unittest.TestCase):

    def test_pure_yaw_rotation_is_recovered(self):
        q_zero = np.array([1., 0., 0., 0.])
        for degrees in (-75.0, -20.0, 0.0, 33.0, 88.0):
            q_now = _axis_quaternion([0., 0., 1.], np.radians(degrees))
            self.assertAlmostEqual(
                np.degrees(waist_yaw_from_quaternion(q_zero, q_now)),
                degrees, places=6)

    def test_zero_pose_offset_is_absorbed(self):
        """IMU 佩戴时歪一点，零位捕获后不应产生固定偏差。"""
        q_zero = _axis_quaternion([0.3, 0.2, 0.9], 0.6)
        q_now = quat_rotate_about_zero(q_zero, np.radians(40.0))
        self.assertAlmostEqual(
            np.degrees(waist_yaw_from_quaternion(q_zero, q_now)),
            40.0, places=6)

    def test_leaning_forward_does_not_produce_fake_yaw(self):
        """只前倾不转身时偏航分量应当很小。"""
        q_zero = np.array([1., 0., 0., 0.])
        q_now = _axis_quaternion([0., 1., 0.], np.radians(25.0))
        self.assertLess(
            abs(np.degrees(waist_yaw_from_quaternion(q_zero, q_now))), 1e-6)

    def test_wrap_to_pi_has_no_jump_near_the_zero(self):
        self.assertAlmostEqual(wrap_to_pi(np.radians(359.0)),
                               np.radians(-1.0), places=9)
        self.assertAlmostEqual(wrap_to_pi(np.radians(-359.0)),
                               np.radians(1.0), places=9)


def quat_rotate_about_zero(q_zero, angle):
    """在零位坐标系中绕其 Z 轴再转 angle。"""
    from arm_calibration import quat_mul
    return quat_mul(q_zero, _axis_quaternion([0., 0., 1.], angle))


class WaistTrackerTest(unittest.TestCase):

    def test_output_is_zero_until_the_zero_pose_is_captured(self):
        tracker = WaistTracker()
        self.assertFalse(tracker.calibrated)
        self.assertEqual(
            tracker.update(_axis_quaternion([0., 0., 1.], 0.5), 0.02), 0.0)

    def test_tracks_yaw_after_capture(self):
        tracker = WaistTracker(max_speed_rad_s=1000.0)
        tracker.capture_zero(np.array([1., 0., 0., 0.]))
        self.assertTrue(tracker.calibrated)

        yaw = tracker.update(
            _axis_quaternion([0., 0., 1.], np.radians(35.0)), 0.02)

        self.assertAlmostEqual(np.degrees(yaw), 35.0, places=5)

    def test_yaw_is_clamped_to_the_human_range(self):
        tracker = WaistTracker(max_yaw_rad=1.0, max_speed_rad_s=1000.0)
        tracker.capture_zero(np.array([1., 0., 0., 0.]))

        yaw = tracker.update(
            _axis_quaternion([0., 0., 1.], np.radians(120.0)), 0.02)

        self.assertAlmostEqual(yaw, 1.0, places=9)
        self.assertTrue(tracker.clamped)
        self.assertAlmostEqual(np.degrees(tracker.raw_yaw), 120.0, places=5)

    def test_speed_limit_applies_between_steps(self):
        tracker = WaistTracker(max_speed_rad_s=2.0)
        tracker.capture_zero(np.array([1., 0., 0., 0.]))

        yaw = tracker.update(
            _axis_quaternion([0., 0., 1.], np.radians(60.0)), 0.02)

        self.assertAlmostEqual(yaw, 0.04, places=9)

    def test_sign_flip_inverts_the_direction(self):
        tracker = WaistTracker(max_speed_rad_s=1000.0, sign=-1.0)
        tracker.capture_zero(np.array([1., 0., 0., 0.]))

        yaw = tracker.update(
            _axis_quaternion([0., 0., 1.], np.radians(30.0)), 0.02)

        self.assertAlmostEqual(np.degrees(yaw), -30.0, places=5)

    def test_non_finite_input_holds_the_previous_output(self):
        tracker = WaistTracker(max_speed_rad_s=1000.0)
        tracker.capture_zero(np.array([1., 0., 0., 0.]))
        tracker.update(_axis_quaternion([0., 0., 1.], np.radians(20.0)), 0.02)
        previous = tracker.yaw

        self.assertEqual(
            tracker.update(np.array([np.nan, 0., 0., 0.]), 0.02), previous)
        self.assertEqual(tracker.update(np.array([1., 0., 0., 0.]), 0.0),
                         previous)

    def test_relax_to_zero_respects_the_speed_limit(self):
        tracker = WaistTracker(max_speed_rad_s=1000.0)
        tracker.capture_zero(np.array([1., 0., 0., 0.]))
        tracker.update(_axis_quaternion([0., 0., 1.], np.radians(40.0)), 0.02)
        tracker.max_speed_rad_s = 1.0

        tracker.relax_to_zero(0.1)

        self.assertAlmostEqual(np.degrees(tracker.yaw), 40.0 - np.degrees(0.1),
                               places=5)

    def test_gain_amplifies_the_measured_yaw(self):
        tracker = WaistTracker(max_speed_rad_s=1000.0, gain=1.5)
        tracker.capture_zero(np.array([1., 0., 0., 0.]))

        yaw = tracker.update(
            _axis_quaternion([0., 0., 1.], np.radians(20.0)), 0.02)

        self.assertAlmostEqual(np.degrees(yaw), 30.0, places=4)
        # raw_yaw 保留未放大的实测值，便于对照日志。
        self.assertAlmostEqual(np.degrees(tracker.raw_yaw), 20.0, places=4)

    def test_gain_is_applied_before_the_clamp(self):
        tracker = WaistTracker(max_yaw_rad=1.0, max_speed_rad_s=1000.0,
                               gain=3.0)
        tracker.capture_zero(np.array([1., 0., 0., 0.]))

        yaw = tracker.update(
            _axis_quaternion([0., 0., 1.], np.radians(40.0)), 0.02)

        self.assertAlmostEqual(yaw, 1.0, places=9)
        self.assertTrue(tracker.clamped)

    def test_gain_is_bounded(self):
        tracker = WaistTracker(gain=99.0)
        self.assertEqual(tracker.gain, 3.0)
        self.assertEqual(WaistTracker(gain=0.0).gain, 0.5)

    def test_adjust_gain_steps_and_saturates(self):
        tracker = WaistTracker(gain=1.0)
        self.assertAlmostEqual(tracker.adjust_gain(0.1), 1.1, places=9)
        for _ in range(50):
            tracker.adjust_gain(0.1)
        self.assertEqual(tracker.gain, 3.0)
        for _ in range(100):
            tracker.adjust_gain(-0.1)
        self.assertEqual(tracker.gain, 0.5)

    def test_large_yaw_is_wrapped_before_the_gain(self):
        """先放大再折回会在 ±180° 附近翻符号，顺序必须是先折回。"""
        tracker = WaistTracker(max_yaw_rad=10.0, max_speed_rad_s=1000.0,
                               gain=2.0)
        tracker.capture_zero(np.array([1., 0., 0., 0.]))

        yaw = tracker.update(
            _axis_quaternion([0., 0., 1.], np.radians(170.0)), 0.02)

        self.assertAlmostEqual(np.degrees(yaw), 340.0, places=3)

    def test_reset_clears_the_zero_pose(self):
        tracker = WaistTracker()
        tracker.capture_zero(np.array([1., 0., 0., 0.]))
        tracker.reset()
        self.assertFalse(tracker.calibrated)
        self.assertEqual(tracker.yaw, 0.0)


if __name__ == "__main__":
    unittest.main()
