import unittest

import numpy as np

from arm_calibration import (
    ArmDirectionCalibrator, IncrementalTracker, TwistCalibration,
    fit_hinge_axis, normalize_vec, palm_pose_consistency_deg,
    quat_mul, quaternion_angle_deg,
)


def _axis_quaternion(axis, angle):
    axis = normalize_vec(np.asarray(axis, dtype=float))
    half = float(angle) * 0.5
    return np.concatenate(([np.cos(half)], axis * np.sin(half)))


def _synthetic_hinge_samples(
        motion_deg=88.0, upper_drift_deg=3.0,
        noise_deg=0.12, outlier=False, seed=7):
    rng = np.random.default_rng(seed)
    count = 480
    times = np.linspace(0.0, 1.0, count)
    hinge_axis = normalize_vec(np.array([0.24, 0.95, -0.18]))
    chest = []
    upper = []
    forearm = []
    for index, t in enumerate(times):
        # 三次由伸直到屈曲再返回的平滑往返。
        elbow_angle = np.radians(motion_deg) * (
            0.5 - 0.5 * np.cos(6.0 * np.pi * t))
        upper_angle = np.radians(upper_drift_deg) * np.sin(np.pi * t)
        q_chest = np.array([1., 0., 0., 0.])
        q_upper = _axis_quaternion([0.1, 0.3, 0.95], upper_angle)
        q_hinge = _axis_quaternion(hinge_axis, elbow_angle)

        noise_axis = normalize_vec(rng.normal(size=3))
        noise_angle = np.radians(rng.normal(0.0, noise_deg))
        q_noise = _axis_quaternion(noise_axis, noise_angle)
        q_forearm = quat_mul(quat_mul(q_upper, q_hinge), q_noise)
        if outlier and index == count // 2:
            q_upper = _axis_quaternion([0., 1., 0.], np.radians(35.0))

        chest.append(q_chest)
        upper.append(q_upper)
        forearm.append(q_forearm)
    return chest, upper, forearm


class HingeAxisCalibrationTest(unittest.TestCase):

    def test_clean_three_repetition_motion_passes(self):
        samples = _synthetic_hinge_samples()
        result = fit_hinge_axis(*samples)
        self.assertTrue(result.valid, result.reason)
        self.assertGreater(result.motion_range_deg, 80.0)
        self.assertLess(result.axis_dispersion_deg, 8.0)
        self.assertLess(result.upper_motion_deg, 6.0)

    def test_single_wireless_outlier_does_not_reject_motion(self):
        samples = _synthetic_hinge_samples(outlier=True)
        result = fit_hinge_axis(*samples)
        self.assertTrue(result.valid, result.reason)
        self.assertLess(result.upper_motion_deg, 10.0)

    def test_small_elbow_range_produces_actionable_reason(self):
        samples = _synthetic_hinge_samples(motion_deg=12.0)
        result = fit_hinge_axis(*samples)
        self.assertFalse(result.valid)
        self.assertIn("有效屈伸范围", result.reason)

    def test_sustained_shoulder_compensation_is_rejected(self):
        samples = _synthetic_hinge_samples(upper_drift_deg=38.0)
        result = fit_hinge_axis(*samples)
        self.assertFalse(result.valid)
        self.assertIn("大臂相对胸部移动", result.reason)


class PalmEndpointCalibrationTest(unittest.TestCase):

    def test_up_and_down_endpoints_recover_palm_axis(self):
        calibrator = ArmDirectionCalibrator(label="测试小臂")
        calibrator.R_align = np.eye(3)
        calibrator.arm_body_dir = np.array([1., 0., 0.])
        calibrator.calibrated = True
        q_down = np.array([1., 0., 0., 0.])
        q_up = _axis_quaternion([1., 0., 0.], np.pi)

        result = TwistCalibration.from_palm_poses(
            q_up, q_down, calibrator)

        np.testing.assert_allclose(
            result.palm_in_forearm_imu, [0., 0., -1.], atol=1e-7)
        np.testing.assert_allclose(
            result.compute_palm_direction(q_up, calibrator),
            [0., 0., 1.], atol=1e-7)
        np.testing.assert_allclose(
            result.compute_palm_direction(q_down, calibrator),
            [0., 0., -1.], atol=1e-7)
        self.assertLess(result.palm_pose_consistency_deg, 1e-6)

    def test_preview_consistency_matches_the_saved_verdict(self):
        """实时预览与最终判定必须是同一份计算，否则预览会骗人。"""
        calibrator = ArmDirectionCalibrator(label="测试小臂")
        calibrator.R_align = _axis_quaternion([0.2, 0.9, 0.3], 0.7)
        calibrator.R_align = np.array([
            [0.936, -0.290, 0.198],
            [0.312, 0.947, -0.081],
            [-0.163, 0.137, 0.977]])
        calibrator.arm_body_dir = np.array([1., 0., 0.])
        calibrator.calibrated = True
        # 朝下端点偏离理想的 180° 翻转，制造一个非零但可接受的偏差。
        q_up = _axis_quaternion([1., 0., 0.], np.pi)
        q_down = _axis_quaternion([1., 0.06, 0.04], np.radians(11.0))

        preview = palm_pose_consistency_deg(q_up, q_down, calibrator)
        saved = TwistCalibration.from_palm_poses(
            q_up, q_down, calibrator).palm_pose_consistency_deg

        self.assertGreater(preview, 0.0)
        self.assertAlmostEqual(preview, saved, places=9)

    def test_preview_predicts_rejection_above_the_threshold(self):
        calibrator = ArmDirectionCalibrator(label="测试小臂")
        calibrator.R_align = np.eye(3)
        calibrator.arm_body_dir = np.array([1., 0., 0.])
        calibrator.calibrated = True
        identity = np.array([1., 0., 0., 0.])

        preview = palm_pose_consistency_deg(identity, identity, calibrator)

        self.assertGreater(preview, 30.0)
        with self.assertRaisesRegex(ValueError, "不一致"):
            TwistCalibration.from_palm_poses(
                identity, identity, calibrator, max_consistency_deg=30.0)

    def test_inconsistent_endpoints_are_rejected(self):
        calibrator = ArmDirectionCalibrator(label="测试小臂")
        calibrator.R_align = np.eye(3)
        calibrator.arm_body_dir = np.array([1., 0., 0.])
        calibrator.calibrated = True
        identity = np.array([1., 0., 0., 0.])

        with self.assertRaisesRegex(ValueError, "不一致"):
            TwistCalibration.from_palm_poses(
                identity, identity, calibrator,
                max_consistency_deg=30.0)

    def test_new_quality_fields_survive_save_and_load(self):
        source = TwistCalibration(
            np.array([1., 0., 0., 0.]),
            np.array([1., 0., 0.]),
            np.array([0., 0., -1.]),
            palm_pose_consistency_deg=4.5,
            palm_up_error_deg=2.0,
            palm_down_error_deg=2.5)

        loaded = TwistCalibration.load_dict(source.save_dict())

        self.assertEqual(loaded.palm_pose_consistency_deg, 4.5)
        self.assertEqual(loaded.palm_up_error_deg, 2.0)
        self.assertEqual(loaded.palm_down_error_deg, 2.5)


class IncrementalTrackerTest(unittest.TestCase):

    def test_stationary_rest_lock_rejects_relative_heading_drift(self):
        tracker = IncrementalTracker(rest_confirm_frames=4)
        identity = np.array([1., 0., 0., 0.])
        tracker.init(identity, identity)

        for index in range(1, 201):
            link = _axis_quaternion(
                [0., 0., 1.], np.radians(0.02 * index))
            tracker.update(
                identity, link, chest_rest=True, link_rest=True)

        self.assertTrue(tracker.drift_locked)
        self.assertLess(
            quaternion_angle_deg(identity, tracker.q_rel_inc), 0.15)

    def test_motion_unlocks_immediately_without_output_jump(self):
        tracker = IncrementalTracker(rest_confirm_frames=2)
        identity = np.array([1., 0., 0., 0.])
        tracker.init(identity, identity)
        drifted = _axis_quaternion([0., 0., 1.], np.radians(0.2))
        tracker.update(identity, drifted, chest_rest=True, link_rest=True)
        tracker.update(identity, drifted, chest_rest=True, link_rest=True)
        locked_pose = tracker.q_rel_inc.copy()

        moved = quat_mul(
            drifted, _axis_quaternion([1., 0., 0.], np.radians(20.0)))
        tracker.update(identity, moved, chest_rest=False, link_rest=False)

        self.assertFalse(tracker.drift_locked)
        self.assertGreater(
            quaternion_angle_deg(locked_pose, tracker.q_rel_inc), 19.0)


if __name__ == "__main__":
    unittest.main()
