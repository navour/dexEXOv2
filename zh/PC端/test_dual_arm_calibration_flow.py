import csv
import json
import os
import tempfile
import time
import unittest
from types import SimpleNamespace

import numpy as np

from arm_calibration import palm_pose_consistency_deg
from dual_arm_viz import (
    DualArmViz, GUIDED_CALIB_POSES, SessionDataLogger, _ArmState,
    _calibration_quality_grade)


def _axis_quaternion(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    half = angle * 0.5
    return np.concatenate(([np.cos(half)], axis * np.sin(half)))


class DualArmCalibrationFlowTest(unittest.TestCase):

    def test_saved_v4_calibration_is_loaded_without_recalibration(self):
        calibration_path = os.path.join(
            os.path.dirname(__file__), "imu_motor_calibration_right.json")
        with open(calibration_path, "r") as calibration_file:
            saved_calibration = json.load(calibration_file)

        viz = DualArmViz.__new__(DualArmViz)
        viz.right = _ArmState("right")
        viz.left = _ArmState("left")
        viz._logs = []
        self.assertTrue(viz._load_calibration())
        self.assertTrue(viz.right.upper_calibrated)
        self.assertTrue(viz.right.forearm_calibrated)
        self.assertEqual(
            viz.right.calibration_quality, saved_calibration.get("quality", {}))

    def test_twenty_degree_boundary_is_robot_safe(self):
        usable, robot_safe, grade = _calibration_quality_grade(20.0, 20.0)
        self.assertTrue(usable)
        self.assertTrue(robot_safe)
        self.assertEqual(grade, "robot_safe")

    def test_moderate_error_remains_available_for_simulation(self):
        usable, robot_safe, grade = _calibration_quality_grade(26.0, 24.0)
        self.assertTrue(usable)
        self.assertFalse(robot_safe)
        self.assertEqual(grade, "simulation_only")

    def test_excessive_error_is_rejected(self):
        usable, robot_safe, grade = _calibration_quality_grade(36.0, 24.0)
        self.assertFalse(usable)
        self.assertFalse(robot_safe)
        self.assertEqual(grade, "rejected")

    def test_simulation_target_is_recognized_as_local(self):
        self.assertTrue(DualArmViz._target_is_local(("127.0.0.1", 9527)))
        self.assertFalse(DualArmViz._target_is_local(("192.168.3.85", 9527)))

    def test_roll_motion_range_uses_relative_p95(self):
        chest = [np.array([1., 0., 0., 0.])] * 100
        angles = np.linspace(0.0, np.radians(70.0), 100)
        forearm = [_axis_quaternion([1., 0., 0.], angle)
                   for angle in angles]
        # 单帧异常值不应成为动作范围结果。
        forearm[50] = _axis_quaternion([0., 1., 0.], np.radians(170.0))
        result = DualArmViz._robust_relative_motion_range(chest, forearm)
        self.assertGreater(result, 60.0)
        self.assertLess(result, 80.0)

    def test_each_arm_has_seven_steps_with_two_palm_endpoints(self):
        viz = DualArmViz.__new__(DualArmViz)
        viz.active_arm = "right"
        for side in ("right", "left"):
            viz._calib_arm = _ArmState(side)
            start, end = viz._get_calib_arm_poses()
            self.assertEqual(end - start + 1, 7)
            self.assertEqual(
                [pose["kind"] for pose in GUIDED_CALIB_POSES[start:end + 1]],
                ["static", "static", "static", "hinge",
                 "palm_up", "palm_down", "validation"])

    def test_palm_poses_use_the_forward_extended_arm(self):
        """翻掌两步必须用前平举，大臂与小臂共线。

        前平举时肩的内外旋轴与小臂指向共线，操作者借肩关节补足翻掌角度
        不会让小臂偏离正前方；换成"肘 90°、小臂向前"则两轴垂直，借来的肩
        部旋转会直接把小臂甩向侧面，成为 palm_down_forearm_deg 的主要来源。
        """
        palm_poses = [pose for pose in GUIDED_CALIB_POSES
                      if pose["kind"] in ("palm_up", "palm_down")]
        self.assertEqual(len(palm_poses), 4)        # 每臂两个端点
        for pose in palm_poses:
            with self.subTest(pose=pose["name"]):
                upper = np.asarray(pose["upper_dir"], dtype=float)
                forearm = np.asarray(pose["forearm_dir"], dtype=float)
                upper /= np.linalg.norm(upper)
                forearm /= np.linalg.norm(forearm)
                self.assertAlmostEqual(float(np.dot(upper, forearm)), 1.0,
                                       places=6)

    def test_palm_endpoints_are_opposite(self):
        """朝上/朝下必须是相反的目标方向，否则求不出掌心轴。"""
        for side in ("right", "left"):
            poses = {pose["kind"]: pose for pose in GUIDED_CALIB_POSES
                     if pose.get("side") == side
                     and pose["kind"] in ("palm_up", "palm_down")}
            up = np.asarray(poses["palm_up"]["palm_dir"], dtype=float)
            down = np.asarray(poses["palm_down"]["palm_dir"], dtype=float)
            self.assertAlmostEqual(float(np.dot(up, down)), -1.0, places=6)

    def test_validation_pose_differs_from_every_training_pose(self):
        """第 7 步是独立验证，不能和任何训练姿势重合。"""
        for side in ("right", "left"):
            poses = [pose for pose in GUIDED_CALIB_POSES
                     if pose.get("side") == side]
            validation = [p for p in poses if p["kind"] == "validation"][0]
            trained = [p for p in poses
                       if p["kind"] in ("static", "palm_up", "palm_down")]
            target = (tuple(validation["upper_dir"]),
                      tuple(validation["forearm_dir"]))
            for pose in trained:
                with self.subTest(side=side, pose=pose["name"]):
                    self.assertNotEqual(
                        (tuple(pose["upper_dir"]),
                         tuple(pose["forearm_dir"])), target)

    def test_pose_range_follows_calib_arm_not_display_focus(self):
        """标定途中切换显示焦点不能让步骤索引漂到另一条臂。"""
        viz = DualArmViz.__new__(DualArmViz)
        viz._calib_arm = _ArmState("right")
        viz.active_arm = "left"
        self.assertEqual(viz._get_calib_arm_poses(), (0, 6))

        viz._calib_arm = _ArmState("left")
        viz.active_arm = "right"
        self.assertEqual(viz._get_calib_arm_poses(), (7, 13))

    def test_no_calib_arm_falls_back_to_active_arm(self):
        viz = DualArmViz.__new__(DualArmViz)
        viz._calib_arm = None
        viz.active_arm = "left"
        self.assertEqual(viz._get_calib_arm_poses(), (7, 13))

    def test_invalid_hinge_refinement_does_not_block_base_calibration(self):
        viz = DualArmViz.__new__(DualArmViz)
        viz._calib_step = 3
        viz._calib_arm = _ArmState("right")
        identity = np.array([1., 0., 0., 0.])
        viz._calib_samples_chest = [identity.copy() for _ in range(20)]
        viz._calib_samples_upper = [identity.copy() for _ in range(20)]
        viz._calib_samples_forearm = [identity.copy() for _ in range(20)]
        viz._hinge_result = None
        viz._hinge_live_result = None
        viz._logs = []
        continued = []
        viz._continue_calibration_after_step = lambda: continued.append(True)

        viz._advance_calibration()

        self.assertIsNotNone(viz._hinge_result)
        self.assertFalse(viz._hinge_result.valid)
        self.assertEqual(continued, [True])
        self.assertTrue(any("继续使用三姿势" in entry for entry in viz._logs))

    @staticmethod
    def _calibration_viz(chain, calib_side, computed_result):
        """构造一个只带标定链所需状态的最小 DualArmViz。"""
        viz = DualArmViz.__new__(DualArmViz)
        viz.right = _ArmState("right")
        viz.left = _ArmState("left")
        viz.active_arm = calib_side
        viz._calib_arm = viz.right if calib_side == "right" else viz.left
        viz._calib_chain = chain
        viz._calib_step = 6 if calib_side == "right" else 13
        viz._calib_waiting = False
        viz._calib_previous = None
        viz._logs = []
        viz.chest_connected = True
        for arm in (viz.right, viz.left):
            arm.upper_connected = True
            arm.forearm_connected = True
        viz._compute_calibration = (
            lambda arm, start, end: computed_result)
        return viz

    def test_right_arm_success_chains_into_left_arm(self):
        viz = self._calibration_viz("chain", "right", True)
        viz._calib_chain = True

        viz._continue_calibration_after_step()

        self.assertIs(viz._calib_arm, viz.left)
        self.assertEqual(viz._calib_step, 7)
        self.assertTrue(viz._calib_waiting)
        self.assertTrue(viz._calib_chain)
        self.assertEqual(viz.active_arm, "left")
        self.assertTrue(any("现在换左臂" in entry for entry in viz._logs))

    def test_rejected_right_arm_terminates_the_chain(self):
        viz = self._calibration_viz(True, "right", False)

        viz._continue_calibration_after_step()

        self.assertIs(viz._calib_arm, viz.right)
        self.assertEqual(viz._calib_step, -1)
        self.assertFalse(viz._calib_chain)
        self.assertTrue(any("已终止" in entry for entry in viz._logs))

    def test_left_arm_completion_ends_the_chain(self):
        viz = self._calibration_viz(True, "left", True)

        viz._continue_calibration_after_step()

        self.assertEqual(viz._calib_step, -1)
        self.assertFalse(viz._calib_chain)
        self.assertTrue(any("全部完成" in entry for entry in viz._logs))

    def test_single_arm_calibration_does_not_switch_arms(self):
        viz = self._calibration_viz(False, "right", True)

        viz._continue_calibration_after_step()

        self.assertIs(viz._calib_arm, viz.right)
        self.assertEqual(viz._calib_step, -1)
        self.assertFalse(viz._calib_chain)

    def test_chain_calibration_starts_from_the_right_arm(self):
        viz = self._calibration_viz(False, "left", True)
        viz._calib_arm = None
        viz._calib_step = -1
        viz.right_arm_two_imu = False
        viz.right_forearm_only = False

        viz.start_calibration(chain=True)

        self.assertTrue(viz._calib_chain)
        self.assertIs(viz._calib_arm, viz.right)
        self.assertEqual(viz.active_arm, "right")
        self.assertEqual(viz._calib_step, 0)

    def test_shift_c_calibrates_only_the_active_arm(self):
        viz = self._calibration_viz(False, "left", True)
        viz._calib_arm = None
        viz._calib_step = -1
        viz.right_arm_two_imu = False
        viz.right_forearm_only = False

        viz.start_calibration(chain=False)

        self.assertFalse(viz._calib_chain)
        self.assertIs(viz._calib_arm, viz.left)
        self.assertEqual(viz._calib_step, 7)

    @staticmethod
    def _palm_preview_viz(pose_count=3, sample_count=40):
        """构造一个停在第 6 步（掌心朝下）采集中的最小 DualArmViz。"""
        from arm_calibration import average_relative_quaternions

        viz = DualArmViz.__new__(DualArmViz)
        viz._logs = []
        viz._palm_live_consistency_deg = None
        viz._palm_preview_calibrator = None
        viz._calib_last_quality_update = 0.0

        # 三个静态姿势对应 R_align=I、arm_body_dir=[-1,0,0] 的自洽解。
        static = [
            (_axis_quaternion([0., 1., 0.], -np.pi / 2), [0., 0., -1.]),
            (_axis_quaternion([0., 0., 1.], np.pi), [1., 0., 0.]),
            (_axis_quaternion([0., 0., 1.], -np.pi / 2), [0., 1., 0.]),
        ]
        viz._pose_averages = [
            {"q_rel_forearm": q, "forearm_dir": np.array(d)}
            for q, d in static[:pose_count]
        ]
        viz._palm_pose_results = {
            "palm_up": {"q_rel_forearm": _axis_quaternion(
                [1., 0., 0.], np.pi)},
        }
        identity = np.array([1., 0., 0., 0.])
        q_down = _axis_quaternion([1., 0.05, 0.03], np.radians(9.0))
        viz._calib_samples_chest = [identity.copy()
                                    for _ in range(sample_count)]
        viz._calib_samples_forearm = [q_down.copy()
                                      for _ in range(sample_count)]
        viz._expected_q_rel_down = average_relative_quaternions(
            viz._calib_samples_chest, viz._calib_samples_forearm)
        return viz

    def test_palm_down_step_exposes_live_consistency(self):
        viz = self._palm_preview_viz()

        viz._update_palm_live_consistency(now=10.0)

        self.assertIsNotNone(viz._palm_live_consistency_deg)
        self.assertIsNotNone(viz._palm_preview_calibrator)
        # 预览必须与最终判定使用的函数得出同一个数。
        expected = palm_pose_consistency_deg(
            viz._palm_pose_results["palm_up"]["q_rel_forearm"],
            viz._expected_q_rel_down,
            viz._palm_preview_calibrator)
        self.assertAlmostEqual(
            viz._palm_live_consistency_deg, expected, places=9)

    def test_palm_preview_is_throttled(self):
        viz = self._palm_preview_viz()
        viz._calib_last_quality_update = 10.0

        viz._update_palm_live_consistency(now=10.1)

        self.assertIsNone(viz._palm_live_consistency_deg)

    def test_palm_preview_needs_all_three_static_poses(self):
        viz = self._palm_preview_viz(pose_count=2)

        viz._update_palm_live_consistency(now=10.0)

        self.assertIsNone(viz._palm_live_consistency_deg)

    def test_palm_preview_waits_for_enough_samples(self):
        viz = self._palm_preview_viz(sample_count=3)

        viz._update_palm_live_consistency(now=10.0)

        self.assertIsNone(viz._palm_live_consistency_deg)

    def _palm_advance_viz(self, q_rel_down_source):
        """把 _palm_preview_viz 补全到可以跑 _advance_calibration。"""
        viz = self._palm_preview_viz()
        viz._calib_step = 12  # 左臂 palm_down
        viz._calib_arm = _ArmState("left")
        viz._calib_samples_forearm = [np.array(q_rel_down_source, dtype=float)
                                      for _ in range(40)]
        viz._calib_samples_upper = [np.array([1., 0., 0., 0.])
                                    for _ in range(40)]
        viz._calib_waiting = False
        viz._calib_start = 0.0
        viz._calib_stability_window = []
        viz._calib_stable_since = None
        viz._hinge_live_result = None
        viz._continued = []
        viz._continue_calibration_after_step = (
            lambda: viz._continued.append(True))
        return viz

    def test_bad_palm_flip_retries_the_step_instead_of_advancing(self):
        # 与 palm_up 端点完全相同 → 掌心轴夹角 180°，必然超限。
        viz = self._palm_advance_viz(_axis_quaternion([1., 0., 0.], np.pi))

        viz._advance_calibration()

        self.assertEqual(viz._continued, [])
        self.assertNotIn("palm_down", viz._palm_pose_results)
        self.assertTrue(viz._calib_waiting)
        self.assertTrue(any("翻掌一致性" in entry for entry in viz._logs))

    def test_good_palm_flip_is_accepted_and_advances(self):
        viz = self._palm_advance_viz(
            _axis_quaternion([1., 0.05, 0.03], np.radians(9.0)))

        viz._advance_calibration()

        self.assertEqual(viz._continued, [True])
        self.assertIn("palm_down", viz._palm_pose_results)

    def test_retry_clears_the_stale_palm_preview(self):
        viz = self._palm_preview_viz()
        viz._update_palm_live_consistency(now=10.0)
        self.assertIsNotNone(viz._palm_live_consistency_deg)
        viz._calib_samples_upper = []
        viz._calib_stability_window = []
        viz._calib_stable_since = None
        viz._hinge_live_result = None
        viz._calib_waiting = False
        viz._calib_start = 1.0

        viz._retry_calib_step("测试重试")

        self.assertIsNone(viz._palm_live_consistency_deg)

    @staticmethod
    def _waist_viz(enabled=True, captured=True):
        import threading
        from waist_tracker import WaistTracker

        viz = DualArmViz.__new__(DualArmViz)
        viz.lock = threading.Lock()
        viz._logs = []
        viz.chest_connected = True
        viz.waist_enabled = enabled
        viz.waist = WaistTracker(max_speed_rad_s=1000.0)
        viz.q_chest = np.array([1., 0., 0., 0.])
        if captured:
            viz.waist.capture_zero(viz.q_chest)
        return viz

    def test_waist_snapshot_follows_when_enabled(self):
        viz = self._waist_viz()
        viz.q_chest = _axis_quaternion([0., 0., 1.], np.radians(30.0))

        yaw, active = viz._waist_snapshot(dt=0.02, mode=3)

        self.assertTrue(active)
        self.assertAlmostEqual(np.degrees(yaw), 30.0, places=4)

    def test_waist_relaxes_to_zero_when_arms_are_paused(self):
        """mode=0 表示操作者暂停或已超时，腰不该继续跟着人转。"""
        viz = self._waist_viz()
        viz.q_chest = _axis_quaternion([0., 0., 1.], np.radians(30.0))
        viz._waist_snapshot(dt=0.02, mode=3)

        yaw, active = viz._waist_snapshot(dt=0.02, mode=0)

        self.assertFalse(active)
        self.assertAlmostEqual(yaw, 0.0, places=6)

    def test_waist_is_inactive_until_the_zero_is_captured(self):
        viz = self._waist_viz(captured=False)
        viz.q_chest = _axis_quaternion([0., 0., 1.], np.radians(30.0))

        yaw, active = viz._waist_snapshot(dt=0.02, mode=3)

        self.assertFalse(active)
        self.assertEqual(yaw, 0.0)

    def test_waist_switch_off_reports_inactive(self):
        viz = self._waist_viz(enabled=False)
        _yaw, active = viz._waist_snapshot(dt=0.02, mode=3)
        self.assertFalse(active)

    def test_waist_needs_the_imu_online(self):
        viz = self._waist_viz(captured=False)
        viz.chest_connected = False

        self.assertFalse(viz.capture_waist_zero())
        self.assertFalse(viz.waist.calibrated)
        self.assertTrue(any("未连接" in entry for entry in viz._logs))

    def test_first_calibration_step_captures_the_waist_zero(self):
        viz = self._calibration_viz(False, "right", True)
        viz._calib_step = 0  # 右臂第一步：自然下垂
        viz._calib_samples_chest = [np.array([1., 0., 0., 0.])] * 20
        viz._calib_samples_upper = [np.array([1., 0., 0., 0.])] * 20
        viz._calib_samples_forearm = [np.array([1., 0., 0., 0.])] * 20
        viz._pose_averages = []
        viz._palm_pose_results = {}
        viz._calib_stability_deg = 0.0
        viz.q_chest = _axis_quaternion([0., 0., 1.], 0.3)
        viz.chest_connected = True
        from waist_tracker import WaistTracker
        viz.waist = WaistTracker()
        viz._continue_calibration_after_step = lambda: None

        viz._advance_calibration()

        self.assertTrue(viz.waist.calibrated)
        np.testing.assert_allclose(viz.waist.q_zero, viz.q_chest, atol=1e-9)

    @staticmethod
    def _follow_viz(target=None):
        viz = DualArmViz.__new__(DualArmViz)
        viz.right = _ArmState("right")
        viz.left = _ArmState("left")
        viz.active_arm = "right"
        viz.right_diagnostic_mode = False
        viz.right_forearm_only = False
        viz.right_arm_two_imu = False
        viz._logs = []
        viz.chest_connected = True
        viz._udp_target = target
        viz.robot_discovered = False
        viz.robot_ip = None
        viz.robot_port = 9527
        for arm in (viz.right, viz.left):
            arm.upper_connected = True
            arm.forearm_connected = True
            arm.upper_calibrated = True
            arm.forearm_calibrated = True
        return viz

    def test_dual_follow_enables_both_arms_when_ready(self):
        viz = self._follow_viz()

        viz.toggle_dual_follow()

        self.assertTrue(viz.right.following)
        self.assertTrue(viz.left.following)

    def test_dual_follow_toggles_both_arms_off(self):
        viz = self._follow_viz()
        viz.right.following = True
        viz.left.following = True

        viz.toggle_dual_follow()

        self.assertFalse(viz.right.following)
        self.assertFalse(viz.left.following)

    def test_dual_follow_starts_only_the_ready_arm(self):
        viz = self._follow_viz()
        viz.left.upper_calibrated = False

        viz.toggle_dual_follow()

        self.assertTrue(viz.right.following)
        self.assertFalse(viz.left.following)
        self.assertTrue(any("左臂尚未标定" in entry for entry in viz._logs))

    def test_dual_follow_blocks_simulation_only_arm_on_real_robot(self):
        viz = self._follow_viz(target=("192.168.3.85", 9527))
        viz.left.calibration_quality = {"safe_for_robot": False}

        viz.toggle_dual_follow()

        self.assertTrue(viz.right.following)
        self.assertFalse(viz.left.following)
        self.assertTrue(any("仅供仿真" in entry for entry in viz._logs))

    def test_dual_follow_is_refused_in_diagnostic_mode(self):
        viz = self._follow_viz()
        viz.right_diagnostic_mode = True

        viz.toggle_dual_follow()

        self.assertFalse(viz.right.following)
        self.assertFalse(viz.left.following)

    def test_command_snapshot_reports_mode_three_for_dual_follow(self):
        import threading

        viz = self._follow_viz()
        viz.lock = threading.Lock()
        viz.right.following = True
        viz.left.following = True

        _, mode, positions = viz._command_snapshot()

        self.assertEqual(mode, 3)
        self.assertEqual(len(positions), 10)

    def test_original_response_mode_is_smoother_than_fast_mode(self):
        previous = np.array([1., 0., 0.])
        current = np.array([np.cos(np.radians(5.0)),
                            np.sin(np.radians(5.0)), 0.])
        original = _ArmState("right", response_mode="original")
        fast = _ArmState("right", response_mode="fast")

        original_result, _, _ = original._one_euro_direction_filter(
            previous, previous, current, 0.0, 0.01)
        fast_result, _, _ = fast._one_euro_direction_filter(
            previous, previous, current, 0.0, 0.01)

        original_motion = np.degrees(np.arccos(np.clip(
            np.dot(previous, original_result), -1.0, 1.0)))
        fast_motion = np.degrees(np.arccos(np.clip(
            np.dot(previous, fast_result), -1.0, 1.0)))
        self.assertLess(original_motion, fast_motion)

    def test_session_logger_streams_unique_packets_and_control(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            logger = SessionDataLogger(temp_dir)
            device = SimpleNamespace(
                node_id="123456", device_id="ABC", connected=True,
                seq=10, sensor_timestamp_us=123000,
                receive_monotonic=10.0,
                quat=[1.0, 0.0, 0.0, 0.0],
                gyro_dps=[1.0, 2.0, 3.0], rest=False,
                pkt_rate_hz=225.0, battery_voltage=4.0,
                battery_percent=80)
            logger.log_imu_devices(
                [device], {"123456": "right_forearm"}, 10.002)
            logger.log_imu_devices(
                [device], {"123456": "right_forearm"}, 10.003)
            device.seq = 11
            logger.log_imu_devices(
                [device], {"123456": "right_forearm"}, 10.006)

            app = SimpleNamespace(
                sample_rate=100.0, _udp_send_rate=100.0,
                _calib_step=-1, active_arm="right",
                _ik_avg_ms=0.5, _ik_p95_ms=0.8,
                q_chest=np.array([1., 0., 0., 0.]),
                right=_ArmState("right"),
                left=_ArmState("left"))
            logger.log_control(time.monotonic() + 1.0, app)
            logger.log_event("测试事件")
            session_dir = logger.close()

            with open(
                    os.path.join(session_dir, "imu_packets.csv"),
                    newline="", encoding="utf-8") as imu_file:
                imu_rows = list(csv.DictReader(imu_file))
            with open(
                    os.path.join(session_dir, "control.csv"),
                    newline="", encoding="utf-8") as control_file:
                control_rows = list(csv.DictReader(control_file))
            with open(
                    os.path.join(session_dir, "metadata.json"),
                    encoding="utf-8") as metadata_file:
                metadata = json.load(metadata_file)

            self.assertEqual(len(imu_rows), 2)
            self.assertEqual(imu_rows[0]["role"], "right_forearm")
            self.assertEqual(len(control_rows), 1)
            self.assertEqual(metadata["imu_rows"], 2)
            self.assertEqual(metadata["control_rows"], 1)
            self.assertEqual(metadata["dropped_log_records"], 0)


if __name__ == "__main__":
    unittest.main()
