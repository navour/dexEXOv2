"""拇指指腹位置重定向的回归测试。

分三类:
  * 运动学常数必须和 hand_mapping / URDF 一致 —— 抄错一个倍率, 仿真画的拇指
    和求解器解的拇指就是两只不同的手, 而且不会有任何报错。
  * 求解器本身: 可达点必须能解回原角度, 不可达点必须落在边界而不是乱跳。
  * 标定映射: 三个标定姿势必须精确映射到因时的三个对应端点。
"""

import math
import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import hand_mapping as hm
from thumb_retarget import (
    GRID_N, ThumbCalibration, ThumbKinematics, ThumbSolver, export_table)


class KinematicsConsistencyTest(unittest.TestCase):
    """与 hand_mapping 的那份表必须一致, 否则两边画/解的不是同一只手。"""

    def test_joint_limits_match_hand_mapping(self):
        for side in ("right", "left"):
            k = ThumbKinematics(side)
            # hand_mapping 通道序: ..., 拇指弯曲(thumb_2), 拇指对掌(thumb_1)
            self.assertAlmostEqual(k.theta_max, hm.JOINT_UPPER[4], places=6)
            self.assertAlmostEqual(k.beta_max, hm.JOINT_UPPER[5], places=6)

    def test_mimic_chain_matches_expand_mimic(self):
        """thumb_3/thumb_4 的倍率是逐级相乘, 两处必须给出同样的角度。"""
        for side in ("right", "left"):
            k = ThumbKinematics(side)
            theta = 0.31
            closure = [0.0] * 6
            closure[4] = theta / k.theta_max
            angles = hm.expand_mimic(
                hm.closure_to_angles(closure, side=side), side=side)
            self.assertAlmostEqual(angles[f"{side}_thumb_2_joint"], theta,
                                   places=6)
            self.assertAlmostEqual(angles[f"{side}_thumb_3_joint"],
                                   k.mimic[1] * theta, places=6)
            self.assertAlmostEqual(angles[f"{side}_thumb_4_joint"],
                                   k.mimic[2] * theta, places=6)

    def test_both_hands_are_mirror_sized(self):
        """左右手是镜像, 工作空间尺寸必须一样大。"""
        spans = []
        for side in ("right", "left"):
            flat = ThumbSolver(side=side).grid.reshape(-1, 3)
            spans.append(tuple(float(flat[:, k].max() - flat[:, k].min())
                               for k in range(3)))
        # 镜像是靠 URDF 里各自写死的常数实现的, 末位有浮点差; 0.01mm 以内
        # 就认为是同一只手的镜像。
        for got, want in zip(*spans):
            self.assertAlmostEqual(got, want, delta=1e-5)


class WorkspaceTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.solver = ThumbSolver(side="right")

    def test_workspace_is_a_real_surface(self):
        """两个自由度必须张成一个面; 退化成线的话位置重定向无解。"""
        flat = self.solver.grid.reshape(-1, 3)
        centred = flat - flat.mean(axis=0)
        sv = np.linalg.svd(centred, compute_uv=False)
        self.assertGreater(sv[1] / sv[0], 0.15)

    def test_no_folding(self):
        """网格不许折叠, 否则同一位置有多组解, 最近邻会来回跳。"""
        grid = self.solver.grid
        n = grid.shape[0]
        smallest = min(
            float(np.linalg.norm(grid[i, j] - grid[i + di, j + dj]))
            for i in range(n) for j in range(n)
            for di, dj in ((1, 0), (0, 1))
            if i + di < n and j + dj < n)
        self.assertGreater(smallest, 1e-5)

    def test_axes_are_better_conditioned_than_rotation_space(self):
        """位置空间的两轴夹角必须明显优于旋转特征空间实测的 31.5 度。

        这是整个方案 C 成立的前提: 换个空间做分解, 条件数才变好。
        """
        k = self.solver.k
        origin = k.pad_position(0.0, 0.0)
        opp = k.pad_position(k.beta_max, 0.0) - origin
        flex = k.pad_position(0.0, k.theta_max) - origin
        cosine = abs(float(opp @ flex)) / (np.linalg.norm(opp)
                                           * np.linalg.norm(flex))
        self.assertLess(cosine, 0.60)
        self.assertGreater(math.degrees(math.acos(cosine)), 45.0)


class SolverTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.solver = ThumbSolver(side="right")

    def test_reachable_points_round_trip(self):
        k = self.solver.k
        for beta_frac, theta_frac in ((0.0, 0.0), (1.0, 0.0), (0.0, 1.0),
                                      (1.0, 1.0), (0.37, 0.62), (0.8, 0.2)):
            beta = beta_frac * k.beta_max
            theta = theta_frac * k.theta_max
            target = k.pad_position(beta, theta)
            got_beta, got_theta, residual = self.solver.solve(target)
            self.assertLess(residual, 5e-4, (beta_frac, theta_frac))
            self.assertAlmostEqual(got_beta, beta, delta=0.02)
            self.assertAlmostEqual(got_theta, theta, delta=0.02)

    def test_unreachable_target_lands_on_boundary(self):
        """够不到的目标要顶在可达边界上, 不能给出界外角度。"""
        k = self.solver.k
        far = k.pad_position(k.beta_max, k.theta_max) + np.array([1.0, 1.0, 1.0])
        beta, theta, residual = self.solver.solve(far)
        self.assertTrue(0.0 <= beta <= k.beta_max)
        self.assertTrue(0.0 <= theta <= k.theta_max)
        self.assertGreater(residual, 0.1)

    def test_rejects_bad_input(self):
        for bad in (np.array([float("nan")] * 3),
                    np.array([float("inf"), 0.0, 0.0])):
            beta, theta, residual = self.solver.solve(bad)
            self.assertEqual((beta, theta), (0.0, 0.0))
            self.assertEqual(residual, float("inf"))


def _human_poses():
    """造一组人手标定点: 张开在原点, 两条轴不正交(真人就是这样)。"""
    open_pos = np.array([0.02, -0.01, 0.05])
    opp_pos = open_pos + np.array([0.05, 0.03, 0.0])
    flex_pos = open_pos + np.array([0.03, -0.04, 0.02])
    return open_pos, flex_pos, opp_pos


class CalibrationTest(unittest.TestCase):

    def setUp(self):
        self.open_pos, self.flex_pos, self.opp_pos = _human_poses()
        self.cal = ThumbCalibration(self.open_pos, self.flex_pos, self.opp_pos,
                                    side="right")

    def test_open_pose_maps_to_fully_open(self):
        flexion, opposition = self.cal.retarget(self.open_pos)
        self.assertAlmostEqual(flexion, 0.0, places=3)
        self.assertAlmostEqual(opposition, 0.0, places=3)

    def test_pure_flex_pose_maps_to_pure_flexion(self):
        flexion, opposition = self.cal.retarget(self.flex_pos)
        self.assertGreater(flexion, 0.97)
        self.assertLess(opposition, 0.03)

    def test_pure_opposition_pose_maps_to_pure_opposition(self):
        flexion, opposition = self.cal.retarget(self.opp_pos)
        self.assertGreater(opposition, 0.97)
        self.assertLess(flexion, 0.03)

    def test_out_of_plane_motion_is_discarded(self):
        """因时拇指做不出平面外的动作; 塞进去不该让两个通道乱动。"""
        normal = np.cross(self.cal.opp_axis, self.cal.flex_axis)
        normal = normal / np.linalg.norm(normal)
        base = self.cal.retarget(self.opp_pos)
        moved = self.cal.retarget(self.opp_pos + normal * 0.02)
        for a, b in zip(base, moved):
            self.assertAlmostEqual(a, b, places=3)

    def test_beyond_calibrated_range_saturates(self):
        """人手比因时能做的大; 超出部分顶到边界, 不许回绕。"""
        beyond = self.open_pos + (self.opp_pos - self.open_pos) * 2.0
        flexion, opposition = self.cal.retarget(beyond)
        self.assertLessEqual(opposition, 1.0)
        self.assertGreaterEqual(opposition, 0.9)
        self.assertTrue(0.0 <= flexion <= 1.0)

    def test_outputs_are_always_in_range(self):
        rng = np.random.default_rng(20260803)
        for _ in range(200):
            point = self.open_pos + rng.normal(scale=0.06, size=3)
            flexion, opposition = self.cal.retarget(point)
            self.assertTrue(0.0 <= flexion <= 1.0)
            self.assertTrue(0.0 <= opposition <= 1.0)

    def test_conditioning_is_reported(self):
        self.assertGreater(self.cal.conditioning_deg, 0.0)
        self.assertLess(self.cal.conditioning_deg, 90.0)


class ExportTest(unittest.TestCase):
    """C++ 加载的那张表是生成物, 必须能从 URDF 一字不差地重新生成。"""

    def test_export_matches_kinematics(self):
        with tempfile.TemporaryDirectory() as folder:
            path = export_table(os.path.join(folder, "t.txt"), side="right",
                                n=9)
            with open(path, encoding="utf-8") as handle:
                rows = [l for l in handle.read().splitlines()
                        if not l.startswith("#")]
            header = rows[0].split()
            self.assertEqual(int(header[0]), 9)
            k = ThumbKinematics("right")
            self.assertAlmostEqual(float(header[1]), k.beta_max, places=9)
            self.assertAlmostEqual(float(header[2]), k.theta_max, places=9)
            self.assertEqual(len(rows) - 1, 81)
            for row in rows[1:]:
                beta, theta, x, y, z = (float(v) for v in row.split())
                expected = k.pad_position(beta, theta)
                for got, want in zip((x, y, z), expected):
                    self.assertAlmostEqual(got, want, places=9)


class GraspGeometryTest(unittest.TestCase):
    """抓握几何 —— 标定第5步"只缩放对掌、不缩放弯曲"就建立在这上面。

    这些数来自 URDF, 所以它们是可验证的事实, 不是调参得到的经验。
    """

    @classmethod
    def setUpClass(cls):
        from thumb_retarget import FingerKinematics
        cls.thumb = ThumbKinematics("right")
        cls.index = FingerKinematics("index", "right")

    def _gap(self, finger_closure, flexion, opposition):
        thumb_pad = self.thumb.pad_position(opposition * self.thumb.beta_max,
                                            flexion * self.thumb.theta_max)
        return float(np.linalg.norm(thumb_pad
                                    - self.index.pad_position(finger_closure)))

    def test_opposition_must_reach_full_travel(self):
        """抓握时对掌要走满 —— 半程的话拇指根本过不到四指那边。

        这就是第5步存在的理由: 把 1.0 定在人手极限上, 日常抓握只到 0.5 左右。
        """
        self.assertLess(self._gap(0.5, 0.5, 1.0), self._gap(0.5, 0.5, 0.5))
        self.assertLess(self._gap(0.5, 0.5, 1.0), 0.030)

    def test_over_flexing_the_thumb_misses_the_fingers(self):
        """弯曲**不能**跟着缩放到抓握位=1.0: 弯过头会从四指旁边错过去。"""
        self.assertLess(self._gap(0.5, 0.5, 1.0), self._gap(0.5, 1.0, 1.0))

    def test_full_fist_puts_fingertips_out_of_reach(self):
        """四指蜷到底时指尖贴掌心, 拇指够不到 —— 真手也是这样, 不是缺陷。

        看到"仿真里四指弯得很深、拇指搭不上"时, 先怀疑四指闭合度饱和。
        """
        self.assertGreater(self._gap(1.0, 0.5, 1.0), self._gap(0.5, 0.5, 1.0))
        self.assertGreater(self._gap(1.0, 1.0, 1.0), 0.050)


class GestureTargetTest(unittest.TestCase):
    """三个标定手势对应的因时姿态必须是从几何解出来的, 不是拍脑袋定的。

    C++ 侧 kThumbPalmTarget / kThumbPinchTarget 抄的是这里的数; 这几条测试是
    那两个常量唯一的推导依据。
    """

    @classmethod
    def setUpClass(cls):
        from thumb_retarget import FingerKinematics, GESTURE_TARGETS
        cls.thumb = ThumbKinematics("right")
        cls.index = FingerKinematics("index", "right")
        cls.targets = GESTURE_TARGETS

    def _gap(self, finger_closure, flexion, opposition):
        pad = self.thumb.pad_position(opposition * self.thumb.beta_max,
                                      flexion * self.thumb.theta_max)
        return float(np.linalg.norm(pad
                                    - self.index.pad_position(finger_closure)))

    def test_thumbs_up_is_the_thumb_zero(self):
        self.assertEqual(self.targets["比赞"], (0.0, 0.0))

    def test_palm_sweep_is_full_opposition_without_flexion(self):
        self.assertEqual(self.targets["横贴掌心"], (1.0, 0.0))

    def test_ok_target_is_the_actual_pinch_optimum(self):
        """OK 的目标必须真的能让两个指腹碰上, 且是扫描出来的最优解。"""
        best = (1e9, None)
        for finger in np.linspace(0, 1, 41):
            for flexion in np.linspace(0, 1, 41):
                for opposition in np.linspace(0, 1, 41):
                    gap = self._gap(finger, flexion, opposition)
                    if gap < best[0]:
                        best = (gap, (flexion, opposition))
        gap, (flexion, opposition) = best
        # 指腹真的碰得上(个位数 mm), 否则这个手势就不该当锚点
        self.assertLess(gap, 0.010)
        want_u, want_v = self.targets["OK"]
        self.assertAlmostEqual(opposition, want_u, delta=0.05)
        self.assertAlmostEqual(flexion, want_v, delta=0.05)

    def test_anchors_are_well_separated(self):
        """两条基底必须分得开, 否则二维分解重新变病态。"""
        origin = self.thumb.pad_position(0.0, 0.0)
        palm_u, palm_v = self.targets["横贴掌心"]
        pinch_u, pinch_v = self.targets["OK"]
        a = self.thumb.pad_position(palm_u * self.thumb.beta_max,
                                    palm_v * self.thumb.theta_max) - origin
        b = self.thumb.pad_position(pinch_u * self.thumb.beta_max,
                                    pinch_v * self.thumb.theta_max) - origin
        cosine = abs(float(a @ b)) / (np.linalg.norm(a) * np.linalg.norm(b))
        self.assertLess(cosine, 0.95)
        self.assertGreater(np.linalg.norm(a), 0.02)
        self.assertGreater(np.linalg.norm(b), 0.02)


class UvEntryPointTest(unittest.TestCase):
    """手套那端只送归一化 (u,v), 这一半必须自成闭环。"""

    @classmethod
    def setUpClass(cls):
        cls.solver = ThumbSolver(side="right")

    def test_corners_map_to_channel_ends(self):
        self.assertEqual(
            tuple(round(x, 3) for x in self.solver.retarget_uv(0.0, 0.0)),
            (0.0, 0.0))
        flexion, opposition = self.solver.retarget_uv(0.0, 1.0)
        self.assertGreater(flexion, 0.97)
        self.assertLess(opposition, 0.03)
        flexion, opposition = self.solver.retarget_uv(1.0, 0.0)
        self.assertGreater(opposition, 0.97)
        self.assertLess(flexion, 0.03)

    def test_matches_the_position_entry_point(self):
        """两个入口必须给出同一个结果, 否则就是两套实现了。"""
        open_pos, flex_pos, opp_pos = _human_poses()
        cal = ThumbCalibration(open_pos, flex_pos, opp_pos,
                               solver=self.solver)
        probe = open_pos + 0.4 * (opp_pos - open_pos) + 0.3 * (flex_pos
                                                               - open_pos)
        u, v = cal.normalized(probe)
        for a, b in zip(cal.retarget(probe), self.solver.retarget_uv(u, v)):
            self.assertAlmostEqual(a, b, places=9)

    def test_rejects_non_finite(self):
        self.assertIsNone(self.solver.retarget_uv(float("nan"), 0.0))
        self.assertIsNone(self.solver.retarget_uv(0.0, float("inf")))

    def test_negative_uv_clamps_to_open(self):
        """人手往反方向动时不许绕到另一端。"""
        self.assertEqual(
            tuple(round(x, 3) for x in self.solver.retarget_uv(-0.5, -0.5)),
            (0.0, 0.0))

    def test_mapping_is_monotonic_and_undistorted(self):
        """曾经用三维线性插值构造目标点再投影, 中段被扭曲得很厉害
        (u=0.75,v=0.25 解出对掌 0.975/弯曲 0.233)。参数对应不许再出现这种事。
        """
        for u in (0.0, 0.25, 0.5, 0.75, 1.0):
            for v in (0.0, 0.25, 0.5, 0.75, 1.0):
                flexion, opposition = self.solver.retarget_uv(u, v)
                self.assertAlmostEqual(opposition, u, places=6)
                self.assertAlmostEqual(flexion, v, places=6)

    def test_chord_basis_leaves_the_surface(self):
        """把当初那个错误留成回归: 弦向量张成的平面确实不在曲面上。

        以后要是有人想"顺手"改回三维插值, 这条会告诉他为什么不行。
        """
        worst = 0.0
        for u in (0.25, 0.5, 0.75, 1.0):
            for v in (0.25, 0.5, 0.75, 1.0):
                target = (self.solver.q_open + u * self.solver.q_opp
                          + v * self.solver.q_flex)
                worst = max(worst, self.solver.solve(target)[2])
        self.assertGreater(worst, 0.03)


class GloveSourceThumbTest(unittest.TestCase):
    """接进 glove_source 之后的降级行为 —— 这几条直接关系到真手安全。"""

    @classmethod
    def setUpClass(cls):
        from glove_source import GloveSource, parse_ctrl
        cls.GloveSource = GloveSource
        cls.parse_ctrl = staticmethod(parse_ctrl)

    def _source(self, **kwargs):
        return self.GloveSource(open_counts=(1000,) * 6, closed_counts=(0,) * 6,
                                port=0, **kwargs)

    def test_parse_reads_thumb_uv(self):
        counts, uv = self.parse_ctrl(
            '{"type":"ctrl","angle_set":[0,0,0,0,0,0],"thumb_uv":[0.25,0.75]}')
        self.assertEqual(counts, (0,) * 6)
        self.assertEqual(uv, (0.25, 0.75))

    def test_parse_without_thumb_uv_is_none(self):
        """老手套端不带这个字段, 必须照常出闭合度。"""
        counts, uv = self.parse_ctrl(
            '{"type":"ctrl","angle_set":[1,2,3,4,5,6]}')
        self.assertEqual(counts, (1, 2, 3, 4, 5, 6))
        self.assertIsNone(uv)

    def test_bad_thumb_uv_does_not_kill_the_frame(self):
        for bad in ('"x"', '[1]', '[null,1]', '["a","b"]'):
            counts, uv = self.parse_ctrl(
                '{"type":"ctrl","angle_set":[0,0,0,0,0,0],'
                f'"thumb_uv":{bad}}}')
            self.assertEqual(counts, (0,) * 6, bad)
            self.assertIsNone(uv, bad)

    def test_retarget_overrides_only_the_thumb(self):
        source = self._source(side="right")
        try:
            if source.thumb_map is None:
                self.skipTest(f"重定向不可用: {source.thumb_error}")
            base = (0.1, 0.2, 0.3, 0.4, 0.9, 0.9)
            out = source._apply_thumb_retarget(base, (0.0, 0.0))
            self.assertEqual(out[:4], base[:4])
            self.assertAlmostEqual(out[4], 0.0, places=3)
            self.assertAlmostEqual(out[5], 0.0, places=3)
        finally:
            source.close()

    def test_missing_uv_falls_back_to_linear_projection(self):
        """退回的必须是原值, 不是 0 —— 送 0 等于命令拇指张开, 那是错的抓握。"""
        source = self._source(side="right")
        try:
            base = (0.1, 0.2, 0.3, 0.4, 0.55, 0.66)
            self.assertEqual(source._apply_thumb_retarget(base, None), base)
        finally:
            source.close()

    def test_solver_failure_falls_back_and_stops_retrying(self):
        source = self._source(side="right")
        try:
            def boom(*_a, **_k):
                raise RuntimeError("boom")

            source.thumb_map = boom
            base = (0.1, 0.2, 0.3, 0.4, 0.55, 0.66)
            self.assertEqual(source._apply_thumb_retarget(base, (0.5, 0.5)),
                             base)
            self.assertIsNone(source.thumb_map)
            self.assertIn("boom", source.thumb_error)
            self.assertEqual(source._apply_thumb_retarget(base, (0.5, 0.5)),
                             base)
        finally:
            source.close()

    def test_can_be_switched_off(self):
        source = self._source(side="right", thumb_retarget=False)
        try:
            self.assertIsNone(source.thumb_map)
            base = (0.1, 0.2, 0.3, 0.4, 0.55, 0.66)
            self.assertEqual(source._apply_thumb_retarget(base, (0.5, 0.5)),
                             base)
        finally:
            source.close()


if __name__ == "__main__":
    unittest.main()
