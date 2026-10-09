#!/usr/bin/env python3
"""hand_mapping 的单测。手部会直接抓东西, 方向和行程弄反的代价很高,
所以端点、方向、硬上限和 mimic 链都单独钉住。"""

import unittest

import hand_mapping as hm


# mhandpro/config/inspire_left.cfg 里逐通道实测的端点, 顺序:
# 小指, 无名指, 中指, 食指, 拇指弯曲, 拇指对掌
OPEN = (980, 965, 957, 946, 949, 922)
CLOSED = (60, 60, 70, 60, 70, 150)


class ChannelTableTest(unittest.TestCase):
    def test_six_channels_everywhere(self):
        self.assertEqual(len(hm.CHANNEL_NAMES), hm.CHANNEL_COUNT)
        self.assertEqual(len(hm.JOINT_UPPER), hm.CHANNEL_COUNT)
        self.assertEqual(len(hm.driver_joints("right")), hm.CHANNEL_COUNT)
        self.assertEqual(len(hm.driver_joints("left")), hm.CHANNEL_COUNT)

    def test_left_is_the_mirror_of_right(self):
        right = hm.driver_joints("right")
        left = hm.driver_joints("left")
        for r, l in zip(right, left):
            self.assertTrue(r.startswith("right_"))
            self.assertEqual(l, "left_" + r[len("right_"):])

    def test_every_driver_joint_has_a_chain_entry(self):
        for side in ("right", "left"):
            chains = hm.mimic_chains(side)
            for name in hm.driver_joints(side):
                self.assertIn(name, chains)

    def test_bad_side_rejected(self):
        with self.assertRaises(ValueError):
            hm.driver_joints("both")


class GloveToClosureTest(unittest.TestCase):
    def test_open_endpoint_is_zero_closure(self):
        self.assertEqual(hm.glove_to_closure(OPEN, OPEN, CLOSED),
                         (0.0,) * hm.CHANNEL_COUNT)

    def test_closed_endpoint_is_full_closure(self):
        self.assertEqual(hm.glove_to_closure(CLOSED, OPEN, CLOSED),
                         (1.0,) * hm.CHANNEL_COUNT)

    def test_direction_is_inverted(self):
        """因时是值越大越张开, 闭合度必须反过来单调。"""
        low = hm.glove_to_closure((200,) * 6, OPEN, CLOSED)
        high = hm.glove_to_closure((800,) * 6, OPEN, CLOSED)
        for a, b in zip(low, high):
            self.assertGreater(a, b)

    def test_out_of_range_is_clamped_not_extrapolated(self):
        self.assertEqual(hm.glove_to_closure((1000,) * 6, OPEN, CLOSED),
                         (0.0,) * hm.CHANNEL_COUNT)
        self.assertEqual(hm.glove_to_closure((0,) * 6, OPEN, CLOSED),
                         (1.0,) * hm.CHANNEL_COUNT)

    def test_degenerate_endpoints_open_the_hand(self):
        """端点重合的坏通道张开, 不能锁死在握紧上。"""
        closure = hm.glove_to_closure((500,) * 6, OPEN, OPEN)
        self.assertEqual(closure, (0.0,) * hm.CHANNEL_COUNT)

    def test_nan_opens_the_hand(self):
        closure = hm.glove_to_closure(
            (float("nan"), 500, 500, 500, 500, 500), OPEN, CLOSED)
        self.assertEqual(closure[0], 0.0)

    def test_wrong_channel_count_rejected(self):
        with self.assertRaises(ValueError):
            hm.glove_to_closure((0, 0, 0), OPEN, CLOSED)


class ClosureToAnglesTest(unittest.TestCase):
    def test_open_is_all_zero(self):
        self.assertEqual(hm.closure_to_angles(hm.open_closure()),
                         (0.0,) * hm.CHANNEL_COUNT)

    def test_full_closure_respects_range_scale(self):
        angles = hm.closure_to_angles((1.0,) * 6, range_scale=0.30)
        for angle, upper in zip(angles, hm.JOINT_UPPER):
            self.assertAlmostEqual(angle, 0.30 * upper)

    def test_default_simulation_uses_full_urdf_range(self):
        angles = hm.closure_to_angles((1.0,) * 6)
        for angle, upper in zip(angles, hm.JOINT_UPPER):
            self.assertAlmostEqual(angle, upper)

    def test_simulation_scale_is_capped_at_one(self):
        angles = hm.closure_to_angles((1.0,) * 6, range_scale=5.0)
        full = hm.closure_to_angles((1.0,) * 6, range_scale=1.0)
        self.assertEqual(angles, full)

    def test_negative_range_scale_freezes_open(self):
        angles = hm.closure_to_angles((1.0,) * 6, range_scale=-5.0)
        self.assertEqual(angles, (0.0,) * hm.CHANNEL_COUNT)

    def test_never_exceeds_urdf_limits(self):
        angles = hm.closure_to_angles((5.0,) * 6, range_scale=1.0)
        for angle, upper in zip(angles, hm.JOINT_UPPER):
            self.assertGreaterEqual(angle, 0.0)
            self.assertLessEqual(angle, upper)


class ClosureToCountsTest(unittest.TestCase):
    def test_open_closure_gives_the_open_endpoint(self):
        self.assertEqual(
            hm.closure_to_counts(hm.open_closure(), OPEN, CLOSED), OPEN)

    def test_full_closure_reaches_the_configured_cap(self):
        """闭合度 1.0 走到 REAL_HAND_MAX_RANGE_SCALE 定义的位置。

        钉的是机制不是那个数字: 上限从 0.30 放开到整程时这条不该改。
        """
        counts = hm.closure_to_counts((1.0,) * 6, OPEN, CLOSED,
                                      range_scale=1.0)
        for count, opened, closed in zip(counts, OPEN, CLOSED):
            self.assertLess(count, opened)      # 确实动了
            self.assertAlmostEqual(
                count,
                opened + (closed - opened) * hm.REAL_HAND_MAX_RANGE_SCALE,
                delta=1)

    def test_caller_cannot_exceed_the_cap(self):
        """无论上限设成多少, 调用方传更大的值都绕不过去。"""
        capped = hm.closure_to_counts((1.0,) * 6, OPEN, CLOSED,
                                      range_scale=5.0)
        at_cap = hm.closure_to_counts(
            (1.0,) * 6, OPEN, CLOSED,
            range_scale=hm.REAL_HAND_MAX_RANGE_SCALE)
        self.assertEqual(capped, at_cap)

    def test_a_smaller_range_scale_still_limits(self):
        """放开上限之后, 保守值必须仍然有效 —— --hand-range 靠的就是这条。"""
        conservative = hm.closure_to_counts(
            (1.0,) * 6, OPEN, CLOSED,
            range_scale=hm.CONSERVATIVE_RANGE_SCALE)
        for count, opened, closed in zip(conservative, OPEN, CLOSED):
            self.assertGreater(count, closed)   # 远没到握死
            self.assertAlmostEqual(
                count,
                opened + (closed - opened) * hm.CONSERVATIVE_RANGE_SCALE,
                delta=1)

    def test_counts_stay_in_register_range(self):
        counts = hm.closure_to_counts((1.0,) * 6, (2000,) * 6, (-500,) * 6,
                                      range_scale=1.0)
        for count in counts:
            self.assertGreaterEqual(count, hm.COUNT_MIN)
            self.assertLessEqual(count, hm.COUNT_MAX)

    def test_counts_are_ints(self):
        for count in hm.closure_to_counts((0.37,) * 6, OPEN, CLOSED):
            self.assertIsInstance(count, int)

    def test_angles_and_counts_agree_on_direction(self):
        """同一个闭合度, 角度变大时寄存器值必须变小 (越握越小)。"""
        soft = 0.2
        hard = 0.9
        angle_soft = hm.closure_to_angles((soft,) * 6)[0]
        angle_hard = hm.closure_to_angles((hard,) * 6)[0]
        count_soft = hm.closure_to_counts((soft,) * 6, OPEN, CLOSED)[0]
        count_hard = hm.closure_to_counts((hard,) * 6, OPEN, CLOSED)[0]
        self.assertGreater(angle_hard, angle_soft)
        self.assertLess(count_hard, count_soft)


class ExpandMimicTest(unittest.TestCase):
    def test_expands_six_drivers_to_twelve_joints(self):
        """6 个驱动 + 6 个联动 (四指各 1, 拇指 2, 对掌 0)。"""
        joints = hm.expand_mimic((0.1,) * 6)
        self.assertEqual(len(joints), 12)

    def test_thumb_chain_multiplies_stepwise(self):
        """thumb_3 跟 thumb_2, thumb_4 跟 thumb_3 —— 是链, 不是都乘 thumb_2。"""
        joints = hm.expand_mimic((0.0, 0.0, 0.0, 0.0, 0.5, 0.0))
        self.assertAlmostEqual(joints["right_thumb_2_joint"], 0.5)
        self.assertAlmostEqual(joints["right_thumb_3_joint"], 0.5 * 0.8024)
        self.assertAlmostEqual(joints["right_thumb_4_joint"],
                               0.5 * 0.8024 * 0.9487)

    def test_finger_chain_single_step(self):
        joints = hm.expand_mimic((0.4, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqual(joints["right_little_1_joint"], 0.4)
        self.assertAlmostEqual(joints["right_little_2_joint"], 0.4 * 1.0843)

    def test_thumb_opposition_has_no_passive_joint(self):
        joints = hm.expand_mimic((0.0, 0.0, 0.0, 0.0, 0.0, 0.3))
        self.assertAlmostEqual(joints["right_thumb_1_joint"], 0.3)
        self.assertNotIn("right_thumb_0_joint", joints)

    def test_left_side_names(self):
        joints = hm.expand_mimic((0.1,) * 6, side="left")
        self.assertIn("left_thumb_4_joint", joints)
        self.assertTrue(all(n.startswith("left_") for n in joints))


class EndToEndTest(unittest.TestCase):
    def test_natural_open_hand_stays_open(self):
        """手套报告张手位时, 关节角全零, 寄存器回到 open 端点。"""
        closure = hm.glove_to_closure(OPEN, OPEN, CLOSED)
        self.assertEqual(hm.closure_to_angles(closure),
                         (0.0,) * hm.CHANNEL_COUNT)
        self.assertEqual(hm.closure_to_counts(closure, OPEN, CLOSED), OPEN)

    def test_full_fist_reaches_full_simulation_range(self):
        closure = hm.glove_to_closure(CLOSED, OPEN, CLOSED)
        self.assertEqual(closure, (1.0,) * hm.CHANNEL_COUNT)
        joints = hm.expand_mimic(hm.closure_to_angles(closure))
        self.assertAlmostEqual(
            joints["right_little_1_joint"], hm.JOINT_UPPER[0])
        # 被动关节仍在它自己的URDF上限3.14以内。
        self.assertLess(joints["right_little_2_joint"], 3.14)


if __name__ == "__main__":
    unittest.main()
