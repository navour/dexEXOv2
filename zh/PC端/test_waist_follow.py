"""机器人端 armsdk 腰部跟随判据的测试。

被测模块在 机器人端/, 按路径加载 —— 那边没有自己的测试环境, 而腰部是
直接改变上半身重心的安全相关逻辑, 必须有回归覆盖。
"""

import importlib.util
import os
import unittest


_MODULE_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "机器人端", "waist_follow.py")
_SPEC = importlib.util.spec_from_file_location("robot_waist_follow", _MODULE_PATH)
waist_follow = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(waist_follow)


DT = 0.002          # 500Hz 控制周期
RETURN_SPEED = 0.3  # 与 robot_arm_receiver.RETURN_SPEED 一致


def command(current, target, allow_waist=True, arm_following=True, dt=DT):
    return waist_follow.waist_yaw_command(
        current, target, allow_waist=allow_waist,
        arm_following=arm_following, dt=dt, return_speed=RETURN_SPEED)


class WaistGateTest(unittest.TestCase):
    def test_gate_off_ignores_a_valid_target(self):
        """真机默认走这条路：尾块有效也必须回中。"""
        value, following = command(0.5, 0.4, allow_waist=False)
        self.assertFalse(following)
        self.assertLess(value, 0.5)

    def test_mode_zero_returns_the_waist_to_centre(self):
        """松手时腰不能留在扭着的姿态。"""
        value, following = command(0.5, 0.4, arm_following=False)
        self.assertFalse(following)
        self.assertAlmostEqual(value, 0.5 - RETURN_SPEED * DT, places=9)

    def test_missing_tail_returns_the_waist_to_centre(self):
        """老发送端或坏尾块给 None，等价于没开腰。"""
        value, following = command(0.5, None)
        self.assertFalse(following)
        self.assertLess(value, 0.5)

    def test_all_three_conditions_met_follows(self):
        value, following = command(0.0, 0.4)
        self.assertTrue(following)
        self.assertAlmostEqual(
            value, waist_follow.WAIST_FOLLOW_SPEED * DT, places=9)

    def test_return_to_centre_never_overshoots(self):
        value, _ = command(1e-6, None)
        self.assertAlmostEqual(value, 0.0, places=12)


class WaistLimitTest(unittest.TestCase):
    def test_target_is_clamped_to_the_limits(self):
        """接收端不信任发送端的夹紧。"""
        upper = waist_follow.WAIST_YAW_LIMITS[1]
        value, _ = command(upper, 99.0, dt=10.0)
        self.assertAlmostEqual(value, upper, places=9)

        lower = waist_follow.WAIST_YAW_LIMITS[0]
        value, _ = command(lower, -99.0, dt=10.0)
        self.assertAlmostEqual(value, lower, places=9)

    def test_output_is_clamped_even_from_a_bad_current_value(self):
        """当前角来自电机反馈, 超限时必须被拉回而不是原样下发。"""
        value, _ = command(5.0, 0.0, arm_following=False, dt=10.0)
        self.assertLessEqual(value, waist_follow.WAIST_YAW_LIMITS[1])
        self.assertGreaterEqual(value, waist_follow.WAIST_YAW_LIMITS[0])

    def test_speed_limit_applies_to_a_large_step(self):
        value, _ = command(0.0, 1.0)
        self.assertAlmostEqual(
            value, waist_follow.WAIST_FOLLOW_SPEED * DT, places=9)

    def test_waist_is_slower_than_the_arms(self):
        """腰带整个上半身, 限速必须明显低于手臂的 5.0 rad/s。"""
        self.assertLess(waist_follow.WAIST_FOLLOW_SPEED, 5.0)

    def test_only_yaw_is_driven(self):
        """roll/pitch 与平衡耦合太强, 不能进入驱动通道。"""
        self.assertEqual(waist_follow.WAIST_YAW_JOINT, 12)
        self.assertEqual(waist_follow.WAIST_JOINTS, (12, 13, 14))

    def test_limits_are_tighter_than_the_urdf_range(self):
        """±2.618 是 URDF 限位, 接收端要收得更紧。"""
        self.assertLess(waist_follow.WAIST_YAW_LIMITS[1], 2.618)
        self.assertGreater(waist_follow.WAIST_YAW_LIMITS[0], -2.618)


class NoRegressionWhenGateOffTest(unittest.TestCase):
    """--waist 关闭时, 关节12 必须与加腰之前的通用回零循环逐位相同。

    这是"手臂性能不受影响"的可执行凭据: 默认路径上腰的算式没有任何变化。
    """

    STARTUP_MOVE_SPEED = 0.2

    @staticmethod
    def _old_zeroing_loop(current, dt, speed):
        """加腰之前 robot_arm_receiver 对 _lower_joints 做的事。"""
        max_step = speed * dt
        diff = 0.0 - current
        if abs(diff) <= max_step:
            return 0.0
        return current + max_step * (1.0 if diff > 0 else -1.0)

    def test_bit_identical_to_the_old_loop(self):
        import random

        random.seed(0)
        for _ in range(5000):
            current = random.uniform(-1.0, 1.0)
            speed = random.choice([self.STARTUP_MOVE_SPEED, RETURN_SPEED])
            # 有无尾块、各种 mode 都不应改变关闭状态下的结果。
            target = random.choice([None, random.uniform(-2.0, 2.0)])
            arm_following = random.random() < 0.5

            value, following = waist_follow.waist_yaw_command(
                current, target, allow_waist=False,
                arm_following=arm_following, dt=DT, return_speed=speed)

            self.assertFalse(following)
            self.assertEqual(
                value, self._old_zeroing_loop(current, DT, speed))


class WaistIntegrationTest(unittest.TestCase):
    def test_a_full_sweep_stays_within_limits(self):
        """扫一遍量程, 任何时刻都不越界, 且能收敛到目标。"""
        value = 0.0
        for _ in range(2000):
            value, _ = command(value, 0.9)
            self.assertLessEqual(value, waist_follow.WAIST_YAW_LIMITS[1])
        self.assertAlmostEqual(value, 0.9, places=6)

    def test_dropping_the_tail_mid_motion_returns_to_centre(self):
        """跟随中突然丢尾块, 应平滑回中而不是跳变。"""
        value = 0.0
        for _ in range(200):
            value, _ = command(value, 0.8)
        moved = value
        self.assertGreater(moved, 0.0)

        value, following = command(value, None)
        self.assertFalse(following)
        self.assertAlmostEqual(value, moved - RETURN_SPEED * DT, places=9)


if __name__ == "__main__":
    unittest.main()
