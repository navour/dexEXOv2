#!/usr/bin/env python3
"""灵巧手回放的端到端测试：PC 打包 → 仿真解析 → 真的写到手指关节上。

手部这条链跨了三个文件（PC端/teleop_protocol.py 打包、本文件所在的仿真解析、
PC端/hand_mapping.py 换算），任何一环的通道顺序或方向反了都不会报错，只会让
仿真里的手和真手对不上。所以这里从字节一路测到 qpos 地址。
"""

import importlib.util
from pathlib import Path
import sys
import unittest

from g1_mujoco_sim import (
    HAND_CHANNEL_COUNT,
    HAND_SIZE,
    WAIST_SIZE,
    find_tail_block,
    load_hand_mapping,
    resolve_hand_addresses,
    unpack_arm_command,
    unpack_hand_block,
)


_ROOT = Path(__file__).resolve().parent.parent
_FTP_MODEL = (_ROOT / "仿真" / "models" / "g1_29dof"
              / "g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf")


def _pc_protocol():
    """按路径加载 PC 端的打包侧，确保两端对的是同一套字节。"""
    path = _ROOT / "PC端" / "teleop_protocol.py"
    spec = importlib.util.spec_from_file_location("pc_teleop_protocol", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RIGHT = (0.0, 0.2, 0.4, 0.6, 0.8, 1.0)


class HandBlockParsingTests(unittest.TestCase):
    def setUp(self):
        self.pc = _pc_protocol()

    def _packet(self, right=RIGHT, left=None, waist=True):
        return self.pc.pack_arm_command(
            3, 1, 2, (0.0,) * 10, (0.0,) * 10,
            waist=(0.3, 0.0, 0.0) if waist else None, waist_enabled=waist,
            right_hand=right, left_hand=left)

    def test_pc_packed_hand_is_read_back(self):
        command = unpack_arm_command(self._packet())
        self.assertIsNotNone(command.hands)
        for actual, expected in zip(command.hands["right"], RIGHT):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_hand_tail_does_not_disturb_waist_or_arms(self):
        command = unpack_arm_command(self._packet())
        self.assertAlmostEqual(command.waist[0], 0.3, places=6)
        self.assertEqual(command.mode, 3)
        self.assertEqual(command.right, (0.0,) * 5)

    def test_one_glove_leaves_the_other_hand_absent(self):
        command = unpack_arm_command(self._packet(right=RIGHT))
        self.assertIsNotNone(command.hands["right"])
        self.assertIsNone(command.hands["left"])

    def test_hand_block_found_without_waist_block(self):
        command = unpack_arm_command(self._packet(waist=False))
        self.assertIsNone(command.waist)
        self.assertIsNotNone(command.hands["right"])

    def test_legacy_packet_has_no_hands(self):
        command = unpack_arm_command(
            self.pc.pack_arm_command(3, 1, 2, (0.0,) * 10, (0.0,) * 10))
        self.assertIsNone(command.hands)

    def test_unknown_tail_stops_the_walk(self):
        """走到不认识的包头就停，绝不能把随机字节当成手部指令。"""
        base = self.pc.pack_arm_command(3, 1, 2, (0.0,) * 10, (0.0,) * 10)
        self.assertIsNone(unpack_hand_block(b"ZZZZ" + b"\x00" * 60))
        self.assertIsNone(unpack_arm_command(base + b"ZZZZ" * 16).hands)

    def test_truncated_hand_block_is_ignored(self):
        packet = self._packet()
        self.assertIsNone(unpack_arm_command(packet[:-8]).hands)

    def test_find_tail_block_skips_over_the_waist(self):
        packet = self._packet()
        tail = packet[len(packet) - WAIST_SIZE - HAND_SIZE:]
        block = find_tail_block(tail, b"UHND", HAND_SIZE)
        self.assertIsNotNone(block)
        self.assertEqual(block[:4], b"UHND")


class HandMappingWiringTests(unittest.TestCase):
    def setUp(self):
        self.hm = load_hand_mapping()
        if self.hm is None:
            self.skipTest("找不到 PC端/hand_mapping.py")

    def test_open_closure_is_all_zero_angles(self):
        angles = self.hm.expand_mimic(
            self.hm.closure_to_angles((0.0,) * HAND_CHANNEL_COUNT))
        for value in angles.values():
            self.assertEqual(value, 0.0)

    def test_full_closure_stays_within_urdf_limits(self):
        angles = self.hm.expand_mimic(
            self.hm.closure_to_angles((1.0,) * HAND_CHANNEL_COUNT))
        # 仿真使用完整驱动行程；联动关节乘完倍率仍不能越自身URDF上限。
        self.assertAlmostEqual(
            angles["right_little_1_joint"], self.hm.JOINT_UPPER[0])
        self.assertLess(angles["right_little_2_joint"], 3.14)
        self.assertLess(angles["right_thumb_4_joint"], 3.14)


@unittest.skipUnless(_FTP_MODEL.is_file(), "缺少因时 FTP 模型")
class HandModelTests(unittest.TestCase):
    """需要真模型：验证关节名对得上，且写进去的角度落在正确的 qpos 上。"""

    @classmethod
    def setUpClass(cls):
        try:
            import mujoco
        except ImportError:
            raise unittest.SkipTest("没装 mujoco")
        sys.modules.setdefault("g1_mujoco_sim", sys.modules[__name__])
        from g1_mujoco_sim import load_model
        cls.mujoco = mujoco
        cls.model = load_model(_FTP_MODEL, mujoco)
        cls.hm = load_hand_mapping()

    def test_every_mapped_joint_exists_in_the_model(self):
        """hand_mapping 的表和 URDF 必须一个不差 —— 少一个就是静默不动。"""
        for side in ("right", "left"):
            addresses = resolve_hand_addresses(
                self.model, self.mujoco, self.hm, side)
            self.assertEqual(len(addresses), 12, f"{side} 手关节数不对")
            for name in self.hm.driver_joints(side):
                self.assertIn(name, addresses)

    def test_expanded_angles_land_on_distinct_qpos_slots(self):
        addresses = resolve_hand_addresses(
            self.model, self.mujoco, self.hm, "right")
        self.assertEqual(len(set(addresses.values())), 12)

    def test_closure_actually_moves_the_fingers(self):
        data = self.mujoco.MjData(self.model)
        addresses = resolve_hand_addresses(
            self.model, self.mujoco, self.hm, "right")
        angles = self.hm.expand_mimic(
            self.hm.closure_to_angles((1.0,) * HAND_CHANNEL_COUNT))
        for name, angle in angles.items():
            data.qpos[addresses[name]] = angle
        self.mujoco.mj_forward(self.model, data)
        moved = [n for n, a in addresses.items() if data.qpos[a] != 0.0]
        self.assertEqual(len(moved), 12)

    def test_model_joint_limits_agree_with_the_mapping_table(self):
        """URDF 上限换了版本就会和 JOINT_UPPER 漂开，这里对拍一次。"""
        for name, upper in zip(self.hm.driver_joints("right"),
                               self.hm.JOINT_UPPER):
            joint_id = self.mujoco.mj_name2id(
                self.model, self.mujoco.mjtObj.mjOBJ_JOINT, name)
            self.assertAlmostEqual(
                float(self.model.jnt_range[joint_id][1]), upper, places=4,
                msg=f"{name} 的上限和 hand_mapping.JOINT_UPPER 对不上")


if __name__ == "__main__":
    unittest.main()
