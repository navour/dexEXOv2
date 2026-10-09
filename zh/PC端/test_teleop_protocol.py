import importlib.util
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teleop_protocol import (
    HAND_SIZE, PACKET_SIZE_V2, WAIST_SIZE, pack_arm_command, pack_hand_block,
    pack_waist_block)


_ROBOT_PROTOCOL = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "机器人端", "teleop_protocol.py")
_SPEC = importlib.util.spec_from_file_location(
    "robot_teleop_protocol", _ROBOT_PROTOCOL)
robot_protocol = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(robot_protocol)


class TeleopProtocolTest(unittest.TestCase):
    def test_v2_round_trip(self):
        positions = tuple(i * 0.1 for i in range(10))
        velocities = tuple(-i * 0.2 for i in range(10))
        packet = pack_arm_command(3, 0xFFFFFFFE, 123456789,
                                  positions, velocities)
        decoded = robot_protocol.unpack_arm_command(packet)
        self.assertEqual(decoded["version"], 2)
        self.assertEqual(decoded["mode"], 3)
        self.assertEqual(decoded["seq"], 0xFFFFFFFE)
        self.assertEqual(decoded["sender_timestamp_us"], 123456789)
        for actual, expected in zip(decoded["positions"], positions):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(decoded["velocities"], velocities):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_legacy_dual_wrist_is_still_supported(self):
        positions = tuple(i * 0.1 for i in range(10))
        packet = struct.pack(
            "!4sBffffffffff", b"UARM", 1, *positions)
        decoded = robot_protocol.unpack_arm_command(packet)
        self.assertEqual(decoded["version"], 1)
        self.assertEqual(decoded["mode"], 1)
        for actual, expected in zip(decoded["positions"], positions):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertEqual(decoded["velocities"], (0.0,) * 10)

    def test_waist_tail_never_disturbs_the_arms(self):
        """尾块只能新增腰部字段，双臂解析结果必须逐字节一致。"""
        positions = tuple(i * 0.1 for i in range(10))
        velocities = tuple(-i * 0.2 for i in range(10))
        plain = pack_arm_command(3, 7, 99, positions, velocities)
        extended = pack_arm_command(
            3, 7, 99, positions, velocities,
            waist=(0.4, 0.0, 0.0), waist_enabled=True)

        self.assertEqual(len(extended), PACKET_SIZE_V2 + WAIST_SIZE)
        self.assertEqual(extended[:PACKET_SIZE_V2], plain)

        with_tail = robot_protocol.unpack_arm_command(extended)
        without = robot_protocol.unpack_arm_command(plain)
        self.assertIsNone(without["waist"])
        self.assertAlmostEqual(with_tail["waist"][0], 0.4, places=6)
        for key in ("version", "mode", "seq", "sender_timestamp_us",
                    "positions", "velocities"):
            self.assertEqual(with_tail[key], without[key], key)

    def test_robot_receiver_reads_the_waist_the_pc_sends(self):
        """PC 打包与真机接收端解析必须对得上。"""
        packet = pack_arm_command(
            3, 1, 2, (0.0,) * 10, (0.0,) * 10,
            waist=(0.37, 0.0, 0.0), waist_enabled=True)

        decoded = robot_protocol.unpack_arm_command(packet)

        self.assertAlmostEqual(decoded["waist"][0], 0.37, places=6)

    def test_robot_receiver_ignores_the_waist_when_disabled(self):
        packet = pack_arm_command(
            3, 1, 2, (0.0,) * 10, (0.0,) * 10,
            waist=(0.37, 0.0, 0.0), waist_enabled=False)

        decoded = robot_protocol.unpack_arm_command(packet)

        self.assertIsNone(decoded["waist"])
        self.assertEqual(decoded["mode"], 3)

    def test_legacy_packets_report_no_waist(self):
        packet = struct.pack(
            "!4sBffffffffff", b"UARM", 1, *(0.0,) * 10)
        self.assertIsNone(robot_protocol.unpack_arm_command(packet)["waist"])

    def test_corrupt_tail_costs_the_waist_but_not_the_arms(self):
        """尾块坏掉只让腰回中，双臂必须照常跟随。"""
        positions = tuple(i * 0.1 for i in range(10))
        body = pack_arm_command(3, 7, 99, positions, (0.0,) * 10)
        for tail in (b"", b"\x00" * WAIST_SIZE, b"XXXX" + b"\x00" * 25,
                     pack_waist_block((0.4, 0.0, 0.0),
                                      waist_enabled=True)[:10]):
            with self.subTest(tail=tail[:4]):
                decoded = robot_protocol.unpack_arm_command(body + tail)
                self.assertIsNotNone(decoded)
                self.assertIsNone(decoded["waist"])
                for actual, expected in zip(decoded["positions"], positions):
                    self.assertAlmostEqual(actual, expected, places=6)

    def test_absurd_waist_angle_is_rejected(self):
        """离谱角度只丢腰，不能丢整包 —— 丢整包等于双臂也停。"""
        for bad in (99.0, float("nan"), float("inf")):
            with self.subTest(bad=bad):
                packet = pack_arm_command(
                    3, 1, 2, (0.0,) * 10, (0.0,) * 10,
                    waist=(bad, 0.0, 0.0), waist_enabled=True)
                decoded = robot_protocol.unpack_arm_command(packet)
                self.assertIsNotNone(decoded)
                self.assertIsNone(decoded["waist"])

    def test_waist_tail_does_not_disturb_the_simulator(self):
        sim_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "仿真", "g1_mujoco_sim.py")
        spec = importlib.util.spec_from_file_location("g1_mujoco_sim", sim_path)
        sim = importlib.util.module_from_spec(spec)
        # dataclass 装饰器会回查 sys.modules，先注册再执行。
        sys.modules["g1_mujoco_sim"] = sim
        try:
            spec.loader.exec_module(sim)
        finally:
            sys.modules.pop("g1_mujoco_sim", None)

        positions = tuple(i * 0.1 for i in range(10))
        velocities = tuple(-i * 0.2 for i in range(10))
        extended = pack_arm_command(
            3, 7, 99, positions, velocities,
            waist=(0.4, 0.0, 0.0), waist_enabled=True)

        decoded = sim.unpack_arm_command(extended)
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.mode, 3)
        self.assertEqual(decoded.version, 2)
        for actual, expected in zip(decoded.right + decoded.left, positions):
            self.assertAlmostEqual(actual, expected, places=6)

    def _load_by_path(self, name, path):
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        # dataclass 装饰器会回查 sys.modules，先注册再执行。
        sys.modules[name] = module
        try:
            spec.loader.exec_module(module)
        finally:
            sys.modules.pop(name, None)
        return module

    def _humdex_protocol(self):
        return self._load_by_path(
            "humdex_ua2m_protocol",
            os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                "HumDex_IMU融合实验", "bridge", "ua2m_protocol.py"))

    def test_humdex_reads_the_waist_the_pc_sends(self):
        """PC 打包与 HumDex 解析必须对得上，这是腰部链路的唯一接缝。"""
        humdex = self._humdex_protocol()
        packet = pack_arm_command(
            3, 1, 2, (0.0,) * 10, (0.0,) * 10,
            waist=(0.37, 0.0, 0.0), waist_enabled=True)

        command = humdex.parse_arm_packet(packet)

        self.assertEqual(command.protocol, "UA2M+W")
        self.assertAlmostEqual(command.waist[0], 0.37, places=6)

    def test_humdex_ignores_the_waist_when_disabled(self):
        humdex = self._humdex_protocol()
        packet = pack_arm_command(
            3, 1, 2, (0.0,) * 10, (0.0,) * 10,
            waist=(0.37, 0.0, 0.0), waist_enabled=False)

        command = humdex.parse_arm_packet(packet)

        self.assertIsNone(command.waist)
        self.assertEqual(command.mode, 3)

    def test_waist_block_rejects_wrong_axis_count(self):
        with self.assertRaises(ValueError):
            pack_waist_block((0.1, 0.2))

    # ---------- 灵巧手尾块 ----------

    RIGHT = (0.0, 0.1, 0.2, 0.3, 0.4, 0.5)
    LEFT = (0.9, 0.8, 0.7, 0.6, 0.5, 0.4)

    def _with_hands(self, right=None, left=None, waist=True):
        return pack_arm_command(
            3, 1, 2, (0.0,) * 10, (0.0,) * 10,
            waist=(0.37, 0.0, 0.0) if waist else None, waist_enabled=waist,
            right_hand=right, left_hand=left)

    def test_hand_tail_round_trip(self):
        decoded = robot_protocol.unpack_arm_command(
            self._with_hands(self.RIGHT, self.LEFT))
        for actual, expected in zip(decoded["hands"]["right"], self.RIGHT):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(decoded["hands"]["left"], self.LEFT):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_right_hand_only_leaves_left_absent(self):
        """只戴一只手套时，另一只手必须是"没有数据"而不是全零指令。"""
        decoded = robot_protocol.unpack_arm_command(
            self._with_hands(right=self.RIGHT))
        self.assertIsNotNone(decoded["hands"]["right"])
        self.assertIsNone(decoded["hands"]["left"])

    def test_packet_layout_is_waist_then_hand(self):
        """顺序不能反：没升级的消费端在固定偏移 97 上读腰块。"""
        packet = self._with_hands(self.RIGHT)
        self.assertEqual(len(packet), PACKET_SIZE_V2 + WAIST_SIZE + HAND_SIZE)
        self.assertEqual(packet[PACKET_SIZE_V2:PACKET_SIZE_V2 + 4], b"UAWS")
        self.assertEqual(
            packet[PACKET_SIZE_V2 + WAIST_SIZE:PACKET_SIZE_V2 + WAIST_SIZE + 4],
            b"UHND")

    def test_hand_tail_never_disturbs_the_waist_or_the_arms(self):
        positions = tuple(i * 0.1 for i in range(10))
        packet = pack_arm_command(
            3, 1, 2, positions, (0.0,) * 10,
            waist=(0.37, 0.0, 0.0), waist_enabled=True,
            right_hand=self.RIGHT)
        decoded = robot_protocol.unpack_arm_command(packet)
        self.assertAlmostEqual(decoded["waist"][0], 0.37, places=6)
        for actual, expected in zip(decoded["positions"], positions):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_hand_tail_without_waist_still_parses(self):
        """腰没开、只开手时，手块排在最前面也要能找到。"""
        decoded = robot_protocol.unpack_arm_command(
            self._with_hands(self.RIGHT, waist=False))
        self.assertIsNone(decoded["waist"])
        self.assertIsNotNone(decoded["hands"]["right"])

    def test_corrupt_hand_tail_costs_the_hand_but_not_the_waist(self):
        packet = self._with_hands(self.RIGHT)
        corrupt = packet[:PACKET_SIZE_V2 + WAIST_SIZE] + b"XXXX" + \
            packet[PACKET_SIZE_V2 + WAIST_SIZE + 4:]
        decoded = robot_protocol.unpack_arm_command(corrupt)
        self.assertIsNotNone(decoded)
        self.assertAlmostEqual(decoded["waist"][0], 0.37, places=6)
        self.assertIsNone(decoded["hands"])

    def test_truncated_hand_tail_is_ignored(self):
        packet = self._with_hands(self.RIGHT)
        decoded = robot_protocol.unpack_arm_command(packet[:-10])
        self.assertIsNotNone(decoded)
        self.assertIsNone(decoded["hands"])
        self.assertAlmostEqual(decoded["waist"][0], 0.37, places=6)

    def test_out_of_range_closure_voids_that_hand(self):
        tail = struct.pack("!4sB12f", b"UHND", 0x1,
                           *((5.0,) * 6 + (0.0,) * 6))
        base = pack_arm_command(3, 1, 2, (0.0,) * 10, (0.0,) * 10)
        decoded = robot_protocol.unpack_arm_command(base + tail)
        self.assertIsNotNone(decoded)
        self.assertIsNone(decoded["hands"])

    def test_nan_closure_is_packed_as_open(self):
        """NaN 不该整包丢弃 —— 那会连手臂一起超时回零。夹成张开。"""
        block = pack_hand_block(right_hand=(float("nan"),) + (0.5,) * 5)
        base = pack_arm_command(3, 1, 2, (0.0,) * 10, (0.0,) * 10)
        decoded = robot_protocol.unpack_arm_command(base + block)
        self.assertEqual(decoded["hands"]["right"][0], 0.0)

    def test_closure_is_clamped_not_extrapolated(self):
        block = pack_hand_block(right_hand=(-1.0, 2.0, 0.5, 0.5, 0.5, 0.5))
        base = pack_arm_command(3, 1, 2, (0.0,) * 10, (0.0,) * 10)
        hands = robot_protocol.unpack_arm_command(base + block)["hands"]
        self.assertEqual(hands["right"][0], 0.0)
        self.assertEqual(hands["right"][1], 1.0)

    def test_hand_block_rejects_wrong_channel_count(self):
        with self.assertRaises(ValueError):
            pack_hand_block(right_hand=(0.1, 0.2))

    def test_legacy_packets_report_no_hands(self):
        packet = pack_arm_command(3, 1, 2, (0.0,) * 10, (0.0,) * 10)
        self.assertIsNone(robot_protocol.unpack_arm_command(packet)["hands"])
        legacy = struct.pack("!4sBffffffffff", b"UARM", 3, *((0.0,) * 10))
        self.assertIsNone(robot_protocol.unpack_arm_command(legacy)["hands"])

    def test_hand_tail_does_not_disturb_the_simulator(self):
        """仿真端还没认 UHND，追加它绝不能让它丢包或读错腰。"""
        sim = self._load_by_path(
            "g1_mujoco_sim",
            os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                "仿真", "g1_mujoco_sim.py"))
        decoded = sim.unpack_arm_command(self._with_hands(self.RIGHT))
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.mode, 3)
        self.assertAlmostEqual(decoded.waist[0], 0.37, places=6)

    def test_hand_tail_does_not_disturb_humdex(self):
        humdex = self._humdex_protocol()
        command = humdex.parse_arm_packet(self._with_hands(self.RIGHT))
        self.assertEqual(command.mode, 3)
        self.assertAlmostEqual(command.waist[0], 0.37, places=6)

    def test_invalid_packet_is_rejected(self):
        self.assertIsNone(robot_protocol.unpack_arm_command(b"bad"))
        packet = pack_arm_command(
            1, 1, 10, (float("nan"),) + (0.0,) * 9, (0.0,) * 10)
        self.assertIsNone(robot_protocol.unpack_arm_command(packet))


if __name__ == "__main__":
    unittest.main()
