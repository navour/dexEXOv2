#!/usr/bin/env python3
"""G1 MuJoCo 仿真器中与模型无关的单元测试。"""

import math
import struct
import unittest

from g1_mujoco_sim import (
    PACKET_FMT_V2,
    PACKET_FMT_DUAL,
    PACKET_FMT_DUAL_WR,
    PACKET_FMT_SINGLE,
    ArmCommand,
    UdpCommandReceiver,
    step_toward,
    unpack_arm_command,
)


class PacketParsingTests(unittest.TestCase):
    def test_v2_packet(self) -> None:
        positions = [float(index) / 10.0 for index in range(10)]
        velocities = [float(index) / 20.0 for index in range(10)]
        packet = struct.pack(
            PACKET_FMT_V2,
            b"UA2M",
            3,
            123,
            456789,
            *(positions + velocities),
        )
        command = unpack_arm_command(packet)
        self.assertIsNotNone(command)
        self.assertEqual(command.version, 2)
        self.assertEqual(command.seq, 123)
        self.assertEqual(command.sender_timestamp_us, 456789)
        self.assertAlmostEqual(command.right[4], 0.4)
        self.assertAlmostEqual(command.left[0], 0.5)
        self.assertAlmostEqual(command.velocities[9], 0.45)

    def test_current_dual_arm_packet(self) -> None:
        packet = struct.pack(
            PACKET_FMT_DUAL_WR,
            b"UARM",
            3,
            *[float(index) / 10.0 for index in range(10)],
        )
        command = unpack_arm_command(packet)
        self.assertIsNotNone(command)
        self.assertEqual(command.mode, 3)
        self.assertAlmostEqual(command.right[4], 0.4)
        self.assertAlmostEqual(command.left[0], 0.5)
        self.assertAlmostEqual(command.left[4], 0.9)

    def test_legacy_packets_fill_wrist_roll_with_zero(self) -> None:
        dual = struct.pack(PACKET_FMT_DUAL, b"UARM", 2, *([0.25] * 8))
        single = struct.pack(PACKET_FMT_SINGLE, b"UARM", 1, *([0.5] * 4))
        dual_command = unpack_arm_command(dual)
        single_command = unpack_arm_command(single)
        self.assertEqual(dual_command.right[4], 0.0)
        self.assertEqual(dual_command.left[4], 0.0)
        self.assertEqual(single_command.left, (0.0,) * 5)

    def test_invalid_packets_are_rejected(self) -> None:
        bad_header = struct.pack(PACKET_FMT_SINGLE, b"NOPE", 1, *([0.0] * 4))
        bad_mode = struct.pack(PACKET_FMT_SINGLE, b"UARM", 9, *([0.0] * 4))
        nan_value = struct.pack(
            PACKET_FMT_SINGLE,
            b"UARM",
            1,
            math.nan,
            0.0,
            0.0,
            0.0,
        )
        self.assertIsNone(unpack_arm_command(b"short"))
        self.assertIsNone(unpack_arm_command(bad_header))
        self.assertIsNone(unpack_arm_command(bad_mode))
        self.assertIsNone(unpack_arm_command(nan_value))


class RateLimitTests(unittest.TestCase):
    def test_step_toward_does_not_overshoot(self) -> None:
        self.assertEqual(step_toward(0.0, 1.0, 0.2), 0.2)
        self.assertEqual(step_toward(0.0, -1.0, 0.2), -0.2)
        self.assertEqual(step_toward(0.9, 1.0, 0.2), 1.0)


class SequenceStatisticsTests(unittest.TestCase):
    def test_v2_loss_duplicate_and_out_of_order(self) -> None:
        # 该方法本身不依赖 socket；绕过构造函数可让测试在禁止网络的
        # CI/沙箱内运行。
        receiver = UdpCommandReceiver.__new__(UdpCommandReceiver)
        receiver.last_seq = None
        receiver.lost_packets = 0
        receiver.duplicate_packets = 0
        receiver.out_of_order_packets = 0
        receiver.last_transport_age_ms = None

        def command(seq: int) -> ArmCommand:
            return ArmCommand(
                mode=3,
                right=(0.0,) * 5,
                left=(0.0,) * 5,
                version=2,
                seq=seq,
            )

        receiver._update_v2_stats(command(10), 1.0)
        receiver._update_v2_stats(command(13), 1.1)
        receiver._update_v2_stats(command(13), 1.2)
        receiver._update_v2_stats(command(12), 1.3)

        self.assertEqual(receiver.last_seq, 13)
        self.assertEqual(receiver.lost_packets, 2)
        self.assertEqual(receiver.duplicate_packets, 1)
        self.assertEqual(receiver.out_of_order_packets, 1)


if __name__ == "__main__":
    unittest.main()
