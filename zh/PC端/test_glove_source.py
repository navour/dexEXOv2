#!/usr/bin/env python3
"""glove_source 的单测。

这一层是手套和仿真之间唯一的接缝：C++ 那边发的是因时寄存器值，仿真要的是
闭合度，方向还是反的。所以端点、方向、失效回退全部钉死。
"""

import json
from pathlib import Path
import socket
import tempfile
import time
import unittest

import glove_source as gs
import hand_mapping as hm


_SIM_CFG = (Path(__file__).resolve().parent.parent
            / "mhandpro" / "config" / "inspire_right_sim.cfg")
_LEFT_CFG = (Path(__file__).resolve().parent.parent
             / "mhandpro" / "config" / "inspire_left.cfg")

FULL_OPEN = (1000,) * 6
FULL_CLOSED = (0,) * 6


def _ctrl(counts):
    return (json.dumps({
        "type": "ctrl", "angle_set": list(counts), "mode": 1,
        "force_set": [100] * 6, "speed_set": [100] * 6,
    }) + "\n").encode()


class EndpointLoadingTests(unittest.TestCase):
    def test_reads_the_sim_cfg(self):
        opened, closed = gs.load_endpoints(_SIM_CFG)
        self.assertEqual(opened, FULL_OPEN)
        self.assertEqual(closed, FULL_CLOSED)

    def test_reads_the_measured_left_cfg(self):
        """左手那份是实测端点，不是满量程 —— 解析器不能只认整齐的值。"""
        opened, closed = gs.load_endpoints(_LEFT_CFG)
        self.assertEqual(opened, (980, 965, 957, 946, 949, 922))
        self.assertEqual(closed, (60, 60, 70, 60, 70, 150))

    def test_comments_and_blank_lines_are_ignored(self):
        with tempfile.NamedTemporaryFile("w", suffix=".cfg",
                                         delete=False) as handle:
            handle.write("# 注释\n\nopen=1,2,3,4,5,6  # 行尾注释\n"
                         "closed=0,0,0,0,0,0\n")
            path = handle.name
        self.assertEqual(gs.load_endpoints(path)[0], (1, 2, 3, 4, 5, 6))

    def test_reads_the_port_from_the_cfg(self):
        """监听端和 C++ 的 teleop 必须读同一个 port，各写各的会连不上。"""
        self.assertEqual(gs.load_port(_SIM_CFG), 9103)
        self.assertEqual(gs.load_port(_LEFT_CFG), 9102)

    def test_missing_port_falls_back_to_default(self):
        with tempfile.NamedTemporaryFile("w", suffix=".cfg",
                                         delete=False) as handle:
            handle.write("open=1,2,3,4,5,6\nclosed=0,0,0,0,0,0\n")
            path = handle.name
        self.assertEqual(gs.load_port(path), gs.DEFAULT_PORT)

    def test_source_defaults_to_the_cfg_port(self):
        """--hand-port 不给时应当落到 cfg 的 port，而不是崩掉。

        用临时 cfg 写一个当前空闲的端口，而不是直接绑 9103 —— 上位机正在
        跑的时候那个口是占着的，测试不该和真机跑的程序抢端口。
        """
        probe = socket.socket()
        probe.bind(("127.0.0.1", 0))
        free_port = probe.getsockname()[1]
        probe.close()

        with tempfile.NamedTemporaryFile("w", suffix=".cfg",
                                         delete=False) as handle:
            handle.write(f"port={free_port}\n"
                         "open=1000,1000,1000,1000,1000,1000\n"
                         "closed=0,0,0,0,0,0\n")
            path = handle.name

        source = gs.GloveSource(cfg_path=path)
        self.addCleanup(source.close)
        self.assertEqual(source.port, free_port)

    def test_explicit_port_zero_still_means_ephemeral(self):
        source = gs.GloveSource(cfg_path=_SIM_CFG, port=0)
        self.addCleanup(source.close)
        self.assertNotEqual(source.port, 0)
        self.assertNotEqual(source.port, 9103)

    def test_missing_endpoint_is_an_error(self):
        with tempfile.NamedTemporaryFile("w", suffix=".cfg",
                                         delete=False) as handle:
            handle.write("open=1,2,3,4,5,6\n")
            path = handle.name
        with self.assertRaises(ValueError):
            gs.load_endpoints(path)

    def test_wrong_channel_count_is_an_error(self):
        with tempfile.NamedTemporaryFile("w", suffix=".cfg",
                                         delete=False) as handle:
            handle.write("open=1,2,3\nclosed=0,0,0\n")
            path = handle.name
        with self.assertRaises(ValueError):
            gs.load_endpoints(path)


class LineParsingTests(unittest.TestCase):
    def test_parses_a_teleop_line(self):
        self.assertEqual(
            gs.parse_ctrl_line(_ctrl((10, 20, 30, 40, 50, 60)).decode()),
            (10, 20, 30, 40, 50, 60))

    def test_rejects_garbage(self):
        for line in ("", "not json", "[1,2,3]", "{}", "null"):
            self.assertIsNone(gs.parse_ctrl_line(line))

    def test_rejects_other_message_types(self):
        self.assertIsNone(gs.parse_ctrl_line(
            json.dumps({"type": "status", "angle_set": [0] * 6})))

    def test_rejects_wrong_channel_count(self):
        self.assertIsNone(gs.parse_ctrl_line(
            json.dumps({"type": "ctrl", "angle_set": [0, 1, 2]})))


class ConversionTests(unittest.TestCase):
    """满量程 cfg 下，寄存器值和闭合度必须严格互逆。"""

    def test_full_open_is_zero_closure(self):
        self.assertEqual(
            hm.glove_to_closure(FULL_OPEN, FULL_OPEN, FULL_CLOSED),
            (0.0,) * 6)

    def test_full_closed_is_one(self):
        self.assertEqual(
            hm.glove_to_closure(FULL_CLOSED, FULL_OPEN, FULL_CLOSED),
            (1.0,) * 6)

    def test_ticks_are_closure_times_thousand(self):
        """C++ 的 to_inspire_ticks 在满量程下就是 1000*(1-闭合度)。"""
        for closure in (0.0, 0.25, 0.5, 0.75, 1.0):
            ticks = (round(1000 * (1 - closure)),) * 6
            back = hm.glove_to_closure(ticks, FULL_OPEN, FULL_CLOSED)
            self.assertAlmostEqual(back[0], closure, places=3)


class SourceTests(unittest.TestCase):
    def setUp(self):
        self.source = gs.GloveSource(cfg_path=_SIM_CFG, port=0)
        self.addCleanup(self.source.close)

    def _connect(self):
        client = socket.create_connection(("127.0.0.1", self.source.port),
                                          timeout=2.0)
        self.addCleanup(client.close)
        return client

    def _wait(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.01)
        return False

    def test_nothing_before_any_data(self):
        self.assertIsNone(self.source.latest_closure())

    def test_receives_and_converts(self):
        client = self._connect()
        client.sendall(_ctrl((1000, 750, 500, 250, 0, 500)))
        self.assertTrue(self._wait(
            lambda: self.source.latest_closure() is not None))
        closure = self.source.latest_closure()
        self.assertAlmostEqual(closure[0], 0.0, places=3)   # 1000 = 张开
        self.assertAlmostEqual(closure[2], 0.5, places=3)
        self.assertAlmostEqual(closure[4], 1.0, places=3)   # 0 = 握紧

    def test_split_packets_are_reassembled(self):
        client = self._connect()
        payload = _ctrl((0,) * 6)
        client.sendall(payload[:12])
        time.sleep(0.05)
        self.assertIsNone(self.source.latest_closure())
        client.sendall(payload[12:])
        self.assertTrue(self._wait(
            lambda: self.source.latest_closure() is not None))

    def test_multiple_lines_in_one_packet_keeps_the_last(self):
        client = self._connect()
        client.sendall(_ctrl((1000,) * 6) + _ctrl((0,) * 6))
        self.assertTrue(self._wait(
            lambda: self.source.latest_closure() is not None))
        self.assertAlmostEqual(self.source.latest_closure()[0], 1.0, places=3)

    def test_bad_line_does_not_break_the_stream(self):
        client = self._connect()
        client.sendall("这不是 json\n".encode("utf-8") + _ctrl((0,) * 6))
        self.assertTrue(self._wait(
            lambda: self.source.latest_closure() is not None))
        self.assertGreaterEqual(self.source.bad_lines, 1)

    def test_stale_data_opens_the_hand(self):
        source = gs.GloveSource(cfg_path=_SIM_CFG, port=0, stale_sec=0.05)
        self.addCleanup(source.close)
        client = socket.create_connection(("127.0.0.1", source.port),
                                          timeout=2.0)
        self.addCleanup(client.close)
        client.sendall(_ctrl((0,) * 6))
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and source.latest_closure() is None:
            time.sleep(0.01)
        self.assertIsNotNone(source.latest_closure())
        time.sleep(0.15)
        self.assertIsNone(source.latest_closure(), "超时后必须回到无数据")

    def test_disconnect_drops_the_last_frame(self):
        client = self._connect()
        client.sendall(_ctrl((0,) * 6))
        self.assertTrue(self._wait(
            lambda: self.source.latest_closure() is not None))
        client.close()
        self.assertTrue(
            self._wait(lambda: self.source.latest_closure() is None),
            "断连后不能留着上一帧让手僵在半握")

    def test_reconnect_works(self):
        first = self._connect()
        first.sendall(_ctrl((0,) * 6))
        self.assertTrue(self._wait(
            lambda: self.source.latest_closure() is not None))
        first.close()
        self.assertTrue(self._wait(
            lambda: self.source.latest_closure() is None))
        second = self._connect()
        second.sendall(_ctrl((1000,) * 6))
        self.assertTrue(self._wait(
            lambda: self.source.latest_closure() is not None))


class EndToEndTests(unittest.TestCase):
    """手套字节 → 闭合度 → UHND → 仿真读到的关节角，一条链走完。

    每一环都有自己的单测，但通道顺序和方向这类错误只有串起来才看得见 ——
    某一环反了、另一环也反了的话，单测各自都是绿的。
    """

    def setUp(self):
        import importlib.util

        root = Path(__file__).resolve().parent.parent
        self.source = gs.GloveSource(cfg_path=_SIM_CFG, port=0)
        self.addCleanup(self.source.close)

        spec = importlib.util.spec_from_file_location(
            "sim_for_glove_test", root / "仿真" / "g1_mujoco_sim.py")
        self.sim = importlib.util.module_from_spec(spec)
        import sys as _sys
        _sys.modules["sim_for_glove_test"] = self.sim
        try:
            spec.loader.exec_module(self.sim)
        finally:
            _sys.modules.pop("sim_for_glove_test", None)

        import teleop_protocol
        self.protocol = teleop_protocol

    def _feed(self, ticks):
        client = socket.create_connection(("127.0.0.1", self.source.port),
                                          timeout=2.0)
        self.addCleanup(client.close)
        client.sendall(_ctrl(ticks))
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            closure = self.source.latest_closure()
            if closure is not None:
                return closure
            time.sleep(0.01)
        self.fail("没有收到手套数据")

    def test_fist_travels_all_the_way_to_joint_angles(self):
        # 全握：因时寄存器 0 = 握紧
        closure = self._feed((0,) * 6)
        self.assertEqual(closure, (1.0,) * 6)

        packet = self.protocol.pack_arm_command(
            3, 1, 2, (0.0,) * 10, (0.0,) * 10, right_hand=closure)
        command = self.sim.unpack_arm_command(packet)
        self.assertEqual(command.hands["right"], (1.0,) * 6)

        angles = hm.expand_mimic(hm.closure_to_angles(command.hands["right"]))
        # 每个手指关节都必须真的弯了，而且不超 URDF 限位。
        for name, angle in angles.items():
            self.assertGreater(angle, 0.0, f"{name} 没动")
        self.assertAlmostEqual(
            angles["right_little_1_joint"], hm.JOINT_UPPER[0])

    def test_open_hand_travels_as_zero(self):
        closure = self._feed((1000,) * 6)
        self.assertEqual(closure, (0.0,) * 6)
        packet = self.protocol.pack_arm_command(
            3, 1, 2, (0.0,) * 10, (0.0,) * 10, right_hand=closure)
        command = self.sim.unpack_arm_command(packet)
        angles = hm.expand_mimic(hm.closure_to_angles(command.hands["right"]))
        for name, angle in angles.items():
            self.assertEqual(angle, 0.0, f"{name} 张手时不该有角度")

    def test_per_channel_order_survives_the_whole_chain(self):
        """只弯小指，到了关节角那头也必须只有小指弯。

        通道顺序错位是这条链上最难查的错 —— 画面里手会动，只是动错手指。
        """
        # 通道 0 = 小指，给它 0（握紧），其余 1000（张开）
        closure = self._feed((0, 1000, 1000, 1000, 1000, 1000))
        packet = self.protocol.pack_arm_command(
            3, 1, 2, (0.0,) * 10, (0.0,) * 10, right_hand=closure)
        command = self.sim.unpack_arm_command(packet)
        angles = hm.expand_mimic(hm.closure_to_angles(command.hands["right"]))

        bent = {name for name, angle in angles.items() if angle > 0.0}
        self.assertEqual(bent, {"right_little_1_joint",
                                "right_little_2_joint"})


if __name__ == "__main__":
    unittest.main()
