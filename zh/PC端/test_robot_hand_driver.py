"""机器人端灵巧手驱动的测试。

被测模块在 机器人端/hand_driver.py, 按路径加载 —— 那边没有自己的测试环
境, 而这一层直接决定真手会不会夹住人, 必须有回归覆盖。理由与
test_waist_follow.py 相同。

覆盖三块:
  1. Modbus 帧的编解码 —— 这份是手写的裸帧, 没有 pymodbus 兜底, 位错一
     个字节就会写到别的寄存器上去
  2. 换算与限速
  3. HandFollower 的四条安全线: 没数据张开、断流张开、断链重连后先张开、
     退出张开
"""

import importlib.util
import os
import struct
import time
import unittest


_MODULE_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "机器人端", "hand_driver.py")
_SPEC = importlib.util.spec_from_file_location("robot_hand_driver",
                                               _MODULE_PATH)
hd = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(hd)

import hand_mapping as hm


OPEN = [hd.COUNT_OPEN] * 6


# ======================== Modbus 帧 ========================

class ModbusFrameTest(unittest.TestCase):
    def test_read_request_layout(self):
        frame = hd.build_read_request(0x1234, 1, hd.REGISTER_ANGLE_ACT, 6)
        txn, proto, length, unit = struct.unpack("!HHHB", frame[:7])
        self.assertEqual(txn, 0x1234)
        self.assertEqual(proto, 0)
        # length 覆盖 unit_id 及其后所有字节: 1 + fc + addr + count = 6
        self.assertEqual(length, 6)
        self.assertEqual(length, len(frame) - 6)
        self.assertEqual(unit, 1)
        fc, addr, count = struct.unpack("!BHH", frame[7:])
        self.assertEqual(fc, hd.MODBUS_FC_READ_HOLDING)
        self.assertEqual(addr, 1546)
        self.assertEqual(count, 6)

    def test_write_request_layout(self):
        values = [1000, 900, 800, 700, 600, 500]
        frame = hd.build_write_request(1, 1, hd.REGISTER_ANGLE_SET, values)
        _, _, length, _ = struct.unpack("!HHHB", frame[:7])
        self.assertEqual(length, len(frame) - 6)
        fc, addr, count, nbytes = struct.unpack("!BHHB", frame[7:13])
        self.assertEqual(fc, hd.MODBUS_FC_WRITE_MULTIPLE)
        self.assertEqual(addr, 1486)
        self.assertEqual(count, 6)
        self.assertEqual(nbytes, 12)
        self.assertEqual(list(struct.unpack("!6H", frame[13:])), values)

    def test_frames_are_big_endian(self):
        """写错端序时寄存器 1486 会变成 0xCE05, 打到完全不相干的地方。"""
        frame = hd.build_write_request(0, 1, 1486, [0] * 6)
        self.assertEqual(frame[8:10], bytes([0x05, 0xCE]))

    def test_transaction_id_advances_and_wraps(self):
        hand = hd.ModbusTcpHand("127.0.0.1")
        hand._transaction = 0xFFFF
        self.assertEqual(hand._next_transaction(), 0)

    def test_decode_read_response(self):
        payload = struct.pack("!6H", 1, 2, 3, 4, 5, 6)
        frame = (struct.pack("!HHHB", 1, 0, 3 + len(payload), 1)
                 + bytes([hd.MODBUS_FC_READ_HOLDING, len(payload)]) + payload)
        self.assertEqual(hd.decode_read_response(frame, 6), [1, 2, 3, 4, 5, 6])

    def test_exception_response_is_detected(self):
        """异常响应是合法的 Modbus 响应, 不识别就会被当成数据解析。"""
        frame = (struct.pack("!HHHB", 1, 0, 3, 1)
                 + bytes([hd.MODBUS_FC_READ_HOLDING | 0x80, 0x02]))
        with self.assertRaises(hd.ModbusError) as caught:
            hd.decode_read_response(frame, 6)
        self.assertIn("异常码 2", str(caught.exception))

    def test_wrong_function_code_rejected(self):
        frame = (struct.pack("!HHHB", 1, 0, 3, 1)
                 + bytes([hd.MODBUS_FC_WRITE_MULTIPLE, 0]))
        with self.assertRaises(hd.ModbusError):
            hd.parse_response(frame, hd.MODBUS_FC_READ_HOLDING)

    def test_short_frame_rejected(self):
        with self.assertRaises(hd.ModbusError):
            hd.parse_response(b"\x00\x01", hd.MODBUS_FC_READ_HOLDING)

    def test_byte_count_mismatch_rejected(self):
        payload = struct.pack("!3H", 1, 2, 3)
        frame = (struct.pack("!HHHB", 1, 0, 3 + len(payload), 1)
                 + bytes([hd.MODBUS_FC_READ_HOLDING, len(payload)]) + payload)
        with self.assertRaises(hd.ModbusError):
            hd.decode_read_response(frame, 6)


# ======================== 换算与限速 ========================

class ConversionTest(unittest.TestCase):
    def test_none_opens_the_hand(self):
        self.assertEqual(hd.closure_to_counts(None), OPEN)

    def test_open_closure_is_full_open(self):
        self.assertEqual(hd.closure_to_counts(hm.open_closure()), OPEN)

    def test_direction_larger_closure_is_more_closed(self):
        soft = hd.closure_to_counts((0.2,) * 6)
        hard = hd.closure_to_counts((0.9,) * 6)
        for a, b in zip(soft, hard):
            self.assertGreater(a, b)

    def test_full_closure_respects_the_real_hand_cap(self):
        """闭合度 1.0 落在上限定义的位置, 并且不越出寄存器量程。

        注意这里**不**断言"永远到不了 0" —— 那只是 0.30 上限的副产物,
        不是安全性质。上限放开到整程后 0 就是正常的完全闭合。
        """
        counts = hd.closure_to_counts((1.0,) * 6)
        expected = round(hd.COUNT_OPEN * (1.0 - hm.REAL_HAND_MAX_RANGE_SCALE))
        for value in counts:
            self.assertAlmostEqual(value, expected, delta=1)
            self.assertGreaterEqual(value, hd.COUNT_CLOSED)
            self.assertLessEqual(value, hd.COUNT_OPEN)

    def test_a_conservative_range_still_stops_short(self):
        """--hand-range 0.3 必须仍然抓不死 —— 放开上限不能让它失效。"""
        counts = hd.closure_to_counts(
            (1.0,) * 6, range_scale=hm.CONSERVATIVE_RANGE_SCALE)
        for value in counts:
            self.assertGreater(value, hd.COUNT_CLOSED)

    def test_caller_cannot_exceed_the_cap(self):
        """板载机这条路和 PC 那条路守的是同一个上限。"""
        self.assertEqual(hd.closure_to_counts((1.0,) * 6, range_scale=1.0),
                         hd.closure_to_counts((1.0,) * 6))

    def test_matches_the_pc_side_conversion(self):
        """两条真手通路必须逐通道一致, 否则换个拓扑手感就变了。"""
        import inspire_hand_ctrl as ihc
        for closure in ((0.0,) * 6, (0.5,) * 6, (1.0,) * 6,
                        (0.1, 0.3, 0.5, 0.7, 0.9, 0.2)):
            self.assertEqual(hd.closure_to_counts(closure),
                             ihc.closure_to_angle_set(closure))

    def test_channel_order_is_preserved(self):
        counts = hd.closure_to_counts((1.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertLess(counts[0], hd.COUNT_OPEN)
        self.assertEqual(counts[1:], [hd.COUNT_OPEN] * 5)

    def test_step_is_rate_limited_both_ways(self):
        self.assertEqual(hd.step_toward(OPEN, [0] * 6, 60), [940] * 6)
        self.assertEqual(hd.step_toward([0] * 6, OPEN, 60), [60] * 6)

    def test_step_arrives_when_close(self):
        self.assertEqual(hd.step_toward(OPEN, [980] * 6, 60), [980] * 6)

    def test_step_output_is_clamped(self):
        for value in hd.step_toward([10] * 6, [-500] * 6, 60):
            self.assertGreaterEqual(value, hd.COUNT_CLOSED)

    def test_step_rejects_wrong_channel_count(self):
        with self.assertRaises(ValueError):
            hd.step_toward([1000] * 3, [0] * 6)
        with self.assertRaises(ValueError):
            hd.step_toward(OPEN, [0] * 3)


# ======================== HandFollower ========================

class _FakeTransport:
    """记录写了什么, 不碰网络。可以按需装死来模拟掉线。"""

    def __init__(self):
        self.writes = []
        self.connected = False
        self.connect_calls = 0
        self.closed_count = 0
        self.fail_connect = False
        self.fail_write = False

    def connect(self):
        self.connect_calls += 1
        if self.fail_connect:
            raise hd.ModbusError("连不上")
        self.connected = True

    def write_angles(self, counts):
        if self.fail_write:
            raise hd.ModbusError("写超时")
        self.writes.append(list(counts))

    def close(self):
        if self.connected:
            self.closed_count += 1
        self.connected = False


def make_follower(transport=None, **kwargs):
    return hd.HandFollower("right", transport=transport or _FakeTransport(),
                           **kwargs)


class FollowerTest(unittest.TestCase):
    """线程不启动, 直接驱动 _step —— 时间可控, 不靠 sleep 赌时序。"""

    def test_starts_open_not_zero(self):
        """零在这个量程里是握死。起始必须是张开。"""
        follower = make_follower()
        self.assertEqual(follower._current, OPEN)

    def test_no_data_walks_open(self):
        transport = _FakeTransport()
        follower = make_follower(transport)
        follower.update((1.0,) * 6, now=0.0)
        for i in range(6):
            follower._step(i * 0.02)
        gripped = list(follower._current)
        self.assertLess(gripped[0], hd.COUNT_OPEN)

        follower.update(None, now=1.0)
        follower._step(1.0)
        for now, before in zip(follower._current, gripped):
            self.assertGreater(now, before)

    def test_stale_data_walks_open(self):
        """发送端整个不发包时 update() 根本不会被调用, 只能靠时间戳发现。"""
        transport = _FakeTransport()
        follower = make_follower(transport, timeout_sec=0.5)
        follower.update((1.0,) * 6, now=0.0)
        for i in range(6):
            follower._step(i * 0.02)
        gripped = list(follower._current)

        # 时间跳过超时窗口, 但闭合度还是上一帧那个"握紧"。
        follower._step(10.0)
        for now, before in zip(follower._current, gripped):
            self.assertGreater(now, before)

    def test_closing_takes_several_frames(self):
        transport = _FakeTransport()
        follower = make_follower(transport, max_step=60)
        follower.update((1.0,) * 6, now=0.0)
        follower._step(0.0)
        self.assertEqual(transport.writes[-1], [940] * 6)
        follower._step(0.02)
        self.assertEqual(transport.writes[-1], [880] * 6)

    def test_connects_lazily_and_writes(self):
        transport = _FakeTransport()
        follower = make_follower(transport)
        self.assertEqual(transport.connect_calls, 0)
        follower._step(0.0)
        self.assertEqual(transport.connect_calls, 1)
        self.assertTrue(transport.writes)

    def test_absent_hand_does_not_raise(self):
        """手不在场只该让手起不来, 绝不能把接收端拖崩。"""
        transport = _FakeTransport()
        transport.fail_connect = True
        follower = make_follower(transport)
        for i in range(5):
            follower._step(i * 0.02)
        self.assertEqual(transport.writes, [])
        self.assertIn("断开", follower.status())

    def test_reconnect_is_rate_limited(self):
        """手断电时不能变成一秒几十次的连接风暴。"""
        transport = _FakeTransport()
        transport.fail_connect = True
        follower = make_follower(transport)
        for i in range(50):
            follower._step(i * 0.02)      # 1 秒
        self.assertLessEqual(transport.connect_calls, 2)

    def test_reconnect_sends_open_first(self):
        """断链期间内部状态照样朝张开推进, 所以重连后不会突然接着握。"""
        transport = _FakeTransport()
        follower = make_follower(transport, max_step=60)
        follower.update((1.0,) * 6, now=0.0)
        for i in range(6):
            follower._step(i * 0.02)
        gripped = list(follower._current)

        transport.fail_write = True
        follower._step(0.2)
        self.assertFalse(transport.connected)
        # 断链那一刻手停在这里。基准要取这个而不是 gripped —— 断链前那一帧
        # 也在继续收紧, 拿 gripped 比会在某些行程设置下刚好相等而误判。
        dropped = list(follower._current)
        self.assertLess(dropped[0], gripped[0])

        # 断链期间没有新数据 → 超时 → 朝张开走。
        transport.fail_write = False
        before = len(transport.writes)
        for i in range(20):
            follower._step(2.0 + i * 0.02)
        self.assertGreater(len(transport.writes), before)
        self.assertGreater(transport.writes[before][0], dropped[0])

    def test_write_failure_closes_the_socket(self):
        transport = _FakeTransport()
        follower = make_follower(transport)
        follower._step(0.0)
        transport.fail_write = True
        follower._step(0.02)
        self.assertFalse(transport.connected)
        self.assertIn("断开", follower.status())

    def test_stop_opens_the_hand(self):
        transport = _FakeTransport()
        follower = make_follower(transport, max_step=60, rate_hz=1000.0)
        follower.update((1.0,) * 6, now=0.0)
        for i in range(20):
            follower._step(i * 0.02)
        self.assertLess(follower._current[0], hd.COUNT_OPEN)

        follower.stop()
        self.assertEqual(transport.writes[-1], OPEN)
        self.assertFalse(transport.connected)

    def test_stop_survives_a_dead_link(self):
        """链路已经断了就张不开了, 但绝不能因此抛异常挡住退出流程。"""
        transport = _FakeTransport()
        follower = make_follower(transport)
        follower._step(0.0)
        transport.fail_write = True
        follower.stop()      # 不抛

    def test_bad_side_rejected(self):
        with self.assertRaises(ValueError):
            hd.HandFollower("both", transport=_FakeTransport())

    def test_official_ips(self):
        self.assertEqual(hd.HAND_IP["left"], "192.168.123.210")
        self.assertEqual(hd.HAND_IP["right"], "192.168.123.211")

    def test_thread_start_and_stop(self):
        """真起一次线程, 确认它自己会连、会写、退出时会张开。"""
        transport = _FakeTransport()
        follower = make_follower(transport, rate_hz=200.0)
        follower.start()
        try:
            deadline = time.monotonic() + 2.0
            while not transport.writes and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertTrue(transport.writes, "线程起来后应该开始下发")
        finally:
            follower.stop()
        self.assertEqual(transport.writes[-1], OPEN)


class HapticOverrideTest(unittest.TestCase):
    """不开TCP socket，直接验证逐指覆盖和超时回退。"""

    def make_override(self):
        return hd.HapticOverrideServer(
            "127.0.0.1", 0, timeout=0.5,
            lock_retreat=0, max_step=60, log=lambda _msg: None)

    def test_locked_finger_applies_step_only_to_its_slot(self):
        override = self.make_override()
        with override.lock:
            override.states = ["FREE", "LOCKED", "FREE", "FREE", "FREE"]
            override.steps = [0.0, 3.0, 0.0, 0.0, 0.0]
            override.updated = time.monotonic()
        glove = [800] * 6
        result = override.apply(glove, [800] * 6)
        self.assertEqual(result[hd.FINGER_ANGLE_SLOTS[1]], 797)
        for slot in (0, 1, 2, 4, 5):
            self.assertEqual(result[slot], 800)

    def test_stale_override_returns_glove_target(self):
        override = self.make_override()
        with override.lock:
            override.states = ["LOCKED"] * 5
            override.steps = [10.0] * 5
            override.updated = time.monotonic() - 1.0
        glove = [900] * 6
        self.assertEqual(override.apply(glove, [700] * 6), glove)
        self.assertIsNone(override.states)


class ModbusInteropTest(unittest.TestCase):
    """拿 pymodbus 的**服务端**当对照, 验证手写的裸帧确实是合法 Modbus。

    其余测试用的是本文件里的假 transport, 那等于自己验自己 —— 帧格式写错
    了也测不出来。pymodbus 是完全独立的实现, 它认这些帧才算数。

    pymodbus 只装在 PC端 的 venv 里 (机器人上不了外网, 装不了, 这正是
    hand_driver 手写帧的原因), 所以没有就跳过。
    """

    def _start_server(self):
        import asyncio
        import threading
        import warnings

        with warnings.catch_warnings():
            # 3.14 把 datastore 那套标记为 v4 移除, 这里只是测试脚手架。
            warnings.simplefilter("ignore", DeprecationWarning)
            from pymodbus.server import ModbusTcpServer
            from pymodbus.datastore import (ModbusDeviceContext,
                                            ModbusServerContext,
                                            ModbusSequentialDataBlock)
            block = ModbusSequentialDataBlock(1, [0] * 2000)
            context = ModbusServerContext(
                devices=ModbusDeviceContext(hr=block), single=True)

        import socket as _socket
        probe = _socket.socket()
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()

        loop = asyncio.new_event_loop()
        ready = threading.Event()
        holder = {}

        def serve():
            asyncio.set_event_loop(loop)

            async def main():
                server = ModbusTcpServer(context, address=("127.0.0.1", port))
                holder["server"] = server
                ready.set()
                await server.serve_forever()

            try:
                loop.run_until_complete(main())
            except Exception:
                ready.set()

        thread = threading.Thread(target=serve, daemon=True)
        thread.start()
        self.assertTrue(ready.wait(10.0), "Modbus 测试服务端没起来")
        time.sleep(0.3)

        def cleanup():
            server = holder.get("server")
            if server is not None:
                asyncio.run_coroutine_threadsafe(
                    server.shutdown(), loop).result(timeout=5)
            loop.call_soon_threadsafe(loop.stop)
            thread.join(timeout=5)

        self.addCleanup(cleanup)
        return port

    def test_roundtrip_against_an_independent_implementation(self):
        try:
            import pymodbus.server  # noqa: F401
        except ImportError:
            self.skipTest("没装 pymodbus")

        port = self._start_server()
        hand = hd.ModbusTcpHand("127.0.0.1", port=port, timeout=2.0)
        hand.connect()
        self.addCleanup(hand.close)

        # connect() 自带的验证读已经过了一次 FC 0x03。
        self.assertEqual(hand.read_angles(), [0] * 6)

        written = [1000, 900, 800, 700, 600, 500]
        hand.write_angles(written)
        frame = hand._transact(hd.build_read_request(
            hand._next_transaction(), hand.device_id,
            hd.REGISTER_ANGLE_SET, 6))
        self.assertEqual(hd.decode_read_response(frame, 6), written,
                         "写进 ANGLE_SET 的值必须能原样读回")

    def test_wire_bytes_match_the_proven_pymodbus_client(self):
        """和 mhandpro/standalone_inspire_bridge.py 发出的字节必须完全一样。

        那份用 pymodbus, 已经在左手上实测跑通过。这里抓 pymodbus 真正写到
        socket 上的字节, 和手写的帧逐字节比 —— 这是整条路上风险最高的假设:
        Modbus 的寄存器编号有 1-based/0-based 两套惯例, 差一位就会打到别的
        寄存器上, 而症状是"手不动"或者更糟的"手乱动", 极难反查。
        """
        try:
            from pymodbus.client import ModbusTcpClient
        except ImportError:
            self.skipTest("没装 pymodbus")

        import socket as _socket
        import threading

        server = _socket.socket()
        server.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", 0))
        port = server.getsockname()[1]
        server.listen(1)
        self.addCleanup(server.close)
        captured = []

        def capture():
            try:
                conn, _ = server.accept()
            except OSError:
                return
            conn.settimeout(3.0)
            try:
                request = conn.recv(4096)
                if not request:
                    return
                captured.append(request)
                # 回一个合法的 FC16 响应, 免得客户端干等到超时。
                txn = struct.unpack("!H", request[0:2])[0]
                addr, count = struct.unpack("!HH", request[8:12])
                pdu = struct.pack("!BHH", 0x10, addr, count)
                conn.sendall(struct.pack("!HHHB", txn, 0, len(pdu) + 1,
                                         request[6]) + pdu)
            except OSError:
                pass
            finally:
                conn.close()

        thread = threading.Thread(target=capture, daemon=True)
        thread.start()

        values = [1000, 900, 800, 700, 600, 500]
        client = ModbusTcpClient("127.0.0.1", port=port, timeout=2.0)
        client.connect()
        try:
            client.write_registers(hd.REGISTER_ANGLE_SET, values, device_id=1)
        finally:
            client.close()
        thread.join(timeout=5)

        self.assertTrue(captured, "没抓到 pymodbus 发出的帧")
        reference = captured[0]
        txn = struct.unpack("!H", reference[0:2])[0]
        mine = hd.build_write_request(txn, 1, hd.REGISTER_ANGLE_SET, values)
        self.assertEqual(mine, reference,
                         f"帧不一致\n  pymodbus: {reference.hex(' ')}\n"
                         f"  hand_driver: {mine.hex(' ')}")
        # 寄存器号在线上是 1486 原值, 没有 1-based 偏移。
        self.assertEqual(reference[8:10], bytes([0x05, 0xCE]))

    def test_connect_refuses_a_closed_port(self):
        """只 connect 成功不算数, 但连都连不上时必须立刻抛。"""
        import socket as _socket
        probe = _socket.socket()
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()

        hand = hd.ModbusTcpHand("127.0.0.1", port=port, timeout=0.5)
        with self.assertRaises(OSError):
            hand.connect()
        self.assertFalse(hand.connected)


class ReceiverWiringTest(unittest.TestCase):
    """接收端的 --hand 解析。整个 robot_arm_receiver 导入不了 (依赖
    unitree_sdk2py), 所以只把这个纯函数抠出来测。"""

    @staticmethod
    def _sides_from_arg():
        import ast
        path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "机器人端", "robot_arm_receiver.py")
        with open(path, encoding="utf-8") as handle:
            tree = ast.parse(handle.read())
        for node in tree.body:
            if (isinstance(node, ast.FunctionDef)
                    and node.name == "hand_sides_from_arg"):
                namespace = {}
                exec(compile(ast.Module([node], []), path, "exec"), namespace)
                return namespace["hand_sides_from_arg"]
        raise AssertionError("robot_arm_receiver 里找不到 hand_sides_from_arg")

    def test_default_drives_no_hand(self):
        """默认必须一只手都不驱动 —— 手指会夹人。"""
        self.assertEqual(self._sides_from_arg()(None), ())

    def test_single_and_both(self):
        sides = self._sides_from_arg()
        self.assertEqual(sides("right"), ("right",))
        self.assertEqual(sides("left"), ("left",))
        self.assertEqual(sides("both"), ("right", "left"))


if __name__ == "__main__":
    unittest.main()
