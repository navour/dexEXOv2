#!/usr/bin/env python3
"""inspire_hand_ctrl 的单测。

这一层直接决定真手会不会握死，所以方向、量程、限速、失效回退全部钉住。
DDS 发布本身依赖 unitree_sdk2py / inspire_sdkpy，PC 上通常没装，所以换算
逻辑刻意和发布分开 —— 这里测的全是不依赖 SDK 的那部分。
"""

import unittest

import hand_mapping as hm
import inspire_hand_ctrl as ihc


class TopicTest(unittest.TestCase):
    def test_official_topics(self):
        self.assertEqual(ihc.topic_for("right"), "rt/inspire_hand/ctrl/r")
        self.assertEqual(ihc.topic_for("left"), "rt/inspire_hand/ctrl/l")

    def test_bad_side_rejected(self):
        with self.assertRaises(ValueError):
            ihc.topic_for("both")


class ClosureToAngleSetTest(unittest.TestCase):
    def test_no_data_opens_the_hand(self):
        """None 是"这一帧没有手部数据", 必须张开而不是保持。"""
        self.assertEqual(ihc.closure_to_angle_set(None),
                         [ihc.ANGLE_OPEN] * 6)

    def test_open_closure_is_full_open(self):
        self.assertEqual(ihc.closure_to_angle_set(hm.open_closure()),
                         [ihc.ANGLE_OPEN] * 6)

    def test_direction_larger_is_more_open(self):
        """因时 angle_set 越大越张开; 闭合度越大越握紧, 方向必须相反。"""
        soft = ihc.closure_to_angle_set((0.2,) * 6)
        hard = ihc.closure_to_angle_set((0.9,) * 6)
        for a, b in zip(soft, hard):
            self.assertGreater(a, b)

    def test_full_closure_respects_the_real_hand_cap(self):
        """闭合度 1.0 落在 REAL_HAND_MAX_RANGE_SCALE 定义的位置。

        不断言"永远到不了 0" —— 那是 0.30 上限的副产物, 不是安全性质。
        """
        angles = ihc.closure_to_angle_set((1.0,) * 6)
        expected = round(ihc.ANGLE_OPEN
                         * (1.0 - hm.REAL_HAND_MAX_RANGE_SCALE))
        for value in angles:
            self.assertAlmostEqual(value, expected, delta=1)
            self.assertGreaterEqual(value, ihc.ANGLE_CLOSED)

    def test_a_conservative_range_still_stops_short(self):
        angles = ihc.closure_to_angle_set(
            (1.0,) * 6, range_scale=hm.CONSERVATIVE_RANGE_SCALE)
        for value in angles:
            self.assertGreater(value, ihc.ANGLE_CLOSED)

    def test_caller_cannot_exceed_the_cap(self):
        """调用方传 1.0 也绕不过 hand_mapping 的真手上限。"""
        self.assertEqual(ihc.closure_to_angle_set((1.0,) * 6, range_scale=1.0),
                         ihc.closure_to_angle_set((1.0,) * 6))

    def test_values_stay_in_range(self):
        for closure in ((0.0,) * 6, (1.0,) * 6, (0.5,) * 6):
            for value in ihc.closure_to_angle_set(closure):
                self.assertGreaterEqual(value, ihc.ANGLE_CLOSED)
                self.assertLessEqual(value, ihc.ANGLE_OPEN)

    def test_values_are_ints(self):
        for value in ihc.closure_to_angle_set((0.37,) * 6):
            self.assertIsInstance(value, int)

    def test_per_channel_order_is_preserved(self):
        """只握小指, 只有第 0 个通道该变小。"""
        angles = ihc.closure_to_angle_set((1.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertLess(angles[0], ihc.ANGLE_OPEN)
        self.assertEqual(angles[1:], [ihc.ANGLE_OPEN] * 5)


class StepTowardTest(unittest.TestCase):
    def test_small_difference_arrives_immediately(self):
        current = [1000] * 6
        target = [980] * 6
        self.assertEqual(
            ihc.step_toward_angle_set(current, target, max_step=60), target)

    def test_large_difference_is_rate_limited(self):
        current = [1000] * 6
        target = [0] * 6
        stepped = ihc.step_toward_angle_set(current, target, max_step=60)
        self.assertEqual(stepped, [940] * 6)

    def test_opening_is_rate_limited_too(self):
        """断流回张也必须是渐变的, 不能一帧弹开。"""
        stepped = ihc.step_toward_angle_set([0] * 6, [1000] * 6, max_step=60)
        self.assertEqual(stepped, [60] * 6)

    def test_output_is_clamped(self):
        stepped = ihc.step_toward_angle_set([10] * 6, [-500] * 6, max_step=60)
        for value in stepped:
            self.assertGreaterEqual(value, ihc.ANGLE_CLOSED)

    def test_converges_in_bounded_steps(self):
        current = [ihc.ANGLE_OPEN] * 6
        target = ihc.closure_to_angle_set((1.0,) * 6)
        for _ in range(100):
            current = ihc.step_toward_angle_set(current, target, max_step=60)
            if current == target:
                break
        self.assertEqual(current, target)

    def test_wrong_channel_count_rejected(self):
        with self.assertRaises(ValueError):
            ihc.step_toward_angle_set([1000] * 3, [0] * 6)
        with self.assertRaises(ValueError):
            ihc.step_toward_angle_set([1000] * 6, [0] * 3)


class SafetyChainTest(unittest.TestCase):
    def test_data_loss_walks_the_hand_open_not_shut(self):
        """握着的时候断流, 每一帧都必须朝张开走。"""
        current = ihc.closure_to_angle_set((1.0,) * 6)
        previous = list(current)
        for _ in range(5):
            current = ihc.step_toward_angle_set(
                current, ihc.closure_to_angle_set(None), max_step=60)
            for now, before in zip(current, previous):
                self.assertGreaterEqual(now, before)
            previous = list(current)
        self.assertGreater(current[0],
                           ihc.closure_to_angle_set((1.0,) * 6)[0])

    def test_sim_and_real_keep_separate_constants(self):
        """仿真和真手的行程是两个独立常量, 不能合并成一个。

        它们眼下都是 1.0, 但含义不同: 仿真那个是 URDF 关节上限, 真手那个是
        安全上限。合并的话, 哪天真手要收紧就会连带把仿真显示也砍了。
        """
        self.assertEqual(hm.SIM_RANGE_SCALE, 1.0)
        self.assertLessEqual(hm.REAL_HAND_MAX_RANGE_SCALE, 1.0)
        self.assertLess(hm.CONSERVATIVE_RANGE_SCALE,
                        hm.REAL_HAND_MAX_RANGE_SCALE)
        sim_angle = hm.closure_to_angles((1.0,) * 6)[0]
        self.assertAlmostEqual(sim_angle, hm.JOINT_UPPER[0])


class _FakeResponse:
    def __init__(self, registers=None, error=False):
        self.registers = registers or []
        self._error = error

    def isError(self):
        return self._error


class _FakeClient:
    """够用的 pymodbus 替身：记录写了什么，不碰网络。"""

    def __init__(self, *args, **kwargs):
        self.init_args = (args, kwargs)
        self.writes = []
        self.closed = False
        self.connected = True
        self.fail_write = False

    def connect(self):
        return self.connected

    def read_holding_registers(self, address, *, count=1, device_id=1,
                               **kwargs):
        self.last_read = (address, count, device_id)
        return _FakeResponse(registers=[500] * count)

    def write_registers(self, address, values, *, device_id=1, **kwargs):
        self.writes.append((address, list(values), device_id))
        return _FakeResponse(error=self.fail_write)

    def close(self):
        self.closed = True


class ModbusTransportTest(unittest.TestCase):
    """Modbus 直连通路。真手不在场也要能验证寄存器、方向和限速。"""

    def setUp(self):
        import pymodbus.client as pymodbus_client
        self._real = pymodbus_client.ModbusTcpClient
        self.fake = None

        def factory(*args, **kwargs):
            self.fake = _FakeClient(*args, **kwargs)
            return self.fake

        pymodbus_client.ModbusTcpClient = factory
        self.addCleanup(
            setattr, pymodbus_client, "ModbusTcpClient", self._real)

    def test_default_ips_match_the_official_doc(self):
        self.assertEqual(ihc.HAND_IP["left"], "192.168.123.210")
        self.assertEqual(ihc.HAND_IP["right"], "192.168.123.211")

    def test_connects_to_the_right_hand_by_default(self):
        hand = ihc.InspireHandModbus(side="right")
        self.assertEqual(self.fake.init_args[0][0], "192.168.123.211")
        self.assertEqual(self.fake.init_args[1]["port"], 6000)
        # 起始姿态必须是张开, 不是零 —— 零在这个量程里是握死。
        self.assertEqual(hand.current, [ihc.ANGLE_OPEN] * 6)

    def test_refuses_when_connect_fails(self):
        import pymodbus.client as pymodbus_client

        def failing(*args, **kwargs):
            client = _FakeClient(*args, **kwargs)
            client.connected = False
            return client

        pymodbus_client.ModbusTcpClient = failing
        with self.assertRaises(ConnectionError):
            ihc.InspireHandModbus(side="right")

    def test_writes_the_angle_set_register(self):
        hand = ihc.InspireHandModbus(side="right")
        hand.send(None)
        address, values, device_id = self.fake.writes[-1]
        self.assertEqual(address, ihc.REGISTER_ANGLE_SET)
        self.assertEqual(len(values), 6)
        self.assertEqual(device_id, 1)

    def test_reads_the_angle_act_register(self):
        hand = ihc.InspireHandModbus(side="right")
        self.assertEqual(hand.read_angles(), [500] * 6)
        self.assertEqual(self.fake.last_read,
                         (ihc.REGISTER_ANGLE_ACT, 6, 1))

    def test_closing_is_rate_limited_over_several_frames(self):
        hand = ihc.InspireHandModbus(side="right", max_step=60)
        first = hand.send((1.0,) * 6)
        self.assertEqual(first, [940] * 6)      # 一帧只走 60
        second = hand.send((1.0,) * 6)
        self.assertEqual(second, [880] * 6)

    def test_data_loss_walks_open_not_shut(self):
        hand = ihc.InspireHandModbus(side="right", max_step=60)
        for _ in range(6):
            hand.send((1.0,) * 6)
        before = list(hand.current)
        hand.send(None)
        for now, prior in zip(hand.current, before):
            self.assertGreater(now, prior)

    def test_open_and_close_leaves_the_hand_open(self):
        hand = ihc.InspireHandModbus(side="right", max_step=60)
        for _ in range(6):
            hand.send((1.0,) * 6)
        hand.open_and_close()
        self.assertEqual(hand.current, [ihc.ANGLE_OPEN] * 6)
        self.assertTrue(self.fake.closed, "退出时必须断开连接")

    def test_write_error_raises(self):
        hand = ihc.InspireHandModbus(side="right")
        self.fake.fail_write = True
        with self.assertRaises(IOError):
            hand.send((0.5,) * 6)

    def test_bad_side_rejected(self):
        with self.assertRaises(ValueError):
            ihc.InspireHandModbus(side="both")

    def test_connect_lying_is_caught_by_the_verification_read(self):
        """pymodbus 3.14 的 connect() 对不存在的地址也返回 True（实测）。

        所以构造时必须真读一次；否则失败会推迟到第一次下发动作时。
        """
        import pymodbus.client as pymodbus_client

        def lying(*args, **kwargs):
            client = _FakeClient(*args, **kwargs)

            def dead_read(*a, **kw):
                raise OSError("No response received after 3 retries")

            client.read_holding_registers = dead_read
            return client

        pymodbus_client.ModbusTcpClient = lying
        with self.assertRaises(ConnectionError) as caught:
            ihc.InspireHandModbus(side="right")
        self.assertIn("读不到寄存器", str(caught.exception))

    def test_read_timeout_becomes_ioerror(self):
        """超时是异常、错误响应是返回值，调用方只该处理一种。"""
        hand = ihc.InspireHandModbus(side="right")

        def dead_read(*a, **kw):
            raise OSError("timeout")

        self.fake.read_holding_registers = dead_read
        with self.assertRaises(IOError):
            hand.read_angles()

    def test_write_timeout_becomes_ioerror(self):
        hand = ihc.InspireHandModbus(side="right")

        def dead_write(*a, **kw):
            raise OSError("timeout")

        self.fake.write_registers = dead_write
        with self.assertRaises(IOError):
            hand.send((0.5,) * 6)


if __name__ == "__main__":
    unittest.main()
