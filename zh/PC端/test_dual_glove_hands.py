"""双手套 → 双手仿真这条链路的回归测试。

两只 mHandPro 手套共用一个接收器和一个 /dev/ttyUSB*, 所以上游只能是**一个**
``mhandpro_diagnostic`` 进程 (``teleop both``), 下游才分成左右两条 TCP。这里
钉住的是分叉之后的部分: 两侧配置不许撞端口、闭合度换算到满行程、协议尾块两
只手都带得上、以及一侧掉线不影响另一侧。

手套本身不在环里 —— 这些都是纯数据换算, 不插手套也能跑。
"""

import importlib.util
import json
import os
import sys
import types
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import hand_mapping as hm
from glove_source import load_endpoints, load_port, parse_cfg, parse_ctrl_line
from teleop_protocol import pack_arm_command


_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
RIGHT_CFG = os.path.join(_ROOT, "mhandpro", "config", "inspire_right_sim.cfg")
LEFT_CFG = os.path.join(_ROOT, "mhandpro", "config", "inspire_left_sim.cfg")

_SPEC = importlib.util.spec_from_file_location(
    "robot_teleop_protocol",
    os.path.join(_ROOT, "机器人端", "teleop_protocol.py"))
robot_protocol = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(robot_protocol)


class SimConfigPairTest(unittest.TestCase):
    """两份仿真配置必须成对、同构、且端口不同。"""

    def test_both_configs_exist(self):
        self.assertTrue(os.path.isfile(RIGHT_CFG), RIGHT_CFG)
        self.assertTrue(os.path.isfile(LEFT_CFG), LEFT_CFG)

    def test_ports_differ(self):
        """撞端口的话两只手收到同一份数据, 症状是'另一只手跟着这只动'。"""
        self.assertNotEqual(load_port(RIGHT_CFG), load_port(LEFT_CFG))

    def test_full_span_endpoints(self):
        """满量程 0~1000 是仿真配置的关键: ticks 就等于闭合度×1000,

        于是不必先实测真手的行程端点, 仿真就能先跑起来。
        """
        for path in (RIGHT_CFG, LEFT_CFG):
            opened, closed = load_endpoints(path)
            self.assertEqual(opened, (1000,) * hm.CHANNEL_COUNT, path)
            self.assertEqual(closed, (0,) * hm.CHANNEL_COUNT, path)

    def test_range_scale_is_full_travel(self):
        for path in (RIGHT_CFG, LEFT_CFG):
            self.assertEqual(float(parse_cfg(path)["range_scale"]), 1.0, path)

    def test_mag_fault_downgraded_only_here(self):
        """仿真配置放宽 BAD_MAG; 真手配置绝不允许。"""
        for path in (RIGHT_CFG, LEFT_CFG):
            self.assertEqual(int(parse_cfg(path)["mag_fault_fatal"]), 0, path)
        real_left = os.path.join(_ROOT, "mhandpro", "config",
                                 "inspire_left.cfg")
        if os.path.isfile(real_left):
            self.assertNotEqual(
                int(parse_cfg(real_left).get("mag_fault_fatal", 1)), 0,
                "驱动真手的配置不能把 BAD_MAG 降级")


class FullTravelChainTest(unittest.TestCase):
    """闭合度从手套一路到 URDF 关节角, 全程不许被二次缩放。"""

    def _ticks_for(self, command, cfg_path):
        """复刻 C++ to_inspire_ticks(): range_scale 已经是 1.0。"""
        cfg = parse_cfg(cfg_path)
        opened, closed = load_endpoints(cfg_path)
        scale = float(cfg["range_scale"])
        ticks = []
        for i in range(hm.CHANNEL_COUNT):
            limited = opened[i] + scale * (closed[i] - opened[i])
            ticks.append(round(opened[i] + command[i] * (limited - opened[i])))
        return tuple(ticks)

    def test_ticks_round_trip_is_identity(self):
        """满量程配置下 ticks→闭合度必须还原成手套发出的六维命令。"""
        for cfg_path in (RIGHT_CFG, LEFT_CFG):
            opened, closed = load_endpoints(cfg_path)
            for command in ((0.0,) * 6, (1.0,) * 6,
                            (0.25, 0.5, 0.75, 1.0, 0.0, 0.6)):
                ticks = self._ticks_for(command, cfg_path)
                closure = hm.glove_to_closure(ticks, opened, closed)
                for actual, expected in zip(closure, command):
                    self.assertAlmostEqual(actual, expected, places=3,
                                           msg=f"{cfg_path} {command}")

    def test_closed_fist_reaches_full_urdf_travel(self):
        """握满拳时仿真里也要握满 —— 这就是"行程 100%"的定义。"""
        for side in ("right", "left"):
            angles = hm.closure_to_angles((1.0,) * hm.CHANNEL_COUNT, side=side)
            for actual, upper in zip(angles, hm.JOINT_UPPER):
                self.assertAlmostEqual(actual, upper, places=6, msg=side)

    def test_sim_scale_is_one(self):
        self.assertEqual(hm.SIM_RANGE_SCALE, 1.0)
        self.assertEqual(hm.REAL_HAND_MAX_RANGE_SCALE, 1.0)

    def test_ctrl_line_from_both_sides_parses(self):
        """C++ 那头发的就是这一行 JSON, 左右手格式完全一样。"""
        for cfg_path in (RIGHT_CFG, LEFT_CFG):
            ticks = self._ticks_for((0.5,) * 6, cfg_path)
            line = json.dumps({"type": "ctrl", "angle_set": list(ticks)})
            self.assertEqual(parse_ctrl_line(line), ticks)


class PhysicalAngleTest(unittest.TestCase):
    """闭合度 ↔ 因时手册物理角度 (PRJ-02-TS-U-010 第22页)。"""

    def test_table_matches_datasheet(self):
        for i in range(4):
            self.assertEqual(hm.INSPIRE_ANGLE_RANGE_DEG[i], (20.0, 176.0))
        self.assertEqual(hm.INSPIRE_ANGLE_RANGE_DEG[4], (70.0, -13.0))
        self.assertEqual(
            sorted(hm.INSPIRE_ANGLE_RANGE_DEG[5]), [90.0, 165.0])

    def test_open_and_closed_ends(self):
        opened = hm.closure_to_physical_deg((0.0,) * 6)
        closed = hm.closure_to_physical_deg((1.0,) * 6)
        for i, (closed_deg, open_deg) in enumerate(hm.INSPIRE_ANGLE_RANGE_DEG):
            self.assertAlmostEqual(opened[i], open_deg, places=6)
            self.assertAlmostEqual(closed[i], closed_deg, places=6)

    def test_fingers_get_smaller_when_closing(self):
        """∠α 是与掌骨平面的夹角: 握紧时角度变小, 不是变大。"""
        half = hm.closure_to_physical_deg((0.5,) * 6)
        self.assertAlmostEqual(half[0], 98.0, places=6)   # (20+176)/2
        self.assertLess(half[0], hm.closure_to_physical_deg((0.0,) * 6)[0])

    def test_thumb_flex_grows_when_closing(self):
        """拇指弯曲 ∠θ 方向相反, 越弯数值越大。"""
        self.assertGreater(hm.closure_to_physical_deg((1.0,) * 6)[4],
                           hm.closure_to_physical_deg((0.0,) * 6)[4])

    def test_round_trip(self):
        for closure in ((0.0,) * 6, (1.0,) * 6, (0.1, 0.3, 0.5, 0.7, 0.9, 0.2)):
            back = hm.physical_deg_to_closure(
                hm.closure_to_physical_deg(closure))
            for actual, expected in zip(back, closure):
                self.assertAlmostEqual(actual, expected, places=6)

    def test_display_only_does_not_touch_command_paths(self):
        """物理角度表只作显示; 换掉它不许影响下发的计数和关节角。"""
        counts = hm.closure_to_counts((0.5,) * 6, (1000,) * 6, (0,) * 6)
        angles = hm.closure_to_angles((0.5,) * 6)
        original = hm.INSPIRE_ANGLE_RANGE_DEG
        try:
            hm.INSPIRE_ANGLE_RANGE_DEG = ((0.0, 1.0),) * 6
            self.assertEqual(
                hm.closure_to_counts((0.5,) * 6, (1000,) * 6, (0,) * 6), counts)
            self.assertEqual(hm.closure_to_angles((0.5,) * 6), angles)
        finally:
            hm.INSPIRE_ANGLE_RANGE_DEG = original


class DualHandPacketTest(unittest.TestCase):
    """UHND 尾块要能同时带上两只手, 并被机器人端原样读出来。"""

    def _pack(self, right, left):
        return pack_arm_command(3, 1, 2, (0.0,) * 10, (0.0,) * 10,
                                waist=(0.1, 0.0, 0.0), waist_enabled=True,
                                right_hand=right, left_hand=left)

    def test_both_hands_round_trip(self):
        right = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
        left = (0.9, 0.8, 0.7, 0.6, 0.5, 0.4)
        decoded = robot_protocol.unpack_arm_command(self._pack(right, left))
        hands = decoded["hands"]
        for actual, expected in zip(hands["right"], right):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(hands["left"], left):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_waist_survives_alongside_two_hands(self):
        """腰块必须留在固定偏移上 —— 手部尾块排在它后面。"""
        decoded = robot_protocol.unpack_arm_command(
            self._pack((0.5,) * 6, (0.5,) * 6))
        self.assertIsNotNone(decoded["waist"])
        self.assertAlmostEqual(decoded["waist"][0], 0.1, places=5)

    def test_one_hand_missing_stays_none(self):
        """"没有数据"和"张开"必须是两回事, 缺的那只不能变成 0。"""
        decoded = robot_protocol.unpack_arm_command(
            self._pack((0.3,) * 6, None))
        self.assertIsNone(decoded["hands"]["left"])
        self.assertIsNotNone(decoded["hands"]["right"])

    def test_no_hand_block_when_both_missing(self):
        without = self._pack(None, None)
        decoded = robot_protocol.unpack_arm_command(without)
        self.assertTrue(decoded["hands"] is None
                        or (decoded["hands"]["right"] is None
                            and decoded["hands"]["left"] is None))


class _FakeGlove:
    def __init__(self, closure):
        self._closure = closure

    def latest_closure(self):
        return self._closure


def _viz_stub(right_cfg=RIGHT_CFG, left_cfg=LEFT_CFG,
              right_glove=None, left_glove=None, real_hand_side=None,
              real_hand=None):
    """只带手部逻辑用到的字段的替身。

    ``DualArmViz`` 的构造要起 pygame/OpenGL, 单测里不碰它 —— 这里直接用未绑定
    方法作用在替身上, 测的是手部逻辑本身。
    """
    stub = types.SimpleNamespace(
        _hand_cfgs={"right": right_cfg, "left": left_cfg},
        gloves={"right": right_glove, "left": left_glove},
        _hand_render_closure={"right": None, "left": None},
        hand_enabled=(right_cfg is not None or left_cfg is not None),
        real_hand=real_hand,
        _real_hand_side=real_hand_side,
        _log=lambda *a, **k: None,
    )
    return stub


class HandRenderAndSnapshotTest(unittest.TestCase):
    """左右两条链在上位机里必须真的独立。"""

    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        import dual_arm_viz
        cls.viz = dual_arm_viz.DualArmViz

    def test_render_joints_cover_both_hands(self):
        joints = self.viz._hand_render_joints(_viz_stub())
        self.assertEqual(len(joints), 24)
        self.assertEqual(sum(1 for n in joints if n.startswith("right_")), 12)
        self.assertEqual(sum(1 for n in joints if n.startswith("left_")), 12)

    def test_render_joints_single_side(self):
        joints = self.viz._hand_render_joints(_viz_stub(left_cfg=None))
        self.assertEqual(len(joints), 12)
        self.assertTrue(all(n.startswith("right_") for n in joints))

    def test_render_defaults_to_open_without_data(self):
        """没有数据时画张开的手, 而不是不画。"""
        joints = self.viz._hand_render_joints(_viz_stub())
        self.assertTrue(all(abs(a) < 1e-9 for a in joints.values()))

    def test_snapshot_sides_are_independent(self):
        """右手套掉线只让右手变 None, 左手照常跟随。"""
        left = (0.7,) * hm.CHANNEL_COUNT
        stub = _viz_stub(right_glove=None, left_glove=_FakeGlove(left))
        closures = self.viz._hand_snapshot(stub)
        self.assertIsNone(closures["right"])
        self.assertEqual(closures["left"], left)
        self.assertEqual(stub._hand_render_closure["left"], left)

    def test_snapshot_feeds_real_hand_from_matching_side(self):
        """--real-hand left 必须拿左手的闭合度, 不能拿成右手的。"""
        sent = []
        real = types.SimpleNamespace(send=sent.append)
        right = (0.2,) * hm.CHANNEL_COUNT
        left = (0.8,) * hm.CHANNEL_COUNT
        stub = _viz_stub(right_glove=_FakeGlove(right),
                         left_glove=_FakeGlove(left),
                         real_hand_side="left", real_hand=real)
        self.viz._hand_snapshot(stub)
        self.assertEqual(sent, [left])

    def test_snapshot_sends_none_to_real_hand_when_side_dropped(self):
        """真手每帧都要收到指令; None 让它自己朝张开推进, 不是停在原姿态。"""
        sent = []
        real = types.SimpleNamespace(send=sent.append)
        stub = _viz_stub(right_glove=None, left_glove=None,
                         real_hand_side="right", real_hand=real)
        self.viz._hand_snapshot(stub)
        self.assertEqual(sent, [None])


if __name__ == "__main__":
    unittest.main()
