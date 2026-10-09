#!/usr/bin/env python3
"""不连硬件验证双手 ARM 前置检查和终端回馈。"""

from __future__ import annotations

import contextlib
import io
import sys
import threading
import types
import unittest
from pathlib import Path


# CI/开发电脑可能没有安装 Dynamixel SDK。被测逻辑不实例化
# SDK 类，因此在导入阶段提供最小占位符即可。
try:
    import dynamixel_sdk  # noqa: F401
except ModuleNotFoundError:
    sdk = types.ModuleType("dynamixel_sdk")
    sdk.COMM_SUCCESS = 0
    sdk.PacketHandler = object
    sdk.PortHandler = object
    sys.modules["dynamixel_sdk"] = sdk

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from dual_hand_force_test import DualHandController  # noqa: E402


class FakeFinger:
    def __init__(self, name: str, servo_id: int, *, initialized: bool = True):
        self.finger_name = name
        self.servo_id = servo_id
        self.init_pos = 100 if initialized else None
        self.fsr_rest_raw = 4.9 if initialized else None
        self.armed = False


class FakeHand:
    def __init__(self, servo_id: int, *, ready: bool = True,
                 reason: str = "", initialized: bool = True):
        self.args = types.SimpleNamespace(contact_on=0.5)
        self.ready = ready
        self.reason = reason
        self.fingers = [FakeFinger("拇指", servo_id, initialized=initialized)]
        self.stop_reason = ""

    def data_ready(self):
        return self.ready, self.reason

    def arm_all(self):
        for finger in self.fingers:
            finger.armed = True

    def stop_all(self, reason):
        self.stop_reason = reason
        for finger in self.fingers:
            finger.armed = False


def make_controller(right: FakeHand, left: FakeHand):
    controller = DualHandController.__new__(DualHandController)
    controller.hands = {"right": right, "left": left}
    return controller


class DualHandArmTest(unittest.TestCase):
    def test_data_blocker_is_printed_instead_of_silent_return(self):
        right = FakeHand(1, ready=False, reason="拇指: Inspire力数据超时")
        left = FakeHand(6)
        controller = make_controller(right, left)

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = controller.arm_all()

        self.assertFalse(result)
        self.assertIn("[ARM拒绝]", output.getvalue())
        self.assertIn("右手数据未就绪", output.getvalue())
        self.assertIn("Inspire力数据超时", output.getvalue())
        self.assertIn("十指保持STOP", output.getvalue())
        self.assertTrue(right.stop_reason)
        self.assertTrue(left.stop_reason)

    def test_missing_init_names_hand_and_finger(self):
        right = FakeHand(1, initialized=False)
        left = FakeHand(6)
        controller = make_controller(right, left)

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = controller.arm_all()

        self.assertFalse(result)
        self.assertIn("右手未INIT: 拇指(ID1)", output.getvalue())

    def test_success_is_explicit_and_arms_both_hands(self):
        right = FakeHand(1)
        left = FakeHand(6)
        controller = make_controller(right, left)

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = controller.arm_all()

        self.assertTrue(result)
        self.assertTrue(right.fingers[0].armed)
        self.assertTrue(left.fingers[0].armed)
        self.assertIn("[ARM成功]", output.getvalue())
        self.assertIn("0.50N", output.getvalue())


class FakeTelemetryHand:
    def __init__(self, *, armed: bool, faulted: bool, health_age_ms=10):
        self.armed = armed
        self.faulted = faulted
        self.health_age_ms = health_age_ms

    def telemetry_snapshot(self, _now):
        return {
            "label": "测试手",
            "armed": self.armed,
            "faulted": self.faulted,
            "ble_fsr": {},
            "inspire_feedback": {},
            "haptic_override": {},
            "fingers": [{
                "armed": self.armed,
                "servo": {"health_age_ms": self.health_age_ms},
            }],
        }


class TelemetrySnapshotTest(unittest.TestCase):
    def make_controller(self, right, left):
        controller = DualHandController.__new__(DualHandController)
        controller.args = types.SimpleNamespace(
            enable_write=True,
            enable_mhandpro=True,
            device="/dev/serial0",
            baudrate=1_000_000,
        )
        controller.dxl_lock = threading.RLock()
        controller.hands = {"right": right, "left": left}
        return controller

    def test_snapshot_reports_armed_and_serial_health_from_cache(self):
        controller = self.make_controller(
            FakeTelemetryHand(armed=True, faulted=False),
            FakeTelemetryHand(armed=True, faulted=False),
        )

        snapshot = controller.telemetry_snapshot()

        self.assertEqual(snapshot["schema_version"], 1)
        self.assertEqual(snapshot["system"]["state"], "ARMED")
        self.assertTrue(snapshot["system"]["serial"]["connected"])
        self.assertEqual(snapshot["system"]["serial"]["owner"],
                         "dual_hand_force_test")

    def test_fault_takes_priority_and_missing_health_marks_serial_down(self):
        controller = self.make_controller(
            FakeTelemetryHand(armed=True, faulted=False),
            FakeTelemetryHand(
                armed=False, faulted=True, health_age_ms=None),
        )

        snapshot = controller.telemetry_snapshot()

        self.assertEqual(snapshot["system"]["state"], "FAULT")
        self.assertFalse(snapshot["system"]["serial"]["connected"])


class FakeStartHand:
    def __init__(self):
        self.fsr_starts = 0
        self.g1_starts = 0
        self.loop_calls = 0
        self.loop_event = threading.Event()

    def start_fsr_input(self):
        self.fsr_starts += 1

    def start_g1_inputs(self):
        self.g1_starts += 1

    def loop(self):
        self.loop_calls += 1
        self.loop_event.set()


class SplitStartTest(unittest.TestCase):
    def make_controller(self):
        controller = DualHandController.__new__(DualHandController)
        controller.running = False  # 联锁线程会立即退出
        controller.control_started = False
        controller.fsr_inputs_started = False
        controller.g1_inputs_started = False
        controller.hands = {
            "right": FakeStartHand(),
            "left": FakeStartHand(),
        }
        return controller

    def test_network_sources_start_once(self):
        controller = self.make_controller()

        self.assertTrue(controller.start_fsr_inputs())
        self.assertFalse(controller.start_fsr_inputs())
        self.assertTrue(controller.start_g1_inputs())
        self.assertFalse(controller.start_g1_inputs())

        for hand in controller.hands.values():
            self.assertEqual(hand.fsr_starts, 1)
            self.assertEqual(hand.g1_starts, 1)

    def test_control_loop_start_is_idempotent(self):
        controller = self.make_controller()

        self.assertTrue(controller.start_control())
        self.assertFalse(controller.start_control())

        for hand in controller.hands.values():
            self.assertTrue(hand.loop_event.wait(timeout=1.0))
            self.assertEqual(hand.loop_calls, 1)


if __name__ == "__main__":
    unittest.main()
