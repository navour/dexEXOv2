#!/usr/bin/env python3
"""不连硬件验证FSR加载/卸载两阶段滞回。"""

from __future__ import annotations

import contextlib
import io
import sys
import types
import unittest
from pathlib import Path


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

from left_index_force_test import Controller  # noqa: E402


def make_controller() -> Controller:
    controller = Controller.__new__(Controller)
    controller.args = types.SimpleNamespace(
        fsr_load_threshold=0.30,
        exo_zero_threshold=0.15,
        fsr_release_hold_seconds=0.30,
        release_hold_seconds=0.15,
    )
    controller.finger_name = "拇指"
    controller.servo_id = 6
    controller.filtered_fsr_excess = 0.0
    controller.fsr_loaded_once = False
    controller.fsr_release_started = 0.0
    controller.locked_release_started = 0.0
    return controller


class FsrReleaseHysteresisTest(unittest.TestCase):
    def test_stable_position_without_fsr_load_cannot_lock(self):
        controller = make_controller()
        controller.position_is_stable = lambda _now: True

        self.assertFalse(controller.lock_condition_met(100.0))

        controller.fsr_loaded_once = True
        self.assertTrue(controller.lock_condition_met(100.0))

    def test_baseline_or_point_one_noise_never_arms_fsr_release(self):
        controller = make_controller()
        controller.filtered_fsr_excess = 0.10
        controller.observe_fsr_load()

        self.assertFalse(controller.fsr_loaded_once)
        self.assertEqual(controller.locked_release_reason(100.0, False), "")
        self.assertEqual(controller.locked_release_reason(101.0, False), "")

    def test_loaded_then_unloaded_for_point_three_seconds_releases(self):
        controller = make_controller()
        controller.filtered_fsr_excess = 0.31
        with contextlib.redirect_stdout(io.StringIO()):
            controller.observe_fsr_load()
        self.assertTrue(controller.fsr_loaded_once)

        controller.filtered_fsr_excess = 0.14
        self.assertEqual(controller.locked_release_reason(100.0, False), "")
        self.assertEqual(controller.locked_release_reason(100.29, False), "")
        self.assertEqual(
            controller.locked_release_reason(100.31, False),
            "FSR加载后卸载",
        )

    def test_rising_above_release_threshold_resets_fsr_timer(self):
        controller = make_controller()
        controller.filtered_fsr_excess = 0.31
        with contextlib.redirect_stdout(io.StringIO()):
            controller.observe_fsr_load()

        controller.filtered_fsr_excess = 0.14
        controller.locked_release_reason(100.0, False)
        controller.filtered_fsr_excess = 0.20
        controller.locked_release_reason(100.20, False)
        controller.filtered_fsr_excess = 0.14
        self.assertEqual(controller.locked_release_reason(101.0, False), "")
        self.assertEqual(
            controller.locked_release_reason(101.31, False),
            "FSR加载后卸载",
        )

    def test_inspire_release_remains_independent(self):
        controller = make_controller()
        self.assertEqual(controller.locked_release_reason(100.0, True), "")
        self.assertEqual(
            controller.locked_release_reason(100.16, True),
            "Inspire触觉释放",
        )


if __name__ == "__main__":
    unittest.main()
