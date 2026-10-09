#!/usr/bin/env python3
"""不创建 PyQt 窗口，验证本地上位机的 STATUS 文本。"""

from __future__ import annotations

from pathlib import Path
import socket
import sys
import types
import unittest


try:
    import dynamixel_sdk  # noqa: F401
except ModuleNotFoundError:
    sdk = types.ModuleType("dynamixel_sdk")
    sdk.COMM_SUCCESS = 0
    sdk.PacketHandler = object
    sdk.PortHandler = object
    sys.modules["dynamixel_sdk"] = sdk

sys.path.insert(0, str(Path(__file__).resolve().parent))
from supervisor_common import BleBrokerManager, format_status  # noqa: E402


class FormatStatusTest(unittest.TestCase):
    def test_status_contains_links_fsr_inspire_and_servo(self):
        snapshot = {
            "system": {
                "state": "STOP",
                "write_enabled": True,
                "serial": {"connected": True},
            },
            "hands": {
                side: {
                    "ble_fsr": {
                        "connected": side == "right",
                        "last_error": None if side == "right" else "refused",
                    },
                    "inspire_feedback": {
                        "connected": False,
                        "last_error": "timed out",
                    },
                    "haptic_override": {"enabled": True, "connected": False},
                    "fingers": [{
                        "name": "拇指",
                        "servo_id": 1 if side == "right" else 6,
                        "state": "STOP",
                        "fsr": {"raw_n": 4.903},
                        "inspire": {"force_n": 0.0},
                        "servo": {"position": 1234, "present_current": 0},
                    }],
                }
                for side in ("right", "left")
            },
        }

        text = format_status(snapshot)

        self.assertIn("系统=STOP", text)
        self.assertIn("右手: BLE/FSR=正常", text)
        self.assertIn("左手: BLE/FSR=断开", text)
        self.assertIn("INSPIRE最近错误: timed out", text)
        self.assertIn("拇指 ID1", text)
        self.assertIn("FSR=4.903N", text)

    def test_broker_manager_detects_occupied_port(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.bind(("127.0.0.1", 0))
            listener.listen(1)
            port = listener.getsockname()[1]
            self.assertEqual(
                (port,),
                BleBrokerManager.occupied_ports(ports=(port,)),
            )


if __name__ == "__main__":
    unittest.main()
