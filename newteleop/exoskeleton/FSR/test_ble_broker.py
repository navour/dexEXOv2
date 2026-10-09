#!/usr/bin/env python3
"""不连BLE硬件验证 broker 的双手参数和默认端口。"""

from __future__ import annotations

import sys
import socket
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))

import ble_broker


class BothHandsTest(unittest.TestCase):
    def test_both_builds_right_9001_and_left_9002(self):
        captured = []

        async def capture(brokers):
            captured.extend(brokers)

        argv = ["ble_broker.py", "--hand", "both"]
        with mock.patch.object(sys, "argv", argv), \
                mock.patch.object(ble_broker, "run_brokers", capture), \
                mock.patch.object(ble_broker.signal, "signal"):
            self.assertEqual(ble_broker.main(), 0)

        self.assertEqual(
            [(broker.address, broker.port) for broker in captured],
            [
                ("F0:FD:45:02:85:B3", 9001),
                ("F0:FD:45:02:67:3B", 9002),
            ],
        )

    def test_both_rejects_ambiguous_single_hand_port(self):
        argv = ["ble_broker.py", "--hand", "both", "--port", "9010"]
        with mock.patch.object(sys, "argv", argv):
            with self.assertRaises(SystemExit) as raised:
                ble_broker.main()
        self.assertEqual(raised.exception.code, 2)

    def test_port_conflict_exits_before_ble_connection(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.bind(("127.0.0.1", 0))
            listener.listen(1)
            port = listener.getsockname()[1]
            argv = [
                "ble_broker.py", "--hand", "right",
                "--host", "127.0.0.1", "--port", str(port),
            ]
            with mock.patch.object(sys, "argv", argv), \
                    mock.patch.object(ble_broker.signal, "signal"):
                result = ble_broker.main()

        self.assertEqual(result, ble_broker.PORT_IN_USE_EXIT_CODE)


if __name__ == "__main__":
    unittest.main()
