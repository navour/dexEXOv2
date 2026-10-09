#!/usr/bin/env python3
"""PyQt 上位机离屏烟雾测试；不打开任何硬件。"""

from __future__ import annotations

import os
from pathlib import Path
import sys
import unittest
from unittest import mock


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    import PyQt5  # noqa: F401
    import pyqtgraph  # noqa: F401
except ModuleNotFoundError:
    QT_AVAILABLE = False
else:
    QT_AVAILABLE = True


@unittest.skipUnless(QT_AVAILABLE, "PyQt5/pyqtgraph 未安装")
class PyQtSupervisorSmokeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        supervisor_dir = Path(__file__).resolve().parent
        force_dir = supervisor_dir.parent / "force_control"
        sys.path.insert(0, str(force_dir))
        sys.path.insert(0, str(supervisor_dir))

        from local_supervisor import QApplication, SupervisorWindow
        from dual_hand_force_test import DualHandController, parse_args

        cls.QApplication = QApplication
        cls.SupervisorWindow = SupervisorWindow
        cls.DualHandController = DualHandController
        cls.parse_args = staticmethod(parse_args)
        cls.app = QApplication.instance() or QApplication(["test-supervisor"])

    def setUp(self):
        self.g1_start_patch = mock.patch.object(
            self.DualHandController, "start_g1_inputs", return_value=True)
        self.g1_start = self.g1_start_patch.start()
        self.window = self.SupervisorWindow(self.parse_args([]))
        self.window.show()
        self.app.processEvents()

    def tearDown(self):
        self.window.allow_close = True
        self.window.close()
        self.app.processEvents()
        self.g1_start_patch.stop()

    def test_main_supervisor_builds_without_hardware(self):
        self.assertEqual(2, len(self.window.hand_panels))
        self.assertTrue(all(len(panel.gauges) == 5
                            for panel in self.window.hand_panels.values()))
        self.assertEqual(20, len(self.window.curves))
        self.assertEqual(15, self.window.table.columnCount())
        self.assertEqual(10, self.window.table.rowCount())
        self.assertEqual(3, self.window.tabs.count())

    def test_arm_stays_locked_before_connections(self):
        allowed, reason = self.window.arm_gate({})
        self.assertFalse(allowed)
        self.assertIn("十舵机", reason)
        self.assertFalse(self.window.arm_button.isEnabled())

    def test_g1_link_clients_start_automatically(self):
        self.g1_start.assert_called_once_with()
        self.assertFalse(hasattr(self.window, "g1_button"))

    def test_init_and_arm_do_not_open_confirmation_dialogs(self):
        self.window.serial_is_open = True
        self.window.run_command = mock.Mock()
        with mock.patch.object(self.window, "arm_gate", return_value=(True, "OK")), \
                mock.patch("local_supervisor.QMessageBox.question") as question, \
                mock.patch("local_supervisor.QMessageBox.warning") as warning:
            self.window.on_init()
            self.window.on_arm()

        question.assert_not_called()
        warning.assert_not_called()
        self.assertEqual(2, self.window.run_command.call_count)

    def test_hidden_heavy_views_are_not_redrawn_by_fast_refresh(self):
        snapshot = self.window.last_snapshot
        self.window.tabs.setCurrentIndex(2)
        self.app.processEvents()
        with mock.patch.object(self.window, "update_table") as update_table, \
                mock.patch.object(self.window, "render_histories") as render_histories:
            self.window.render_snapshot(snapshot)
        update_table.assert_not_called()
        render_histories.assert_not_called()

    def test_history_keeps_thirty_second_window_at_lower_sample_rate(self):
        self.assertEqual(100, self.window.REFRESH_MS)
        self.assertEqual(200, self.window.HISTORY_SAMPLE_MS)
        self.assertAlmostEqual(-29.8, self.window._history_x[0])
        self.assertEqual(0.0, self.window._history_x[-1])


if __name__ == "__main__":
    unittest.main()
