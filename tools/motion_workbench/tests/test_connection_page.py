from __future__ import annotations

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from motion_workbench.app import MotionWorkbenchWindow
from pid_tuner.models import Telemetry


class ConnectionPageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_connection_tab_is_first_and_uses_the_window_session(self) -> None:
        window = MotionWorkbenchWindow()
        self.assertEqual(window.tabs.tabText(0), "连接")
        self.assertIsNotNone(window.connection_panel)
        window.close()

    def test_none_heading_mode_is_not_overwritten_by_ops_telemetry(self) -> None:
        window = MotionWorkbenchWindow()
        try:
            none_index = window.point_panel.heading_mode.findData("NONE")
            window.point_panel.heading_mode.setCurrentIndex(none_index)
            item = Telemetry(1, 1, 0, 1, 0x87, (0, 1000, 0), (0, 10, 0),
                             (0, 990, 0), (0, 100, 0), (0, 95, 0), (0, 0, 0))
            window._sync_heading_source(item)

            self.assertEqual(window.point_panel.current_heading_mode(), "NONE")
            self.assertEqual(window.plots.heading_mode, "NONE")
            self.assertIn("关闭", window.heading_status.text())
        finally:
            window.close()
