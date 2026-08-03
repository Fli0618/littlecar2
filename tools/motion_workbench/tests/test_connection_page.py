from __future__ import annotations

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from motion_workbench.app import MotionWorkbenchWindow


class ConnectionPageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_connection_tab_is_first_and_uses_the_window_session(self) -> None:
        window = MotionWorkbenchWindow()
        self.assertEqual(window.tabs.tabText(0), "连接")
        self.assertIsNotNone(window.connection_panel)
        window.close()
