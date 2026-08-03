from __future__ import annotations

import os
import unittest
from unittest.mock import Mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from motion_workbench.app import MotionWorkbenchWindow
from motion_workbench.models import TargetPose


class OriginShortcutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_origin_buttons_follow_connection_and_motion_state(self) -> None:
        window = MotionWorkbenchWindow()
        try:
            session = window.controller.session
            self.assertFalse(window.reset_origin_button.isEnabled())
            self.assertFalse(window.return_origin_button.isEnabled())

            session.connected = True
            window._refresh_origin_controls()
            self.assertTrue(window.reset_origin_button.isEnabled())
            self.assertTrue(window.return_origin_button.isEnabled())

            session.motion_active = True
            window._refresh_origin_controls()
            self.assertFalse(window.reset_origin_button.isEnabled())
            self.assertFalse(window.return_origin_button.isEnabled())
        finally:
            window.close()

    def test_reset_and_return_use_existing_session_and_point_limits(self) -> None:
        window = MotionWorkbenchWindow()
        try:
            session = window.controller.session
            session.connected = True
            session.reset_origin = Mock()  # type: ignore[method-assign]
            session.start_motion = Mock()  # type: ignore[method-assign]
            window.point_panel.vmax.setValue(321.0)
            window.point_panel.wmax.setValue(45.0)
            window.point_panel.timeout.setValue(6789)
            window._refresh_origin_controls()

            window.reset_origin_button.click()
            session.reset_origin.assert_called_once()
            self.assertFalse(window.return_origin_button.isEnabled())

            session.origin_reset.emit()
            self.assertEqual(window.controller.candidate, TargetPose(0.0, 0.0, 0.0))
            self.assertIn("零点已重置", window.pose_status.text())

            window.return_origin_button.click()
            goal = session.start_motion.call_args.args[0]
            self.assertEqual((goal.x_mm, goal.y_mm, goal.yaw_deg), (0.0, 0.0, 0.0))
            self.assertEqual((goal.vmax_mm_s, goal.wmax_deg_s, goal.timeout_ms),
                             (321.0, 45.0, 6789))
            self.assertTrue(goal.use_position)
        finally:
            window.close()
