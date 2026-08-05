from __future__ import annotations

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from motion_workbench.control_panel import HolonomicControlPanel, PointControlPanel
from pid_tuner.models import HolonomicConfig, HolonomicConfigState


class HolonomicPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_panel_has_twelve_fields_and_tracks_revision(self) -> None:
        panel = HolonomicControlPanel()
        try:
            self.assertEqual(len(panel.inputs), 12)
            panel.set_connected(True)
            state = HolonomicConfigState(
                7, HolonomicConfig(600, 800, 150, 0.8, 0.3, 0.8, 0.3,
                                   2.0, 0.3, 1.0, 1.0, 1.0))
            panel.set_config(state)
            self.assertEqual(panel.current_config(), state.config)
            self.assertIn("r7", panel.status.text())
        finally:
            panel.close()

    def test_unsupported_firmware_disables_only_holonomic_panel(self) -> None:
        panel = HolonomicControlPanel()
        try:
            panel.set_connected(True)
            panel.set_unsupported(True)
            self.assertFalse(panel.isEnabled())
            self.assertFalse(panel.apply_button.isEnabled())
            panel.set_connected(False)
            panel.set_connected(True)
            self.assertFalse(panel.isEnabled())
            panel.set_unsupported(False)
            self.assertTrue(panel.isEnabled())
            self.assertTrue(panel.apply_button.isEnabled())
        finally:
            panel.close()

    def test_point_panel_exposes_single_controller_selector(self) -> None:
        panel = PointControlPanel()
        try:
            self.assertEqual(panel.current_controller(), "classic")
            panel.controller.setCurrentIndex(panel.controller.findData("holonomic"))
            self.assertEqual(panel.current_controller(), "holonomic")
        finally:
            panel.close()


if __name__ == "__main__":
    unittest.main()
