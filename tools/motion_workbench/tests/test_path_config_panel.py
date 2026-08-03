from __future__ import annotations

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from motion_workbench.app import PathControlPanel
from pid_tuner.models import PathControlConfig


class PathControlPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_panel_round_trips_all_fourteen_values(self) -> None:
        panel = PathControlPanel()
        config = PathControlConfig(
            1.0, 0.5, 1.5, 0.4, 700.0, 90.0, 600.0, 800.0,
            500.0, 50.0, 70.0, 0.12, 100.0, 160.0,
        )
        requested: list[PathControlConfig] = []
        panel.apply_config_requested.connect(requested.append)
        try:
            panel.set_config(12, config)
            self.assertIn("12", panel.config_status.text())
            panel.apply_config.click()
            self.assertEqual(panel.current_config(), config)
            self.assertEqual(requested, [config])
        finally:
            panel.close()

    def test_panel_rejects_inconsistent_lookahead(self) -> None:
        panel = PathControlPanel()
        requested: list[PathControlConfig] = []
        panel.apply_config_requested.connect(requested.append)
        try:
            panel.config_inputs["lookahead_min_mm"].setValue(200.0)
            panel.config_inputs["lookahead_base_mm"].setValue(100.0)
            panel.apply_config.click()
            self.assertEqual(requested, [])
            self.assertIn("最小", panel.config_status.text())
        finally:
            panel.close()


if __name__ == "__main__":
    unittest.main()
