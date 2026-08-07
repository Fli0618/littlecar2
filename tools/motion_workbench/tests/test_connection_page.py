from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from motion_workbench.app import MotionConfigExportDialog, MotionWorkbenchWindow
from pid_tuner.models import (GotoStrategySnapshot, HolonomicConfig, HolonomicConfigState,
                               PathConfigSnapshot, PathConfigState, PidConfig, PidConfigState,
                               Telemetry)


class ConnectionPageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_connection_tab_is_first_and_uses_the_window_session(self) -> None:
        window = MotionWorkbenchWindow()
        self.assertEqual(window.tabs.tabText(0), "连接")
        self.assertIsNotNone(window.connection_panel)
        window.close()

    def test_path_tab_only_edits_parameters_and_realtime_page_executes(self) -> None:
        window = MotionWorkbenchWindow()
        try:
            self.assertFalse(hasattr(window.path_panel, "upload"))
            self.assertFalse(hasattr(window.path_panel, "start"))
            self.assertFalse(hasattr(window.path_panel, "abort"))
            self.assertIsNotNone(window.map_editor.execution_step_button)
            self.assertIsNotNone(window.map_editor.execution_run_button)

            index = window.map_editor.execution_controller_combo.findData("holonomic")
            window.map_editor.execution_controller_combo.setCurrentIndex(index)
            self.assertEqual(window.controller._plan_point_controller, "holonomic")
        finally:
            window.close()

    def test_hardware_execution_reports_missing_serial_connection(self) -> None:
        window = MotionWorkbenchWindow()
        try:
            window.map_editor.execution_enabled_switch.click()
            self.assertFalse(window.map_editor.execution_enabled)
            self.assertIn("请先连接串口", window.map_editor.execution_status_label.text())
        finally:
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

    def test_motion_config_export_requires_current_session_sync(self) -> None:
        window = MotionWorkbenchWindow()
        try:
            self.assertFalse(window.export_motion_config.isEnabled())
            window.controller.session.connected = True
            window._cache_pid_state(PidConfigState(12, PidConfig(1, 2, 3, 4, 5, 6)))
            self.assertFalse(window.export_motion_config.isEnabled())
            path = PathConfigState(8, PathConfigSnapshot(*[float(value) for value in range(1, 22)]))
            window._cache_path_state(path)
            self.assertFalse(window.export_motion_config.isEnabled())
            window._cache_goto_strategy(GotoStrategySnapshot(True))
            self.assertFalse(window.export_motion_config.isEnabled())
            window._cache_holonomic_state(HolonomicConfigState(
                9, HolonomicConfig(600, 800, 150, 0.8, 0.3, 0.8, 0.3,
                                   2.0, 0.3, 1.0, 1.0, 1.0)))
            self.assertTrue(window.export_motion_config.isEnabled())

            window.pid_panel.pid[0].setValue(19.0)
            exported = window._active_pid_state
            self.assertEqual(exported, PidConfigState(12, PidConfig(1, 2, 3, 4, 5, 6)))
            window._on_connection_changed(False)
            self.assertFalse(window.export_motion_config.isEnabled())
            self.assertIsNone(window._active_pid_state)
        finally:
            window.close()

    def test_motion_config_export_dialog_copies_and_saves_utf8(self) -> None:
        text = "#define VALUE (1.0f)\n"
        dialog = MotionConfigExportDialog(text)
        try:
            dialog._copy_all()
            self.assertEqual(QApplication.clipboard().text(), text)
            self.assertIn("已复制", dialog.status.text())
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "advance_motion_config.h"
                self.assertTrue(dialog._save_to_path(path))
                self.assertEqual(path.read_text(encoding="utf-8"), text)
        finally:
            dialog.close()
