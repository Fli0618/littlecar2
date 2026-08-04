from __future__ import annotations

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QPoint, QPointF, Qt
from PySide6.QtGui import QWheelEvent
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from motion_workbench.control_panel import (
    HEADING_MODE_NONE,
    HEADING_MODE_OPS,
    GotoControlConfigPanel,
    PointControlPanel,
    ProtectedDoubleSpinBox,
    WorkbenchPidControlPanel,
)
from pid_tuner.models import (GotoControlConfig, GotoControlConfigState, PidConfig,
                               PidConfigState)


class ControlPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_protected_spinbox_ignores_wheel_and_allows_double_click_edit(self) -> None:
        box = ProtectedDoubleSpinBox()
        box.setRange(0.0, 10.0)
        box.setSingleStep(0.5)
        box.setValue(1.0)
        editor = box.lineEdit()

        wheel = QWheelEvent(
            QPointF(2.0, 2.0), QPointF(2.0, 2.0), QPoint(), QPoint(0, 120),
            Qt.MouseButton.NoButton, Qt.KeyboardModifier.NoModifier,
            Qt.ScrollPhase.ScrollUpdate, False,
        )
        box.wheelEvent(wheel)
        self.assertEqual(box.value(), 1.0)
        self.assertFalse(wheel.isAccepted())
        self.assertTrue(editor.isReadOnly())

        box.show()
        QTest.mouseDClick(editor, Qt.MouseButton.LeftButton)
        self.assertFalse(editor.isReadOnly())
        editor.setText("2.5")
        box.interpretText()
        box.editingFinished.emit()
        self.assertEqual(box.value(), 2.5)
        self.assertTrue(editor.isReadOnly())

        box.stepUp()
        self.assertEqual(box.value(), 3.0)
        box.close()

    def test_workbench_pid_panel_round_trips_values(self) -> None:
        panel = WorkbenchPidControlPanel()
        expected = PidConfig(1.1, 0.02, 0.3, 2.2, 0.04, 0.5)
        applied: list[PidConfig] = []
        panel.apply_requested.connect(applied.append)
        try:
            panel.set_connected(True)
            panel.set_pid_state(PidConfigState(12, expected))
            panel.apply_pid.click()
            self.assertEqual(panel.current_pid(), expected)
            self.assertEqual(applied, [expected])
            self.assertTrue(all(box.lineEdit().isReadOnly() for box in panel.pid))
            self.assertIn("r12", panel.status.text())
        finally:
            panel.close()

    def test_pid_limits_defaults_and_goto_strategy_motion_lock(self) -> None:
        panel = WorkbenchPidControlPanel()
        changed: list[bool] = []
        panel.goto_strategy_changed.connect(changed.append)
        try:
            self.assertFalse(panel.apply_pid.isEnabled())
            self.assertFalse(panel.restore_pid.isEnabled())
            panel.set_connected(True)
            self.assertEqual(panel.current_pid(), PidConfig(1.5, 0.10, 0.78, 2.50, 1.0, 0.80))
            self.assertTrue(all(box.minimum() == 0.0 and box.maximum() == 20.0 for box in panel.pid))
            panel.set_goto_strategy(True)
            self.assertTrue(panel.large_yaw_align.isChecked())
            panel.large_yaw_align.click()
            self.assertEqual(changed, [False])
            panel.set_motion_active(True)
            self.assertFalse(panel.large_yaw_align.isEnabled())
            panel.set_motion_active(False)
            self.assertTrue(panel.large_yaw_align.isEnabled())
            panel.set_connected(False)
            self.assertFalse(panel.apply_pid.isEnabled())
            self.assertEqual(panel.status.text(), "PID 未同步")
        finally:
            panel.close()

    def test_heading_mode_controls_single_point_goal_flags(self) -> None:
        panel = PointControlPanel()
        modes: list[str] = []
        goals = []
        panel.heading_mode_requested.connect(modes.append)
        panel.send_requested.connect(goals.append)
        try:
            panel.heading_mode.setCurrentIndex(panel.heading_mode.findData(HEADING_MODE_NONE))
            panel.goto.click()
            self.assertEqual(modes[-1], HEADING_MODE_NONE)
            self.assertFalse(goals[-1].use_yaw)
            self.assertTrue(goals[-1].use_position)
            self.assertFalse(panel.yaw.isEnabled())
            self.assertFalse(panel.wmax.isEnabled())
            self.assertFalse(panel.rotate.isEnabled())

            panel.heading_mode.setCurrentIndex(panel.heading_mode.findData(HEADING_MODE_OPS))
            panel.goto.click()
            self.assertEqual(modes[-1], HEADING_MODE_OPS)
            self.assertTrue(goals[-1].use_yaw)
            self.assertTrue(panel.yaw.isEnabled())
            self.assertTrue(panel.rotate.isEnabled())
        finally:
            panel.close()

    def test_goto_config_panel_round_trips_all_twenty_one_values_while_running(self) -> None:
        panel = GotoControlConfigPanel()
        config = GotoControlConfig(
            40.0, 700.0, 1200.0, 1500.0, 40.0, 100.0, 160.0, 0.98, 0.62,
            150.0, 80.0, 200.0, 280.0, 40.0, 15.0, 25.0, 1.42, 0.427, 20.0,
            500, 1000,
        )
        requested: list[GotoControlConfig] = []
        panel.apply_requested.connect(requested.append)
        try:
            panel.set_connected(True)
            panel.set_config_state(GotoControlConfigState(9, config))
            self.assertIn("9", panel.status.text())
            panel.apply_config.click()
            self.assertEqual(len(panel.config_inputs), 21)
            self.assertEqual(panel.current_config(), config)
            self.assertEqual(requested, [config])
            panel.set_connected(False)
            self.assertFalse(panel.read_config.isEnabled())
            self.assertFalse(panel.apply_config.isEnabled())
            self.assertFalse(panel.restore_config.isEnabled())
        finally:
            panel.close()

    def test_goto_config_allows_capture_distances_at_profile_threshold(self) -> None:
        panel = GotoControlConfigPanel()
        try:
            panel.config_inputs["profile_threshold_mm"].setValue(40.0)
            panel.config_inputs["capture_distance_mm"].setValue(40.0)
            panel.config_inputs["yaw_capture_equivalent_mm"].setValue(40.0)

            config = panel.current_config()

            self.assertEqual(config.capture_distance_mm, 40.0)
            self.assertEqual(config.yaw_capture_equivalent_mm, 40.0)
        finally:
            panel.close()

    def test_goto_config_rejects_capture_distances_above_profile_threshold(self) -> None:
        panel = GotoControlConfigPanel()
        try:
            panel.config_inputs["profile_threshold_mm"].setValue(40.0)
            panel.config_inputs["capture_distance_mm"].setValue(45.0)
            with self.assertRaisesRegex(ValueError, "捕获距离不能超过规划距离阈值"):
                panel.current_config()

            panel.config_inputs["capture_distance_mm"].setValue(40.0)
            panel.config_inputs["yaw_capture_equivalent_mm"].setValue(45.0)
            with self.assertRaisesRegex(ValueError, "航向捕获等效距离不能超过规划距离阈值"):
                panel.current_config()
        finally:
            panel.close()


if __name__ == "__main__":
    unittest.main()
