import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from pid_tuner.gui.buffer import TelemetryBuffer
from pid_tuner.gui.plots import TelemetryPlots
from pid_tuner.models import Telemetry


class TelemetryPlotsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_pose_and_diagnostics_are_split_by_axis(self) -> None:
        plots = TelemetryPlots()

        self.assertEqual(len(plots.plots), 10)
        self.assertIsNot(plots.position_x, plots.position_y)
        self.assertEqual(len(plots.diag), 4)
        self.assertIn("X 误差", plots.error_x.titleLabel.text)
        self.assertIn("Y 误差", plots.error_y.titleLabel.text)
        self.assertIn("X 速度", plots.speed_x.titleLabel.text)
        self.assertIn("Y 速度", plots.speed_y.titleLabel.text)
        self.assertIn("WIT 航向误差", plots.error_wit_yaw.titleLabel.text)
        self.assertIn("OPS 航向误差", plots.error_ops_yaw.titleLabel.text)

    def test_diagnostic_mode_updates_each_axis_title(self) -> None:
        plots = TelemetryPlots()
        buffer = TelemetryBuffer()
        buffer.append(Telemetry(0, 1, 0, 1, 3, (1, 2, 3), (4, 5, 6), (7, 8, 9), (10, 11, 12), (0, 0, 0), (0, 0, 0)))

        plots.mode.setCurrentIndex(1)
        plots.refresh(buffer)

        self.assertIn("X 误差积分累计", plots.error_x.titleLabel.text)
        self.assertIn("Y 误差积分累计", plots.error_y.titleLabel.text)
        self.assertIn("WIT 航向误差", plots.error_wit_yaw.titleLabel.text)

    def test_error_and_command_measured_speed_are_visible_together(self) -> None:
        plots = TelemetryPlots()
        buffer = TelemetryBuffer()
        buffer.append(Telemetry(0, 1, 0, 1, 3, (100, 200, 0), (90, 180, 0), (10, 20, 0),
                                (120, 240, 0), (110, 220, 0), (1, 2, 0)))

        plots.refresh(buffer)

        self.assertEqual(plots.mode.currentText(), "误差")
        self.assertEqual(plots.curves["command_vx"].getData()[1].tolist(), [120.0])
        self.assertEqual(plots.curves["measured_vx"].getData()[1].tolist(), [110.0])
        self.assertEqual(plots.curves["command_vy"].getData()[1].tolist(), [240.0])
        self.assertEqual(plots.curves["measured_vy"].getData()[1].tolist(), [220.0])
        self.assertIn("当前 +10.00", plots.error_x.titleLabel.text)
        self.assertIn("当前 +20.00", plots.error_y.titleLabel.text)

    def test_heading_error_titles_mark_control_and_observation_roles(self) -> None:
        plots = TelemetryPlots()
        buffer = TelemetryBuffer()
        buffer.append(Telemetry(0, 1, 0, 1, 3, (0, 0, 10), (0, 0, 8), (0, 0, 2),
                                (0, 0, 0), (0, 0, 0), (0, 0, 0),
                                wit_yaw_deg=7, ops_yaw_deg=12))

        plots.set_heading_mode("OPS")
        plots.refresh(buffer)
        self.assertIn("对照源", plots.error_wit_yaw.titleLabel.text)
        self.assertIn("当前控制源", plots.error_ops_yaw.titleLabel.text)
        self.assertIn("当前 -2.00", plots.error_ops_yaw.titleLabel.text)

        plots.set_heading_mode("NONE")
        self.assertIn("未参与控制", plots.error_wit_yaw.titleLabel.text)
        self.assertIn("未参与控制", plots.error_ops_yaw.titleLabel.text)

    def test_normal_pose_and_error_ranges_are_fixed(self) -> None:
        plots = TelemetryPlots()
        buffer = TelemetryBuffer()
        buffer.append(Telemetry(0, 1, 0, 1, 3, (100, -100, 45), (90, -80, 40), (10, -20, 5), (0, 0, 0), (0, 0, 0), (0, 0, 0)))

        plots.refresh(buffer)

        self.assertEqual(plots.position_x.viewRange()[1], [-500.0, 500.0])
        self.assertEqual(plots.position_y.viewRange()[1], [-500.0, 500.0])
        self.assertEqual(plots.speed_x.viewRange()[1], [-100.0, 100.0])
        self.assertEqual(plots.speed_y.viewRange()[1], [-100.0, 100.0])
        self.assertEqual(plots.wit_yaw.viewRange()[1], [-180.0, 180.0])
        self.assertEqual(plots.error_x.viewRange()[1], [-500.0, 500.0])
        self.assertEqual(plots.error_wit_yaw.viewRange()[1], [-180.0, 180.0])

    def test_out_of_range_pose_expands_y_range(self) -> None:
        plots = TelemetryPlots()
        buffer = TelemetryBuffer()
        buffer.append(Telemetry(0, 1, 0, 1, 3, (800, 0, 0), (750, 0, 0), (50, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)))

        plots.refresh(buffer)

        y_range = plots.position_x.viewRange()[1]
        self.assertLess(y_range[0], 750.0)
        self.assertGreater(y_range[1], 800.0)
