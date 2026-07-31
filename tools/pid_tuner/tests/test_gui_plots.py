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

        self.assertEqual(len(plots.plots), 6)
        self.assertIsNot(plots.position_x, plots.position_y)
        self.assertEqual(len(plots.diag), 3)
        self.assertIn("X 误差", plots.error_x.titleLabel.text)
        self.assertIn("Y 误差", plots.error_y.titleLabel.text)
        self.assertIn("航向误差", plots.error_yaw.titleLabel.text)

    def test_diagnostic_mode_updates_each_axis_title(self) -> None:
        plots = TelemetryPlots()
        buffer = TelemetryBuffer()
        buffer.append(Telemetry(0, 1, 0, 1, 3, (1, 2, 3), (4, 5, 6), (7, 8, 9), (10, 11, 12), (0, 0, 0), (0, 0, 0)))

        plots.mode.setCurrentIndex(1)
        plots.refresh(buffer)

        self.assertIn("X 命令-实际速度", plots.error_x.titleLabel.text)
        self.assertIn("Y 命令-实际速度", plots.error_y.titleLabel.text)
        self.assertIn("航向命令-实际速度", plots.error_yaw.titleLabel.text)
