import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from pid_tuner.gui.app import GOTO_YAW_LABEL, MainWindow, format_telemetry_status, validate_motion_goal
from pid_tuner.models import MotionGoal, Telemetry


class GuiMotionGoalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_default_motion_goal_is_accepted(self) -> None:
        window = MainWindow()
        try:
            goal = MotionGoal(*(widget.value() for widget in window.goal), window.timeout.value())
            self.assertEqual((goal.x_mm, goal.y_mm, goal.yaw_deg), (0.0, 0.0, 0.0))
            self.assertEqual((goal.vmax_mm_s, goal.wmax_deg_s, goal.timeout_ms), (50.0, 30.0, 5000))
            self.assertIsNone(validate_motion_goal(goal))
        finally:
            window.close()

    def test_invalid_motion_goal_is_rejected_locally(self) -> None:
        goal = MotionGoal(0.0, 0.0, 0.0, 0.0, 30.0, 5000)
        self.assertEqual(validate_motion_goal(goal), "vmax 必须在 0-1500 mm/s 之间")

    def test_arrived_status_explains_that_motion_is_not_needed(self) -> None:
        telemetry = Telemetry(1, 2, 0, 2, 0x07, (0.0, 0.0, 0.0),
                              (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                              (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                              (0.0, 0.0, 0.0))
        self.assertIn("已到达，无需运动", format_telemetry_status(telemetry))

    def test_yaw_label_is_relative_to_initialization_zero(self) -> None:
        self.assertEqual(GOTO_YAW_LABEL, "yaw 相对初始化零点 deg")


if __name__ == "__main__":
    unittest.main()
