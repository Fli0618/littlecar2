import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from pid_tuner.gui.app import MainWindow, validate_motion_goal
from pid_tuner.models import MotionGoal


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


if __name__ == "__main__":
    unittest.main()
