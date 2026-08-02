import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from topology_planner.gui import PlannerWindow


class GuiSmokeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_window_controls_and_defaults(self):
        window = PlannerWindow()
        try:
            self.assertEqual(window.windowTitle(), "LittleCar2 拓扑路径规划")
            self.assertEqual(window.start_combo.count(), 15)
            self.assertEqual(window.goal_combo.count(), 15)
            self.assertEqual(window.distance_weight.value(), 1.0)
            self.assertEqual(window.turn_weight.value(), 0.75)
            self.assertEqual(window.stop_weight.value(), 1.0)
        finally:
            window.close()


if __name__ == "__main__":
    unittest.main()
