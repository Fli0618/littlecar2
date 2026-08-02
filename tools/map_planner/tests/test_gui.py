import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from map_planner.codegen_c import generate_task_function
from map_planner.gui import PlannerWindow
from map_planner.models import ContinuousPathSegment, RotateInPlace, Waypoint


class GuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def window(self):
        result = PlannerWindow()
        result.calibration_pending = False
        result.calibration_stage = "complete"
        result.update_calibration_ui()
        return result

    def test_flow_list_and_locked_continuous_entry(self):
        window = self.window()
        try:
            window.plan.steps.append(Waypoint(100, 200, 30))
            window.add_continuous_segment()
            segment = window.plan.steps[-1]
            self.assertIsInstance(segment, ContinuousPathSegment)
            self.assertEqual((segment.points[0].x_mm, segment.points[0].y_mm, segment.points[0].yaw_deg), (100, 200, 30))
            self.assertFalse(window.continuous_x.isEnabled())
            self.assertIn("入口点", window.continuous_list.item(0).text())
        finally:
            window.close()

    def test_editing_predecessor_synchronizes_entry_and_step_order(self):
        window = self.window()
        try:
            window.plan.steps = [Waypoint(100, 200, 0), ContinuousPathSegment([]), RotateInPlace(90)]
            window._sync_continuous_entries()
            window.active_index = 0
            window.show_node(0)
            window.x.setValue(300); window.y.setValue(400); window.yaw.setValue(45)
            window.update_waypoint()
            segment = window.plan.steps[1]
            self.assertEqual((segment.points[0].x_mm, segment.points[0].y_mm, segment.points[0].yaw_deg), (300, 400, 45))
            window.active_index = 2; window.move_step(-1)
            self.assertIsInstance(window.plan.steps[1], RotateInPlace)
        finally:
            window.close()

    def test_continuous_points_and_mixed_codegen_order(self):
        window = self.window()
        try:
            window.plan.steps = [Waypoint(10, 20, 0), ContinuousPathSegment([]), Waypoint(120, 30, 0)]
            window._sync_continuous_entries()
            window.active_index = 1; window.active_point_index = 0
            window.set_mode("add")
            window.confirm_preview(100, 20)
            segment = window.plan.steps[1]
            self.assertIn("最终停车点", window.continuous_list.item(1).text())
            code = generate_task_function(window.plan, "Task_Mixed")
            self.assertLess(code.index("GOTO"), code.index("FOLLOW PATH"))
            self.assertLess(code.index("FOLLOW PATH"), code.rindex("GOTO"))
        finally:
            window.close()
