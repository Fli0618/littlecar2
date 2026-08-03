import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from map_planner.codegen_c import CodeGenerationMode, generate_task_function
from map_planner.codegen_dialog import CodeGenerationDialog
import map_planner.gui as gui_module
from map_planner.gui import PlannerWindow
from map_planner.models import BezierPathSegment, ContinuousPathSegment, PathPosePoint, Plan, RotateInPlace, Waypoint


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

    def test_gui_module_exposes_launch_entrypoint(self):
        self.assertTrue(callable(gui_module.main))

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

    def test_goto_action_is_a_peer_button_and_uses_map_click(self):
        window = self.window()
        try:
            window.begin_goto_add()
            self.assertEqual(window.mode, "add")
            window.confirm_preview(2200, 300)
            self.assertIsInstance(window.plan.steps[-1], Waypoint)
            self.assertIsNone(window.pending_action)
        finally:
            window.close()

    def test_bezier_creation_stays_transient_until_confirmed(self):
            window = self.window()
        try:
            window.begin_bezier_add()
            window.on_map_release(1950, 150)
            self.assertEqual(len(window.plan.steps), 0)
            self.assertIsInstance(window.bezier_draft, BezierPathSegment)
            self.assertIn("bezier_sample", [item.data(0) for item in window.scene.items()])
            window.confirm_bezier_draft()
            self.assertEqual(len(window.plan.steps), 1)
            self.assertIsInstance(window.plan.steps[-1], BezierPathSegment)
        finally:
            window.close()

    def test_bezier_cancel_discards_draft_and_validation_does_not_access_points(self):
        window = self.window()
        try:
            window.plan.steps = [BezierPathSegment(100, 0, 200, 0, 300, 0, 0)]
            self.assertIsInstance(window.invalid_waypoints(), list)
            window.begin_bezier_add()
            window.confirm_preview(1950, 150)
            window.cancel_bezier_draft()
            self.assertIsNone(window.bezier_draft)
            self.assertEqual(len(window.plan.steps), 1)
        finally:
            window.close()

    def test_bezier_preview_and_confirmation_skip_region_validation(self):
        window = self.window()
        try:
            window.begin_bezier_add()
            window.confirm_preview(1950, 150)
            self.assertIsNotNone(window.bezier_draft)
            original_sweep_violations = window.sweep_violations
            window.sweep_violations = lambda sweep: self.fail("Bezier preview must not run region validation")
            window.redraw()
            self.assertIn("bezier_preview_coverage", [item.data(0) for item in window.scene.items()])
            window.sweep_violations = original_sweep_violations
            window.confirm_bezier_draft()
            self.assertIsInstance(window.plan.steps[-1], BezierPathSegment)
            self.assertEqual(window.invalid_waypoints(), [])
            self.assertIn("FOLLOW BEZIER PATH", generate_task_function(window.plan, "Task_Bezier"))
        finally:
            window.close()

    def test_preview_draws_direction_and_reports_blocked_simulation(self):
        window = self.window()
        try:
            window.add_continuous_segment()
            window.play()
            self.assertIn("至少需要", window.path_check.text())
            window.begin_goto_add()
            window.update_preview(2200, 300)
            markers = [item.data(0) for item in window.scene.items()]
            self.assertIn("preview_sweep", markers)
            self.assertIn("preview_direction", markers)
            self.assertIn("car_direction", markers)
        finally:
            window.close()

    def test_preview_rotation_persists_to_confirmed_goto_and_rotate_step(self):
        window = self.window()
        try:
            window.begin_goto_add(); window.update_preview(2200, 300)
            window.rotate_preview_clockwise(); yaw = window.preview_yaw_deg
            window.update_preview(2190, 300); self.assertEqual(window.preview_yaw_deg, yaw)
            window.confirm_preview(2190, 300)
            self.assertEqual(window.plan.steps[-1].yaw_deg, yaw)
            window.plan.steps.append(RotateInPlace(0)); window.active_index = len(window.plan.steps) - 1
            window.clear_preview(False); window.rotate_preview_clockwise()
            self.assertEqual(window.plan.steps[-1].yaw_deg, -90)
        finally:
            window.close()

    def test_codegen_dialog_switches_between_feedback_and_open_loop_modes(self):
        dialog = CodeGenerationDialog(Plan(steps=[Waypoint(10, 20, 30)]))
        try:
            self.assertEqual(dialog.mode, CodeGenerationMode.FEEDBACK)
            self.assertIn("AdvanceMotion_Cancel();", dialog.generated_code)
            self.assertIn("严谨反馈", dialog.mode_button.text())

            dialog.mode_button.click()
            self.assertEqual(dialog.mode, CodeGenerationMode.OPEN_LOOP)
            self.assertNotIn("AdvanceMotion_Cancel();", dialog.generated_code)
            self.assertIn("(void)AdvanceMotion_GotoPoseBlocking(", dialog.generated_code)
            self.assertIn("开环忽略结果", dialog.mode_button.text())

            dialog.mode_button.click()
            self.assertEqual(dialog.mode, CodeGenerationMode.FEEDBACK)
            self.assertIn("AdvanceMotion_Cancel();", dialog.generated_code)
        finally:
            dialog.close()

    def test_batch_delete_uses_step_indices_and_resynchronizes_continuous_entry(self):
        window = self.window()
        try:
            window.plan.steps = [
                Waypoint(100, 200, 0),
                ContinuousPathSegment([PathPosePoint(100, 200, 0), PathPosePoint(300, 200, 0)]),
                Waypoint(400, 200, 0),
            ]
            window.selected_indices = {0, 2}
            window.active_index = 2
            window.remove_selected_step()

            self.assertEqual(len(window.plan.steps), 1)
            segment = window.plan.steps[0]
            self.assertIsInstance(segment, ContinuousPathSegment)
            self.assertEqual((segment.points[0].x_mm, segment.points[0].y_mm, segment.points[0].yaw_deg), (0.0, 0.0, 0.0))
        finally:
            window.close()
