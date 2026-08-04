import os
import unittest
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QPointF
from PySide6.QtWidgets import QApplication

from map_planner.gui import MapEditorWidget
from map_planner.geometry import world_to_paper
from map_planner.models import BezierPathSegment, Plan, Pose, StepTurnPathSegment, Waypoint


class MapEditorWidgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_plan_api_returns_copy_and_emits_change(self):
        widget = MapEditorWidget()
        changes = []
        widget.plan_changed.connect(changes.append)
        try:
            source = Plan(steps=[Waypoint(100, 200, 30)])
            widget.set_plan(source)
            exported = widget.get_plan()
            exported.steps[0].x_mm = 999

            self.assertEqual(widget.plan.steps[0].x_mm, 100)
            self.assertTrue(changes)
        finally:
            widget.close()

    def test_canvas_selection_and_runtime_snapshot_overlay(self):
        widget = MapEditorWidget()
        selected = []
        widget.candidate_selected.connect(selected.append)
        try:
            widget.set_plan(Plan(steps=[Waypoint(100, 200, 30)]))
            self.assertIs(widget.canvas, widget.view)
            widget.select_candidate(0)
            self.assertEqual((widget.selected_candidate_index, selected), (0, [0]))

            widget.apply_runtime_snapshot(SimpleNamespace(
                actual_pose=Pose(300, 400, 90), target_pose=None, error=None,
                path_telemetry=None, new_trace_points=(), trace_reset=True,
                pose_valid=True, motion_active=False,
            ))
            self.assertIn("runtime_car", [item.data(0) for item in widget.scene.items()])
            widget.apply_runtime_snapshot(SimpleNamespace(
                actual_pose=None, target_pose=None, error=None,
                path_telemetry=None, new_trace_points=(), trace_reset=False,
                pose_valid=False, motion_active=False,
            ))
            runtime_car = next(item for item in widget.scene.items() if item.data(0) == "runtime_car")
            self.assertFalse(runtime_car.isVisible())
        finally:
            widget.close()

    def test_hardware_execution_controls_and_telemetry_overlay(self):
        widget = MapEditorWidget()
        enabled = []
        requests = []
        try:
            widget.hardware_enabled_changed.connect(enabled.append)
            widget.single_step_requested.connect(lambda index: requests.append(("step", index)))
            widget.continuous_requested.connect(lambda index: requests.append(("continuous", index)))
            self.assertFalse(widget.execution_step_button.isEnabled())

            widget.set_execution_enabled(True)
            widget.plan.steps = [Waypoint(100, 200, 30)]
            widget.active_index = 0
            widget.execution_step_button.click()
            widget.execution_run_button.click()
            widget.apply_runtime_snapshot(SimpleNamespace(
                actual_pose=Pose(110, 210, 40), target_pose=Pose(100, 200, 30),
                error=(10, 10, 10), path_telemetry=None,
                new_trace_points=(Pose(0, 0, 0), Pose(110, 210, 40)),
                trace_reset=True, pose_valid=True, motion_active=False,
            ))

            markers = [item.data(0) for item in widget.scene.items()]
            self.assertEqual(enabled, [True])
            self.assertEqual(requests, [("step", 0), ("continuous", 0)])
            self.assertIn("runtime_target", markers)
            self.assertIn("runtime_car", markers)
            self.assertIn("runtime_trace", markers)
            self.assertIn("误差 X=10.0 mm", widget.execution_status_label.text())

            widget.set_execution_enabled(False)
            self.assertFalse(widget.execution_step_button.isEnabled())
            self.assertFalse(widget.execution_run_button.isEnabled())
            self.assertFalse(widget.execution_stop_button.isEnabled())
        finally:
            widget.close()

    def test_runtime_trace_appends_and_clears_on_reset_or_start_change(self):
        widget = MapEditorWidget()
        try:
            snapshot = lambda points, reset=False: SimpleNamespace(
                actual_pose=None, target_pose=None, error=None, path_telemetry=None,
                new_trace_points=points, trace_reset=reset, pose_valid=True, motion_active=False,
            )
            widget.apply_runtime_snapshot(snapshot((Pose(0, 0, 0), Pose(100, 0, 0)), True))
            path = widget._runtime_trace_path
            widget.apply_runtime_snapshot(snapshot((Pose(200, 0, 0),)))
            self.assertIs(widget._runtime_trace_path, path)
            self.assertEqual(widget._runtime_trace_path.elementCount(), 3)

            widget.apply_runtime_snapshot(snapshot((Pose(0, 100, 0),), True))
            self.assertEqual(widget._runtime_trace_path.elementCount(), 1)
            widget.set_start_frame(2250, 2250, 180)
            self.assertEqual(widget._execution_trace, [])
            self.assertEqual(widget._runtime_trace_path.elementCount(), 0)
        finally:
            widget.close()

    def test_start_preset_uses_start_frame_rebase(self):
        widget = MapEditorWidget()
        try:
            widget.set_plan(Plan(steps=[Waypoint(0, 100, 30)]))
            original = world_to_paper(widget.plan.steps[0], 2250, 150, 180)

            widget.begin_start("启停区 2")

            rebased = world_to_paper(widget.plan.steps[0], 2250, 2250, 180)
            self.assertAlmostEqual(rebased[0], original[0])
            self.assertAlmostEqual(rebased[1], original[1])
        finally:
            widget.close()

    def test_start_frame_change_is_blocked_while_executing(self):
        widget = MapEditorWidget()
        try:
            widget.set_hardware_motion_active(True)
            with self.assertRaisesRegex(RuntimeError, "执行期间"):
                widget.begin_start("启停区 2")
        finally:
            widget.close()

    def test_tangent_bezier_closes_following_step_with_derived_endpoint_yaw(self):
        widget = MapEditorWidget()
        try:
            widget.set_plan(Plan(steps=[
                BezierPathSegment(0, 100, 100, 300, 200, 300, 17, "tangent"),
                Waypoint(300, 400, 0),
            ]))

            endpoint = widget._step_end_pose(1)

            self.assertAlmostEqual(endpoint.yaw_deg, 90.0)
        finally:
            widget.close()

    def test_yellow_zone_passage_defaults_allowed_and_keeps_field_boundary(self):
        widget = MapEditorWidget()
        try:
            platform_center = QPointF(775, 775)
            start = QPointF(400, 775)
            end = QPointF(1100, 775)

            self.assertTrue(widget.allow_yellow_zone.isChecked())
            self.assertTrue(widget._is_valid_start_candidate(775, 775))
            self.assertTrue(widget.is_valid_route_segment(start, end))
            self.assertTrue(widget.is_valid_continuous_segment(start, end))
            self.assertTrue(widget.is_valid_rotation(platform_center, 0, 90))
            self.assertFalse(widget._is_valid_start_candidate(50, 50))
            self.assertIn("黄色区限制已关闭", widget.yellow_zone_status_label.text())

            widget.begin_start("自定义")
            widget.update_preview(775, 775)
            allowed_preview = next(
                item for item in widget.scene.items()
                if item.data(0) == "start_pose_preview")
            self.assertEqual(allowed_preview.pen().color().name(), "#1565c0")

            widget.allow_yellow_zone.setChecked(False)
            self.assertFalse(widget._is_valid_start_candidate(775, 775))
            self.assertFalse(widget.is_valid_route_segment(start, end))
            self.assertFalse(widget.is_valid_continuous_segment(start, end))
            self.assertFalse(widget.is_valid_rotation(platform_center, 0, 90))
            self.assertFalse(widget._is_valid_start_candidate(50, 50))
            self.assertIn("黄色区限制已启用", widget.yellow_zone_status_label.text())
            blocked_preview = next(
                item for item in widget.scene.items()
                if item.data(0) == "start_pose_preview")
            self.assertEqual(blocked_preview.pen().color().name(), "#c62828")
        finally:
            widget.close()

    def test_custom_start_shows_hover_heading_and_invalid_preview(self):
        widget = MapEditorWidget()
        try:
            widget.begin_start("自定义")
            widget.update_preview(400, 400)
            preview = next(item for item in widget.scene.items()
                           if item.data(0) == "start_pose_preview")
            self.assertEqual((preview.x(), preview.y()), (400.0, 400.0))

            widget.on_map_click(400, 400)
            self.assertEqual(widget.calibration_stage, "heading")
            before = widget.plan.start_heading_deg
            widget.rotate_start_clockwise()
            self.assertNotEqual(widget.plan.start_heading_deg, before)

            widget.begin_start("自定义")
            widget.update_preview(50, 50)
            invalid = next(item for item in widget.scene.items()
                           if item.data(0) == "start_pose_preview")
            self.assertEqual(invalid.pen().color().name(), "#c62828")
        finally:
            widget.close()

    def test_step_turn_draft_confirm_cancel_continue_and_path_preview(self):
        widget = MapEditorWidget()
        try:
            widget.set_plan(Plan(), calibrated=True)
            widget.begin_step_turn_add()
            widget.on_map_release(2100, 200)
            widget.on_map_release(1900, 400)
            self.assertEqual(len(widget.step_turn_draft.route_points), 2)
            self.assertIn("step_turn_preview_materialized", [item.data(0) for item in widget.scene.items()])

            widget.confirm_draft()
            self.assertIsInstance(widget.plan.steps[0], StepTurnPathSegment)
            self.assertGreaterEqual(len(widget.selected_step_path_points()), 2)

            original_count = len(widget.plan.steps[0].route_points)
            widget.continue_step_turn_edit()
            widget.on_map_release(1700, 600)
            widget.cancel_draft()
            self.assertEqual(len(widget.plan.steps[0].route_points), original_count)

            widget.continue_step_turn_edit()
            widget.on_map_release(1700, 600)
            widget.remove_last_step_turn_node()
            self.assertEqual(len(widget.step_turn_draft.route_points), original_count)
        finally:
            widget.close()
