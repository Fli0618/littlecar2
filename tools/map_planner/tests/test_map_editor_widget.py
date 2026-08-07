import os
import unittest
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QPointF, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from map_planner.gui import MapEditorWidget
from map_planner.geometry import world_to_paper
from map_planner.models import (BezierPathSegment, ContinuousPathSegment,
                                Plan, Pose, Waypoint)


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

    def test_auto_plan_click_creates_uploadable_continuous_path_and_overlay(self):
        widget = MapEditorWidget()
        try:
            widget.set_plan(Plan(), calibrated=True)
            widget.create_auto_path(QPointF(1200, 300))

            step = widget.plan.steps[-1]
            self.assertIsInstance(step, ContinuousPathSegment)
            self.assertTrue(step.name.startswith("自动规划"))
            self.assertGreater(len(step.points), 2)
            self.assertLessEqual(len(step.points), 256)
            self.assertIn("自动规划完成", widget.status.text())
            self.assertEqual(widget.mode, "auto_plan")
            self.assertTrue(any(item.data(0) == "boundary_cost_band"
                                for item in widget.scene.items()))
            self.assertTrue(any(item.data(0) == "auto_goal_label"
                                for item in widget.scene.items()))
            self.assertTrue(any(item.data(0) == "soft_cost_zone"
                                for item in widget.scene.items()))
            self.assertTrue(any(item.data(0) == "boundary_hard_zone"
                                for item in widget.scene.items()))
            self.assertFalse(widget.advanced_group.isChecked())
            self.assertEqual(
                [widget.editor_tabs.tabText(index)
                 for index in range(widget.editor_tabs.count())],
                ["1 自动导航", "2 路径制作", "3 实时运行", "4 方案与输出"],
            )
        finally:
            widget.close()

    def test_costmap_change_marks_auto_paths_stale_and_replan_clears_it(self):
        widget = MapEditorWidget()
        try:
            widget.set_plan(Plan(), calibrated=True)
            widget.create_auto_path(QPointF(1200, 300))
            self.assertFalse(widget._auto_paths_stale)
            widget.boundary_safety.setValue(25)

            self.assertTrue(widget._auto_paths_stale)
            self.assertEqual(widget.plan.layout.costmap.boundary_safety_margin_mm, 25)
            with self.assertRaisesRegex(ValueError, "重新规划"):
                widget.selected_step_path_points()

            widget.replan_all_auto_paths()

            self.assertFalse(widget._auto_paths_stale)
            self.assertIn("重新规划 1 段", widget.status.text())
        finally:
            widget.close()

    def test_vehicle_dimensions_feed_costmap_and_preview_outline(self):
        widget = MapEditorWidget()
        try:
            widget.set_plan(Plan(), calibrated=True)
            widget.vehicle_length.setValue(360)
            widget.vehicle_width.setValue(240)

            self.assertEqual(widget.plan.layout.costmap.vehicle_length_mm, 360)
            self.assertEqual(widget.plan.layout.costmap.vehicle_width_mm, 300)
            outline = next(item for item in widget.scene.items()
                           if item.data(0) == "start_pose_preview")
            self.assertAlmostEqual(outline.rect().width(), 360)
            self.assertAlmostEqual(outline.rect().height(), 300)
        finally:
            widget.close()

    def test_rviz_style_two_click_goal_draws_guide_and_auto_plans(self):
        widget = MapEditorWidget()
        try:
            widget.set_plan(Plan(), calibrated=True)
            widget.set_mode("mark_pose")
            widget.on_map_click(1800, 300)
            widget.update_preview(1900, 300)
            expected_yaw = widget.preview_yaw_deg
            markers = [item.data(0) for item in widget.scene.items()]
            self.assertIn("nav_goal_arrow", markers)
            self.assertIn("nav_goal_current_heading", markers)
            self.assertIn("nav_goal_angle_label", markers)
            self.assertIn("nav_goal_heading_ring", markers)
            self.assertIn("nav_goal_heading_cross", markers)
            widget.on_map_click(1900, 300)

            self.assertFalse(widget.plan.steps)
            self.assertTrue(any(item.data(0) == "pending_navigation_goal_arrow"
                                for item in widget.scene.items()))
            widget.generate_segment_button.click()

            path = widget.plan.steps[-2]
            rotation = widget.plan.steps[-1]
            expected = widget.paper_of(path.points[-1])
            self.assertIsInstance(path, ContinuousPathSegment)
            self.assertAlmostEqual(rotation.yaw_deg, expected_yaw)
            self.assertAlmostEqual(expected.x_mm, 1800, places=5)
            self.assertAlmostEqual(expected.y_mm, 300, places=5)
            self.assertEqual(widget.mode, "mark_pose")
            self.assertFalse(widget._auto_paths_stale)
        finally:
            widget.close()

    def test_costmap_obstacle_tools_place_and_delete(self):
        widget = MapEditorWidget()
        try:
            widget.set_plan(Plan(), calibrated=True)
            widget.add_obstacle(QPointF(400, 400))
            obstacle_item = next(item for item in widget.scene.items()
                                 if item.data(0) == "obstacle")
            obstacle_item.setSelected(True)
            widget.remove_selected_obstacles()

            self.assertEqual(widget.plan.layout.obstacles, [])
            self.assertEqual(widget.obstacle_count_label.text(), "障碍物：0 个")
            self.assertIs(widget.trajectory_group.parentWidget(), widget.editor_tabs.widget(1))
        finally:
            widget.close()

    def test_clicking_obstacle_keeps_selection_until_delete(self):
        widget = MapEditorWidget()
        try:
            widget.set_plan(Plan(), calibrated=True)
            widget.add_obstacle(QPointF(400, 400))
            widget.resize(1400, 900); widget.show(); widget.fit_map()
            QTest.mouseClick(widget.select_obstacle_button, Qt.MouseButton.LeftButton)
            target = widget.view.mapFromScene(QPointF(400, 400))
            QTest.mouseClick(widget.view.viewport(), Qt.MouseButton.LeftButton,
                             pos=target)

            self.assertEqual(len(widget.scene.selectedItems()), 1)
            widget.remove_selected_obstacles()
            self.assertEqual(widget.plan.layout.obstacles, [])
        finally:
            widget.close()

    def test_obstacle_button_places_on_real_mouse_press(self):
        widget = MapEditorWidget()
        try:
            widget.set_plan(Plan(), calibrated=True)
            widget.resize(1400, 900); widget.show(); widget.fit_map()
            QTest.mouseClick(widget.obstacle_button, Qt.MouseButton.LeftButton)
            target = widget.view.mapFromScene(QPointF(400, 400))
            QTest.mouseClick(widget.view.viewport(), Qt.MouseButton.LeftButton,
                             pos=target)

            self.assertEqual(widget.mode, "obstacle")
            self.assertEqual(len(widget.plan.layout.obstacles), 1)
        finally:
            widget.close()

    def test_navigation_goal_real_mouse_two_click_plans_on_second_click(self):
        widget = MapEditorWidget()
        try:
            widget.set_plan(Plan(), calibrated=True)
            widget.resize(1400, 900); widget.show(); widget.fit_map()
            QTest.mouseClick(widget.mark_pose_button, Qt.MouseButton.LeftButton)
            start = widget.view.mapFromScene(QPointF(1800, 300))
            direction = widget.view.mapFromScene(QPointF(1900, 300))

            QTest.mouseClick(widget.view.viewport(), Qt.MouseButton.LeftButton,
                             pos=start)
            QTest.mouseMove(widget.view.viewport(), direction)
            self.assertTrue(any(item.data(0) == "nav_goal_arrow"
                                for item in widget.scene.items()))
            self.assertFalse(widget.plan.steps)
            QTest.mouseClick(widget.view.viewport(), Qt.MouseButton.LeftButton,
                             pos=direction)

            self.assertFalse(widget.plan.steps)
            self.assertIn("目标位姿已确定", widget.status.text())
            QTest.mouseClick(widget.generate_segment_button,
                             Qt.MouseButton.LeftButton)
            self.assertTrue(any(isinstance(step, ContinuousPathSegment)
                                for step in widget.plan.steps))
            self.assertIn("自动规划完成", widget.status.text())
            self.assertTrue(any(item.data(0) == "navigation_goal_pose"
                                for item in widget.scene.items()))
        finally:
            widget.close()

    def test_yellow_generate_button_updates_only_selected_auto_segment(self):
        widget = MapEditorWidget()
        try:
            widget.set_plan(Plan(), calibrated=True)
            widget.create_auto_path(QPointF(1800, 300), yaw_mode="fixed")
            sentinel = Waypoint(999, 888, 12)
            widget.plan.steps.append(sentinel)
            widget.active_index = 0

            widget.auto_corner_radius.setValue(80)
            self.assertFalse(widget._auto_paths_stale)
            self.assertIn("应用参数", widget.generate_segment_button.text())
            widget.generate_segment_button.click()

            self.assertEqual(widget.plan.steps[-1], sentinel)
            self.assertEqual(sum(isinstance(step, ContinuousPathSegment)
                                 for step in widget.plan.steps), 1)
            self.assertIn("自动规划完成", widget.status.text())
        finally:
            widget.close()

    def test_navigation_goal_snaps_centerline_and_near_right_angle(self):
        widget = MapEditorWidget()
        try:
            widget.set_plan(Plan(), calibrated=True)
            widget.set_mode("mark_pose")
            widget.on_map_click(1225, 300)

            self.assertEqual(widget._rviz_pose_anchor.x(), 1200)
            widget.update_preview(1300, 310)
            self.assertAlmostEqual(widget._rviz_drag_point.y(), 300, places=5)
            widget.on_map_click(1300, 310)
            self.assertEqual(widget.auto_goal_x.value(), 1200)
            self.assertEqual(widget.auto_goal_y.value(), 300)
        finally:
            widget.close()

    def test_screenshot_tangent_goal_exits_boundary_before_turning(self):
        widget = MapEditorWidget()
        try:
            widget.set_plan(Plan(), calibrated=True)
            widget._pending_navigation_goal_paper = QPointF(1200, 268.8)
            widget._pending_navigation_goal_yaw = -77.91
            widget._set_goal_controls(1200, 268.8, -77.91)
            widget.navigation_strategy.setCurrentIndex(
                widget.navigation_strategy.findData("tangent"))

            widget.generate_segment_button.click()

            self.assertIn("自动规划完成", widget.status.text())
            self.assertTrue(any(isinstance(step, ContinuousPathSegment)
                                for step in widget.plan.steps))
            self.assertIsNone(widget.auto_yaw_mode.parent())
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
            widget.plan.steps = [Waypoint(100, 200, 30),
                                 Waypoint(300, 400, 50)]
            widget.refresh_waypoints()
            self.assertEqual(widget.runtime_waypoint_list.count(), 2)
            widget.runtime_waypoint_list.setCurrentRow(0)
            self.assertEqual(widget.active_index, 0)
            self.assertEqual(widget.waypoint_list.currentRow(), 0)
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
            trace = next(item for item in widget.scene.items()
                         if item.data(0) == "runtime_trace")
            self.assertEqual(trace.pen().color().name(), "#8e24aa")
            self.assertEqual(trace.pen().style(), Qt.PenStyle.SolidLine)
            self.assertIn("X=110.0 mm", widget.runtime_position_label.text())
            self.assertIs(widget.execution_status_label.parentWidget(),
                          widget.runtime_page)

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

    def test_yellow_zone_passage_defaults_allowed_and_can_be_forbidden(self):
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

    def test_start_preset_position_and_heading_can_be_edited_numerically(self):
        widget = MapEditorWidget()
        try:
            widget.begin_start("启停区 1")
            widget.start_x_input.setValue(2235.0)
            widget.start_y_input.setValue(165.0)
            widget.start_heading_input.setValue(90.0)
            widget.apply_start_frame_button.click()

            self.assertEqual(widget.plan.start_paper_x_mm, 2235.0)
            self.assertEqual(widget.plan.start_paper_y_mm, 165.0)
            self.assertEqual(widget.plan.start_heading_deg, 90.0)
            self.assertEqual(widget.calibration_stage, "heading")
        finally:
            widget.close()

    def test_platform_soft_cost_overlay_is_split_at_outer_corner_boundary(self):
        widget = MapEditorWidget()
        try:
            widget.redraw()
            regions = {item.data(1) for item in widget.scene.items()
                       if item.data(0) == "soft_cost_zone"}
            self.assertIn("inner", regions)
            self.assertIn("outer", regions)
            inner = [item for item in widget.scene.items()
                     if item.data(0) == "soft_cost_zone" and
                     item.data(1) == "inner"]
            outer = [item for item in widget.scene.items()
                     if item.data(0) == "soft_cost_zone" and
                     item.data(1) == "outer"]
            self.assertEqual(len(inner), 4)
            self.assertEqual(len(outer), 1)
            self.assertTrue(any(item.path().contains(QPointF(1180, 775))
                                for item in inner))
            self.assertTrue(outer[0].path().contains(QPointF(500, 1200)))
            body_clearances = [item for item in widget.scene.items()
                               if item.data(0) == "platform_body_clearance"]
            safety_clearances = [item for item in widget.scene.items()
                                 if item.data(0) == "platform_safety_clearance"]
            self.assertEqual(len(body_clearances), 4)
            self.assertEqual(len(safety_clearances), 4)
            self.assertTrue(all(item.pen().color().name() == "#ec407a"
                                for item in body_clearances + safety_clearances))
            self.assertTrue(all(item.pen().style() == Qt.PenStyle.DashLine
                                for item in safety_clearances))
            self.assertTrue(any(
                item.data(0) == "platform_cost_split_boundary"
                for item in widget.scene.items()))
        finally:
            widget.close()

    def test_saved_default_costmap_and_editable_boundary_indent_controls(self):
        widget = MapEditorWidget()
        try:
            config = widget.current_costmap_settings()
            self.assertEqual(config.boundary_safety_margin_mm, 20)
            self.assertEqual(config.platform_inflation_mm, 30)
            self.assertEqual(config.platform_outer_inflation_mm, 260)
            self.assertEqual(config.platform_outer_cost_weight, 3.8)
            self.assertEqual(config.boundary_zone_half_width_mm, 200)
            self.assertEqual(config.boundary_zone_depth_mm, 85)
            self.assertEqual(config.side_zone_half_length_mm, 290)
            self.assertEqual(config.side_zone_depth_mm, 150)
            self.assertEqual(config.boundary_zone_inflation_mm, 35)
            self.assertEqual(len([item for item in widget.scene.items()
                                  if item.data(0) == "boundary_inset_zone"]), 3)
            body_line = next(item for item in widget.scene.items()
                             if item.data(0) == "boundary_inset_body_clearance")
            safety_line = next(item for item in widget.scene.items()
                               if item.data(0) == "boundary_inset_safety_clearance")
            self.assertEqual(body_line.pen().style(), Qt.PenStyle.SolidLine)
            self.assertEqual(safety_line.pen().style(), Qt.PenStyle.DashLine)
            self.assertEqual(body_line.pen().color().name(), "#d32f2f")
            self.assertEqual(safety_line.pen().color().name(), "#d32f2f")
            widget.boundary_zone_half_width.setValue(225)
            widget.boundary_zone_depth.setValue(95)
            self.assertEqual(widget.plan.layout.costmap.boundary_zone_half_width_mm,
                             225)
            self.assertEqual(widget.plan.layout.costmap.boundary_zone_depth_mm, 95)
            widget.side_zone_half_length.setValue(275)
            widget.side_zone_depth.setValue(145)
            self.assertEqual(widget.plan.layout.costmap.side_zone_half_length_mm,
                             275)
            self.assertEqual(widget.plan.layout.costmap.side_zone_depth_mm, 145)
            widget.boundary_zone_inflation.setValue(160)
            self.assertEqual(
                widget.plan.layout.costmap.boundary_zone_inflation_mm, 160)
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
