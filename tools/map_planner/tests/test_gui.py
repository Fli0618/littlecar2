import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QPushButton
from PySide6.QtCore import QPoint, QPointF, QRectF, Qt
from PySide6.QtTest import QTest

from map_planner.gui import PlannerWindow, StartItem
from map_planner.models import RotateInPlace


class GuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    @staticmethod
    def calibrated(window):
        window.calibration_pending = False
        window.calibration_stage = "complete"
        window.update_calibration_ui()
        window.set_mode("add")

    def test_map_click_adds_waypoint_and_updates_properties(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.on_map_click(2200, 200)
            self.assertEqual(len(window.plan.waypoints), 1)
            window.waypoint_list.setCurrentRow(0)
            window.x.setValue(25)
            window.update_waypoint()
            self.assertEqual(window.plan.waypoints[0].x_mm, 25)
            window.move_waypoint(0, QPointF(2200, 200), QPointF(2200, 220))
            self.assertNotEqual(window.plan.waypoints[0].y_mm, 0)
        finally:
            window.close()

    def test_box_selection_and_undo_restore_plan(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.on_map_click(2200, 200)
            window.on_map_click(2100, 250)
            window.select_box(QRectF(2050, 150, 250, 200), False)
            self.assertEqual(len(window.scene.selectedItems()), 2)
            window.remove_waypoint()
            self.assertEqual(len(window.plan.waypoints), 0)
            window.undo()
            self.assertEqual(len(window.plan.waypoints), 2)
        finally:
            window.close()

    def test_numeric_input_has_no_buttons(self):
        window = PlannerWindow()
        try:
            self.assertEqual(window.vmax.buttonSymbols().name, "NoButtons")
        finally:
            window.close()

    def test_empty_mode_switch_exposes_continuous_editor_and_adds_pose_point(self):
        window = PlannerWindow()
        try:
            window.plan_mode_combo.setCurrentIndex(window.plan_mode_combo.findData("continuous"))
            self.assertEqual(window.plan.mode, "continuous")
            self.assertFalse(window.continuous_panel.isHidden())
            self.calibrated(window)
            window.on_map_click(2200, 200)
            self.assertEqual(len(window.plan.path_points), 1)
            self.assertEqual(len(window.plan.waypoints), 0)
            self.assertTrue(any(item.data(0) == "continuous_path_point" for item in window.scene.items()))
        finally:
            window.close()

    def test_plan_toolbar_has_rename_button(self):
        window = PlannerWindow()
        try:
            self.assertTrue(any(button.text() == "重命名" for button in window.findChildren(QPushButton)))
        finally:
            window.close()

    def test_complete_material_map_items_are_drawn(self):
        window = PlannerWindow()
        try:
            markers = [item.data(0) for item in window.scene.items()]
            self.assertIn("raw_turntable", markers)
            self.assertEqual(markers.count("raw_pick_hole"), 3)
            self.assertEqual(markers.count("material_slot_outer"), 6)
            self.assertEqual(markers.count("material_slot_inner"), 6)
            self.assertIn("storage_label", markers)
            self.assertIn("rough_label", markers)
        finally:
            window.close()

    def test_toolbar_is_exclusive_and_ctrl_a_selects_nodes(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.on_map_click(2200, 200)
            window.on_map_click(2100, 300)
            self.assertTrue(window.add_button.isChecked())
            self.assertFalse(window.select_button.isChecked())
            window.select_all()
            self.assertEqual(len([item for item in window.scene.selectedItems() if hasattr(item, "index")]), 2)
        finally:
            window.close()

    def test_space_left_mouse_and_middle_mouse_start_panning(self):
        window = PlannerWindow()
        try:
            window.show()
            window.view.setFocus()
            QTest.keyPress(window.view, Qt.Key.Key_Space)
            QTest.mousePress(window.view.viewport(), Qt.MouseButton.LeftButton, pos=QPoint(80, 80))
            self.assertTrue(window.view._panning)
            QTest.mouseRelease(window.view.viewport(), Qt.MouseButton.LeftButton, pos=QPoint(80, 80))
            QTest.keyRelease(window.view, Qt.Key.Key_Space)
            QTest.mousePress(window.view.viewport(), Qt.MouseButton.MiddleButton, pos=QPoint(80, 80))
            self.assertTrue(window.view._panning)
            QTest.mouseRelease(window.view.viewport(), Qt.MouseButton.MiddleButton, pos=QPoint(80, 80))
        finally:
            window.close()

    def test_shift_snap_and_goto_pose_defaults(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.on_map_click(2200, 200)
            window.on_map_click(2100, 260, True)
            self.assertTrue(window.plan.waypoints[-1].stop)
            self.assertTrue(window.plan.waypoints[-1].use_yaw)
            point = window.paper_of(window.plan.waypoints[-1])
            self.assertAlmostEqual(abs(point.x_mm - 2200), abs(point.y_mm - 200), delta=1)
        finally:
            window.close()

    def test_calibration_gates_route_editing(self):
        window = PlannerWindow()
        try:
            window.set_mode("add")
            window.on_map_click(2000, 200)
            self.assertEqual(window.mode, "select")
            self.assertEqual(window.plan.waypoints, [])
            self.assertFalse(window.add_button.isEnabled())
            window.begin_start("启停区 1")
            self.assertEqual(window.calibration_stage, "heading")
        finally:
            window.close()

    def test_hover_preview_snaps_rotates_and_confirms_on_left_release(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.plan.start_paper_x_mm, window.plan.start_paper_y_mm = 2000, 300
            window.redraw()
            window.update_preview(2100, 300, True)
            self.assertIsNotNone(window.preview_paper)
            self.assertTrue(window.preview_shift)
            self.assertTrue(any(item.data(0) == "snap_preview_axis" for item in window.scene.items()))
            window.rotate_preview_clockwise()
            self.assertEqual(window.preview_yaw_deg, -90)
            window.show(); self.app.processEvents()
            position = window.view.mapFromScene(QPointF(2100, 300))
            QTest.mousePress(window.view.viewport(), Qt.MouseButton.LeftButton, pos=position)
            self.assertEqual(window.plan.waypoints, [])
            QTest.mouseRelease(window.view.viewport(), Qt.MouseButton.LeftButton, pos=position)
            self.assertEqual(len(window.plan.waypoints), 1)
            self.assertTrue(window.plan.waypoints[0].use_yaw)
            self.assertEqual(window.plan.waypoints[0].yaw_deg, -90)
            self.assertIsNone(window.preview_paper)
        finally:
            window.close()

    def test_start_heading_uses_repeated_right_click_rotation_and_confirmation(self):
        window = PlannerWindow()
        try:
            window.begin_start("启停区 1")
            self.assertEqual(window.calibration_stage, "heading")
            self.assertFalse(window.confirm_start_button.isHidden())
            initial_heading = window.plan.start_heading_deg
            window.rotate_start_clockwise()
            window.rotate_start_clockwise()
            self.assertEqual(window.plan.start_heading_deg, ((initial_heading - 180 + 180) % 360) - 180)
            self.assertTrue(window.calibration_pending)
            window.confirm_start_heading()
            self.assertFalse(window.calibration_pending)
            self.assertEqual(window.calibration_stage, "complete")
            self.assertEqual(window.mode, "select")
            self.assertTrue(window.confirm_start_button.isHidden())
        finally:
            window.close()

    def test_right_click_on_start_arrow_rotates_heading(self):
        window = PlannerWindow()
        try:
            window.begin_start("启停区 1")
            window.show()
            self.app.processEvents()
            start = next(item for item in window.scene.items() if isinstance(item, StartItem))
            initial_heading = window.plan.start_heading_deg
            position = window.view.mapFromScene(start.scenePos())
            QTest.mouseClick(window.view.viewport(), Qt.MouseButton.RightButton, pos=position)
            self.assertEqual(window.plan.start_heading_deg, ((initial_heading - 90 + 180) % 360) - 180)
        finally:
            window.close()

    def test_new_waypoint_uses_820_mm_per_second_default(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.on_map_click(2200, 200)
            self.assertEqual(window.plan.waypoints[0].vmax_mm_s, 820)
            self.assertEqual(window.node_vmax.value(), 820)
        finally:
            window.close()

    def test_rotation_actions_can_be_appended_and_inserted(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.on_map_click(2200, 200)
            window.append_rotation()
            self.assertIsInstance(window.plan.waypoints[1], RotateInPlace)
            window.active_index = 0
            window.insert_rotation_after_active()
            self.assertIsInstance(window.plan.waypoints[1], RotateInPlace)
            self.assertEqual(len(window.plan.waypoints), 3)
            window.active_index = 1
            window.remove_waypoint()
            self.assertEqual(len(window.plan.waypoints), 2)
        finally:
            window.close()

    def test_code_generator_dialog_validates_name_and_previews_code(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.on_map_click(2200, 200)
            window.open_code_generator()
            self.app.processEvents()
            dialog = window.codegen_dialog
            self.assertTrue(dialog.copy_button.isEnabled())
            self.assertIn("void Task_", dialog.code_preview.toPlainText())
            self.assertIn("AdvanceMotion_GotoPoseBlocking", dialog.code_preview.toPlainText())
            dialog.function_name_edit.setText("Task_1bad")
            dialog.regenerate()
            self.assertFalse(dialog.copy_button.isEnabled())
            dialog.function_name_edit.setText("Task_Valid")
            dialog.regenerate()
            self.assertTrue(dialog.copy_button.isEnabled())
        finally:
            window.close()

    def test_code_generator_reports_unsupported_yaw_constraint(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.on_map_click(2200, 200)
            window.plan.waypoints[0].use_yaw = False
            window.open_code_generator()
            self.assertIn("未启用航向约束", window.status.text())
            self.assertFalse(hasattr(window, "codegen_dialog"))
        finally:
            window.close()

    def test_right_click_rotation_updates_selected_rotate_action(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.append_rotation()
            window.rotate_car_clockwise()
            self.assertEqual(window.plan.waypoints[0].yaw_deg, -90)
        finally:
            window.close()

    def test_rotation_markers_follow_last_goto_and_do_not_break_route_lines(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.append_rotation()
            marker = next(item for item in window.scene.items() if item.data(0) == "rotate_in_place_marker")
            self.assertEqual(marker.rect().center() + marker.pos(), QPointF(2250, 150))
            window.on_map_click(2200, 200)
            window.append_rotation()
            markers = [item for item in window.scene.items() if item.data(0) == "rotate_in_place_marker"]
            self.assertEqual(len(markers), 2)
            self.assertEqual(markers[0].rect().center() + markers[0].pos(), QPointF(2200, 200))
        finally:
            window.close()


    def test_selected_nodes_move_as_a_group(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.on_map_click(2200, 300)
            window.on_map_click(2100, 400)
            window.select_all()
            before=[window.paper_of(point) for point in window.plan.waypoints]
            window.active_index=1
            window.move_waypoint(1,QPointF(before[1].x_mm,before[1].y_mm),QPointF(before[1].x_mm+40,before[1].y_mm+20))
            after=[window.paper_of(point) for point in window.plan.waypoints]
            for old,new in zip(before,after):
                self.assertAlmostEqual(new.x_mm-old.x_mm,40)
                self.assertAlmostEqual(new.y_mm-old.y_mm,20)
        finally:
            window.close()

    def test_measurement_mode_reports_absolute_distances_without_adding_node(self):
        window = PlannerWindow()
        try:
            window.set_mode("measure")
            window.on_map_click(100, 200)
            window.on_map_click(400, 600)
            self.assertEqual(window.plan.waypoints, [])
            self.assertIn("300.0 mm", window.measurement_label.text())
            self.assertIn("400.0 mm", window.measurement_label.text())
            self.assertIn("500.0 mm", window.measurement_label.text())
            self.assertEqual(len([item for item in window.scene.items() if item.data(0) == "measurement_horizontal_guide"]), 1)
            self.assertEqual(len([item for item in window.scene.items() if item.data(0) == "measurement_vertical_guide"]), 1)
            window.set_mode("select")
            self.assertEqual(window.measurement_points, [])
            self.assertFalse(any(item.data(0) == "measurement_horizontal_guide" for item in window.scene.items()))
        finally:
            window.close()

    def test_measurement_shift_snaps_second_point(self):
        window = PlannerWindow()
        try:
            window.set_mode("measure")
            window.on_map_click(100, 200)
            window.on_map_click(400, 300, True)
            point = window.measurement_points[1]
            self.assertTrue(point.y() == 200 or point.x() == 100 or abs(point.x() - 100) == abs(point.y() - 200))
        finally:
            window.close()

    def test_obstacles_and_edge_objects_are_drag_limited(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.add_obstacle(QPointF(100, 100))
            obstacle = window.plan.layout.obstacles[0]
            window.move_obstacle(0, QPointF(-10, 2500))
            self.assertEqual((obstacle.paper_x_mm, obstacle.paper_y_mm), (25, 2375))
            window.move_obstacle(0, QPointF(300, 400))
            self.assertEqual((obstacle.paper_x_mm, obstacle.paper_y_mm), (300, 400))
            obstacle_item = next(item for item in window.scene.items() if item.data(0) == "obstacle")
            obstacle_item.setSelected(True)
            window.remove_waypoint()
            self.assertEqual(window.plan.layout.obstacles, [])
            window.move_raw_area(QPointF(1400, 0))
            window.move_qr_board(QPointF(0, 1000))
            self.assertEqual(window.plan.layout.raw_center_x_mm, 1300)
            self.assertEqual(window.plan.layout.qr_center_y_mm, 1100)
        finally:
            window.close()

    def test_platforms_reject_intersecting_route_segments(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            self.assertFalse(window.is_valid_route_segment(QPointF(2250, 150), QPointF(1000, 600)))
            self.assertTrue(window.is_valid_route_segment(QPointF(2250, 150), QPointF(2200, 300)))
            window.update_preview(1000, 600)
            window.confirm_preview(1000, 600)
            self.assertEqual(window.plan.waypoints, [])
        finally:
            window.close()

    def test_preview_sweep_uses_actual_yaw_and_marks_invalid_areas(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.update_preview(2200, 300)
            self.assertTrue(any(item.data(0) == "preview_sweep" for item in window.scene.items()))
            self.assertFalse(any(item.data(0) == "preview_platform_collision" for item in window.scene.items()))
            sweep = window.route_sweep(QPointF(2250, 150), QPointF(2200, 300), 0, 90)
            self.assertNotEqual(sweep.polygons[0], sweep.polygons[-1])
            window.update_preview(1000, 600)
            self.assertTrue(any(item.data(0) == "preview_platform_collision" for item in window.scene.items()))
            window.update_preview(2350, 150)
            out_of_bounds, collisions = window.sweep_violations(window.route_sweep(QPointF(2250, 150), QPointF(2350, 150)))
            self.assertTrue(out_of_bounds)
            self.assertTrue(collisions.isEmpty())
        finally:
            window.close()

    def test_start_pose_uses_the_same_sweep_boundary_check(self):
        window = PlannerWindow()
        try:
            window.plan.start_paper_x_mm, window.plan.start_paper_y_mm = 775, 775
            self.assertFalse(window.is_valid_start_pose())
            window.plan.start_paper_x_mm, window.plan.start_paper_y_mm = 2250, 150
            self.assertTrue(window.is_valid_start_pose())
        finally:
            window.close()

    def test_rotation_sweep_rejects_boundary_and_accepts_safe_position(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.append_rotation()
            window.plan.waypoints[0].yaw_deg = 45
            window.redraw()
            self.assertEqual(window.invalid_waypoints(), [0])
            marker = next(item for item in window.scene.items() if item.data(0) == "rotate_in_place_marker")
            self.assertEqual(marker.pen().color().name(), "#c62828")
            window.plan.start_paper_x_mm, window.plan.start_paper_y_mm = 1200, 1200
            self.assertEqual(window.invalid_waypoints(), [])
        finally:
            window.close()

    def test_layout_sliders_stay_outside_the_competition_area(self):
        window = PlannerWindow()
        try:
            window.show(); self.app.processEvents(); window.fit_map(); self.app.processEvents()
            raw_corners = [window.view.mapToScene(point) for point in (window.raw_slider.geometry().topLeft(), window.raw_slider.geometry().bottomRight())]
            qr_corners = [window.view.mapToScene(point) for point in (window.qr_slider.geometry().topLeft(), window.qr_slider.geometry().bottomRight())]
            self.assertTrue(all(point.y() < 0 for point in raw_corners))
            self.assertTrue(all(point.x() > 2400 for point in qr_corners))
        finally:
            window.close()

    def test_action_edit_generates_a_paused_seekable_timeline(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.on_map_click(2200, 300)
            self.assertTrue(window.timeline)
            self.assertTrue(window.progress.isEnabled())
            self.assertFalse(window.timer.isActive())
            window.progress.setValue(max(1, window.progress.maximum() // 2))
            self.assertGreater(window.timeline_position, 0)
        finally:
            window.close()

    def test_layout_sliders_update_at_one_millimeter_and_are_undoable(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.on_map_click(2200, 300)
            window.play(); window.pause()
            self.assertTrue(window.timeline)
            window._begin_layout_slider_edit()
            window.raw_slider.setValue(1257)
            window.qr_slider.setValue(1143)
            window._finish_layout_slider_edit()
            self.assertEqual(window.plan.layout.raw_center_x_mm, 1257)
            self.assertEqual(window.plan.layout.qr_center_y_mm, 1143)
            self.assertEqual(window.raw_slider.singleStep(), 1)
            self.assertEqual(window.qr_slider.singleStep(), 1)
            self.assertTrue(window.timeline)
            window.undo()
            self.assertEqual(window.plan.layout.raw_center_x_mm, 1200)
            self.assertEqual(window.plan.layout.qr_center_y_mm, 1200)
        finally:
            window.close()

    def test_right_click_rotation_updates_active_waypoint_heading(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.on_map_click(2200, 200)
            window.rotate_car_clockwise()
            waypoint = window.plan.waypoints[0]
            self.assertEqual(waypoint.yaw_deg, -90)
            self.assertTrue(waypoint.use_yaw)
            window.undo()
            self.assertTrue(window.plan.waypoints[0].use_yaw)
        finally:
            window.close()

    def test_timeline_slider_seeks_and_stays_paused(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.on_map_click(2200, 200)
            window.play()
            window.pause()
            self.assertTrue(window.progress.isEnabled())
            middle = max(1, window.progress.maximum() // 2)
            window.progress.setValue(middle)
            self.assertEqual(window.timeline_position, middle)
            self.assertFalse(window.timer.isActive())
            self.assertEqual(len(window.actual_trace), middle)
        finally:
            window.close()

    def test_raw_turntable_slots_follow_documented_geometry(self):
        window = PlannerWindow()
        try:
            holes = [item for item in window.scene.items() if item.data(0) == "raw_pick_hole"]
            self.assertEqual(len(holes), 3)
            for hole in holes:
                center = hole.rect().center() + hole.pos()
                self.assertAlmostEqual((center.x() - 1200) ** 2 + (center.y() + 70) ** 2, 10000, delta=1)
        finally:
            window.close()
