import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QPoint, QPointF, QRectF, Qt
from PySide6.QtTest import QTest

from map_planner.gui import PlannerWindow


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
            window.on_map_click(2250, 100)
            self.assertEqual(len(window.plan.waypoints), 1)
            window.waypoint_list.setCurrentRow(0)
            window.x.setValue(25)
            window.update_waypoint()
            self.assertEqual(window.plan.waypoints[0].x_mm, 25)
            window.move_waypoint(0, QPointF(2250, 100), QPointF(2250, 80))
            self.assertNotEqual(window.plan.waypoints[0].y_mm, 0)
        finally:
            window.close()

    def test_box_selection_and_undo_restore_plan(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.on_map_click(2250, 100)
            window.on_map_click(2200, 100)
            window.select_box(QRectF(2100, 0, 250, 250), False)
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
            self.assertFalse(window.plan.waypoints[-1].use_yaw)
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

    def test_selected_nodes_move_as_a_group(self):
        window = PlannerWindow()
        try:
            self.calibrated(window)
            window.on_map_click(2000, 300)
            window.on_map_click(1900, 400)
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
            window.set_mode("select")
            self.assertEqual(window.measurement_points, [])
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
            self.assertFalse(window.plan.waypoints[0].use_yaw)
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
