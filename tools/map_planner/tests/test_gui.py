import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from map_planner.gui import PlannerWindow


class GuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_map_click_adds_waypoint_and_updates_properties(self):
        window = PlannerWindow()
        try:
            window.on_map_click(2250, 100)
            self.assertEqual(len(window.plan.waypoints), 1)
            window.waypoint_list.setCurrentRow(0)
            window.x.setValue(25)
            window.update_waypoint()
            self.assertEqual(window.plan.waypoints[0].x_mm, 25)
            window.move_waypoint(0, 2250, 80)
            self.assertNotEqual(window.plan.waypoints[0].y_mm, 0)
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
