import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from map_planner.gui import MapEditorWidget
from map_planner.models import Plan, Pose, Waypoint


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

    def test_canvas_selection_and_runtime_overlay_api(self):
        widget = MapEditorWidget()
        selected = []
        overlays = []
        widget.candidate_selected.connect(selected.append)
        widget.runtime_overlay_changed.connect(overlays.append)
        try:
            widget.set_plan(Plan(steps=[Waypoint(100, 200, 30)]))
            self.assertIs(widget.canvas, widget.view)
            widget.select_candidate(0)
            self.assertEqual((widget.selected_candidate_index, selected), (0, [0]))

            widget.set_runtime_pose(Pose(300, 400, 90))
            self.assertIn("runtime_car", [item.data(0) for item in widget.scene.items()])
            widget.clear_runtime_pose()
            self.assertEqual(len(overlays), 2)
            self.assertNotIn("runtime_car", [item.data(0) for item in widget.scene.items()])
        finally:
            widget.close()
