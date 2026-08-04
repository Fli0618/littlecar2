import os
import time
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from topology_planner.gui import PlannerWindow
from topology_planner.planner import edge_key, nodes


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
            self.assertEqual(window.size().width(), 1440)
            self.assertEqual(window.size().height(), 900)
            self.assertGreaterEqual(window.minimumSize().width(), 1100)
            self.assertGreaterEqual(window.minimumSize().height(), 760)
            for node_id, node in window.node_items.items():
                self.assertIn(node_id, node.label_item.toPlainText())
                self.assertIn(nodes[node_id].label, node.label_item.toPlainText())
        finally:
            window.close()

    def test_graph_node_clicks_sync_start_and_goal(self):
        window = PlannerWindow()
        try:
            window.show()
            self.app.processEvents()
            nw_pos = window.view.mapFromScene(window.node_items["NW"].scenePos())
            QTest.mouseClick(window.view.viewport(), Qt.MouseButton.LeftButton, pos=nw_pos)
            self.assertEqual(window.start_combo.currentData(), "NW")
            se_pos = window.view.mapFromScene(window.node_items["SE"].scenePos())
            QTest.mouseClick(window.view.viewport(), Qt.MouseButton.RightButton, pos=se_pos)
            self.assertEqual(window.goal_combo.currentData(), "SE")
            window.set_start_node("NW")
            self.assertEqual(window.start_combo.currentData(), "NW")
            self.assertTrue(window.node_items["NW"].start_ring.isVisible())
            window.set_goal_node("SE")
            self.assertEqual(window.goal_combo.currentData(), "SE")
            self.assertTrue(window.node_items["SE"].goal_ring.isVisible())
            window.set_goal_node("NW")
            self.assertTrue(window.node_items["NW"].start_ring.isVisible())
            self.assertTrue(window.node_items["NW"].goal_ring.isVisible())
        finally:
            window.close()

    def test_edge_toggle_is_reversible(self):
        window = PlannerWindow()
        try:
            key = ("C", "N")
            window._toggle_edge(key)
            self.assertIn(tuple(sorted(key)), window.blocked_edges)
            window._toggle_edge(key)
            self.assertNotIn(tuple(sorted(key)), window.blocked_edges)
        finally:
            window.close()

    def test_mission_plan_controls_and_animation_update(self):
        window = PlannerWindow()
        try:
            self.assertEqual(window.tabs.currentIndex(), 0)
            self.assertEqual(
                [window.mission_start_combo.itemData(index) for index in range(2)],
                ["START1", "START2"],
            )
            self.assertFalse(window.play_mission_button.isEnabled())
            window.tabs.setCurrentIndex(1)
            window._generate_mission_plan()
            self.assertEqual(window.mission_list.count(), 8)
            self.assertTrue(window.play_mission_button.isEnabled())
            route_item_ids = tuple(id(item) for item in window.mission_route_items)

            window._play_mission()
            window._last_animation_time = time.monotonic() - 0.2
            window._advance_mission_animation()

            self.assertGreater(window.mission_progress.value(), 0)
            self.assertTrue(window.vehicle_item.isVisible())
            self.assertEqual(tuple(id(item) for item in window.mission_route_items), route_item_ids)
            self.assertFalse(window.pause_mission_button.text() == "继续")
        finally:
            window.close()

    def test_task_mode_ignores_single_segment_node_selection_and_invalidates_plan(self):
        window = PlannerWindow()
        try:
            window.tabs.setCurrentIndex(1)
            window._generate_mission_plan()
            initial_start = window.start_combo.currentData()
            initial_goal = window.goal_combo.currentData()
            window.set_start_node("NW")
            window.set_goal_node("SE")
            self.assertEqual(window.start_combo.currentData(), initial_start)
            self.assertEqual(window.goal_combo.currentData(), initial_goal)

            window._toggle_edge(("C", "N"))
            self.assertIsNone(window.mission_plan)
            self.assertFalse(window.play_mission_button.isEnabled())
        finally:
            window.close()

    def test_mission_start_change_invalidates_plan(self):
        window = PlannerWindow()
        try:
            window.tabs.setCurrentIndex(1)
            window._generate_mission_plan()
            self.assertIsNotNone(window.mission_plan)
            window.mission_start_combo.setCurrentIndex(1)
            self.assertIsNone(window.mission_plan)
            self.assertFalse(window.play_mission_button.isEnabled())
        finally:
            window.close()

    def test_unreachable_mission_reports_segment_and_clears_visuals(self):
        window = PlannerWindow()
        try:
            window.tabs.setCurrentIndex(1)
            window._generate_mission_plan()
            window._toggle_edge(edge_key("START1", "NE"))
            with patch("topology_planner.gui.QMessageBox.warning") as warning:
                window._generate_mission_plan()
            self.assertIsNone(window.mission_plan)
            self.assertFalse(window.play_mission_button.isEnabled())
            self.assertEqual(window.mission_route_items, [])
            self.assertIn("START1", warning.call_args.args[2])
            self.assertIn("QR", warning.call_args.args[2])
        finally:
            window.close()


if __name__ == "__main__":
    unittest.main()
