import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from topology_planner.gui import PlannerWindow
from topology_planner.planner import nodes


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


if __name__ == "__main__":
    unittest.main()
