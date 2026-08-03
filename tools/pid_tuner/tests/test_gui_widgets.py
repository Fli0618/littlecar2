import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from pid_tuner.gui.widgets import (
    HEADING_MODE_NONE,
    HEADING_MODE_OPS,
    ConnectionMotionPanel,
    ConnectionPanel,
    PidControlPanel,
)
from pid_tuner.models import PidConfig


class GuiWidgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_pid_panel_exposes_and_updates_pid_values(self) -> None:
        panel = PidControlPanel()
        requested: list[PidConfig] = []
        panel.apply_requested.connect(requested.append)
        panel.set_pid(PidConfig(3, .2, .4, 5, .6, .8))

        panel.apply_pid.click()

        self.assertEqual(panel.current_pid(), PidConfig(3, .2, .4, 5, .6, .8))
        self.assertEqual(requested, [PidConfig(3, .2, .4, 5, .6, .8)])

    def test_connection_motion_panel_emits_connection_and_single_axis_goals(self) -> None:
        panel = ConnectionMotionPanel()
        connections: list[tuple[str, int]] = []
        goals = []
        panel.connect_requested.connect(lambda port, baud: connections.append((port, baud)))
        panel.motion_requested.connect(goals.append)
        panel.set_available_ports(["COM9"])

        panel.connect_button.click()
        panel.goto_position.click()
        panel.goto_yaw.click()

        self.assertEqual(connections, [("COM9", 115200)])
        self.assertEqual(len(goals), 2)
        self.assertTrue(goals[0].use_position)
        self.assertFalse(goals[0].use_yaw)
        self.assertFalse(goals[1].use_position)
        self.assertTrue(goals[1].use_yaw)

    def test_heading_mode_drives_combined_goal_and_yaw_controls(self) -> None:
        panel = ConnectionMotionPanel()
        modes: list[str] = []
        goals = []
        panel.yaw_source_requested.connect(modes.append)
        panel.motion_requested.connect(goals.append)

        panel.yaw_source.setCurrentIndex(panel.yaw_source.findData(HEADING_MODE_NONE))
        panel.goto.click()

        self.assertEqual(modes, [HEADING_MODE_NONE])
        self.assertFalse(goals[-1].use_yaw)
        self.assertTrue(goals[-1].use_position)
        self.assertFalse(panel.goal[2].isEnabled())
        self.assertFalse(panel.goal[4].isEnabled())
        self.assertFalse(panel.goto_yaw.isEnabled())
        self.assertIn("仅位置", panel.goto.text())

        panel.yaw_source.setCurrentIndex(panel.yaw_source.findData(HEADING_MODE_OPS))
        panel.goto.click()

        self.assertEqual(modes[-1], HEADING_MODE_OPS)
        self.assertTrue(goals[-1].use_yaw)
        self.assertTrue(panel.goal[2].isEnabled())
        self.assertTrue(panel.goto_yaw.isEnabled())

    def test_connection_panel_switches_between_connect_and_disconnect(self) -> None:
        panel = ConnectionPanel()
        disconnected: list[bool] = []
        panel.disconnect_requested.connect(lambda: disconnected.append(True))

        panel.set_connected(True)
        panel.connect_button.click()

        self.assertEqual(panel.connect_button.text(), "断开")
        self.assertEqual(disconnected, [True])

    def test_connection_panel_requires_a_selected_port(self) -> None:
        panel = ConnectionPanel()
        requested: list[tuple[str, int]] = []
        panel.connect_requested.connect(lambda port, baud: requested.append((port, baud)))
        self.assertFalse(panel.connect_button.isEnabled())
        panel.set_available_ports(["COM7"])
        self.assertTrue(panel.connect_button.isEnabled())
        panel.connect_button.click()
        self.assertEqual(requested, [("COM7", 115200)])


if __name__ == "__main__":
    unittest.main()
