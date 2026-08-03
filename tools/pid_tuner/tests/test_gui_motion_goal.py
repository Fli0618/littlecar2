import os
import threading
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QAbstractSpinBox

from pid_tuner.gui.app import GOTO_YAW_LABEL, MainWindow, format_pid_apply_log, format_telemetry_status, validate_motion_goal
from pid_tuner.gui.session import SessionController
from pid_tuner.models import MotionGoal, PidConfig, Telemetry


class GuiMotionGoalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_default_motion_goal_is_accepted(self) -> None:
        window = MainWindow()
        try:
            goal = MotionGoal(*(widget.value() for widget in window.goal), window.timeout.value())
            self.assertEqual((goal.x_mm, goal.y_mm, goal.yaw_deg), (0.0, 0.0, 0.0))
            self.assertEqual((goal.vmax_mm_s, goal.wmax_deg_s, goal.timeout_ms), (600.0, 120.0, 15000))
            self.assertTrue(window.connection_motion.uses_yaw())
            self.assertFalse(window.large_yaw_align.isChecked())
            self.assertFalse(window.large_yaw_align.isEnabled())
            self.assertIsNone(validate_motion_goal(goal))
        finally:
            window.close()

    def test_invalid_motion_goal_is_rejected_locally(self) -> None:
        goal = MotionGoal(0.0, 0.0, 0.0, 0.0, 30.0, 5000)
        self.assertEqual(validate_motion_goal(goal), "vmax 必须在 0-1200 mm/s 之间")

    def test_vmax_limit_accepts_1200_and_rejects_values_above_it(self) -> None:
        at_limit = MotionGoal(0.0, 0.0, 0.0, 1200.0, 120.0, 5000)
        above_limit = MotionGoal(0.0, 0.0, 0.0, 1200.1, 120.0, 5000)
        self.assertIsNone(validate_motion_goal(at_limit))
        self.assertEqual(validate_motion_goal(above_limit), "vmax 必须在 0-1200 mm/s 之间")

    def test_independent_motion_goals_validate_only_their_used_axis(self) -> None:
        self.assertIsNone(validate_motion_goal(MotionGoal(0, 0, 0, 0, 30, 1000, use_position=False)))
        self.assertIsNone(validate_motion_goal(MotionGoal(0, 0, 0, 300, 0, 1000, use_yaw=False)))

    def test_pid_apply_log_contains_revision_and_all_values(self) -> None:
        text = format_pid_apply_log(7, PidConfig(1, .03, .1, 2, .05, .08))
        self.assertIn("r7", text)
        self.assertIn("kp_pos=1.0000", text)
        self.assertIn("kd_yaw=0.0800", text)

    def test_float_inputs_hide_spinbox_buttons(self) -> None:
        window = MainWindow()
        try:
            inputs = [*window.pid, *window.goal]
            self.assertTrue(inputs)
            self.assertTrue(all(widget.buttonSymbols() == QAbstractSpinBox.ButtonSymbols.NoButtons for widget in inputs))
        finally:
            window.close()

    def test_low_resolution_uses_scrollable_controls_and_plots(self) -> None:
        window = MainWindow()
        try:
            window.resize(800, 600)
            window.show()
            self.app.processEvents()
            self.assertIsNotNone(window.controls_scroll.widget())
            self.assertIs(window.plots_scroll.widget(), window.plots)
            self.assertGreaterEqual(window.plots.minimumWidth(), 900)
            self.assertGreaterEqual(window.plots.minimumHeight(), 820)
        finally:
            window.close()

    def test_arrived_status_explains_that_motion_is_not_needed(self) -> None:
        telemetry = Telemetry(1, 2, 0, 2, 0x07, (0.0, 0.0, 0.0),
                              (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                              (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                              (0.0, 0.0, 0.0))
        self.assertIn("已到达，无需运动", format_telemetry_status(telemetry))

    def test_heartbeat_timeout_is_visible_in_gui_status(self) -> None:
        window = MainWindow()
        try:
            telemetry = Telemetry(1, 2, 0, 6, 0x07, (0.0, 0.0, 0.0),
                                  (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                  (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                  (0.0, 0.0, 0.0), remote_link_status=0x4000)
            window.on_telemetry(telemetry)
            self.assertIn("心跳超时停车", window.status.text())
        finally:
            window.close()

    def test_goto_strategy_control_is_disabled_while_motion_is_active(self) -> None:
        window = MainWindow()
        try:
            window.session.connected = True
            window.on_goto_strategy_changed(True)
            self.assertTrue(window.large_yaw_align.isEnabled())
            window.on_motion_changed(True)
            self.assertFalse(window.large_yaw_align.isEnabled())
            window.on_motion_changed(False)
            self.assertTrue(window.large_yaw_align.isEnabled())
        finally:
            window.close()

    def test_yaw_label_is_relative_to_initialization_zero(self) -> None:
        self.assertEqual(GOTO_YAW_LABEL, "yaw 相对初始化零点 deg")


    def test_heartbeat_does_not_queue_while_previous_request_is_running(self) -> None:
        class BlockingClient:
            def __init__(self) -> None:
                self.calls = 0
                self.started = threading.Event()
                self.release = threading.Event()

            def heartbeat(self) -> None:
                self.calls += 1
                self.started.set()
                self.release.wait(1.0)

        controller = SessionController()
        client = BlockingClient()
        controller._client = client  # type: ignore[assignment]
        controller.connected = True
        controller.motion_active = True
        try:
            for _ in range(10):
                controller.heartbeat()
            self.assertTrue(client.started.wait(1.0))
            self.assertEqual(client.calls, 1)
            client.release.set()
            time.sleep(0.05)
            controller.heartbeat()
            deadline = time.monotonic() + 1.0
            while client.calls != 2 and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual(client.calls, 2)
        finally:
            client.release.set()
            controller._executor.shutdown(wait=True, cancel_futures=True)

    def test_stop_and_terminal_telemetry_disable_heartbeat(self) -> None:
        controller = SessionController()
        controller.motion_active = True
        try:
            controller.stop()
            self.assertFalse(controller.motion_active)
            controller.motion_active = True
            controller._handle_telemetry(Telemetry(1, 1, 0, 2, 0, (0.0, 0.0, 0.0),
                                                  (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                                  (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                                  (0.0, 0.0, 0.0)))
            self.assertFalse(controller.motion_active)
        finally:
            controller._executor.shutdown(wait=True, cancel_futures=True)

if __name__ == "__main__":
    unittest.main()
