import os
import threading
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from pid_tuner.gui.app import GOTO_YAW_LABEL, MainWindow, format_telemetry_status, validate_motion_goal
from pid_tuner.gui.session import SessionController
from pid_tuner.models import MotionGoal, Telemetry


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
            self.assertTrue(window.use_yaw.isChecked())
            self.assertIsNone(validate_motion_goal(goal))
        finally:
            window.close()

    def test_invalid_motion_goal_is_rejected_locally(self) -> None:
        goal = MotionGoal(0.0, 0.0, 0.0, 0.0, 30.0, 5000)
        self.assertEqual(validate_motion_goal(goal), "vmax 必须在 0-600 mm/s 之间")

    def test_arrived_status_explains_that_motion_is_not_needed(self) -> None:
        telemetry = Telemetry(1, 2, 0, 2, 0x07, (0.0, 0.0, 0.0),
                              (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                              (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                              (0.0, 0.0, 0.0))
        self.assertIn("已到达，无需运动", format_telemetry_status(telemetry))

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
