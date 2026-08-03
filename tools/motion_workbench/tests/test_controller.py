from __future__ import annotations

import unittest

from PySide6.QtCore import QObject, Signal

from pid_tuner.models import MotionGoal, Telemetry

from motion_workbench.controller import MotionWorkbenchController
from motion_workbench.models import TargetPose


class FakeSession(QObject):
    telemetry = Signal(object)
    motion_changed = Signal(bool)
    status = Signal(str)
    failure = Signal(str)

    def __init__(self) -> None:
        super().__init__(); self.started: list[MotionGoal] = []; self.stopped = 0

    def start_motion(self, goal: MotionGoal) -> None: self.started.append(goal)
    def stop(self) -> None: self.stopped += 1


class ControllerTests(unittest.TestCase):
    def test_candidate_is_snapshotted_when_explicit_send_occurs(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        controller.select_candidate(TargetPose(10, 20, 30))
        controller.start_goal(MotionGoal(10, 20, 30, 100, 50, 1000))
        controller.select_candidate(TargetPose(40, 50, 60))
        self.assertEqual(controller.execution, TargetPose(10, 20, 30))
        self.assertEqual(len(session.started), 1)

    def test_no_command_is_sent_for_candidate_selection(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        controller.select_candidate(TargetPose(10, 20, 30))
        self.assertEqual(session.started, [])

    def test_actual_trace_uses_valid_telemetry_only(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        item = Telemetry(1, 0, 0, 0, 0x03, (0, 0, 0), (3, 4, 5), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0))
        session.telemetry.emit(item)
        self.assertEqual(controller.trace, (TargetPose(3, 4, 5),))
