from __future__ import annotations

import unittest

from PySide6.QtCore import QObject, Signal

from pid_tuner.models import MotionGoal, Telemetry
from map_planner.models import (BezierPathSegment, ContinuousPathSegment, PathPosePoint, Plan,
                                RotateInPlace, StepTurnNode, StepTurnPathSegment, Waypoint)

from motion_workbench.controller import MotionWorkbenchController
from motion_workbench.models import (CoordinateSyncState, PathUploadState, PlanExecutionState,
                                     TargetPose)


class FakeSession(QObject):
    telemetry = Signal(object)
    motion_changed = Signal(bool)
    status = Signal(str)
    failure = Signal(str)

    def __init__(self) -> None:
        super().__init__(); self.started: list[MotionGoal] = []; self.stopped = 0
        self.uploaded = []; self.paths_started = []
        self.connected = True; self.motion_active = False

    def start_motion(self, goal: MotionGoal) -> None: self.started.append(goal)
    def stop(self) -> None: self.stopped += 1
    def upload_path(self, begin, chunks, commit) -> None: self.uploaded.append((begin, chunks, commit))
    def start_path(self, command) -> None: self.paths_started.append(command)
    def upload_and_start_path(self, begin, chunks, commit, start) -> None:
        self.upload_path(begin, chunks, commit); self.start_path(start)
    def reset_origin(self) -> None: pass


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

    def test_single_workflow_sends_only_current_step_and_advances_after_success(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        controller.set_plan(Plan(steps=[Waypoint(10, 20, 30), RotateInPlace(90)]))

        self.assertTrue(controller.start_single(0))
        self.assertEqual([goal.x_mm for goal in session.started], [10])
        session.telemetry.emit(self._telemetry(2))

        self.assertEqual(controller.plan_execution.cursor, 1)
        self.assertEqual(controller.plan_execution.state, PlanExecutionState.IDLE)
        self.assertEqual(len(session.started), 1)

    def test_continuous_workflow_waits_for_terminal_telemetry_before_next_step(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        controller.set_plan(Plan(steps=[Waypoint(10, 20, 30), RotateInPlace(90)]))

        self.assertTrue(controller.start_continuous(0))
        self.assertEqual(len(session.started), 1)
        session.telemetry.emit(self._telemetry(1))
        self.assertEqual(len(session.started), 1)
        session.telemetry.emit(self._telemetry(2))

        self.assertEqual(len(session.started), 2)
        self.assertEqual(session.started[-1], MotionGoal(10, 20, 90, 0.0, 90.0, 15000, True, False))
        session.telemetry.emit(self._telemetry(2))
        self.assertEqual(controller.plan_execution.state, PlanExecutionState.COMPLETED)

    def test_continuous_path_and_bezier_use_session_path_operations(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        controller.set_plan(Plan(steps=[
            ContinuousPathSegment([PathPosePoint(0, 0, 0), PathPosePoint(100, 0, 0)]),
            BezierPathSegment(125, 0, 175, 0, 200, 0, 0),
        ]))

        controller.start_continuous(0)
        self.assertEqual(len(session.uploaded), 1)
        self.assertEqual(len(session.paths_started), 1)
        session.telemetry.emit(self._telemetry(2))
        self.assertEqual(len(session.uploaded), 2)
        self.assertEqual(len(session.paths_started), 2)

    def test_step_turn_uploads_once_and_starts_one_path(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        controller.set_plan(Plan(steps=[StepTurnPathSegment(
            [StepTurnNode(200, 0), StepTurnNode(200, 200)], 50,
        )]))

        self.assertTrue(controller.start_single(0))
        self.assertEqual(len(session.uploaded), 1)
        self.assertEqual(len(session.paths_started), 1)

    def test_workflow_failure_and_stop_terminate_without_advancing_cursor(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        controller.set_plan(Plan(steps=[Waypoint(10, 20, 30), Waypoint(40, 50, 60)]))
        controller.start_continuous(0); session.telemetry.emit(self._telemetry(3))
        self.assertEqual(controller.plan_execution.state, PlanExecutionState.FAILED)
        self.assertEqual(controller.plan_execution.cursor, 0)

        controller.set_plan(Plan(steps=[Waypoint(10, 20, 30)])); controller.start_continuous(0); controller.stop()
        self.assertEqual(controller.plan_execution.state, PlanExecutionState.CANCELED)
        self.assertEqual(session.stopped, 1)

    def test_single_step_can_repeat_the_same_selected_action(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        controller.set_plan(Plan(steps=[Waypoint(10, 20, 30)]))

        self.assertTrue(controller.start_single(0)); session.telemetry.emit(self._telemetry(2))
        self.assertTrue(controller.start_single(0)); session.telemetry.emit(self._telemetry(2))

        self.assertEqual(len(session.started), 2)
        self.assertEqual(session.started[0], session.started[1])

    def test_commands_use_fixed_world_coordinates_without_runtime_transform(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        controller.select_candidate(TargetPose(10, 20, 30))
        controller.start_goal(MotionGoal(10, 20, 30, 100, 50, 1000))

        self.assertEqual(controller.execution, TargetPose(10, 20, 30))
        self.assertEqual(session.started[-1], MotionGoal(10, 20, 30, 100, 50, 1000))

    def test_path_can_start_only_after_commit_confirmation(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        points = [PathPosePoint(0, 0, 0), PathPosePoint(100, 0, 0)]
        controller.upload_path(9, points)
        controller.start_path(9)
        self.assertEqual(session.paths_started, [])
        controller.path_committed(9)
        self.assertEqual(controller.upload.state, PathUploadState.COMMITTED)
        controller.start_path(9)
        self.assertEqual([item.path_id for item in session.paths_started], [9])

    def test_runtime_snapshot_consumes_incremental_trace_and_reset_marker(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        session.telemetry.emit(self._telemetry(1))
        first = controller.consume_runtime_ui_snapshot()
        second = controller.consume_runtime_ui_snapshot()
        self.assertEqual(first.new_trace_points, (TargetPose(0, 0, 0),))
        self.assertTrue(first.trace_reset)
        self.assertEqual(second.new_trace_points, ())
        self.assertFalse(second.trace_reset)

    def test_origin_reset_requires_zero_telemetry_before_sync(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        controller.set_map_calibrated(True)
        self.assertTrue(controller.start_origin_reset())
        controller.confirm_origin_reset()
        session.telemetry.emit(self._telemetry(1))
        self.assertEqual(controller.coordinate_sync_state, CoordinateSyncState.SYNCED)

    @staticmethod
    def _telemetry(state: int) -> Telemetry:
        return Telemetry(1, 0, 0, state, 0x03, (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0))
