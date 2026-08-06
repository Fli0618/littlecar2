from __future__ import annotations

import unittest

from PySide6.QtCore import QObject, Signal

from pid_tuner.models import HolonomicTelemetry, MotionGoal, Telemetry
from map_planner.models import (BezierPathSegment, ContinuousPathSegment, PathPosePoint, Plan,
                                RotateInPlace, StepTurnNode, StepTurnPathSegment, Waypoint)

from motion_workbench.controller import MotionWorkbenchController
from motion_workbench.models import (CoordinateSyncState, PathUploadState, PlanExecutionState,
                                     TargetPose)


class FakeSession(QObject):
    telemetry = Signal(object)
    holonomic_telemetry = Signal(object)
    motion_changed = Signal(bool)
    status = Signal(str)
    failure = Signal(str)

    def __init__(self) -> None:
        super().__init__(); self.started: list[MotionGoal] = []; self.stopped = 0
        self.uploaded = []; self.paths_started = []
        self.holonomic_started: list[MotionGoal] = []
        self.connected = True; self.motion_active = False; self.auto_accept_motion = True
        self.telemetry.connect(self._track_terminal_motion)
        self.holonomic_telemetry.connect(self._track_holonomic_terminal_motion)

    def _track_terminal_motion(self, item: Telemetry) -> None:
        if item.state not in (0, 1):
            self.motion_active = False; self.motion_changed.emit(False)

    def _track_holonomic_terminal_motion(self, item: HolonomicTelemetry) -> None:
        if item.state not in (0, 1, 2):
            self.motion_active = False; self.motion_changed.emit(False)

    def _accept_motion(self) -> None:
        self.motion_active = True; self.motion_changed.emit(True)
    def start_motion(self, goal: MotionGoal) -> None:
        self.started.append(goal)
        if self.auto_accept_motion: self._accept_motion()
    def start_holonomic_motion(self, goal: MotionGoal) -> None:
        self.started.append(goal); self.holonomic_started.append(goal)
        if self.auto_accept_motion: self._accept_motion()
    def stop(self) -> None: self.stopped += 1
    def upload_path(self, begin, chunks, commit) -> None: self.uploaded.append((begin, chunks, commit))
    def start_path(self, command) -> None: self.paths_started.append(command)
    def upload_and_start_path(self, begin, chunks, commit, start) -> None:
        self.upload_path(begin, chunks, commit); self.start_path(start)
        if self.auto_accept_motion: self._accept_motion()
    def reset_origin(self) -> None: pass


class ControllerTests(unittest.TestCase):
    def test_off_path_terminal_state_has_specific_failure_reason(self) -> None:
        self.assertEqual(
            MotionWorkbenchController._terminal_reason(7),
            "偏离路径，已安全停车",
        )

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

    def test_classic_telemetry_updates_shared_pose_without_overwriting_holonomic_error(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        controller.select_candidate(TargetPose(10, 20, 30))
        controller.start_goal(MotionGoal(10, 20, 30, 100, 50, 1000), controller="holonomic")
        session.holonomic_telemetry.emit(self._holonomic_telemetry(error=(9.0, 8.0, 7.0)))

        session.telemetry.emit(self._telemetry(1, actual=(40.0, 50.0, 60.0), error=(1.0, 2.0, 3.0)))

        self.assertEqual(controller.actual, TargetPose(40.0, 50.0, 60.0))
        self.assertTrue(controller._pose_valid)
        self.assertEqual(controller._last_error, (9.0, 8.0, 7.0))

    def test_holonomic_to_waypoint_uses_classic_terminal(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        self._finish_holonomic(controller, session)
        controller.set_plan(Plan(steps=[Waypoint(10, 20, 30)]))

        self.assertTrue(controller.start_single(0))
        self.assertEqual(controller._active_controller, "classic")
        session.telemetry.emit(self._telemetry(2))
        self.assertEqual(controller.plan_execution.state, PlanExecutionState.COMPLETED)

    def test_holonomic_to_full_plan_completes_from_classic_terminals(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        self._finish_holonomic(controller, session)
        controller.set_plan(Plan(steps=[Waypoint(10, 20, 30), RotateInPlace(90)]))

        self.assertTrue(controller.start_full_plan())
        self.assertEqual(controller._active_controller, "classic")
        session.telemetry.emit(self._telemetry(2))
        session.telemetry.emit(self._telemetry(2))
        self.assertEqual(controller.plan_execution.state, PlanExecutionState.COMPLETED)

    def test_holonomic_to_path_marks_classic_controller(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        self._finish_holonomic(controller, session)
        points = [PathPosePoint(0, 0, 0), PathPosePoint(100, 0, 0)]
        controller.upload_path(9, points)
        controller.path_committed(9)
        controller.start_path(9)

        self.assertEqual(controller._active_controller, "classic")

    def test_holonomic_to_reset_origin_accepts_classic_zero_telemetry(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        self._finish_holonomic(controller, session)
        controller.set_map_calibrated(True)
        self.assertTrue(controller.start_origin_reset())
        controller.confirm_origin_reset()
        self.assertEqual(controller._active_controller, "classic")
        session.telemetry.emit(self._telemetry(1, actual=(0.0, 0.0, 0.0)))
        self.assertEqual(controller.coordinate_sync_state, CoordinateSyncState.SYNCED)

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

    def test_full_plan_uses_shared_workflow_and_motion_interlock(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        controller.set_plan(Plan(steps=[Waypoint(10, 20, 30),
                                        Waypoint(40, 50, 60)]))

        session.motion_active = True
        self.assertFalse(controller.start_full_plan())
        self.assertEqual(session.started, [])

        session.motion_active = False
        self.assertTrue(controller.start_full_plan())
        self.assertEqual(session.started[0].x_mm, 10)
        self.assertEqual(controller.plan_execution.cursor, 0)

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

    def test_path_to_terminal_rotation_ignores_stale_arrived_until_command_is_accepted(self) -> None:
        session = FakeSession(); session.auto_accept_motion = False
        controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        controller.set_plan(Plan(steps=[
            ContinuousPathSegment([PathPosePoint(0, 0, 0), PathPosePoint(100, 0, 45)]),
            RotateInPlace(90),
        ]))

        self.assertTrue(controller.start_continuous(0))
        session._accept_motion()
        session.telemetry.emit(self._telemetry(2))
        self.assertEqual(len(session.started), 1)
        self.assertEqual(controller.plan_execution.cursor, 1)

        # The old path terminal state can remain in telemetry while the queued
        # GOTO-yaw request is awaiting its ACK.  It must not finish the turn.
        session.telemetry.emit(self._telemetry(2))
        self.assertEqual(controller.plan_execution.cursor, 1)
        self.assertEqual(controller.plan_execution.state, PlanExecutionState.RUNNING)

        session._accept_motion()
        session.telemetry.emit(self._telemetry(1))
        session.telemetry.emit(self._telemetry(2))
        self.assertEqual(controller.plan_execution.state, PlanExecutionState.COMPLETED)

    def test_plan_point_controller_can_use_holonomic_for_waypoints_and_turns(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        controller.set_plan_point_controller("holonomic")
        controller.set_plan(Plan(steps=[Waypoint(10, 20, 30), RotateInPlace(90)]))

        self.assertTrue(controller.start_continuous(0))
        self.assertEqual(len(session.holonomic_started), 1)
        session.holonomic_telemetry.emit(self._holonomic_telemetry(3))
        self.assertEqual(len(session.holonomic_started), 2)
        session.holonomic_telemetry.emit(self._holonomic_telemetry(3))
        self.assertEqual(controller.plan_execution.state, PlanExecutionState.COMPLETED)

    def test_continuous_path_stays_on_path_controller_when_points_use_holonomic(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        controller.set_plan_point_controller("holonomic")
        points = [PathPosePoint(0, 0, 0), PathPosePoint(100, 0, 0)]
        controller.set_plan(Plan(steps=[ContinuousPathSegment(points)]))

        self.assertTrue(controller.start_single(0))
        self.assertEqual(controller._active_controller, "classic")
        self.assertEqual(len(session.holonomic_started), 0)
        self.assertEqual(len(session.paths_started), 1)

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

    def test_existing_board_zero_telemetry_restores_sync_without_another_reset(self) -> None:
        session = FakeSession(); controller = MotionWorkbenchController(session)  # type: ignore[arg-type]
        controller.set_map_calibrated(True)

        session.telemetry.emit(self._telemetry(1, actual=(100.0, 0.0, 0.0)))
        self.assertEqual(controller.coordinate_sync_state,
                         CoordinateSyncState.BOARD_ORIGIN_UNKNOWN)
        self.assertIsNotNone(controller.map_execution_block_reason())

        session.telemetry.emit(self._telemetry(1, actual=(8.0, -6.0, 1.0)))

        self.assertEqual(controller.coordinate_sync_state, CoordinateSyncState.SYNCED)
        self.assertIsNone(controller.map_execution_block_reason())

    @staticmethod
    def _telemetry(state: int, actual=(0.0, 0.0, 0.0), error=(0.0, 0.0, 0.0)) -> Telemetry:
        return Telemetry(1, 0, 0, state, 0x03, (0, 0, 0), actual, error,
                         (0, 0, 0), (0, 0, 0), (0, 0, 0))

    @staticmethod
    def _holonomic_telemetry(state: int = 3, error=(0.0, 0.0, 0.0)) -> HolonomicTelemetry:
        return HolonomicTelemetry(
            1, 0, state, 0x04, 0,
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), error,
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0, 0.0, 0.0)

    @classmethod
    def _finish_holonomic(cls, controller: MotionWorkbenchController, session: FakeSession) -> None:
        controller.select_candidate(TargetPose(10, 20, 30))
        controller.start_goal(MotionGoal(10, 20, 30, 100, 50, 1000), controller="holonomic")
        session.holonomic_telemetry.emit(cls._holonomic_telemetry())
