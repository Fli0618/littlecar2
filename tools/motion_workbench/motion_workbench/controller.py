"""State coordinator between one tuner session and all workbench views."""

from __future__ import annotations

import math
import time
from dataclasses import replace

from PySide6.QtCore import QObject, Signal

from pid_tuner.gui.buffer import TelemetryBuffer
from pid_tuner.gui.session import SessionController
from pid_tuner.models import MotionGoal, Telemetry
from map_planner.bezier import generate_bezier_path_points
from map_planner.models import (BezierPathSegment, ContinuousPathSegment, PathPosePoint, Plan, Pose, RotateInPlace,
                                Waypoint)

from .models import (ExperimentResult, PathTelemetry, PlanExecution, PlanExecutionState, SinglePointState, TargetPose,
                     inverse_transform_target_pose)
from .path_transfer import build_path_begin, build_path_chunks, build_path_commit, build_path_start


class MotionWorkbenchController(QObject):
    """Owns workbench state; widgets only render it or request explicit actions."""

    candidate_changed = Signal(object)
    execution_changed = Signal(object)
    actual_pose_changed = Signal(object, bool)
    trace_changed = Signal(object)
    motion_state_changed = Signal(str)
    experiment_finished = Signal(object)
    status_changed = Signal(str)
    path_telemetry_changed = Signal(object)
    upload_changed = Signal(str)
    plan_changed = Signal(object)
    plan_execution_changed = Signal(object)
    plan_finished = Signal(object)

    def __init__(self, session: SessionController | None = None) -> None:
        super().__init__()
        self.session = session or SessionController()
        self.buffer = TelemetryBuffer()
        self.candidate: TargetPose | None = None
        self.execution: TargetPose | None = None
        self.actual: TargetPose | None = None
        self._trace: list[TargetPose] = []
        self._started_at: float | None = None
        self._pid_revision = 0
        self._last_state = SinglePointState.NO_TARGET
        self._last_path: PathTelemetry | None = None
        self._plan: Plan | None = None
        self._plan_cursor = 0
        self._selected_step_index = -1
        self._plan_pose = Pose()
        self._plan_state = PlanExecutionState.IDLE
        self._plan_continuous = False
        self._plan_waiting = False
        self._plan_active_step_name = ""
        self._plan_reason = ""
        self._plan_path_id = 100
        self._command_swap_xy = False
        self._command_flip_x = False
        self._command_flip_y = False
        self.session.telemetry.connect(self.on_telemetry)
        self.session.motion_changed.connect(self._on_motion_changed)
        self.session.status.connect(self.status_changed)
        self.session.failure.connect(self._on_failure)
        if hasattr(self.session, "path_telemetry"):
            self.session.path_telemetry.connect(self.on_path_telemetry)

    @property
    def state(self) -> SinglePointState:
        return self._last_state

    @property
    def trace(self) -> tuple[TargetPose, ...]:
        return tuple(self._trace)

    @property
    def plan(self) -> Plan | None:
        return self._plan

    @property
    def plan_execution(self) -> PlanExecution:
        return self._plan_snapshot()

    def set_plan(self, plan: Plan | None) -> None:
        """Select a workflow and reset its execution cursor to the world origin."""
        if self._plan_state == PlanExecutionState.RUNNING:
            raise RuntimeError("流程正在执行，不能替换方案")
        selected = self._selected_step_index
        self._plan = plan
        self._plan_cursor = 0
        self._selected_step_index = (selected if plan is not None and
                                     0 <= selected < len(plan.steps) else -1)
        self._plan_pose = Pose()
        self._plan_state = PlanExecutionState.IDLE
        self._plan_continuous = False
        self._plan_waiting = False
        self._plan_active_step_name = ""
        self._plan_reason = ""
        self.plan_changed.emit(plan)
        self.plan_execution_changed.emit(self._plan_snapshot())

    def set_plan_cursor(self, index: int) -> None:
        """Select the next plan step while the workflow is idle."""
        if self._plan is None or self._plan_state == PlanExecutionState.RUNNING:
            return
        if 0 <= index < len(self._plan.steps):
            self._selected_step_index = index
            self.plan_execution_changed.emit(self._plan_snapshot())

    def start_single(self, step_index: int) -> bool:
        """Execute the explicitly selected step without changing the selection."""
        return self._start_plan(continuous=False, start_index=step_index)

    def start_continuous(self, step_index: int) -> bool:
        """Execute from the explicitly selected step through the remaining workflow."""
        return self._start_plan(continuous=True, start_index=step_index)

    def select_candidate(self, pose: TargetPose) -> None:
        self.candidate = pose
        self._last_state = SinglePointState.TARGET_SELECTED
        self.candidate_changed.emit(pose)
        self.motion_state_changed.emit(self._last_state.value)

    def clear_candidate(self) -> None:
        self.candidate = None
        self._last_state = SinglePointState.NO_TARGET if self.execution is None else self._last_state
        self.candidate_changed.emit(None)
        self.motion_state_changed.emit(self._last_state.value)

    def start_goal(self, goal: MotionGoal) -> bool:
        if self.candidate is None:
            self.status_changed.emit("请先选择目标位姿")
            return False
        self.execution = replace(self.candidate)
        self.execution_changed.emit(self.execution)
        self._trace.clear()
        self.trace_changed.emit(self.trace)
        self._started_at = time.monotonic()
        self._last_state = SinglePointState.RUNNING
        self.motion_state_changed.emit(self._last_state.value)
        self.session.start_motion(self._transform_goal_for_command(goal))
        return True

    def set_command_axis_transform(
        self, swap_xy: bool, flip_x: bool, flip_y: bool
    ) -> None:
        """Use the inverse display transform for every board-bound pose."""
        self._command_swap_xy = bool(swap_xy)
        self._command_flip_x = bool(flip_x)
        self._command_flip_y = bool(flip_y)

    def _transform_goal_for_command(self, goal: MotionGoal) -> MotionGoal:
        pose = inverse_transform_target_pose(
            TargetPose(goal.x_mm, goal.y_mm, goal.yaw_deg),
            self._command_swap_xy, self._command_flip_x, self._command_flip_y)
        return replace(goal, x_mm=pose.x_mm, y_mm=pose.y_mm, yaw_deg=pose.yaw_deg)

    def _transform_path_for_command(
        self, points: list[PathPosePoint]
    ) -> list[PathPosePoint]:
        transformed: list[PathPosePoint] = []
        for point in points:
            pose = inverse_transform_target_pose(
                TargetPose(point.x_mm, point.y_mm, point.yaw_deg),
                self._command_swap_xy, self._command_flip_x, self._command_flip_y)
            transformed.append(PathPosePoint(pose.x_mm, pose.y_mm, pose.yaw_deg))
        return transformed

    def stop(self) -> None:
        if self._plan_state == PlanExecutionState.RUNNING:
            self._plan_waiting = False
            self._finish_plan(PlanExecutionState.CANCELED, "STOP")
        self.session.stop()
        self._finish(SinglePointState.CANCELED, "STOP")

    def new_experiment(self) -> None:
        self.execution = None
        self._trace.clear()
        self._started_at = None
        self.buffer.clear()
        self.execution_changed.emit(None)
        self.trace_changed.emit(self.trace)

    def on_telemetry(self, item: Telemetry) -> None:
        self.buffer.append(item)
        self._pid_revision = item.pid_revision
        pose = TargetPose(*item.actual)
        valid = bool(item.flags & 0x01) and bool(item.flags & 0x02)
        self.actual = pose if valid else None
        self.actual_pose_changed.emit(pose, valid)
        if valid and self._should_append_trace(pose):
            self._trace.append(pose)
            if len(self._trace) > 2000:
                del self._trace[:len(self._trace) - 2000]
            self.trace_changed.emit(self.trace)
        if self._plan_state == PlanExecutionState.RUNNING and self._plan_waiting and item.state not in (0, 1):
            self._handle_plan_terminal(item.state)
        if self.execution is not None and self._last_state == SinglePointState.RUNNING and item.state not in (0, 1):
            states = {2: (SinglePointState.ARRIVED, "到达"), 3: (SinglePointState.TIMEOUT, "超时"),
                      4: (SinglePointState.NO_POSE, "位姿无效"), 5: (SinglePointState.NO_ORIGIN, "原点无效"),
                      6: (SinglePointState.CANCELED, "取消")}
            state, reason = states.get(item.state, (SinglePointState.CANCELED, "结束"))
            self._finish(state, reason)

    def on_path_telemetry(self, item: PathTelemetry) -> None:
        self._last_path = item
        self.path_telemetry_changed.emit(item)

    def upload_path(self, path_id: int, points: list[PathPosePoint]) -> None:
        command_points = self._transform_path_for_command(points)
        self.session.upload_path(build_path_begin(path_id, command_points), build_path_chunks(path_id, command_points),
                                 build_path_commit(path_id))

    def start_path(self, path_id: int) -> None:
        self.session.start_path(build_path_start(path_id))

    def abort_path(self) -> None:
        self.session.abort_path()

    def _on_motion_changed(self, active: bool) -> None:
        if self._plan_state == PlanExecutionState.RUNNING:
            # Workflow advancement is deliberately driven by terminal telemetry,
            # not by completion of an asynchronous serial request.
            return
        if not active and self._last_state == SinglePointState.RUNNING and self.execution is not None:
            # The terminal telemetry normally supplies the precise reason; retain a safe fallback.
            self._finish(SinglePointState.CANCELED, "运动停止")

    def _on_failure(self, message: str) -> None:
        if self._plan_state == PlanExecutionState.RUNNING:
            self._plan_waiting = False
            self.session.stop()
            self._finish_plan(PlanExecutionState.FAILED, "通信失败")
        if self._last_state == SinglePointState.RUNNING:
            self.session.stop()
            self._finish(SinglePointState.CANCELED, "通信失败")
        self.status_changed.emit(f"错误: {message}")

    def _finish(self, state: SinglePointState, reason: str) -> None:
        if self.execution is None or self._last_state != SinglePointState.RUNNING:
            return
        actual = self.actual
        duration = 0.0 if self._started_at is None else time.monotonic() - self._started_at
        error_x = error_y = position = yaw = None
        if actual is not None:
            error_x = self.execution.x_mm - actual.x_mm
            error_y = self.execution.y_mm - actual.y_mm
            position = math.hypot(error_x, error_y)
            yaw = ((self.execution.yaw_deg - actual.yaw_deg + 180.0) % 360.0) - 180.0
        self._last_state = state
        self.motion_state_changed.emit(state.value)
        self.experiment_finished.emit(ExperimentResult(self.execution, actual, error_x, error_y, position, yaw,
                                                       duration, reason, self._pid_revision,
                                                       "OPS" if self.actual else "未知"))

    def _should_append_trace(self, pose: TargetPose) -> bool:
        if not self._trace:
            return True
        last = self._trace[-1]
        return math.hypot(pose.x_mm - last.x_mm, pose.y_mm - last.y_mm) >= 5.0 or \
            abs(((pose.yaw_deg - last.yaw_deg + 180.0) % 360.0) - 180.0) >= 2.0

    def _start_plan(self, continuous: bool, start_index: int) -> bool:
        if self._plan is None:
            self.status_changed.emit("请先选择流程方案")
            return False
        if self._plan_state == PlanExecutionState.RUNNING:
            self.status_changed.emit("流程正在执行")
            return False
        if not 0 <= start_index < len(self._plan.steps):
            self.status_changed.emit("请选择有效的流程动作")
            return False
        self._selected_step_index = start_index
        self._plan_cursor = start_index
        self._plan_pose = self._pose_before_step(start_index)
        self._plan_state = PlanExecutionState.RUNNING
        self._plan_continuous = continuous
        self._plan_reason = ""
        self._send_current_plan_step()
        return self._plan_state == PlanExecutionState.RUNNING

    def _pose_before_step(self, index: int) -> Pose:
        pose = Pose()
        if self._plan is None:
            return pose
        for step in self._plan.steps[:index]:
            if isinstance(step, Waypoint):
                pose = Pose(step.x_mm, step.y_mm,
                            step.yaw_deg if step.use_yaw else pose.yaw_deg)
            elif isinstance(step, RotateInPlace):
                pose = Pose(pose.x_mm, pose.y_mm, step.yaw_deg)
            elif isinstance(step, ContinuousPathSegment) and step.points:
                point = step.points[-1]
                pose = Pose(point.x_mm, point.y_mm, point.yaw_deg)
            elif isinstance(step, BezierPathSegment):
                pose = Pose(step.end_x_mm, step.end_y_mm, step.end_yaw_deg)
        return pose

    def _send_current_plan_step(self) -> None:
        if self._plan is None:
            return
        step = self._plan.steps[self._plan_cursor]
        self._plan_waiting = True
        self._plan_active_step_name = getattr(step, "name", "") or type(step).__name__
        self._emit_plan_execution()
        try:
            if isinstance(step, Waypoint):
                self.session.start_motion(self._transform_goal_for_command(MotionGoal(
                    step.x_mm, step.y_mm, step.yaw_deg, step.vmax_mm_s, step.wmax_deg_s,
                    round(step.timeout_s * 1000), step.use_yaw, True,
                )))
            elif isinstance(step, RotateInPlace):
                self.session.start_motion(self._transform_goal_for_command(MotionGoal(
                    self._plan_pose.x_mm, self._plan_pose.y_mm, step.yaw_deg, 0.0, step.wmax_deg_s,
                    round(step.timeout_s * 1000), True, False,
                )))
            elif isinstance(step, ContinuousPathSegment):
                self._send_plan_path(step.points)
            elif isinstance(step, BezierPathSegment):
                points = generate_bezier_path_points(
                    self._plan_pose,
                    (step.control_1_x_mm, step.control_1_y_mm),
                    (step.control_2_x_mm, step.control_2_y_mm),
                    Pose(step.end_x_mm, step.end_y_mm, step.end_yaw_deg),
                    step.yaw_mode,
                    step.sample_spacing_mm,
                )
                self._send_plan_path(points)
            else:
                raise ValueError("流程包含不支持的步骤类型")
        except (TypeError, ValueError) as error:
            self._plan_waiting = False
            self._finish_plan(PlanExecutionState.FAILED, str(error))

    def _send_plan_path(self, points: list[PathPosePoint]) -> None:
        path_id = self._plan_path_id
        self._plan_path_id += 1
        command_points = self._transform_path_for_command(points)
        self.session.upload_path(build_path_begin(path_id, command_points), build_path_chunks(path_id, command_points),
                                 build_path_commit(path_id))
        self.session.start_path(build_path_start(path_id))

    def _handle_plan_terminal(self, telemetry_state: int) -> None:
        self._plan_waiting = False
        if telemetry_state != 2:
            self._finish_plan(PlanExecutionState.FAILED, self._terminal_reason(telemetry_state))
            return
        self._advance_plan_cursor()
        if self._plan_continuous and self._plan is not None and self._plan_cursor < len(self._plan.steps):
            self._send_current_plan_step()
            return
        self._plan_state = PlanExecutionState.COMPLETED if self._plan is not None and self._plan_cursor >= len(self._plan.steps) else PlanExecutionState.IDLE
        self._plan_active_step_name = ""
        self._emit_plan_execution("步骤完成" if self._plan_state == PlanExecutionState.IDLE else "流程完成")
        if self._plan_state == PlanExecutionState.COMPLETED:
            self.plan_finished.emit(self._plan_snapshot())

    def _advance_plan_cursor(self) -> None:
        if self._plan is None:
            return
        step = self._plan.steps[self._plan_cursor]
        if isinstance(step, Waypoint):
            self._plan_pose = Pose(step.x_mm, step.y_mm, step.yaw_deg if step.use_yaw else self._plan_pose.yaw_deg)
        elif isinstance(step, RotateInPlace):
            self._plan_pose = Pose(self._plan_pose.x_mm, self._plan_pose.y_mm, step.yaw_deg)
        elif isinstance(step, ContinuousPathSegment) and step.points:
            point = step.points[-1]
            self._plan_pose = Pose(point.x_mm, point.y_mm, point.yaw_deg)
        elif isinstance(step, BezierPathSegment):
            self._plan_pose = Pose(step.end_x_mm, step.end_y_mm, step.end_yaw_deg)
        self._plan_cursor += 1

    @staticmethod
    def _terminal_reason(state: int) -> str:
        return {3: "超时", 4: "位姿无效", 5: "原点无效", 6: "已取消"}.get(state, "运动失败")

    def _finish_plan(self, state: PlanExecutionState, reason: str) -> None:
        if self._plan_state != PlanExecutionState.RUNNING:
            return
        self._plan_state = state
        self._plan_active_step_name = ""
        self._plan_reason = reason
        self._emit_plan_execution(reason)
        self.plan_finished.emit(self._plan_snapshot())

    def _plan_snapshot(self) -> PlanExecution:
        return PlanExecution(self._plan_state, self._plan_cursor, 0 if self._plan is None else len(self._plan.steps),
                             self._plan_continuous, self._plan_active_step_name, self._plan_reason)

    def _emit_plan_execution(self, reason: str = "") -> None:
        if reason:
            self._plan_reason = reason
        snapshot = self._plan_snapshot()
        self.plan_execution_changed.emit(snapshot)
        if reason:
            self.status_changed.emit(reason)
