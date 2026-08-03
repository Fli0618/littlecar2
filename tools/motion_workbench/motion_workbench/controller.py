"""State coordinator between one tuner session and all workbench views."""

from __future__ import annotations

import math
import time
from dataclasses import replace

from PySide6.QtCore import QObject, Signal

from pid_tuner.gui.buffer import TelemetryBuffer
from pid_tuner.gui.session import SessionController
from pid_tuner.models import MotionGoal, Telemetry
from map_planner.models import PathPosePoint

from .models import ExperimentResult, PathTelemetry, SinglePointState, TargetPose
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
        self.session.telemetry.connect(self.on_telemetry)
        self.session.motion_changed.connect(self._on_motion_changed)
        self.session.status.connect(self.status_changed)
        self.session.failure.connect(self._on_failure)

    @property
    def state(self) -> SinglePointState:
        return self._last_state

    @property
    def trace(self) -> tuple[TargetPose, ...]:
        return tuple(self._trace)

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
        self.session.start_motion(goal)
        return True

    def stop(self) -> None:
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
        self.session.upload_path(build_path_begin(path_id, points), build_path_chunks(path_id, points),
                                 build_path_commit(path_id))

    def start_path(self, path_id: int) -> None:
        self.session.start_path(build_path_start(path_id))

    def abort_path(self) -> None:
        self.session.abort_path()

    def _on_motion_changed(self, active: bool) -> None:
        if not active and self._last_state == SinglePointState.RUNNING and self.execution is not None:
            # The terminal telemetry normally supplies the precise reason; retain a safe fallback.
            self._finish(SinglePointState.CANCELED, "运动停止")

    def _on_failure(self, message: str) -> None:
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
