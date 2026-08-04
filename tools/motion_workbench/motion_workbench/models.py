"""Workbench-only immutable runtime state models."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from pid_tuner.models import PathTelemetry


@dataclass(frozen=True)
class TargetPose:
    x_mm: float
    y_mm: float
    yaw_deg: float


class SinglePointState(str, Enum):
    NO_TARGET = "NO_TARGET"
    TARGET_SELECTED = "TARGET_SELECTED"
    RUNNING = "RUNNING"
    ARRIVED = "ARRIVED"
    TIMEOUT = "TIMEOUT"
    CANCELED = "CANCELED"
    NO_POSE = "NO_POSE"
    NO_ORIGIN = "NO_ORIGIN"


class PlanExecutionState(str, Enum):
    """State exposed to the workflow controls while a map plan is executed."""

    IDLE = "IDLE"
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


@dataclass(frozen=True)
class PlanExecution:
    """A small immutable workflow snapshot suitable for Qt views."""

    state: PlanExecutionState
    cursor: int
    step_count: int
    continuous: bool
    active_step_name: str = ""
    reason: str = ""


@dataclass(frozen=True)
class ExperimentResult:
    target: TargetPose
    final_pose: TargetPose | None
    error_x_mm: float | None
    error_y_mm: float | None
    position_error_mm: float | None
    yaw_error_deg: float | None
    duration_s: float
    reason: str
    pid_revision: int
    yaw_source: str
