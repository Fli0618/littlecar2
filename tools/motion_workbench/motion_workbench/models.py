"""Workbench-only immutable runtime state models."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


@dataclass(frozen=True)
class TargetPose:
    x_mm: float
    y_mm: float
    yaw_deg: float


def reflect_target_pose(pose: TargetPose, flip_x: bool, flip_y: bool) -> TargetPose:
    """Reflect a runtime pose about the world origin, including its heading."""
    if not flip_x and not flip_y:
        return pose
    direction_x = math.sin(math.radians(pose.yaw_deg))
    direction_y = math.cos(math.radians(pose.yaw_deg))
    if flip_x:
        direction_x = -direction_x
    if flip_y:
        direction_y = -direction_y
    yaw_deg = (math.degrees(math.atan2(direction_x, direction_y)) + 180.0) % 360.0 - 180.0
    return TargetPose(-pose.x_mm if flip_x else pose.x_mm,
                      -pose.y_mm if flip_y else pose.y_mm,
                      yaw_deg)


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


@dataclass(frozen=True)
class PathTelemetry:
    path_id: int
    state: int
    nearest_segment_index: int
    target_segment_index: int
    progress_mm: float
    remaining_mm: float
    projection_x_mm: float
    projection_y_mm: float
    lookahead_x_mm: float
    lookahead_y_mm: float
    lookahead_mm: float
    reference_speed_mm_s: float
    curvature_1_mm: float
    yaw_gradient_deg_per_mm: float
    final_stage: bool
