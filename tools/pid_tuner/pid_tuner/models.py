"""Domain models shared by the protocol, serial client and CLI."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any


@dataclass(frozen=True)
class PidConfigSnapshot:
    kp_pos: float
    ki_pos: float
    kd_pos: float
    kp_yaw: float
    ki_yaw: float
    kd_yaw: float

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class PidConfigState:
    """One PID configuration revision reported by the controller."""

    revision: int
    config: PidConfigSnapshot


@dataclass(frozen=True)
class PathConfigSnapshot:
    """Runtime-tunable continuous-path controller and speed-planner values."""

    kp_cross_track: float
    kd_cross_track_velocity: float
    kp_yaw: float
    kd_yaw_rate: float
    cruise_speed_mm_s: float
    max_yaw_rate_deg_s: float
    accel_mm_s2: float
    decel_mm_s2: float
    max_lateral_accel_mm_s2: float
    curvature_preview_mm: float
    curvature_ff_time_s: float
    lookahead_min_mm: float
    lookahead_base_mm: float
    lookahead_speed_gain_s: float
    lookahead_curve_gain_mm: float
    lookahead_max_mm: float
    lookahead_rate_mm_s: float
    initial_lookahead_mm: float
    final_capture_distance_mm: float
    final_capture_speed_mm_s: float

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class PathConfigState:
    """One path-controller configuration revision reported by the controller."""

    revision: int
    config: PathConfigSnapshot


@dataclass(frozen=True)
class GotoControlConfigSnapshot:
    """Runtime-tunable single-pose GOTO controller values."""

    profile_threshold_mm: float
    cruise_speed_mm_s: float
    accel_mm_s2: float
    decel_mm_s2: float
    capture_distance_mm: float
    capture_speed_mm_s: float
    final_max_speed_mm_s: float
    cross_track_kp: float
    cross_track_kd: float
    cross_track_correction_max_mm_s: float
    yaw_cruise_rate_deg_s: float
    yaw_accel_deg_s2: float
    yaw_decel_deg_s2: float
    yaw_capture_equivalent_mm: float
    yaw_capture_rate_deg_s: float
    yaw_final_max_rate_deg_s: float
    yaw_correction_kp: float
    yaw_correction_kd: float
    yaw_correction_max_deg_s: float
    correction_open_loop_ms: int
    correction_blend_ms: int

    def to_dict(self) -> dict[str, float | int]:
        return asdict(self)


@dataclass(frozen=True)
class GotoControlConfigState:
    """One single-pose GOTO configuration revision reported by the controller."""

    revision: int
    config: GotoControlConfigSnapshot


@dataclass(frozen=True)
class MotionGoal:
    x_mm: float
    y_mm: float
    yaw_deg: float
    vmax_mm_s: float
    wmax_deg_s: float
    timeout_ms: int
    use_yaw: bool = True
    use_position: bool = True


@dataclass(frozen=True)
class Telemetry:
    tick: int
    pid_revision: int
    overwritten_count: int
    state: int
    flags: int
    target: tuple[float, float, float]
    actual: tuple[float, float, float]
    error: tuple[float, float, float]
    command_velocity: tuple[float, float, float]
    measured_velocity: tuple[float, float, float]
    integrals: tuple[float, float, float]
    remote_link_status: int = 0
    wit_yaw_deg: float = 0.0
    ops_yaw_deg: float = 0.0

    @property
    def remote_goal_active(self) -> bool:
        return bool(self.remote_link_status & 0x8000)

    @property
    def heartbeat_timed_out(self) -> bool:
        return bool(self.remote_link_status & 0x4000)

    @property
    def heartbeat_age_ms(self) -> int:
        return self.remote_link_status & 0x3FFF

    @property
    def yaw_source(self) -> str:
        return "OPS" if (self.flags & 0x80) else "WIT"

    @property
    def yaw_aligning(self) -> bool:
        return bool(self.flags & 0x20)

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class GotoStrategySnapshot:
    large_yaw_align_enabled: bool


@dataclass(frozen=True)
class AckResponse:
    command: int
    sequence: int
    revision: int | None = None


@dataclass(frozen=True)
class PathStatus:
    motion_state: int
    active_present: bool
    staging_state: int
    active_count: int
    staging_count: int
    received_count: int
    active_id: int
    staging_id: int


@dataclass(frozen=True)
class PathPointSnapshot:
    x_mm: float
    y_mm: float
    yaw_deg: float


@dataclass(frozen=True)
class PathBeginCommand:
    path_id: int
    point_count: int
    crc16: int


@dataclass(frozen=True)
class PathChunkCommand:
    path_id: int
    first_index: int
    points: tuple[PathPointSnapshot, ...]


@dataclass(frozen=True)
class PathCommitCommand:
    path_id: int


@dataclass(frozen=True)
class PathStartCommand:
    path_id: int


@dataclass(frozen=True)
class PathTelemetry:
    tick: int
    path_id: int
    path_config_revision: int
    state: int
    nearest_segment_index: int
    target_segment_index: int
    final_stage: int
    progress_mm: float
    remaining_mm: float
    projection_x_mm: float
    projection_y_mm: float
    lookahead_x_mm: float
    lookahead_y_mm: float
    signed_curvature_1_mm: float
    curvature_preview_1_mm: float
    yaw_gradient_deg_per_mm: float
    reference_speed_mm_s: float
    lookahead_mm: float
    feedforward_vx_mm_s: float
    feedforward_vy_mm_s: float
    feedforward_wz_deg_s: float
    cross_track_mm: float
    measured_normal_velocity_mm_s: float
    normal_velocity_ff_mm_s: float
    normal_feedback_mm_s: float
    command_wz_deg_s: float

    @property
    def normal_command_mm_s(self) -> float:
        """Return the final normal command while preserving the V3 frame layout."""
        return self.normal_feedback_mm_s - self.normal_velocity_ff_mm_s

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


# 兼容已有工具调用方；新代码应使用带 Snapshot 后缀的类型名。
PidConfig = PidConfigSnapshot
PathControlConfig = PathConfigSnapshot
GotoControlConfig = GotoControlConfigSnapshot


class TunerError(RuntimeError):
    """Base exception for the PC-side tuner."""


class RequestTimeout(TunerError):
    """The board did not produce a matching response after all retries."""


class BoardError(TunerError):
    """The board explicitly rejected one command."""

    def __init__(self, command: int, code: int | None) -> None:
        self.command = command
        self.code = code
        super().__init__(f"board rejected command 0x{command:02X}, error={code}")
