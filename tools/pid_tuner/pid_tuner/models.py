"""Domain models shared by the protocol, serial client and CLI."""

from __future__ import annotations

from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class PidConfig:
    kp_pos: float
    ki_pos: float
    kd_pos: float
    kp_yaw: float
    ki_yaw: float
    kd_yaw: float

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class MotionGoal:
    x_mm: float
    y_mm: float
    yaw_deg: float
    vmax_mm_s: float
    wmax_deg_s: float
    timeout_ms: int
    use_yaw: bool = True


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

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


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
