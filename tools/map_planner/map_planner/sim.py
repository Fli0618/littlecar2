"""按 GOTO Pose 命令逐节点执行的离线运动仿真。"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import random

from .geometry import world_to_paper, wrap_deg
from .models import CAR_SIZE_MM, FIELD_SIZE_MM, Pose, SimulationSettings, Waypoint


POSITION_TOLERANCE_MM = 25.0
YAW_TOLERANCE_DEG = 3.0
STOP_SPEED_MM_S = 5.0
STOP_YAW_SPEED_DEG_S = 3.0


@dataclass
class SimulationFrame:
    time_s: float
    reference: Pose
    actual: Pose
    speed_mm_s: float
    error_mm: float
    command_index: int
    stopped: bool
    out_of_bounds: bool
    timed_out: bool


@dataclass
class Simulation:
    commands: list[Waypoint]
    settings: SimulationSettings
    start_paper_x_mm: float = FIELD_SIZE_MM / 2.0
    start_paper_y_mm: float = FIELD_SIZE_MM / 2.0
    start_heading_deg: float = 0.0
    actual: Pose = field(default_factory=Pose)
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    ix: float = 0.0
    iy: float = 0.0
    iyaw: float = 0.0
    elapsed_s: float = 0.0
    command_elapsed_s: float = 0.0
    command_index: int = 0
    dwell_remaining_s: float = 0.0
    settling: bool = False
    finished: bool = False
    failed: bool = False
    failure_reason: str = ""
    sensor_history: list[Pose] = field(default_factory=list)
    _random: random.Random = field(default_factory=lambda: random.Random(0))

    def __post_init__(self) -> None:
        if not self.commands:
            self.finished = True

    def step(self) -> SimulationFrame:
        if not self.commands:
            raise ValueError("没有可仿真的命令")
        reference = self._reference()
        if self.finished or self.failed:
            return self._frame(reference, True, self.failed)

        dt = self.settings.dt_s
        if self.dwell_remaining_s > 0.0:
            self.dwell_remaining_s = max(0.0, self.dwell_remaining_s - dt)
            self.elapsed_s += dt
            if self.dwell_remaining_s == 0.0:
                self._advance_command()
                reference = self._reference()
            return self._frame(reference, True, False)

        command_index = self.command_index
        command = self.commands[command_index]
        sensed = self._sense(dt)
        ex = command.x_mm - sensed.x_mm
        ey = command.y_mm - sensed.y_mm
        eyaw = wrap_deg(command.yaw_deg - sensed.yaw_deg)
        command_vx, command_vy = self._position_control(ex, ey, command.vmax_mm_s, dt)
        command_wz = self._yaw_control(eyaw, command, dt)
        self._apply_velocity_commands(command_vx, command_vy, command_wz, dt)
        self._integrate(dt)
        self.elapsed_s += dt
        self.command_elapsed_s += dt

        position_reached = math.hypot(command.x_mm - self.actual.x_mm, command.y_mm - self.actual.y_mm) <= POSITION_TOLERANCE_MM
        yaw_reached = not command.use_yaw or abs(wrap_deg(command.yaw_deg - self.actual.yaw_deg)) <= YAW_TOLERANCE_DEG
        if position_reached and yaw_reached:
            if command.stop:
                self.settling = True
                if math.hypot(self.vx, self.vy) <= STOP_SPEED_MM_S and abs(self.wz) <= STOP_YAW_SPEED_DEG_S:
                    self.vx = self.vy = self.wz = 0.0
                    self.settling = False
                    if command.dwell_s > 0.0:
                        self.dwell_remaining_s = command.dwell_s
                    else:
                        self._advance_command()
            else:
                self._advance_command()
        else:
            self.settling = False
        if not self.finished and self.dwell_remaining_s == 0.0 and self._has_timed_out(command):
            self._fail_timeout()
        if self.command_index != command_index:
            reference = self._reference()
        return self._frame(reference, self.dwell_remaining_s > 0.0, self.failed)

    def _sense(self, dt: float) -> Pose:
        self.sensor_history.append(Pose(self.actual.x_mm, self.actual.y_mm, self.actual.yaw_deg))
        delay_steps = max(0, int(self.settings.sensor_delay_s / dt))
        sensed = self.sensor_history[max(0, len(self.sensor_history) - 1 - delay_steps)]
        if not self.settings.sensor_noise_mm:
            return sensed
        return Pose(
            sensed.x_mm + self._random.gauss(0.0, self.settings.sensor_noise_mm),
            sensed.y_mm + self._random.gauss(0.0, self.settings.sensor_noise_mm),
            sensed.yaw_deg,
        )

    def _position_control(self, ex: float, ey: float, vmax: float, dt: float) -> tuple[float, float]:
        raw_vx = self.settings.kp_pos * ex + self.settings.ki_pos * self.ix - self.settings.kd_pos * self.vx
        raw_vy = self.settings.kp_pos * ey + self.settings.ki_pos * self.iy - self.settings.kd_pos * self.vy
        magnitude = math.hypot(raw_vx, raw_vy)
        limit = max(0.0, vmax)
        scale = min(1.0, limit / magnitude) if magnitude else 1.0
        command_vx, command_vy = raw_vx * scale, raw_vy * scale
        if magnitude <= limit or ex * command_vx + ey * command_vy <= 0.0:
            self.ix = max(-1000.0, min(1000.0, self.ix + ex * dt))
            self.iy = max(-1000.0, min(1000.0, self.iy + ey * dt))
        return command_vx, command_vy

    def _yaw_control(self, eyaw: float, command: Waypoint, dt: float) -> float:
        if not command.use_yaw:
            self.iyaw = 0.0
            return 0.0
        raw_wz = self.settings.kp_yaw * eyaw + self.settings.ki_yaw * self.iyaw - self.settings.kd_yaw * self.wz
        limit = max(0.0, command.wmax_deg_s)
        command_wz = max(-limit, min(limit, raw_wz))
        if abs(raw_wz) <= limit or eyaw * command_wz <= 0.0:
            self.iyaw = max(-180.0, min(180.0, self.iyaw + eyaw * dt))
        return command_wz

    def _apply_velocity_commands(self, vx: float, vy: float, wz: float, dt: float) -> None:
        linear_alpha = min(1.0, dt / max(0.001, self.settings.linear_response_s))
        yaw_alpha = min(1.0, dt / max(0.001, self.settings.yaw_response_s))
        self.vx += (vx - self.vx) * linear_alpha
        self.vy += (vy - self.vy) * linear_alpha
        self.wz += (wz - self.wz) * yaw_alpha

    def _integrate(self, dt: float) -> None:
        self.actual.x_mm += self.vx * dt
        self.actual.y_mm += self.vy * dt
        self.actual.yaw_deg = wrap_deg(self.actual.yaw_deg + self.wz * dt)

    def _has_timed_out(self, command: Waypoint) -> bool:
        return self.command_elapsed_s >= max(0.0, command.timeout_s)

    def _fail_timeout(self) -> None:
        self.failed = True
        self.failure_reason = f"命令 {self.command_index + 1} 超时"
        self.vx = self.vy = self.wz = 0.0

    def _advance_command(self) -> None:
        self.command_index += 1
        self.command_elapsed_s = 0.0
        self.dwell_remaining_s = 0.0
        self.settling = False
        self.ix = self.iy = self.iyaw = 0.0
        if self.command_index >= len(self.commands):
            self.command_index = len(self.commands) - 1
            self.finished = True

    def _reference(self) -> Pose:
        command = self.commands[self.command_index]
        # 未启用 yaw 时，参考航向就是当前方向，避免显示为“回零”目标。
        yaw = command.yaw_deg if command.use_yaw else self.actual.yaw_deg
        return Pose(command.x_mm, command.y_mm, yaw)

    def _frame(self, reference: Pose, stopped: bool, timed_out: bool) -> SimulationFrame:
        margin = CAR_SIZE_MM / 2.0
        paper_x, paper_y = world_to_paper(
            self.actual,
            self.start_paper_x_mm,
            self.start_paper_y_mm,
            self.start_heading_deg,
        )
        out = not (margin <= paper_x <= FIELD_SIZE_MM - margin and margin <= paper_y <= FIELD_SIZE_MM - margin)
        return SimulationFrame(
            self.elapsed_s,
            reference,
            Pose(self.actual.x_mm, self.actual.y_mm, self.actual.yaw_deg),
            math.hypot(self.vx, self.vy),
            math.hypot(reference.x_mm - self.actual.x_mm, reference.y_mm - self.actual.y_mm),
            self.command_index,
            stopped,
            out,
            timed_out,
        )
