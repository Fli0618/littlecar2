"""离线轨迹跟踪仿真，采用 STM32 advance_motion 同类 PID、限幅和积分抗饱和规则。"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import random

from .geometry import wrap_deg
from .geometry import world_to_paper
from .models import CAR_SIZE_MM, FIELD_SIZE_MM, Pose, SimulationSettings


@dataclass
class SimulationFrame:
    time_s: float
    reference: Pose
    actual: Pose
    speed_mm_s: float
    error_mm: float
    segment_index: int
    stopped: bool
    out_of_bounds: bool


@dataclass
class Simulation:
    route: list[Pose]
    stop_indices: list[int]
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
    route_index: int = 0
    dwell_remaining_s: float = 0.0
    finished: bool = False
    sensor_history: list[Pose] = field(default_factory=list)
    _random: random.Random = field(default_factory=lambda: random.Random(0))

    def __post_init__(self) -> None:
        if self.route:
            self.actual = Pose(self.route[0].x_mm, self.route[0].y_mm, self.route[0].yaw_deg)

    def step(self) -> SimulationFrame:
        if not self.route:
            raise ValueError("没有可仿真的路径")
        dt = self.settings.dt_s
        if self.finished:
            return self._frame(self.route[-1], True)
        if self.dwell_remaining_s > 0.0:
            self.dwell_remaining_s = max(0.0, self.dwell_remaining_s - dt)
            self.elapsed_s += dt
            return self._frame(self.route[self.route_index], True)
        reference = self.route[min(len(self.route) - 1, self.route_index + max(1, int(self.settings.lookahead_mm / 10.0)))]
        self.sensor_history.append(Pose(self.actual.x_mm, self.actual.y_mm, self.actual.yaw_deg))
        delay_steps = int(self.settings.sensor_delay_s / dt)
        sensed = self.sensor_history[max(0, len(self.sensor_history) - 1 - delay_steps)]
        if self.settings.sensor_noise_mm:
            sensed = Pose(sensed.x_mm + self._random.gauss(0.0, self.settings.sensor_noise_mm), sensed.y_mm + self._random.gauss(0.0, self.settings.sensor_noise_mm), sensed.yaw_deg)
        ex, ey = reference.x_mm - sensed.x_mm, reference.y_mm - sensed.y_mm
        eyaw = wrap_deg(reference.yaw_deg - sensed.yaw_deg)
        raw_vx = self.settings.kp_pos * ex + self.settings.ki_pos * self.ix - self.settings.kd_pos * self.vx
        raw_vy = self.settings.kp_pos * ey + self.settings.ki_pos * self.iy - self.settings.kd_pos * self.vy
        magnitude = math.hypot(raw_vx, raw_vy)
        scale = min(1.0, self.settings.vmax_mm_s / magnitude) if magnitude else 1.0
        command_vx, command_vy = raw_vx * scale, raw_vy * scale
        raw_wz = self.settings.kp_yaw * eyaw + self.settings.ki_yaw * self.iyaw - self.settings.kd_yaw * self.wz
        command_wz = max(-self.settings.wmax_deg_s, min(self.settings.wmax_deg_s, raw_wz))
        if magnitude <= self.settings.vmax_mm_s or ex * command_vx + ey * command_vy <= 0.0:
            self.ix = max(-1000.0, min(1000.0, self.ix + ex * dt)); self.iy = max(-1000.0, min(1000.0, self.iy + ey * dt))
        if abs(raw_wz) <= self.settings.wmax_deg_s or eyaw * command_wz <= 0.0:
            self.iyaw = max(-180.0, min(180.0, self.iyaw + eyaw * dt))
        self.vx += (command_vx - self.vx) * min(1.0, dt / max(0.001, self.settings.linear_response_s))
        self.vy += (command_vy - self.vy) * min(1.0, dt / max(0.001, self.settings.linear_response_s))
        self.wz += (command_wz - self.wz) * min(1.0, dt / max(0.001, self.settings.yaw_response_s))
        self.actual.x_mm += self.vx * dt; self.actual.y_mm += self.vy * dt; self.actual.yaw_deg = wrap_deg(self.actual.yaw_deg + self.wz * dt)
        if math.hypot(self.route[self.route_index].x_mm - self.actual.x_mm, self.route[self.route_index].y_mm - self.actual.y_mm) < 25.0:
            self.route_index += 1
            if self.route_index - 1 in self.stop_indices:
                self.dwell_remaining_s = 0.5
            if self.route_index >= len(self.route):
                self.route_index = len(self.route) - 1; self.finished = True
        self.elapsed_s += dt
        return self._frame(reference, False)

    def _frame(self, reference: Pose, stopped: bool) -> SimulationFrame:
        margin = CAR_SIZE_MM / 2.0
        paper_x, paper_y = world_to_paper(self.actual, self.start_paper_x_mm, self.start_paper_y_mm, self.start_heading_deg)
        out = not (margin <= paper_x <= FIELD_SIZE_MM - margin and margin <= paper_y <= FIELD_SIZE_MM - margin)
        return SimulationFrame(self.elapsed_s, reference, Pose(self.actual.x_mm, self.actual.y_mm, self.actual.yaw_deg), math.hypot(self.vx, self.vy), math.hypot(reference.x_mm - self.actual.x_mm, reference.y_mm - self.actual.y_mm), self.route_index, stopped, out)
