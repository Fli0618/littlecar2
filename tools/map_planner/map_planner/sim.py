"""按固定加减速度执行动作序列的离线运动近似仿真。"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from .geometry import world_to_paper, wrap_deg
from .models import CAR_SIZE_MM, FIELD_SIZE_MM, MotionCommand, Pose, RotateInPlace, Waypoint


POSITION_TOLERANCE_MM = 25.0
YAW_TOLERANCE_DEG = 3.0
STOP_SPEED_MM_S = 5.0
STOP_YAW_SPEED_DEG_S = 3.0
DT_S = 0.02
LINEAR_ACCELERATION_MM_S2 = 1000.0
ANGULAR_ACCELERATION_DEG_S2 = 360.0


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
    commands: list[MotionCommand]
    start_paper_x_mm: float = FIELD_SIZE_MM / 2.0
    start_paper_y_mm: float = FIELD_SIZE_MM / 2.0
    start_heading_deg: float = 0.0
    actual: Pose = field(default_factory=Pose)
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    elapsed_s: float = 0.0
    command_elapsed_s: float = 0.0
    command_index: int = 0
    dwell_remaining_s: float = 0.0
    finished: bool = False
    failed: bool = False
    failure_reason: str = ""

    def __post_init__(self) -> None:
        if not self.commands:
            self.finished = True

    def step(self) -> SimulationFrame:
        if not self.commands:
            raise ValueError("没有可仿真的命令")
        if self.finished or self.failed:
            return self._frame(self._reference(), True, self.failed)

        if self.dwell_remaining_s > 0.0:
            self.dwell_remaining_s = max(0.0, self.dwell_remaining_s - DT_S)
            self.elapsed_s += DT_S
            if self.dwell_remaining_s == 0.0:
                self._advance_command()
            return self._frame(self._reference(), True, False)

        command = self.commands[self.command_index]
        if isinstance(command, RotateInPlace):
            completed = self._step_rotation(command.yaw_deg, command.wmax_deg_s)
        else:
            position_done = self._step_position(command)
            yaw_done = self._step_heading(command.yaw_deg, command.wmax_deg_s) if command.use_yaw else True
            completed = position_done and yaw_done

        self.elapsed_s += DT_S
        self.command_elapsed_s += DT_S
        if completed:
            self._complete_command(command)
        elif self._has_timed_out(command):
            self._fail_timeout()
        return self._frame(self._reference(), completed or self.dwell_remaining_s > 0.0, self.failed)

    def _step_position(self, command: Waypoint) -> bool:
        dx = command.x_mm - self.actual.x_mm
        dy = command.y_mm - self.actual.y_mm
        distance = math.hypot(dx, dy)
        speed = math.hypot(self.vx, self.vy)
        if distance <= 1e-6:
            if speed <= STOP_SPEED_MM_S:
                self.vx = self.vy = 0.0
                return True
            next_speed = self._approach(speed, 0.0, LINEAR_ACCELERATION_MM_S2 * DT_S)
            if speed:
                self.vx *= next_speed / speed
                self.vy *= next_speed / speed
            return False

        braking_distance = speed * speed / (2.0 * LINEAR_ACCELERATION_MM_S2)
        target_speed = 0.0 if distance <= braking_distance else max(0.0, command.vmax_mm_s)
        next_speed = self._approach(speed, target_speed, LINEAR_ACCELERATION_MM_S2 * DT_S)
        if distance > 0.0:
            ux, uy = dx / distance, dy / distance
            travel = min(distance, next_speed * DT_S)
            self.actual.x_mm += ux * travel
            self.actual.y_mm += uy * travel
            self.vx, self.vy = ux * next_speed, uy * next_speed
        return False

    def _step_rotation(self, target_yaw_deg: float, max_speed_deg_s: float) -> bool:
        completed = self._step_heading(target_yaw_deg, max_speed_deg_s)
        self.vx = self.vy = 0.0
        return completed

    def _step_heading(self, target_yaw_deg: float, max_speed_deg_s: float) -> bool:
        error = wrap_deg(target_yaw_deg - self.actual.yaw_deg)
        speed = abs(self.wz)
        if abs(error) <= YAW_TOLERANCE_DEG and speed <= STOP_YAW_SPEED_DEG_S:
            self.actual.yaw_deg = target_yaw_deg
            self.wz = 0.0
            return True

        braking_angle = speed * speed / (2.0 * ANGULAR_ACCELERATION_DEG_S2)
        target_speed = 0.0 if abs(error) <= braking_angle else max(0.0, max_speed_deg_s)
        signed_target = math.copysign(target_speed, error) if error else 0.0
        self.wz = self._approach(self.wz, signed_target, ANGULAR_ACCELERATION_DEG_S2 * DT_S)
        turn = self.wz * DT_S
        if abs(turn) >= abs(error):
            self.actual.yaw_deg = target_yaw_deg
            self.wz = 0.0
            return True
        self.actual.yaw_deg = wrap_deg(self.actual.yaw_deg + turn)
        return False

    @staticmethod
    def _approach(current: float, target: float, amount: float) -> float:
        return min(current + amount, target) if current < target else max(current - amount, target)

    def _complete_command(self, command: MotionCommand) -> None:
        self.vx = self.vy = self.wz = 0.0
        if isinstance(command, Waypoint) and command.stop and command.dwell_s > 0.0:
            self.dwell_remaining_s = command.dwell_s
        else:
            self._advance_command()

    def _has_timed_out(self, command: MotionCommand) -> bool:
        return self.command_elapsed_s >= max(0.0, command.timeout_s)

    def _fail_timeout(self) -> None:
        self.failed = True
        self.failure_reason = f"命令 {self.command_index + 1} 超时"
        self.vx = self.vy = self.wz = 0.0

    def _advance_command(self) -> None:
        self.command_index += 1
        self.command_elapsed_s = 0.0
        self.dwell_remaining_s = 0.0
        if self.command_index >= len(self.commands):
            self.command_index = len(self.commands) - 1
            self.finished = True

    def _reference(self) -> Pose:
        command = self.commands[self.command_index]
        if isinstance(command, RotateInPlace):
            return Pose(self.actual.x_mm, self.actual.y_mm, command.yaw_deg)
        return Pose(command.x_mm, command.y_mm, command.yaw_deg if command.use_yaw else self.actual.yaw_deg)

    def _frame(self, reference: Pose, stopped: bool, timed_out: bool) -> SimulationFrame:
        margin = CAR_SIZE_MM / 2.0
        paper_x, paper_y = world_to_paper(self.actual, self.start_paper_x_mm, self.start_paper_y_mm, self.start_heading_deg)
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


def build_timeline(
    commands: list[MotionCommand], start_paper_x_mm: float, start_paper_y_mm: float, start_heading_deg: float
) -> list[SimulationFrame]:
    """返回直到完成或超时的确定性仿真帧。"""
    simulation = Simulation(commands, start_paper_x_mm, start_paper_y_mm, start_heading_deg)
    frames: list[SimulationFrame] = []
    while not simulation.finished and not simulation.failed:
        frames.append(simulation.step())
    return frames
