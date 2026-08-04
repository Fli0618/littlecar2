"""固定任务链的纯 Python 时间推进仿真器。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite

from .mission import MissionPlan, MissionStop
from .planner import edge_length


class SimulationPhase(str, Enum):
    IDLE = "IDLE"
    TRAVEL = "TRAVEL"
    DWELL = "DWELL"
    PAUSED = "PAUSED"
    FINISHED = "FINISHED"


@dataclass(frozen=True)
class TraversalEdge:
    """MissionPlan 展开后的有序拓扑边。"""

    leg_index: int
    from_node: str
    to_node: str
    length: float


@dataclass(frozen=True)
class SimulationSnapshot:
    phase: SimulationPhase
    running: bool
    paused: bool
    finished: bool
    leg_index: int
    leg_count: int
    from_node: str | None
    to_node: str | None
    edge_progress: float
    traveled_distance: float
    total_distance: float
    total_progress: float
    current_stop_index: int
    current_node: str | None
    current_action_label: str
    dwell_remaining_s: float


class MissionSimulator:
    """以调用方提供的时间步长推进 ``MissionPlan``。

    仿真器不依赖 GUI 定时器；调用方可用任意刷新周期调用 :meth:`tick`。
    """

    def __init__(self, plan: MissionPlan, base_speed: float = 1.0) -> None:
        if not isfinite(float(base_speed)) or float(base_speed) <= 0.0:
            raise ValueError("base_speed 必须大于 0")
        if not plan.legs or plan.total_distance <= 0.0:
            raise ValueError("MissionPlan 必须包含可行驶的任务链")
        self.plan = plan
        self.base_speed = float(base_speed)
        self._traversal_edges = tuple(
            TraversalEdge(leg.index, first, second, edge_length(first, second))
            for leg in plan.legs
            for first, second in zip(leg.path.nodes, leg.path.nodes[1:])
        )
        self._speed_multiplier = 1.0
        self.reset()

    @property
    def traversal_edges(self) -> tuple[TraversalEdge, ...]:
        """返回按任务顺序展开的只读拓扑边。"""

        return self._traversal_edges

    def start(self) -> SimulationSnapshot:
        """开始播放；已结束的仿真会从任务链起点重新播放。"""

        if self._phase is SimulationPhase.FINISHED:
            self.reset()
        if self._phase is SimulationPhase.PAUSED:
            return self.resume()
        if self._phase is SimulationPhase.IDLE:
            self._phase = SimulationPhase.TRAVEL if self.plan.legs else SimulationPhase.FINISHED
        return self.snapshot()

    def pause(self) -> SimulationSnapshot:
        if self._phase in {SimulationPhase.TRAVEL, SimulationPhase.DWELL}:
            self._paused_phase = self._phase
            self._phase = SimulationPhase.PAUSED
        return self.snapshot()

    def resume(self) -> SimulationSnapshot:
        if self._phase is SimulationPhase.PAUSED:
            self._phase = self._paused_phase
        return self.snapshot()

    def stop(self) -> SimulationSnapshot:
        """停止并回到初始空闲状态。"""

        return self.reset()

    def reset(self) -> SimulationSnapshot:
        self._phase = SimulationPhase.IDLE
        self._paused_phase = SimulationPhase.IDLE
        self._leg_index = 0
        self._segment_index = 0
        self._segment_distance = 0.0
        self._traveled_distance = 0.0
        self._current_stop_index = 0
        self._current_node: str | None = self.plan.stops[0].node_id if self.plan.stops else None
        self._dwell_remaining_s = 0.0
        return self.snapshot()

    def set_speed_multiplier(self, multiplier: float) -> SimulationSnapshot:
        if not isfinite(float(multiplier)) or float(multiplier) <= 0.0:
            raise ValueError("multiplier 必须大于 0")
        self._speed_multiplier = float(multiplier)
        return self.snapshot()

    def tick(self, dt_s: float) -> SimulationSnapshot:
        """推进仿真，支持单个大时间步跨越多条边和任务停留。"""

        if not isfinite(float(dt_s)) or float(dt_s) < 0.0:
            raise ValueError("dt_s 不能为负数")
        dt_s = float(dt_s)
        if self._phase not in {SimulationPhase.TRAVEL, SimulationPhase.DWELL}:
            return self.snapshot()

        remaining_s = dt_s
        while remaining_s > 0.0 and self._phase in {SimulationPhase.TRAVEL, SimulationPhase.DWELL}:
            if self._phase is SimulationPhase.DWELL:
                used_s = min(remaining_s, self._dwell_remaining_s)
                self._dwell_remaining_s -= used_s
                remaining_s -= used_s
                if self._dwell_remaining_s <= 0.0:
                    self._dwell_remaining_s = 0.0
                    self._begin_next_leg_or_finish()
                continue

            speed = self.base_speed * self._speed_multiplier
            segment_length = self._active_segment_length()
            remaining_distance = segment_length - self._segment_distance
            used_s = min(remaining_s, remaining_distance / speed)
            moved_distance = used_s * speed
            self._segment_distance += moved_distance
            self._traveled_distance += moved_distance
            remaining_s -= used_s
            if self._segment_distance >= segment_length:
                self._traveled_distance = min(self._traveled_distance, self.plan.total_distance)
                self._complete_segment()

        return self.snapshot()

    def snapshot(self) -> SimulationSnapshot:
        from_node, to_node, edge_progress = self._edge_state()
        total_progress = (
            min(1.0, self._traveled_distance / self.plan.total_distance)
            if self.plan.total_distance > 0.0
            else float(self._phase is SimulationPhase.FINISHED)
        )
        stop = self._current_stop()
        effective_phase = self._paused_phase if self._phase is SimulationPhase.PAUSED else self._phase
        if effective_phase is SimulationPhase.TRAVEL and self._leg_index < len(self.plan.legs):
            stop = self.plan.legs[self._leg_index].goal_stop
        return SimulationSnapshot(
            phase=self._phase,
            running=self._phase in {SimulationPhase.TRAVEL, SimulationPhase.DWELL},
            paused=self._phase is SimulationPhase.PAUSED,
            finished=self._phase is SimulationPhase.FINISHED,
            leg_index=min(self._leg_index, len(self.plan.legs) - 1),
            leg_count=len(self.plan.legs),
            from_node=from_node,
            to_node=to_node,
            edge_progress=min(max(edge_progress, 0.0), 1.0),
            traveled_distance=self._traveled_distance,
            total_distance=self.plan.total_distance,
            total_progress=min(max(total_progress, 0.0), 1.0),
            current_stop_index=self._current_stop_index,
            current_node=self._current_node,
            current_action_label=stop.action_label if stop else "",
            dwell_remaining_s=self._dwell_remaining_s,
        )

    def _current_stop(self) -> MissionStop | None:
        if 0 <= self._current_stop_index < len(self.plan.stops):
            return self.plan.stops[self._current_stop_index]
        return None

    def _active_nodes(self) -> tuple[str, ...]:
        return self.plan.legs[self._leg_index].path.nodes

    def _active_segment_length(self) -> float:
        nodes = self._active_nodes()
        return edge_length(nodes[self._segment_index], nodes[self._segment_index + 1])

    def _complete_segment(self) -> None:
        nodes = self._active_nodes()
        self._segment_distance = 0.0
        if self._segment_index + 1 < len(nodes) - 1:
            self._segment_index += 1
            self._current_node = nodes[self._segment_index]
            return

        self._current_stop_index += 1
        self._current_node = self.plan.stops[self._current_stop_index].node_id
        self._leg_index += 1
        self._segment_index = 0
        self._dwell_remaining_s = self.plan.stops[self._current_stop_index].dwell_s
        if self._dwell_remaining_s > 0.0:
            self._phase = SimulationPhase.DWELL
        else:
            self._begin_next_leg_or_finish()

    def _begin_next_leg_or_finish(self) -> None:
        if self._leg_index >= len(self.plan.legs):
            self._phase = SimulationPhase.FINISHED
            self._current_node = self.plan.stops[-1].node_id if self.plan.stops else None
            return
        self._phase = SimulationPhase.TRAVEL
        self._segment_index = 0
        self._segment_distance = 0.0

    def _edge_state(self) -> tuple[str | None, str | None, float]:
        if self._phase is SimulationPhase.PAUSED:
            phase = self._paused_phase
        else:
            phase = self._phase
        if phase is SimulationPhase.TRAVEL and self._leg_index < len(self.plan.legs):
            nodes = self._active_nodes()
            from_node = nodes[self._segment_index]
            to_node = nodes[self._segment_index + 1]
            length = edge_length(from_node, to_node)
            return from_node, to_node, self._segment_distance / length
        return self._current_node, self._current_node, 1.0 if self._current_node else 0.0


__all__ = ["MissionSimulator", "SimulationPhase", "SimulationSnapshot", "TraversalEdge"]
