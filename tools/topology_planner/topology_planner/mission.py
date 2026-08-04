"""固定比赛任务链的分段拓扑规划。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .planner import PathResult, edge_key, find_best_paths


TASK_DWELL_S = 0.8
"""车辆在每个需要执行动作的任务点停留的时长。"""


@dataclass(frozen=True)
class MissionStop:
    """比赛任务链中的一个停靠点及其动作。"""

    index: int
    node_id: str
    action_id: str
    action_label: str
    dwell_s: float


@dataclass(frozen=True)
class MissionLeg:
    """两个相邻任务点之间的已选拓扑路径。"""

    index: int
    start_stop: MissionStop
    goal_stop: MissionStop
    path: PathResult

    @property
    def start_node_id(self) -> str:
        return self.start_stop.node_id

    @property
    def goal_node_id(self) -> str:
        return self.goal_stop.node_id

    @property
    def distance(self) -> float:
        return self.path.distance

    @property
    def total_cost(self) -> float:
        return self.path.total_cost


@dataclass(frozen=True)
class MissionPlan:
    """可直接交给仿真器消费的完整固定比赛任务链。"""

    start_zone: str
    stops: tuple[MissionStop, ...]
    legs: tuple[MissionLeg, ...]
    flattened_nodes: tuple[str, ...]
    total_distance: float
    total_cost: float
    blocked_edges: frozenset[tuple[str, str]]
    distance_weight: float
    turn_weight: float
    stop_weight: float


class MissionPlanningError(RuntimeError):
    """某一固定任务段在当前禁用边配置下不可达。"""

    def __init__(self, leg_index: int, start_id: str, goal_id: str, message: str) -> None:
        super().__init__(message)
        self.leg_index = leg_index
        self.start_id = start_id
        self.goal_id = goal_id
        self.message = message


def _fixed_stops(start_zone: str) -> tuple[MissionStop, ...]:
    if start_zone not in {"START1", "START2"}:
        raise ValueError("start_zone 必须是 START1 或 START2")

    definitions = (
        (start_zone, "START_MISSION", "待机并开始任务", 0.0),
        ("QR", "READ_QR_TASK", "读取二维码任务码并显示", TASK_DWELL_S),
        ("RAW", "PICK_BATCH_1", "抓取第一批三个物料", TASK_DWELL_S),
        ("ROUGH", "ROUGH_BATCH_1", "放置并按顺序取回第一批物料", TASK_DWELL_S),
        ("BUFFER", "BUFFER_BATCH_1", "放置第一批三个物料", TASK_DWELL_S),
        ("RAW", "PICK_BATCH_2", "抓取第二批三个物料", TASK_DWELL_S),
        ("ROUGH", "ROUGH_BATCH_2", "放置并按顺序取回第二批物料", TASK_DWELL_S),
        ("BUFFER", "BUFFER_BATCH_2", "将第二批物料按同色要求码垛", TASK_DWELL_S),
        (start_zone, "FINISH_MISSION", "返回启停区并结束任务", 0.0),
    )
    return tuple(MissionStop(index, *definition) for index, definition in enumerate(definitions))


def build_mission_plan(
    start_zone: str = "START1",
    blocked_edges: Iterable[tuple[str, str]] = frozenset(),
    distance_weight: float = 1.0,
    turn_weight: float = 0.75,
    stop_weight: float = 1.0,
) -> MissionPlan:
    """规划固定 9 个停靠点、8 个路段的比赛任务链。

    每个路段只取 ``find_best_paths`` 的首选结果，避免各段候选组合改变
    固定任务链的确定性。
    """

    distance_weight = float(distance_weight)
    turn_weight = float(turn_weight)
    stop_weight = float(stop_weight)
    normalized_blocked = frozenset(edge_key(node_a, node_b) for node_a, node_b in blocked_edges)
    stops = _fixed_stops(start_zone)
    legs: list[MissionLeg] = []
    flattened_nodes: list[str] = []

    for leg_index, (start_stop, goal_stop) in enumerate(zip(stops, stops[1:])):
        paths = find_best_paths(
            start_stop.node_id,
            goal_stop.node_id,
            normalized_blocked,
            distance_weight,
            turn_weight,
            stop_weight,
            limit=1,
        )
        if not paths:
            message = (
                f"第 {leg_index + 1} 段任务路径不可达："
                f"{start_stop.node_id} -> {goal_stop.node_id}"
            )
            raise MissionPlanningError(leg_index, start_stop.node_id, goal_stop.node_id, message)

        path = paths[0]
        legs.append(MissionLeg(leg_index, start_stop, goal_stop, path))
        if flattened_nodes and flattened_nodes[-1] == path.nodes[0]:
            flattened_nodes.extend(path.nodes[1:])
        else:
            flattened_nodes.extend(path.nodes)

    return MissionPlan(
        start_zone=start_zone,
        stops=stops,
        legs=tuple(legs),
        flattened_nodes=tuple(flattened_nodes),
        total_distance=sum(leg.path.distance for leg in legs),
        total_cost=sum(leg.path.total_cost for leg in legs),
        blocked_edges=normalized_blocked,
        distance_weight=distance_weight,
        turn_weight=turn_weight,
        stop_weight=stop_weight,
    )


def plan_fixed_mission(
    start_zone: str = "START1",
    blocked_edges: Iterable[tuple[str, str]] = frozenset(),
    distance_weight: float = 1.0,
    turn_weight: float = 0.75,
    stop_weight: float = 1.0,
) -> MissionPlan:
    """``build_mission_plan`` 的语义化别名。"""

    return build_mission_plan(
        start_zone, blocked_edges, distance_weight, turn_weight, stop_weight
    )


def build_fixed_mission_plan(
    start_zone: str = "START1",
    blocked_edges: Iterable[tuple[str, str]] = frozenset(),
    distance_weight: float = 1.0,
    turn_weight: float = 0.75,
    stop_weight: float = 1.0,
) -> MissionPlan:
    """构建固定比赛任务链的兼容入口。"""

    return build_mission_plan(
        start_zone, blocked_edges, distance_weight, turn_weight, stop_weight
    )


__all__ = [
    "TASK_DWELL_S",
    "MissionLeg",
    "MissionPlan",
    "MissionPlanningError",
    "MissionStop",
    "build_fixed_mission_plan",
    "build_mission_plan",
    "plan_fixed_mission",
]
