"""固定 3x3 比赛场地拓扑和无依赖路径搜索算法。"""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Iterable


@dataclass(frozen=True)
class Node:
    id: str
    label: str
    kind: str
    x: float
    y: float


@dataclass(frozen=True)
class Edge:
    node_a: str
    node_b: str
    length: float


@dataclass(frozen=True)
class PathResult:
    nodes: tuple[str, ...]
    distance: float
    quarter_turns: int
    stops: int
    distance_cost: float
    turn_cost: float
    stop_cost: float
    total_cost: float


def edge_key(node_a: str, node_b: str) -> tuple[str, str]:
    """返回无向边的稳定键。"""

    return tuple(sorted((node_a, node_b)))


_NAVIGATION = {
    "NW": ("左上", 0.0, 0.0),
    "N": ("上", 1.0, 0.0),
    "NE": ("右上", 2.0, 0.0),
    "W": ("左", 0.0, 1.0),
    "C": ("中心", 1.0, 1.0),
    "E": ("右", 2.0, 1.0),
    "SW": ("左下", 0.0, 2.0),
    "S": ("下", 1.0, 2.0),
    "SE": ("右下", 2.0, 2.0),
}
_TASK = {
    "RAW": ("原料区", 1.0, -0.5, "原料区"),
    "QR": ("二维码区", 2.5, 1.0, "二维码区"),
    "ROUGH": ("粗加工区", 1.0, 2.5, "粗加工区"),
    "BUFFER": ("暂存区", -0.5, 1.0, "暂存区"),
    "START1": ("启停区1", 2.5, 0.0, "启停区1"),
    "START2": ("启停区2", 2.5, 2.0, "启停区2"),
}

nodes: dict[str, Node] = {
    node_id: Node(node_id, label, "navigation", x, y)
    for node_id, (label, x, y) in _NAVIGATION.items()
}
nodes.update(
    {
        node_id: Node(node_id, label, "task", x, y)
        for node_id, (label, x, y, _display) in _TASK.items()
    }
)

_NAVIGATION_EDGES = (
    ("NW", "N"), ("N", "NE"),
    ("W", "C"), ("C", "E"),
    ("SW", "S"), ("S", "SE"),
    ("NW", "W"), ("W", "SW"),
    ("N", "C"), ("C", "S"),
    ("NE", "E"), ("E", "SE"),
)
_TASK_EDGES = (
    ("RAW", "N"), ("BUFFER", "W"), ("ROUGH", "S"),
    ("QR", "E"), ("START1", "NE"), ("START2", "SE"),
)
edges: tuple[Edge, ...] = tuple(
    Edge(a, b, hypot(nodes[a].x - nodes[b].x, nodes[a].y - nodes[b].y))
    for a, b in (*_NAVIGATION_EDGES, *_TASK_EDGES)
)

_EDGE_LENGTHS = {edge_key(edge.node_a, edge.node_b): edge.length for edge in edges}
_ADJACENCY: dict[str, tuple[str, ...]] = {node_id: tuple() for node_id in nodes}
for _edge in edges:
    _ADJACENCY[_edge.node_a] += (_edge.node_b,)
    _ADJACENCY[_edge.node_b] += (_edge.node_a,)


def _direction(node_a: str, node_b: str) -> int:
    dx = nodes[node_b].x - nodes[node_a].x
    dy = nodes[node_b].y - nodes[node_a].y
    if abs(dx) >= abs(dy):
        return 0 if dx > 0 else 2
    return 1 if dy > 0 else 3


def _metrics(path: tuple[str, ...]) -> tuple[float, int, int]:
    distance = sum(_EDGE_LENGTHS[edge_key(a, b)] for a, b in zip(path, path[1:]))
    quarter_turns = 0
    stops = 0
    for previous, current, following in zip(path, path[1:], path[2:]):
        delta = abs(_direction(previous, current) - _direction(current, following))
        turns = min(delta, 4 - delta)
        quarter_turns += turns
        if turns:
            stops += 1
    return distance, quarter_turns, stops


def _result(
    path: tuple[str, ...], distance_weight: float, turn_weight: float, stop_weight: float
) -> PathResult:
    distance, quarter_turns, stops = _metrics(path)
    distance_cost = distance_weight * distance
    turn_cost = turn_weight * quarter_turns
    stop_cost = stop_weight * stops
    return PathResult(
        path, distance, quarter_turns, stops,
        distance_cost, turn_cost, stop_cost,
        distance_cost + turn_cost + stop_cost,
    )


def find_best_paths(
    start_id: str,
    goal_id: str,
    blocked_edges: set[tuple[str, str]] | Iterable[tuple[str, str]] = frozenset(),
    distance_weight: float = 1.0,
    turn_weight: float = 0.75,
    stop_weight: float = 1.0,
    limit: int = 4,
) -> list[PathResult]:
    """枚举并返回固定拓扑中的最低成本简单路径。"""

    if start_id not in nodes or goal_id not in nodes or limit <= 0:
        return []
    blocked = {edge_key(a, b) for a, b in blocked_edges}
    if start_id == goal_id:
        return [_result((start_id,), distance_weight, turn_weight, stop_weight)]

    found: list[tuple[str, ...]] = []

    def visit(current: str, path: tuple[str, ...]) -> None:
        if current == goal_id:
            found.append(path)
            return
        if current != start_id and nodes[current].kind == "task":
            return
        for neighbor in _ADJACENCY[current]:
            if neighbor in path or edge_key(current, neighbor) in blocked:
                continue
            if nodes[neighbor].kind == "task" and neighbor != goal_id:
                continue
            visit(neighbor, path + (neighbor,))

    visit(start_id, (start_id,))
    results = [_result(path, distance_weight, turn_weight, stop_weight) for path in found]
    results.sort(key=lambda item: (item.total_cost, item.distance, item.quarter_turns, item.stops, item.nodes))
    return results[:limit]
