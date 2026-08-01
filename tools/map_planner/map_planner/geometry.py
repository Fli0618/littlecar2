"""图纸坐标与以车辆起点为原点的固定世界坐标之间的换算。"""

from __future__ import annotations

import math

from .models import Pose


def wrap_deg(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def heading_vector(yaw_deg: float) -> tuple[float, float]:
    rad = math.radians(yaw_deg)
    return math.sin(rad), math.cos(rad)


def world_to_paper(pose: Pose, start_x: float, start_y: float, heading_deg: float) -> tuple[float, float]:
    """将 +Y 前方、+X 右方的世界坐标映射到 Y 向下的图纸坐标。"""
    forward_x, forward_y = heading_vector(heading_deg)
    right_x, right_y = forward_y, -forward_x
    return (
        start_x + pose.x_mm * right_x + pose.y_mm * forward_x,
        start_y + pose.x_mm * right_y + pose.y_mm * forward_y,
    )


def paper_to_world(paper_x: float, paper_y: float, start_x: float, start_y: float, heading_deg: float) -> Pose:
    dx, dy = paper_x - start_x, paper_y - start_y
    forward_x, forward_y = heading_vector(heading_deg)
    right_x, right_y = forward_y, -forward_x
    return Pose(dx * right_x + dy * right_y, dx * forward_x + dy * forward_y, 0.0)


def polyline_length(points: list[Pose]) -> float:
    return sum(math.hypot(b.x_mm - a.x_mm, b.y_mm - a.y_mm) for a, b in zip(points, points[1:]))
