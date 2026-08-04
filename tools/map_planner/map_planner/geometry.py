"""图纸坐标与以车辆起点为原点的固定世界坐标之间的换算。"""

from __future__ import annotations

import math
from copy import deepcopy
from dataclasses import dataclass

from .models import Pose


@dataclass(frozen=True)
class StartFrame:
    paper_x_mm: float
    paper_y_mm: float
    heading_deg: float


def wrap_deg(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def paper_forward_vector(start_heading_deg: float) -> tuple[float, float]:
    """返回起点世界前方向在纸面坐标中的单位向量。"""
    rad = math.radians(start_heading_deg)
    return math.cos(rad), -math.sin(rad)


def paper_right_vector(start_heading_deg: float) -> tuple[float, float]:
    """返回起点世界右方向在纸面坐标中的单位向量。"""
    rad = math.radians(start_heading_deg)
    return math.sin(rad), math.cos(rad)


def heading_vector(yaw_deg: float) -> tuple[float, float]:
    """兼容旧调用：世界 yaw 转为纸面中的朝向向量。"""
    rad = math.radians(yaw_deg)
    return math.cos(rad), -math.sin(rad)


def world_to_paper(pose: Pose, start_x: float, start_y: float, heading_deg: float) -> tuple[float, float]:
    """将 +Y 前方、+X 右方的世界坐标映射到 Y 向下的图纸坐标。"""
    right_x, right_y = paper_right_vector(heading_deg)
    forward_x, forward_y = paper_forward_vector(heading_deg)
    return (
        start_x + pose.x_mm * right_x + pose.y_mm * forward_x,
        start_y + pose.x_mm * right_y + pose.y_mm * forward_y,
    )


def paper_to_world(paper_x: float, paper_y: float, start_x: float, start_y: float, heading_deg: float) -> Pose:
    dx, dy = paper_x - start_x, paper_y - start_y
    right_x, right_y = paper_right_vector(heading_deg)
    forward_x, forward_y = paper_forward_vector(heading_deg)
    return Pose(dx * right_x + dy * right_y, dx * forward_x + dy * forward_y, 0.0)


def world_yaw_to_paper_heading(start_heading_deg: float, world_yaw_deg: float) -> float:
    return wrap_deg(start_heading_deg + world_yaw_deg)


def paper_heading_to_world_yaw(start_heading_deg: float, paper_heading_deg: float) -> float:
    return wrap_deg(paper_heading_deg - start_heading_deg)


def paper_vector_to_heading(delta_u: float, delta_v: float) -> float:
    if math.hypot(delta_u, delta_v) <= 1e-9:
        return 0.0
    return wrap_deg(math.degrees(math.atan2(-delta_v, delta_u)))


def qgraphics_rotation_deg(start_heading_deg: float, world_yaw_deg: float) -> float:
    return -world_yaw_to_paper_heading(start_heading_deg, world_yaw_deg)


def rebase_plan_world_frame(plan: object, old_frame: StartFrame, new_frame: StartFrame) -> object:
    """将计划中的固定世界目标从旧起点帧重基准到新起点帧。"""
    from .models import BezierPathSegment, ContinuousPathSegment, RotateInPlace, Waypoint

    updated = deepcopy(plan)

    def rebase_xy(x_mm: float, y_mm: float) -> tuple[float, float]:
        paper_x, paper_y = world_to_paper(
            Pose(x_mm, y_mm), old_frame.paper_x_mm, old_frame.paper_y_mm, old_frame.heading_deg)
        pose = paper_to_world(paper_x, paper_y, new_frame.paper_x_mm,
                              new_frame.paper_y_mm, new_frame.heading_deg)
        return pose.x_mm, pose.y_mm

    def rebase_yaw(yaw_deg: float) -> float:
        absolute = world_yaw_to_paper_heading(old_frame.heading_deg, yaw_deg)
        return paper_heading_to_world_yaw(new_frame.heading_deg, absolute)

    for step in updated.steps:
        if isinstance(step, Waypoint):
            step.x_mm, step.y_mm = rebase_xy(step.x_mm, step.y_mm)
            step.yaw_deg = rebase_yaw(step.yaw_deg)
        elif isinstance(step, RotateInPlace):
            step.yaw_deg = rebase_yaw(step.yaw_deg)
        elif isinstance(step, ContinuousPathSegment):
            for point in step.points:
                point.x_mm, point.y_mm = rebase_xy(point.x_mm, point.y_mm)
                point.yaw_deg = rebase_yaw(point.yaw_deg)
        elif isinstance(step, BezierPathSegment):
            step.control_1_x_mm, step.control_1_y_mm = rebase_xy(step.control_1_x_mm, step.control_1_y_mm)
            step.control_2_x_mm, step.control_2_y_mm = rebase_xy(step.control_2_x_mm, step.control_2_y_mm)
            step.end_x_mm, step.end_y_mm = rebase_xy(step.end_x_mm, step.end_y_mm)
            if step.yaw_mode != "tangent":
                step.end_yaw_deg = rebase_yaw(step.end_yaw_deg)
    updated.start_paper_x_mm = new_frame.paper_x_mm
    updated.start_paper_y_mm = new_frame.paper_y_mm
    updated.start_heading_deg = wrap_deg(new_frame.heading_deg)
    return updated


def polyline_length(points: list[Pose]) -> float:
    return sum(math.hypot(b.x_mm - a.x_mm, b.y_mm - a.y_mm) for a, b in zip(points, points[1:]))
