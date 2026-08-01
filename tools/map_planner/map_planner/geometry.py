"""曲线采样、场地图纸坐标和以小车起点为零点的世界坐标换算。"""

from __future__ import annotations

import math

from .models import Pose, Segment, Waypoint


def wrap_deg(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def heading_vector(yaw_deg: float) -> tuple[float, float]:
    rad = math.radians(yaw_deg)
    return math.sin(rad), math.cos(rad)


def world_to_paper(pose: Pose, start_x: float, start_y: float, heading_deg: float) -> tuple[float, float]:
    """将 +Y 前方、+X 右方的世界坐标映射到图纸坐标（图纸 Y 向下）。"""
    forward_x, forward_y = heading_vector(heading_deg)
    right_x, right_y = forward_y, -forward_x
    return start_x + pose.x_mm * right_x + pose.y_mm * forward_x, start_y + pose.x_mm * right_y + pose.y_mm * forward_y


def paper_to_world(paper_x: float, paper_y: float, start_x: float, start_y: float, heading_deg: float) -> Pose:
    dx, dy = paper_x - start_x, paper_y - start_y
    forward_x, forward_y = heading_vector(heading_deg)
    right_x, right_y = forward_y, -forward_x
    return Pose(dx * right_x + dy * right_y, dx * forward_x + dy * forward_y, 0.0)


def bezier_points(start: Waypoint, end: Waypoint, handle_length: float, count: int = 40) -> list[Pose]:
    sx, sy = heading_vector(start.yaw_deg)
    ex, ey = heading_vector(end.yaw_deg)
    p0, p1 = (start.x_mm, start.y_mm), (start.x_mm + sx * handle_length, start.y_mm + sy * handle_length)
    p2, p3 = (end.x_mm - ex * handle_length, end.y_mm - ey * handle_length), (end.x_mm, end.y_mm)
    values: list[Pose] = []
    for index in range(count + 1):
        t = index / count
        mt = 1.0 - t
        x = mt**3 * p0[0] + 3 * mt**2 * t * p1[0] + 3 * mt * t**2 * p2[0] + t**3 * p3[0]
        y = mt**3 * p0[1] + 3 * mt**2 * t * p1[1] + 3 * mt * t**2 * p2[1] + t**3 * p3[1]
        dx = 3 * mt**2 * (p1[0] - p0[0]) + 6 * mt * t * (p2[0] - p1[0]) + 3 * t**2 * (p3[0] - p2[0])
        dy = 3 * mt**2 * (p1[1] - p0[1]) + 6 * mt * t * (p2[1] - p1[1]) + 3 * t**2 * (p3[1] - p2[1])
        values.append(Pose(x, y, math.degrees(math.atan2(dx, dy))))
    return values


def arc_points(start: Waypoint, end: Waypoint, radius: float, clockwise: bool, count: int = 40) -> list[Pose]:
    dx, dy = end.x_mm - start.x_mm, end.y_mm - start.y_mm
    chord = math.hypot(dx, dy)
    if radius <= 0.0 or chord <= 0.0 or chord > 2.0 * radius:
        raise ValueError("圆弧半径必须不小于弦长的一半")
    mid_x, mid_y = (start.x_mm + end.x_mm) / 2.0, (start.y_mm + end.y_mm) / 2.0
    offset = math.sqrt(max(0.0, radius * radius - chord * chord / 4.0))
    normal_x, normal_y = -dy / chord, dx / chord
    side = -1.0 if clockwise else 1.0
    center_x, center_y = mid_x + side * offset * normal_x, mid_y + side * offset * normal_y
    first = math.atan2(start.y_mm - center_y, start.x_mm - center_x)
    last = math.atan2(end.y_mm - center_y, end.x_mm - center_x)
    delta = (last - first) % (2.0 * math.pi)
    if clockwise:
        delta -= 2.0 * math.pi
    values: list[Pose] = []
    for index in range(count + 1):
        angle = first + delta * index / count
        x, y = center_x + radius * math.cos(angle), center_y + radius * math.sin(angle)
        tangent = math.degrees(angle + (-math.pi / 2.0 if clockwise else math.pi / 2.0))
        values.append(Pose(x, y, tangent))
    return values


def sample_route(waypoints: list[Waypoint], segments: list[Segment]) -> tuple[list[Pose], list[int], list[str]]:
    points: list[Pose] = []
    stops: list[int] = []
    errors: list[str] = []
    for index, segment in enumerate(segments):
        try:
            start, end = waypoints[index], waypoints[index + 1]
            values = bezier_points(start, end, segment.handle_length_mm) if segment.kind == "bezier" else arc_points(start, end, segment.arc_radius_mm, segment.clockwise)
            points.extend(values if not points else values[1:])
            if end.stop:
                stops.append(len(points) - 1)
        except ValueError as error:
            errors.append(f"第 {index + 1} 段: {error}")
    return points, stops, errors


def polyline_length(points: list[Pose]) -> float:
    return sum(math.hypot(b.x_mm - a.x_mm, b.y_mm - a.y_mm) for a, b in zip(points, points[1:]))
