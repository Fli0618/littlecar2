"""Deterministic cubic Bezier sampling for map-planner paths."""

from __future__ import annotations

import math

from .geometry import wrap_deg
from .models import PathPosePoint, Pose

DEFAULT_BEZIER_SAMPLE_SPACING_MM = 20.0
MIN_BEZIER_SAMPLE_SPACING_MM = 10.0
MAX_BEZIER_SAMPLE_SPACING_MM = 50.0
MAX_BEZIER_SAMPLE_POINTS = 512


def bezier_tangent_yaw(start: Pose, control_1: tuple[float, float], control_2: tuple[float, float],
                       end: Pose, t: float) -> float:
    """Return the path tangent heading at ``t`` in the planner's world frame."""
    dx, dy = evaluate_cubic_derivative(
        (start.x_mm, start.y_mm), control_1, control_2, (end.x_mm, end.y_mm), t)
    return wrap_deg(math.degrees(math.atan2(dx, dy))) if math.hypot(dx, dy) > 1e-6 else start.yaw_deg


def evaluate_cubic(p0, p1, p2, p3, t: float) -> tuple[float, float]:
    u = 1.0 - t
    return (u**3 * p0[0] + 3*u*u*t*p1[0] + 3*u*t*t*p2[0] + t**3*p3[0],
            u**3 * p0[1] + 3*u*u*t*p1[1] + 3*u*t*t*p2[1] + t**3*p3[1])


def evaluate_cubic_derivative(p0, p1, p2, p3, t: float) -> tuple[float, float]:
    u = 1.0 - t
    return (3*u*u*(p1[0]-p0[0]) + 6*u*t*(p2[0]-p1[0]) + 3*t*t*(p3[0]-p2[0]),
            3*u*u*(p1[1]-p0[1]) + 6*u*t*(p2[1]-p1[1]) + 3*t*t*(p3[1]-p2[1]))


def build_arc_length_lookup(p0, p1, p2, p3, subdivisions: int = 2048):
    points = [evaluate_cubic(p0, p1, p2, p3, i / subdivisions) for i in range(subdivisions + 1)]
    lengths = [0.0]
    for first, second in zip(points, points[1:]):
        lengths.append(lengths[-1] + math.dist(first, second))
    return [(i / subdivisions, length) for i, length in enumerate(lengths)]


def sample_cubic_by_arc_length(p0, p1, p2, p3, spacing_mm: float):
    if not MIN_BEZIER_SAMPLE_SPACING_MM <= spacing_mm <= MAX_BEZIER_SAMPLE_SPACING_MM:
        raise ValueError("Bezier sample spacing must be between 10 and 50 mm")
    lookup = build_arc_length_lookup(p0, p1, p2, p3)
    total = lookup[-1][1]
    if total < 1.0:
        raise ValueError("Bezier path is too short")
    count = math.ceil(total / spacing_mm) + 1
    if count > MAX_BEZIER_SAMPLE_POINTS:
        raise ValueError("Bezier path exceeds sample point limit")
    samples = []
    for index in range(count):
        target = min(total, index * spacing_mm)
        for (left_t, left_l), (right_t, right_l) in zip(lookup, lookup[1:]):
            if target <= right_l:
                ratio = 0.0 if right_l == left_l else (target-left_l)/(right_l-left_l)
                samples.append(left_t + (right_t-left_t)*ratio)
                break
        else:
            samples.append(1.0)
    samples[-1] = 1.0
    return samples


def generate_bezier_path_points(start: Pose, control_1: tuple[float, float], control_2: tuple[float, float], end: Pose,
                                yaw_mode: str = "interpolate", sample_spacing_mm: float = DEFAULT_BEZIER_SAMPLE_SPACING_MM) -> list[PathPosePoint]:
    if yaw_mode not in ("interpolate", "tangent", "fixed"):
        raise ValueError("Invalid Bezier yaw mode")
    values = (start.x_mm, start.y_mm, *control_1, *control_2, end.x_mm, end.y_mm, end.yaw_deg)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Bezier values must be finite")
    p0, p3 = (start.x_mm, start.y_mm), (end.x_mm, end.y_mm)
    ts = sample_cubic_by_arc_length(p0, control_1, control_2, p3, sample_spacing_mm)
    result = []
    for t in ts:
        x, y = evaluate_cubic(p0, control_1, control_2, p3, t)
        if yaw_mode == "fixed": yaw = start.yaw_deg
        elif yaw_mode == "interpolate": yaw = wrap_deg(start.yaw_deg + wrap_deg(end.yaw_deg-start.yaw_deg)*t)
        else:
            yaw = bezier_tangent_yaw(start, control_1, control_2, end, t)
        result.append(PathPosePoint(x, y, yaw))
    return result
