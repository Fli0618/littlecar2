"""用于编辑期禁行检查和预览的车体扫掠几何。"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .geometry import wrap_deg
from .models import (BezierPathSegment, CAR_SIZE_MM, ContinuousPathSegment, PathPosePoint,
                     Plan, Pose, RotateInPlace, StepTurnPathSegment, Waypoint)
from .path_materializer import materialize_steps
from .sim import Simulation


MAX_SAMPLE_DISTANCE_MM = 5.0
MAX_SAMPLE_YAW_DEG = 2.0


@dataclass(frozen=True)
class SweepGeometry:
    """GOTO 运行期间的连续位姿和每个采样位姿下的车体矩形。"""

    poses: list[Pose]
    polygons: list[list[tuple[float, float]]]


def _interpolate_poses(previous: Pose, current: Pose) -> list[Pose]:
    distance = math.hypot(current.x_mm - previous.x_mm, current.y_mm - previous.y_mm)
    yaw_delta = wrap_deg(current.yaw_deg - previous.yaw_deg)
    count = max(1, math.ceil(distance / MAX_SAMPLE_DISTANCE_MM), math.ceil(abs(yaw_delta) / MAX_SAMPLE_YAW_DEG))
    return [
        Pose(
            previous.x_mm + (current.x_mm - previous.x_mm) * step / count,
            previous.y_mm + (current.y_mm - previous.y_mm) * step / count,
            wrap_deg(previous.yaw_deg + yaw_delta * step / count),
        )
        for step in range(1, count + 1)
    ]


def car_polygon(pose: Pose) -> list[tuple[float, float]]:
    """返回图纸 Y 轴向下坐标系中，按实际航向旋转后的车体四角。"""
    half = CAR_SIZE_MM / 2.0
    angle = -math.radians(pose.yaw_deg)
    cosine, sine = math.cos(angle), math.sin(angle)
    return [
        (pose.x_mm + x * cosine - y * sine, pose.y_mm + x * sine + y * cosine)
        for x, y in ((-half, -half), (half, -half), (half, half), (-half, half))
    ]


def build_goto_sweep(start: Pose, target: Pose, vmax_mm_s: float, wmax_deg_s: float, timeout_s: float) -> SweepGeometry:
    """以正式仿真的同一固定加减速状态机生成 GOTO 的车体扫掠。"""
    command = Waypoint(target.x_mm, target.y_mm, target.yaw_deg, use_yaw=True, vmax_mm_s=vmax_mm_s, wmax_deg_s=wmax_deg_s, timeout_s=timeout_s)
    simulation = Simulation([command])
    simulation.actual = Pose(start.x_mm, start.y_mm, start.yaw_deg)
    poses = [Pose(start.x_mm, start.y_mm, start.yaw_deg)]
    while not simulation.finished and not simulation.failed:
        frame = simulation.step()
        poses.extend(_interpolate_poses(poses[-1], frame.actual))
    return SweepGeometry(poses, [car_polygon(pose) for pose in poses])


def build_rotation_sweep(start: Pose, target_yaw_deg: float, wmax_deg_s: float, timeout_s: float) -> SweepGeometry:
    """以正式仿真的同一状态机生成原地转向期间的车体扫掠。"""
    simulation = Simulation([RotateInPlace(target_yaw_deg, wmax_deg_s, timeout_s)])
    simulation.actual = Pose(start.x_mm, start.y_mm, start.yaw_deg)
    poses = [Pose(start.x_mm, start.y_mm, start.yaw_deg)]
    while not simulation.finished and not simulation.failed:
        frame = simulation.step()
        poses.extend(_interpolate_poses(poses[-1], frame.actual))
    return SweepGeometry(poses, [car_polygon(pose) for pose in poses])


def build_continuous_segment_sweep(start: Pose, target: Pose) -> SweepGeometry:
    """连续路径的纯几何扫掠；不复用停点动作的加减速模型。"""

    poses = [start, *_interpolate_poses(start, target)]
    return SweepGeometry(poses, [car_polygon(pose) for pose in poses])


def build_path_segment_sweep(
    start: Pose, step: ContinuousPathSegment | BezierPathSegment | StepTurnPathSegment,
) -> SweepGeometry:
    """Build a geometric sweep after expanding a path segment through the shared materializer."""

    materialized = materialize_steps(Plan(steps=[
        Waypoint(start.x_mm, start.y_mm, start.yaw_deg), step,
    ]))
    resolved = materialized[-1]
    if isinstance(resolved, BezierPathSegment):
        from .bezier import generate_bezier_path_points
        points = generate_bezier_path_points(
            start,
            (resolved.control_1_x_mm, resolved.control_1_y_mm),
            (resolved.control_2_x_mm, resolved.control_2_y_mm),
            Pose(resolved.end_x_mm, resolved.end_y_mm, resolved.end_yaw_deg),
            resolved.yaw_mode,
            resolved.sample_spacing_mm,
        )
    elif isinstance(resolved, ContinuousPathSegment):
        points = resolved.points
    else:
        raise ValueError("路径步骤无法展开为连续路径")
    if len(points) < 2:
        raise ValueError("连续路径至少需要两个点")
    poses = [Pose(points[0].x_mm, points[0].y_mm, points[0].yaw_deg)]
    for target in points[1:]:
        poses.extend(_interpolate_poses(poses[-1], Pose(target.x_mm, target.y_mm, target.yaw_deg)))
    return SweepGeometry(poses, [car_polygon(pose) for pose in poses])
