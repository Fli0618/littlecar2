"""将编辑态的垫步路径展开为既有运行时路径步骤。"""

from __future__ import annotations

from copy import deepcopy

from .bezier import generate_bezier_path_points
from .models import (BezierPathSegment, ContinuousPathSegment, MotionStep, PathPosePoint,
                     Plan, Pose, RotateInPlace, StepTurnPathSegment, Waypoint)
from .step_turn import generate_step_turn_path_points


def materialize_path_step(start_pose: Pose, step: MotionStep) -> list[PathPosePoint]:
    """展开单个路径步骤，不修改输入路径或其控制点。"""

    if isinstance(step, ContinuousPathSegment):
        return deepcopy(step.points)
    if isinstance(step, BezierPathSegment):
        return generate_bezier_path_points(
            start_pose,
            (step.control_1_x_mm, step.control_1_y_mm),
            (step.control_2_x_mm, step.control_2_y_mm),
            Pose(step.end_x_mm, step.end_y_mm, step.end_yaw_deg),
            step.yaw_mode,
            step.sample_spacing_mm,
        )
    if isinstance(step, StepTurnPathSegment):
        return list(generate_step_turn_path_points(start_pose, step))
    raise TypeError("仅可展开连续路径、Bezier 路径或垫步路径")


def materialize_steps(plan: Plan) -> list[MotionStep]:
    """返回可供既有仿真和代码生成使用的步骤，不修改原始方案。"""

    cursor = Pose(0.0, 0.0, 0.0)
    result: list[MotionStep] = []
    for step in plan.steps:
        if isinstance(step, (ContinuousPathSegment, BezierPathSegment, StepTurnPathSegment)):
            points = materialize_path_step(cursor, step)
            # 仅垫步路径是编辑态语义，需替换为运行时连续路径；保留既有路径类型。
            result.append(ContinuousPathSegment(points=points, name=step.name)
                          if isinstance(step, StepTurnPathSegment) else deepcopy(step))
            if points:
                cursor = Pose(points[-1].x_mm, points[-1].y_mm, points[-1].yaw_deg)
        else:
            result.append(deepcopy(step))
            cursor = _step_end_pose(cursor, step)
    return result


def materialize_plan(plan: Plan) -> Plan:
    """以展开后的步骤复制方案，保留元数据和场地布局。"""

    materialized = deepcopy(plan)
    materialized.steps = materialize_steps(plan)
    return materialized


def _step_end_pose(cursor: Pose, step: MotionStep) -> Pose:
    if isinstance(step, Waypoint):
        return Pose(step.x_mm, step.y_mm, step.yaw_deg)
    if isinstance(step, RotateInPlace):
        return Pose(cursor.x_mm, cursor.y_mm, step.yaw_deg)
    if isinstance(step, ContinuousPathSegment) and step.points:
        point = step.points[-1]
        return Pose(point.x_mm, point.y_mm, point.yaw_deg)
    if isinstance(step, BezierPathSegment):
        return Pose(step.end_x_mm, step.end_y_mm, step.end_yaw_deg)
    return cursor
