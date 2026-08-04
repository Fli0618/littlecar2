"""Generate deterministic STM32 C task functions from a map plan."""
from __future__ import annotations
import math
import re
from enum import Enum
from .bezier import generate_bezier_path_points
from .models import (BezierPathSegment, ContinuousPathSegment, Plan, Pose, RotateInPlace,
                     StepTurnPathSegment, Waypoint)
from .path_materializer import materialize_plan
from .step_turn import analyze_step_turn_path

class CodeGenerationError(ValueError):
    """Raised when a plan cannot be represented by the blocking STM32 API."""


class CodeGenerationMode(str, Enum):
    """How generated task code handles blocking motion results."""

    FEEDBACK = "feedback"
    OPEN_LOOP = "open_loop"


_TASK_NAME = re.compile(r"^Task_[A-Za-z_][A-Za-z0-9_]*$")

def _safe_comment(value: object) -> str:
    return str(value).replace("\r", " ").replace("\n", " ").replace("/*", "/ *").replace("*/", "* /")

def default_task_function_name(plan_name: str) -> str:
    stem = re.sub(r"_+", "_", "".join(c if c.isascii() and c.isalnum() else "_" for c in str(plan_name))).strip("_") or "Plan"
    return f"Task_{'Plan_' if stem[0].isdigit() else ''}{stem}"

def validate_task_function_name(function_name: str) -> None:
    if not _TASK_NAME.fullmatch(function_name): raise CodeGenerationError("函数名称必须符合 Task_<Name> 格式。")

def format_c_float(value: float) -> str:
    number = float(value)
    if not math.isfinite(number): raise CodeGenerationError("C 浮点常量不能是 NaN 或 Infinity。")
    if number == 0.0: return "0.0f"
    rendered = format(number, ".9f").rstrip("0").rstrip(".")
    return f"{rendered if '.' in rendered else rendered + '.0'}f"

def _finite(value: float, label: str, step: int) -> None:
    if not math.isfinite(float(value)): raise CodeGenerationError(f"步骤 {step} 的 {label} 必须是有限数值。")

def validate_plan_for_blocking_codegen(plan: Plan) -> list[str]:
    source_steps = plan.steps
    try:
        plan = materialize_plan(plan)
    except ValueError as error:
        raise CodeGenerationError(str(error)) from error
    if not plan.steps: raise CodeGenerationError("当前方案没有任何运动步骤。")
    current_x = current_y = current_yaw = 0.0
    for index, step in enumerate(plan.steps, 1):
        if isinstance(step, Waypoint):
            for value, label in ((step.x_mm, "X 坐标"), (step.y_mm, "Y 坐标"), (step.yaw_deg, "航向角"), (step.dwell_s, "停留时间")): _finite(value, label, index)
            if not step.use_yaw or not step.stop or step.dwell_s < 0: raise CodeGenerationError(f"步骤 {index} 的 GOTO 语义不受阻塞接口支持。")
            current_x, current_y, current_yaw = step.x_mm, step.y_mm, step.yaw_deg
        elif isinstance(step, RotateInPlace):
            _finite(step.yaw_deg, "航向角", index)
            current_yaw = step.yaw_deg
        elif isinstance(step, ContinuousPathSegment):
            if len(step.points) < 2: raise CodeGenerationError(f"步骤 {index} 的连续路径至少需要两个点。")
            first = step.points[0]
            if (not math.isclose(first.x_mm, current_x, abs_tol=1e-6)
                    or not math.isclose(first.y_mm, current_y, abs_tol=1e-6)
                    or (not isinstance(source_steps[index - 1], StepTurnPathSegment)
                        and not math.isclose(first.yaw_deg, current_yaw, abs_tol=1e-6))):
                raise CodeGenerationError(f"步骤 {index} 的入口点必须与上一动作终点一致。")
            for point_index, point in enumerate(step.points):
                for value, label in ((point.x_mm, "X 坐标"), (point.y_mm, "Y 坐标"), (point.yaw_deg, "航向角")): _finite(value, label, index)
                if point_index and math.hypot(point.x_mm - step.points[point_index-1].x_mm, point.y_mm - step.points[point_index-1].y_mm) < 1.0:
                    raise CodeGenerationError(f"步骤 {index} 的连续路径相邻点距离必须至少为 1 mm。")
            current_x, current_y, current_yaw = step.points[-1].x_mm, step.points[-1].y_mm, step.points[-1].yaw_deg
        elif isinstance(step, BezierPathSegment):
            for value, label in ((step.control_1_x_mm, "控制点 1 X"), (step.control_1_y_mm, "控制点 1 Y"), (step.control_2_x_mm, "控制点 2 X"), (step.control_2_y_mm, "控制点 2 Y"), (step.end_x_mm, "终点 X"), (step.end_y_mm, "终点 Y"), (step.end_yaw_deg, "终点航向"), (step.sample_spacing_mm, "采样间距")):
                _finite(value, label, index)
            try:
                points = generate_bezier_path_points(Pose(current_x, current_y, current_yaw), (step.control_1_x_mm, step.control_1_y_mm), (step.control_2_x_mm, step.control_2_y_mm), Pose(step.end_x_mm, step.end_y_mm, step.end_yaw_deg), step.yaw_mode, step.sample_spacing_mm)
            except ValueError as error:
                raise CodeGenerationError(f"步骤 {index} 的贝塞尔曲线无效：{error}") from error
            current_x, current_y, current_yaw = points[-1].x_mm, points[-1].y_mm, points[-1].yaw_deg
        else: raise CodeGenerationError(f"步骤 {index} 使用了不支持的动作类型。")
    return []

def _goto_block(x: float, y: float, yaw: float) -> list[str]:
    return ["    if (AdvanceMotion_GotoPoseBlocking(", f"            {format_c_float(x)},", f"            {format_c_float(y)},", f"            {format_c_float(yaw)},", "            CHASSIS_DEFAULT_ACC) != ADVANCE_MOTION_STATE_ARRIVED)", "    {", "        AdvanceMotion_Cancel();", "        return;", "    }"]


def _goto_open_loop(x: float, y: float, yaw: float) -> list[str]:
    return ["    (void)AdvanceMotion_GotoPoseBlocking(", f"            {format_c_float(x)},", f"            {format_c_float(y)},", f"            {format_c_float(yaw)},", "            CHASSIS_DEFAULT_ACC);"]


def generate_task_function(plan: Plan, function_name: str,
                           mode: CodeGenerationMode = CodeGenerationMode.FEEDBACK) -> str:
    """Generate one sequential STM32 task function in the selected result-handling mode."""

    validate_task_function_name(function_name); validate_plan_for_blocking_codegen(plan)
    source_steps = plan.steps
    try:
        plan = materialize_plan(plan)
    except ValueError as error:
        raise CodeGenerationError(str(error)) from error
    lines = ["/*", f" * Plan: {_safe_comment(plan.name)}", " * Generated by LittleCar2 map planner.", " */", f"void {function_name}(void)", "{"]
    current_x = current_y = current_yaw = 0.0
    for index, step in enumerate(plan.steps, 1):
        lines.append("")
        if isinstance(step, Waypoint):
            goto = _goto_block if mode is CodeGenerationMode.FEEDBACK else _goto_open_loop
            lines += [f"    /* {index}. GOTO */", *goto(step.x_mm, step.y_mm, step.yaw_deg)]
            delay = round(step.dwell_s * 1000)
            if delay: lines.append(f"    HAL_Delay({delay}U);")
            current_x, current_y, current_yaw = step.x_mm, step.y_mm, step.yaw_deg
        elif isinstance(step, RotateInPlace):
            goto = _goto_block if mode is CodeGenerationMode.FEEDBACK else _goto_open_loop
            lines += [f"    /* {index}. ROTATE */", *goto(current_x, current_y, step.yaw_deg)]
        elif isinstance(step, ContinuousPathSegment):
            name = f"path_{index}"
            lines += _step_turn_codegen_summary(current_x, current_y, current_yaw,
                                                source_steps[index - 1], len(step.points))
            lines.append(f"    static const AdvanceMotion_PathPoint_t {name}[] = {{")
            lines += [f"        {{{format_c_float(p.x_mm)}, {format_c_float(p.y_mm)}, {format_c_float(p.yaw_deg)}}}," for p in step.points]
            lines += ["    };", f"    /* {index}. FOLLOW PATH */"]
            if mode is CodeGenerationMode.FEEDBACK:
                lines += [f"    if (AdvanceMotion_FollowPathBlocking({name}, sizeof({name}) / sizeof({name}[0])) != ADVANCE_MOTION_STATE_ARRIVED)", "    {", "        AdvanceMotion_Cancel();", "        return;", "    }"]
            else:
                lines += [f"    (void)AdvanceMotion_FollowPathBlocking({name}, sizeof({name}) / sizeof({name}[0]));"]
            current_x, current_y, current_yaw = step.points[-1].x_mm, step.points[-1].y_mm, step.points[-1].yaw_deg
        else:
            points = generate_bezier_path_points(Pose(current_x, current_y, current_yaw), (step.control_1_x_mm, step.control_1_y_mm), (step.control_2_x_mm, step.control_2_y_mm), Pose(step.end_x_mm, step.end_y_mm, step.end_yaw_deg), step.yaw_mode, step.sample_spacing_mm)
            name = f"path_{index}"
            lines += ["", f"    static const AdvanceMotion_PathPoint_t {name}[] = {{", *[f"        {{{format_c_float(p.x_mm)}, {format_c_float(p.y_mm)}, {format_c_float(p.yaw_deg)}}}," for p in points], "    };", f"    /* {index}. FOLLOW BEZIER PATH */"]
            if mode is CodeGenerationMode.FEEDBACK: lines += [f"    if (AdvanceMotion_FollowPathBlocking({name}, sizeof({name}) / sizeof({name}[0])) != ADVANCE_MOTION_STATE_ARRIVED)", "    {", "        AdvanceMotion_Cancel();", "        return;", "    }"]
            else: lines += [f"    (void)AdvanceMotion_FollowPathBlocking({name}, sizeof({name}) / sizeof({name}[0]));"]
            current_x, current_y, current_yaw = points[-1].x_mm, points[-1].y_mm, points[-1].yaw_deg
    return "\n".join([*lines, "}", ""])


def _step_turn_codegen_summary(x_mm: float, y_mm: float, yaw_deg: float,
                               source_step: object, point_count: int) -> list[str]:
    if not isinstance(source_step, StepTurnPathSegment):
        return []
    result = analyze_step_turn_path(Pose(x_mm, y_mm, yaw_deg), source_step)
    turns = [corner for corner in result.corners if corner.has_step]
    angles = [corner.angle_deg for corner in result.corners]
    distances = [corner.effective_distance_mm for corner in turns if corner.effective_distance_mm is not None]
    return [
        "    /* STEP TURN: "
        f"nodes={len(source_step.route_points) + 1}, turns={len(turns)}, points={point_count}, "
        f"max_angle={max(angles, default=0.0):.1f} deg, min_step={min(distances, default=0.0):.1f} mm */",
    ]
