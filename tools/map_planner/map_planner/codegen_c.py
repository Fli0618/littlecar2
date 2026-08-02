"""Generate deterministic STM32 C task functions from a map plan."""

from __future__ import annotations

import math
import re

from .models import PathPosePoint, Plan, RotateInPlace, Waypoint


class CodeGenerationError(ValueError):
    """Raised when a plan cannot be represented by the blocking STM32 API."""


_TASK_NAME = re.compile(r"^Task_[A-Za-z_][A-Za-z0-9_]*$")
_FIRMWARE_VMAX_MM_S = 820.0
_FIRMWARE_WMAX_DEG_S = 100.0
_FIRMWARE_TIMEOUT_S = 10.0
_DEFAULT_PARAMETER_WARNING = (
    "生成代码使用 AdvanceMotion_GotoPoseBlocking，节点的最大线速度、最大角速度和超时"
    "参数不会写入代码，运行时使用 STM32 固件中的默认参数。"
)


def _safe_comment(value: object) -> str:
    """Keep user-provided text from terminating a generated C comment."""

    text = str(value).replace("\r", " ").replace("\n", " ")
    return text.replace("/*", "/ *").replace("*/", "* /")


def default_task_function_name(plan_name: str) -> str:
    """Return the stable ``Task_...`` name derived from a plan name."""

    pieces: list[str] = []
    for character in str(plan_name):
        if character in "-_" or character.isspace():
            pieces.append("_")
        elif (
            ("A" <= character <= "Z")
            or ("a" <= character <= "z")
            or ("0" <= character <= "9")
        ):
            pieces.append(character)
    stem = re.sub(r"_+", "_", "".join(pieces)).strip("_") or "Plan"
    if stem[0].isdigit():
        stem = f"Plan_{stem}"
    return f"Task_{stem}"


def validate_task_function_name(function_name: str) -> None:
    """Validate a complete generated C function name."""

    if not _TASK_NAME.fullmatch(function_name):
        raise CodeGenerationError(
            "函数名称必须符合 Task_<Name> 格式，只能包含 ASCII 字母、数字和下划线，"
            "且 Task_ 后的首字符不能是数字。"
        )


def _finite(value: float, label: str, step: int) -> None:
    if not math.isfinite(float(value)):
        raise CodeGenerationError(f"步骤 {step} 的 {label} 必须是有限数值。")


def validate_plan_for_blocking_codegen(plan: Plan) -> list[str]:
    """Validate blocking-codegen semantics and return non-blocking warnings."""

    if plan.mode == "continuous":
        return _validate_continuous_plan(plan)
    if not plan.waypoints:
        raise CodeGenerationError("当前方案没有任何运动动作。")

    has_non_default = False
    for index, action in enumerate(plan.waypoints, start=1):
        if isinstance(action, Waypoint):
            _finite(action.x_mm, "X 坐标", index)
            _finite(action.y_mm, "Y 坐标", index)
            _finite(action.yaw_deg, "航向角", index)
            _finite(action.dwell_s, "停留时间", index)
            _finite(action.vmax_mm_s, "最大线速度", index)
            _finite(action.wmax_deg_s, "最大角速度", index)
            _finite(action.timeout_s, "超时", index)
            if action.dwell_s < 0:
                raise CodeGenerationError(f"步骤 {index} 的停留时间不能为负数。")
            if not action.use_yaw:
                raise CodeGenerationError(
                    f"步骤 {index} 未启用航向约束，AdvanceMotion_GotoPoseBlocking 无法表达该动作。"
                    "请启用航向约束后重新生成。"
                )
            if not action.stop:
                raise CodeGenerationError(
                    f"步骤 {index} 设置为不停顿途经点，当前阻塞式接口无法实现连续通过。"
                    "请启用“到点停止”后重新生成。"
                )
            has_non_default |= (
                action.vmax_mm_s != _FIRMWARE_VMAX_MM_S
                or action.wmax_deg_s != _FIRMWARE_WMAX_DEG_S
                or action.timeout_s != _FIRMWARE_TIMEOUT_S
            )
        elif isinstance(action, RotateInPlace):
            _finite(action.yaw_deg, "航向角", index)
            _finite(action.wmax_deg_s, "最大角速度", index)
            _finite(action.timeout_s, "超时", index)
            has_non_default |= (
                action.wmax_deg_s != _FIRMWARE_WMAX_DEG_S
                or action.timeout_s != _FIRMWARE_TIMEOUT_S
            )
        else:  # Protect callers from future MotionCommand variants.
            raise CodeGenerationError(f"步骤 {index} 使用了不支持的动作类型。")

    return [_DEFAULT_PARAMETER_WARNING] if has_non_default else []


def _validate_continuous_plan(plan: Plan) -> list[str]:
    if not plan.path_points:
        raise CodeGenerationError("当前连续路径没有任何位姿点。")
    for index, point in enumerate(plan.path_points, start=1):
        _finite(point.x_mm, "X 坐标", index)
        _finite(point.y_mm, "Y 坐标", index)
        _finite(point.yaw_deg, "航向角", index)
    return []


def format_c_float(value: float) -> str:
    """Format a finite Python float as a stable C floating-point literal."""

    number = float(value)
    if not math.isfinite(number):
        raise CodeGenerationError("C 浮点常量不能是 NaN 或 Infinity。")
    if number == 0.0:
        return "0.0f"

    rendered = format(number, ".9f").rstrip("0").rstrip(".")
    if rendered in {"-0", "0"}:
        rendered = "0.0"
    elif "." not in rendered:
        rendered += ".0"
    return f"{rendered}f"


def _motion_block(x_mm: float, y_mm: float, yaw_deg: float, indent: str = "    ") -> list[str]:
    return [
        f"{indent}if (AdvanceMotion_GotoPoseBlocking(",
        f"{indent}        {format_c_float(x_mm)},",
        f"{indent}        {format_c_float(y_mm)},",
        f"{indent}        {format_c_float(yaw_deg)},",
        f"{indent}        CHASSIS_DEFAULT_ACC) != ADVANCE_MOTION_STATE_ARRIVED)",
        f"{indent}{{",
        f"{indent}    AdvanceMotion_Cancel();",
        f"{indent}    return;",
        f"{indent}}}",
    ]


def generate_task_function(plan: Plan, function_name: str) -> str:
    """Generate one complete, deterministic ``void Task_...(void)`` function."""

    validate_task_function_name(function_name)
    validate_plan_for_blocking_codegen(plan)

    plan_name = _safe_comment(plan.name)
    lines = [
        "/*",
        f" * 方案：{plan_name}",
        " * 由 LittleCar2 地图路径规划工具生成。",
        " *",
        " * 本函数使用 AdvanceMotion_GotoPoseBlocking。",
        " * 节点速度、角速度和超时使用 STM32 固件默认配置。",
        " * 调用前必须完成 AdvanceWorld 世界坐标原点建立。",
        " */",
        f"void {function_name}(void)",
        "{",
    ]

    if plan.mode == "continuous":
        return _generate_continuous_task_function(plan, function_name)

    previous_x = 0.0
    previous_y = 0.0
    blocks: list[list[str]] = []
    for index, action in enumerate(plan.waypoints, start=1):
        if isinstance(action, Waypoint):
            name = _safe_comment(action.name).strip()
            label = f"节点 {index}" if not name else f"节点 {index}：{name}"
            block = [
                f"    /* {index}. {label}：GOTO ({format_c_float(action.x_mm)[:-1]}, "
                f"{format_c_float(action.y_mm)[:-1]}), yaw={format_c_float(action.yaw_deg)[:-1]} deg */",
                *_motion_block(action.x_mm, action.y_mm, action.yaw_deg),
            ]
            dwell_ms = round(action.dwell_s * 1000.0)
            if dwell_ms > 0:
                block.extend(["", f"    HAL_Delay({dwell_ms}U);"])
            blocks.append(block)
            previous_x = action.x_mm
            previous_y = action.y_mm
        else:
            block = [
                "    /*",
                f"     * {index}. 原地转向至 {format_c_float(action.yaw_deg)[:-1]} deg。",
                f"     * 保持上一目标位置 ({format_c_float(previous_x)[:-1]}, "
                f"{format_c_float(previous_y)[:-1]})。",
                "     */",
                "    /* 原地转向：保持上一目标位置并更新绝对航向。 */",
                *_motion_block(previous_x, previous_y, action.yaw_deg),
            ]
            blocks.append(block)

    for block_index, block in enumerate(blocks):
        if block_index:
            lines.append("")
        lines.extend(block)
    lines.extend(["}", ""])
    return "\n".join(lines)


def _generate_continuous_task_function(plan: Plan, function_name: str) -> str:
    """Generate one FollowPathBlocking call backed by a static pose array."""

    points = plan.path_points
    lines = [
        "/*",
        f" * 方案：{_safe_comment(plan.name)}（连续路径）",
        " * 由 LittleCar2 地图路径规划工具生成。",
        " * 连续路径由 AdvanceMotion_FollowPathBlocking 一次执行；不在中间位姿点停车。",
        " */",
        f"void {function_name}(void)",
        "{",
        "    static const AdvancePathPose path[] = {",
    ]
    for index, point in enumerate(points, start=1):
        name = _safe_comment(point.name).strip()
        suffix = f" /* {index}. {name} */" if name else f" /* {index}. */"
        lines.append(
            "        {"
            f"{format_c_float(point.x_mm)}, {format_c_float(point.y_mm)}, {format_c_float(point.yaw_deg)}"
            f"}},{suffix}"
        )
    lines.extend(
        [
            "    };",
            "",
            "    if (AdvanceMotion_FollowPathBlocking(path, sizeof(path) / sizeof(path[0]), CHASSIS_DEFAULT_ACC) != ADVANCE_MOTION_STATE_ARRIVED)",
            "    {",
            "        AdvanceMotion_Cancel();",
            "        return;",
            "    }",
            "}",
            "",
        ]
    )
    return "\n".join(lines)
