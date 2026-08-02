import math

import pytest

from map_planner.codegen_c import (
    CodeGenerationError,
    default_task_function_name,
    format_c_float,
    generate_task_function,
    validate_task_function_name,
    validate_plan_for_blocking_codegen,
)
from map_planner.models import Plan, RotateInPlace, Waypoint


def test_default_function_names_are_ascii_and_stable():
    assert default_task_function_name("StartArea1-Raw") == "Task_StartArea1_Raw"
    assert default_task_function_name("area 1 test") == "Task_area_1_test"
    assert default_task_function_name("启停区") == "Task_Plan"
    assert default_task_function_name("123Test") == "Task_Plan_123Test"


@pytest.mark.parametrize("name", ["Task_1Bad", "Task_bad-name", "Task_bad()", "Task_中文", "Task_bad/*x*/"])
def test_invalid_function_names_are_rejected(name):
    with pytest.raises(CodeGenerationError):
        validate_task_function_name(name)


@pytest.mark.parametrize(
    ("value", "expected"),
    [(0.0, "0.0f"), (-0.0, "0.0f"), (500, "500.0f"), (-12, "-12.0f"), (12.5, "12.5f"), (1.2345678912, "1.234567891f")],
)
def test_format_c_float(value, expected):
    assert format_c_float(value) == expected


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_format_c_float_rejects_non_finite(value):
    with pytest.raises(CodeGenerationError):
        format_c_float(value)


def test_single_goto_contains_pose_and_failure_guard():
    plan = Plan(name="StartArea1", waypoints=[Waypoint(500, 800, 90, dwell_s=0)])
    code = generate_task_function(plan, "Task_StartArea1")
    assert "void Task_StartArea1(void)" in code
    assert "AdvanceMotion_GotoPoseBlocking(" in code
    assert "500.0f," in code and "800.0f," in code and "90.0f," in code
    assert "CHASSIS_DEFAULT_ACC" in code
    assert "ADVANCE_MOTION_STATE_ARRIVED" in code
    assert "AdvanceMotion_Cancel();" in code
    assert "HAL_Delay" not in code


def test_multiple_actions_keep_order_and_rotate_at_last_goto():
    plan = Plan(
        name="Route",
        waypoints=[
            RotateInPlace(45),
            Waypoint(100, 200, 0, dwell_s=0),
            RotateInPlace(90),
            RotateInPlace(-90),
            Waypoint(300, 400, 180, dwell_s=0),
            RotateInPlace(0),
        ],
    )
    code = generate_task_function(plan, "Task_Route")
    assert code.index("0.0f,\n            0.0f,\n            45.0f") < code.index("100.0f,\n            200.0f")
    assert code.count("AdvanceMotion_GotoPoseBlocking(") == 6
    assert "100.0f,\n            200.0f,\n            90.0f" in code
    assert "100.0f,\n            200.0f,\n            -90.0f" in code
    assert "300.0f,\n            400.0f,\n            0.0f" in code
    assert "/* 原地转向：保持上一目标位置并更新绝对航向。 */" in code


def test_dwell_rounds_to_milliseconds():
    code = generate_task_function(Plan(waypoints=[Waypoint(1, 2, dwell_s=0.5)]), "Task_Plan")
    assert "HAL_Delay(500U);" in code


@pytest.mark.parametrize(
    "waypoint",
    [Waypoint(1, 2, use_yaw=False), Waypoint(1, 2, stop=False)],
)
def test_unsupported_waypoint_semantics_are_rejected(waypoint):
    with pytest.raises(CodeGenerationError):
        validate_plan_for_blocking_codegen(Plan(waypoints=[waypoint]))


def test_empty_plan_is_rejected():
    with pytest.raises(CodeGenerationError, match="没有任何运动动作"):
        validate_plan_for_blocking_codegen(Plan())


def test_comments_are_safe_and_generation_is_deterministic():
    plan = Plan(name="方案 */\n名称", waypoints=[Waypoint(1, 2, name="节点 */\n名称", dwell_s=0)])
    first = generate_task_function(plan, "Task_Plan")
    second = generate_task_function(plan, "Task_Plan")
    assert first == second
    header = first.split("void Task_Plan", 1)[0]
    assert "方案 * / 名称" in header
    assert "节点 * / 名称" in first


def test_non_default_firmware_parameters_return_warning():
    warnings = validate_plan_for_blocking_codegen(
        Plan(waypoints=[Waypoint(1, 2, dwell_s=0, vmax_mm_s=900, wmax_deg_s=100, timeout_s=10)])
    )
    assert len(warnings) == 1
    assert "默认参数" in warnings[0]
