import math
import pytest
from map_planner.codegen_c import CodeGenerationError, CodeGenerationMode, format_c_float, generate_task_function, validate_plan_for_blocking_codegen
from map_planner.models import ContinuousPathSegment, Plan, RotateInPlace, Waypoint, PathPosePoint

def test_mixed_steps_generate_in_declared_order():
    plan = Plan(steps=[
        Waypoint(10, 20, 30, dwell_s=0), RotateInPlace(90),
        ContinuousPathSegment([PathPosePoint(10, 20, 90), PathPosePoint(100, 20, 90)]),
        Waypoint(110, 30, 0, dwell_s=0),
    ])
    code = generate_task_function(plan, "Task_Mixed")
    assert code.index("/* 1. GOTO */") < code.index("/* 2. ROTATE */") < code.index("path_3") < code.index("/* 4. GOTO */")
    assert "static const AdvanceMotion_PathPoint_t path_3[]" in code
    assert "AdvanceMotion_FollowPathBlocking(path_3, sizeof(path_3) / sizeof(path_3[0]))" in code
    assert code.count("AdvanceMotion_GotoPoseBlocking(") == 3


def test_open_loop_codegen_ignores_motion_results_and_preserves_step_order():
    plan = Plan(steps=[
        Waypoint(10, 20, 30, dwell_s=0.5), RotateInPlace(90),
        ContinuousPathSegment([PathPosePoint(10, 20, 90), PathPosePoint(100, 20, 90)]),
    ])
    code = generate_task_function(plan, "Task_OpenLoop", CodeGenerationMode.OPEN_LOOP)

    assert code.index("/* 1. GOTO */") < code.index("/* 2. ROTATE */") < code.index("/* 3. FOLLOW PATH */")
    assert code.count("(void)AdvanceMotion_GotoPoseBlocking(") == 2
    assert "(void)AdvanceMotion_FollowPathBlocking(path_3, sizeof(path_3) / sizeof(path_3[0]));" in code
    assert "HAL_Delay(500U);" in code
    assert "AdvanceMotion_Cancel()" not in code
    assert "ADVANCE_MOTION_STATE_ARRIVED" not in code
    assert "    if (" not in code


def test_default_codegen_mode_remains_feedback_control():
    plan = Plan(steps=[Waypoint(10, 20, 30)])
    code = generate_task_function(plan, "Task_Default")

    assert "if (AdvanceMotion_GotoPoseBlocking(" in code
    assert "AdvanceMotion_Cancel();" in code

def test_segment_requires_previous_endpoint_as_entry_point():
    plan = Plan(steps=[Waypoint(10, 20), ContinuousPathSegment([PathPosePoint(11, 20), PathPosePoint(50, 20)])])
    with pytest.raises(CodeGenerationError, match="入口点"):
        validate_plan_for_blocking_codegen(plan)

def test_first_segment_uses_world_origin_entry_point():
    plan = Plan(steps=[ContinuousPathSegment([PathPosePoint(0, 0), PathPosePoint(10, 0)])])
    assert "path_1" in generate_task_function(plan, "Task_Path")

@pytest.mark.parametrize("points", [[], [PathPosePoint(0, 0)], [PathPosePoint(0, 0), PathPosePoint(0.5, 0)]])
def test_segment_rejects_short_or_degenerate_sequences(points):
    with pytest.raises(CodeGenerationError):
        validate_plan_for_blocking_codegen(Plan(steps=[ContinuousPathSegment(points)]))

@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_format_c_float_rejects_non_finite(value):
    with pytest.raises(CodeGenerationError): format_c_float(value)
