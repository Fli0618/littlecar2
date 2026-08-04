import pytest

from map_planner.models import Plan, Pose, StepTurnNode, StepTurnPathSegment
from map_planner.path_materializer import materialize_path_step, materialize_steps
from map_planner.step_turn import StepTurnCompiler


def test_compiler_inserts_deterministic_ab_points_and_line_yaw():
    segment = StepTurnPathSegment([StepTurnNode(0, 200), StepTurnNode(200, 200)])

    result = StepTurnCompiler.analyze(Pose(), segment)

    assert not result.has_errors
    assert [(point.x_mm, point.y_mm) for point in result.points] == [(0, 0), (0.0, 140.0), (60.0, 200.0), (200, 200)]
    assert [point.yaw_deg for point in result.points] == [0.0, -45.0, -90.0, -90.0]


def test_compiler_rejects_short_effective_step_and_generate_raises():
    segment = StepTurnPathSegment([StepTurnNode(0, 100), StepTurnNode(100, 100)])

    analysis = StepTurnCompiler.analyze(Pose(), segment)

    assert analysis.has_errors
    assert analysis.diagnostics[0].code == "effective_step_too_short"
    with pytest.raises(ValueError, match="35 mm"):
        StepTurnCompiler.generate(Pose(), segment)


def test_step_turn_is_v10_only_and_materializes_without_mutation():
    plan = Plan(steps=[StepTurnPathSegment([StepTurnNode(0, 200)])])
    raw = plan.to_dict()

    assert raw["map_version"] == 10
    assert Plan.from_dict(raw).steps == plan.steps
    raw["map_version"] = 9
    with pytest.raises(ValueError):
        Plan.from_dict(raw)
    materialized = materialize_steps(plan)
    assert materialized[0].points[0].x_mm == 0
    assert isinstance(plan.steps[0], StepTurnPathSegment)


def test_v9_documents_remain_readable_without_step_turn_path():
    raw = Plan().to_dict()
    raw["map_version"] = 9

    assert Plan.from_dict(raw).to_dict()["map_version"] == 10


def test_materialize_path_step_copies_continuous_path_and_rejects_non_path_step():
    from map_planner.models import ContinuousPathSegment, PathPosePoint, Waypoint

    segment = ContinuousPathSegment([PathPosePoint(0, 0), PathPosePoint(0, 100)])
    points = materialize_path_step(Pose(), segment)

    assert points == segment.points
    assert points is not segment.points and points[0] is not segment.points[0]
    with pytest.raises(TypeError):
        materialize_path_step(Pose(), Waypoint(1, 2))
