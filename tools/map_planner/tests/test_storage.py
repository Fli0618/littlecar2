import json
from pathlib import Path
import tempfile
import pytest
from map_planner.models import MAP_VERSION, ContinuousPathSegment, Plan, PathPosePoint, RotateInPlace, Waypoint
from map_planner.storage import load_plan, save_plan

def test_v7_round_trip_writes_only_steps():
    plan = Plan(name="mixed", steps=[Waypoint(1, 2), RotateInPlace(90), ContinuousPathSegment([PathPosePoint(1, 2), PathPosePoint(3, 4)])])
    with tempfile.TemporaryDirectory() as directory:
        path = save_plan(plan, directory=Path(directory))
        raw = json.loads(path.read_text(encoding="utf-8"))
        assert raw["map_version"] == MAP_VERSION
        assert [item["type"] for item in raw["steps"]] == ["goto_pose", "rotate_in_place", "continuous_path"]
        assert "commands" not in raw and "path_points" not in raw and "mode" not in raw
        assert load_plan("mixed", Path(directory)).steps == plan.steps

@pytest.mark.parametrize("version", [5, 6])
def test_v5_v6_migrate_to_ordered_steps(version):
    value = Plan(name="old").to_dict()
    value["map_version"] = version
    value.pop("steps")
    value["mode"] = "stop_point"
    value["commands"] = [{"type": "goto_pose", "x_mm": 1, "y_mm": 2}]
    loaded = Plan.from_dict(value)
    assert isinstance(loaded.steps[0], Waypoint)
    assert loaded.migration_warnings

def test_v6_continuous_migrates_as_one_segment():
    value = Plan(name="old").to_dict()
    value.update({"map_version": 6, "mode": "continuous", "path_points": [{"x_mm": 0, "y_mm": 0}, {"x_mm": 1, "y_mm": 0}]})
    value.pop("steps")
    loaded = Plan.from_dict(value)
    assert isinstance(loaded.steps[0], ContinuousPathSegment)
