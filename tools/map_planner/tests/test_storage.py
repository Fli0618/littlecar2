import json
from pathlib import Path
import tempfile

import pytest

from map_planner.models import (MAP_VERSION, BezierPathSegment, ContinuousPathSegment,
                                PathPosePoint, Plan, Waypoint)
from map_planner.storage import load_plan, save_plan


def test_v7_round_trip_writes_only_steps():
    plan = Plan(name="mixed", steps=[Waypoint(1, 2), ContinuousPathSegment([PathPosePoint(1, 2), PathPosePoint(3, 4)])])
    with tempfile.TemporaryDirectory() as directory:
        path = save_plan(plan, directory=Path(directory))
        raw = json.loads(path.read_text(encoding="utf-8"))
        assert raw["map_version"] == MAP_VERSION
        assert "mode" not in raw and "commands" not in raw and "path_points" not in raw
        assert load_plan("mixed", Path(directory)).steps == plan.steps


@pytest.mark.parametrize("version", [5, 6])
def test_unsupported_legacy_versions_are_rejected(version):
    value = Plan().to_dict()
    value["map_version"] = version
    with pytest.raises(ValueError, match="map_version 7、8 或 9"):
        Plan.from_dict(value)


@pytest.mark.parametrize("version", [7, 8])
def test_v7_and_v8_plans_are_migrated_to_v9(version):
    value = Plan(steps=[Waypoint(1, 2)]).to_dict()
    value["map_version"] = version

    migrated = Plan.from_dict(value)

    assert migrated.steps == [Waypoint(2, 1)]
    assert migrated.to_dict()["map_version"] == MAP_VERSION


def test_v8_bezier_coordinates_are_migrated_to_v9_axes():
    value = Plan(steps=[BezierPathSegment(1, 2, 3, 4, 5, 6, 30)]).to_dict()
    value["map_version"] = 8

    migrated = Plan.from_dict(value)

    assert migrated.steps == [BezierPathSegment(2, 1, 4, 3, 6, 5, 30)]


@pytest.mark.parametrize("legacy_key", ["nodes", "commands", "path_points"])
def test_v7_plan_rejects_legacy_top_level_fields(legacy_key):
    value = Plan().to_dict()
    value[legacy_key] = []

    with pytest.raises(ValueError, match="格式无效"):
        Plan.from_dict(value)
