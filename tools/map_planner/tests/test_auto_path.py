import math

import pytest

from map_planner.auto_path import (AutoPathError, AutoPathSettings,
                                   _center_bounds, _segment_hits_rect,
                                   _shape_contains, _weighted_edge_cost,
                                   boundary_inset_rects,
                                   build_inflated_obstacles, plan_auto_path)
from map_planner.models import CostmapSettings, Obstacle, Pose


START_FRAME = Pose(2250.0, 150.0, 180.0)


def test_inflation_uses_vehicle_half_width_margin_and_custom_radius():
    settings = AutoPathSettings(costmap=CostmapSettings(
        platform_safety_margin_mm=20,
        obstacle_safety_margin_mm=20,
    ))
    rects = build_inflated_obstacles([Obstacle(300, 300)], settings)

    platform = next(rect for rect in rects if rect.source == "platform")
    assert (platform.left, platform.top, platform.right, platform.bottom) == (380, 380, 1170, 1170)
    assert platform.corner_radius == 170
    assert not _shape_contains((380, 380), platform)
    assert _shape_contains((775, 380), platform)
    custom = rects[-1]
    assert (custom.left, custom.top, custom.right, custom.bottom) == (105, 105, 495, 495)
    assert custom.corner_radius == 195
    assert not _shape_contains((105, 105), custom)
    assert _shape_contains((300, 105), custom)
    assert _segment_hits_rect((100, 105), (500, 105), custom)
    assert not _segment_hits_rect((100, 104), (500, 104), custom)


def test_inflation_uses_configured_vehicle_dimensions():
    settings = AutoPathSettings(costmap=CostmapSettings(
        vehicle_length_mm=360,
        vehicle_width_mm=240,
        platform_safety_margin_mm=20,
    ))

    platform = next(rect for rect in build_inflated_obstacles([], settings)
                    if rect.source == "platform")

    assert (platform.left, platform.top, platform.right, platform.bottom) == (350, 350, 1200, 1200)
    assert platform.corner_radius == 200


def test_boundary_work_zones_indent_top_left_and_bottom_edges():
    costmap = CostmapSettings(boundary_zone_half_width_mm=200,
                              boundary_zone_depth_mm=85)
    settings = AutoPathSettings(costmap=costmap,
                                include_fixed_platforms=False)
    rects = build_inflated_obstacles([], settings)
    zones = [rect for rect in rects if rect.source == "boundary_zone"]

    assert boundary_inset_rects(costmap) == (
        (1000, 0, 1400, 85), (0, 910, 150, 1490),
        (910, 2250, 1490, 2400))
    assert len(zones) == 3
    assert (zones[0].left, zones[0].top,
            zones[0].right, zones[0].bottom) == (830, -170, 1570, 255)
    assert zones[0].corner_radius == 170
    assert _shape_contains((1200, 250), zones[0])
    assert not _shape_contains((1200, 256), zones[0])


def test_boundary_cost_moves_a_free_route_away_from_green_limit():
    result = plan_auto_path(
        (2250, 150), (1500, 300), START_FRAME, 0, 0, [],
        AutoPathSettings(sample_spacing_mm=20, yaw_mode="fixed"),
    )

    assert result.route_paper[0] == (2250, 150)
    assert result.route_paper[-1] == (1500, 300)
    assert any(y > 150 for _, y in result.route_paper[1:-1])
    assert 35 < len(result.world_points) < 60
    assert all(point.yaw_deg == 0 for point in result.world_points)
    assert result.length_mm > 760


def test_platform_soft_cost_uses_mutually_exclusive_inner_and_outer_regions():
    settings = AutoPathSettings(costmap=CostmapSettings(
        boundary_cost_weight=0,
        platform_inflation_mm=100,
        platform_cost_weight=1,
        platform_outer_inflation_mm=100,
        platform_outer_cost_weight=10,
    ))
    rects = build_inflated_obstacles([], settings)
    bounds = _center_bounds(settings)

    inner_cost = _weighted_edge_cost((600, 1200), (601, 1200), bounds,
                                     rects, settings)
    outer_cost = _weighted_edge_cost((499, 1200), (500, 1200), bounds,
                                     rects, settings)

    assert outer_cost > inner_cost


def test_green_vehicle_center_boundary_is_a_hard_limit():
    with pytest.raises(AutoPathError, match="车体中心可达范围外"):
        plan_auto_path(
            (2250, 150), (149, 1200), START_FRAME, 0, 0, [],
            AutoPathSettings(include_fixed_platforms=False),
        )


def test_boundary_hard_safety_keeps_official_start_zone_exit_open():
    result = plan_auto_path(
        (2250, 150), (2000, 400), START_FRAME, 0, 0, [],
        AutoPathSettings(
            costmap=CostmapSettings(boundary_safety_margin_mm=20),
            include_fixed_platforms=False,
        ),
    )

    assert result.route_paper[0] == (2250, 150)
    assert result.route_paper[-1] == (2000, 400)
    assert any(rect.source == "boundary" for rect in result.inflated_rects)


def test_visibility_route_avoids_inflated_platform_and_smoothing_stays_free():
    result = plan_auto_path(
        (1200, 300), (350, 1200), Pose(1200, 300, 90), 0, 90, [],
        AutoPathSettings(costmap=CostmapSettings(platform_safety_margin_mm=20),
                         corner_radius_mm=100,
                         sample_spacing_mm=20, yaw_mode="tangent"),
    )

    assert len(result.route_paper) >= 3
    assert len(result.samples_paper) > len(result.route_paper)
    for x, y in result.samples_paper:
        assert not any(_shape_contains((x, y), rect)
                       for rect in result.inflated_rects)
    assert all(math.isfinite(point.yaw_deg) for point in result.world_points)


def test_goal_in_inflated_forbidden_area_is_rejected():
    with pytest.raises(AutoPathError, match="终点位于膨胀禁区"):
        plan_auto_path(
            (2250, 150), (700, 700), START_FRAME, 0, 0, [],
            AutoPathSettings(),
        )


def test_custom_obstacle_is_used_by_search():
    result = plan_auto_path(
        (300, 300), (900, 300), Pose(300, 300, 0), 0, 0,
        [Obstacle(600, 300)],
        AutoPathSettings(include_fixed_platforms=False,
                         costmap=CostmapSettings(obstacle_safety_margin_mm=0),
                         corner_radius_mm=60, sample_spacing_mm=20),
    )

    assert len(result.route_paper) >= 3
    assert result.length_mm > 600
