import unittest

from map_planner.geometry import (paper_to_world, paper_heading_to_world_yaw,
                                  StartFrame, polyline_length, rebase_plan_world_frame,
                                  world_to_paper,
                                  world_yaw_to_paper_heading, wrap_deg)
from map_planner.models import Plan, Pose, Waypoint


class GeometryTests(unittest.TestCase):
    def test_world_paper_round_trip(self):
        source = Pose(120, 340, 0)
        paper = world_to_paper(source, 2250, 150, 180)
        result = paper_to_world(*paper, 2250, 150, 180)
        self.assertAlmostEqual(result.x_mm, source.x_mm)
        self.assertAlmostEqual(result.y_mm, source.y_mm)

    def test_heading_changes_world_axes(self):
        paper = world_to_paper(Pose(0, 100), 1000, 1000, 90)
        self.assertAlmostEqual(paper[0], 1000)
        self.assertAlmostEqual(paper[1], 900)
        self.assertAlmostEqual(paper_to_world(1100, 1000, 1000, 1000, 0).y_mm, 100)
        self.assertAlmostEqual(paper_to_world(1100, 1000, 1000, 1000, 90).x_mm, 100)

    def test_heading_round_trip(self):
        self.assertAlmostEqual(world_yaw_to_paper_heading(90, 30), 120)
        self.assertAlmostEqual(paper_heading_to_world_yaw(90, 120), 30)

    def test_rebase_preserves_paper_position_and_absolute_heading(self):
        old = StartFrame(1000, 1000, 0)
        new = StartFrame(1200, 800, 90)
        plan = Plan(start_paper_x_mm=old.paper_x_mm, start_paper_y_mm=old.paper_y_mm,
                    start_heading_deg=old.heading_deg,
                    steps=[Waypoint(0, 100, 30)])
        rebased = rebase_plan_world_frame(plan, old, new)
        old_paper = world_to_paper(plan.steps[0], old.paper_x_mm, old.paper_y_mm, old.heading_deg)
        new_paper = world_to_paper(rebased.steps[0], new.paper_x_mm, new.paper_y_mm, new.heading_deg)
        self.assertAlmostEqual(old_paper[0], new_paper[0])
        self.assertAlmostEqual(old_paper[1], new_paper[1])
        self.assertAlmostEqual(world_yaw_to_paper_heading(old.heading_deg, plan.steps[0].yaw_deg),
                               world_yaw_to_paper_heading(new.heading_deg, rebased.steps[0].yaw_deg))

    def test_wrap_and_polyline_length(self):
        self.assertEqual(wrap_deg(190), -170)
        self.assertAlmostEqual(polyline_length([Pose(0, 0), Pose(3, 4), Pose(6, 8)]), 10)
