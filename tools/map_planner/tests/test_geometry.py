import unittest

from map_planner.geometry import paper_to_world, polyline_length, world_to_paper, wrap_deg
from map_planner.models import Pose


class GeometryTests(unittest.TestCase):
    def test_world_paper_round_trip(self):
        source = Pose(120, 340, 0)
        paper = world_to_paper(source, 2250, 150, 180)
        result = paper_to_world(*paper, 2250, 150, 180)
        self.assertAlmostEqual(result.x_mm, source.x_mm)
        self.assertAlmostEqual(result.y_mm, source.y_mm)

    def test_heading_changes_world_axes(self):
        paper = world_to_paper(Pose(0, 100), 1000, 1000, 90)
        self.assertAlmostEqual(paper[0], 1100)
        self.assertAlmostEqual(paper[1], 1000)

    def test_wrap_and_polyline_length(self):
        self.assertEqual(wrap_deg(190), -170)
        self.assertAlmostEqual(polyline_length([Pose(0, 0), Pose(3, 4), Pose(6, 8)]), 10)
