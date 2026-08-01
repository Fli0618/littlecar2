import unittest

from map_planner.geometry import arc_points, bezier_points, paper_to_world, polyline_length, sample_route, world_to_paper
from map_planner.models import Pose, Segment, Waypoint


class GeometryTests(unittest.TestCase):
    def test_world_paper_round_trip(self):
        source = Pose(120, 340, 0)
        paper = world_to_paper(source, 2250, 150, 180)
        result = paper_to_world(*paper, 2250, 150, 180)
        self.assertAlmostEqual(result.x_mm, source.x_mm); self.assertAlmostEqual(result.y_mm, source.y_mm)

    def test_bezier_reaches_endpoints(self):
        values = bezier_points(Waypoint(0, 0), Waypoint(300, 300), 100)
        self.assertEqual((values[0].x_mm, values[-1].y_mm), (0.0, 300.0)); self.assertGreater(polyline_length(values), 0)

    def test_invalid_arc_is_reported(self):
        _, _, errors = sample_route([Waypoint(0, 0), Waypoint(1000, 0)], [Segment("arc", arc_radius_mm=100)])
        self.assertTrue(errors)

    def test_arc_reaches_end(self):
        values = arc_points(Waypoint(0, 0), Waypoint(200, 0), 150, False)
        self.assertAlmostEqual(values[-1].x_mm, 200)
