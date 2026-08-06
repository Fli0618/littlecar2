"""Unit tests for motion_studio0806 models and session logic."""

import unittest
from motion_studio0806.core.models import (
    ControlMode, HolonomicParams, Obstacle, PathTemplate, PathWaypoint, TargetPose
)


class TestMotionStudioModels(unittest.TestCase):
    """Test suite for models without hardware dependency."""

    def test_target_pose(self):
        pose = TargetPose(x_mm=100.0, y_mm=200.0, yaw_deg=45.0)
        self.assertEqual(pose.x_mm, 100.0)
        self.assertEqual(pose.y_mm, 200.0)
        self.assertEqual(pose.yaw_deg, 45.0)
        self.assertTrue(pose.use_position)

    def test_path_template_serialization(self):
        wps = [
            PathWaypoint(0, 0, 0, 800),
            PathWaypoint(500, 500, 45, 600)
        ]
        tmpl = PathTemplate(name="TestS", description="Demo S curve", waypoints=wps)
        json_str = tmpl.to_json()
        self.assertIn("TestS", json_str)

        loaded_tmpl = PathTemplate.from_json(json_str)
        self.assertEqual(loaded_tmpl.name, "TestS")
        self.assertEqual(len(loaded_tmpl.waypoints), 2)
        self.assertEqual(loaded_tmpl.waypoints[1].x_mm, 500.0)

    def test_obstacle_model(self):
        obs = Obstacle(id="obs1", shape="rect", x_mm=100, y_mm=200, width_mm=300, height_mm=400)
        d = obs.to_dict()
        self.assertEqual(d["shape"], "rect")
        self.assertEqual(d["width_mm"], 300)


if __name__ == "__main__":
    unittest.main()
