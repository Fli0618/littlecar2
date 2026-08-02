import unittest

from map_planner.models import PathPosePoint, RotateInPlace, Waypoint
from map_planner.sim import (
    ANGULAR_ACCELERATION_DEG_S2,
    DT_S,
    LINEAR_ACCELERATION_MM_S2,
    Simulation,
    build_timeline,
    build_continuous_timeline,
)
from map_planner.models import Pose
from map_planner.sweep import MAX_SAMPLE_DISTANCE_MM, MAX_SAMPLE_YAW_DEG, build_goto_sweep, build_rotation_sweep


class SimulationTests(unittest.TestCase):
    def test_linear_motion_accelerates_then_brakes_at_fixed_rate(self):
        simulation = Simulation([Waypoint(1000, 0, vmax_mm_s=100, timeout_s=30)])
        positions = [simulation.actual.x_mm]
        simulation.step()
        self.assertAlmostEqual(simulation.vx, LINEAR_ACCELERATION_MM_S2 * DT_S)
        for _ in range(20):
            positions.append(simulation.step().actual.x_mm)
        self.assertLessEqual(abs(simulation.vx), 100)
        for _ in range(1000):
            positions.append(simulation.step().actual.x_mm)
            if simulation.finished:
                break
        self.assertTrue(simulation.finished)
        self.assertEqual((simulation.actual.x_mm, simulation.actual.y_mm), (1000, 0))
        self.assertEqual((simulation.vx, simulation.vy), (0, 0))
        self.assertLessEqual(max(b-a for a,b in zip(positions,positions[1:])), 100 * DT_S)

    def test_rotation_keeps_position_and_obeys_fixed_angular_acceleration(self):
        simulation = Simulation([RotateInPlace(90, wmax_deg_s=60, timeout_s=30)])
        simulation.actual.x_mm = 123
        simulation.actual.y_mm = 456
        simulation.step()
        self.assertAlmostEqual(simulation.wz, ANGULAR_ACCELERATION_DEG_S2 * DT_S)
        self.assertLessEqual(abs(simulation.wz), 60)
        for _ in range(1000):
            simulation.step()
            if simulation.finished:
                break
        self.assertTrue(simulation.finished)
        self.assertEqual((simulation.actual.x_mm, simulation.actual.y_mm), (123, 456))
        self.assertEqual(simulation.actual.yaw_deg, 90)

    def test_timeout_marks_failed_frame(self):
        simulation = Simulation([Waypoint(1000, 0, vmax_mm_s=1, timeout_s=0.05)])
        for _ in range(3):
            frame = simulation.step()
        self.assertTrue(simulation.failed)
        self.assertTrue(frame.timed_out)

    def test_timeline_is_deterministic(self):
        commands = [Waypoint(100, 0, vmax_mm_s=100), RotateInPlace(90)]
        first = build_timeline(commands, 1200, 1200, 0)
        second = build_timeline(commands, 1200, 1200, 0)
        self.assertEqual([frame.actual for frame in first], [frame.actual for frame in second])
        self.assertTrue(first[-1].stopped)

    def test_goto_sweep_interpolates_position_and_heading_at_preview_resolution(self):
        sweep = build_goto_sweep(Pose(0, 0, 0), Pose(300, 0, 90), 100, 60, 30)
        self.assertEqual(sweep.poses[0], Pose(0, 0, 0))
        self.assertEqual(sweep.poses[-1], Pose(300, 0, 90))
        self.assertGreater(len(sweep.polygons), 2)
        for first, second in zip(sweep.poses, sweep.poses[1:]):
            self.assertLessEqual(((second.x_mm-first.x_mm)**2 + (second.y_mm-first.y_mm)**2) ** .5, MAX_SAMPLE_DISTANCE_MM + 1e-6)
            self.assertLessEqual(abs(((second.yaw_deg-first.yaw_deg+180) % 360)-180), MAX_SAMPLE_YAW_DEG + 1e-6)

    def test_rotation_sweep_keeps_position_and_limits_heading_samples(self):
        sweep = build_rotation_sweep(Pose(123, 456, 0), 90, 60, 30)
        self.assertEqual(sweep.poses[0], Pose(123, 456, 0))
        self.assertEqual(sweep.poses[-1], Pose(123, 456, 90))
        for first, second in zip(sweep.poses, sweep.poses[1:]):
            self.assertEqual((second.x_mm, second.y_mm), (123, 456))
            self.assertLessEqual(abs(((second.yaw_deg-first.yaw_deg+180) % 360)-180), MAX_SAMPLE_YAW_DEG + 1e-6)

    def test_continuous_timeline_is_geometric_and_does_not_stop_at_intermediate_point(self):
        frames = build_continuous_timeline([PathPosePoint(100, 0, 0), PathPosePoint(200, 0, 90)], 1200, 1200, 0)
        self.assertTrue(frames)
        self.assertEqual(frames[-1].actual, Pose(200, 0, 90))
        self.assertFalse(any(frame.stopped for frame in frames[:-1]))
