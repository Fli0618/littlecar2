import unittest

from map_planner.models import RotateInPlace, Waypoint
from map_planner.sim import (
    ANGULAR_ACCELERATION_DEG_S2,
    DT_S,
    LINEAR_ACCELERATION_MM_S2,
    Simulation,
    build_timeline,
)


class SimulationTests(unittest.TestCase):
    def test_linear_motion_accelerates_then_brakes_at_fixed_rate(self):
        simulation = Simulation([Waypoint(1000, 0, vmax_mm_s=100, timeout_s=30)])
        simulation.step()
        self.assertAlmostEqual(simulation.vx, LINEAR_ACCELERATION_MM_S2 * DT_S)
        for _ in range(20):
            simulation.step()
        self.assertLessEqual(abs(simulation.vx), 100)
        for _ in range(1000):
            simulation.step()
            if simulation.finished:
                break
        self.assertTrue(simulation.finished)
        self.assertEqual((simulation.actual.x_mm, simulation.actual.y_mm), (1000, 0))
        self.assertEqual((simulation.vx, simulation.vy), (0, 0))

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
