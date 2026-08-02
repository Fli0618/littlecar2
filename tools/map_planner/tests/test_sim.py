import unittest

from map_planner.models import SimulationSettings, Waypoint
from map_planner.sim import Simulation, build_timeline


class SimulationTests(unittest.TestCase):
    def test_simulation_moves_towards_first_command(self):
        simulation = Simulation([Waypoint(0, 500, stop=False)], SimulationSettings())
        frame = simulation.step()
        for _ in range(120):
            frame = simulation.step()
        self.assertGreater(frame.actual.y_mm, 0)
        self.assertLess(frame.error_mm, 500)

    def test_each_command_uses_its_own_speed_limit(self):
        settings = SimulationSettings(linear_response_s=0.001, sensor_delay_s=0)
        simulation = Simulation([Waypoint(0, 1000, vmax_mm_s=40, stop=False)], settings)
        frame = simulation.step()
        self.assertLessEqual(frame.speed_mm_s, 40)

    def test_yaw_disabled_preserves_nonzero_heading(self):
        simulation = Simulation([Waypoint(100, 0, yaw_deg=0, use_yaw=False)], SimulationSettings(sensor_delay_s=0))
        simulation.actual.yaw_deg = 73
        for _ in range(20):
            simulation.step()
        self.assertAlmostEqual(simulation.actual.yaw_deg, 73)

    def test_yaw_enabled_uses_node_angular_limit(self):
        settings = SimulationSettings(yaw_response_s=0.001, sensor_delay_s=0)
        simulation = Simulation([Waypoint(0, 0, yaw_deg=90, use_yaw=True, wmax_deg_s=12)], settings)
        simulation.step()
        self.assertLessEqual(abs(simulation.wz), 12)
        self.assertGreater(simulation.actual.yaw_deg, 0)

    def test_stop_converges_then_dwells_before_next_command(self):
        settings = SimulationSettings(dt_s=0.01, linear_response_s=0.01, sensor_delay_s=0)
        commands = [
            Waypoint(0, 100, stop=True, dwell_s=0.1, vmax_mm_s=1000),
            Waypoint(0, 200, stop=False, vmax_mm_s=1000),
        ]
        simulation = Simulation(commands, settings)
        saw_dwell = False
        for _ in range(1000):
            frame = simulation.step()
            if simulation.dwell_remaining_s > 0:
                saw_dwell = True
                self.assertTrue(frame.stopped)
                self.assertEqual(simulation.command_index, 0)
                self.assertLessEqual(frame.error_mm, 25)
                self.assertLessEqual(frame.speed_mm_s, 5)
            if simulation.command_index == 1:
                break
        self.assertTrue(saw_dwell)
        self.assertEqual(simulation.command_index, 1)
        self.assertEqual(simulation.command_elapsed_s, 0)

    def test_timeout_sets_failure_state_and_frame_flag(self):
        settings = SimulationSettings(dt_s=0.02, sensor_delay_s=0)
        simulation = Simulation([Waypoint(0, 1000, vmax_mm_s=1, timeout_s=0.05)], settings)
        for _ in range(4):
            frame = simulation.step()
        self.assertTrue(simulation.failed)
        self.assertFalse(simulation.finished)
        self.assertTrue(frame.timed_out)
        self.assertIn("命令 1", simulation.failure_reason)

    def test_out_of_bounds_uses_start_marker(self):
        simulation = Simulation([Waypoint(0, 100)], SimulationSettings(), 100, 100, 0)
        self.assertTrue(simulation.step().out_of_bounds)

    def test_timeline_is_deterministic_and_stops_at_terminal_frame(self):
        settings = SimulationSettings(dt_s=0.02, sensor_delay_s=0)
        commands = [Waypoint(0, 1000, vmax_mm_s=1, timeout_s=0.05)]
        first = build_timeline(commands, settings, 1200, 1200, 0)
        second = build_timeline(commands, settings, 1200, 1200, 0)
        self.assertGreater(len(first), 0)
        self.assertEqual([frame.actual for frame in first], [frame.actual for frame in second])
        self.assertTrue(first[-1].timed_out)
        self.assertEqual(first[-1].time_s, 0.06)
