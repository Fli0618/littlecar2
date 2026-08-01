import unittest

from map_planner.models import Pose, SimulationSettings
from map_planner.sim import Simulation


class SimulationTests(unittest.TestCase):
    def test_pid_simulation_moves_towards_route(self):
        simulation = Simulation([Pose(0, 0, 0), Pose(0, 500, 0)], [], SimulationSettings())
        frame = simulation.step()
        for _ in range(120): frame = simulation.step()
        self.assertGreater(frame.actual.y_mm, 0); self.assertLess(frame.error_mm, 500)

    def test_stop_index_creates_dwell(self):
        settings = SimulationSettings(vmax_mm_s=1000, linear_response_s=.01)
        simulation = Simulation([Pose(0, 0, 0), Pose(0, 20, 0), Pose(0, 200, 0)], [1], settings)
        for _ in range(20): simulation.step()
        self.assertGreaterEqual(simulation.dwell_remaining_s, 0)

    def test_out_of_bounds_uses_start_marker(self):
        simulation = Simulation([Pose(0, 0), Pose(0, 100)], [], SimulationSettings(), 100, 100, 0)
        self.assertTrue(simulation.step().out_of_bounds)
