import unittest

from topology_planner.mission import build_fixed_mission_plan
from topology_planner.simulation import MissionSimulator, SimulationPhase


class MissionSimulatorTests(unittest.TestCase):
    def setUp(self):
        self.plan = build_fixed_mission_plan()

    def test_initial_start_and_segment_progress(self):
        simulator = MissionSimulator(self.plan)
        initial = simulator.snapshot()
        self.assertEqual(initial.phase, SimulationPhase.IDLE)
        self.assertEqual(initial.current_node, "START1")

        simulator.start()
        frame = simulator.tick(0.75)
        self.assertEqual(frame.phase, SimulationPhase.TRAVEL)
        self.assertEqual((frame.from_node, frame.to_node), ("NE", "E"))
        self.assertAlmostEqual(frame.edge_progress, 0.25)
        self.assertAlmostEqual(frame.traveled_distance, 0.75)

    def test_task_dwell_and_large_time_step(self):
        simulator = MissionSimulator(self.plan)
        simulator.start()
        first_leg_distance = self.plan.legs[0].path.distance
        at_qr = simulator.tick(first_leg_distance)
        self.assertEqual(at_qr.phase, SimulationPhase.DWELL)
        self.assertEqual(at_qr.current_stop_index, 1)
        self.assertEqual(at_qr.current_node, "QR")
        self.assertAlmostEqual(at_qr.dwell_remaining_s, 0.8)

        during_dwell = simulator.tick(0.3)
        self.assertAlmostEqual(during_dwell.dwell_remaining_s, 0.5)
        self.assertEqual(during_dwell.phase, SimulationPhase.DWELL)

        finished = simulator.tick(100.0)
        self.assertEqual(finished.phase, SimulationPhase.FINISHED)
        self.assertTrue(finished.finished)
        self.assertAlmostEqual(finished.traveled_distance, self.plan.total_distance)
        self.assertEqual((finished.current_stop_index, finished.current_node), (8, "START1"))

    def test_pause_resume_validation_and_replay(self):
        simulator = MissionSimulator(self.plan)
        with self.assertRaises(ValueError):
            simulator.tick(-0.01)
        with self.assertRaises(ValueError):
            simulator.set_speed_multiplier(0.0)
        with self.assertRaises(ValueError):
            MissionSimulator(self.plan, base_speed=0.0)

        simulator.start()
        simulator.tick(0.5)
        paused = simulator.pause()
        self.assertEqual(paused.phase, SimulationPhase.PAUSED)
        self.assertTrue(paused.paused)
        unchanged = simulator.tick(1.0)
        self.assertAlmostEqual(unchanged.traveled_distance, 0.5)
        self.assertEqual(simulator.resume().phase, SimulationPhase.TRAVEL)

        simulator.tick(100.0)
        replay = simulator.start()
        self.assertEqual(replay.phase, SimulationPhase.TRAVEL)
        self.assertEqual(replay.current_stop_index, 0)
        self.assertEqual(replay.current_node, "START1")
        self.assertEqual(replay.leg_index, 0)


if __name__ == "__main__":
    unittest.main()
