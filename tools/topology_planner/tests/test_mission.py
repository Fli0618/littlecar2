import unittest
from unittest.mock import patch

from topology_planner.mission import (
    TASK_DWELL_S,
    MissionPlanningError,
    build_fixed_mission_plan,
)
from topology_planner.planner import edge_key, find_best_paths


class MissionPlanTests(unittest.TestCase):
    def test_fixed_mission_has_nine_stops_eight_legs_and_actions(self):
        plan = build_fixed_mission_plan()

        self.assertEqual(len(plan.stops), 9)
        self.assertEqual(len(plan.legs), 8)
        self.assertEqual(
            [stop.node_id for stop in plan.stops],
            ["START1", "QR", "RAW", "ROUGH", "BUFFER", "RAW", "ROUGH", "BUFFER", "START1"],
        )
        self.assertEqual(
            [stop.action_id for stop in plan.stops],
            [
                "START_MISSION", "READ_QR_TASK", "PICK_BATCH_1", "ROUGH_BATCH_1",
                "BUFFER_BATCH_1", "PICK_BATCH_2", "ROUGH_BATCH_2", "BUFFER_BATCH_2",
                "FINISH_MISSION",
            ],
        )
        self.assertEqual(
            [stop.action_label for stop in plan.stops],
            [
                "待机并开始任务", "读取二维码任务码并显示", "抓取第一批三个物料",
                "放置并按顺序取回第一批物料", "放置第一批三个物料", "抓取第二批三个物料",
                "放置并按顺序取回第二批物料", "将第二批物料按同色要求码垛",
                "返回启停区并结束任务",
            ],
        )
        self.assertEqual(plan.stops[0].dwell_s, 0.0)
        self.assertEqual(plan.stops[-1].dwell_s, 0.0)
        self.assertTrue(all(stop.dwell_s == TASK_DWELL_S for stop in plan.stops[1:-1]))

    def test_legs_use_single_best_path_and_flatten_only_connection_nodes(self):
        with patch("topology_planner.mission.find_best_paths", wraps=find_best_paths) as mocked:
            plan = build_fixed_mission_plan()

        self.assertEqual(mocked.call_count, 8)
        self.assertTrue(all(call.kwargs["limit"] == 1 for call in mocked.call_args_list))
        expected = []
        for leg in plan.legs:
            if expected and expected[-1] == leg.path.nodes[0]:
                expected.extend(leg.path.nodes[1:])
            else:
                expected.extend(leg.path.nodes)
        self.assertEqual(plan.flattened_nodes, tuple(expected))
        self.assertGreater(plan.flattened_nodes.count("RAW"), 1)

    def test_blocked_edges_are_normalized_and_reports_failed_leg(self):
        plan = build_fixed_mission_plan(blocked_edges=[("N", "START1")])
        self.assertEqual(plan.blocked_edges, frozenset({edge_key("N", "START1")}))
        weighted = build_fixed_mission_plan(distance_weight="2.5")
        self.assertEqual(weighted.distance_weight, 2.5)

        with self.assertRaises(MissionPlanningError) as context:
            build_fixed_mission_plan(blocked_edges=[("START1", "NE")])
        error = context.exception
        self.assertEqual((error.leg_index, error.start_id, error.goal_id), (0, "START1", "QR"))
        self.assertEqual(str(error), error.message)

    def test_start_zone_can_be_selected_and_invalid_zone_is_rejected(self):
        plan = build_fixed_mission_plan("START2")
        self.assertEqual(plan.start_zone, "START2")
        self.assertEqual((plan.stops[0].node_id, plan.stops[-1].node_id), ("START2", "START2"))
        with self.assertRaises(ValueError):
            build_fixed_mission_plan("QR")


if __name__ == "__main__":
    unittest.main()
