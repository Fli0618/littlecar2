import unittest

from topology_planner.planner import edge_key, edges, find_best_paths, nodes


class PlannerTests(unittest.TestCase):
    def test_fixed_topology_counts_and_lengths(self):
        self.assertEqual(len(nodes), 15)
        self.assertEqual(sum(node.kind == "navigation" for node in nodes.values()), 9)
        self.assertEqual(sum(node.kind == "task" for node in nodes.values()), 6)
        self.assertEqual(len(edges), 18)
        self.assertEqual(sum(edge.length == 1.0 for edge in edges), 12)
        self.assertEqual(sum(edge.length == 0.5 for edge in edges), 6)

    def test_task_nodes_are_leaves(self):
        degrees = {node_id: 0 for node_id in nodes}
        for edge in edges:
            degrees[edge.node_a] += 1
            degrees[edge.node_b] += 1
        self.assertTrue(all(degrees[node_id] == 1 for node_id, node in nodes.items() if node.kind == "task"))

    def test_straight_path_has_no_turn_or_stop(self):
        result = find_best_paths("NW", "NE")[0]
        self.assertEqual(result.nodes, ("NW", "N", "NE"))
        self.assertEqual((result.quarter_turns, result.stops), (0, 0))

    def test_ninety_and_one_eighty_turns(self):
        ninety = find_best_paths("NW", "C")[0]
        self.assertEqual((ninety.quarter_turns, ninety.stops), (1, 1))
        u_turn = find_best_paths("START1", "START1")[0]
        self.assertEqual((u_turn.quarter_turns, u_turn.stops), (0, 0))

    def test_block_and_restore_edge(self):
        key = edge_key("N", "C")
        self.assertTrue(find_best_paths("RAW", "ROUGH"))
        blocked = find_best_paths("RAW", "ROUGH", {key})
        self.assertTrue(all(key not in {edge_key(a, b) for a, b in zip(path.nodes, path.nodes[1:])} for path in blocked))
        self.assertTrue(find_best_paths("RAW", "ROUGH", set()))

    def test_task_nodes_are_only_endpoints(self):
        results = find_best_paths("START1", "ROUGH")
        self.assertTrue(results)
        for result in results:
            self.assertTrue(all(nodes[node_id].kind == "navigation" for node_id in result.nodes[1:-1]))

    def test_task_and_navigation_endpoints_and_same_node(self):
        self.assertTrue(find_best_paths("START1", "NE"))
        self.assertTrue(find_best_paths("NW", "ROUGH"))
        same = find_best_paths("C", "C")[0]
        self.assertEqual(same.nodes, ("C",))
        self.assertEqual(same.total_cost, 0.0)

    def test_default_start1_to_rough_cost(self):
        results = find_best_paths("START1", "ROUGH")
        expected = ("START1", "NE", "N", "C", "S", "ROUGH")
        result = next(item for item in results if item.nodes == expected)
        self.assertAlmostEqual(result.distance, 4.0)
        self.assertEqual((result.quarter_turns, result.stops), (1, 1))
        self.assertAlmostEqual(result.total_cost, 5.75)

    def test_raw_to_qr_center_route_has_fewer_turns(self):
        results = find_best_paths("RAW", "QR")
        self.assertEqual(results[0].nodes, ("RAW", "N", "C", "E", "QR"))
        self.assertEqual(results[0].quarter_turns, 1)

    def test_weight_changes_order_and_limit_is_four(self):
        default = find_best_paths("START1", "ROUGH")
        turn_heavy = find_best_paths("START1", "ROUGH", turn_weight=10.0)
        self.assertGreater(turn_heavy[0].total_cost, default[0].total_cost)
        self.assertLessEqual(len(default), 4)
        self.assertEqual(len({item.nodes for item in default}), len(default))

    def test_no_route_and_stable_tie_order(self):
        all_edges = {edge_key(edge.node_a, edge.node_b) for edge in edges}
        self.assertEqual(find_best_paths("START1", "ROUGH", all_edges), [])
        first = find_best_paths("NW", "SE", distance_weight=0, turn_weight=0, stop_weight=0)
        second = find_best_paths("NW", "SE", distance_weight=0, turn_weight=0, stop_weight=0)
        self.assertEqual([item.nodes for item in first], [item.nodes for item in second])


if __name__ == "__main__":
    unittest.main()
