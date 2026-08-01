import tempfile
import unittest
from pathlib import Path

from map_planner.models import Plan, Waypoint
from map_planner.storage import list_plans, load_plan, save_plan


class StorageTests(unittest.TestCase):
    def test_save_load_plan(self):
        with tempfile.TemporaryDirectory() as directory:
            plan = Plan(name="测试方案", waypoints=[Waypoint(12, 34, stop=True)])
            save_plan(plan, directory=Path(directory))
            loaded = load_plan("测试方案", Path(directory))
            self.assertTrue(loaded.waypoints[0].stop); self.assertEqual(list_plans(Path(directory)), ["测试方案"])

    def test_rejects_bad_name(self):
        with self.assertRaises(ValueError): save_plan(Plan(name="bad name"), directory=Path("."))
