import json
import tempfile
import unittest
from pathlib import Path

from map_planner.models import MAP_VERSION, Plan, RotateInPlace, Waypoint
from map_planner.storage import list_plans, load_plan, save_plan


class StorageTests(unittest.TestCase):
    def test_save_load_plan_v3(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan = Plan(
                name="测试方案",
                start_kind="custom",
                start_paper_x_mm=123,
                start_paper_y_mm=456,
                start_heading_deg=37,
                waypoints=[Waypoint(12, 34, yaw_deg=56, use_yaw=True, stop=True), RotateInPlace(90)],
            )
            plan.settings.kp_pos = 2.5
            plan.settings.ki_pos = 0.25
            plan.settings.kd_pos = 0.5
            plan.settings.kp_yaw = 3.5
            plan.settings.ki_yaw = 0.75
            plan.settings.kd_yaw = 1.25
            path = save_plan(plan, directory=root)
            raw = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(raw["map_version"], MAP_VERSION)
            self.assertEqual(raw["start"], {"kind": "custom", "paper_x_mm": 123, "paper_y_mm": 456, "heading_deg": 37})
            self.assertEqual(raw["commands"][0]["type"], "goto_pose")
            self.assertEqual(raw["commands"][1]["type"], "rotate_in_place")
            self.assertNotIn("segments", raw)

            loaded = load_plan("测试方案", root)
            self.assertEqual(loaded.start_kind, "custom")
            self.assertTrue(loaded.waypoints[0].use_yaw)
            self.assertIsInstance(loaded.waypoints[1], RotateInPlace)
            self.assertEqual(loaded.settings, plan.settings)
            self.assertEqual(list_plans(root), ["测试方案"])

    def test_v1_is_rejected_instead_of_migrated(self):
        with self.assertRaisesRegex(ValueError, "版本"):
            Plan.from_dict({"map_version": 1})

    def test_v2_is_rejected_instead_of_migrated(self):
        with self.assertRaisesRegex(ValueError, "版本"):
            Plan.from_dict({"map_version": 2})

    def test_unknown_command_type_is_rejected(self):
        value = Plan().to_dict()
        value["commands"] = [{"type": "arc"}]
        with self.assertRaisesRegex(ValueError, "格式"):
            Plan.from_dict(value)

    def test_non_object_plan_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "格式"):
            Plan.from_dict([])  # type: ignore[arg-type]

    def test_rejects_bad_name(self):
        with self.assertRaises(ValueError):
            save_plan(Plan(name="bad name"), directory=Path("."))
