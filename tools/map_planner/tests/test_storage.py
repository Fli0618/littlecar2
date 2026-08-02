import json
import tempfile
import unittest
from pathlib import Path

from map_planner.models import MAP_VERSION, Obstacle, Plan, RotateInPlace, Waypoint
from map_planner.storage import list_plans, load_plan, rename_plan, save_plan


class StorageTests(unittest.TestCase):
    def test_save_load_plan_v5(self):
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
            plan.layout.obstacles.append(Obstacle(500, 600))
            plan.layout.raw_center_x_mm = 1250
            plan.layout.qr_center_y_mm = 1150
            path = save_plan(plan, directory=root)
            raw = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(raw["map_version"], MAP_VERSION)
            self.assertEqual(raw["start"], {"kind": "custom", "paper_x_mm": 123, "paper_y_mm": 456, "heading_deg": 37})
            self.assertEqual(raw["commands"][0]["type"], "goto_pose")
            self.assertEqual(raw["commands"][1]["type"], "rotate_in_place")
            self.assertNotIn("segments", raw)
            self.assertNotIn("settings", raw)
            self.assertEqual(raw["layout"]["obstacles"], [{"paper_x_mm": 500, "paper_y_mm": 600}])

            loaded = load_plan("测试方案", root)
            self.assertEqual(loaded.start_kind, "custom")
            self.assertTrue(loaded.waypoints[0].use_yaw)
            self.assertIsInstance(loaded.waypoints[1], RotateInPlace)
            self.assertEqual(loaded.layout, plan.layout)
            self.assertEqual(list_plans(root), ["测试方案"])

    def test_v1_is_rejected_instead_of_migrated(self):
        with self.assertRaisesRegex(ValueError, "版本"):
            Plan.from_dict({"map_version": 1})

    def test_v2_is_rejected_instead_of_migrated(self):
        with self.assertRaisesRegex(ValueError, "版本"):
            Plan.from_dict({"map_version": 2})

    def test_v3_is_rejected_instead_of_migrated(self):
        with self.assertRaisesRegex(ValueError, "版本"):
            Plan.from_dict({"map_version": 3})

    def test_v4_is_rejected_instead_of_migrated(self):
        with self.assertRaisesRegex(ValueError, "版本"):
            Plan.from_dict({"map_version": 4})

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

    def test_rename_plan_moves_file_and_updates_document_name(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            save_plan(Plan(name="旧方案"), directory=root)
            renamed = rename_plan("旧方案", "新方案", directory=root)
            self.assertEqual(renamed.name, "新方案.json")
            self.assertFalse((root / "旧方案.json").exists())
            self.assertEqual(load_plan("新方案", root).name, "新方案")
            self.assertEqual(list_plans(root), ["新方案"])

    def test_rename_plan_rejects_existing_target(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            save_plan(Plan(name="旧方案"), directory=root)
            save_plan(Plan(name="新方案"), directory=root)
            with self.assertRaisesRegex(ValueError, "存在"):
                rename_plan("旧方案", "新方案", directory=root)
