import ast
from collections import Counter
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "map_planner"


def test_planner_window_has_no_duplicate_method_definitions():
    tree = ast.parse((PACKAGE_ROOT / "gui.py").read_text(encoding="utf-8"))
    planner = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "PlannerWindow")
    names = [node.name for node in planner.body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))]

    assert not {name: count for name, count in Counter(names).items() if count > 1}


def test_production_modules_do_not_reference_legacy_flow_api():
    forbidden = (
        "plan.mode",
        "plan.path_points",
        "plan.waypoints",
        "plan.migration_warnings",
        "plan_mode_combo",
        "convert_plan_mode",
        "on_plan_mode_changed",
        "MotionCommand",
        "_legacy_normalize_property",
    )
    source = "\n".join(path.read_text(encoding="utf-8") for path in PACKAGE_ROOT.glob("*.py"))

    assert not [token for token in forbidden if token in source]
