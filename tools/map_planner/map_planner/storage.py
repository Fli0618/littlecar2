"""本地 JSON 方案读写。"""

from __future__ import annotations

import json
from pathlib import Path
import re

from .models import Plan

DEFAULT_PLANS_DIR = Path(__file__).resolve().parents[1] / "plans"
_NAME = re.compile(r"^[A-Za-z0-9\u4e00-\u9fff][A-Za-z0-9_\-\u4e00-\u9fff]{0,63}$")


def plan_path(name: str, directory: Path = DEFAULT_PLANS_DIR) -> Path:
    if not _NAME.fullmatch(name):
        raise ValueError("方案名称仅可使用中文、字母、数字、下划线和连字符，最多 64 个字符")
    return directory / f"{name}.json"


def save_plan(plan: Plan, name: str | None = None, directory: Path = DEFAULT_PLANS_DIR) -> Path:
    if name:
        plan.name = name
    path = plan_path(plan.name, directory)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(plan.to_dict(), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def rename_plan(old_name: str, new_name: str, directory: Path = DEFAULT_PLANS_DIR) -> Path:
    """Rename a saved plan and update the name stored inside its JSON document."""

    old_path = plan_path(old_name, directory)
    new_path = plan_path(new_name, directory)
    if not old_path.exists():
        raise ValueError("原方案不存在")
    if old_path != new_path and new_path.exists():
        raise ValueError("目标方案名称已存在")
    plan = load_plan(old_name, directory)
    save_plan(plan, new_name, directory)
    if old_path != new_path:
        old_path.unlink()
    return new_path


def load_plan(name: str, directory: Path = DEFAULT_PLANS_DIR) -> Plan:
    try:
        return Plan.from_dict(json.loads(plan_path(name, directory).read_text(encoding="utf-8")))
    except json.JSONDecodeError as error:
        raise ValueError("方案 JSON 无法解析") from error


def list_plans(directory: Path = DEFAULT_PLANS_DIR) -> list[str]:
    return sorted(path.stem for path in directory.glob("*.json") if _NAME.fullmatch(path.stem)) if directory.exists() else []
