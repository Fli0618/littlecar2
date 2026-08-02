"""地图规划工具的数据模型；距离单位为 mm，航向单位为度。"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Literal, Union


FIELD_SIZE_MM = 2400.0
CAR_SIZE_MM = 300.0
MAP_VERSION = 7
MAP_VERSION_V5 = 5
MAP_VERSION_V6 = 6
StartKind = Literal["zone_1", "zone_2", "custom"]
# 仅用于旧调用方的类型兼容；v7 的持久化语义由 steps 决定。
PlanMode = Literal["stop_point", "continuous"]


@dataclass
class Pose:
    x_mm: float = 0.0
    y_mm: float = 0.0
    yaw_deg: float = 0.0


@dataclass
class Waypoint:
    x_mm: float
    y_mm: float
    yaw_deg: float = 0.0
    stop: bool = True
    dwell_s: float = 0.5
    name: str = ""
    use_yaw: bool = True
    vmax_mm_s: float = 820.0
    wmax_deg_s: float = 90.0
    timeout_s: float = 15.0


@dataclass
class RotateInPlace:
    yaw_deg: float = 0.0
    wmax_deg_s: float = 90.0
    timeout_s: float = 15.0
    name: str = ""


@dataclass
class PathPosePoint:
    x_mm: float
    y_mm: float
    yaw_deg: float = 0.0
    name: str = ""


@dataclass
class ContinuousPathSegment:
    """一次连续跟踪的路径；第一个点必须是该段的入口点。"""

    points: list[PathPosePoint] = field(default_factory=list)
    name: str = ""


MotionStep = Union[Waypoint, RotateInPlace, ContinuousPathSegment]
# 保留旧名称，供外部类型标注逐步迁移。
MotionCommand = MotionStep


@dataclass
class Obstacle:
    paper_x_mm: float
    paper_y_mm: float


@dataclass
class MapLayout:
    obstacles: list[Obstacle] = field(default_factory=list)
    raw_center_x_mm: float = 1200.0
    qr_center_y_mm: float = 1200.0


@dataclass
class Plan:
    name: str = "未命名方案"
    created_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    updated_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    start_kind: StartKind = "zone_1"
    start_paper_x_mm: float = 2250.0
    start_paper_y_mm: float = 150.0
    start_heading_deg: float = 180.0
    steps: list[MotionStep] = field(default_factory=list)
    layout: MapLayout = field(default_factory=MapLayout)
    migration_warnings: list[str] = field(default_factory=list, repr=False, compare=False)

    # 临时兼容旧 GUI：v7 的持久化和代码生成只使用 steps。
    @property
    def mode(self) -> PlanMode:
        return "stop_point"

    @property
    def waypoints(self) -> list[MotionStep]:
        return self.steps

    @property
    def path_points(self) -> list[PathPosePoint]:
        return []

    def normalize(self) -> None:
        self.updated_at = datetime.now(timezone.utc).isoformat()

    def to_dict(self) -> dict[str, object]:
        self.normalize()
        serialized_steps: list[dict[str, object]] = []
        for step in self.steps:
            if isinstance(step, Waypoint):
                serialized_steps.append({"type": "goto_pose", **asdict(step)})
            elif isinstance(step, RotateInPlace):
                serialized_steps.append({"type": "rotate_in_place", **asdict(step)})
            elif isinstance(step, ContinuousPathSegment):
                serialized_steps.append({"type": "continuous_path", "name": step.name, "points": [asdict(point) for point in step.points]})
            else:
                raise ValueError("方案包含不支持的步骤类型")
        return {
            "map_version": MAP_VERSION, "name": self.name, "created_at": self.created_at,
            "updated_at": self.updated_at,
            "start": {"kind": self.start_kind, "paper_x_mm": self.start_paper_x_mm,
                      "paper_y_mm": self.start_paper_y_mm, "heading_deg": self.start_heading_deg},
            "steps": serialized_steps,
            "layout": {"obstacles": [asdict(obstacle) for obstacle in self.layout.obstacles],
                       "raw_center_x_mm": self.layout.raw_center_x_mm, "qr_center_y_mm": self.layout.qr_center_y_mm},
        }

    @classmethod
    def from_dict(cls, value: dict[str, object]) -> "Plan":
        if not isinstance(value, dict):
            raise ValueError("方案 JSON 格式无效")
        warnings: list[str] = []
        version = value.get("map_version")
        if version == MAP_VERSION_V5:
            value, warnings = migrate_v5_plan(value)
        elif version == MAP_VERSION_V6:
            value, warnings = migrate_v6_plan(value)
        if value.get("map_version") != MAP_VERSION:
            raise ValueError("不支持的地图方案版本")
        try:
            start, steps_value, layout = value["start"], value.get("steps", []), value["layout"]
            if not isinstance(start, dict) or not isinstance(steps_value, list) or not isinstance(layout, dict):
                raise TypeError
            kind = str(start["kind"])
            if kind not in ("zone_1", "zone_2", "custom"):
                raise ValueError
            steps: list[MotionStep] = []
            for item in steps_value:
                if not isinstance(item, dict): raise ValueError
                step_type = item.get("type")
                fields = {key: field_value for key, field_value in item.items() if key != "type"}
                if step_type == "goto_pose": steps.append(Waypoint(**fields))
                elif step_type == "rotate_in_place": steps.append(RotateInPlace(**fields))
                elif step_type == "continuous_path":
                    points = fields.pop("points", [])
                    if not isinstance(points, list) or not all(isinstance(point, dict) for point in points): raise ValueError
                    steps.append(ContinuousPathSegment(points=[PathPosePoint(**point) for point in points], **fields))
                else: raise ValueError
            obstacles = layout.get("obstacles", [])
            if not isinstance(obstacles, list) or not all(isinstance(item, dict) for item in obstacles): raise ValueError
            return cls(name=str(value["name"]), created_at=str(value["created_at"]), updated_at=str(value["updated_at"]),
                start_kind=kind, start_paper_x_mm=float(start["paper_x_mm"]), start_paper_y_mm=float(start["paper_y_mm"]),
                start_heading_deg=float(start["heading_deg"]), steps=steps,
                layout=MapLayout(obstacles=[Obstacle(float(item["paper_x_mm"]), float(item["paper_y_mm"])) for item in obstacles],
                    raw_center_x_mm=float(layout["raw_center_x_mm"]), qr_center_y_mm=float(layout["qr_center_y_mm"])), migration_warnings=warnings)
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("方案 JSON 格式无效") from error


def _migrate_legacy(value: dict[str, object], version: int) -> tuple[dict[str, object], list[str]]:
    if value.get("map_version") != version: raise ValueError(f"仅支持迁移 map_version: {version} 的方案")
    steps = value.get("commands", []) if value.get("mode", "stop_point") == "stop_point" else [{"type": "continuous_path", "points": value.get("path_points", [])}]
    migrated = {key: item for key, item in value.items() if key not in ("mode", "commands", "path_points")}
    migrated.update({"map_version": MAP_VERSION, "steps": steps})
    return migrated, [f"已从 map_version: {version} 迁移为统一步骤流程。"]


def migrate_v5_plan(value: dict[str, object]) -> tuple[dict[str, object], list[str]]:
    return _migrate_legacy(value, MAP_VERSION_V5)


def migrate_v6_plan(value: dict[str, object]) -> tuple[dict[str, object], list[str]]:
    return _migrate_legacy(value, MAP_VERSION_V6)


def convert_plan_mode(plan: Plan, mode: PlanMode) -> None:
    """旧 GUI 的兼容入口；v7 新流程不再以全局模式切换。"""
    if mode != "stop_point":
        raise ValueError("请在流程列表中新增连续路径段，而非切换全局模式。")
