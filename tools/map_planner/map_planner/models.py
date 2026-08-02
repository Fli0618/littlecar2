"""地图规划工具的数据模型；距离单位为 mm，航向单位为度。"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Literal, Union


FIELD_SIZE_MM = 2400.0
CAR_SIZE_MM = 300.0
MAP_VERSION = 6
MAP_VERSION_V5 = 5
StartKind = Literal["zone_1", "zone_2", "custom"]
PlanMode = Literal["stop_point", "continuous"]


@dataclass
class Pose:
    x_mm: float = 0.0
    y_mm: float = 0.0
    yaw_deg: float = 0.0


@dataclass
class Waypoint:
    """固定在起点世界坐标系中的一条 GOTO Pose 命令。"""

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
    """在当前位置旋转至绝对世界航向的动作。"""

    yaw_deg: float = 0.0
    wmax_deg_s: float = 90.0
    timeout_s: float = 15.0
    name: str = ""


MotionCommand = Union[Waypoint, RotateInPlace]


@dataclass
class PathPosePoint:
    """连续路径中的一个几何目标位姿，不代表车辆必须在此停止。"""

    x_mm: float
    y_mm: float
    yaw_deg: float = 0.0
    name: str = ""


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
    mode: PlanMode = "stop_point"
    waypoints: list[MotionCommand] = field(default_factory=list)
    path_points: list[PathPosePoint] = field(default_factory=list)
    layout: MapLayout = field(default_factory=MapLayout)
    migration_warnings: list[str] = field(default_factory=list, repr=False, compare=False)

    def normalize(self) -> None:
        self.updated_at = datetime.now(timezone.utc).isoformat()

    def to_dict(self) -> dict[str, object]:
        self.normalize()
        commands = [
            {"type": "goto_pose", **asdict(command)}
            if isinstance(command, Waypoint)
            else {"type": "rotate_in_place", **asdict(command)}
            for command in self.waypoints
        ]
        return {
            "map_version": MAP_VERSION,
            "name": self.name,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "start": {
                "kind": self.start_kind,
                "paper_x_mm": self.start_paper_x_mm,
                "paper_y_mm": self.start_paper_y_mm,
                "heading_deg": self.start_heading_deg,
            },
            "mode": self.mode,
            "commands": commands,
            "path_points": [asdict(point) for point in self.path_points],
            "layout": {
                "obstacles": [asdict(obstacle) for obstacle in self.layout.obstacles],
                "raw_center_x_mm": self.layout.raw_center_x_mm,
                "qr_center_y_mm": self.layout.qr_center_y_mm,
            },
        }

    @classmethod
    def from_dict(cls, value: dict[str, object]) -> "Plan":
        warnings: list[str] = []
        if not isinstance(value, dict):
            raise ValueError("方案 JSON 格式无效")
        if value.get("map_version") == MAP_VERSION_V5:
            value, warnings = migrate_v5_plan(value)
        if value.get("map_version") != MAP_VERSION:
            raise ValueError("不支持的地图方案版本")
        try:
            start = value["start"]
            commands = value.get("commands", [])
            layout = value["layout"]
            path_points = value.get("path_points", [])
            mode = value.get("mode", "stop_point")
            if not isinstance(start, dict) or not isinstance(commands, list) or not isinstance(layout, dict) or not isinstance(path_points, list):
                raise TypeError
            if mode not in ("stop_point", "continuous"):
                raise ValueError
            waypoints: list[MotionCommand] = []
            for command in commands:
                if not isinstance(command, dict):
                    raise ValueError
                fields = {key: item for key, item in command.items() if key != "type"}
                if command.get("type") == "goto_pose":
                    waypoints.append(Waypoint(**fields))
                elif command.get("type") == "rotate_in_place":
                    waypoints.append(RotateInPlace(**fields))
                else:
                    raise ValueError
            kind = str(start["kind"])
            if kind not in ("zone_1", "zone_2", "custom"):
                raise ValueError
            obstacles = layout.get("obstacles", [])
            if not isinstance(obstacles, list) or not all(isinstance(item, dict) for item in obstacles):
                raise ValueError
            return cls(
                name=str(value["name"]),
                created_at=str(value["created_at"]),
                updated_at=str(value["updated_at"]),
                start_kind=kind,  # type: ignore[arg-type]
                start_paper_x_mm=float(start["paper_x_mm"]),
                start_paper_y_mm=float(start["paper_y_mm"]),
                start_heading_deg=float(start["heading_deg"]),
                mode=mode,  # type: ignore[arg-type]
                waypoints=waypoints,
                path_points=[PathPosePoint(**item) for item in path_points if isinstance(item, dict)],
                layout=MapLayout(
                    obstacles=[Obstacle(float(item["paper_x_mm"]), float(item["paper_y_mm"])) for item in obstacles],
                    raw_center_x_mm=float(layout["raw_center_x_mm"]),
                    qr_center_y_mm=float(layout["qr_center_y_mm"]),
                ),
                migration_warnings=warnings,
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("方案 JSON 格式无效") from error


def migrate_v5_plan(value: dict[str, object]) -> tuple[dict[str, object], list[str]]:
    """将 v5 停点方案升级为 v6，且不猜测连续行驶语义。"""

    if value.get("map_version") != MAP_VERSION_V5:
        raise ValueError("仅支持迁移 map_version: 5 的方案")
    migrated = dict(value)
    migrated["map_version"] = MAP_VERSION
    migrated["mode"] = "stop_point"
    migrated["path_points"] = []
    return migrated, ["已从 map_version: 5 迁移为 v6，并按原有停点路径模式加载。"]


def convert_plan_mode(plan: Plan, mode: PlanMode) -> None:
    """在两种编辑语义间明确转换，调用方负责在界面中确认。"""

    if plan.mode == mode:
        return
    if mode == "continuous":
        points: list[PathPosePoint] = []
        x_mm = y_mm = yaw_deg = 0.0
        for action in plan.waypoints:
            if isinstance(action, RotateInPlace):
                yaw_deg = action.yaw_deg
            else:
                x_mm, y_mm = action.x_mm, action.y_mm
                yaw_deg = action.yaw_deg if action.use_yaw else yaw_deg
            points.append(PathPosePoint(x_mm, y_mm, yaw_deg, action.name))
        plan.path_points = points
    else:
        plan.waypoints = [Waypoint(point.x_mm, point.y_mm, point.yaw_deg, name=point.name) for point in plan.path_points]
    plan.mode = mode
