"""地图规划工具的数据模型；距离单位为 mm，航向单位为度。"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Literal, Union


FIELD_SIZE_MM = 2400.0
CAR_SIZE_MM = 300.0
MAP_VERSION = 3
StartKind = Literal["zone_1", "zone_2", "custom"]


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
    use_yaw: bool = False
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
class SimulationSettings:
    kp_pos: float = 1.28
    ki_pos: float = 0.13
    kd_pos: float = 0.72
    kp_yaw: float = 1.65
    ki_yaw: float = 1.0
    kd_yaw: float = 0.65
    linear_response_s: float = 0.18
    yaw_response_s: float = 0.14
    sensor_delay_s: float = 0.04
    sensor_noise_mm: float = 0.0
    dt_s: float = 0.02


@dataclass
class Plan:
    name: str = "未命名方案"
    created_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    updated_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    start_kind: StartKind = "zone_1"
    start_paper_x_mm: float = 2250.0
    start_paper_y_mm: float = 150.0
    start_heading_deg: float = 180.0
    waypoints: list[MotionCommand] = field(default_factory=list)
    settings: SimulationSettings = field(default_factory=SimulationSettings)

    def normalize(self) -> None:
        self.updated_at = datetime.now(timezone.utc).isoformat()

    def to_dict(self) -> dict[str, object]:
        self.normalize()
        commands = [
            {"type": "goto_pose", **asdict(command)} if isinstance(command, Waypoint)
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
            "commands": commands,
            "settings": asdict(self.settings),
        }

    @classmethod
    def from_dict(cls, value: dict[str, object]) -> "Plan":
        if not isinstance(value, dict):
            raise ValueError("方案 JSON 格式无效")
        if value.get("map_version") != MAP_VERSION:
            raise ValueError("不支持的地图方案版本")
        try:
            start = value["start"]
            commands = value.get("commands", [])
            if not isinstance(start, dict) or not isinstance(commands, list):
                raise TypeError
            waypoints: list[MotionCommand] = []
            for command in commands:
                if not isinstance(command, dict):
                    raise ValueError
                fields = {key: item for key, item in command.items() if key != "type"}
                if command.get("type") == "goto_pose": waypoints.append(Waypoint(**fields))
                elif command.get("type") == "rotate_in_place": waypoints.append(RotateInPlace(**fields))
                else: raise ValueError
            kind = str(start["kind"])
            if kind not in ("zone_1", "zone_2", "custom"):
                raise ValueError
            return cls(
                name=str(value["name"]),
                created_at=str(value["created_at"]),
                updated_at=str(value["updated_at"]),
                start_kind=kind,  # type: ignore[arg-type]
                start_paper_x_mm=float(start["paper_x_mm"]),
                start_paper_y_mm=float(start["paper_y_mm"]),
                start_heading_deg=float(start["heading_deg"]),
                waypoints=waypoints,
                settings=SimulationSettings(**value.get("settings", {})),
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("方案 JSON 格式无效") from error
