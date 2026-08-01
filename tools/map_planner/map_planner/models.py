"""规划工具的数据模型；所有距离单位均为 mm，航向单位为度。"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Literal


FIELD_SIZE_MM = 2400.0
CAR_SIZE_MM = 300.0
MAP_VERSION = 1


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
    # GOTO Pose 默认只约束位置；拖动旋转手柄后才约束航向。
    stop: bool = True
    dwell_s: float = 0.5
    name: str = ""
    use_position: bool = True
    use_yaw: bool = False
    vmax_mm_s: float = 200.0
    wmax_deg_s: float = 90.0
    timeout_s: float = 15.0


@dataclass
class Segment:
    kind: Literal["bezier", "arc"] = "bezier"
    handle_length_mm: float = 180.0
    arc_radius_mm: float = 300.0
    clockwise: bool = False


@dataclass
class SimulationSettings:
    kp_pos: float = 1.28
    ki_pos: float = 0.13
    kd_pos: float = 0.72
    kp_yaw: float = 1.65
    ki_yaw: float = 1.0
    kd_yaw: float = 0.65
    vmax_mm_s: float = 200.0
    wmax_deg_s: float = 90.0
    linear_response_s: float = 0.18
    yaw_response_s: float = 0.14
    sensor_delay_s: float = 0.04
    sensor_noise_mm: float = 0.0
    lookahead_mm: float = 80.0
    dt_s: float = 0.02


@dataclass
class Plan:
    name: str = "未命名方案"
    created_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    updated_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    start_paper_x_mm: float = 2250.0
    start_paper_y_mm: float = 150.0
    start_heading_deg: float = 180.0
    waypoints: list[Waypoint] = field(default_factory=list)
    segments: list[Segment] = field(default_factory=list)
    settings: SimulationSettings = field(default_factory=SimulationSettings)

    def normalize(self) -> None:
        expected = max(0, len(self.waypoints) - 1)
        self.segments = (self.segments + [Segment() for _ in range(expected)])[:expected]
        self.updated_at = datetime.now(timezone.utc).isoformat()

    def to_dict(self) -> dict[str, object]:
        self.normalize()
        return {"map_version": MAP_VERSION, **asdict(self)}

    @classmethod
    def from_dict(cls, value: dict[str, object]) -> "Plan":
        if value.get("map_version") != MAP_VERSION:
            raise ValueError("不支持的地图方案版本")
        try:
            plan = cls(
                name=str(value["name"]), created_at=str(value["created_at"]), updated_at=str(value["updated_at"]),
                start_paper_x_mm=float(value["start_paper_x_mm"]), start_paper_y_mm=float(value["start_paper_y_mm"]),
                start_heading_deg=float(value["start_heading_deg"]),
                waypoints=[Waypoint(**item) for item in value.get("waypoints", [])],
                segments=[Segment(**item) for item in value.get("segments", [])],
                settings=SimulationSettings(**value.get("settings", {})),
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("方案 JSON 格式无效") from error
        plan.normalize()
        return plan
