"""地图规划工具的数据模型；距离单位为 mm，航向单位为度。"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Literal, Union


FIELD_SIZE_MM = 2400.0
CAR_SIZE_MM = 300.0
MAP_VERSION = 10
StartKind = Literal["zone_1", "zone_2", "custom"]
# 仅用于旧调用方的类型兼容；v7 的持久化语义由 steps 决定。


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
class AutoSegmentSettings:
    """Parameters owned by one generated navigation segment."""

    corner_radius_mm: float = 120.0
    sample_spacing_mm: float = 20.0
    terminal_straight_mm: float = 300.0
    yaw_mode: str = "fixed"
    goal_yaw_deg: float = 0.0
    strategy: str = "auto"


@dataclass
class ContinuousPathSegment:
    """一次连续跟踪的路径；第一个点必须是该段的入口点。"""

    points: list[PathPosePoint] = field(default_factory=list)
    name: str = ""
    auto_settings: AutoSegmentSettings | None = None


BezierYawMode = Literal["interpolate", "tangent", "fixed"]


@dataclass
class BezierPathSegment:
    control_1_x_mm: float
    control_1_y_mm: float
    control_2_x_mm: float
    control_2_y_mm: float
    end_x_mm: float
    end_y_mm: float
    end_yaw_deg: float
    yaw_mode: BezierYawMode = "interpolate"
    sample_spacing_mm: float = 20.0
    name: str = ""


@dataclass
class StepTurnNode:
    """User control point for a step-turn path."""

    x_mm: float
    y_mm: float
    name: str = ""


@dataclass
class StepTurnPathSegment:
    """Polyline expanded into an executable continuous path by the compiler."""

    route_points: list[StepTurnNode] = field(default_factory=list)
    step_distance_mm: float = 60.0
    name: str = ""


MotionStep = Union[Waypoint, RotateInPlace, ContinuousPathSegment, BezierPathSegment, StepTurnPathSegment]


@dataclass
class Obstacle:
    paper_x_mm: float
    paper_y_mm: float


@dataclass
class CostmapSettings:
    """Static-map hard clearance and soft inflation, grouped by object type."""

    vehicle_length_mm: float = 300.0
    vehicle_width_mm: float = 300.0
    boundary_safety_margin_mm: float = 0.0
    boundary_inflation_mm: float = 120.0
    boundary_cost_weight: float = 3.0
    platform_safety_margin_mm: float = 20.0
    platform_inflation_mm: float = 80.0
    platform_cost_weight: float = 3.0
    obstacle_radius_mm: float = 25.0
    obstacle_safety_margin_mm: float = 20.0
    obstacle_inflation_mm: float = 80.0
    obstacle_cost_weight: float = 3.0


@dataclass
class MapLayout:
    obstacles: list[Obstacle] = field(default_factory=list)
    raw_center_x_mm: float = 1200.0
    qr_center_y_mm: float = 1200.0
    costmap: CostmapSettings = field(default_factory=CostmapSettings)


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
                serialized_steps.append({
                    "type": "continuous_path", "name": step.name,
                    "points": [asdict(point) for point in step.points],
                    "auto_settings": (None if step.auto_settings is None
                                      else asdict(step.auto_settings)),
                })
            elif isinstance(step, BezierPathSegment):
                serialized_steps.append({"type": "bezier_path", **asdict(step)})
            elif isinstance(step, StepTurnPathSegment):
                serialized_steps.append({
                    "type": "step_turn_path",
                    "name": step.name,
                    "step_distance_mm": step.step_distance_mm,
                    "route_points": [asdict(point) for point in step.route_points],
                })
            else:
                raise ValueError("方案包含不支持的步骤类型")
        return {
            "map_version": MAP_VERSION, "name": self.name, "created_at": self.created_at,
            "updated_at": self.updated_at,
            "start": {"kind": self.start_kind, "paper_x_mm": self.start_paper_x_mm,
                      "paper_y_mm": self.start_paper_y_mm, "heading_deg": self.start_heading_deg},
            "steps": serialized_steps,
            "layout": {"obstacles": [asdict(obstacle) for obstacle in self.layout.obstacles],
                       "raw_center_x_mm": self.layout.raw_center_x_mm,
                       "qr_center_y_mm": self.layout.qr_center_y_mm,
                       "costmap": asdict(self.layout.costmap)},
        }

    @classmethod
    def from_dict(cls, value: dict[str, object]) -> "Plan":
        if not isinstance(value, dict):
            raise ValueError("方案 JSON 格式无效")
        version = value.get("map_version")
        if version not in (7, 8, 9, MAP_VERSION):
            raise ValueError("仅支持 map_version 7、8、9 或 10 的流程方案")
        allowed_keys = {"map_version", "name", "created_at", "updated_at", "start", "steps", "layout"}
        if set(value) != allowed_keys:
            raise ValueError("方案 JSON 格式无效")
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
                if step_type == "goto_pose":
                    step = Waypoint(**fields)
                    if version in (7, 8): step.x_mm, step.y_mm = step.y_mm, step.x_mm
                    steps.append(step)
                elif step_type == "rotate_in_place": steps.append(RotateInPlace(**fields))
                elif step_type == "continuous_path":
                    points = fields.pop("points", [])
                    if not isinstance(points, list) or not all(isinstance(point, dict) for point in points): raise ValueError
                    auto_settings_value = fields.pop("auto_settings", None)
                    if auto_settings_value is not None and not isinstance(auto_settings_value, dict):
                        raise ValueError
                    path_points = [PathPosePoint(**point) for point in points]
                    if version in (7, 8):
                        for point in path_points: point.x_mm, point.y_mm = point.y_mm, point.x_mm
                    steps.append(ContinuousPathSegment(
                        points=path_points,
                        auto_settings=(None if auto_settings_value is None
                                       else AutoSegmentSettings(**auto_settings_value)),
                        **fields))
                elif step_type == "bezier_path" and version in (8, 9, MAP_VERSION):
                    step = BezierPathSegment(**fields)
                    if version == 8:
                        step.control_1_x_mm, step.control_1_y_mm = step.control_1_y_mm, step.control_1_x_mm
                        step.control_2_x_mm, step.control_2_y_mm = step.control_2_y_mm, step.control_2_x_mm
                        step.end_x_mm, step.end_y_mm = step.end_y_mm, step.end_x_mm
                    steps.append(step)
                elif step_type == "step_turn_path" and version == MAP_VERSION:
                    route_points = fields.pop("route_points", [])
                    if not isinstance(route_points, list) or not all(isinstance(point, dict) for point in route_points):
                        raise ValueError
                    steps.append(StepTurnPathSegment(
                        route_points=[StepTurnNode(**point) for point in route_points], **fields))
                else: raise ValueError
            obstacles = layout.get("obstacles", [])
            if not isinstance(obstacles, list) or not all(isinstance(item, dict) for item in obstacles): raise ValueError
            costmap_value = layout.get("costmap", {})
            if not isinstance(costmap_value, dict): raise ValueError
            costmap = CostmapSettings(**costmap_value) if costmap_value else CostmapSettings()
            return cls(name=str(value["name"]), created_at=str(value["created_at"]), updated_at=str(value["updated_at"]),
                start_kind=kind, start_paper_x_mm=float(start["paper_x_mm"]), start_paper_y_mm=float(start["paper_y_mm"]),
                start_heading_deg=float(start["heading_deg"]), steps=steps,
                layout=MapLayout(obstacles=[Obstacle(float(item["paper_x_mm"]), float(item["paper_y_mm"])) for item in obstacles],
                    raw_center_x_mm=float(layout["raw_center_x_mm"]), qr_center_y_mm=float(layout["qr_center_y_mm"]),
                    costmap=costmap))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("方案 JSON 格式无效") from error
