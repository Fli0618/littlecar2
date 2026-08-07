"""Data models for Motion Studio 0806."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from enum import Enum, auto
from typing import Any, List, Optional, Tuple


class ControlMode(Enum):
    """Subsystem control mode."""
    NONE = "NONE"
    HOLONOMIC = "HOLONOMIC"
    WORLD_PATH = "WORLD_PATH"
    CLASSIC_PID = "CLASSIC_PID"


@dataclass
class TargetPose:
    """Single point target pose (x, y, yaw)."""
    x_mm: float = 0.0
    y_mm: float = 0.0
    yaw_deg: float = 0.0
    v_max_mm_s: float = 800.0
    w_max_deg_s: float = 120.0
    use_position: bool = True
    use_yaw: bool = True


@dataclass
class HolonomicParams:
    """12 runtime parameters for Holonomic position controller."""
    max_acc_x: float = 1000.0
    max_acc_y: float = 1000.0
    max_acc_yaw: float = 180.0
    kp_x: float = 2.0
    kv_x: float = 1.0
    kp_y: float = 2.0
    kv_y: float = 1.0
    kp_yaw: float = 2.0
    kv_yaw: float = 1.0
    scale_x: float = 1.0
    scale_y: float = 1.0
    scale_yaw: float = 1.0


@dataclass
class PathWaypoint:
    """Single waypoint along a path trajectory."""
    x_mm: float
    y_mm: float
    yaw_deg: float = 0.0
    v_max_mm_s: float = 800.0
    lock_yaw: bool = True

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> PathWaypoint:
        return cls(**data)


@dataclass
class PathTemplate:
    """A named path template that can be saved/loaded for benchmark testing."""
    name: str
    description: str = ""
    waypoints: List[PathWaypoint] = field(default_factory=list)

    def to_json(self) -> str:
        return json.dumps({
            "name": self.name,
            "description": self.description,
            "waypoints": [wp.to_dict() for wp in self.waypoints]
        }, indent=2, ensure_ascii=False)

    @classmethod
    def from_json(cls, json_str: str) -> PathTemplate:
        data = json.loads(json_str)
        wps = [PathWaypoint.from_dict(w) for w in data.get("waypoints", [])]
        return cls(name=data.get("name", "Unnamed"), description=data.get("description", ""), waypoints=wps)


@dataclass
class Obstacle:
    """Obstacle shape on map."""
    id: str
    shape: str  # "rect" or "circle"
    x_mm: float
    y_mm: float
    width_mm: float  # width for rect, radius for circle
    height_mm: float = 0.0  # height for rect
    name: str = "Obstacle"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class TelemetryFrame:
    """Realtime telemetry state from car."""
    timestamp: float = 0.0
    x_mm: float = 0.0
    y_mm: float = 0.0
    yaw_deg: float = 0.0
    v_actual_mm_s: float = 0.0
    v_target_mm_s: float = 0.0
    cross_track_error_mm: float = 0.0
    active_mode: str = "NONE"
