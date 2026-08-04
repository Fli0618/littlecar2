"""垫步路径的唯一纯几何编译器。"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .geometry import wrap_deg
from .models import PathPosePoint, Pose, StepTurnPathSegment


STEP_TURN_DEFAULT_DISTANCE_MM = 60.0
STEP_TURN_MIN_EFFECTIVE_DISTANCE_MM = 35.0
STEP_TURN_MAX_SEGMENT_RATIO = 0.30
STEP_TURN_MIN_ANGLE_DEG = 15.0
STEP_TURN_MAX_ANGLE_DEG = 120.0
STEP_TURN_MAX_PATH_POINTS = 256
_MIN_POINT_DISTANCE_MM = 1.0

# Compatibility aliases kept for callers introduced while the feature was developed.
DEFAULT_STEP_DISTANCE_MM = STEP_TURN_DEFAULT_DISTANCE_MM
MIN_EFFECTIVE_STEP_DISTANCE_MM = STEP_TURN_MIN_EFFECTIVE_DISTANCE_MM
MAX_SEGMENT_RATIO = STEP_TURN_MAX_SEGMENT_RATIO
MIN_ANGLE_DEG = STEP_TURN_MIN_ANGLE_DEG
MAX_ANGLE_DEG = STEP_TURN_MAX_ANGLE_DEG
MAX_PATH_POINTS = STEP_TURN_MAX_PATH_POINTS
DEFAULT_STEP_TURN_DISTANCE_MM = STEP_TURN_DEFAULT_DISTANCE_MM
MIN_EFFECTIVE_STEP_MM = STEP_TURN_MIN_EFFECTIVE_DISTANCE_MM
MAX_STEP_TURN_SEGMENT_RATIO = STEP_TURN_MAX_SEGMENT_RATIO
MIN_STEP_TURN_ANGLE_DEG = STEP_TURN_MIN_ANGLE_DEG
MAX_STEP_TURN_ANGLE_DEG = STEP_TURN_MAX_ANGLE_DEG
MAX_STEP_TURN_PATH_POINTS = STEP_TURN_MAX_PATH_POINTS


@dataclass(frozen=True)
class StepTurnDiagnostic:
    code: str
    message: str
    node_index: int | None = None


@dataclass(frozen=True)
class StepTurnCornerDiagnostic:
    node_index: int
    angle_deg: float
    effective_distance_mm: float | None
    has_step: bool


@dataclass(frozen=True)
class StepTurnCompileResult:
    points: tuple[PathPosePoint, ...]
    diagnostics: tuple[StepTurnDiagnostic, ...]
    corners: tuple[StepTurnCornerDiagnostic, ...] = ()

    @property
    def has_errors(self) -> bool:
        return bool(self.diagnostics)

    @property
    def errors(self) -> tuple[StepTurnDiagnostic, ...]:
        return self.diagnostics

    @property
    def warnings(self) -> tuple[StepTurnDiagnostic, ...]:
        return ()

    @property
    def valid(self) -> bool:
        return not self.has_errors


class StepTurnCompiler:
    """将原始 Start/C/End 节点确定性地展开为 START/A/B/END 路径点。"""

    @staticmethod
    def analyze(start: Pose, segment: StepTurnPathSegment) -> StepTurnCompileResult:
        errors: list[StepTurnDiagnostic] = []
        corners: list[StepTurnCornerDiagnostic] = []
        route = segment.route_points
        if not math.isfinite(segment.step_distance_mm) or segment.step_distance_mm <= 0.0:
            errors.append(StepTurnDiagnostic("invalid_step_distance", "垫步距离必须为有限正数"))
        if not route:
            errors.append(StepTurnDiagnostic("missing_endpoint", "垫步路径至少需要一个终点"))
            return StepTurnCompileResult((), tuple(errors))

        raw = [(start.x_mm, start.y_mm)] + [(node.x_mm, node.y_mm) for node in route]
        if not all(math.isfinite(value) for point in raw for value in point):
            errors.append(StepTurnDiagnostic("non_finite_coordinate", "路径坐标必须为有限数值"))
            return StepTurnCompileResult((), tuple(errors))

        points: list[tuple[float, float]] = [raw[0]]
        for index in range(1, len(raw) - 1):
            previous, corner, following = raw[index - 1], raw[index], raw[index + 1]
            incoming = _unit_vector(previous, corner)
            outgoing = _unit_vector(corner, following)
            node_index = index - 1
            if incoming is None or outgoing is None:
                errors.append(StepTurnDiagnostic("coincident_route_points", "相邻路径点间距必须至少为 1 mm", node_index))
                continue
            angle = math.degrees(math.acos(max(-1.0, min(1.0, incoming[0] * outgoing[0] + incoming[1] * outgoing[1]))))
            if angle > STEP_TURN_MAX_ANGLE_DEG:
                errors.append(StepTurnDiagnostic("turn_angle_too_large", "转角不能大于 120 度", node_index))
                corners.append(StepTurnCornerDiagnostic(node_index, angle, None, False))
                continue
            if angle < STEP_TURN_MIN_ANGLE_DEG:
                points.append(corner)
                corners.append(StepTurnCornerDiagnostic(node_index, angle, None, False))
                continue
            effective = min(segment.step_distance_mm, math.dist(previous, corner) * STEP_TURN_MAX_SEGMENT_RATIO,
                            math.dist(corner, following) * STEP_TURN_MAX_SEGMENT_RATIO)
            if effective < STEP_TURN_MIN_EFFECTIVE_DISTANCE_MM:
                errors.append(StepTurnDiagnostic("effective_step_too_short", "有效垫步距离不能小于 35 mm", node_index))
                corners.append(StepTurnCornerDiagnostic(node_index, angle, effective, False))
                continue
            # C is intent only. The executable path contains A and B, never C.
            points.extend(((corner[0] - effective * incoming[0], corner[1] - effective * incoming[1]),
                           (corner[0] + effective * outgoing[0], corner[1] + effective * outgoing[1])))
            corners.append(StepTurnCornerDiagnostic(node_index, angle, effective, True))

        points.append(raw[-1])
        if len(points) > STEP_TURN_MAX_PATH_POINTS:
            errors.append(StepTurnDiagnostic("too_many_materialized_points", "编译后的路径点数超过 256 个限制"))
        if any(math.dist(first, second) < _MIN_POINT_DISTANCE_MM for first, second in zip(points, points[1:])):
            errors.append(StepTurnDiagnostic("coincident_path_points", "编译后的相邻路径点间距必须至少为 1 mm"))
        return StepTurnCompileResult(tuple(_with_line_yaw(points)), tuple(errors), tuple(corners))

    @staticmethod
    def generate(start: Pose, segment: StepTurnPathSegment) -> tuple[PathPosePoint, ...]:
        result = StepTurnCompiler.analyze(start, segment)
        if result.has_errors:
            raise ValueError("；".join(diagnostic.message for diagnostic in result.diagnostics))
        return result.points


def analyze_step_turn_path(start_pose: Pose, segment: StepTurnPathSegment) -> StepTurnCompileResult:
    return StepTurnCompiler.analyze(start_pose, segment)


def generate_step_turn_path_points(start_pose: Pose, segment: StepTurnPathSegment) -> tuple[PathPosePoint, ...]:
    return StepTurnCompiler.generate(start_pose, segment)


def _unit_vector(first: tuple[float, float], second: tuple[float, float]) -> tuple[float, float] | None:
    dx, dy = second[0] - first[0], second[1] - first[1]
    length = math.hypot(dx, dy)
    return None if length < _MIN_POINT_DISTANCE_MM else (dx / length, dy / length)


def _with_line_yaw(points: list[tuple[float, float]]) -> list[PathPosePoint]:
    if len(points) < 2:
        return []
    result: list[PathPosePoint] = []
    for index, point in enumerate(points):
        first, second = (points[index], points[index + 1]) if index + 1 < len(points) else (points[-2], points[-1])
        yaw = wrap_deg(math.degrees(math.atan2(-(second[0] - first[0]), second[1] - first[1])))
        result.append(PathPosePoint(point[0], point[1], yaw, "START" if index == 0 else "END" if index == len(points) - 1 else ("A" if index % 2 else "B")))
    return result
