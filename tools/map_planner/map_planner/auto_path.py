"""Fixed-map any-angle planning and safe smoothing for the motion workbench."""

from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math

from .geometry import (paper_heading_to_world_yaw, paper_to_world,
                       paper_vector_to_heading, wrap_deg)
from .models import (FIELD_SIZE_MM, VEHICLE_WIDTH_MM, CostmapSettings, Obstacle,
                     PathPosePoint, Pose)


FIXED_PLATFORM_RECTS = (
    (550.0, 550.0, 1000.0, 1000.0),
    (1400.0, 550.0, 1850.0, 1000.0),
    (550.0, 1400.0, 1000.0, 1850.0),
    (1400.0, 1400.0, 1850.0, 1850.0),
)
VISIBILITY_CORNER_EPSILON_MM = 1.0


class AutoPathError(ValueError):
    """Raised when no safe path can be produced for the requested geometry."""


@dataclass(frozen=True)
class InflatedRect:
    left: float
    top: float
    right: float
    bottom: float
    source: str
    corner_radius: float = 0.0


@dataclass(frozen=True)
class AutoPathSettings:
    costmap: CostmapSettings = field(default_factory=CostmapSettings)
    corner_radius_mm: float = 120.0
    sample_spacing_mm: float = 20.0
    terminal_straight_mm: float = 300.0
    yaw_mode: str = "fixed"
    include_fixed_platforms: bool = True
    max_points: int = 256

    def validate(self) -> None:
        costmap = self.costmap
        values = (costmap.boundary_safety_margin_mm,
                  costmap.vehicle_length_mm, costmap.vehicle_width_mm,
                  costmap.boundary_inflation_mm,
                  costmap.boundary_cost_weight,
                  costmap.platform_safety_margin_mm,
                  costmap.platform_inflation_mm,
                  costmap.platform_cost_weight,
                  costmap.obstacle_radius_mm,
                  costmap.obstacle_safety_margin_mm,
                  costmap.obstacle_inflation_mm,
                  costmap.obstacle_cost_weight,
                  self.corner_radius_mm, self.sample_spacing_mm,
                  self.terminal_straight_mm)
        if not all(math.isfinite(value) for value in values):
            raise AutoPathError("规划参数必须是有限数值")
        for label, value in (("车身长度", costmap.vehicle_length_mm),
                             ("车身宽度", costmap.vehicle_width_mm)):
            if not 50.0 <= value <= 500.0:
                raise AutoPathError(f"{label}必须在 50~500 mm")
        for label, value, maximum in (
            ("场地边线安全距离", costmap.boundary_safety_margin_mm, 150.0),
            ("场地边线软膨胀距离", costmap.boundary_inflation_mm, 500.0),
            ("平台安全距离", costmap.platform_safety_margin_mm, 150.0),
            ("平台软膨胀距离", costmap.platform_inflation_mm, 500.0),
            ("障碍物半径", costmap.obstacle_radius_mm, 100.0),
            ("障碍物安全距离", costmap.obstacle_safety_margin_mm, 150.0),
            ("障碍物软膨胀距离", costmap.obstacle_inflation_mm, 500.0),
        ):
            if not 0.0 <= value <= maximum:
                raise AutoPathError(f"{label}必须在 0~{maximum:g} mm")
        for label, value in (("场地边线", costmap.boundary_cost_weight),
                             ("平台", costmap.platform_cost_weight),
                             ("障碍物", costmap.obstacle_cost_weight)):
            if not 0.0 <= value <= 20.0:
                raise AutoPathError(f"{label}软代价权重必须在 0~20")
        if not 0.0 <= self.corner_radius_mm <= 400.0:
            raise AutoPathError("圆角半径必须在 0~400 mm")
        if not 10.0 <= self.sample_spacing_mm <= 50.0:
            raise AutoPathError("采样间距必须在 10~50 mm")
        if not 0.0 <= self.terminal_straight_mm <= 1000.0:
            raise AutoPathError("末端直线长度必须在 0~1000 mm")
        if self.yaw_mode not in ("fixed", "interpolate", "tangent"):
            raise AutoPathError("航向模式必须是 fixed、interpolate 或 tangent")
        if not 2 <= self.max_points <= 256:
            raise AutoPathError("轨迹点上限必须在 2~256")


@dataclass(frozen=True)
class AutoPathResult:
    route_paper: tuple[tuple[float, float], ...]
    samples_paper: tuple[tuple[float, float], ...]
    world_points: tuple[PathPosePoint, ...]
    inflated_rects: tuple[InflatedRect, ...]
    length_mm: float


def build_inflated_obstacles(
    obstacles: list[Obstacle] | tuple[Obstacle, ...],
    settings: AutoPathSettings,
) -> tuple[InflatedRect, ...]:
    """Return configuration-space rectangles for the vehicle center."""

    settings.validate()
    costmap = settings.costmap
    body_half = max(costmap.vehicle_length_mm, VEHICLE_WIDTH_MM) / 2.0
    boundary_safety = costmap.boundary_safety_margin_mm
    if boundary_safety > 0.0:
        # Start zones occupy the top-right and bottom-right 300 mm squares.
        # Leave center-line exits through those zones while applying the hard
        # boundary clearance to the remaining field perimeter.
        result = [
            # Left edge: storage opening y=910..1490.
            InflatedRect(body_half, body_half, body_half + boundary_safety,
                         910.0, "boundary"),
            InflatedRect(body_half, 1490.0, body_half + boundary_safety,
                         FIELD_SIZE_MM - body_half, "boundary"),
            # Top edge: raw-material opening x=1050..1350 and start zone 1.
            InflatedRect(body_half, body_half, 1050.0,
                         body_half + boundary_safety, "boundary"),
            InflatedRect(1350.0, body_half, 2100.0,
                         body_half + boundary_safety, "boundary"),
            # Right edge: start zones and QR opening y=1100..1300.
            InflatedRect(FIELD_SIZE_MM - body_half - boundary_safety, 300.0,
                         FIELD_SIZE_MM - body_half, 1100.0, "boundary"),
            InflatedRect(FIELD_SIZE_MM - body_half - boundary_safety, 1300.0,
                         FIELD_SIZE_MM - body_half, 2100.0, "boundary"),
            # Bottom edge: rough-processing opening x=910..1490 and start zone 2.
            InflatedRect(body_half, FIELD_SIZE_MM - body_half - boundary_safety,
                         910.0, FIELD_SIZE_MM - body_half, "boundary"),
            InflatedRect(1490.0, FIELD_SIZE_MM - body_half - boundary_safety,
                         2100.0, FIELD_SIZE_MM - body_half, "boundary"),
        ]
    else:
        result = []
    platform_clearance = body_half + costmap.platform_safety_margin_mm
    if settings.include_fixed_platforms:
        result.extend(
            InflatedRect(left - platform_clearance, top - platform_clearance,
                         right + platform_clearance, bottom + platform_clearance,
                         "platform", platform_clearance)
            for left, top, right, bottom in FIXED_PLATFORM_RECTS
        )
    custom_clearance = (body_half + costmap.obstacle_radius_mm +
                        costmap.obstacle_safety_margin_mm)
    result.extend(
        InflatedRect(obstacle.paper_x_mm - custom_clearance,
                     obstacle.paper_y_mm - custom_clearance,
                     obstacle.paper_x_mm + custom_clearance,
                     obstacle.paper_y_mm + custom_clearance, "custom",
                     custom_clearance)
        for obstacle in obstacles
    )
    return tuple(result)


def _center_bounds(settings: AutoPathSettings) -> tuple[float, float, float, float]:
    # Physical center bound. Extra hard boundary safety is represented as
    # rectangles with explicit openings for the two legal start zones.
    clearance = max(settings.costmap.vehicle_length_mm, VEHICLE_WIDTH_MM) / 2.0
    return clearance, clearance, FIELD_SIZE_MM - clearance, FIELD_SIZE_MM - clearance


def _point_is_free(point: tuple[float, float], rects: tuple[InflatedRect, ...],
                   bounds: tuple[float, float, float, float]) -> bool:
    x, y = point
    left, top, right, bottom = bounds
    if not (left <= x <= right and top <= y <= bottom):
        return False
    return not any(_shape_contains(point, rect) for rect in rects)


def _shape_contains(point: tuple[float, float], shape: InflatedRect) -> bool:
    """Exact point test for a rectangle offset by a circular radius."""
    radius = max(0.0, shape.corner_radius)
    if radius <= 0.0:
        return (shape.left <= point[0] <= shape.right and
                shape.top <= point[1] <= shape.bottom)
    core_left, core_right = shape.left + radius, shape.right - radius
    core_top, core_bottom = shape.top + radius, shape.bottom - radius
    nearest_x = min(core_right, max(core_left, point[0]))
    nearest_y = min(core_bottom, max(core_top, point[1]))
    return math.hypot(point[0] - nearest_x,
                      point[1] - nearest_y) <= radius + 1e-9


def _segment_hits_rect(start: tuple[float, float], end: tuple[float, float],
                       rect: InflatedRect) -> bool:
    """Exact segment test for rectangles, rounded rectangles and circles."""

    radius = max(0.0, rect.corner_radius)
    if radius > 0.0:
        core = InflatedRect(rect.left + radius, rect.top + radius,
                            rect.right - radius, rect.bottom - radius,
                            rect.source)
        if _segment_hits_rect(start, end, core):
            return True
        corners = ((core.left, core.top), (core.right, core.top),
                   (core.right, core.bottom), (core.left, core.bottom))
        edges = tuple(zip(corners, (*corners[1:], corners[0])))
        return min(_segment_distance(start, end, first, second)
                   for first, second in edges) <= radius + 1e-9

    x0, y0 = start
    dx, dy = end[0] - x0, end[1] - y0
    low, high = 0.0, 1.0
    for origin, direction, minimum, maximum in (
        (x0, dx, rect.left, rect.right),
        (y0, dy, rect.top, rect.bottom),
    ):
        if abs(direction) <= 1e-12:
            if minimum <= origin <= maximum:
                continue
            return False
        first = (minimum - origin) / direction
        second = (maximum - origin) / direction
        if first > second:
            first, second = second, first
        low = max(low, first)
        high = min(high, second)
        if low > high:
            return False
    return high >= low


def _point_segment_distance(point: tuple[float, float],
                            start: tuple[float, float],
                            end: tuple[float, float]) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-18:
        return math.dist(point, start)
    ratio = max(0.0, min(1.0,
                         ((point[0] - start[0]) * dx +
                          (point[1] - start[1]) * dy) / length_squared))
    return math.hypot(point[0] - (start[0] + ratio * dx),
                      point[1] - (start[1] + ratio * dy))


def _segments_intersect(first_start: tuple[float, float],
                        first_end: tuple[float, float],
                        second_start: tuple[float, float],
                        second_end: tuple[float, float]) -> bool:
    def cross(a: tuple[float, float], b: tuple[float, float],
              c: tuple[float, float]) -> float:
        return ((b[0] - a[0]) * (c[1] - a[1]) -
                (b[1] - a[1]) * (c[0] - a[0]))

    first = cross(first_start, first_end, second_start)
    second = cross(first_start, first_end, second_end)
    third = cross(second_start, second_end, first_start)
    fourth = cross(second_start, second_end, first_end)
    if ((first > 1e-12 and second < -1e-12) or
            (first < -1e-12 and second > 1e-12)) and (
            (third > 1e-12 and fourth < -1e-12) or
            (third < -1e-12 and fourth > 1e-12)):
        return True

    def on_segment(a: tuple[float, float], b: tuple[float, float],
                   point: tuple[float, float], value: float) -> bool:
        return (abs(value) <= 1e-12 and
                min(a[0], b[0]) - 1e-12 <= point[0] <= max(a[0], b[0]) + 1e-12 and
                min(a[1], b[1]) - 1e-12 <= point[1] <= max(a[1], b[1]) + 1e-12)

    return (on_segment(first_start, first_end, second_start, first) or
            on_segment(first_start, first_end, second_end, second) or
            on_segment(second_start, second_end, first_start, third) or
            on_segment(second_start, second_end, first_end, fourth))


def _segment_distance(first_start: tuple[float, float],
                      first_end: tuple[float, float],
                      second_start: tuple[float, float],
                      second_end: tuple[float, float]) -> float:
    if _segments_intersect(first_start, first_end, second_start, second_end):
        return 0.0
    return min(_point_segment_distance(first_start, second_start, second_end),
               _point_segment_distance(first_end, second_start, second_end),
               _point_segment_distance(second_start, first_start, first_end),
               _point_segment_distance(second_end, first_start, first_end))


def _segment_is_free(start: tuple[float, float], end: tuple[float, float],
                     rects: tuple[InflatedRect, ...],
                     bounds: tuple[float, float, float, float]) -> bool:
    return (_point_is_free(start, rects, bounds) and
            _point_is_free(end, rects, bounds) and
            not any(_segment_hits_rect(start, end, rect) for rect in rects))


def _visibility_nodes(start: tuple[float, float], goal: tuple[float, float],
                      rects: tuple[InflatedRect, ...],
                      bounds: tuple[float, float, float, float],
                      settings: AutoPathSettings) -> list[tuple[float, float]]:
    nodes = [start, goal]
    for rect in rects:
        nodes.extend(_shape_boundary_nodes(rect))
        if rect.source in ("platform", "custom"):
            soft = (settings.costmap.platform_inflation_mm
                    if rect.source == "platform"
                    else settings.costmap.obstacle_inflation_mm)
            if soft > 0.0:
                nodes.extend(_shape_boundary_nodes(rect, soft))
    # Add candidates on the inner edge of the soft boundary-cost band. Without
    # these nodes a visibility graph containing only start/goal cannot choose a
    # slightly longer route away from a field edge.
    influence = min(settings.costmap.boundary_safety_margin_mm +
                    settings.costmap.boundary_inflation_mm,
                    (bounds[2] - bounds[0]) / 2.0,
                    (bounds[3] - bounds[1]) / 2.0)
    if influence > 0.0 and settings.costmap.boundary_cost_weight > 0.0:
        inner_left, inner_top = bounds[0] + influence, bounds[1] + influence
        inner_right, inner_bottom = bounds[2] - influence, bounds[3] - influence
        nodes.extend(((inner_left, inner_top), (inner_right, inner_top),
                      (inner_right, inner_bottom), (inner_left, inner_bottom)))
        for x, y in (start, goal):
            nodes.extend(((min(inner_right, max(inner_left, x)), inner_top),
                          (min(inner_right, max(inner_left, x)), inner_bottom),
                          (inner_left, min(inner_bottom, max(inner_top, y))),
                          (inner_right, min(inner_bottom, max(inner_top, y)))))
    unique: list[tuple[float, float]] = []
    for point in nodes:
        if _point_is_free(point, rects, bounds) and point not in unique:
            unique.append(point)
    return unique


def _shape_boundary_nodes(shape: InflatedRect,
                          extra_offset: float = 0.0) -> list[tuple[float, float]]:
    """Visibility candidates sampled just outside straight edges and arcs."""
    epsilon = VISIBILITY_CORNER_EPSILON_MM
    radius = max(0.0, shape.corner_radius + extra_offset)
    left, top = shape.left - extra_offset, shape.top - extra_offset
    right, bottom = shape.right + extra_offset, shape.bottom + extra_offset
    if radius <= 0.0:
        return [(left - epsilon, top - epsilon),
                (right + epsilon, top - epsilon),
                (right + epsilon, bottom + epsilon),
                (left - epsilon, bottom + epsilon)]

    core_left, core_right = left + radius, right - radius
    core_top, core_bottom = top + radius, bottom - radius
    arc_radius = radius + epsilon
    nodes = []
    for center, start_angle in (
        ((core_left, core_top), 180.0),
        ((core_right, core_top), 270.0),
        ((core_right, core_bottom), 0.0),
        ((core_left, core_bottom), 90.0),
    ):
        for index in range(5):
            radians = math.radians(start_angle + 90.0 * index / 4.0)
            nodes.append((center[0] + arc_radius * math.cos(radians),
                          center[1] + arc_radius * math.sin(radians)))
    return nodes


def _point_to_rect_distance(point: tuple[float, float], rect: InflatedRect) -> float:
    radius = max(0.0, rect.corner_radius)
    core_left, core_right = rect.left + radius, rect.right - radius
    core_top, core_bottom = rect.top + radius, rect.bottom - radius
    dx = max(core_left - point[0], 0.0, point[0] - core_right)
    dy = max(core_top - point[1], 0.0, point[1] - core_bottom)
    return max(0.0, math.hypot(dx, dy) - radius)


def _weighted_edge_cost(start: tuple[float, float], end: tuple[float, float],
                        bounds: tuple[float, float, float, float],
                        rects: tuple[InflatedRect, ...],
                        settings: AutoPathSettings) -> float:
    distance = math.dist(start, end)
    costmap = settings.costmap
    penalties = []
    for index in range(9):
        ratio = index / 8.0
        x = start[0] + (end[0] - start[0]) * ratio
        y = start[1] + (end[1] - start[1]) * ratio
        hard_clearance = (max(costmap.vehicle_length_mm,
                              costmap.vehicle_width_mm) / 2.0 +
                          costmap.boundary_safety_margin_mm)
        edge_distance = min(x - hard_clearance,
                            FIELD_SIZE_MM - hard_clearance - x,
                            y - hard_clearance,
                            FIELD_SIZE_MM - hard_clearance - y)
        penalty = 0.0
        if costmap.boundary_inflation_mm > 0.0:
            normalized = max(0.0, 1.0 - edge_distance /
                             costmap.boundary_inflation_mm)
            penalty += costmap.boundary_cost_weight * normalized * normalized
        for rect in rects:
            if rect.source == "boundary":
                continue
            influence = (costmap.platform_inflation_mm if rect.source == "platform"
                         else costmap.obstacle_inflation_mm)
            weight = (costmap.platform_cost_weight if rect.source == "platform"
                      else costmap.obstacle_cost_weight)
            if influence <= 0.0 or weight <= 0.0:
                continue
            normalized = max(0.0, 1.0 - _point_to_rect_distance((x, y), rect) /
                             influence)
            penalty += weight * normalized * normalized
        penalties.append(penalty)
    return distance * (1.0 + sum(penalties) / len(penalties))


def _shortest_visibility_route(start: tuple[float, float], goal: tuple[float, float],
                               rects: tuple[InflatedRect, ...],
                               bounds: tuple[float, float, float, float],
                               settings: AutoPathSettings) -> list[tuple[float, float]]:
    if not _point_is_free(start, rects, bounds):
        raise AutoPathError("起点位于膨胀禁区或车体中心可达范围外")
    if not _point_is_free(goal, rects, bounds):
        raise AutoPathError("终点位于膨胀禁区或车体中心可达范围外")
    if math.dist(start, goal) < 1.0:
        raise AutoPathError("起点与终点距离过小")

    nodes = _visibility_nodes(start, goal, rects, bounds, settings)
    start_index, goal_index = nodes.index(start), nodes.index(goal)
    adjacency: list[list[tuple[int, float]]] = [[] for _ in nodes]
    for first in range(len(nodes)):
        for second in range(first + 1, len(nodes)):
            if _segment_is_free(nodes[first], nodes[second], rects, bounds):
                edge_cost = _weighted_edge_cost(
                    nodes[first], nodes[second], bounds, rects, settings)
                adjacency[first].append((second, edge_cost))
                adjacency[second].append((first, edge_cost))

    costs = [math.inf] * len(nodes)
    previous = [-1] * len(nodes)
    costs[start_index] = 0.0
    queue = [(math.dist(start, goal), 0.0, start_index)]
    while queue:
        _, cost, current = heapq.heappop(queue)
        if cost != costs[current]:
            continue
        if current == goal_index:
            break
        for neighbor, edge_cost in adjacency[current]:
            candidate = cost + edge_cost
            if candidate + 1e-9 < costs[neighbor]:
                costs[neighbor] = candidate
                previous[neighbor] = current
                heuristic = math.dist(nodes[neighbor], goal)
                heapq.heappush(queue, (candidate + heuristic, candidate, neighbor))
    if not math.isfinite(costs[goal_index]):
        raise AutoPathError("膨胀后没有可通行路线；请减小安全余量或调整障碍物")

    indices = []
    current = goal_index
    while current >= 0:
        indices.append(current)
        if current == start_index:
            break
        current = previous[current]
    indices.reverse()
    return [nodes[index] for index in indices]


def _append_line(points: list[tuple[float, float]], target: tuple[float, float],
                 spacing_mm: float) -> None:
    start = points[-1]
    distance = math.dist(start, target)
    if distance <= 1e-9:
        return
    count = max(1, math.ceil(distance / spacing_mm))
    points.extend((start[0] + (target[0] - start[0]) * index / count,
                   start[1] + (target[1] - start[1]) * index / count)
                  for index in range(1, count + 1))


def _quintic_bezier(controls: tuple[tuple[float, float], ...],
                    t: float) -> tuple[float, float]:
    one_minus = 1.0 - t
    weights = (one_minus ** 5, 5 * one_minus ** 4 * t,
               10 * one_minus ** 3 * t * t,
               10 * one_minus * one_minus * t ** 3,
               5 * one_minus * t ** 4, t ** 5)
    return (sum(weight * point[0] for weight, point in zip(weights, controls)),
            sum(weight * point[1] for weight, point in zip(weights, controls)))


def _corner_curve(previous: tuple[float, float], corner: tuple[float, float],
                  following: tuple[float, float], cut_mm: float,
                  spacing_mm: float) -> list[tuple[float, float]]:
    incoming = math.dist(previous, corner)
    outgoing = math.dist(corner, following)
    entry = (corner[0] + (previous[0] - corner[0]) * cut_mm / incoming,
             corner[1] + (previous[1] - corner[1]) * cut_mm / incoming)
    exit_point = (corner[0] + (following[0] - corner[0]) * cut_mm / outgoing,
                  corner[1] + (following[1] - corner[1]) * cut_mm / outgoing)
    incoming_unit = ((corner[0] - previous[0]) / incoming,
                     (corner[1] - previous[1]) / incoming)
    outgoing_unit = ((following[0] - corner[0]) / outgoing,
                     (following[1] - corner[1]) / outgoing)
    handle = cut_mm * 0.4
    controls = (
        entry,
        (entry[0] + incoming_unit[0] * handle,
         entry[1] + incoming_unit[1] * handle),
        (entry[0] + incoming_unit[0] * handle * 2.0,
         entry[1] + incoming_unit[1] * handle * 2.0),
        (exit_point[0] - outgoing_unit[0] * handle * 2.0,
         exit_point[1] - outgoing_unit[1] * handle * 2.0),
        (exit_point[0] - outgoing_unit[0] * handle,
         exit_point[1] - outgoing_unit[1] * handle),
        exit_point,
    )
    estimated_length = math.dist(entry, corner) + math.dist(corner, exit_point)
    count = max(2, math.ceil(estimated_length / spacing_mm))
    return [_quintic_bezier(controls, index / count)
            for index in range(count + 1)]


def _curve_is_free(points: list[tuple[float, float]], rects: tuple[InflatedRect, ...],
                   bounds: tuple[float, float, float, float]) -> bool:
    return all(_segment_is_free(first, second, rects, bounds)
               for first, second in zip(points, points[1:]))


def _smooth_and_sample(route: list[tuple[float, float]], settings: AutoPathSettings,
                       rects: tuple[InflatedRect, ...],
                       bounds: tuple[float, float, float, float]) -> list[tuple[float, float]]:
    result = [route[0]]
    corners = list(zip(route, route[1:], route[2:]))
    for corner_index, (previous, corner, following) in enumerate(corners):
        incoming = math.dist(previous, corner)
        outgoing = math.dist(corner, following)
        cut = min(settings.corner_radius_mm, incoming * 0.35, outgoing * 0.35)
        if corner_index == len(corners) - 1:
            cut = min(cut, max(0.0, outgoing - settings.terminal_straight_mm))
        curve: list[tuple[float, float]] | None = None
        while cut >= settings.sample_spacing_mm / 2.0:
            candidate = _corner_curve(previous, corner, following, cut,
                                      settings.sample_spacing_mm)
            if _curve_is_free(candidate, rects, bounds):
                curve = candidate
                break
            cut *= 0.5
        if curve is None:
            _append_line(result, corner, settings.sample_spacing_mm)
        else:
            _append_line(result, curve[0], settings.sample_spacing_mm)
            result.extend(curve[1:])
    _append_line(result, route[-1], settings.sample_spacing_mm)
    if not _curve_is_free(result, rects, bounds):
        raise AutoPathError("平滑轨迹复验失败")
    return result


def _world_path(samples: list[tuple[float, float]], start_frame: Pose,
                start_yaw_deg: float, goal_yaw_deg: float,
                settings: AutoPathSettings) -> tuple[PathPosePoint, ...]:
    cumulative = [0.0]
    for first, second in zip(samples, samples[1:]):
        cumulative.append(cumulative[-1] + math.dist(first, second))
    total = cumulative[-1]
    result = []
    for index, point in enumerate(samples):
        if settings.yaw_mode == "fixed":
            yaw = start_yaw_deg
        elif settings.yaw_mode == "interpolate":
            ratio = cumulative[index] / total if total > 0.0 else 1.0
            yaw = wrap_deg(start_yaw_deg + wrap_deg(goal_yaw_deg - start_yaw_deg) * ratio)
        else:
            before = samples[index - 1] if index else samples[index]
            after = samples[index + 1] if index + 1 < len(samples) else samples[index]
            heading = paper_vector_to_heading(after[0] - before[0], after[1] - before[1])
            yaw = paper_heading_to_world_yaw(start_frame.yaw_deg, heading)
            # At an official boundary start zone the center may be only half a
            # body width from the line. Keep the calibrated start heading until
            # the circumscribed body radius fits; otherwise a tangent yaw can
            # sweep a corner outside the field before the car has exited.
            body_radius = 0.5 * math.hypot(
                settings.costmap.vehicle_length_mm,
                settings.costmap.vehicle_width_mm)
            boundary_clearance = (body_radius +
                                  settings.costmap.boundary_safety_margin_mm)
            if min(point[0], point[1], FIELD_SIZE_MM - point[0],
                   FIELD_SIZE_MM - point[1]) < boundary_clearance:
                yaw = start_yaw_deg
        world = paper_to_world(point[0], point[1], start_frame.x_mm,
                               start_frame.y_mm, start_frame.yaw_deg)
        result.append(PathPosePoint(world.x_mm, world.y_mm, yaw))
    return tuple(result)


def plan_auto_path(
    start_paper: tuple[float, float],
    goal_paper: tuple[float, float],
    start_frame: Pose,
    start_yaw_deg: float,
    goal_yaw_deg: float,
    obstacles: list[Obstacle] | tuple[Obstacle, ...],
    settings: AutoPathSettings = AutoPathSettings(),
) -> AutoPathResult:
    """Plan, smooth, validate and transform one fixed-map path for STM32 upload."""

    settings.validate()
    rects = build_inflated_obstacles(obstacles, settings)
    bounds = _center_bounds(settings)
    route = _shortest_visibility_route(start_paper, goal_paper, rects, bounds,
                                       settings)
    samples = _smooth_and_sample(route, settings, rects, bounds)
    if len(samples) > settings.max_points:
        raise AutoPathError(
            f"规划得到 {len(samples)} 个点，超过协议上限 {settings.max_points}；请增大采样间距")
    world_points = _world_path(samples, start_frame, start_yaw_deg,
                               goal_yaw_deg, settings)
    return AutoPathResult(tuple(route), tuple(samples), world_points, rects,
                          sum(math.dist(first, second)
                              for first, second in zip(samples, samples[1:])))
