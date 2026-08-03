import math

from map_planner.bezier import evaluate_cubic, evaluate_cubic_derivative, generate_bezier_path_points
from map_planner.models import Pose


def test_cubic_endpoints_and_tangent_yaw():
    p0, p1, p2, p3 = (0, 0), (0, 100), (100, 100), (100, 0)
    assert evaluate_cubic(p0, p1, p2, p3, 0) == p0
    assert evaluate_cubic(p0, p1, p2, p3, 1) == p3
    assert evaluate_cubic_derivative(p0, p1, p2, p3, 0) == (0.0, 300.0)


def test_arc_length_sampling_is_deterministic_and_preserves_endpoints():
    start, end = Pose(0, 0, 0), Pose(200, 0, 90)
    first = generate_bezier_path_points(start, (0, 150), (200, 150), end, "interpolate", 20)
    second = generate_bezier_path_points(start, (0, 150), (200, 150), end, "interpolate", 20)
    assert first == second
    assert (first[0].x_mm, first[0].y_mm) == (0, 0)
    assert (first[-1].x_mm, first[-1].y_mm) == (200, 0)
    assert all(math.hypot(b.x_mm-a.x_mm, b.y_mm-a.y_mm) >= 1 for a, b in zip(first, first[1:]))


def test_interpolated_and_tangent_headings_follow_the_selected_mode():
    interpolated = generate_bezier_path_points(Pose(0, 0, 170), (0, 100), (100, 100), Pose(100, 0, -170), "interpolate", 20)
    assert interpolated[0].yaw_deg == 170
    assert interpolated[-1].yaw_deg == -170
    assert max(abs(point.yaw_deg) for point in interpolated) >= 170

    tangent = generate_bezier_path_points(Pose(0, 0, 0), (100, 0), (100, 100), Pose(100, 200, 0), "tangent", 20)
    assert abs(tangent[0].yaw_deg - 90) < 1e-6
    assert abs(tangent[-1].yaw_deg) < 1e-6
