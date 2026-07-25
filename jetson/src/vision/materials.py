"""物料盘中心推断。"""

from __future__ import annotations

from typing import Any

import numpy as np

from .yolo import _validate_frame


def detect_disk_center(frame_bgr: np.ndarray, color_result: dict[str, Any]) -> dict[str, Any]:
    """根据颜色检测结果推断物料盘中心，并以支持点数反馈推断状态。"""
    _validate_frame(frame_bgr)
    detections = color_result.get("detections")
    if not isinstance(detections, list):
        raise ValueError("color_result must contain a detections list")

    selected = sorted(
        (detection for detection in detections if _is_detection(detection)),
        key=lambda detection: float(detection["confidence"]),
        reverse=True,
    )[:3]
    points = [list(map(int, detection["center"])) for detection in selected]
    support_count = len(points)
    measured_count = sum(1 for detection in selected if detection.get("measured", True))
    image_height, image_width = frame_bgr.shape[:2]

    if support_count == 3:
        center = np.mean(points, axis=0)
    elif support_count == 2:
        center = _estimate_from_two_points(points, image_width, image_height)
    elif support_count == 1:
        center = np.asarray(points[0], dtype=float)
    else:
        center = np.asarray([0.0, 0.0])

    return {
        "center": [int(round(center[0])), int(round(center[1]))],
        "status": 1 if support_count else 0,
        "support_count": support_count,
        "measured_count": measured_count,
        "support_points": points,
    }


def advance_detect_disk_center(frame_bgr: np.ndarray) -> dict[str, Any]:
    """使用带时间窗口和卡尔曼滤波的物料检测结果推断物料盘中心。"""
    from .advance_yolo import advance_detect_color

    return detect_disk_center(frame_bgr, advance_detect_color(frame_bgr))


def _is_detection(detection: Any) -> bool:
    if not isinstance(detection, dict) or not isinstance(detection.get("center"), (list, tuple)):
        return False
    center = detection["center"]
    return len(center) == 2 and isinstance(detection.get("confidence"), (int, float))


def _estimate_from_two_points(points: list[list[int]], image_width: int, image_height: int) -> np.ndarray:
    p1, p2 = np.asarray(points, dtype=float)
    distance = float(np.linalg.norm(p2 - p1))
    if distance == 0.0:
        return p1

    midpoint = (p1 + p2) / 2.0
    height = distance / (2.0 * np.sqrt(3.0))
    normal = np.array([-(p2 - p1)[1], (p2 - p1)[0]]) / distance
    candidates = (midpoint + height * normal, midpoint - height * normal)
    image_center = np.array([image_width / 2.0, image_height / 2.0])
    return min(candidates, key=lambda candidate: np.linalg.norm(candidate - image_center))
