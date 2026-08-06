"""带多目标卡尔曼滤波与时间窗口确认的 YOLO 高层检测接口。"""

from __future__ import annotations

from collections import deque
from typing import Any, Callable

import cv2
import numpy as np

from .yolo import detect_circle, detect_color
from .hsv_color import detect_color_hsv
from .hybrid_color import detect_color_hybrid


CAMERA_INDEX_COLOR_CIRCLE = 1
CAMERA_INDEX_QR = 0
WINDOW_SIZE = 5
MIN_DETECTIONS = 2
MATCH_THRESHOLD_RATIO = 0.1
KALMAN_Q_COEF = 0.5
KALMAN_R_COEF = 1.5

_TRACKERS: dict[str, dict[int, dict[str, Any]]] = {"color": {}, "circle": {}}
_NEXT_TRACKER_ID: dict[str, int] = {"color": 0, "circle": 0}
_DEBUG_FIELDS = (
    "yolo_type",
    "yolo_confidence",
    "hsv_color",
    "hsv_coverage",
    "hsv_purity",
    "hsv_margin",
    "classification_source",
)


def _copy_debug_fields(target: dict[str, Any], source: dict[str, Any]) -> None:
    """复制可选调试字段，兼容旧 YOLO/HSV detection 结构。"""
    for field in _DEBUG_FIELDS:
        if field in source:
            target[field] = source[field]
        else:
            target.pop(field, None)


def _create_kalman_filter(center: list[int]) -> cv2.KalmanFilter:
    kalman = cv2.KalmanFilter(4, 2)
    kalman.transitionMatrix = np.array(
        [[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]],
        dtype=np.float32,
    )
    kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32)
    kalman.processNoiseCov = np.eye(4, dtype=np.float32) * KALMAN_Q_COEF
    kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * KALMAN_R_COEF
    kalman.errorCovPost = np.eye(4, dtype=np.float32)
    kalman.statePost = np.array([[center[0]], [center[1]], [0], [0]], dtype=np.float32)
    return kalman


def _advance_detect(
    tracker_group: str,
    frame_bgr: np.ndarray,
    detector: Callable[[np.ndarray], dict[str, list[dict[str, Any]]]],
) -> dict[str, list[dict[str, Any]]]:
    raw_detections = detector(frame_bgr).get("detections", [])
    trackers = _TRACKERS[tracker_group]
    match_threshold = frame_bgr.shape[1] * MATCH_THRESHOLD_RATIO

    predictions: dict[int, np.ndarray] = {}
    for tracker_id, tracker in trackers.items():
        predictions[tracker_id] = tracker["kalman"].predict()[:2].reshape(-1)

    matched_detections: set[int] = set()
    matched_trackers: set[int] = set()
    matches: list[tuple[int, int]] = []
    for tracker_id, prediction in predictions.items():
        best_distance = float("inf")
        best_detection_index = -1
        for detection_index, detection in enumerate(raw_detections):
            if detection_index in matched_detections or detection["type"] != trackers[tracker_id]["type"]:
                continue
            distance = float(np.linalg.norm(prediction - np.asarray(detection["center"])))
            if distance < best_distance:
                best_distance = distance
                best_detection_index = detection_index
        if best_distance < match_threshold:
            matches.append((tracker_id, best_detection_index))
            matched_trackers.add(tracker_id)
            matched_detections.add(best_detection_index)

    for tracker_id, detection_index in matches:
        tracker = trackers[tracker_id]
        detection = raw_detections[detection_index]
        measurement = np.asarray(detection["center"], dtype=np.float32).reshape(2, 1)
        corrected = tracker["kalman"].correct(measurement)[:2].reshape(-1)
        tracker["history"].append(True)
        tracker["confidence"] = float(detection["confidence"])
        tracker["center"] = [int(round(corrected[0])), int(round(corrected[1]))]
        _copy_debug_fields(tracker, detection)
        if "bbox" in detection:
            tracker["bbox"] = list(detection["bbox"])
        elif "bbox" not in tracker:
            tracker["bbox"] = _default_bbox(tracker["center"])

    for tracker_id in set(trackers) - matched_trackers:
        tracker = trackers[tracker_id]
        tracker["history"].append(False)
        prediction = predictions[tracker_id]
        old_center = tracker["center"]
        tracker["center"] = [int(round(prediction[0])), int(round(prediction[1]))]
        if "bbox" in tracker:
            dx = tracker["center"][0] - old_center[0]
            dy = tracker["center"][1] - old_center[1]
            x1, y1, x2, y2 = tracker["bbox"]
            tracker["bbox"] = [x1 + dx, y1 + dy, x2 + dx, y2 + dy]

    for detection_index, detection in enumerate(raw_detections):
        if detection_index in matched_detections:
            continue
        tracker_id = _NEXT_TRACKER_ID[tracker_group]
        tracker = {
            "type": detection["type"],
            "kalman": _create_kalman_filter(detection["center"]),
            "history": deque([True], maxlen=WINDOW_SIZE),
            "confidence": float(detection["confidence"]),
            "center": list(detection["center"]),
            "bbox": list(detection.get("bbox", _default_bbox(detection["center"]))),
        }
        _copy_debug_fields(tracker, detection)
        trackers[tracker_id] = tracker
        _NEXT_TRACKER_ID[tracker_group] += 1

    for tracker_id in [item for item, tracker in trackers.items() if sum(tracker["history"]) == 0]:
        del trackers[tracker_id]

    detections: list[dict[str, Any]] = []
    for tracker_id, tracker in trackers.items():
        if sum(tracker["history"]) >= MIN_DETECTIONS:
            output = {
                "type": tracker["type"],
                "center": tracker["center"],
                "confidence": tracker["confidence"],
                "measured": bool(tracker["history"][-1]),
                "support_count": sum(tracker["history"]),
                "bbox": tracker["bbox"],
            }
            _copy_debug_fields(output, tracker)
            detections.append(output)
    return {"detections": detections}


def _default_bbox(center: list[int], size: int = 80) -> list[int]:
    """兼容旧检测器结果；没有框时为预览提供一个保守的占位框。"""
    half = size // 2
    return [center[0] - half, center[1] - half, center[0] + half, center[1] + half]


def advance_detect_color(frame_bgr: np.ndarray) -> dict[str, list[dict[str, Any]]]:
    """检测并平滑彩色物料与 EmptySlot 的多目标中心点。"""
    return _advance_detect("color", frame_bgr, detect_color)


def advance_detect_color_hsv(frame_bgr: np.ndarray) -> dict[str, list[dict[str, Any]]]:
    """使用 HSV 规则检测并平滑红、黄、蓝、绿四类物料。"""
    return _advance_detect("color", frame_bgr, detect_color_hsv)


def advance_detect_color_hybrid(frame_bgr: np.ndarray) -> dict[str, list[dict[str, Any]]]:
    """使用 YOLO 定位、HSV 分类，并在分类后执行多目标跟踪。"""
    return _advance_detect("color", frame_bgr, detect_color_hybrid)


def advance_detect_circle(frame_bgr: np.ndarray) -> dict[str, list[dict[str, Any]]]:
    """检测并平滑带数字同心圆的多目标中心点。"""
    return _advance_detect("circle", frame_bgr, detect_circle)


def reset_advance_tracking() -> None:
    """清空高级检测状态，仅用于测试或显式重置视频会话。"""
    for group in _TRACKERS:
        _TRACKERS[group].clear()
        _NEXT_TRACKER_ID[group] = 0


def _reset_advance_tracking() -> None:
    """兼容旧调用方；新代码应使用 ``reset_advance_tracking``。"""
    reset_advance_tracking()
