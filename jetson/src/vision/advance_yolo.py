"""带多目标卡尔曼滤波与时间窗口确认的 YOLO 高层检测接口。"""

from __future__ import annotations

from collections import deque
from typing import Any, Callable

import cv2
import numpy as np

from .yolo import detect_circle, detect_color


CAMERA_INDEX_COLOR_CIRCLE = 1
CAMERA_INDEX_QR = 0
WINDOW_SIZE = 5
MIN_DETECTIONS = 2
MATCH_THRESHOLD_RATIO = 0.1
KALMAN_Q_COEF = 0.5
KALMAN_R_COEF = 1.5

_TRACKERS: dict[str, dict[int, dict[str, Any]]] = {"color": {}, "circle": {}}
_NEXT_TRACKER_ID: dict[str, int] = {"color": 0, "circle": 0}


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

    for tracker_id in set(trackers) - matched_trackers:
        tracker = trackers[tracker_id]
        tracker["history"].append(False)
        prediction = predictions[tracker_id]
        tracker["center"] = [int(round(prediction[0])), int(round(prediction[1]))]

    for detection_index, detection in enumerate(raw_detections):
        if detection_index in matched_detections:
            continue
        tracker_id = _NEXT_TRACKER_ID[tracker_group]
        trackers[tracker_id] = {
            "type": detection["type"],
            "kalman": _create_kalman_filter(detection["center"]),
            "history": deque([True], maxlen=WINDOW_SIZE),
            "confidence": float(detection["confidence"]),
            "center": list(detection["center"]),
        }
        _NEXT_TRACKER_ID[tracker_group] += 1

    for tracker_id in [item for item, tracker in trackers.items() if sum(tracker["history"]) == 0]:
        del trackers[tracker_id]

    detections: list[dict[str, Any]] = []
    for tracker_id, tracker in trackers.items():
        if sum(tracker["history"]) >= MIN_DETECTIONS:
            detections.append(
                {
                    "tracking_id": tracker_id,
                    "type": tracker["type"],
                    "center": tracker["center"],
                    "confidence": tracker["confidence"],
                    "measured": bool(tracker["history"][-1]),
                }
            )
    return {"detections": detections}


def advance_detect_color(frame_bgr: np.ndarray) -> dict[str, list[dict[str, Any]]]:
    """检测并平滑彩色物料与 EmptySlot 的多目标中心点。"""
    return _advance_detect("color", frame_bgr, detect_color)


def advance_detect_circle(frame_bgr: np.ndarray) -> dict[str, list[dict[str, Any]]]:
    """检测并平滑带数字同心圆的多目标中心点。"""
    return _advance_detect("circle", frame_bgr, detect_circle)


def _reset_advance_tracking() -> None:
    """清空高级检测状态，仅用于测试或显式重置视频会话。"""
    for group in _TRACKERS:
        _TRACKERS[group].clear()
        _NEXT_TRACKER_ID[group] = 0
