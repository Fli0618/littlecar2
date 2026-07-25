"""实时显示带数字同心圆检测，并使用卡尔曼滤波与时间窗口消抖。"""

from __future__ import annotations

import sys
import time
from collections import deque
from pathlib import Path
from typing import Any

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from vision import detect_circle, detect_color


CAMERA_INDEX = 1
WINDOW_NAME = "Concentric Circle Kalman Filter | Press Q or ESC to exit"
WINDOW_SIZE = 5
MIN_DETECTIONS = 3


def create_kalman_filter(center: list[int]) -> cv2.KalmanFilter:
    kalman = cv2.KalmanFilter(4, 2)
    kalman.transitionMatrix = np.array(
        [[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]],
        dtype=np.float32,
    )
    kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32)
    kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
    kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 4.0
    kalman.errorCovPost = np.eye(4, dtype=np.float32)
    kalman.statePost = np.array([[center[0]], [center[1]], [0], [0]], dtype=np.float32)
    return kalman


def main() -> None:
    camera = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    if not camera.isOpened():
        raise RuntimeError(f"无法打开相机: {CAMERA_INDEX}")

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    trackers: dict[str, dict[str, Any]] = {}

    try:
        while True:
            success, frame_bgr = camera.read()
            if not success:
                raise RuntimeError("无法读取相机画面")

            raw_frame = frame_bgr.copy()
            started = time.perf_counter()
            # 如需测试物料颜色模型，可将下一行改为 detect_color(frame_bgr)。
            result = detect_circle(frame_bgr)
            inference_ms = (time.perf_counter() - started) * 1000.0

            best_detections: dict[str, dict[str, Any]] = {}
            for detection in result["detections"]:
                target_type = detection["type"]
                if target_type not in best_detections or detection["confidence"] > best_detections[target_type]["confidence"]:
                    best_detections[target_type] = detection

            filtered_targets: list[tuple[str, list[int], float, bool]] = []
            for target_type in set(trackers) | set(best_detections):
                detection = best_detections.get(target_type)
                if target_type not in trackers:
                    if detection is None:
                        continue
                    trackers[target_type] = {
                        "kalman": create_kalman_filter(detection["center"]),
                        "history": deque(maxlen=WINDOW_SIZE),
                        "confidence": 0.0,
                    }

                tracker = trackers[target_type]
                prediction = tracker["kalman"].predict()[:2].reshape(-1)
                tracker["history"].append(detection is not None)
                measured = detection is not None
                if measured:
                    measurement = np.asarray(detection["center"], dtype=np.float32).reshape(2, 1)
                    prediction = tracker["kalman"].correct(measurement)[:2].reshape(-1)
                    tracker["confidence"] = float(detection["confidence"])

                if sum(tracker["history"]) >= MIN_DETECTIONS:
                    filtered_targets.append(
                        (
                            target_type,
                            [int(round(prediction[0])), int(round(prediction[1]))],
                            float(tracker["confidence"]),
                            measured,
                        )
                    )

            result_frame = frame_bgr.copy()
            for target_type, center, confidence, measured in filtered_targets:
                color = (0, 255, 0) if measured else (0, 255, 255)
                state = "measured" if measured else "predicted"
                cv2.circle(result_frame, tuple(center), 8, color, -1)
                cv2.putText(
                    result_frame,
                    f"{target_type} {confidence:.2f} {state}",
                    (center[0] + 12, max(24, center[1] - 12)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    color,
                    2,
                    cv2.LINE_AA,
                )

            cv2.putText(raw_frame, "Camera", (20, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(
                result_frame,
                f"YOLO: {len(result['detections'])} targets, {inference_ms:.1f} ms, 3/5 window",
                (20, 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(WINDOW_NAME, cv2.hconcat([raw_frame, result_frame]))

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
