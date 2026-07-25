"""实时显示多目标同心圆检测，采用距离关联、多点卡尔曼滤波与时间窗口消抖。"""

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

# ==================== 顶部可调参数 ====================
WINDOW_SIZE = 5         # 判定窗口大小
MIN_DETECTIONS = 2      # 降低消抖门槛，5帧里检测到2帧就显示，对断续小目标更友好

# 1. 匹配距离比例：占图像宽度的比例。超出此比例则不视为同一目标
MATCH_THRESHOLD_RATIO = 0.1

# 2. 卡尔曼滤波过程噪声（Q）系数：
# 数值越大，越灵敏（更贴合目标突然拐弯、加减速），但可能会有微小抖动；越小越平缓但有滞后
KALMAN_Q_COEF = 0.5

# 3. 卡尔曼滤波测量噪声（R）系数：
# 数值越小，越信任 YOLO 的定位坐标，灵敏度越高；数值越大，越能过滤 YOLO 坐标的抖动
KALMAN_R_COEF = 1.5
# ====================================================


def create_kalman_filter(center: list[int]) -> cv2.KalmanFilter:
    """创建并初始化卡尔曼滤波器，使用顶部的全局调优系数。"""
    kalman = cv2.KalmanFilter(4, 2)
    kalman.transitionMatrix = np.array(
        [[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]],
        dtype=np.float32,
    )
    kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32)

    # 使用顶部定义好的可调系数
    kalman.processNoiseCov = np.eye(4, dtype=np.float32) * KALMAN_Q_COEF
    kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * KALMAN_R_COEF

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

    # 跟踪器字典，Key 为自增的唯一数字 ID，Value 存储该具体目标的跟踪状态
    trackers: dict[int, dict[str, Any]] = {}
    next_tracker_id = 0

    try:
        while True:
            success, frame_bgr = camera.read()
            if not success:
                raise RuntimeError("无法读取相机画面")

            # 动态获取图像宽度，并根据比例计算出当下的绝对像素匹配阈值
            frame_width = frame_bgr.shape[1]
            match_threshold = frame_width * MATCH_THRESHOLD_RATIO

            raw_frame = frame_bgr.copy()
            started = time.perf_counter()
            result = detect_circle(frame_bgr)
            inference_ms = (time.perf_counter() - started) * 1000.0

            detections = result.get("detections", [])

            # 1. 第一步：对所有已有 tracker 进行卡尔曼 Predict 并保存预测位置
            tracker_predictions = {}
            for tid, tracker in trackers.items():
                pred = tracker["kalman"].predict()[:2].reshape(-1)
                tracker_predictions[tid] = (int(pred[0]), int(pred[1]))

            # 2. 第二步：多目标最近邻关联 (Distance-based Nearest Neighbor Association)
            matched_detections = set()
            matched_trackers = set()
            matches = []

            # 计算已有 Tracker 与新 Detection 之间的欧氏距离，进行匹配
            for tid, pred_pos in tracker_predictions.items():
                best_dist = float("inf")
                best_det_idx = -1
                for idx, det in enumerate(detections):
                    if idx in matched_detections:
                        continue
                    # 只有类别一致（比如都是同一种标签）才允许匹配
                    if det["type"] != trackers[tid]["type"]:
                        continue

                    dist = float(np.linalg.norm(np.array(pred_pos) - np.array(det["center"])))
                    if dist < best_dist:
                        best_dist = dist
                        best_det_idx = idx

                # 判定条件：使用动态计算出来的绝对像素距离阈值
                if best_dist < match_threshold:
                    matches.append((tid, best_det_idx))
                    matched_detections.add(best_det_idx)
                    matched_trackers.add(tid)

            # 3. 第三步：更新已匹配的跟踪器
            for tid, det_idx in matches:
                tracker = trackers[tid]
                det = detections[det_idx]
                measurement = np.asarray(det["center"], dtype=np.float32).reshape(2, 1)

                # 使用真实测量值进行卡尔曼修正
                prediction = tracker["kalman"].correct(measurement)[:2].reshape(-1)
                tracker["history"].append(True)
                tracker["confidence"] = float(det["confidence"])
                tracker["center"] = [int(round(prediction[0])), int(round(prediction[1]))]

            # 4. 第四步：处理未匹配的跟踪器（本帧丢失的目标，依靠预测外推位置）
            for tid in set(trackers.keys()) - matched_trackers:
                tracker = trackers[tid]
                tracker["history"].append(False)
                # 使用上一阶段 Predict 推算的预测位置
                pred_pos = tracker_predictions[tid]
                tracker["center"] = [pred_pos[0], pred_pos[1]]

            # 5. 第五步：处理未匹配的检测值（新出现的目标，初始化跟踪器）
            for idx, det in enumerate(detections):
                if idx not in matched_detections:
                    trackers[next_tracker_id] = {
                        "type": det["type"],
                        "kalman": create_kalman_filter(det["center"]),
                        "history": deque([True], maxlen=WINDOW_SIZE),
                        "confidence": float(det["confidence"]),
                        "center": det["center"],
                    }
                    next_tracker_id += 1

            # 6. 第六步：清理彻底丢失的目标（在消抖窗口内被检测到的帧数为 0 触发释放）
            inactive_tids = [tid for tid, t in trackers.items() if sum(t["history"]) == 0]
            for tid in inactive_tids:
                del trackers[tid]

            # 7. 渲染结果
            result_frame = frame_bgr.copy()
            active_targets_count = 0

            for tid, tracker in trackers.items():
                # 判断消抖窗口是否达标
                if sum(tracker["history"]) >= MIN_DETECTIONS:
                    active_targets_count += 1
                    measured = tracker["history"][-1]
                    color = (0, 255, 0) if measured else (0, 255, 255)
                    state = "measured" if measured else "predicted"
                    center = tracker["center"]

                    cv2.circle(result_frame, tuple(center), 8, color, -1)
                    cv2.putText(
                        result_frame,
                        f"ID:{tid} {tracker['type']} {tracker['confidence']:.2f} {state}",
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
                f"YOLO: {len(detections)} targets, Tracked: {active_targets_count}, {inference_ms:.1f} ms",
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
