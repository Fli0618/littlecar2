"""实时显示相机画面与 YOLO 物料颜色检测结果。"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import cv2


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from vision import detect_color


CAMERA_INDEX = 0
WINDOW_NAME = "Material Color Detection | Press Q or ESC to exit"


def main() -> None:
    camera = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    if not camera.isOpened():
        raise RuntimeError(f"无法打开相机: {CAMERA_INDEX}")

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        while True:
            success, frame_bgr = camera.read()
            if not success:
                raise RuntimeError("无法读取相机画面")

            raw_frame = frame_bgr.copy()
            started = time.perf_counter()
            result = detect_color(frame_bgr)
            inference_ms = (time.perf_counter() - started) * 1000.0

            result_frame = frame_bgr.copy()
            for detection in result["detections"]:
                center_x, center_y = detection["center"]
                label = f"{detection['type']} {detection['confidence']:.2f}"
                cv2.circle(result_frame, (center_x, center_y), 7, (0, 255, 0), -1)
                cv2.putText(
                    result_frame,
                    label,
                    (center_x + 10, max(24, center_y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

            cv2.putText(raw_frame, "Camera", (20, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(
                result_frame,
                f"YOLO: {len(result['detections'])} targets, {inference_ms:.1f} ms",
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
