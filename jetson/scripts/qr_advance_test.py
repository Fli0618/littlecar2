"""实时显示二维码相机画面及高级检测状态。"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import cv2


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from vision import QR_CONFIRM_COUNT, QR_MISSING_THRESHOLD, QR_WINDOW_SIZE, advance_detect_qr, reset_qr_tracking


CAMERA_QR_ID = 0
DETECTION_PERIOD_MS = 40
WINDOW_NAME = "Advance QR Detection | Press Q or ESC to exit"


def _draw_status(frame_bgr, result: dict[str, str | None], inference_ms: float) -> None:
    lines = (
        f"Raw: {result['raw_code']}",
        f"Status: {result['status']}",
        f"Return: {result['code']}",
        f"Window: {QR_WINDOW_SIZE}, Confirm: {QR_CONFIRM_COUNT}, Missing: {QR_MISSING_THRESHOLD}",
        f"Detection: {inference_ms:.1f} ms",
    )
    for index, text in enumerate(lines):
        color = (0, 255, 0) if index == 1 and result["code"] is not None else (255, 255, 255)
        cv2.putText(frame_bgr, text, (20, 36 + index * 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)


def main() -> None:
    camera = cv2.VideoCapture(CAMERA_QR_ID)
    if not camera.isOpened():
        raise RuntimeError(f"cannot open QR camera {CAMERA_QR_ID}")

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    try:
        while True:
            started = time.perf_counter()
            ok, frame_bgr = camera.read()
            if not ok:
                time.sleep(DETECTION_PERIOD_MS / 1000.0)
                continue
            try:
                result = advance_detect_qr(frame_bgr)
            except Exception:
                result = {"raw_code": None, "code": None, "status": "DETECTION_FAILED"}
            inference_ms = (time.perf_counter() - started) * 1000.0

            result_frame = frame_bgr.copy()
            _draw_status(result_frame, result, inference_ms)
            cv2.imshow(WINDOW_NAME, result_frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            remaining = DETECTION_PERIOD_MS / 1000.0 - (time.perf_counter() - started)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        reset_qr_tracking()
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
