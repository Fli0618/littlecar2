"""实时观察高级二维码检测状态，不创建图形窗口。"""

from __future__ import annotations

import sys
import time
from datetime import datetime
from pathlib import Path

import cv2


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from vision import QR_CONFIRM_COUNT, QR_MISSING_THRESHOLD, QR_WINDOW_SIZE, advance_detect_qr, reset_qr_tracking


CAMERA_QR_ID = 0
DETECTION_PERIOD_MS = 40


def main() -> None:
    camera = cv2.VideoCapture(CAMERA_QR_ID)
    if not camera.isOpened():
        raise RuntimeError(f"cannot open QR camera {CAMERA_QR_ID}")

    print(
        f"QR camera={CAMERA_QR_ID}, period={DETECTION_PERIOD_MS} ms, "
        f"window={QR_WINDOW_SIZE}, confirm={QR_CONFIRM_COUNT}, missing={QR_MISSING_THRESHOLD}"
    )
    try:
        while True:
            started = time.monotonic()
            ok, frame_bgr = camera.read()
            if not ok:
                print(f"[{datetime.now():%H:%M:%S.%f}] raw=None status=CAMERA_READ_FAILED return=None")
            else:
                try:
                    result = advance_detect_qr(frame_bgr)
                except Exception as error:
                    print(f"[{datetime.now():%H:%M:%S.%f}] raw=None status=DETECTION_FAILED return=None error={error}")
                else:
                    print(
                        f"[{datetime.now():%H:%M:%S.%f}] raw={result['raw_code']} "
                        f"status={result['status']} return={result['code']}"
                    )
            remaining = DETECTION_PERIOD_MS / 1000.0 - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\nQR test stopped")
    finally:
        reset_qr_tracking()
        camera.release()


if __name__ == "__main__":
    main()
