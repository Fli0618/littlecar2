"""Jetson QR camera test: print the advanced QR result without displaying video."""

from __future__ import annotations

import time

import cv2

from vision import advance_detect_qr, reset_qr_tracking


CAMERA_DEVICE = "/dev/video0"
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
DETECTION_PERIOD_MS = 40


def main() -> None:
    camera = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"cannot open QR camera: {CAMERA_DEVICE}")

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    reset_qr_tracking()
    frame_number = 0

    try:
        while True:
            started = time.perf_counter()
            ok, frame_bgr = camera.read()
            if not ok:
                print(f"frame={frame_number}: READ_FAILED device={CAMERA_DEVICE}", flush=True)
                time.sleep(DETECTION_PERIOD_MS / 1000.0)
                continue

            frame_number += 1
            result = advance_detect_qr(frame_bgr)
            inference_ms = (time.perf_counter() - started) * 1000.0
            print(
                f"frame={frame_number} device={CAMERA_DEVICE} "
                f"raw_code={result['raw_code']!r} code={result['code']!r} "
                f"status={result['status']} inference_ms={inference_ms:.1f}",
                flush=True,
            )

            remaining = DETECTION_PERIOD_MS / 1000.0 - (time.perf_counter() - started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\nQR test stopped.", flush=True)
    finally:
        reset_qr_tracking()
        camera.release()


if __name__ == "__main__":
    main()
