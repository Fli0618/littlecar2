"""Jetson vision test: run circle and color detection on the same frame."""

from __future__ import annotations

import json
import time

import cv2

from vision import advance_detect_circle, advance_detect_color, reset_advance_tracking


CAMERA_DEVICE = "/dev/video0"
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
DETECTION_PERIOD_MS = 40


def _print_result(name: str, result: dict[str, object], inference_ms: float) -> None:
    print(
        f"  {name}: {json.dumps(result, ensure_ascii=False, separators=(',', ':'))} "
        f"inference_ms={inference_ms:.1f}",
        flush=True,
    )


def main() -> None:
    camera = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"cannot open vision camera: {CAMERA_DEVICE}")

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    reset_advance_tracking()
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
            circle_started = time.perf_counter()
            circle_result = advance_detect_circle(frame_bgr)
            circle_ms = (time.perf_counter() - circle_started) * 1000.0

            color_started = time.perf_counter()
            color_result = advance_detect_color(frame_bgr)
            color_ms = (time.perf_counter() - color_started) * 1000.0

            total_ms = (time.perf_counter() - started) * 1000.0
            print(f"frame={frame_number} device={CAMERA_DEVICE} total_inference_ms={total_ms:.1f}", flush=True)
            _print_result("circle", circle_result, circle_ms)
            _print_result("color", color_result, color_ms)

            remaining = DETECTION_PERIOD_MS / 1000.0 - (time.perf_counter() - started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\nVision test stopped.", flush=True)
    finally:
        reset_advance_tracking()
        camera.release()


if __name__ == "__main__":
    main()
