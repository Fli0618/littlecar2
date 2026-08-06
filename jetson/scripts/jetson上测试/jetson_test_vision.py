"""Jetson vision test: run circle and color detection on the same frame."""

from __future__ import annotations

import json
import time

import cv2
import Jetson.GPIO as GPIO

from vision import (
    advance_detect_circle,
    advance_detect_color,
    advance_detect_color_hybrid,
    configure_model_backend,
    reset_advance_tracking,
)


CAMERA_DEVICE = "/dev/video2"
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
DETECTION_PERIOD_MS = 40
PWM_PIN = 32  # BOARD 编号：40 针排针的物理脚 32，也就是 GPIO07
PWM_FREQUENCY_HZ = 10000
PWM_DUTY_CYCLE_PERCENT = 100.0
LIGHT_SETTLE_SECONDS = 0.3
# 与 main.py 保持一致：正式颜色链路使用 YOLO 候选框和 HSV 分类。
COLOR_DETECTION_BACKEND = "yolo_hsv"
WINDOW_NAME = "Jetson Vision Detection"
COLOR_BOX = (0, 220, 0)
CIRCLE_BOX = (255, 80, 0)


def _color_detection_backend():
    """返回当前测试脚本选择的颜色检测后端，不自动回退。"""
    backends = {
        "yolo": advance_detect_color,
        "yolo_hsv": advance_detect_color_hybrid,
    }
    try:
        return backends[COLOR_DETECTION_BACKEND]
    except KeyError as exc:
        raise ValueError(f"unsupported color detection backend: {COLOR_DETECTION_BACKEND!r}") from exc


def _print_result(name: str, result: dict[str, object], inference_ms: float) -> None:
    print(
        f"  {name}: {json.dumps(result, ensure_ascii=False, separators=(',', ':'))} "
        f"inference_ms={inference_ms:.1f}",
        flush=True,
    )


def _draw_detections(frame_bgr, result: dict[str, object], name: str, color) -> None:
    """在相机帧上绘制高级检测结果中的跟踪框和状态。"""
    detections = result.get("detections", [])
    if not isinstance(detections, list):
        return

    for detection in detections:
        if not isinstance(detection, dict):
            continue
        bbox = detection.get("bbox")
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            continue
        x1, y1, x2, y2 = (int(round(value)) for value in bbox)
        cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), color, 2)
        center = detection.get("center")
        if isinstance(center, (list, tuple)) and len(center) == 2:
            cv2.drawMarker(
                frame_bgr,
                (int(center[0]), int(center[1])),
                color,
                cv2.MARKER_TILTED_CROSS,
                16,
                2,
            )
        label = (
            f"{name} type={detection.get('type', '-')} "
            f"conf={float(detection.get('confidence', 0.0)):.2f} "
            f"{'measured' if detection.get('measured', False) else 'predicted'}"
        )
        text_y = max(18, y1 - 8)
        cv2.putText(frame_bgr, label, (max(0, x1), text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)


def _start_fill_light():
    """初始化物理脚 32 的 PWM 补光灯并以最大亮度开启。"""
    pwm = GPIO.PWM(PWM_PIN, PWM_FREQUENCY_HZ)
    pwm.start(PWM_DUTY_CYCLE_PERCENT)
    print(
        f"fill light enabled pin={PWM_PIN} frequency_hz={PWM_FREQUENCY_HZ} "
        f"duty_cycle_percent={PWM_DUTY_CYCLE_PERCENT:.1f}",
        flush=True,
    )
    return pwm


def _stop_fill_light(pwm, gpio_configured: bool) -> None:
    """将补光灯置低并释放 GPIO，避免视觉测试退出后灯常亮。"""
    if pwm is not None:
        try:
            pwm.ChangeDutyCycle(0)
            pwm.stop()
        except Exception as error:
            print(f"fill light stop failed: {error}", flush=True)
    if gpio_configured:
        try:
            GPIO.output(PWM_PIN, GPIO.LOW)
        finally:
            GPIO.cleanup(PWM_PIN)


def main() -> None:
    configure_model_backend("engine")
    color_detector = _color_detection_backend()
    print(
        f"color_detection_backend={COLOR_DETECTION_BACKEND}",
        flush=True,
    )
    camera = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"cannot open vision camera: {CAMERA_DEVICE}")

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    reset_advance_tracking()
    frame_number = 0
    pwm = None
    gpio_configured = False

    try:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(PWM_PIN, GPIO.OUT, initial=GPIO.LOW)
        gpio_configured = True
        pwm = _start_fill_light()
        time.sleep(LIGHT_SETTLE_SECONDS)

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
            color_result = color_detector(frame_bgr)
            color_ms = (time.perf_counter() - color_started) * 1000.0
            total_ms = (time.perf_counter() - started) * 1000.0

            display_frame = frame_bgr.copy()
            _draw_detections(display_frame, circle_result, "CIRCLE", CIRCLE_BOX)
            _draw_detections(display_frame, color_result, "COLOR", COLOR_BOX)
            cv2.putText(
                display_frame,
                f"frame={frame_number} color={COLOR_DETECTION_BACKEND} "
                f"total={total_ms:.1f}ms | q/Esc quit",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.imshow(WINDOW_NAME, display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                print("\nVision test stopped by window keyboard.", flush=True)
                break

            print(f"frame={frame_number} device={CAMERA_DEVICE} total_inference_ms={total_ms:.1f}", flush=True)
            _print_result("circle", circle_result, circle_ms)
            _print_result("color", color_result, color_ms)

            remaining = DETECTION_PERIOD_MS / 1000.0 - (time.perf_counter() - started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\nVision test stopped.", flush=True)
    finally:
        _stop_fill_light(pwm, gpio_configured)
        reset_advance_tracking()
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
