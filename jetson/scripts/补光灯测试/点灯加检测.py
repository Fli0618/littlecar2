"""开启 Jetson 补光灯后执行 YOLO 同心圆或颜色物料检测。"""

from __future__ import annotations

import time
from pathlib import Path

import cv2
import Jetson.GPIO as GPIO


PROJECT_ROOT = Path(__file__).resolve().parents[2]

from vision.yolo import detect_yolo, load_yolo_model


# 用户直接修改本区常量即可调整补光灯与视觉测试配置。
PWM_PIN = 33
PWM_FREQUENCY_HZ = 10000
PWM_DUTY_CYCLE_PERCENT = 50.0
LIGHT_SETTLE_SECONDS = 0.3

VISION_MODE = "circle"  # 可选 circle 或 color
CIRCLE_MODEL_PATH = PROJECT_ROOT / "assets" / "models" / "circle-with-number-v3.engine"
COLOR_MODEL_PATH = PROJECT_ROOT / "assets" / "models" / "6color-circle-v3.engine"

CAMERA_SOURCE = 0
CONF_THRESHOLD = 0.5
IMAGE_SIZE = 640
SHOW_WINDOW = True

WINDOW_NAME = "Light-assisted YOLO Detection | Press Q or ESC to exit"


def _model_path_for_mode() -> Path:
    if VISION_MODE not in ("circle", "color"):
        raise ValueError("VISION_MODE 必须为 'circle' 或 'color'")

    configured_path = CIRCLE_MODEL_PATH if VISION_MODE == "circle" else COLOR_MODEL_PATH
    model_path = Path(configured_path)
    if not model_path.is_absolute():
        script_relative = Path(__file__).resolve().parent / model_path
        model_path = script_relative if script_relative.is_file() else PROJECT_ROOT / model_path
    model_path = model_path.resolve()

    if model_path.suffix.lower() not in {".pt", ".engine"}:
        raise ValueError("模型文件扩展名必须为 .pt 或 .engine")
    if not model_path.is_file():
        raise FileNotFoundError(f"未找到 {VISION_MODE} 模型: {model_path}")
    return model_path


def _draw_detections(frame_bgr, detections: list[dict[str, object]], inference_ms: float):
    result_frame = frame_bgr.copy()
    for detection in detections:
        center_x = int(detection["center_x"])
        center_y = int(detection["center_y"])
        label = f"{detection['class_name']} {float(detection['confidence']):.2f}"
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

    cv2.putText(
        result_frame,
        f"{VISION_MODE}: {len(detections)} targets, {inference_ms:.1f} ms, imgsz={IMAGE_SIZE}",
        (20, 36),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    return result_frame


def main() -> None:
    if PWM_FREQUENCY_HZ <= 0:
        raise ValueError("PWM_FREQUENCY_HZ 必须大于 0")
    if not 0.0 <= PWM_DUTY_CYCLE_PERCENT <= 100.0:
        raise ValueError("PWM_DUTY_CYCLE_PERCENT 必须在 0 到 100 之间")
    if LIGHT_SETTLE_SECONDS < 0:
        raise ValueError("LIGHT_SETTLE_SECONDS 不能小于 0")
    if not 0.0 <= CONF_THRESHOLD <= 1.0:
        raise ValueError("CONF_THRESHOLD 必须在 0 到 1 之间")
    if IMAGE_SIZE <= 0:
        raise ValueError("IMAGE_SIZE 必须大于 0")

    camera = None
    pwm = None
    gpio_configured = False
    try:
        model_path = _model_path_for_mode()
        try:
            model = load_yolo_model(model_path)
        except Exception as exc:
            raise RuntimeError(f"加载 {VISION_MODE} 模型失败: {model_path}") from exc

        camera = cv2.VideoCapture(CAMERA_SOURCE, cv2.CAP_V4L2)
        if not camera.isOpened():
            raise RuntimeError(f"无法打开摄像头: {CAMERA_SOURCE}")

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(PWM_PIN, GPIO.OUT, initial=GPIO.LOW)
        gpio_configured = True
        # 补光灯独立供电时，控制地必须与 Jetson GND 共地。
        pwm = GPIO.PWM(PWM_PIN, PWM_FREQUENCY_HZ)
        pwm.start(PWM_DUTY_CYCLE_PERCENT)
        print(
            f"补光灯已开启: Pin {PWM_PIN}, {PWM_FREQUENCY_HZ} Hz, "
            f"占空比 {PWM_DUTY_CYCLE_PERCENT:.1f}%；等待 {LIGHT_SETTLE_SECONDS:.1f} 秒稳定曝光。"
        )
        time.sleep(LIGHT_SETTLE_SECONDS)

        if SHOW_WINDOW:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

        while True:
            ok, frame_bgr = camera.read()
            if not ok:
                print(f"摄像头读取失败: {CAMERA_SOURCE}，正在安全退出。")
                break

            started = time.perf_counter()
            detections = detect_yolo(
                frame_bgr,
                model,
                conf_thres=CONF_THRESHOLD,
                image_size=IMAGE_SIZE,
            )
            inference_ms = (time.perf_counter() - started) * 1000.0

            if SHOW_WINDOW:
                result_frame = _draw_detections(frame_bgr, detections, inference_ms)
                cv2.imshow(WINDOW_NAME, cv2.hconcat([frame_bgr, result_frame]))
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
    except KeyboardInterrupt:
        print("\n收到退出信号，正在关闭补光灯和检测。")
    finally:
        if pwm is not None:
            try:
                pwm.ChangeDutyCycle(0)
                pwm.stop()
            except Exception as exc:
                print(f"停止 PWM 时发生错误: {exc}")
        if gpio_configured:
            try:
                GPIO.output(PWM_PIN, GPIO.LOW)
            finally:
                GPIO.cleanup(PWM_PIN)
        if camera is not None:
            camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
