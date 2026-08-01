"""分别对物料颜色与带数字同心圆模型执行批量推理压力测试。"""

from __future__ import annotations

import random
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from vision.yolo import detect_yolo, load_yolo_model


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIDENCE = 0.5
IOU = 0.45
SAMPLE_COUNT = 200

TESTS = (
    (
        "颜色物料",
        PROJECT_ROOT / "assets" / "models" / "6color-circle-v3.engine",
        PROJECT_ROOT / "assets" / "彩色物料数据集v3" / "images",
    ),
    (
        "带数字同心圆",
        PROJECT_ROOT / "assets" / "models" / "circle-with-number-v3.engine",
        PROJECT_ROOT / "assets" / "circle_with_number_v3" / "images",
    ),
)


def run_benchmark(name: str, model: object, image_dir: Path) -> None:
    image_paths = [
        path
        for path in image_dir.rglob("*")
        if path.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"}
    ]
    if not image_paths:
        raise FileNotFoundError(f"未找到测试图片: {image_dir}")

    selected_paths = random.sample(image_paths, min(SAMPLE_COUNT, len(image_paths)))
    latencies_ms: list[float] = []
    detection_counts: list[int] = []
    class_counts: Counter[str] = Counter()

    for index, image_path in enumerate(selected_paths, start=1):
        frame = cv2.imdecode(np.fromfile(str(image_path), dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue

        started = time.perf_counter()
        detections = detect_yolo(frame, model, conf_thres=CONFIDENCE, iou_thres=IOU)
        latencies_ms.append((time.perf_counter() - started) * 1000.0)

        detection_counts.append(len(detections))
        for detection in detections:
            class_counts[str(detection["class_name"])] += 1

        if index % 20 == 0 or index == len(selected_paths):
            print(f"{name}: {index}/{len(selected_paths)}")

    if not latencies_ms:
        raise RuntimeError(f"{name} 没有成功读取任何测试图片")

    print(f"\n{name} 压力测试结果")
    print(f"样本数: {len(latencies_ms)}")
    print(f"平均推理耗时: {sum(latencies_ms) / len(latencies_ms):.2f} ms")
    print(f"最小/最大推理耗时: {min(latencies_ms):.2f} / {max(latencies_ms):.2f} ms")
    print(f"平均检测目标数: {sum(detection_counts) / len(detection_counts):.2f}")
    print(f"类别检测总数: {dict(class_counts)}")


def main() -> None:
    models = []
    for name, model_path, image_dir in TESTS:
        if not model_path.is_file():
            raise FileNotFoundError(f"未找到 {name} 模型: {model_path}")
        models.append((name, load_yolo_model(model_path), image_dir))

    print("推理后端: TensorRT engine")
    for name, model, image_dir in models:
        run_benchmark(name, model, image_dir)


if __name__ == "__main__":
    main()
