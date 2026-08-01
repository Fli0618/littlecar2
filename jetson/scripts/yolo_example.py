from pathlib import Path

import cv2
import numpy as np
from vision.yolo import detect_yolo, load_yolo_model


PROJECT_ROOT = Path(__file__).resolve().parents[1]
COLOR_MODEL_PATH = PROJECT_ROOT / "assets" / "models" / "6color-circle-v3.engine"
CIRCLE_MODEL_PATH = PROJECT_ROOT / "assets" / "models" / "circle-with-number-v3.engine"
COLOR_IMAGE_PATH = next((PROJECT_ROOT / "assets" / "物料盘").glob("*.jpg"))
CIRCLE_IMAGE_PATH = next((PROJECT_ROOT / "assets" / "circle_with_number").glob("*.jpg"))


def main() -> None:
    color_model = load_yolo_model(COLOR_MODEL_PATH)
    circle_model = load_yolo_model(CIRCLE_MODEL_PATH)

    for name, model, image_path in (
        ("颜色物料", color_model, COLOR_IMAGE_PATH),
        ("带数字同心圆", circle_model, CIRCLE_IMAGE_PATH),
    ):
        frame = cv2.imdecode(np.fromfile(str(image_path), dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError(f"无法读取示例图片: {image_path}")

        raw_detections = detect_yolo(frame, model, conf_thres=0.5, iou_thres=0.45)
        detections = [
            {"type": item["class_name"], "center": [item["center_x"], item["center_y"]], "confidence": item["confidence"]}
            for item in raw_detections
        ]
        print(f"{name}: {detections}")


if __name__ == "__main__":
    main()
