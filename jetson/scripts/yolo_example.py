from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


PROJECT_ROOT = Path(__file__).resolve().parents[1]
COLOR_MODEL_PATH = PROJECT_ROOT / "assets" / "models" / "6color-circle-v3.pt"
CIRCLE_MODEL_PATH = PROJECT_ROOT / "assets" / "models" / "circle-with-number-v3.pt"
COLOR_IMAGE_PATH = next((PROJECT_ROOT / "assets" / "物料盘").glob("*.jpg"))
CIRCLE_IMAGE_PATH = next((PROJECT_ROOT / "assets" / "circle_with_number").glob("*.jpg"))


def main() -> None:
    color_model = YOLO(str(COLOR_MODEL_PATH))
    circle_model = YOLO(str(CIRCLE_MODEL_PATH))

    for name, model, image_path in (
        ("颜色物料", color_model, COLOR_IMAGE_PATH),
        ("带数字同心圆", circle_model, CIRCLE_IMAGE_PATH),
    ):
        frame = cv2.imdecode(np.fromfile(str(image_path), dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError(f"无法读取示例图片: {image_path}")

        result = model.predict(source=frame, conf=0.5, iou=0.45, verbose=False)[0]
        detections = []
        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                detections.append(
                    {
                        "type": str(result.names[class_id]),
                        "center": [int((x1 + x2) / 2), int((y1 + y2) / 2)],
                        "confidence": float(box.conf[0]),
                    }
                )
        print(f"{name}: {detections}")


if __name__ == "__main__":
    main()
