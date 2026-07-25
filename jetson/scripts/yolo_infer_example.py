from pathlib import Path
import sys

import cv2


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from vision import detect_color


IMAGE_PATH = PROJECT_ROOT / "assets" / "sim_train_00025.jpg"
OUTPUT_PATH = PROJECT_ROOT / "outputs" / "yolo_infer_example.jpg"


def main() -> None:
    frame = cv2.imread(str(IMAGE_PATH))
    if frame is None:
        raise SystemExit(f"无法读取图像: {IMAGE_PATH}")

    result = detect_color(frame)
    annotated = frame.copy()
    for detection in result["detections"]:
        center_x, center_y = detection["center"]
        cv2.circle(annotated, (center_x, center_y), 6, (0, 255, 0), -1)
        cv2.putText(
            annotated,
            f"{detection['type']} {detection['confidence']:.2f}",
            (center_x + 8, center_y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(OUTPUT_PATH), annotated)
    print(f"检测数量: {len(result['detections'])}")
    print(f"结果图已保存: {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
