"""Run YOLO material detection and material-disk center evaluation."""

from __future__ import annotations

import os
import random
import re
import sys
import time
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from vision import detect_color, detect_disk_center


DATASET_DIR = PROJECT_ROOT / "assets" / "物料盘"
NUM_TEST_IMAGES = 9
CONF_THRES = 0.5
IOU_THRES = 0.45
COLOR_CLASSES = {
    "Red": (230, 20, 30),
    "Yellow": (255, 215, 0),
    "Blue": (10, 60, 210),
    "Green": (0, 170, 50),
    "Black": (50, 50, 50),
    "LightBlue": (0, 191, 255),
    "EmptySlot": (120, 120, 120),
}


def parse_center_gt_from_filename(filename: str) -> tuple[int, int] | None:
    match = re.search(r"\((\d+),\s*(\d+)\)", filename)
    return (int(match.group(1)), int(match.group(2))) if match else None


def main() -> None:
    if not DATASET_DIR.exists():
        raise SystemExit(f"Dataset not found: {DATASET_DIR}")

    image_files = [
        path for path in DATASET_DIR.iterdir()
        if path.suffix.lower() in {".jpg", ".jpeg", ".png"}
    ]
    if not image_files:
        raise SystemExit(f"No images found in {DATASET_DIR}")

    selected_files = random.sample(image_files, min(NUM_TEST_IMAGES, len(image_files)))
    fig, axes = plt.subplots(3, 3, figsize=(15, 15))
    axes = np.asarray(axes).reshape(-1)
    latencies: list[float] = []
    errors: list[float] = []

    for index, image_path in enumerate(selected_files):
        image = cv2.imdecode(np.fromfile(str(image_path), dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            continue

        started = time.perf_counter()
        color_result = detect_color(image)
        detections = color_result["detections"]
        latency_ms = (time.perf_counter() - started) * 1000
        latencies.append(latency_ms)

        center = detect_disk_center(image, color_result)
        estimated = tuple(center["center"])
        ground_truth = parse_center_gt_from_filename(image_path.name)
        error = None
        if ground_truth is not None:
            error = float(np.linalg.norm(np.asarray(estimated) - np.asarray(ground_truth)))
            errors.append(error)

        axis = axes[index]
        axis.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        for detection in detections:
            color = np.asarray(COLOR_CLASSES.get(detection["type"], (128, 128, 128))) / 255
            axis.plot(*detection["center"], "o", color=color, markersize=5)

        if center["status"] == 3:
            axis.add_patch(plt.Polygon(center["support_points"], closed=True, fill=False, color="cyan", linestyle="--"))
        axis.plot(*estimated, marker="x", color="red", markersize=14, markeredgewidth=3)
        if ground_truth is not None:
            axis.plot(*ground_truth, marker="+", color="lime", markersize=14, markeredgewidth=2)
        axis.axis("off")
        error_text = f"{error:.1f}px" if error is not None else "N/A"
        axis.set_title(f"{image_path.name}\n{latency_ms:.1f} ms | error {error_text}\nstatus={center['status']}")

    for axis in axes[len(selected_files):]:
        axis.axis("off")

    average_latency = np.mean(latencies) if latencies else 0.0
    average_error = np.mean(errors) if errors else 0.0
    plt.suptitle(f"YOLO disk center evaluation | {average_latency:.1f} ms | {average_error:.1f}px")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
