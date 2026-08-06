#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从已标注照片生成颜色物料 YOLO 数据集。

默认：
- 生成总数：1500 张
- 划分比例：train:val:test = 7:2:1
- 输出尺寸：1000x750（保持 4:3，最长边 1000）
- 纯背景负样本比例：10%
- 每张合成图目标数：1~5
- 目标最长边：直接使用像素设置，默认 24~96 px（可通过参数修改）
- 可使用 --object-size-px 设置固定目标尺寸
- 目标之间：严格不重叠，默认最小间距 4 px

输入目录约定：
照片根目录/
├── IMG_001.jpg
├── IMG_002.jpg
├── ...
├── 标注/
│   ├── IMG_001.txt
│   ├── IMG_002.txt
│   └── ...
└── 背景/
    ├── BG_001.jpg
    └── ...

运行方式一：命令行参数
    python generate_color_material_dataset.py ^
        --photos "F:\\path\\物料照片" ^
        --output "F:\\path\\color_dataset" ^
        --overwrite

运行方式二：不传参数，按终端提示输入路径
    python generate_color_material_dataset.py

依赖：
    python -m pip install opencv-python numpy

类别顺序固定：
    0 Red
    1 Yellow
    2 Blue
    3 Green
    4 Black
    5 LightBlue
    6 EmptySlot

说明：
- 默认不将 EmptySlot 作为可移动 cutout，因为它依赖槽位/转盘背景结构。
- data.yaml 仍保留 7 类，确保类别编号与现有模型、Jetson、STM32 一致。
- 该脚本生成的是合成数据集。真实部署效果仍应使用独立真实图片验证。
"""

from __future__ import annotations

import argparse
import math
import random
import shutil
import sys
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import TypeVar

import cv2
import numpy as np


CLASS_NAMES = [
    "Red",
    "Yellow",
    "Blue",
    "Green",
    "Black",
    "LightBlue",
    "EmptySlot",
]

MOVABLE_CLASS_IDS = {0, 1, 2, 3, 4, 5}
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

TARGET_WIDTH = 1000
TARGET_HEIGHT = 750

DEFAULT_TOTAL = 1500
DEFAULT_SPLIT = (0.7, 0.2, 0.1)
DEFAULT_NEGATIVE_RATIO = 0.10
DEFAULT_MIN_OBJECTS = 1
DEFAULT_MAX_OBJECTS = 5
DEFAULT_SEED = 42
DEFAULT_JPEG_QUALITY = 95
DEFAULT_MIN_OBJECT_PX = 24
DEFAULT_MAX_OBJECT_PX = 96
DEFAULT_MIN_OBJECT_GAP_PX = 4
SCRIPT_VERSION = "4.0-output-1000x750"


@dataclass(frozen=True)
class Box:
    class_id: int
    x1: float
    y1: float
    x2: float
    y2: float

    def normalized(self) -> "Box":
        return Box(
            self.class_id,
            min(self.x1, self.x2),
            min(self.y1, self.y2),
            max(self.x1, self.x2),
            max(self.y1, self.y2),
        )

    def clipped(self, width: int, height: int) -> "Box":
        box = self.normalized()
        return Box(
            box.class_id,
            max(0.0, min(float(width), box.x1)),
            max(0.0, min(float(height), box.y1)),
            max(0.0, min(float(width), box.x2)),
            max(0.0, min(float(height), box.y2)),
        )

    @property
    def width(self) -> float:
        box = self.normalized()
        return max(0.0, box.x2 - box.x1)

    @property
    def height(self) -> float:
        box = self.normalized()
        return max(0.0, box.y2 - box.y1)

    @property
    def area(self) -> float:
        return self.width * self.height

    def to_yolo(self, width: int, height: int) -> str:
        box = self.clipped(width, height)
        center_x = ((box.x1 + box.x2) * 0.5) / width
        center_y = ((box.y1 + box.y2) * 0.5) / height
        box_width = (box.x2 - box.x1) / width
        box_height = (box.y2 - box.y1) / height
        return (
            f"{box.class_id} "
            f"{center_x:.6f} {center_y:.6f} "
            f"{box_width:.6f} {box_height:.6f}"
        )

    @staticmethod
    def from_yolo(line: str, width: int, height: int) -> "Box":
        parts = line.strip().split()
        if len(parts) != 5:
            raise ValueError(f"字段数应为 5，实际为 {len(parts)}：{line!r}")

        class_id = int(parts[0])
        center_x, center_y, box_width, box_height = map(float, parts[1:])

        if not 0 <= class_id < len(CLASS_NAMES):
            raise ValueError(f"class_id={class_id} 不在 0~{len(CLASS_NAMES) - 1}")
        if not all(math.isfinite(value) for value in (center_x, center_y, box_width, box_height)):
            raise ValueError("标签包含非有限数值")
        if box_width <= 0.0 or box_height <= 0.0:
            raise ValueError("bbox 宽高必须大于 0")

        x1 = (center_x - box_width * 0.5) * width
        y1 = (center_y - box_height * 0.5) * height
        x2 = (center_x + box_width * 0.5) * width
        y2 = (center_y + box_height * 0.5) * height
        return Box(class_id, x1, y1, x2, y2).clipped(width, height)


@dataclass
class SourceImage:
    image_path: Path
    label_path: Path
    image: np.ndarray
    boxes: list[Box]


@dataclass
class Cutout:
    class_id: int
    image: np.ndarray
    alpha: np.ndarray
    source_name: str
    mask_method: str


@dataclass
class GenerationStats:
    images: int = 0
    labels: int = 0
    empty_labels: int = 0
    object_instances: int = 0
    placement_failures: int = 0
    class_instances: Counter[int] | None = None

    def __post_init__(self) -> None:
        if self.class_instances is None:
            self.class_instances = Counter()


def log(message: str) -> None:
    print(message, flush=True)


def read_image(path: Path) -> np.ndarray | None:
    """支持 Windows 中文路径。"""
    try:
        raw = np.fromfile(str(path), dtype=np.uint8)
        if raw.size == 0:
            return None
        return cv2.imdecode(raw, cv2.IMREAD_COLOR)
    except Exception:
        return None


def write_image(path: Path, image: np.ndarray, quality: int = DEFAULT_JPEG_QUALITY) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    suffix = path.suffix.lower()

    if suffix in {".jpg", ".jpeg"}:
        extension = ".jpg"
        params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    elif suffix == ".png":
        extension = ".png"
        params = [int(cv2.IMWRITE_PNG_COMPRESSION), 3]
    else:
        extension = ".jpg"
        params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]

    ok, encoded = cv2.imencode(extension, image, params)
    if not ok:
        raise RuntimeError(f"图像编码失败：{path}")
    encoded.tofile(str(path))


def write_labels(path: Path, boxes: list[Box]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    valid_boxes = [
        box.clipped(TARGET_WIDTH, TARGET_HEIGHT)
        for box in boxes
        if box.area >= 16.0
    ]
    lines = [
        box.to_yolo(TARGET_WIDTH, TARGET_HEIGHT)
        for box in valid_boxes
    ]
    path.write_text(
        "\n".join(lines) + ("\n" if lines else ""),
        encoding="utf-8",
    )


def find_subdirectory(root: Path, candidates: tuple[str, ...]) -> Path | None:
    for name in candidates:
        path = root / name
        if path.is_dir():
            return path
    return None


def list_images(directory: Path, recursive: bool = True) -> list[Path]:
    iterator = directory.rglob("*") if recursive else directory.iterdir()
    return sorted(
        path
        for path in iterator
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )


def list_source_images(photo_root: Path, label_dir: Path, background_dir: Path) -> list[Path]:
    """
    扫描照片根目录及普通子目录，但排除标注、背景和常见输出目录。
    """
    excluded = {
        label_dir.resolve(),
        background_dir.resolve(),
    }
    excluded_names = {
        "images", "labels", "preview", "dataset",
        "train", "val", "test", "__pycache__",
    }

    result: list[Path] = []
    for path in photo_root.rglob("*"):
        if not path.is_file() or path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue

        parents = {parent.resolve() for parent in path.parents}
        if any(excluded_path in parents for excluded_path in excluded):
            continue
        if any(part.lower() in excluded_names for part in path.parts):
            continue

        # 标签按 stem 匹配，必须存在才视为已标注输入。
        if (label_dir / f"{path.stem}.txt").is_file():
            result.append(path)

    return sorted(result)


def load_label_file(path: Path, width: int, height: int) -> list[Box]:
    boxes: list[Box] = []
    lines = path.read_text(encoding="utf-8-sig").splitlines()

    for line_number, line in enumerate(lines, 1):
        stripped = line.strip()
        if not stripped:
            continue
        try:
            boxes.append(Box.from_yolo(stripped, width, height))
        except Exception as exc:
            raise ValueError(f"{path}:{line_number}: {exc}") from exc

    return boxes


def load_sources(photo_root: Path, label_dir: Path, background_dir: Path) -> list[SourceImage]:
    image_paths = list_source_images(photo_root, label_dir, background_dir)
    if not image_paths:
        raise RuntimeError(
            "没有找到“图片 + 同名标注文件”的有效组合。\n"
            f"预期标签目录：{label_dir}\n"
            "例如：IMG_001.jpg 对应 标注/IMG_001.txt"
        )

    sources: list[SourceImage] = []
    invalid_count = 0

    log("\n[1/7] 检查标注照片")
    for index, image_path in enumerate(image_paths, 1):
        image = read_image(image_path)
        if image is None:
            log(f"  [跳过] 无法读取图片：{image_path}")
            invalid_count += 1
            continue

        height, width = image.shape[:2]
        label_path = label_dir / f"{image_path.stem}.txt"

        try:
            boxes = load_label_file(label_path, width, height)
        except Exception as exc:
            log(f"  [跳过] 标签错误：{exc}")
            invalid_count += 1
            continue

        if not boxes:
            log(f"  [提示] 空标签图片：{image_path.name}")

        sources.append(SourceImage(image_path, label_path, image, boxes))

        if index <= 5 or index == len(image_paths):
            log(
                f"  [{index:>3}/{len(image_paths)}] "
                f"{image_path.name} | {width}x{height} | boxes={len(boxes)}"
            )

    if not sources:
        raise RuntimeError("所有标注照片均读取失败或标签无效")

    log(f"  有效标注照片：{len(sources)}")
    log(f"  跳过文件：{invalid_count}")
    return sources


def load_backgrounds(background_dir: Path) -> list[np.ndarray]:
    paths = list_images(background_dir, recursive=True)
    if not paths:
        raise RuntimeError(f"背景目录中没有图片：{background_dir}")

    backgrounds: list[np.ndarray] = []
    failed = 0

    log("\n[2/7] 读取无物料背景")
    for index, path in enumerate(paths, 1):
        image = read_image(path)
        if image is None:
            log(f"  [跳过] 无法读取背景：{path}")
            failed += 1
            continue
        backgrounds.append(resize_cover(image, TARGET_WIDTH, TARGET_HEIGHT))

        if index <= 5 or index == len(paths):
            height, width = image.shape[:2]
            log(f"  [{index:>3}/{len(paths)}] {path.name} | 原始={width}x{height}")

    if not backgrounds:
        raise RuntimeError("所有背景图片均读取失败")

    log(f"  有效背景：{len(backgrounds)}")
    log(f"  读取失败：{failed}")
    return backgrounds


def resize_cover(image: np.ndarray, width: int, height: int) -> np.ndarray:
    """等比例缩放并中心裁剪成固定尺寸，避免拉伸背景。"""
    source_height, source_width = image.shape[:2]
    scale = max(width / source_width, height / source_height)

    resized_width = max(width, int(round(source_width * scale)))
    resized_height = max(height, int(round(source_height * scale)))

    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=interpolation,
    )

    x1 = (resized_width - width) // 2
    y1 = (resized_height - height) // 2
    return resized[y1:y1 + height, x1:x1 + width].copy()


def keep_largest_component(mask: np.ndarray) -> np.ndarray:
    binary = (mask > 0).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        binary,
        connectivity=8,
    )
    if count <= 1:
        return binary * 255

    largest_id = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return np.where(labels == largest_id, 255, 0).astype(np.uint8)


def trim_cutout(
    image: np.ndarray,
    alpha: np.ndarray,
    margin: int = 2,
) -> tuple[np.ndarray, np.ndarray] | None:
    ys, xs = np.where(alpha > 12)
    if len(xs) < 20:
        return None

    x1 = max(0, int(xs.min()) - margin)
    y1 = max(0, int(ys.min()) - margin)
    x2 = min(alpha.shape[1], int(xs.max()) + margin + 1)
    y2 = min(alpha.shape[0], int(ys.max()) + margin + 1)

    return image[y1:y2, x1:x2].copy(), alpha[y1:y2, x1:x2].copy()


def ellipse_fallback(
    crop_height: int,
    crop_width: int,
    rect_x: int,
    rect_y: int,
    rect_width: int,
    rect_height: int,
) -> np.ndarray:
    alpha = np.zeros((crop_height, crop_width), dtype=np.uint8)
    center = (
        rect_x + rect_width // 2,
        rect_y + rect_height // 2,
    )
    axes = (
        max(2, int(rect_width * 0.50)),
        max(2, int(rect_height * 0.50)),
    )
    cv2.ellipse(
        alpha,
        center,
        axes,
        0,
        0,
        360,
        255,
        thickness=-1,
    )
    return alpha


def extract_cutout(source: SourceImage, box: Box) -> Cutout | None:
    if box.class_id not in MOVABLE_CLASS_IDS:
        return None

    image = source.image
    image_height, image_width = image.shape[:2]
    box = box.clipped(image_width, image_height).normalized()

    if box.width < 8.0 or box.height < 8.0:
        return None

    padding_x = max(4, int(round(box.width * 0.10)))
    padding_y = max(4, int(round(box.height * 0.10)))

    crop_x1 = max(0, int(math.floor(box.x1)) - padding_x)
    crop_y1 = max(0, int(math.floor(box.y1)) - padding_y)
    crop_x2 = min(image_width, int(math.ceil(box.x2)) + padding_x)
    crop_y2 = min(image_height, int(math.ceil(box.y2)) + padding_y)

    crop = image[crop_y1:crop_y2, crop_x1:crop_x2].copy()
    crop_height, crop_width = crop.shape[:2]
    if crop_width < 10 or crop_height < 10:
        return None

    rect_x = max(1, int(round(box.x1)) - crop_x1)
    rect_y = max(1, int(round(box.y1)) - crop_y1)
    rect_width = min(
        crop_width - rect_x - 1,
        max(2, int(round(box.width))),
    )
    rect_height = min(
        crop_height - rect_y - 1,
        max(2, int(round(box.height))),
    )
    if rect_width < 2 or rect_height < 2:
        return None

    grabcut_mask = np.full(
        (crop_height, crop_width),
        cv2.GC_BGD,
        dtype=np.uint8,
    )
    grabcut_mask[
        rect_y:rect_y + rect_height,
        rect_x:rect_x + rect_width,
    ] = cv2.GC_PR_FGD

    # 圆形/圆柱物料的中心区域作为确定前景，提高稳定性。
    sure_foreground = np.zeros((crop_height, crop_width), dtype=np.uint8)
    cv2.ellipse(
        sure_foreground,
        (
            rect_x + rect_width // 2,
            rect_y + rect_height // 2,
        ),
        (
            max(2, int(rect_width * 0.30)),
            max(2, int(rect_height * 0.30)),
        ),
        0,
        0,
        360,
        255,
        thickness=-1,
    )
    grabcut_mask[sure_foreground > 0] = cv2.GC_FGD

    method = "grabcut"
    try:
        background_model = np.zeros((1, 65), dtype=np.float64)
        foreground_model = np.zeros((1, 65), dtype=np.float64)
        cv2.grabCut(
            crop,
            grabcut_mask,
            None,
            background_model,
            foreground_model,
            5,
            cv2.GC_INIT_WITH_MASK,
        )
        alpha = np.where(
            (grabcut_mask == cv2.GC_FGD)
            | (grabcut_mask == cv2.GC_PR_FGD),
            255,
            0,
        ).astype(np.uint8)
    except cv2.error:
        alpha = ellipse_fallback(
            crop_height,
            crop_width,
            rect_x,
            rect_y,
            rect_width,
            rect_height,
        )
        method = "ellipse_fallback"

    kernel = np.ones((3, 3), dtype=np.uint8)
    alpha = cv2.morphologyEx(alpha, cv2.MORPH_OPEN, kernel)
    alpha = cv2.morphologyEx(
        alpha,
        cv2.MORPH_CLOSE,
        kernel,
        iterations=2,
    )
    alpha = keep_largest_component(alpha)

    foreground_ratio = (
        float(np.count_nonzero(alpha))
        / float(max(1, rect_width * rect_height))
    )

    # GrabCut 明显失败时，保证仍能生成可用素材。
    if foreground_ratio < 0.25 or foreground_ratio > 1.35:
        alpha = ellipse_fallback(
            crop_height,
            crop_width,
            rect_x,
            rect_y,
            rect_width,
            rect_height,
        )
        method = "ellipse_fallback"

    alpha = cv2.GaussianBlur(alpha, (0, 0), sigmaX=1.0)

    trimmed = trim_cutout(crop, alpha)
    if trimmed is None:
        return None

    trimmed_image, trimmed_alpha = trimmed
    return Cutout(
        class_id=box.class_id,
        image=trimmed_image,
        alpha=trimmed_alpha,
        source_name=source.image_path.name,
        mask_method=method,
    )


def build_cutouts(sources: list[SourceImage]) -> list[Cutout]:
    cutouts: list[Cutout] = []
    method_counter: Counter[str] = Counter()
    class_counter: Counter[int] = Counter()
    failed = 0
    ignored_empty_slot = 0

    log("\n[3/7] 从标注框提取物料 cutout")
    total_boxes = sum(len(source.boxes) for source in sources)
    processed = 0

    for source in sources:
        for box in source.boxes:
            processed += 1

            if box.class_id == 6:
                ignored_empty_slot += 1
                continue

            cutout = extract_cutout(source, box)
            if cutout is None:
                failed += 1
                log(
                    f"  [失败] {source.image_path.name} "
                    f"class={box.class_id}:{CLASS_NAMES[box.class_id]}"
                )
                continue

            cutouts.append(cutout)
            method_counter[cutout.mask_method] += 1
            class_counter[cutout.class_id] += 1

            if processed % 50 == 0 or processed == total_boxes:
                log(f"  处理进度：{processed}/{total_boxes}")

    if not cutouts:
        raise RuntimeError(
            "没有成功提取任何 0~5 类物料 cutout。\n"
            "请确认标注文件内容正确，且标注框覆盖了颜色物料。"
        )

    log(f"  成功 cutout：{len(cutouts)}")
    log(f"  提取失败：{failed}")
    log(f"  忽略 EmptySlot：{ignored_empty_slot}")
    log(
        "  mask 方法："
        + ", ".join(f"{name}={count}" for name, count in sorted(method_counter.items()))
    )

    log("  cutout 类别分布：")
    for class_id, class_name in enumerate(CLASS_NAMES):
        log(f"    {class_id} {class_name:<10} : {class_counter[class_id]}")

    missing = [
        CLASS_NAMES[class_id]
        for class_id in MOVABLE_CLASS_IDS
        if class_counter[class_id] == 0
    ]
    if missing:
        log(
            "  [警告] 以下类别没有素材，不会出现在合成图中："
            + ", ".join(missing)
        )

    return cutouts


def parse_split(value: str) -> tuple[float, float, float]:
    cleaned = value.replace("：", ":").replace(",", ":").replace("/", ":")
    parts = [part.strip() for part in cleaned.split(":") if part.strip()]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            "split 应为三个数，例如 7:2:1 或 0.7:0.2:0.1"
        )

    try:
        values = tuple(float(part) for part in parts)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("split 包含非法数字") from exc

    if any(value < 0.0 for value in values):
        raise argparse.ArgumentTypeError("split 不能包含负数")

    total = sum(values)
    if total <= 0.0:
        raise argparse.ArgumentTypeError("split 总和必须大于 0")

    return (
        values[0] / total,
        values[1] / total,
        values[2] / total,
    )


def allocate_counts(
    total: int,
    ratios: tuple[float, float, float],
) -> dict[str, int]:
    names = ("train", "val", "test")
    exact = [total * ratio for ratio in ratios]
    counts = [int(math.floor(value)) for value in exact]

    remaining = total - sum(counts)
    fractional_order = sorted(
        range(3),
        key=lambda index: exact[index] - counts[index],
        reverse=True,
    )
    for index in fractional_order[:remaining]:
        counts[index] += 1

    return dict(zip(names, counts))


T = TypeVar("T")


def split_items(
    items: list[T],
    ratios: tuple[float, float, float],
    seed: int,
) -> dict[str, list[T]]:
    shuffled = list(items)
    random.Random(seed).shuffle(shuffled)
    counts = allocate_counts(len(shuffled), ratios)

    result: dict[str, list[T]] = {}
    cursor = 0
    for split_name in ("train", "val", "test"):
        count = counts[split_name]
        result[split_name] = shuffled[cursor:cursor + count]
        cursor += count

    # 小型数据集下尽量避免某个 split 完全没有资源。
    nonempty_splits = [
        name for name in ("train", "val", "test")
        if counts[name] > 0
    ]
    for split_name in nonempty_splits:
        if result[split_name]:
            continue

        donor = max(
            nonempty_splits,
            key=lambda name: len(result[name]),
        )
        if len(result[donor]) > 1:
            result[split_name].append(result[donor].pop())

    return result


def transform_cutout(
    cutout: Cutout,
    rng: random.Random,
    min_object_px: int,
    max_object_px: int,
) -> Cutout:
    """
    对 cutout 做颜色扰动和旋转，再按像素直接缩放。

    min_object_px / max_object_px 表示最终可见目标 bbox 最长边范围。
    例如设置 24~80，最终物料最长边约为 24~80 像素，
    与原始素材拍摄距离和裁剪尺寸无关。
    """
    image = cutout.image.astype(np.float32)

    contrast = rng.uniform(0.90, 1.10)
    brightness = rng.uniform(0.88, 1.12)
    image = (image - 127.5) * contrast + 127.5
    image *= brightness
    image = np.clip(image, 0, 255).astype(np.uint8)

    if rng.random() < 0.20:
        sigma = rng.uniform(0.25, 0.90)
        image = cv2.GaussianBlur(image, (0, 0), sigmaX=sigma)

    # 先旋转，再根据旋转后的 alpha 外接框执行绝对缩放。
    angle = rng.uniform(-180.0, 180.0)
    source_height, source_width = image.shape[:2]
    center = (source_width / 2.0, source_height / 2.0)
    matrix = cv2.getRotationMatrix2D(center, angle, 1.0)

    cos_value = abs(matrix[0, 0])
    sin_value = abs(matrix[0, 1])
    rotated_width = max(
        2,
        int(round(source_height * sin_value + source_width * cos_value)),
    )
    rotated_height = max(
        2,
        int(round(source_height * cos_value + source_width * sin_value)),
    )

    matrix[0, 2] += rotated_width / 2.0 - center[0]
    matrix[1, 2] += rotated_height / 2.0 - center[1]

    rotated_image = cv2.warpAffine(
        image,
        matrix,
        (rotated_width, rotated_height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0),
    )
    rotated_alpha = cv2.warpAffine(
        cutout.alpha,
        matrix,
        (rotated_width, rotated_height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )

    trimmed = trim_cutout(rotated_image, rotated_alpha, margin=2)
    if trimmed is None:
        return Cutout(
            class_id=cutout.class_id,
            image=rotated_image,
            alpha=rotated_alpha,
            source_name=cutout.source_name,
            mask_method=cutout.mask_method,
        )

    rotated_image, rotated_alpha = trimmed
    current_height, current_width = rotated_image.shape[:2]
    current_long_side = max(current_width, current_height)

    target_long_side = rng.uniform(
        float(min_object_px),
        float(max_object_px),
    )
    resize_scale = target_long_side / max(1.0, float(current_long_side))

    output_width = max(2, int(round(current_width * resize_scale)))
    output_height = max(2, int(round(current_height * resize_scale)))
    interpolation = cv2.INTER_AREA if resize_scale < 1.0 else cv2.INTER_LINEAR

    transformed_image = cv2.resize(
        rotated_image,
        (output_width, output_height),
        interpolation=interpolation,
    )
    transformed_alpha = cv2.resize(
        rotated_alpha,
        (output_width, output_height),
        interpolation=interpolation,
    )

    final_trimmed = trim_cutout(
        transformed_image,
        transformed_alpha,
        margin=1,
    )
    if final_trimmed is not None:
        transformed_image, transformed_alpha = final_trimmed

    return Cutout(
        class_id=cutout.class_id,
        image=transformed_image,
        alpha=transformed_alpha,
        source_name=cutout.source_name,
        mask_method=cutout.mask_method,
    )


def boxes_conflict(
    first: Box,
    second: Box,
    minimum_gap_px: int,
) -> bool:
    """
    判断两个外接框是否重叠或间距不足。

    minimum_gap_px=0：只禁止重叠。
    minimum_gap_px=4：两个目标外接框之间至少留 4 px。
    """
    first = first.normalized()
    second = second.normalized()
    gap = float(max(0, minimum_gap_px))

    separated = (
        first.x2 + gap <= second.x1
        or second.x2 + gap <= first.x1
        or first.y2 + gap <= second.y1
        or second.y2 + gap <= first.y1
    )
    return not separated


def visible_box_for_cutout(
    cutout: Cutout,
    center_x: int,
    center_y: int,
) -> Box | None:
    """
    在真正粘贴前，根据 alpha 的可见区域计算精确 bbox。
    用该 bbox 进行硬不重叠检查，避免先粘贴后才发现冲突。
    """
    visible_y, visible_x = np.where(cutout.alpha > 38)
    if len(visible_x) < 40:
        return None

    object_height, object_width = cutout.image.shape[:2]
    origin_x = center_x - object_width // 2
    origin_y = center_y - object_height // 2

    return Box(
        class_id=cutout.class_id,
        x1=float(origin_x + visible_x.min()),
        y1=float(origin_y + visible_y.min()),
        x2=float(origin_x + visible_x.max() + 1),
        y2=float(origin_y + visible_y.max() + 1),
    )


def paste_cutout(
    canvas: np.ndarray,
    cutout: Cutout,
    center_x: int,
    center_y: int,
) -> Box | None:
    canvas_height, canvas_width = canvas.shape[:2]
    object_height, object_width = cutout.image.shape[:2]

    object_x1 = center_x - object_width // 2
    object_y1 = center_y - object_height // 2
    object_x2 = object_x1 + object_width
    object_y2 = object_y1 + object_height

    destination_x1 = max(0, object_x1)
    destination_y1 = max(0, object_y1)
    destination_x2 = min(canvas_width, object_x2)
    destination_y2 = min(canvas_height, object_y2)

    if destination_x2 <= destination_x1 or destination_y2 <= destination_y1:
        return None

    source_x1 = destination_x1 - object_x1
    source_y1 = destination_y1 - object_y1
    source_x2 = source_x1 + (destination_x2 - destination_x1)
    source_y2 = source_y1 + (destination_y2 - destination_y1)

    patch = cutout.image[source_y1:source_y2, source_x1:source_x2]
    alpha_u8 = cutout.alpha[source_y1:source_y2, source_x1:source_x2]
    alpha = alpha_u8.astype(np.float32) / 255.0

    visible = alpha > 0.15
    if np.count_nonzero(visible) < 40:
        return None

    destination = canvas[
        destination_y1:destination_y2,
        destination_x1:destination_x2,
    ].astype(np.float32)

    blended = (
        patch.astype(np.float32) * alpha[..., None]
        + destination * (1.0 - alpha[..., None])
    )
    canvas[
        destination_y1:destination_y2,
        destination_x1:destination_x2,
    ] = np.clip(blended, 0, 255).astype(np.uint8)

    visible_y, visible_x = np.where(visible)
    return Box(
        class_id=cutout.class_id,
        x1=float(destination_x1 + visible_x.min()),
        y1=float(destination_y1 + visible_y.min()),
        x2=float(destination_x1 + visible_x.max() + 1),
        y2=float(destination_y1 + visible_y.max() + 1),
    ).clipped(canvas_width, canvas_height)


def augment_background(
    background: np.ndarray,
    rng: random.Random,
) -> np.ndarray:
    canvas = background.copy().astype(np.float32)

    if rng.random() < 0.55:
        contrast = rng.uniform(0.92, 1.08)
        brightness_offset = rng.uniform(-10.0, 10.0)
        canvas = (canvas - 127.5) * contrast + 127.5 + brightness_offset

    canvas = np.clip(canvas, 0, 255).astype(np.uint8)

    if rng.random() < 0.12:
        sigma = rng.uniform(0.25, 0.70)
        canvas = cv2.GaussianBlur(canvas, (0, 0), sigmaX=sigma)

    if rng.random() < 0.15:
        generator = np.random.default_rng(rng.randint(0, 2**31 - 1))
        noise = generator.normal(
            0.0,
            rng.uniform(0.8, 2.5),
            canvas.shape,
        )
        canvas = np.clip(
            canvas.astype(np.float32) + noise,
            0,
            255,
        ).astype(np.uint8)

    return canvas


def synthesize_image(
    backgrounds: list[np.ndarray],
    cutouts: list[Cutout],
    rng: random.Random,
    negative_ratio: float,
    min_objects: int,
    max_objects: int,
    min_object_px: int,
    max_object_px: int,
    minimum_object_gap_px: int,
) -> tuple[np.ndarray, list[Box], int]:
    canvas = augment_background(rng.choice(backgrounds), rng)

    if rng.random() < negative_ratio:
        return canvas, [], 0

    desired_objects = rng.randint(min_objects, max_objects)
    boxes: list[Box] = []
    placement_failures = 0

    for _ in range(desired_objects):
        placed = False

        # 空间不足时最多尝试 100 次；仍失败则少生成一个目标，
        # 绝不通过允许重叠来凑足数量。
        for _attempt in range(100):
            transformed = transform_cutout(
                rng.choice(cutouts),
                rng,
                min_object_px,
                max_object_px,
            )
            object_height, object_width = transformed.image.shape[:2]

            # 完整放入画面，不裁掉目标。
            half_width_left = object_width // 2
            half_width_right = object_width - half_width_left
            half_height_top = object_height // 2
            half_height_bottom = object_height - half_height_top

            minimum_x = half_width_left
            maximum_x = TARGET_WIDTH - half_width_right
            minimum_y = half_height_top
            maximum_y = TARGET_HEIGHT - half_height_bottom

            if minimum_x > maximum_x or minimum_y > maximum_y:
                continue

            center_x = rng.randint(minimum_x, maximum_x)
            center_y = rng.randint(minimum_y, maximum_y)

            candidate_box = visible_box_for_cutout(
                transformed,
                center_x,
                center_y,
            )
            if candidate_box is None:
                continue

            candidate_box = candidate_box.clipped(
                TARGET_WIDTH,
                TARGET_HEIGHT,
            )
            if candidate_box.area < 64.0:
                continue

            # 硬约束：外接框不重叠，并保留指定像素间距。
            if any(
                boxes_conflict(
                    candidate_box,
                    existing,
                    minimum_object_gap_px,
                )
                for existing in boxes
            ):
                continue

            pasted_box = paste_cutout(
                canvas,
                transformed,
                center_x,
                center_y,
            )
            if pasted_box is None or pasted_box.area < 64.0:
                continue

            # 防止数值/alpha 阈值差异导致最终 bbox 与候选 bbox 略有变化。
            if any(
                boxes_conflict(
                    pasted_box,
                    existing,
                    minimum_object_gap_px,
                )
                for existing in boxes
            ):
                # 理论上不应进入此分支。为避免污染图像，候选框与粘贴框
                # 使用相同 alpha 阈值，实际应完全一致。
                raise RuntimeError("内部错误：粘贴后检测到目标框重叠")

            boxes.append(pasted_box)
            placed = True
            break

        if not placed:
            placement_failures += 1

    return canvas, boxes, placement_failures


def draw_preview(image: np.ndarray, boxes: list[Box]) -> np.ndarray:
    preview = image.copy()

    colors = [
        (50, 50, 255),
        (0, 220, 255),
        (255, 100, 50),
        (60, 220, 80),
        (100, 100, 100),
        (255, 220, 60),
        (0, 150, 255),
    ]

    for box in boxes:
        box = box.clipped(TARGET_WIDTH, TARGET_HEIGHT)
        color = colors[box.class_id]
        point1 = (int(round(box.x1)), int(round(box.y1)))
        point2 = (int(round(box.x2)), int(round(box.y2)))

        cv2.rectangle(preview, point1, point2, color, 2)
        label = f"{box.class_id}:{CLASS_NAMES[box.class_id]}"
        cv2.putText(
            preview,
            label,
            (point1[0], max(18, point1[1] - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
            cv2.LINE_AA,
        )

    return preview


def prepare_output(output_root: Path, overwrite: bool) -> None:
    generated_paths = [
        output_root / "images",
        output_root / "labels",
        output_root / "preview",
        output_root / "data.yaml",
        output_root / "generation_report.txt",
    ]

    existing = [path for path in generated_paths if path.exists()]
    if existing and not overwrite:
        raise RuntimeError(
            "输出目录中已经存在生成内容：\n"
            + "\n".join(f"  {path}" for path in existing)
            + "\n请增加 --overwrite 后重新运行。"
        )

    if overwrite:
        for path in generated_paths:
            if path.is_dir():
                shutil.rmtree(path)
            elif path.is_file():
                path.unlink()

    for split_name in ("train", "val", "test"):
        (output_root / "images" / split_name).mkdir(
            parents=True,
            exist_ok=True,
        )
        (output_root / "labels" / split_name).mkdir(
            parents=True,
            exist_ok=True,
        )

    (output_root / "preview").mkdir(parents=True, exist_ok=True)


def generate_split(
    split_name: str,
    count: int,
    backgrounds: list[np.ndarray],
    cutouts: list[Cutout],
    output_root: Path,
    seed: int,
    negative_ratio: float,
    min_objects: int,
    max_objects: int,
    min_object_px: int,
    max_object_px: int,
    minimum_object_gap_px: int,
    jpeg_quality: int,
    preview_limit: int,
) -> GenerationStats:
    rng = random.Random(seed)
    stats = GenerationStats()

    image_directory = output_root / "images" / split_name
    label_directory = output_root / "labels" / split_name
    preview_directory = output_root / "preview"

    log(f"\n  开始生成 {split_name}：{count} 张")

    for index in range(count):
        # 非负样本如果第一次因放置失败变成空图，最多重试 5 次。
        image: np.ndarray | None = None
        boxes: list[Box] = []
        placement_failures = 0

        for _retry in range(5):
            candidate_image, candidate_boxes, failures = synthesize_image(
                backgrounds,
                cutouts,
                rng,
                negative_ratio,
                min_objects,
                max_objects,
                min_object_px,
                max_object_px,
                minimum_object_gap_px,
            )
            image = candidate_image
            boxes = candidate_boxes
            placement_failures += failures

            # 空图本身可以是负样本；不要强制每张都有目标。
            if boxes or rng.random() < negative_ratio:
                break

        assert image is not None

        stem = f"{split_name}_{index:06d}"
        image_path = image_directory / f"{stem}.jpg"
        label_path = label_directory / f"{stem}.txt"

        write_image(image_path, image, jpeg_quality)
        write_labels(label_path, boxes)

        stats.images += 1
        stats.labels += 1
        stats.placement_failures += placement_failures

        if not boxes:
            stats.empty_labels += 1

        stats.object_instances += len(boxes)
        for box in boxes:
            stats.class_instances[box.class_id] += 1

        if index < preview_limit:
            preview = draw_preview(image, boxes)
            write_image(
                preview_directory / f"{stem}_preview.jpg",
                preview,
                jpeg_quality,
            )

        if (
            (index + 1) % 50 == 0
            or index + 1 == count
        ):
            percentage = 100.0 * (index + 1) / max(1, count)
            log(
                f"    {split_name}: {index + 1:>4}/{count} "
                f"({percentage:5.1f}%) | "
                f"objects={stats.object_instances} | "
                f"empty={stats.empty_labels}"
            )

    return stats


def verify_output(
    output_root: Path,
    expected_counts: dict[str, int],
) -> None:
    log("\n[6/7] 核验输出文件")

    total_images = 0
    total_labels = 0
    mismatches: list[str] = []

    for split_name in ("train", "val", "test"):
        image_directory = output_root / "images" / split_name
        label_directory = output_root / "labels" / split_name

        image_paths = sorted(image_directory.glob("*.jpg"))
        label_paths = sorted(label_directory.glob("*.txt"))

        total_images += len(image_paths)
        total_labels += len(label_paths)

        image_stems = {path.stem for path in image_paths}
        label_stems = {path.stem for path in label_paths}

        missing_labels = sorted(image_stems - label_stems)
        missing_images = sorted(label_stems - image_stems)

        if len(image_paths) != expected_counts[split_name]:
            mismatches.append(
                f"{split_name} 图片数={len(image_paths)}，"
                f"预期={expected_counts[split_name]}"
            )
        if len(label_paths) != expected_counts[split_name]:
            mismatches.append(
                f"{split_name} 标签数={len(label_paths)}，"
                f"预期={expected_counts[split_name]}"
            )
        if missing_labels:
            mismatches.append(
                f"{split_name} 缺少标签：{missing_labels[:5]}"
            )
        if missing_images:
            mismatches.append(
                f"{split_name} 缺少图片：{missing_images[:5]}"
            )

        log(
            f"  {split_name:<5} images={len(image_paths):>4} "
            f"labels={len(label_paths):>4} "
            f"expected={expected_counts[split_name]:>4}"
        )

    log(f"  合计 images={total_images}, labels={total_labels}")

    if mismatches:
        raise RuntimeError(
            "输出核验失败：\n  " + "\n  ".join(mismatches)
        )

    log("  输出核验通过：每张图片均有同名标签文件")


def write_data_yaml(output_root: Path) -> None:
    lines = [
        f"path: {output_root.resolve().as_posix()}",
        "train: images/train",
        "val: images/val",
        "test: images/test",
        "names:",
    ]
    lines.extend(
        f"  {class_id}: {class_name}"
        for class_id, class_name in enumerate(CLASS_NAMES)
    )
    (output_root / "data.yaml").write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )


def build_report(
    photo_root: Path,
    output_root: Path,
    label_dir: Path,
    background_dir: Path,
    source_count: int,
    cutout_count: int,
    split_counts: dict[str, int],
    split_stats: dict[str, GenerationStats],
    elapsed_seconds: float,
    args: argparse.Namespace,
) -> str:
    lines = [
        "颜色物料 YOLO 合成数据集生成报告",
        "=" * 48,
        f"照片根目录: {photo_root}",
        f"标签目录: {label_dir}",
        f"背景目录: {background_dir}",
        f"输出目录: {output_root}",
        "",
        f"有效标注照片: {source_count}",
        f"有效 cutout: {cutout_count}",
        f"生成总数: {sum(split_counts.values())}",
        f"输出尺寸: {TARGET_WIDTH}x{TARGET_HEIGHT}",
        f"划分: train={split_counts['train']}, "
        f"val={split_counts['val']}, test={split_counts['test']}",
        f"纯背景比例参数: {args.negative_ratio:.3f}",
        f"每张目标范围: {args.min_objects}~{args.max_objects}",
        f"脚本版本: {SCRIPT_VERSION}",
        f"目标最长边像素约束: "
        f"{args.min_object_px}~{args.max_object_px}px",
        f"固定目标尺寸: "
        f"{args.object_size_px if args.object_size_px is not None else '未启用'}",
        "目标重叠: 禁止",
        f"目标最小框间距: {args.min_object_gap}px",
        f"随机种子: {args.seed}",
        f"耗时: {elapsed_seconds:.2f} 秒",
        "",
        "各 split 统计",
        "-" * 48,
    ]

    total_instances = 0
    total_empty = 0
    total_failures = 0
    total_class_counter: Counter[int] = Counter()

    for split_name in ("train", "val", "test"):
        stats = split_stats[split_name]
        total_instances += stats.object_instances
        total_empty += stats.empty_labels
        total_failures += stats.placement_failures
        total_class_counter.update(stats.class_instances)

        lines.extend([
            f"[{split_name}]",
            f"  图片: {stats.images}",
            f"  标签: {stats.labels}",
            f"  空标签: {stats.empty_labels}",
            f"  目标实例: {stats.object_instances}",
            f"  放置失败次数: {stats.placement_failures}",
        ])

    lines.extend([
        "",
        "总体类别实例分布",
        "-" * 48,
    ])
    for class_id, class_name in enumerate(CLASS_NAMES):
        lines.append(
            f"{class_id} {class_name:<10}: {total_class_counter[class_id]}"
        )

    lines.extend([
        "",
        f"总体目标实例: {total_instances}",
        f"总体空标签: {total_empty}",
        f"总体放置失败次数: {total_failures}",
        "",
        "说明",
        "-" * 48,
        "1. EmptySlot 默认不参与 copy-paste，因此通常为 0。",
        "2. preview/ 保存了部分带框预览图，应人工抽查。",
        "3. 合成 val/test 不能替代独立真实验证集。",
    ])

    return "\n".join(lines) + "\n"


def print_final_summary(
    output_root: Path,
    split_counts: dict[str, int],
    split_stats: dict[str, GenerationStats],
    elapsed_seconds: float,
) -> None:
    total_objects = sum(
        stats.object_instances
        for stats in split_stats.values()
    )
    total_empty = sum(
        stats.empty_labels
        for stats in split_stats.values()
    )
    total_failures = sum(
        stats.placement_failures
        for stats in split_stats.values()
    )

    class_counter: Counter[int] = Counter()
    for stats in split_stats.values():
        class_counter.update(stats.class_instances)

    log("\n[7/7] 生成完成")
    log("=" * 68)
    log(f"输出目录：{output_root}")
    log(
        "数据划分："
        f"train={split_counts['train']}，"
        f"val={split_counts['val']}，"
        f"test={split_counts['test']}"
    )
    log(f"合成图片总数：{sum(split_counts.values())}")
    log(f"目标实例总数：{total_objects}")
    log(f"空标签/纯背景图片：{total_empty}")
    log(f"放置失败次数：{total_failures}")
    log(f"耗时：{elapsed_seconds:.2f} 秒")
    log("类别实例：")
    for class_id, class_name in enumerate(CLASS_NAMES):
        log(
            f"  {class_id} {class_name:<10} "
            f"{class_counter[class_id]}"
        )
    log(f"配置文件：{output_root / 'data.yaml'}")
    log(f"生成报告：{output_root / 'generation_report.txt'}")
    log(f"预览图片：{output_root / 'preview'}")
    log("=" * 68)


def resolve_path_argument(
    current: str | None,
    prompt: str,
) -> Path:
    value = current
    while not value:
        value = input(prompt).strip().strip('"').strip("'")
    return Path(value).expanduser().resolve()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="从已标注照片生成颜色物料 YOLO 数据集",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--photos",
        type=str,
        default=None,
        help="照片根目录，内含“标注”和“背景”文件夹",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="YOLO 数据集输出目录",
    )
    parser.add_argument(
        "--total",
        type=int,
        default=DEFAULT_TOTAL,
        help="生成图片总数",
    )
    parser.add_argument(
        "--split",
        type=parse_split,
        default=DEFAULT_SPLIT,
        help="train:val:test 比例，例如 7:2:1",
    )
    parser.add_argument(
        "--negative-ratio",
        type=float,
        default=DEFAULT_NEGATIVE_RATIO,
        help="纯背景负样本概率",
    )
    parser.add_argument(
        "--min-objects",
        type=int,
        default=DEFAULT_MIN_OBJECTS,
        help="非负样本最少目标数",
    )
    parser.add_argument(
        "--max-objects",
        type=int,
        default=DEFAULT_MAX_OBJECTS,
        help="非负样本最多目标数",
    )
    parser.add_argument(
        "--min-object-px",
        type=int,
        default=DEFAULT_MIN_OBJECT_PX,
        help="目标可见 bbox 最长边的最小像素",
    )
    parser.add_argument(
        "--max-object-px",
        type=int,
        default=DEFAULT_MAX_OBJECT_PX,
        help="目标可见 bbox 最长边的最大像素",
    )
    parser.add_argument(
        "--object-size-px",
        type=int,
        default=None,
        help="固定目标最长边像素；设置后覆盖 min/max-object-px",
    )
    parser.add_argument(
        "--min-object-gap",
        type=int,
        default=DEFAULT_MIN_OBJECT_GAP_PX,
        help="目标外接框之间的最小像素间距；目标始终禁止重叠",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=DEFAULT_SEED,
        help="随机种子",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=DEFAULT_JPEG_QUALITY,
        help="JPEG 保存质量",
    )
    parser.add_argument(
        "--preview-count",
        type=int,
        default=6,
        help="每个 split 输出的预览图数量",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="删除输出目录中既有 images/labels/preview 后重新生成",
    )
    return parser


def validate_arguments(args: argparse.Namespace) -> None:
    if args.total <= 0:
        raise ValueError("--total 必须大于 0")
    if not 0.0 <= args.negative_ratio < 1.0:
        raise ValueError("--negative-ratio 必须位于 [0, 1)")
    if args.min_objects <= 0:
        raise ValueError("--min-objects 必须大于 0")
    if args.max_objects < args.min_objects:
        raise ValueError("--max-objects 不能小于 --min-objects")
    if args.object_size_px is not None:
        if args.object_size_px < 8:
            raise ValueError("--object-size-px 必须不小于 8")
        args.min_object_px = args.object_size_px
        args.max_object_px = args.object_size_px

    if args.min_object_px < 8:
        raise ValueError("--min-object-px 必须不小于 8")
    if args.max_object_px < args.min_object_px:
        raise ValueError("--max-object-px 不能小于 --min-object-px")
    if args.max_object_px > min(TARGET_WIDTH, TARGET_HEIGHT):
        raise ValueError("--max-object-px 不能大于输出画面的短边 750")
    if args.min_object_gap < 0:
        raise ValueError("--min-object-gap 不能为负数")
    if not 1 <= args.jpeg_quality <= 100:
        raise ValueError("--jpeg-quality 必须位于 1~100")
    if args.preview_count < 0:
        raise ValueError("--preview-count 不能为负数")


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        validate_arguments(args)

        photo_root = resolve_path_argument(
            args.photos,
            "请输入照片根目录：",
        )
        output_root = resolve_path_argument(
            args.output,
            "请输入数据集输出目录：",
        )

        if not photo_root.is_dir():
            raise FileNotFoundError(f"照片根目录不存在：{photo_root}")

        label_dir = find_subdirectory(
            photo_root,
            ("标注", "labels", "annotations"),
        )
        if label_dir is None:
            raise FileNotFoundError(
                "没有找到标注目录。预期为：\n"
                f"  {photo_root / '标注'}"
            )

        background_dir = find_subdirectory(
            photo_root,
            ("背景", "background", "backgrounds"),
        )
        if background_dir is None:
            raise FileNotFoundError(
                "没有找到背景目录。预期为：\n"
                f"  {photo_root / '背景'}"
            )

        log("=" * 68)
        log("颜色物料 YOLO 数据集生成器")
        log("=" * 68)
        log(f"照片根目录：{photo_root}")
        log(f"标签目录：{label_dir}")
        log(f"背景目录：{background_dir}")
        log(f"输出目录：{output_root}")
        log(f"生成总数：{args.total}")
        log(
            "划分比例："
            f"{args.split[0]:.3f}:"
            f"{args.split[1]:.3f}:"
            f"{args.split[2]:.3f}"
        )
        log(f"输出尺寸：{TARGET_WIDTH}x{TARGET_HEIGHT}")
        log("缩放规则：保持 4:3 比例，最长边固定为 1000px，不裁剪、不拉伸")
        log(f"脚本版本：{SCRIPT_VERSION}")
        log(
            "目标最长边直接像素范围："
            f"{args.min_object_px}~{args.max_object_px}px"
        )
        if args.object_size_px is not None:
            log(f"固定目标尺寸：{args.object_size_px}px")
        log(
            f"目标重叠：禁止；最小框间距：{args.min_object_gap}px"
        )
        log(f"随机种子：{args.seed}")

        started_at = time.perf_counter()

        sources = load_sources(
            photo_root,
            label_dir,
            background_dir,
        )
        backgrounds = load_backgrounds(background_dir)
        cutouts = build_cutouts(sources)

        prepare_output(output_root, args.overwrite)

        split_counts = allocate_counts(args.total, args.split)
        background_splits = split_items(
            backgrounds,
            args.split,
            args.seed + 1000,
        )
        cutout_splits = split_items(
            cutouts,
            args.split,
            args.seed + 2000,
        )

        log("\n[4/7] 准备数据划分")
        for split_name in ("train", "val", "test"):
            # 极端情况下某个 split 无资源，使用全局资源兜底并显式告警。
            if not background_splits[split_name]:
                log(
                    f"  [警告] {split_name} 无独立背景，"
                    "使用全局背景池兜底"
                )
                background_splits[split_name] = backgrounds

            if not cutout_splits[split_name]:
                log(
                    f"  [警告] {split_name} 无独立 cutout，"
                    "使用全局 cutout 池兜底；该 split 存在素材泄漏风险"
                )
                cutout_splits[split_name] = cutouts

            class_counts = Counter(
                cutout.class_id
                for cutout in cutout_splits[split_name]
            )
            log(
                f"  {split_name:<5} "
                f"images={split_counts[split_name]:>4} | "
                f"backgrounds={len(background_splits[split_name]):>3} | "
                f"cutouts={len(cutout_splits[split_name]):>3} | "
                f"classes={dict(sorted(class_counts.items()))}"
            )

        log("\n[5/7] 生成合成图像与 YOLO 标签")
        split_stats: dict[str, GenerationStats] = {}

        for split_index, split_name in enumerate(("train", "val", "test")):
            split_stats[split_name] = generate_split(
                split_name=split_name,
                count=split_counts[split_name],
                backgrounds=background_splits[split_name],
                cutouts=cutout_splits[split_name],
                output_root=output_root,
                seed=args.seed + split_index * 100_003,
                negative_ratio=args.negative_ratio,
                min_objects=args.min_objects,
                max_objects=args.max_objects,
                min_object_px=args.min_object_px,
                max_object_px=args.max_object_px,
                minimum_object_gap_px=args.min_object_gap,
                jpeg_quality=args.jpeg_quality,
                preview_limit=args.preview_count,
            )

        write_data_yaml(output_root)
        verify_output(output_root, split_counts)

        elapsed_seconds = time.perf_counter() - started_at

        report = build_report(
            photo_root=photo_root,
            output_root=output_root,
            label_dir=label_dir,
            background_dir=background_dir,
            source_count=len(sources),
            cutout_count=len(cutouts),
            split_counts=split_counts,
            split_stats=split_stats,
            elapsed_seconds=elapsed_seconds,
            args=args,
        )
        (output_root / "generation_report.txt").write_text(
            report,
            encoding="utf-8",
        )

        print_final_summary(
            output_root,
            split_counts,
            split_stats,
            elapsed_seconds,
        )
        return 0

    except KeyboardInterrupt:
        log("\n[中止] 用户取消")
        return 130
    except Exception as exc:
        log("\n[失败] 数据集生成未完成")
        log(f"原因：{exc}")
        log(
            "\n请重点检查：\n"
            "1. 照片根目录中是否存在“标注”和“背景”文件夹；\n"
            "2. 标注文件名是否与图片 stem 一致；\n"
            "3. 标注是否为标准 YOLO 五字段格式；\n"
            "4. 输出目录是否需要增加 --overwrite。"
        )
        return 1


if __name__ == "__main__":
    sys.exit(main())
