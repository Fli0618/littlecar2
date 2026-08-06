#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
颜色物料 YOLO 标注与合成数据集工具（单文件）

目录约定：
照片根目录/
├── IMG_001.jpg
├── IMG_002.jpg
├── ...
└── 背景/
    ├── BG_001.jpg
    └── ...

标注快捷键：
- 左键拖动：新增半透明矩形框
- 单击已有框：选中
- 1~7：切换类别
- Delete：删除选中框
- Backspace / Ctrl+Z：撤销最后一个框
- Enter：保存并进入下一张
- ← / →：保存并切换图片
- Ctrl+S：只保存

数据集生成：
- 从标注框内自动提取带 alpha 的物料 cutout
- 随机缩放、旋转、亮度扰动并粘贴到“背景”图片
- 支持随机生成总数量
- 支持 train / val / test 比例
- 先按原始图片划分，再生成各自数据，降低数据泄漏
- 自动整理为 YOLO 数据集并生成 data.yaml

依赖：
    python -m pip install opencv-python pillow numpy
"""

from __future__ import annotations

import math
import random
import shutil
import threading
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageTk
import tkinter as tk
from tkinter import filedialog, messagebox, ttk


CLASS_NAMES = [
    "Red",
    "Yellow",
    "Blue",
    "Green",
    "Black",
    "LightBlue",
    "EmptySlot",
]

CLASS_COLORS = [
    (255, 64, 64),
    (255, 215, 0),
    (64, 128, 255),
    (64, 220, 96),
    (96, 96, 96),
    (80, 210, 255),
    (255, 150, 40),
]

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
BACKGROUND_DIR = "背景"
ANNOTATION_DIR = "标注"
TARGET_W = 640
TARGET_H = 480

# EmptySlot 与场景结构耦合，不做 copy-paste；真实标注图仍会进入数据集。
SYNTHETIC_CLASS_IDS = {0, 1, 2, 3, 4, 5}


@dataclass
class Box:
    class_id: int
    x1: float
    y1: float
    x2: float
    y2: float

    def norm(self) -> "Box":
        return Box(
            self.class_id,
            min(self.x1, self.x2),
            min(self.y1, self.y2),
            max(self.x1, self.x2),
            max(self.y1, self.y2),
        )

    def clip(self, w: int, h: int) -> "Box":
        b = self.norm()
        return Box(
            b.class_id,
            max(0.0, min(w - 1.0, b.x1)),
            max(0.0, min(h - 1.0, b.y1)),
            max(0.0, min(w - 1.0, b.x2)),
            max(0.0, min(h - 1.0, b.y2)),
        )

    def area(self) -> float:
        b = self.norm()
        return max(0.0, b.x2 - b.x1) * max(0.0, b.y2 - b.y1)

    def to_yolo(self, w: int, h: int) -> str:
        b = self.clip(w, h)
        cx = ((b.x1 + b.x2) / 2.0) / w
        cy = ((b.y1 + b.y2) / 2.0) / h
        bw = (b.x2 - b.x1) / w
        bh = (b.y2 - b.y1) / h
        return f"{b.class_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}"

    @staticmethod
    def from_yolo(line: str, w: int, h: int) -> "Box":
        parts = line.strip().split()
        if len(parts) != 5:
            raise ValueError(f"非法 YOLO 标签：{line!r}")
        cid = int(parts[0])
        cx, cy, bw, bh = map(float, parts[1:])
        return Box(
            cid,
            (cx - bw / 2.0) * w,
            (cy - bh / 2.0) * h,
            (cx + bw / 2.0) * w,
            (cy + bh / 2.0) * h,
        ).clip(w, h)


@dataclass
class Cutout:
    class_id: int
    bgr: np.ndarray
    alpha: np.ndarray


def imread_u(path: Path) -> np.ndarray | None:
    try:
        raw = np.fromfile(str(path), dtype=np.uint8)
        return cv2.imdecode(raw, cv2.IMREAD_COLOR) if raw.size else None
    except Exception:
        return None


def imwrite_u(path: Path, image: np.ndarray, quality: int = 94) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    suffix = path.suffix.lower()
    if suffix in {".jpg", ".jpeg"}:
        ext = ".jpg"
        params = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    else:
        ext = ".png"
        params = [int(cv2.IMWRITE_PNG_COMPRESSION), 3]
    ok, encoded = cv2.imencode(ext, image, params)
    if not ok:
        return False
    encoded.tofile(str(path))
    return True


def root_images(root: Path) -> list[Path]:
    if not root.is_dir():
        return []
    return sorted(p for p in root.iterdir() if p.is_file() and p.suffix.lower() in IMAGE_EXTS)


def background_images(root: Path) -> list[Path]:
    bg = root / BACKGROUND_DIR
    if not bg.is_dir():
        return []
    return sorted(p for p in bg.rglob("*") if p.is_file() and p.suffix.lower() in IMAGE_EXTS)


def label_path(root: Path, image_path: Path) -> Path:
    return root / ANNOTATION_DIR / f"{image_path.stem}.txt"


def save_boxes(path: Path, boxes: list[Box], w: int, h: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [b.to_yolo(w, h) for b in boxes if b.area() >= 16.0]
    path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def load_boxes(path: Path, w: int, h: int) -> list[Box]:
    if not path.is_file():
        return []
    result: list[Box] = []
    for n, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        box = Box.from_yolo(line, w, h)
        if not 0 <= box.class_id < len(CLASS_NAMES):
            raise ValueError(f"{path}:{n}: class_id 越界")
        result.append(box)
    return result


def resize_cover(image: np.ndarray, w: int, h: int) -> np.ndarray:
    ih, iw = image.shape[:2]
    scale = max(w / iw, h / ih)
    nw, nh = max(w, round(iw * scale)), max(h, round(ih * scale))
    resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR)
    x1, y1 = (nw - w) // 2, (nh - h) // 2
    return resized[y1:y1 + h, x1:x1 + w].copy()


def resize_real(image: np.ndarray, boxes: list[Box]) -> tuple[np.ndarray, list[Box]]:
    ih, iw = image.shape[:2]
    if (iw, ih) == (TARGET_W, TARGET_H):
        return image.copy(), [b.clip(TARGET_W, TARGET_H) for b in boxes]
    sx, sy = TARGET_W / iw, TARGET_H / ih
    resized = cv2.resize(image, (TARGET_W, TARGET_H), interpolation=cv2.INTER_AREA)
    out = [Box(b.class_id, b.x1 * sx, b.y1 * sy, b.x2 * sx, b.y2 * sy).clip(TARGET_W, TARGET_H) for b in boxes]
    return resized, out


def largest_component(mask: np.ndarray) -> np.ndarray:
    binary = (mask > 0).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
    if count <= 1:
        return binary * 255
    idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return np.where(labels == idx, 255, 0).astype(np.uint8)


def extract_cutout(image: np.ndarray, box: Box) -> Cutout | None:
    if box.class_id not in SYNTHETIC_CLASS_IDS:
        return None
    ih, iw = image.shape[:2]
    b = box.clip(iw, ih).norm()
    bw, bh = b.x2 - b.x1, b.y2 - b.y1
    if bw < 8 or bh < 8:
        return None

    px, py = max(4, round(bw * 0.10)), max(4, round(bh * 0.10))
    x1, y1 = max(0, math.floor(b.x1) - px), max(0, math.floor(b.y1) - py)
    x2, y2 = min(iw, math.ceil(b.x2) + px), min(ih, math.ceil(b.y2) + py)
    crop = image[y1:y2, x1:x2].copy()
    ch, cw = crop.shape[:2]
    if cw < 10 or ch < 10:
        return None

    rx, ry = max(1, round(b.x1) - x1), max(1, round(b.y1) - y1)
    rw = min(cw - rx - 1, max(2, round(bw)))
    rh = min(ch - ry - 1, max(2, round(bh)))
    if rw < 2 or rh < 2:
        return None

    mask = np.full((ch, cw), cv2.GC_BGD, np.uint8)
    mask[ry:ry + rh, rx:rx + rw] = cv2.GC_PR_FGD
    ix, iy = max(1, int(rw * 0.16)), max(1, int(rh * 0.16))
    if rw > 2 * ix and rh > 2 * iy:
        mask[ry + iy:ry + rh - iy, rx + ix:rx + rw - ix] = cv2.GC_FGD

    try:
        bg_model = np.zeros((1, 65), np.float64)
        fg_model = np.zeros((1, 65), np.float64)
        cv2.grabCut(crop, mask, None, bg_model, fg_model, 5, cv2.GC_INIT_WITH_MASK)
        alpha = np.where((mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD), 255, 0).astype(np.uint8)
    except cv2.error:
        alpha = np.zeros((ch, cw), np.uint8)

    alpha = cv2.morphologyEx(alpha, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    alpha = cv2.morphologyEx(alpha, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations=2)
    alpha = largest_component(alpha)
    ratio = np.count_nonzero(alpha) / alpha.size

    if not 0.08 <= ratio <= 0.85:
        alpha[:] = 0
        cv2.ellipse(alpha, (rx + rw // 2, ry + rh // 2), (max(1, rw // 2), max(1, rh // 2)), 0, 0, 360, 255, -1)

    alpha = cv2.GaussianBlur(alpha, (0, 0), 1.2)
    return Cutout(box.class_id, crop, alpha)


def transform_cutout(c: Cutout, rng: random.Random) -> Cutout:
    image = c.bgr.astype(np.float32)
    image = (image - 127.5) * rng.uniform(0.90, 1.10) + 127.5
    image *= rng.uniform(0.88, 1.12)
    image = np.clip(image, 0, 255).astype(np.uint8)
    if rng.random() < 0.18:
        image = cv2.GaussianBlur(image, (0, 0), rng.uniform(0.25, 0.85))

    scale, angle = rng.uniform(0.78, 1.22), rng.uniform(-180.0, 180.0)
    h, w = image.shape[:2]
    center = (w / 2.0, h / 2.0)
    matrix = cv2.getRotationMatrix2D(center, angle, scale)
    cs, sn = abs(matrix[0, 0]), abs(matrix[0, 1])
    nw, nh = max(2, round(h * sn + w * cs)), max(2, round(h * cs + w * sn))
    matrix[0, 2] += nw / 2.0 - center[0]
    matrix[1, 2] += nh / 2.0 - center[1]

    out_img = cv2.warpAffine(image, matrix, (nw, nh), flags=cv2.INTER_LINEAR, borderValue=(0, 0, 0))
    out_alpha = cv2.warpAffine(c.alpha, matrix, (nw, nh), flags=cv2.INTER_LINEAR, borderValue=0)
    return Cutout(c.class_id, out_img, out_alpha)


def iou(a: Box, b: Box) -> float:
    a, b = a.norm(), b.norm()
    x1, y1 = max(a.x1, b.x1), max(a.y1, b.y1)
    x2, y2 = min(a.x2, b.x2), min(a.y2, b.y2)
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = a.area() + b.area() - inter
    return 0.0 if union <= 0 else inter / union


def paste(canvas: np.ndarray, c: Cutout, cx: int, cy: int) -> Box | None:
    ch, cw = canvas.shape[:2]
    oh, ow = c.bgr.shape[:2]
    x1, y1 = cx - ow // 2, cy - oh // 2
    x2, y2 = x1 + ow, y1 + oh
    dx1, dy1, dx2, dy2 = max(0, x1), max(0, y1), min(cw, x2), min(ch, y2)
    if dx2 <= dx1 or dy2 <= dy1:
        return None

    sx1, sy1 = dx1 - x1, dy1 - y1
    sx2, sy2 = sx1 + dx2 - dx1, sy1 + dy2 - dy1
    patch = c.bgr[sy1:sy2, sx1:sx2]
    alpha = c.alpha[sy1:sy2, sx1:sx2].astype(np.float32) / 255.0
    visible = alpha > 0.15
    if np.count_nonzero(visible) < 40:
        return None

    dst = canvas[dy1:dy2, dx1:dx2].astype(np.float32)
    canvas[dy1:dy2, dx1:dx2] = np.clip(patch * alpha[..., None] + dst * (1 - alpha[..., None]), 0, 255).astype(np.uint8)
    ys, xs = np.where(visible)
    return Box(c.class_id, dx1 + xs.min(), dy1 + ys.min(), dx1 + xs.max() + 1, dy1 + ys.max() + 1).clip(cw, ch)


def normalize_ratios(train: float, val: float, test: float) -> tuple[float, float, float]:
    values = [train, val, test]
    if any(v < 0 for v in values):
        raise ValueError("数据集比例不能为负数")
    total = sum(values)
    if total <= 0:
        raise ValueError("train/val/test 比例之和必须大于 0")
    return tuple(v / total for v in values)  # type: ignore[return-value]


def allocate(total: int, ratios: tuple[float, float, float]) -> dict[str, int]:
    names = ["train", "val", "test"]
    exact = [total * r for r in ratios]
    counts = [math.floor(x) for x in exact]
    for idx in sorted(range(3), key=lambda i: exact[i] - counts[i], reverse=True)[:total - sum(counts)]:
        counts[idx] += 1
    return dict(zip(names, counts))


def split_sources(paths: list[Path], ratios: tuple[float, float, float], seed: int) -> dict[str, list[Path]]:
    items = list(paths)
    random.Random(seed).shuffle(items)
    counts = allocate(len(items), ratios)
    out: dict[str, list[Path]] = {}
    pos = 0
    for name in ("train", "val", "test"):
        out[name] = items[pos:pos + counts[name]]
        pos += counts[name]
    return out


def prepare_output(root: Path, overwrite: bool) -> None:
    if root.exists() and any(root.iterdir()):
        if not overwrite:
            raise RuntimeError("输出目录非空，请勾选“覆盖输出目录”或选择新目录")
        shutil.rmtree(root)
    for split in ("train", "val", "test"):
        (root / "images" / split).mkdir(parents=True, exist_ok=True)
        (root / "labels" / split).mkdir(parents=True, exist_ok=True)


def build_pool(photo_root: Path, paths: list[Path], log: Callable[[str], None]):
    cutouts: list[Cutout] = []
    real_items: list[tuple[Path, np.ndarray, list[Box]]] = []
    for path in paths:
        image = imread_u(path)
        if image is None:
            log(f"[警告] 无法读取：{path.name}")
            continue
        h, w = image.shape[:2]
        boxes = load_boxes(label_path(photo_root, path), w, h)
        real_image, real_boxes = resize_real(image, boxes)
        real_items.append((path, real_image, real_boxes))
        for box in boxes:
            c = extract_cutout(image, box)
            if c is not None:
                cutouts.append(c)
    return cutouts, real_items


def synthesize(backgrounds: list[np.ndarray], cutouts: list[Cutout], rng: random.Random,
               negative_ratio: float, min_objects: int, max_objects: int):
    canvas = rng.choice(backgrounds).copy()
    if rng.random() < 0.35:
        canvas = np.clip(canvas.astype(np.float32) * rng.uniform(0.92, 1.08), 0, 255).astype(np.uint8)
    boxes: list[Box] = []
    if not cutouts or rng.random() < negative_ratio:
        return canvas, boxes

    for _ in range(rng.randint(min_objects, max_objects)):
        obj = transform_cutout(rng.choice(cutouts), rng)
        oh, ow = obj.bgr.shape[:2]
        for _attempt in range(50):
            min_x, max_x = max(0, int(ow * 0.35)), min(TARGET_W - 1, TARGET_W - int(ow * 0.35))
            min_y, max_y = max(0, int(oh * 0.35)), min(TARGET_H - 1, TARGET_H - int(oh * 0.35))
            if min_x > max_x or min_y > max_y:
                break
            cx, cy = rng.randint(min_x, max_x), rng.randint(min_y, max_y)
            estimate = Box(obj.class_id, cx - ow / 2, cy - oh / 2, cx + ow / 2, cy + oh / 2).clip(TARGET_W, TARGET_H)
            if estimate.area() < 100 or any(iou(estimate, b) > 0.28 for b in boxes):
                continue
            result = paste(canvas, obj, cx, cy)
            if result is not None and result.area() >= 100:
                boxes.append(result)
                break

    if rng.random() < 0.16:
        noise = np.random.default_rng(rng.randint(0, 2**31 - 1)).normal(0, rng.uniform(1, 3), canvas.shape)
        canvas = np.clip(canvas.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return canvas, boxes


def generate_dataset(photo_root: Path, output_root: Path, total: int,
                     ratios: tuple[float, float, float], seed: int,
                     negative_ratio: float, min_objects: int, max_objects: int,
                     overwrite: bool, log: Callable[[str], None]) -> None:
    if total < 0:
        raise ValueError("随机生成总数量不能小于 0")
    if not 0 <= negative_ratio < 1:
        raise ValueError("纯背景负样本比例必须位于 [0,1)")
    if min_objects <= 0 or max_objects < min_objects:
        raise ValueError("每张目标数量范围非法")

    photos = root_images(photo_root)
    bg_paths = background_images(photo_root)
    annotated = [p for p in photos if label_path(photo_root, p).is_file()]
    if not annotated:
        raise RuntimeError("没有已保存标注的照片")
    if not bg_paths:
        raise RuntimeError(f"“{BACKGROUND_DIR}”文件夹中没有背景图片")

    backgrounds = []
    for p in bg_paths:
        image = imread_u(p)
        if image is not None:
            backgrounds.append(resize_cover(image, TARGET_W, TARGET_H))
    if not backgrounds:
        raise RuntimeError("背景图片均无法读取")

    prepare_output(output_root, overwrite)
    source_split = split_sources(annotated, ratios, seed)
    synthetic_counts = allocate(total, ratios)

    log(f"[信息] 已标注原图：{len(annotated)}")
    log(f"[信息] 背景图：{len(backgrounds)}")
    log(f"[信息] 合成图总数：{total}")

    for split_index, split in enumerate(("train", "val", "test")):
        cutouts, real_items = build_pool(photo_root, source_split[split], log)
        log(f"[信息] {split}: 原图={len(real_items)} cutout={len(cutouts)} 合成={synthetic_counts[split]}")

        for i, (source, image, boxes) in enumerate(real_items):
            stem = f"real_{i:04d}_{source.stem}"
            imwrite_u(output_root / "images" / split / f"{stem}.jpg", image)
            save_boxes(output_root / "labels" / split / f"{stem}.txt", boxes, TARGET_W, TARGET_H)

        rng = random.Random(seed + split_index * 100003)
        for i in range(synthetic_counts[split]):
            image, boxes = synthesize(
                backgrounds,
                cutouts,
                rng,
                negative_ratio if cutouts else 1.0,
                min_objects,
                max_objects,
            )
            stem = f"synthetic_{i:06d}"
            imwrite_u(output_root / "images" / split / f"{stem}.jpg", image)
            save_boxes(output_root / "labels" / split / f"{stem}.txt", boxes, TARGET_W, TARGET_H)
            if (i + 1) % 100 == 0 or i + 1 == synthetic_counts[split]:
                log(f"[进度] {split}: {i + 1}/{synthetic_counts[split]}")

    yaml = [
        f"path: {output_root.resolve().as_posix()}",
        "train: images/train",
        "val: images/val",
        "test: images/test",
        "names:",
    ] + [f"  {i}: {name}" for i, name in enumerate(CLASS_NAMES)]
    (output_root / "data.yaml").write_text("\n".join(yaml) + "\n", encoding="utf-8")
    log(f"[完成] 数据集：{output_root}")


class App:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("颜色物料 YOLO 标注与数据集生成工具")
        self.root.geometry("1380x860")
        self.root.minsize(1050, 680)

        self.photo_root: Path | None = None
        self.output_root: Path | None = None
        self.images: list[Path] = []
        self.index = 0
        self.current_rgb: np.ndarray | None = None
        self.current_bgr: np.ndarray | None = None
        self.iw = self.ih = 0
        self.boxes: list[Box] = []
        self.current_class = 0
        self.selected: int | None = None
        self.drag_start: tuple[float, float] | None = None
        self.preview_id: int | None = None
        self.scale = 1.0
        self.off_x = self.off_y = 0.0
        self.tk_image: ImageTk.PhotoImage | None = None

        self.build_ui()
        self.bind_keys()

    def build_ui(self) -> None:
        root_frame = ttk.Frame(self.root, padding=8)
        root_frame.pack(fill=tk.BOTH, expand=True)

        top = ttk.Frame(root_frame)
        top.pack(fill=tk.X)
        ttk.Button(top, text="选择照片根目录", command=self.choose_photos).pack(side=tk.LEFT)
        self.photo_var = tk.StringVar(value="尚未选择照片目录")
        ttk.Label(top, textvariable=self.photo_var).pack(side=tk.LEFT, padx=8, fill=tk.X, expand=True)
        ttk.Button(top, text="选择数据集输出目录", command=self.choose_output).pack(side=tk.LEFT)
        self.output_var = tk.StringVar(value="尚未选择输出目录")
        ttk.Label(top, textvariable=self.output_var).pack(side=tk.LEFT, padx=8, fill=tk.X, expand=True)

        nav = ttk.Frame(root_frame)
        nav.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(nav, text="上一张 [←]", command=self.prev).pack(side=tk.LEFT)
        ttk.Button(nav, text="下一张 [→]", command=self.next).pack(side=tk.LEFT, padx=4)
        ttk.Button(nav, text="保存 [Ctrl+S]", command=self.save).pack(side=tk.LEFT, padx=(12, 4))
        ttk.Button(nav, text="保存并下一张 [Enter]", command=self.save_next).pack(side=tk.LEFT)
        ttk.Button(nav, text="删除选中 [Delete]", command=self.delete).pack(side=tk.LEFT, padx=(12, 4))
        ttk.Button(nav, text="撤销 [Backspace]", command=self.undo).pack(side=tk.LEFT)
        self.progress = tk.StringVar(value="0/0")
        ttk.Label(nav, textvariable=self.progress).pack(side=tk.RIGHT)

        pane = ttk.Panedwindow(root_frame, orient=tk.HORIZONTAL)
        pane.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        left, right = ttk.Frame(pane), ttk.Frame(pane, width=330)
        pane.add(left, weight=5)
        pane.add(right, weight=0)

        self.canvas = tk.Canvas(left, bg="#151515", highlightthickness=0, cursor="crosshair")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda _e: self.redraw())
        self.canvas.bind("<ButtonPress-1>", self.mouse_down)
        self.canvas.bind("<B1-Motion>", self.mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.mouse_up)

        classes = ttk.LabelFrame(right, text="类别（数字键 1~7）", padding=8)
        classes.pack(fill=tk.X, padx=(8, 0))
        self.class_var = tk.IntVar(value=0)
        for i, name in enumerate(CLASS_NAMES):
            ttk.Radiobutton(classes, text=f"{i + 1}. {name}", variable=self.class_var,
                            value=i, command=self.class_changed).pack(anchor=tk.W, pady=2)

        gen = ttk.LabelFrame(right, text="数据集生成", padding=8)
        gen.pack(fill=tk.X, padx=(8, 0), pady=(10, 0))
        self.count_var = tk.StringVar(value="3000")
        self.train_var = tk.StringVar(value="0.8")
        self.val_var = tk.StringVar(value="0.1")
        self.test_var = tk.StringVar(value="0.1")
        self.seed_var = tk.StringVar(value="42")
        self.neg_var = tk.StringVar(value="0.10")
        self.min_obj_var = tk.StringVar(value="1")
        self.max_obj_var = tk.StringVar(value="5")
        self.overwrite_var = tk.BooleanVar(value=False)
        for label, var in [
            ("随机生成总数量", self.count_var),
            ("train 比例", self.train_var),
            ("val 比例", self.val_var),
            ("test 比例", self.test_var),
            ("随机种子", self.seed_var),
            ("纯背景比例", self.neg_var),
            ("每张最少目标", self.min_obj_var),
            ("每张最多目标", self.max_obj_var),
        ]:
            row = ttk.Frame(gen)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=label, width=17).pack(side=tk.LEFT)
            ttk.Entry(row, textvariable=var).pack(side=tk.RIGHT, fill=tk.X, expand=True)
        ttk.Checkbutton(gen, text="覆盖非空输出目录", variable=self.overwrite_var).pack(anchor=tk.W, pady=5)
        self.generate_btn = ttk.Button(gen, text="生成 YOLO 数据集", command=self.start_generation)
        self.generate_btn.pack(fill=tk.X)

        log_box = ttk.LabelFrame(right, text="状态", padding=6)
        log_box.pack(fill=tk.BOTH, expand=True, padx=(8, 0), pady=(10, 0))
        self.log_text = tk.Text(log_box, wrap=tk.WORD, state=tk.DISABLED, height=12)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def bind_keys(self) -> None:
        self.root.bind("<Return>", lambda _e: self.save_next())
        self.root.bind("<Left>", lambda _e: self.prev())
        self.root.bind("<Right>", lambda _e: self.next())
        self.root.bind("<Control-s>", lambda _e: self.save())
        self.root.bind("<Delete>", lambda _e: self.delete())
        self.root.bind("<BackSpace>", lambda _e: self.undo())
        self.root.bind("<Control-z>", lambda _e: self.undo())
        for i in range(7):
            self.root.bind(str(i + 1), lambda _e, v=i: self.select_class(v))

    def log(self, text: str) -> None:
        def write():
            self.log_text.configure(state=tk.NORMAL)
            self.log_text.insert(tk.END, text + "\n")
            self.log_text.see(tk.END)
            self.log_text.configure(state=tk.DISABLED)
        self.root.after(0, write)

    def choose_photos(self) -> None:
        selected = filedialog.askdirectory(title="选择照片根目录")
        if not selected:
            return
        root = Path(selected).resolve()
        images = root_images(root)
        if not images:
            messagebox.showerror("错误", "根目录中没有图片")
            return
        self.photo_root, self.images, self.index = root, images, 0
        self.photo_var.set(str(root))
        if not (root / BACKGROUND_DIR).is_dir():
            messagebox.showwarning("提示", f"尚未找到 {BACKGROUND_DIR} 文件夹")
        self.load()

    def choose_output(self) -> None:
        selected = filedialog.askdirectory(title="选择数据集输出目录")
        if selected:
            self.output_root = Path(selected).resolve()
            self.output_var.set(str(self.output_root))

    def save(self) -> None:
        if self.photo_root is None or self.current_bgr is None or not self.images:
            return
        save_boxes(label_path(self.photo_root, self.images[self.index]), self.boxes, self.iw, self.ih)
        self.log(f"[保存] {self.images[self.index].name}，框={len(self.boxes)}")

    def save_next(self) -> None:
        if not self.images:
            return
        self.save()
        if self.index < len(self.images) - 1:
            self.index += 1
            self.load()
        else:
            messagebox.showinfo("完成", "已经是最后一张")

    def prev(self) -> None:
        if self.images:
            self.save()
            if self.index > 0:
                self.index -= 1
                self.load()

    def next(self) -> None:
        if self.images:
            self.save()
            if self.index < len(self.images) - 1:
                self.index += 1
                self.load()

    def load(self) -> None:
        if self.photo_root is None or not self.images:
            return
        image = imread_u(self.images[self.index])
        if image is None:
            messagebox.showerror("错误", f"无法读取 {self.images[self.index]}")
            return
        self.current_bgr = image
        self.current_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.ih, self.iw = image.shape[:2]
        self.boxes = load_boxes(label_path(self.photo_root, self.images[self.index]), self.iw, self.ih)
        self.selected = None
        saved = sum(label_path(self.photo_root, p).is_file() for p in self.images)
        self.progress.set(f"{self.index + 1}/{len(self.images)} | {self.images[self.index].name} | {self.iw}×{self.ih} | 框 {len(self.boxes)} | 已保存 {saved}")
        self.redraw()

    def select_class(self, cid: int) -> None:
        self.current_class = cid
        self.class_var.set(cid)
        if self.selected is not None:
            self.boxes[self.selected].class_id = cid
        self.redraw()

    def class_changed(self) -> None:
        self.select_class(int(self.class_var.get()))

    def redraw(self) -> None:
        if self.current_rgb is None:
            return
        cw, ch = max(1, self.canvas.winfo_width()), max(1, self.canvas.winfo_height())
        self.scale = min(cw / self.iw, ch / self.ih)
        rw, rh = max(1, round(self.iw * self.scale)), max(1, round(self.ih * self.scale))
        self.off_x, self.off_y = (cw - rw) / 2, (ch - rh) / 2
        image = Image.fromarray(self.current_rgb).resize((rw, rh), Image.Resampling.LANCZOS).convert("RGBA")
        overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay, "RGBA")
        for i, b in enumerate(self.boxes):
            x1, y1, x2, y2 = [round(v * self.scale) for v in (b.x1, b.y1, b.x2, b.y2)]
            color = CLASS_COLORS[b.class_id]
            draw.rectangle((x1, y1, x2, y2), fill=(*color, 105 if i == self.selected else 72),
                           outline=(*color, 255), width=4 if i == self.selected else 2)
            draw.text((x1 + 3, y1 + 2), f"{b.class_id + 1}:{CLASS_NAMES[b.class_id]}", fill=(255, 255, 255, 255))
        self.tk_image = ImageTk.PhotoImage(Image.alpha_composite(image, overlay))
        self.canvas.delete("all")
        self.canvas.create_image(self.off_x, self.off_y, anchor=tk.NW, image=self.tk_image)

    def c2i(self, x: float, y: float) -> tuple[float, float]:
        return (max(0.0, min(self.iw - 1.0, (x - self.off_x) / self.scale)),
                max(0.0, min(self.ih - 1.0, (y - self.off_y) / self.scale)))

    def find_box(self, x: float, y: float) -> int | None:
        ix, iy = self.c2i(x, y)
        hits = [(b.area(), i) for i, b in enumerate(self.boxes) if b.norm().x1 <= ix <= b.norm().x2 and b.norm().y1 <= iy <= b.norm().y2]
        return min(hits)[1] if hits else None

    def mouse_down(self, event) -> None:
        if self.current_rgb is None:
            return
        hit = self.find_box(event.x, event.y)
        if hit is not None:
            self.selected = hit
            self.select_class(self.boxes[hit].class_id)
            return
        self.selected = None
        self.drag_start = (event.x, event.y)
        self.preview_id = self.canvas.create_rectangle(event.x, event.y, event.x, event.y, outline="white", width=2, dash=(5, 3))

    def mouse_drag(self, event) -> None:
        if self.drag_start is not None and self.preview_id is not None:
            self.canvas.coords(self.preview_id, *self.drag_start, event.x, event.y)

    def mouse_up(self, event) -> None:
        if self.drag_start is None:
            return
        x0, y0 = self.drag_start
        self.drag_start = None
        if self.preview_id is not None:
            self.canvas.delete(self.preview_id)
            self.preview_id = None
        x1, y1 = self.c2i(x0, y0)
        x2, y2 = self.c2i(event.x, event.y)
        box = Box(self.current_class, x1, y1, x2, y2).norm()
        if box.area() >= 36:
            self.boxes.append(box)
            self.selected = len(self.boxes) - 1
        self.redraw()

    def delete(self) -> None:
        if self.selected is not None:
            del self.boxes[self.selected]
            self.selected = None
            self.redraw()

    def undo(self) -> None:
        if self.boxes:
            self.boxes.pop()
            self.selected = None
            self.redraw()

    def start_generation(self) -> None:
        if self.photo_root is None or self.output_root is None:
            messagebox.showerror("错误", "请先选择照片根目录和输出目录")
            return
        try:
            total = int(self.count_var.get())
            ratios = normalize_ratios(float(self.train_var.get()), float(self.val_var.get()), float(self.test_var.get()))
            seed = int(self.seed_var.get())
            neg = float(self.neg_var.get())
            min_obj, max_obj = int(self.min_obj_var.get()), int(self.max_obj_var.get())
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc))
            return
        self.save()
        overwrite = bool(self.overwrite_var.get())
        photo_root = self.photo_root
        output_root = self.output_root
        self.generate_btn.configure(state=tk.DISABLED)

        def worker():
            try:
                generate_dataset(photo_root, output_root, total, ratios, seed, neg,
                                 min_obj, max_obj, overwrite, self.log)
                self.root.after(0, lambda: messagebox.showinfo("完成", "YOLO 数据集生成完成"))
            except Exception as exc:
                error_text = str(exc)
                self.log("[错误] " + error_text)
                self.log(traceback.format_exc())
                self.root.after(0, lambda msg=error_text: messagebox.showerror("生成失败", msg))
            finally:
                self.root.after(0, lambda: self.generate_btn.configure(state=tk.NORMAL))

        threading.Thread(target=worker, daemon=True).start()

    def run(self) -> None:
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.mainloop()

    def close(self) -> None:
        self.save()
        self.root.destroy()


if __name__ == "__main__":
    App().run()
