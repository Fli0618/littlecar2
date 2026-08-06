"""Tkinter 图形化 HSV 阈值调参工具。

启动后通过文件选择器载入图片和配置；图片预览、阈值滑块、ROI 统计、保存
与导出均在同一个桌面窗口完成，不访问相机、串口、YOLO 或 TensorRT。
"""

from __future__ import annotations

import tkinter as tk
import tkinter.font as tkfont
from functools import lru_cache
import os
import math
from dataclasses import dataclass, replace
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Dict, Optional, Sequence, Tuple, Union

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageTk

try:
    from .hsv_tuner import (
        COLOR_ORDER,
        DEFAULT_CONFIG_PATH,
        ImportedConfig,
        PREVIEW_COLORS,
        TunerState,
        build_composite_preview,
        build_color_mask_from_hsv,
        classify_bbox_hsv,
        discover_image_paths,
        import_hsv_config,
        normalize_roi,
        output_filename,
        resolve_default_config_path,
        scale_preview,
    )
except ImportError:  # 允许从文件管理器或 IDE 直接运行本文件
    from hsv_tuner import (  # type: ignore[no-redef]
        COLOR_ORDER,
        DEFAULT_CONFIG_PATH,
        ImportedConfig,
        PREVIEW_COLORS,
        TunerState,
        build_composite_preview,
        build_color_mask_from_hsv,
        classify_bbox_hsv,
        discover_image_paths,
        import_hsv_config,
        normalize_roi,
        output_filename,
        resolve_default_config_path,
        scale_preview,
    )
from vision.hsv_color import (
    HSVClassification,
    HSVColorProfile,
    HSVConfig,
    HSVRange,
    HSVSampleRegion,
    HSVSamplingConfig,
    build_hsv_sample_region,
    build_color_mask_from_hsv as _build_color_mask_from_hsv,
    load_hsv_config,
    save_hsv_config,
)
try:
    from .recommendation import HSVRangeOverlap, HSVRecommendation, analyze_hsv_range_overlap, merge_duplicate_ranges, recommend_hsv_profile
except ImportError:  # 允许直接运行脚本
    from recommendation import HSVRangeOverlap, HSVRecommendation, analyze_hsv_range_overlap, merge_duplicate_ranges, recommend_hsv_profile  # type: ignore[no-redef]


WINDOW_TITLE = "HSV 颜色阈值调参"
DEFAULT_TILE_SIZE = 360
GRID_GAP = 10
TILE_HEADER = 30
TILE_PADDING = 8
_EMPTY_CLASSIFICATION = HSVClassification(None, None, 0.0, 0.0, 0.0, 0.0, 0, 0, {}, (0, 0, 0, 0))


def create_tuner_state(
    image_paths: Sequence[Path],
    config_path: Path = DEFAULT_CONFIG_PATH,
    config: Optional[HSVConfig] = None,
    color_name: str = "red",
) -> TunerState:
    """构造 GUI 使用的可测试调参状态。"""
    paths = list(image_paths)
    loaded = config if config is not None else import_hsv_config(config_path, color_name).config
    available = {profile.name for profile in loaded.colors}
    selected = color_name if color_name in available else next(iter(available), "red")
    return TunerState(paths, Path(config_path).expanduser().resolve(), loaded, loaded, loaded, color_name=selected)


def _odd_value(value: int) -> int:
    value = max(0, min(9, int(value)))
    return 0 if value == 0 else value if value % 2 else value - 1


def safe_hsv_range(lower: Sequence[int], upper: Sequence[int]) -> HSVRange:
    """修正滑块交叉值，确保每个 HSV 通道的 lower 不大于 upper。"""
    low = tuple(min(int(a), int(b)) for a, b in zip(lower, upper))
    high = tuple(max(int(a), int(b)) for a, b in zip(lower, upper))
    return HSVRange(low, high)


@lru_cache(maxsize=1)
def _resolve_cjk_font_path() -> Optional[Path]:
    """按平台查找 CJK 字体；找不到时由调用方回退到英文标签。"""
    windir = Path(os.environ.get("WINDIR", r"C:\Windows"))
    candidates = (
        windir / "Fonts" / "msyh.ttc",
        windir / "Fonts" / "msyh.ttf",
        windir / "Fonts" / "simhei.ttf",
        windir / "Fonts" / "simsun.ttc",
        windir / "Fonts" / "msjh.ttc",
        Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
        Path("/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf"),
        Path("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"),
        Path("/System/Library/Fonts/PingFang.ttc"),
        Path("/System/Library/Fonts/STHeiti Light.ttc"),
    )
    return next((path for path in candidates if path.is_file()), None)


@lru_cache(maxsize=4)
def _load_cjk_font(size: int) -> ImageFont.ImageFont:
    path = _resolve_cjk_font_path()
    if path is not None:
        try:
            return ImageFont.truetype(str(path), size)
        except OSError:
            pass
    return ImageFont.load_default()


def _draw_chart_text(frame: np.ndarray, text: str, origin: Tuple[int, int], color: Tuple[int, int, int], size: int = 16) -> None:
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image = Image.fromarray(rgb)
    ImageDraw.Draw(image).text(origin, text, fill=tuple(reversed(color)), font=_load_cjk_font(size))
    frame[:] = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)


def _select_tk_font(root: tk.Misc) -> str:
    families = set(tkfont.families(root))
    for candidate in ("Microsoft YaHei UI", "Microsoft YaHei", "SimHei", "SimSun", "Microsoft JhengHei", "Noto Sans CJK SC", "WenQuanYi Micro Hei", "PingFang SC", "Heiti SC"):
        if candidate in families:
            return candidate
    return tkfont.nametofont("TkDefaultFont").actual("family")


def _chart_frame(title: str, width: int, height: int) -> np.ndarray:
    frame = np.full((height, width, 3), 31, dtype=np.uint8)
    _draw_chart_text(frame, title if _resolve_cjk_font_path() else title.encode("ascii", "replace").decode("ascii"), (12, 6), (240, 240, 240), 18)
    return frame


def _draw_histogram_panel(
    frame: np.ndarray,
    histogram: np.ndarray,
    maximum: int,
    thresholds: Sequence[Tuple[int, int, int]],
    axis_label: str,
) -> None:
    left, top = 42, 38
    right, bottom = frame.shape[1] - 14, frame.shape[0] - 28
    plot_width, plot_height = right - left, bottom - top
    values = histogram.reshape(-1).astype(np.float32)
    peak = max(float(values.max()), 1.0)
    points = []
    for index, value in enumerate(values):
        x = left + int(index * plot_width / max(1, maximum - 1))
        y = bottom - int(float(value) / peak * (plot_height - 2))
        points.append((x, max(top, y)))
    cv2.rectangle(frame, (left, top), (right, bottom), (90, 90, 90), 1)
    if len(points) > 1:
        cv2.polylines(frame, [np.asarray(points, dtype=np.int32)], False, (220, 220, 220), 1, cv2.LINE_AA)
    for lower, upper, color_index in thresholds:
        color = ((80, 180, 255), (120, 255, 120), (255, 180, 100))[color_index % 3]
        for value in (lower, upper):
            x = left + int(value * plot_width / max(1, maximum - 1))
            cv2.line(frame, (x, top), (x, bottom), color, 1, cv2.LINE_AA)
        cv2.putText(frame, f"{lower}-{upper}", (left + int(lower * plot_width / max(1, maximum - 1)), bottom - 6 - color_index * 14), cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA)
    cv2.putText(frame, axis_label, (right - 28, bottom + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (190, 190, 190), 1, cv2.LINE_AA)


def _legacy_histogram_dashboard(
    hsv_image: np.ndarray,
    roi: Optional[Tuple[int, int, int, int]],
    profile: object,
    width: int = 960,
    height: int = 700,
) -> np.ndarray:
    """生成 ROI 范围内的 H/S/V 直方图和 H-S 热力图。"""
    if not isinstance(hsv_image, np.ndarray) or hsv_image.ndim != 3 or hsv_image.shape[2] != 3:
        raise TypeError("hsv_image must be an HSV image")
    source = hsv_image
    if roi is not None:
        x1, y1, x2, y2 = roi
        source = hsv_image[max(0, y1):min(hsv_image.shape[0], y2), max(0, x1):min(hsv_image.shape[1], x2)]
    if source.size == 0:
        source = hsv_image
    histograms = [cv2.calcHist([source], [channel], None, [bins], [0, bins]).reshape(-1) for channel, bins in ((0, 180), (1, 256), (2, 256))]
    ranges = getattr(profile, "ranges", ())
    thresholds = [
        [(item.lower[0], item.upper[0], index) for index, item in enumerate(ranges)],
        [(item.lower[1], item.upper[1], index) for index, item in enumerate(ranges)],
        [(item.lower[2], item.upper[2], index) for index, item in enumerate(ranges)],
    ]
    dashboard = np.full((height, width, 3), 24, dtype=np.uint8)
    left_width = width // 2
    left_panel = _chart_frame("H/S/V 直方图" if _resolve_cjk_font_path() else "H/S/V histograms", left_width, height)
    histogram_specs = ((histograms[0], 180, "H"), (histograms[1], 256, "S"), (histograms[2], 256, "V"))
    section_height = max(80, (height - 34) // 3)
    for index, (histogram, maximum, label) in enumerate(histogram_specs):
        section = np.full((section_height, left_width, 3), 31, dtype=np.uint8)
        _draw_histogram_panel(section, histogram, maximum, thresholds[index], label)
        if _resolve_cjk_font_path():
            _draw_chart_text(section, f"{label} 通道", (12, 4), (230, 230, 230), 15)
        y = 30 + index * section_height
        left_panel[y:min(height, y + section_height), :left_width] = section[:max(0, min(section_height, height - y)), :left_width]
    dashboard[:, :left_width] = left_panel
    hs_panel = _chart_frame("H-S 二维热力图" if _resolve_cjk_font_path() else "H-S heatmap", left_width, height)
    hs_hist = cv2.calcHist([source], [0, 1], None, [180, 256], [0, 180, 0, 256])
    hs_hist = np.log1p(hs_hist)
    hs_hist = cv2.normalize(hs_hist, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    heatmap = cv2.applyColorMap(hs_hist.T, cv2.COLORMAP_TURBO)
    plot_left, plot_top = 42, 38
    plot_w, plot_h = hs_panel.shape[1] - 56, hs_panel.shape[0] - 66
    heatmap = cv2.resize(heatmap, (plot_w, plot_h), interpolation=cv2.INTER_NEAREST)
    hs_panel[plot_top:plot_top + plot_h, plot_left:plot_left + plot_w] = heatmap
    for index, item in enumerate(ranges):
        color = ((80, 180, 255), (120, 255, 120), (255, 180, 100))[index % 3]
        for hue in (item.lower[0], item.upper[0]):
            x = plot_left + int(hue * (plot_w - 1) / 179)
            cv2.line(hs_panel, (x, plot_top), (x, plot_top + plot_h), color, 1, cv2.LINE_AA)
        for saturation in (item.lower[1], item.upper[1]):
            y = plot_top + plot_h - 1 - int(saturation * (plot_h - 1) / 255)
            cv2.line(hs_panel, (plot_left, y), (plot_left + plot_w, y), color, 1, cv2.LINE_AA)
    cv2.putText(hs_panel, "H", (hs_panel.shape[1] - 28, hs_panel.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(hs_panel, "S", (10, plot_top + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)
    dashboard[:, left_width:width] = hs_panel
    return dashboard


@dataclass(frozen=True)
class TileTransform:
    """正方形卡片内图像与 Canvas 坐标之间的不可变变换。"""

    tile_size: int
    image_size: Tuple[int, int]
    content_rect: Tuple[int, int, int, int]
    image_rect: Tuple[int, int, int, int]
    scale: float
    title_height: int = TILE_HEADER
    padding: int = TILE_PADDING

    @property
    def image_width(self) -> int:
        return self.image_size[0]

    @property
    def image_height(self) -> int:
        return self.image_size[1]


def _font_or_fallback(size: int, bold: bool = False) -> ImageFont.ImageFont:
    font = _load_cjk_font(max(8, int(size)))
    return font


def _title_text(title: str) -> str:
    if _resolve_cjk_font_path() is not None:
        return title
    fallbacks = {
        "原图 + ROI": "Original + ROI",
        "当前颜色 Mask": "Current color mask",
        "H/S/V 直方图": "H/S/V histograms",
        "H-S 二维热力图": "H-S heatmap",
        "请在原图中框选一个物料区域": "Select an ROI in the original image",
        "尚未选择 ROI": "ROI not selected",
        "H 通道": "H channel",
        "S 通道": "S channel",
        "V 通道": "V channel",
    }
    return fallbacks.get(title, title.encode("ascii", "replace").decode("ascii"))


def compose_square_tile(
    image_bgr: np.ndarray,
    tile_size: int,
    title: str,
    background: Tuple[int, int, int] = (31, 31, 31),
) -> Tuple[np.ndarray, TileTransform]:
    """以 contain 方式把 BGR 图像绘制到最终正方形卡片。"""
    if not isinstance(image_bgr, np.ndarray) or image_bgr.ndim != 3 or image_bgr.shape[2] != 3 or image_bgr.size == 0:
        raise ValueError("image_bgr must be a non-empty BGR image")
    tile_size = max(1, int(tile_size))
    height, width = image_bgr.shape[:2]
    content_left = TILE_PADDING
    content_top = TILE_HEADER + TILE_PADDING
    content_right = tile_size - TILE_PADDING
    content_bottom = tile_size - TILE_PADDING
    content_width = max(1, content_right - content_left)
    content_height = max(1, content_bottom - content_top)
    scale = min(content_width / float(width), content_height / float(height))
    resized_width = max(1, int(round(width * scale)))
    resized_height = max(1, int(round(height * scale)))
    resized = cv2.resize(image_bgr, (resized_width, resized_height), interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR)
    image_left = content_left + (content_width - resized_width) // 2
    image_top = content_top + (content_height - resized_height) // 2
    tile = np.full((tile_size, tile_size, 3), tuple(int(value) for value in background), dtype=np.uint8)
    tile[image_top:image_top + resized_height, image_left:image_left + resized_width] = resized
    cv2.rectangle(tile, (0, 0), (tile_size - 1, tile_size - 1), (92, 92, 92), 1)
    cv2.line(tile, (0, TILE_HEADER - 1), (tile_size - 1, TILE_HEADER - 1), (70, 70, 70), 1)
    rgb = cv2.cvtColor(tile, cv2.COLOR_BGR2RGB)
    pil = Image.fromarray(rgb)
    draw = ImageDraw.Draw(pil)
    draw.text((10, 5), _title_text(str(title)), fill=(238, 238, 238), font=_font_or_fallback(13))
    tile = cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)
    transform = TileTransform(
        tile_size=tile_size,
        image_size=(width, height),
        content_rect=(content_left, content_top, content_right, content_bottom),
        image_rect=(image_left, image_top, image_left + resized_width, image_top + resized_height),
        scale=scale,
    )
    return tile, transform


def _unpack_point(point_or_x: Union[Sequence[float], float], y: Optional[float]) -> Tuple[float, float]:
    if y is None:
        if not isinstance(point_or_x, Sequence) or len(point_or_x) != 2:
            raise ValueError("point must contain x and y")
        return float(point_or_x[0]), float(point_or_x[1])
    return float(point_or_x), float(y)


def canvas_point_to_image(
    point_or_x: Union[Sequence[float], float],
    transform: Union[TileTransform, float],
    y: Optional[Union[float, TileTransform]] = None,
    tile_origin: Tuple[float, float] = (0.0, 0.0),
) -> Optional[Tuple[int, int]]:
    """将 Canvas 点映射到原图；标题栏和 letterbox 留白返回 None。"""
    if not isinstance(transform, TileTransform) and isinstance(y, TileTransform):
        transform, y = y, transform
    if not isinstance(transform, TileTransform):
        raise TypeError("transform must be a TileTransform")
    x, point_y = _unpack_point(point_or_x, y)
    x -= float(tile_origin[0])
    point_y -= float(tile_origin[1])
    left, top, right, bottom = transform.image_rect
    if x < left or x >= right or point_y < top or point_y >= bottom:
        return None
    image_x = int(math.floor((x - left) / transform.scale))
    image_y = int(math.floor((point_y - top) / transform.scale))
    if image_x < 0 or image_y < 0 or image_x >= transform.image_width or image_y >= transform.image_height:
        return None
    return image_x, image_y


def image_point_to_canvas(
    point_or_x: Union[Sequence[float], float],
    transform: Union[TileTransform, float],
    y: Optional[Union[float, TileTransform]] = None,
    tile_origin: Tuple[float, float] = (0.0, 0.0),
) -> Optional[Tuple[int, int]]:
    """将原图点映射到卡片 Canvas 坐标。"""
    if not isinstance(transform, TileTransform) and isinstance(y, TileTransform):
        transform, y = y, transform
    if not isinstance(transform, TileTransform):
        raise TypeError("transform must be a TileTransform")
    x, point_y = _unpack_point(point_or_x, y)
    if x < 0 or point_y < 0 or x >= transform.image_width or point_y >= transform.image_height:
        return None
    left, top, _, _ = transform.image_rect
    return (
        int(round(float(tile_origin[0]) + left + x * transform.scale)),
        int(round(float(tile_origin[1]) + top + point_y * transform.scale)),
    )


def _masked_values(hsv_image: np.ndarray, sample_mask: Optional[np.ndarray]) -> np.ndarray:
    if sample_mask is None:
        return np.empty((0, 3), dtype=np.uint8)
    if sample_mask.shape != hsv_image.shape[:2]:
        raise ValueError("sample_mask shape must match hsv_image")
    return hsv_image[sample_mask > 0].reshape(-1, 3)


def _thresholds(profile: object, channel: int) -> Sequence[Tuple[int, int, int]]:
    return tuple((int(item.lower[channel]), int(item.upper[channel]), index) for index, item in enumerate(getattr(profile, "ranges", ())))


def _draw_text_bgr(frame: np.ndarray, text: str, xy: Tuple[int, int], size: int, color: Tuple[int, int, int] = (235, 235, 235)) -> None:
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image = Image.fromarray(rgb)
    ImageDraw.Draw(image).text(xy, _title_text(text), font=_font_or_fallback(size), fill=tuple(reversed(color)))
    frame[:] = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)


def _plot_histogram_tile(
    values: np.ndarray,
    maximum: int,
    thresholds: Sequence[Tuple[int, int, int]],
    title: str,
    quantiles: Sequence[float],
    size: int,
) -> np.ndarray:
    frame = np.full((size, size, 3), 31, dtype=np.uint8)
    margin_left, margin_right = 42, 12
    top, bottom = 38, size - 26
    plot_w, plot_h = max(1, size - margin_left - margin_right), max(1, bottom - top)
    values = values.astype(np.float32).reshape(-1)
    peak = max(float(values.max()) if values.size else 0.0, 1.0)
    for lower, upper, index in thresholds:
        color = ((80, 180, 255), (120, 255, 120), (255, 180, 100))[index % 3]
        x1 = margin_left + int(round(lower * (plot_w - 1) / max(1, maximum - 1)))
        x2 = margin_left + int(round(upper * (plot_w - 1) / max(1, maximum - 1)))
        overlay = frame.copy()
        cv2.rectangle(overlay, (min(x1, x2), top), (max(x1, x2), bottom), color, -1)
        frame[:] = cv2.addWeighted(overlay, 0.20, frame, 0.80, 0)
        cv2.line(frame, (x1, top), (x1, bottom), color, 2, cv2.LINE_AA)
        cv2.line(frame, (x2, top), (x2, bottom), color, 2, cv2.LINE_AA)
    points = []
    for index, value in enumerate(values):
        x = margin_left + int(round(index * (plot_w - 1) / max(1, maximum - 1)))
        y = bottom - int(round(float(value) / peak * max(1, plot_h - 2)))
        points.append((x, max(top, y)))
    cv2.rectangle(frame, (margin_left, top), (margin_left + plot_w - 1, bottom), (100, 100, 100), 1)
    if len(points) > 1:
        cv2.polylines(frame, [np.asarray(points, dtype=np.int32)], False, (225, 225, 225), 1, cv2.LINE_AA)
    _draw_text_bgr(frame, title, (10, 6), max(11, size // 18))
    _draw_text_bgr(frame, f"0  max {maximum - 1}", (margin_left, bottom + 4), max(8, size // 28), (170, 170, 170))
    if quantiles:
        _draw_text_bgr(frame, "P05/P50/P95 " + "/".join(str(int(value)) for value in quantiles), (margin_left, top + 4), max(8, size // 30), (180, 200, 220))
    return frame


def render_hsv_histogram_tile(
    hsv_image: np.ndarray,
    sample_mask: Optional[np.ndarray],
    profile: object,
    size: int,
) -> np.ndarray:
    """直接按最终正方形尺寸渲染 H/S/V 三通道直方图。"""
    if not isinstance(hsv_image, np.ndarray) or hsv_image.ndim != 3 or hsv_image.shape[2] != 3:
        raise TypeError("hsv_image must be an HSV image")
    size = max(1, int(size))
    values = _masked_values(hsv_image, sample_mask)
    if values.size == 0:
        frame = np.full((size, size, 3), 31, dtype=np.uint8)
        _draw_text_bgr(frame, "H/S/V 直方图", (10, 8), max(12, size // 18))
        _draw_text_bgr(frame, "请在原图中框选一个物料区域", (10, size // 2 - 10), max(10, size // 24), (200, 200, 200))
        return frame
    histograms = [np.bincount(values[:, channel], minlength=bins)[:bins] for channel, bins in ((0, 180), (1, 256), (2, 256))]
    rows = []
    row_height = max(50, (size - TILE_HEADER - 16) // 3)
    for index, (histogram, maximum, title) in enumerate(zip(histograms, (180, 256, 256), ("H 通道", "S 通道", "V 通道"))):
        quantiles = np.percentile(values[:, index], (5, 50, 95)).astype(int).tolist()
        rows.append(_plot_histogram_tile(histogram, maximum, _thresholds(profile, index), title, quantiles, size))
    frame = np.full((size, size, 3), 31, dtype=np.uint8)
    usable_top = TILE_HEADER
    for index, row in enumerate(rows):
        y1 = usable_top + index * row_height
        y2 = min(size, y1 + row_height)
        frame[y1:y2] = cv2.resize(row, (size, row_height), interpolation=cv2.INTER_AREA)[0:y2 - y1]
    return frame


def render_hs_heatmap_tile(
    hsv_image: np.ndarray,
    sample_mask: Optional[np.ndarray],
    profile: object,
    size: int,
) -> np.ndarray:
    """直接按最终正方形尺寸渲染 H 横轴、S 纵轴的热力图。"""
    if not isinstance(hsv_image, np.ndarray) or hsv_image.ndim != 3 or hsv_image.shape[2] != 3:
        raise TypeError("hsv_image must be an HSV image")
    size = max(1, int(size))
    frame = np.full((size, size, 3), 31, dtype=np.uint8)
    values = _masked_values(hsv_image, sample_mask)
    _draw_text_bgr(frame, "H-S 二维热力图", (10, 7), max(12, size // 18))
    if values.size == 0:
        _draw_text_bgr(frame, "请在原图中框选一个物料区域", (10, size // 2 - 10), max(10, size // 24), (200, 200, 200))
        return frame
    hist = np.zeros((256, 180), dtype=np.float32)
    np.add.at(hist, (values[:, 1], values[:, 0]), 1.0)
    heat = np.log1p(hist)
    if float(heat.max()) > 0:
        heat = (heat / float(heat.max()) * 255.0).astype(np.uint8)
    else:
        heat = np.zeros_like(heat, dtype=np.uint8)
    # OpenCV histogram 的 S 轴从 0 到 255 向下排列，显示时翻转为高饱和度在上。
    heat = np.flipud(heat)
    plot_left, plot_top = 40, 38
    plot_right, plot_bottom = size - 12, size - 30
    plot_w, plot_h = max(1, plot_right - plot_left), max(1, plot_bottom - plot_top)
    colored = cv2.applyColorMap(heat, cv2.COLORMAP_TURBO)
    colored = cv2.resize(colored, (plot_w, plot_h), interpolation=cv2.INTER_NEAREST)
    frame[plot_top:plot_bottom, plot_left:plot_right] = colored
    for item in getattr(profile, "ranges", ()):
        x1 = plot_left + int(round(item.lower[0] * (plot_w - 1) / 179))
        x2 = plot_left + int(round(item.upper[0] * (plot_w - 1) / 179))
        y1 = plot_top + int(round((255 - item.upper[1]) * (plot_h - 1) / 255))
        y2 = plot_top + int(round((255 - item.lower[1]) * (plot_h - 1) / 255))
        color = ((80, 180, 255), (120, 255, 120), (255, 180, 100))[getattr(profile, "ranges", ()).index(item) % 3]
        overlay = frame.copy()
        cv2.rectangle(overlay, (min(x1, x2), min(y1, y2)), (max(x1, x2), max(y1, y2)), color, -1)
        frame[:] = cv2.addWeighted(overlay, 0.12, frame, 0.88, 0)
        cv2.rectangle(frame, (min(x1, x2), min(y1, y2)), (max(x1, x2), max(y1, y2)), color, 2)
    for hue in (0, 30, 60, 90, 120, 150, 179):
        x = plot_left + int(round(hue * (plot_w - 1) / 179))
        cv2.line(frame, (x, plot_bottom), (x, plot_bottom + 3), (205, 205, 205), 1)
        _draw_text_bgr(frame, str(hue), (max(0, x - 8), plot_bottom + 5), max(7, size // 40), (190, 190, 190))
    for saturation in (0, 64, 128, 192, 255):
        y = plot_top + int(round((255 - saturation) * (plot_h - 1) / 255))
        cv2.line(frame, (plot_left - 3, y), (plot_left, y), (205, 205, 205), 1)
        _draw_text_bgr(frame, str(saturation), (2, max(plot_top, y - 5)), max(7, size // 40), (190, 190, 190))
    return frame


def build_histogram_dashboard(
    hsv_image: np.ndarray,
    roi: Optional[Tuple[int, int, int, int]],
    profile: object,
    width: int = 960,
    height: int = 700,
) -> np.ndarray:
    """兼容旧调用方；新 GUI 使用正方形 tile 渲染函数。"""
    mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
    if roi is not None:
        try:
            region = build_hsv_sample_region(roi, hsv_image.shape[1], hsv_image.shape[0], HSVSamplingConfig(1.0, 1.0, 1, 0.0, 0.0))
        except (TypeError, ValueError):
            region = None
        if region is not None:
            x1, y1, x2, y2 = region.sample_bbox
            mask[y1:y2, x1:x2] = region.ellipse_mask
    tile_size = max(64, min(int(height), int(width // 2)))
    hist_tile = render_hsv_histogram_tile(hsv_image, mask if roi is not None else None, profile, tile_size)
    hs_tile = render_hs_heatmap_tile(hsv_image, mask if roi is not None else None, profile, tile_size)
    dashboard = np.full((int(height), int(width), 3), 24, dtype=np.uint8)
    dashboard[:tile_size, :tile_size] = hist_tile
    dashboard[:tile_size, width - tile_size:width] = hs_tile
    return dashboard


def imwrite_unicode(path: Union[str, Path], image: np.ndarray) -> bool:
    """使用 imencode + tofile 支持 Windows 中文路径导出。"""
    suffix = Path(path).suffix or ".png"
    success, encoded = cv2.imencode(suffix, image)
    if not success:
        return False
    try:
        encoded.tofile(str(path))
    except OSError:
        return False
    return True


class HSVTunerGUI:
    """Tkinter 主窗口和调参交互控制器。"""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.font_family = _select_tk_font(root)
        try:
            dpi = float(root.winfo_fpixels("1i"))
        except tk.TclError:
            dpi = 96.0
        self.dpi_scale = max(0.9, min(1.5, dpi / 96.0))
        try:
            root.tk.call("tk", "scaling", self.dpi_scale)
        except tk.TclError:
            pass
        self.ui_font = (self.font_family, max(11, int(round(11 * self.dpi_scale))))
        self.root.option_add("*Font", self.ui_font)
        style = ttk.Style(root)
        for name in ("TLabel", "TButton", "TCheckbutton", "TCombobox", "TLabelframe.Label"):
            style.configure(name, font=self.ui_font)
        self.root.title(WINDOW_TITLE)
        self.root.geometry("1480x940")
        self.root.minsize(1180, 780)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.state: Optional[TunerState] = None
        self.output_dir: Optional[Path] = None
        self._refreshing = False
        self._drag_start: Optional[Tuple[int, int]] = None
        self._drag_current: Optional[Tuple[int, int]] = None
        self._photo_refs: list[ImageTk.PhotoImage] = []
        self._preview_bgr: Optional[np.ndarray] = None
        self._preview_hsv: Optional[np.ndarray] = None
        self._preview_scale = 1.0
        self._analysis_bgr: Optional[np.ndarray] = None
        self._analysis_hsv: Optional[np.ndarray] = None
        self._analysis_scale_x = 1.0
        self._analysis_scale_y = 1.0
        self._original_transform: Optional[TileTransform] = None
        self._grid_origin: Tuple[int, int] = (0, 0)
        self._tile_size = DEFAULT_TILE_SIZE
        self._grid_gap = GRID_GAP
        self._resize_job: Optional[str] = None
        self._active_tab = 0
        self._full_image_analysis = False
        self._recommendation: Optional[HSVRecommendation] = None
        self._recommendation_before: Optional[Tuple[HSVRange, ...]] = None
        self._overlap_report: Optional[HSVRangeOverlap] = None
        self._refresh_job: Optional[str] = None
        self._build_startup()

    def _build_startup(self) -> None:
        self.startup = ttk.Frame(self.root, padding=32)
        self.startup.pack(fill=tk.BOTH, expand=True)
        ttk.Label(self.startup, text=WINDOW_TITLE, font=(self.font_family, 22, "bold")).pack(anchor=tk.W)
        ttk.Label(self.startup, text="选择图片目录后开始调参；HSV 配置默认自动加载", padding=(0, 8, 0, 16)).pack(anchor=tk.W)
        self.path_labels: Dict[str, ttk.Label] = {}
        self.config_path = resolve_default_config_path()
        self.startup_config: Optional[HSVConfig] = None
        self.config_status = tk.StringVar(value="")
        self._default_config_error: Optional[str] = None
        self._load_startup_default_config()
        for key, title, value in (
            ("image", "图片来源", "未选择"),
            ("config", "HSV 配置", str(self.config_path)),
            ("output", "导出目录", "未设置"),
        ):
            row = ttk.Frame(self.startup)
            row.pack(fill=tk.X, pady=6)
            ttk.Label(row, text=title, width=12).pack(side=tk.LEFT)
            label = ttk.Label(row, text=value, relief=tk.GROOVE, padding=8)
            label.pack(side=tk.LEFT, fill=tk.X, expand=True)
            self.path_labels[key] = label
        ttk.Label(self.startup, textvariable=self.config_status, foreground="#b36b00", padding=(0, 2, 0, 8)).pack(anchor=tk.W)
        actions = ttk.Frame(self.startup)
        actions.pack(fill=tk.X, pady=(20, 8))
        ttk.Button(actions, text="选择图片目录", command=self.choose_directory).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(actions, text="选择单张图片", command=self.choose_image).pack(side=tk.LEFT, padx=8)
        ttk.Button(actions, text="选择其他配置", command=self.choose_config).pack(side=tk.LEFT, padx=8)
        ttk.Button(actions, text="选择输出目录", command=self.choose_output).pack(side=tk.LEFT, padx=8)
        self.start_button = ttk.Button(self.startup, text="打开调参界面", command=self.start_tuning, state=tk.DISABLED)
        self.start_button.pack(anchor=tk.W, pady=(28, 0))
        self.image_source: Optional[Path] = None
        self.image_is_directory = False

    def _load_startup_default_config(self) -> None:
        try:
            imported = import_hsv_config(self.config_path, "red")
        except (OSError, ValueError) as exc:
            self.startup_config = None
            self._default_config_error = f"默认配置不可用：{self.config_path}\n{exc}"
            self.config_status.set(self._default_config_error)
            return
        self.startup_config = imported.config
        self._default_config_error = None
        self.config_status.set(f"已加载默认配置：{imported.config_path}")

    def _set_path_label(self, key: str, value: Optional[Path]) -> None:
        self.path_labels[key].configure(text=str(value) if value else "未设置")

    def _update_start_button(self) -> None:
        self.start_button.configure(state=tk.NORMAL if self.image_source and self.startup_config is not None else tk.DISABLED)

    def choose_directory(self) -> None:
        selected = filedialog.askdirectory(title="选择图片目录")
        if selected:
            self.image_source = Path(selected)
            self.image_is_directory = True
            self._set_path_label("image", self.image_source)
            self._update_start_button()

    def choose_image(self) -> None:
        selected = filedialog.askopenfilename(title="选择图片", filetypes=[("图片", "*.jpg *.jpeg *.png *.bmp *.webp"), ("所有文件", "*.*")])
        if selected:
            self.image_source = Path(selected)
            self.image_is_directory = False
            self._set_path_label("image", self.image_source)
            self._update_start_button()

    def choose_config(self) -> None:
        selected = filedialog.askopenfilename(title="选择 HSV 配置", initialfile=str(self.config_path), filetypes=[("JSON", "*.json"), ("所有文件", "*.*")])
        if selected:
            try:
                imported = import_hsv_config(selected, "red")
            except (OSError, ValueError) as exc:
                self.startup_config = None
                self.config_status.set(f"配置无效：{Path(selected).resolve()}\n{exc}")
                self._update_start_button()
                return
            self.config_path = imported.config_path
            self.startup_config = imported.config
            self.config_status.set(f"已加载配置：{self.config_path}")
            self._set_path_label("config", self.config_path)
            self._update_start_button()

    def choose_output(self) -> None:
        selected = filedialog.askdirectory(title="选择导出目录")
        if selected:
            self.output_dir = Path(selected)
            self._set_path_label("output", self.output_dir)

    def start_tuning(self) -> None:
        if self.image_source is None:
            return
        try:
            paths = discover_image_paths(self.image_source) if self.image_is_directory else [self.image_source]
            if not paths:
                raise ValueError("未找到可用图片")
            self.state = create_tuner_state(paths, self.config_path, config=self.startup_config)
            self.state.config_is_default = self.config_path == resolve_default_config_path()
            self.state.config_status = self.config_status.get()
            if not self.state.load_current_image():
                raise ValueError("图片全部无法读取")
        except (OSError, ValueError, cv2.error) as exc:
            messagebox.showerror("加载失败", str(exc), parent=self.root)
            return
        self.startup.destroy()
        self._build_editor()
        self._prepare_preview()
        self._load_controls_from_state()
        self.refresh_view()

    def _build_editor(self) -> None:
        toolbar = ttk.Frame(self.root, padding=(8, 8, 8, 4))
        toolbar.pack(fill=tk.X)
        for title, command in (("首张", lambda: self.navigate(0)), ("上一张", lambda: self.navigate(-1)), ("下一张", lambda: self.navigate(1)), ("末张", lambda: self.navigate(-2))):
            ttk.Button(toolbar, text=title, command=command).pack(side=tk.LEFT, padx=2)
        self.image_status = ttk.Label(toolbar, text="")
        self.image_status.pack(side=tk.LEFT, padx=12)
        for title, command in (
            ("导入默认参数", self.import_default_config),
            ("导入配置文件", self.import_external_config),
            ("重载当前配置", self.reload_config),
            ("保存", self.save_config),
            ("另存为", self.save_as_config),
            ("恢复当前颜色", self.restore_current),
            ("恢复全部", self.restore_all),
            ("导出 Mask", lambda: self.export("mask")),
            ("导出四宫格", lambda: self.export("preview")),
        ):
            ttk.Button(toolbar, text=title, command=command).pack(side=tk.RIGHT, padx=2)

        body = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 4))
        left = ttk.Frame(body)
        right = ttk.Frame(body, width=440)
        body.add(left, weight=4)
        body.add(right, weight=1)
        self.preview_frame = left
        self.canvas = tk.Canvas(left, width=DEFAULT_TILE_SIZE * 2 + GRID_GAP, height=DEFAULT_TILE_SIZE * 2 + GRID_GAP, background="#202020", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<ButtonPress-1>", self.canvas_press)
        self.canvas.bind("<B1-Motion>", self.canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self.canvas_release)
        self.canvas.bind("<Button-3>", lambda _event: self.clear_roi())
        self.canvas.bind("<Configure>", self._on_canvas_resize)
        self.canvas.bind("<ButtonPress-2>", lambda _event: self.clear_roi())
        self.controls = right
        self._build_controls()
        self._build_status_bar()
        self.root.bind_all("<Key>", self._on_key, add="+")

    def _build_controls(self) -> None:
        self.color_var = tk.StringVar()
        self.range2_var = tk.BooleanVar()
        self.enabled_var = tk.BooleanVar()
        self.scales: Dict[str, ttk.Scale] = {}
        self.spinboxes: Dict[str, ttk.Spinbox] = {}
        self.scale_vars: Dict[str, tk.DoubleVar] = {}
        self.range2_widgets: list[tk.Widget] = []
        notebook = ttk.Notebook(self.controls)
        notebook.pack(fill=tk.BOTH, expand=True)
        self.notebook = notebook
        color_outer, color_tab = self._scrollable_tab(notebook)
        process_outer, process_tab = self._scrollable_tab(notebook)
        stats_tab = ttk.Frame(notebook, padding=10)
        notebook.add(color_outer, text="颜色阈值")
        notebook.add(process_outer, text="处理与采样")
        notebook.add(stats_tab, text="统计与诊断")
        self.color_combo = ttk.Combobox(color_tab, textvariable=self.color_var, state="readonly", values=COLOR_ORDER)
        self.color_combo.pack(fill=tk.X, pady=(0, 6))
        self.color_combo.bind("<<ComboboxSelected>>", lambda _event: self.change_color())
        ttk.Checkbutton(color_tab, text="启用当前颜色", variable=self.enabled_var, command=self.toggle_current_color).pack(anchor=tk.W, pady=3)
        recommendation_actions = ttk.Frame(color_tab)
        recommendation_actions.pack(fill=tk.X, pady=(2, 6))
        self.recommend_button = ttk.Button(recommendation_actions, text="一键推荐当前颜色", command=self.recommend_current_color, state=tk.DISABLED)
        self.recommend_button.pack(side=tk.LEFT, padx=(0, 6))
        self.undo_recommend_button = ttk.Button(recommendation_actions, text="撤销推荐", command=self.undo_recommendation, state=tk.DISABLED)
        self.undo_recommend_button.pack(side=tk.LEFT)
        self.recommend_hint = ttk.Label(color_tab, text="推荐值仅基于当前 ROI，是调参初始值；保存前请浏览其他图片验证。", wraplength=360)
        self.recommend_hint.pack(anchor=tk.W, pady=(0, 6))
        ttk.Checkbutton(color_tab, text="启用 HSV 区间 B", variable=self.range2_var, command=self.controls_changed).pack(anchor=tk.W, pady=3)
        ttk.Label(color_tab, text="用于红色跨越 H=0/179 或其他离散色相分布；普通颜色通常关闭。", wraplength=360).pack(anchor=tk.W, pady=(0, 4))
        for group_title, prefix in (("第一组", "1"), ("第二组", "2")):
            display_title = "HSV 区间 A" if prefix == "1" else "HSV 区间 B"
            frame = ttk.LabelFrame(color_tab, text=display_title, padding=6)
            frame.pack(fill=tk.X, pady=5)
            if prefix == "2":
                self.range2_widgets.append(frame)
            for channel, maximum in (("H", 179), ("S", 255), ("V", 255)):
                row = ttk.Frame(frame)
                row.pack(fill=tk.X, pady=2)
                ttk.Label(row, text=f"{channel}{prefix}", width=4).pack(side=tk.LEFT)
                for bound in ("Min", "Max"):
                    name = f"{channel}{prefix} {bound}"
                    var = tk.DoubleVar(value=0)
                    scale = ttk.Scale(row, from_=0, to=maximum, variable=var, command=lambda _value, key=name: self.controls_changed(key))
                    scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
                    spin = ttk.Spinbox(row, from_=0, to=maximum, width=5, textvariable=var, command=lambda key=name: self.controls_changed(key))
                    spin.pack(side=tk.LEFT, padx=(2, 0))
                    spin.bind("<FocusOut>", lambda _event, key=name: self.controls_changed(key))
                    self.scales[name] = scale
                    self.spinboxes[name] = spin
                    self.scale_vars[name] = var

        process_specs = (
            ("Blur Kernel", 9), ("Open Kernel", 9), ("Open Iterations", 5), ("Close Kernel", 9),
            ("Close Iterations", 5), ("Sample Scale X", 100), ("Sample Scale Y", 100),
            ("Min Coverage", 100), ("Min Margin", 100), ("Min Pixels", 50000),
        )
        for name, maximum in process_specs:
            self._add_slider(process_tab, name, maximum)
        self.analysis_var = tk.StringVar(value="部署分辨率 640×480")
        ttk.Label(process_tab, text="分析分辨率").pack(anchor=tk.W, pady=(8, 2))
        ttk.Combobox(process_tab, textvariable=self.analysis_var, state="readonly", values=("部署分辨率 640×480", "原始分辨率"),).pack(fill=tk.X)
        self.analysis_var.trace_add("write", lambda *_args: self._prepare_preview_and_refresh())
        self.full_analysis_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(process_tab, text="允许无 ROI 时分析整图（默认关闭）", variable=self.full_analysis_var, command=self.refresh_view).pack(anchor=tk.W, pady=6)
        self.stats = tk.Text(stats_tab, height=16, width=44, state=tk.DISABLED, wrap=tk.WORD, font=("Consolas", max(10, int(10 * self.dpi_scale))))
        self.stats.pack(fill=tk.BOTH, expand=True)
        self.merge_ranges_button = ttk.Button(stats_tab, text="合并重复区间", command=self.merge_current_ranges)
        self.merge_ranges_button.pack(anchor=tk.W, pady=(6, 0))

    def _scrollable_tab(self, parent: ttk.Notebook) -> Tuple[ttk.Frame, ttk.Frame]:
        outer = ttk.Frame(parent)
        canvas = tk.Canvas(outer, highlightthickness=0, background="#202020")
        scrollbar = ttk.Scrollbar(outer, orient=tk.VERTICAL, command=canvas.yview)
        inner = ttk.Frame(canvas, padding=10)
        window_id = canvas.create_window((0, 0), window=inner, anchor=tk.NW)
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        inner.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(window_id, width=event.width))
        return outer, inner

    def _add_slider(self, parent: tk.Widget, name: str, maximum: int) -> None:
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text=name, width=18).pack(side=tk.LEFT)
        var = tk.DoubleVar(value=0)
        scale = ttk.Scale(row, from_=0, to=maximum, variable=var, command=lambda _value, key=name: self.controls_changed(key))
        scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        spin = ttk.Spinbox(row, from_=0, to=maximum, width=7, textvariable=var, command=lambda key=name: self.controls_changed(key))
        spin.pack(side=tk.LEFT)
        spin.bind("<FocusOut>", lambda _event, key=name: self.controls_changed(key))
        self.scales[name] = scale
        self.spinboxes[name] = spin
        self.scale_vars[name] = var

    def _build_status_bar(self) -> None:
        self.status_var = tk.StringVar(value="就绪")
        ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W, padding=(8, 4)).pack(fill=tk.X, side=tk.BOTTOM)

    def _on_canvas_resize(self, _event: tk.Event) -> None:
        if self._resize_job is not None:
            self.root.after_cancel(self._resize_job)
        self._resize_job = self.root.after(100, self.refresh_view)

    def _load_controls_from_state(self) -> None:
        assert self.state is not None
        self._refreshing = True
        try:
            profile = self.state.current_profile()
            first = profile.ranges[0]
            second = profile.ranges[1] if len(profile.ranges) > 1 else HSVRange((0, 0, 0), (0, 0, 0))
            values = {"H1 Min": first.lower[0], "H1 Max": first.upper[0], "S1 Min": first.lower[1], "S1 Max": first.upper[1], "V1 Min": first.lower[2], "V1 Max": first.upper[2], "H2 Min": second.lower[0], "H2 Max": second.upper[0], "S2 Min": second.lower[1], "S2 Max": second.upper[1], "V2 Min": second.lower[2], "V2 Max": second.upper[2], "Blur Kernel": self.state.working_config.processing.blur_kernel, "Open Kernel": self.state.working_config.processing.open_kernel, "Open Iterations": self.state.working_config.processing.open_iterations, "Close Kernel": self.state.working_config.processing.close_kernel, "Close Iterations": self.state.working_config.processing.close_iterations, "Sample Scale X": round(self.state.working_config.sampling.scale_x * 100), "Sample Scale Y": round(self.state.working_config.sampling.scale_y * 100), "Min Coverage": round(self.state.working_config.sampling.min_coverage * 100), "Min Margin": round(self.state.working_config.sampling.min_margin * 100), "Min Pixels": self.state.working_config.sampling.min_pixels}
            for name, value in values.items():
                if name in self.scale_vars:
                    self.scale_vars[name].set(float(value))
            self.range2_var.set(len(profile.ranges) > 1)
            self.enabled_var.set(bool(profile.enabled))
            self.color_var.set(self.state.color_name)
            self._set_range2_state()
            self._update_recommend_controls()
        finally:
            self._refreshing = False

    def change_color(self) -> None:
        if self.state is None:
            return
        self.state.color_name = self.color_var.get()
        self._load_controls_from_state()
        self.refresh_view()

    def _set_range2_state(self) -> None:
        enabled = bool(self.range2_var.get())
        state = tk.NORMAL if enabled else tk.DISABLED
        for widget in self.range2_widgets:
            try:
                widget.configure(state=state)
            except tk.TclError:
                pass
        for name, spin in self.spinboxes.items():
            if name.startswith(("H2", "S2", "V2")):
                try:
                    spin.configure(state="normal" if enabled else "disabled")
                except tk.TclError:
                    pass

    def _update_recommend_controls(self) -> None:
        if not hasattr(self, "recommend_button"):
            return
        has_roi = self.state is not None and self.state.roi is not None and self.state.image_bgr is not None
        self.recommend_button.configure(state=tk.NORMAL if has_roi else tk.DISABLED)
        has_backup = self.state is not None and self.state.recommendation_backup is not None
        self.undo_recommend_button.configure(state=tk.NORMAL if has_backup else tk.DISABLED)

    def toggle_current_color(self) -> None:
        if self.state is None or self._refreshing:
            return
        profile = replace(self.state.current_profile(), enabled=bool(self.enabled_var.get()))
        self.state.working_config = self.state.working_config.__class__(
            self.state.working_config.version,
            tuple(profile if item.name == profile.name else item for item in self.state.working_config.colors),
            self.state.working_config.processing,
            self.state.working_config.sampling,
        )
        self.state.dirty = True
        self.refresh_view()

    def _value(self, name: str) -> int:
        try:
            return int(round(float(self.scale_vars[name].get())))
        except (KeyError, tk.TclError, ValueError):
            return 0

    def controls_changed(self, changed_name: Optional[str] = None) -> None:
        if self._refreshing or self.state is None:
            return
        values = {name: self._value(name) for name in self.scale_vars}
        for channel, maximum in (("H", 179), ("S", 255), ("V", 255)):
            for group in ("1", "2"):
                minimum = f"{channel}{group} Min"
                maximum_name = f"{channel}{group} Max"
                low = max(0, min(maximum, values[minimum]))
                high = max(0, min(maximum, values[maximum_name]))
                if low > high:
                    if changed_name == minimum:
                        low = high
                    else:
                        high = low
                values[minimum], values[maximum_name] = low, high
        first = safe_hsv_range((values["H1 Min"], values["S1 Min"], values["V1 Min"]), (values["H1 Max"], values["S1 Max"], values["V1 Max"]))
        second = safe_hsv_range((values["H2 Min"], values["S2 Min"], values["V2 Min"]), (values["H2 Max"], values["S2 Max"], values["V2 Max"]))
        self._refreshing = True
        try:
            for name, value in {
                "H1 Min": first.lower[0], "H1 Max": first.upper[0], "S1 Min": first.lower[1], "S1 Max": first.upper[1], "V1 Min": first.lower[2], "V1 Max": first.upper[2],
                "H2 Min": second.lower[0], "H2 Max": second.upper[0], "S2 Min": second.lower[1], "S2 Max": second.upper[1], "V2 Min": second.lower[2], "V2 Max": second.upper[2],
            }.items():
                if name in self.scale_vars:
                    self.scale_vars[name].set(float(value))
        finally:
            self._refreshing = False
        self._set_range2_state()
        profile = self.state.current_profile()
        ranges = [first]
        if self.range2_var.get():
            ranges.append(second)
        ranges.extend(profile.ranges[2:])
        processing = self.state.working_config.processing
        processing = processing.__class__(_odd_value(values["Blur Kernel"]), _odd_value(values["Open Kernel"]), values["Open Iterations"], _odd_value(values["Close Kernel"]), values["Close Iterations"])
        sampling = self.state.working_config.sampling
        sampling = sampling.__class__(values["Sample Scale X"] / 100.0, values["Sample Scale Y"] / 100.0, values["Min Pixels"], values["Min Coverage"] / 100.0, values["Min Margin"] / 100.0)
        self.state.update_current_profile(tuple(ranges))
        self.state.working_config = self.state.working_config.__class__(self.state.working_config.version, self.state.working_config.colors, processing, sampling)
        self.refresh_view()

    def _replace_current_profile(self, profile: HSVColorProfile) -> None:
        assert self.state is not None
        self.state.working_config = replace(
            self.state.working_config,
            colors=tuple(profile if item.name == profile.name else item for item in self.state.working_config.colors),
        )
        self.state.dirty = self.state.working_config != self.state.disk_config

    def recommend_current_color(self) -> None:
        if self.state is None or self.state.image_bgr is None or self.state.roi is None:
            return
        try:
            recommendation = recommend_hsv_profile(self.state.image_bgr, self.state.roi, self.state.working_config, self.state.color_name)
        except (ValueError, TypeError, cv2.error) as exc:
            self.state.config_status = f"推荐失败：{exc}"
            self.status_var.set(self.state.config_status)
            self._set_stats(self.state.config_status)
            return
        self.state.recommendation_backup = self.state.working_config
        current = self.state.current_profile()
        self._recommendation_before = tuple(current.ranges)
        ranges = tuple(recommendation.ranges) + tuple(current.ranges[2:])
        self._replace_current_profile(replace(current, ranges=ranges))
        self._recommendation = recommendation
        self.state.config_status = "已生成推荐值，仅修改工作配置，尚未保存"
        self._load_controls_from_state()
        self.status_var.set(self.state.config_status)
        self.refresh_view()

    def undo_recommendation(self) -> None:
        if self.state is None or self.state.recommendation_backup is None:
            return
        self.state.working_config = self.state.recommendation_backup
        self.state.recommendation_backup = None
        self._recommendation = None
        self._recommendation_before = None
        self.state.dirty = self.state.working_config != self.state.disk_config
        self.state.config_status = "已撤销推荐值"
        self._load_controls_from_state()
        self.status_var.set(self.state.config_status)
        self.refresh_view()

    def merge_current_ranges(self) -> None:
        if self.state is None:
            return
        current = self.state.current_profile()
        merged = merge_duplicate_ranges(current)
        if merged == current:
            return
        self._replace_current_profile(merged)
        self.state.config_status = "已合并重复区间，尚未保存"
        self._load_controls_from_state()
        self.status_var.set(self.state.config_status)
        self.refresh_view()

    def navigate(self, amount: int) -> None:
        if self.state is None:
            return
        index = 0 if amount == 0 else len(self.state.image_paths) - 1 if amount == -2 else self.state.index + amount
        self.state.set_index(index)
        self._prepare_preview()
        self.refresh_view()

    def _prepare_preview(self) -> None:
        if self.state is None or self.state.image_bgr is None:
            self._preview_bgr = None
            self._preview_hsv = None
            self._analysis_bgr = None
            self._analysis_hsv = None
            self._preview_scale = 1.0
            return
        self._preview_bgr, self._preview_scale = scale_preview(self.state.image_bgr, 1400, 900)
        self._preview_hsv = cv2.cvtColor(self._preview_bgr, cv2.COLOR_BGR2HSV)
        if self.analysis_var.get().startswith("原始"):
            self._analysis_bgr = self.state.image_bgr.copy()
            self._analysis_scale_x = self._analysis_scale_y = 1.0
        else:
            self._analysis_bgr, analysis_scale = scale_preview(self.state.image_bgr, 640, 480)
            self._analysis_scale_x = self._analysis_scale_y = analysis_scale
        self._analysis_hsv = cv2.cvtColor(self._analysis_bgr, cv2.COLOR_BGR2HSV)

    def _prepare_preview_and_refresh(self) -> None:
        if self.state is not None:
            self._prepare_preview()
        self.refresh_view()

    def _preview_roi(self) -> Optional[Tuple[int, int, int, int]]:
        if self.state is None or self.state.roi is None:
            return None
        x1, y1, x2, y2 = self.state.roi
        scale = self._preview_scale
        return (int(x1 * scale), int(y1 * scale), int(x2 * scale), int(y2 * scale))

    def _analysis_roi(self) -> Optional[Tuple[int, int, int, int]]:
        if self.state is None or self.state.roi is None:
            return None
        x1, y1, x2, y2 = self.state.roi
        return (
            int(round(x1 * self._analysis_scale_x)),
            int(round(y1 * self._analysis_scale_y)),
            int(round(x2 * self._analysis_scale_x)),
            int(round(y2 * self._analysis_scale_y)),
        )

    def _canvas_to_image(self, x: int, y: int) -> Optional[Tuple[int, int]]:
        if self.state is None or self.state.image_bgr is None or self._original_transform is None:
            return None
        return canvas_point_to_image((x, y), self._original_transform, tile_origin=self._grid_origin)

    def canvas_press(self, event: tk.Event) -> None:
        point = self._canvas_to_image(event.x, event.y)
        if point is not None:
            self._drag_start = point
            self._drag_current = point

    def canvas_drag(self, event: tk.Event) -> None:
        point = self._canvas_to_image(event.x, event.y)
        if point is not None:
            self._drag_current = point
            self.schedule_refresh()

    def canvas_release(self, event: tk.Event) -> None:
        if self.state is None or self.state.image_bgr is None or self._drag_start is None:
            return
        point = self._canvas_to_image(event.x, event.y)
        if point is None:
            point = self._drag_current or self._drag_start
        self.state.point = point
        if self._drag_start != point:
            self.state.roi = normalize_roi(self._drag_start, point, self.state.image_bgr.shape[1], self.state.image_bgr.shape[0])
        else:
            self.state.roi = None
        self._drag_start = None
        self._drag_current = None
        self._update_recommend_controls()
        self.refresh_view()

    def clear_roi(self) -> None:
        if self.state is None:
            return
        self.state.roi = None
        self.state.point = None
        self._drag_start = None
        self._drag_current = None
        self._update_recommend_controls()
        self.refresh_view()

    def schedule_refresh(self) -> None:
        if self._refresh_job is not None:
            self.root.after_cancel(self._refresh_job)
        self._refresh_job = self.root.after(80, self._refresh_now)

    def refresh_view(self) -> None:
        self.schedule_refresh()

    def _refresh_now(self) -> None:
        self._refresh_job = None
        self._resize_job = None
        if self.state is None or self.state.image_bgr is None or self._preview_bgr is None or self._preview_hsv is None or self._analysis_hsv is None:
            return
        image = self._preview_bgr
        hsv = self._analysis_hsv
        profile = self.state.current_profile()
        roi = self._analysis_roi()
        try:
            current_mask = build_color_mask_from_hsv(hsv, profile, self.state.working_config.processing)
            classification = classify_bbox_hsv(self._analysis_bgr, roi, self.state.working_config) if roi or self.full_analysis_var.get() else _EMPTY_CLASSIFICATION
            self._overlap_report = analyze_hsv_range_overlap(hsv, profile, self.state.working_config.processing)
        except (ValueError, TypeError, cv2.error) as exc:
            self._set_stats(f"参数错误：{exc}\n请检查 HSV 最小值和最大值。")
            return
        canvas_w = max(1, int(self.canvas.winfo_width()))
        canvas_h = max(1, int(self.canvas.winfo_height()))
        self._tile_size = max(160, min((canvas_w - GRID_GAP) // 2, (canvas_h - GRID_GAP) // 2))
        sample_mask = None
        if roi is not None:
            region = build_hsv_sample_region(roi, hsv.shape[1], hsv.shape[0], self.state.working_config.sampling)
            if region is not None:
                sample_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
                x1, y1, x2, y2 = region.sample_bbox
                sample_mask[y1:y2, x1:x2] = region.ellipse_mask
        histogram = render_hsv_histogram_tile(hsv, sample_mask, profile, self._tile_size)
        heatmap = render_hs_heatmap_tile(hsv, sample_mask, profile, self._tile_size)
        mask_bgr = cv2.cvtColor(current_mask, cv2.COLOR_GRAY2BGR)
        tile_specs = (
            ("原图 + ROI", self.state.image_bgr),
            (f"当前颜色 Mask: {self.state.color_name}", mask_bgr),
            ("H/S/V 直方图", histogram),
            ("H-S 二维热力图", heatmap),
        )
        grid_w = self._tile_size * 2 + GRID_GAP
        grid_h = self._tile_size * 2 + GRID_GAP
        self._grid_origin = ((canvas_w - grid_w) // 2, (canvas_h - grid_h) // 2)
        self.canvas.delete("all")
        self._photo_refs = []
        self._original_transform = None
        for index, (title, panel) in enumerate(tile_specs):
            tile, transform = compose_square_tile(panel, self._tile_size, title)
            tile_x = self._grid_origin[0] + (index % 2) * (self._tile_size + GRID_GAP)
            tile_y = self._grid_origin[1] + (index // 2) * (self._tile_size + GRID_GAP)
            photo = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(tile, cv2.COLOR_BGR2RGB)))
            self._photo_refs.append(photo)
            self.canvas.create_image(tile_x, tile_y, image=photo, anchor=tk.NW)
            if index == 0:
                self._original_transform = transform
                self._draw_roi_overlay(tile_x, tile_y, transform)
        self.image_status.configure(text=f"{self.state.image_paths[self.state.index].name}  {self.state.index + 1}/{len(self.state.image_paths)}")
        original_h, original_w = self.state.image_bgr.shape[:2]
        analysis_h, analysis_w = self._analysis_bgr.shape[:2]
        roi_text = "未选择 ROI" if self.state.roi is None else str(self.state.roi)
        source_label = "默认配置" if self.state.config_is_default else "外部配置"
        self.status_var.set(f"{source_label} | {self.state.config_path} | {self.state.image_paths[self.state.index].name}  {self.state.index + 1}/{len(self.state.image_paths)} | 原始 {original_w}×{original_h} | 分析 {analysis_w}×{analysis_h} | 颜色 {self.state.color_name} | ROI {roi_text} | {'● 未保存' if self.state.dirty else '已保存'}")
        lines = [
            f"配置版本={self.state.working_config.version}  来源={'默认配置' if self.state.config_is_default else '外部配置'}",
            f"配置路径={self.state.config_path}",
            f"状态={self.state.config_status or ('未保存' if self.state.dirty else '已保存')}",
            f"当前颜色启用={profile.enabled}  区间数量={len(profile.ranges)}",
            f"分类: {classification.color_name or '不确定'}  type={classification.type_id}",
            f"coverage={classification.coverage:.3f}  purity={classification.purity:.3f}  margin={classification.margin:.3f}",
            f"有效像素={classification.valid_pixel_count}  采样像素={classification.sample_pixel_count}",
        ]
        if self._overlap_report is not None and self._overlap_report.warning:
            lines.append(f"警告：{self._overlap_report.warning}")
            lines.append(f"区间交集比例={self._overlap_report.volume_overlap_ratio:.3f}  B新增像素={self._overlap_report.new_pixel_count}")
            self.merge_ranges_button.configure(state=tk.NORMAL)
        elif hasattr(self, "merge_ranges_button"):
            self.merge_ranges_button.configure(state=tk.DISABLED)
        if roi is None:
            lines.append("请在原图中框选一个物料区域")
            lines.append("尚未选择 ROI")
        if self.state.point and self.state.image_hsv is not None:
            px, py = self.state.point
            full_image = self.state.image_bgr
            full_hsv = self.state.image_hsv
            pixel_bgr = full_image[py, px].tolist()
            pixel_hsv = full_hsv[py, px].tolist()
            patch = full_hsv[max(0, py - 2):py + 3, max(0, px - 2):px + 3].reshape(-1, 3)
            lines.append(f"点击像素 ({px}, {py}) BGR={pixel_bgr} HSV={pixel_hsv}")
            lines.append(f"5×5 P05/P50/P95={np.percentile(patch, (5, 50, 95), axis=0).astype(int).tolist()}")
        if self._recommendation is not None:
            recommendation = self._recommendation
            before_text = self._format_ranges(self._recommendation_before or ())
            after_text = self._format_ranges(self.state.current_profile().ranges[:2])
            lines.extend(
                (
                    f"推荐前：{before_text}",
                    f"推荐后：{after_text}",
                    f"推荐覆盖率={recommendation.foreground_coverage:.3f}",
                    f"背景泄漏率={recommendation.background_leakage:.3f}",
                    f"颜色冲突率={recommendation.conflict_ratio:.3f}",
                    f"推荐置信度={recommendation.confidence:.3f}",
                    "推荐警告：" + ("；".join(recommendation.warnings) if recommendation.warnings else "无"),
                )
            )
        self._set_stats("\n".join(lines))

    @staticmethod
    def _format_ranges(ranges: Sequence[HSVRange]) -> str:
        if not ranges:
            return "无"
        labels = "AB"
        return "  ".join(
            f"{labels[index] if index < len(labels) else str(index + 1)} H{item.lower[0]}-{item.upper[0]} S{item.lower[1]}-{item.upper[1]} V{item.lower[2]}-{item.upper[2]}"
            for index, item in enumerate(ranges[:2])
        )

    def _set_stats(self, text: str) -> None:
        self.stats.configure(state=tk.NORMAL)
        self.stats.delete("1.0", tk.END)
        self.stats.insert("1.0", text)
        self.stats.configure(state=tk.DISABLED)

    def _draw_roi_overlay(self, tile_x: int, tile_y: int, transform: TileTransform) -> None:
        if self.state is None or self.state.image_bgr is None:
            return
        if self.state.roi is not None:
            x1, y1, x2, y2 = self.state.roi
            p1 = image_point_to_canvas((x1, y1), transform, tile_origin=(tile_x, tile_y))
            p2 = image_point_to_canvas((max(x1, x2 - 1), max(y1, y2 - 1)), transform, tile_origin=(tile_x, tile_y))
            if p1 and p2:
                self.canvas.create_rectangle(*p1, *p2, outline="#ffd400", width=2)
            sample = build_hsv_sample_region(self.state.roi, self.state.image_bgr.shape[1], self.state.image_bgr.shape[0], self.state.working_config.sampling)
            if sample is not None:
                sx1, sy1, sx2, sy2 = sample.sample_bbox
                sp1 = image_point_to_canvas((sx1, sy1), transform, tile_origin=(tile_x, tile_y))
                sp2 = image_point_to_canvas((sx2 - 1, sy2 - 1), transform, tile_origin=(tile_x, tile_y))
                if sp1 and sp2:
                    self.canvas.create_rectangle(*sp1, *sp2, outline="#00e5ff", dash=(5, 3), width=2)
                center = ((sx1 + sx2 - 1) / 2, (sy1 + sy2 - 1) / 2)
                center_canvas = image_point_to_canvas(center, transform, tile_origin=(tile_x, tile_y))
                if center_canvas:
                    radius_x = max(2, int((sx2 - sx1) * transform.scale / 2))
                    radius_y = max(2, int((sy2 - sy1) * transform.scale / 2))
                    self.canvas.create_oval(center_canvas[0] - radius_x, center_canvas[1] - radius_y, center_canvas[0] + radius_x, center_canvas[1] + radius_y, outline="#00e5ff", width=2)

    def _on_key(self, event: tk.Event) -> Optional[str]:
        editing_widget = isinstance(event.widget, (tk.Entry, ttk.Entry, ttk.Spinbox))
        if editing_widget and not (event.state & 0x4 and str(event.keysym).lower() in ("s", "r")):
            return None
        key = str(event.keysym)
        if event.state & 0x4 and key.lower() == "s":
            self.save_config()
            return "break"
        if event.state & 0x4 and key.lower() == "r":
            self.reload_config()
            return "break"
        if key in ("Left",) or key.lower() == "p":
            self.navigate(-1)
        elif key in ("Right",) or key.lower() == "n":
            self.navigate(1)
        elif key == "Home":
            self.navigate(0)
        elif key == "End":
            self.navigate(-2)
        elif key in ("Escape", "c", "C", "Delete"):
            self.clear_roi()
        elif key.isdigit() and 1 <= int(key) <= len(COLOR_ORDER):
            name = COLOR_ORDER[int(key) - 1]
            if self.state is not None and any(item.name == name for item in self.state.working_config.colors):
                self.color_var.set(name)
                self.change_color()
        elif key.lower() == "f":
            self.full_analysis_var.set(not self.full_analysis_var.get())
            self.refresh_view()
        return None

    def _save_current_config(self, show_message: bool = True) -> bool:
        if self.state is None:
            return False
        try:
            save_hsv_config(self.state.working_config, self.state.config_path)
        except (OSError, ValueError) as exc:
            self.state.config_status = f"保存失败：{exc}"
            self.status_var.set(self.state.config_status)
            if show_message:
                messagebox.showerror("保存失败", str(exc), parent=self.root)
            return False
        self.state.disk_config = self.state.working_config
        self.state.dirty = False
        self.state.config_status = f"已保存：{self.state.config_path}"
        self.status_var.set(self.state.config_status)
        if show_message:
            messagebox.showinfo("保存成功", f"已保存配置：\n{self.state.config_path}", parent=self.root)
        return True

    def save_config(self) -> None:
        self._save_current_config(True)

    def save_as_config(self) -> None:
        if self.state is None:
            return
        selected = filedialog.asksaveasfilename(
            title="另存为 HSV 配置",
            defaultextension=".json",
            initialfile=self.state.config_path.name,
            filetypes=[("JSON", "*.json"), ("所有文件", "*.*")],
        )
        if not selected:
            return
        target = Path(selected).expanduser().resolve()
        try:
            save_hsv_config(self.state.working_config, target)
        except (OSError, ValueError) as exc:
            messagebox.showerror("另存为失败", str(exc), parent=self.root)
            return
        self.state.config_path = target
        self.state.disk_config = self.state.working_config
        self.state.dirty = False
        self.state.config_is_default = target == resolve_default_config_path().resolve()
        self.state.config_status = f"已另存为：{target}"
        self.status_var.set(self.state.config_status)
        self.refresh_view()

    def _dirty_transition(self, operation: str) -> bool:
        if self.state is None or not self.state.dirty:
            return True
        dialog = tk.Toplevel(self.root)
        dialog.title("未保存修改")
        dialog.transient(self.root)
        dialog.grab_set()
        ttk.Label(dialog, text=f"当前参数尚未保存，继续{operation}将丢失修改。", padding=16).pack(fill=tk.X)
        result = {"value": "cancel"}
        actions = ttk.Frame(dialog, padding=(16, 0, 16, 16))
        actions.pack(fill=tk.X)
        def choose(value: str) -> None:
            result["value"] = value
            dialog.destroy()
        ttk.Button(actions, text="保存并继续", command=lambda: choose("save")).pack(side=tk.LEFT, padx=3)
        ttk.Button(actions, text="放弃修改", command=lambda: choose("discard")).pack(side=tk.LEFT, padx=3)
        ttk.Button(actions, text="取消", command=lambda: choose("cancel")).pack(side=tk.RIGHT, padx=3)
        dialog.protocol("WM_DELETE_WINDOW", lambda: choose("cancel"))
        self.root.wait_window(dialog)
        if result["value"] == "save":
            return self._save_current_config(False)
        return result["value"] == "discard"

    def _apply_imported_config(self, imported: ImportedConfig) -> None:
        assert self.state is not None
        self.state.config_path = imported.config_path
        self.state.config_is_default = imported.is_default
        self.state.disk_config = imported.config
        self.state.working_config = imported.config
        self.state.startup_config = imported.config
        self.state.color_name = imported.selected_color
        self.state.recommendation_backup = None
        self._recommendation = None
        self._recommendation_before = None
        self._overlap_report = None
        self.state.dirty = False
        self.state.config_status = f"已导入{'默认' if imported.is_default else '外部'}参数：{imported.config_path}"
        self._load_controls_from_state()
        self.status_var.set(self.state.config_status)
        self.refresh_view()

    def import_default_config(self) -> None:
        if self.state is None or not self._dirty_transition("导入默认参数"):
            return
        try:
            imported = import_hsv_config(resolve_default_config_path(), self.state.color_name)
        except (OSError, ValueError) as exc:
            messagebox.showerror("导入默认参数失败", str(exc), parent=self.root)
            return
        self._apply_imported_config(imported)

    def import_external_config(self) -> None:
        if self.state is None:
            return
        selected = filedialog.askopenfilename(title="导入 HSV 配置", filetypes=[("JSON", "*.json"), ("所有文件", "*.*")])
        if not selected or not self._dirty_transition("导入配置文件"):
            return
        try:
            imported = import_hsv_config(selected, self.state.color_name)
        except (OSError, ValueError) as exc:
            messagebox.showerror("导入配置失败", str(exc), parent=self.root)
            return
        self._apply_imported_config(imported)

    def reload_config(self) -> None:
        if self.state is None or not self._dirty_transition("重载当前配置"):
            return
        try:
            imported = import_hsv_config(self.state.config_path, self.state.color_name)
        except (OSError, ValueError) as exc:
            messagebox.showerror("重载失败", str(exc), parent=self.root)
            return
        self._apply_imported_config(imported)

    def restore_current(self) -> None:
        if self.state is None:
            return
        self.state.working_config = self.state.working_config.__class__(self.state.startup_config.version, tuple(item if item.name != self.state.color_name else self.state.startup_config.colors[index] for index, item in enumerate(self.state.working_config.colors)), self.state.working_config.processing, self.state.working_config.sampling)
        self.state.dirty = self.state.working_config != self.state.disk_config
        self._load_controls_from_state()
        self.refresh_view()

    def restore_all(self) -> None:
        if self.state is None:
            return
        self.state.working_config = self.state.startup_config
        self.state.dirty = self.state.working_config != self.state.disk_config
        self._load_controls_from_state()
        self.refresh_view()

    def export(self, kind: str) -> None:
        if self.state is None or self.state.image_bgr is None or self.state.image_hsv is None:
            return
        if self.output_dir is None:
            selected = filedialog.askdirectory(title="选择导出目录")
            if not selected:
                return
            self.output_dir = Path(selected)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        profile = self.state.current_profile()
        mask = build_color_mask_from_hsv(self.state.image_hsv, profile, self.state.working_config.processing)
        if kind == "mask":
            output = self.output_dir / output_filename(self.state.image_paths[self.state.index], "mask", self.state.color_name)
            if not imwrite_unicode(output, mask):
                messagebox.showerror("导出失败", f"无法写入：\n{output}", parent=self.root)
                return
        else:
            output = self.output_dir / output_filename(self.state.image_paths[self.state.index], "preview")
            composite, _ = build_composite_preview(self.state.image_bgr, {item.name: build_color_mask_from_hsv(self.state.image_hsv, item, self.state.working_config.processing) for item in self.state.working_config.colors if item.enabled}, self.state.working_config.colors)
            masked = cv2.bitwise_and(self.state.image_bgr, self.state.image_bgr, mask=mask)
            panels = (self.state.image_bgr, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), masked, composite)
            tile_size = max(160, int(self._tile_size))
            gap = GRID_GAP
            grid = np.full((tile_size * 2 + gap, tile_size * 2 + gap, 3), 24, dtype=np.uint8)
            for index, panel in enumerate(panels):
                tile, _ = compose_square_tile(panel, tile_size, ("原图", "Mask", "Mask 后", "启用颜色合成")[index])
                y = (index // 2) * (tile_size + gap)
                x = (index % 2) * (tile_size + gap)
                grid[y:y + tile_size, x:x + tile_size] = tile
            if not imwrite_unicode(output, grid):
                messagebox.showerror("导出失败", f"无法写入：\n{output}", parent=self.root)
                return
        messagebox.showinfo("导出成功", f"已导出：\n{output}", parent=self.root)

    def close(self) -> None:
        if self._refresh_job is not None:
            self.root.after_cancel(self._refresh_job)
        if self.state is not None and not self._dirty_transition("退出"):
            return
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    HSVTunerGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
