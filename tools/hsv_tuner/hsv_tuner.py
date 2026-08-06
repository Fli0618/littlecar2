"""独立离线 HSV 阈值调参工具。

本脚本只依赖 OpenCV、NumPy、标准库和 ``vision.hsv_color``，不会加载模型、
访问相机/串口或导入 YOLO 模块。
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple, Union

import cv2
import numpy as np

from vision.hsv_color import (
    HSVClassification,
    HSVColorProfile,
    HSVConfig,
    HSVProcessingConfig,
    HSVRange,
    HSVSamplingConfig,
    build_color_mask_from_hsv,
    classify_bbox_hsv,
    load_hsv_config,
    save_hsv_config,
)


DEFAULT_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")
COLOR_ORDER = ("red", "yellow", "blue", "green", "black", "light_blue")
PREVIEW_COLORS = {
    "red": (0, 0, 255),
    "yellow": (0, 255, 255),
    "blue": (255, 0, 0),
    "green": (0, 255, 0),
    "black": (32, 32, 32),
    "light_blue": (255, 255, 0),
}
CONFLICT_COLOR = (255, 0, 255)
WINDOW_NAME = "HSV Tuner"
CONTROL_WINDOW = "HSV Controls"
KEY_RIGHT = 83
KEY_LEFT = 81
def resolve_default_config_path() -> Path:
    """根据工具文件位置解析项目默认 HSV 配置，不依赖当前工作目录。"""
    repository_root = Path(__file__).resolve().parents[2]
    return repository_root / "jetson" / "assets" / "config" / "hsv_colors.json"


DEFAULT_CONFIG_PATH = resolve_default_config_path()


@dataclass(frozen=True)
class ImportedConfig:
    config: HSVConfig
    config_path: Path
    selected_color: str
    is_default: bool


def import_hsv_config(path: Union[str, Path], current_color: str = "red") -> ImportedConfig:
    """读取并校验配置，返回 GUI 可直接应用的导入结果。"""
    config_path = Path(path).expanduser().resolve()
    config = load_hsv_config(config_path)
    names = tuple(profile.name for profile in config.colors)
    selected_color = current_color if current_color in names else names[0]
    default_path = resolve_default_config_path().resolve()
    return ImportedConfig(config, config_path, selected_color, config_path == default_path)


def natural_sort_key(value: Union[str, Path]) -> List[Union[int, str]]:
    """将数字片段按数值排序，例如 image2 排在 image10 前。"""
    text = str(value).lower()
    return [int(part) if part.isdigit() else part for part in re.split(r"(\d+)", text)]


def normalize_extensions(extensions: Iterable[str]) -> Tuple[str, ...]:
    if isinstance(extensions, str):
        extensions = extensions.split(",")
    result = []
    for extension in extensions:
        normalized = str(extension).strip().lower()
        if not normalized:
            continue
        if not normalized.startswith("."):
            normalized = "." + normalized
        result.append(normalized)
    return tuple(dict.fromkeys(result)) or DEFAULT_EXTENSIONS


def discover_image_paths(directory: Union[str, Path], recursive: bool = False, extensions: Iterable[str] = DEFAULT_EXTENSIONS) -> List[Path]:
    """发现目录中的图片并按自然顺序返回。"""
    root = Path(directory)
    if not root.is_dir():
        raise ValueError(f"image directory does not exist or is not a directory: {root}")
    allowed = set(normalize_extensions(extensions))
    iterator = root.rglob("*") if recursive else root.iterdir()
    return sorted((item for item in iterator if item.is_file() and item.suffix.lower() in allowed), key=natural_sort_key)


def clamp_index(index: int, count: int) -> int:
    """将图片索引限制到有效范围；空列表返回 0。"""
    if count <= 0:
        return 0
    return max(0, min(int(index), count - 1))


def normalize_roi(start: Sequence[int], end: Sequence[int], width: int, height: int) -> Optional[Tuple[int, int, int, int]]:
    """规范化任意方向拖拽的 ROI，返回半开区间 ``x1,y1,x2,y2``。"""
    if len(start) != 2 or len(end) != 2 or width <= 0 or height <= 0:
        return None
    try:
        x1, x2 = sorted((max(0, min(width - 1, int(start[0]))), max(0, min(width - 1, int(end[0])))))
        y1, y2 = sorted((max(0, min(height - 1, int(start[1]))), max(0, min(height - 1, int(end[1])))))
    except (TypeError, ValueError):
        return None
    x2 += 1
    y2 += 1
    if x2 - x1 < 3 or y2 - y1 < 3:
        return None
    return x1, y1, x2, y2


def scale_preview(image: np.ndarray, max_width: int = 640, max_height: int = 360) -> Tuple[np.ndarray, float]:
    """将图像缩放到最大尺寸，返回缩放图和缩放因子。"""
    if not isinstance(image, np.ndarray) or image.size == 0 or max_width <= 0 or max_height <= 0:
        raise ValueError("image must be a non-empty numpy array")
    height, width = image.shape[:2]
    scale = min(1.0, float(max_width) / width, float(max_height) / height)
    if scale == 1.0:
        return image.copy(), scale
    resized = cv2.resize(image, (max(1, int(round(width * scale))), max(1, int(round(height * scale)))), interpolation=cv2.INTER_AREA)
    return resized, scale


def output_filename(image_path: Union[str, Path], kind: str, color_name: Optional[str] = None) -> str:
    """生成稳定的导出文件名。"""
    stem = Path(image_path).stem
    suffix = f"_{color_name}" if color_name else ""
    return f"{stem}{suffix}_{kind}.png"


def read_image_file(image_path: Union[str, Path]) -> Optional[np.ndarray]:
    """使用 Unicode 安全的方式读取图片，兼容 Windows 中文路径。"""
    try:
        encoded = np.fromfile(str(image_path), dtype=np.uint8)
    except OSError:
        return None
    if encoded.size == 0:
        return None
    return cv2.imdecode(encoded, cv2.IMREAD_COLOR)


def build_composite_preview(frame_bgr: np.ndarray, masks: Dict[str, np.ndarray], profiles: Sequence[HSVColorProfile]) -> Tuple[np.ndarray, int]:
    """绘制启用颜色合成图；重叠像素显示洋红色并返回冲突数量。"""
    if not isinstance(frame_bgr, np.ndarray) or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
        raise TypeError("frame_bgr must be a BGR image")
    result = np.zeros_like(frame_bgr)
    hit_count = np.zeros(frame_bgr.shape[:2], dtype=np.uint16)
    for profile in profiles:
        if not profile.enabled or profile.name not in masks:
            continue
        mask = masks[profile.name]
        hit = mask > 0
        hit_count[hit] += 1
        result[hit] = PREVIEW_COLORS.get(profile.name, (255, 255, 255))
    conflicts = hit_count > 1
    result[conflicts] = CONFLICT_COLOR
    return result, int(np.count_nonzero(conflicts))


def _config_with_profile(config: HSVConfig, profile: HSVColorProfile) -> HSVConfig:
    return replace(config, colors=tuple(profile if item.name == profile.name else item for item in config.colors))


def _profile(config: HSVConfig, name: str) -> HSVColorProfile:
    for profile in config.colors:
        if profile.name == name:
            return profile
    raise KeyError(name)


def _clamp_triplet(values: Sequence[int], maximum: Sequence[int]) -> Tuple[int, int, int]:
    return tuple(max(0, min(int(value), int(limit))) for value, limit in zip(values, maximum))  # type: ignore[return-value]


def _safe_range(lower: Sequence[int], upper: Sequence[int]) -> HSVRange:
    low = _clamp_triplet(lower, (179, 255, 255))
    high = _clamp_triplet(upper, (179, 255, 255))
    high = tuple(max(a, b) for a, b in zip(low, high))  # type: ignore[assignment]
    return HSVRange(low, high)


@dataclass
class TunerState:
    image_paths: List[Path]
    config_path: Path
    disk_config: HSVConfig
    working_config: HSVConfig
    startup_config: HSVConfig
    index: int = 0
    color_name: str = "red"
    show_all: bool = True
    roi: Optional[Tuple[int, int, int, int]] = None
    point: Optional[Tuple[int, int]] = None
    image_bgr: Optional[np.ndarray] = None
    image_hsv: Optional[np.ndarray] = None
    dragging: bool = False
    drag_start: Optional[Tuple[int, int]] = None
    drag_current: Optional[Tuple[int, int]] = None
    trackbar_refreshing: bool = False
    dirty: bool = False
    config_is_default: bool = False
    config_status: str = ""
    recommendation_backup: Optional[HSVConfig] = None

    def current_profile(self) -> HSVColorProfile:
        return _profile(self.working_config, self.color_name)

    def load_current_image(self) -> bool:
        if not self.image_paths:
            return False
        start_index = clamp_index(self.index, len(self.image_paths))
        for offset in range(len(self.image_paths)):
            candidate_index = (start_index + offset) % len(self.image_paths)
            candidate = self.image_paths[candidate_index]
            image = read_image_file(candidate)
            if image is None:
                print(f"warning: unable to read image, skipped: {candidate}")
                continue
            self.index = candidate_index
            self.image_bgr = image
            self.image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            self.roi = None
            self.point = None
            return True
        self.image_bgr = None
        self.image_hsv = None
        return False

    def set_index(self, index: int) -> None:
        self.index = clamp_index(index, len(self.image_paths))
        self.load_current_image()

    def update_current_profile(self, ranges: Tuple[HSVRange, ...]) -> None:
        self.working_config = _config_with_profile(self.working_config, replace(self.current_profile(), ranges=ranges))
        self.dirty = True


class HSVTuner:
    """OpenCV 窗口状态和交互控制器。"""

    def __init__(self, state: TunerState, output_dir: Optional[Path] = None) -> None:
        self.state = state
        self.output_dir = output_dir
        self.tile_size = (640, 360)
        self.display_scale = 1.0
        self._last_render: Optional[np.ndarray] = None

    def _trackbar_value(self, name: str, default: int = 0) -> int:
        try:
            return int(cv2.getTrackbarPos(name, CONTROL_WINDOW))
        except cv2.error:
            return default

    def _create_trackbars(self) -> None:
        cv2.namedWindow(CONTROL_WINDOW, cv2.WINDOW_NORMAL)
        specs = [
            ("H1 Min", 0, 179), ("H1 Max", 179, 179), ("S1 Min", 0, 255), ("S1 Max", 255, 255), ("V1 Min", 0, 255), ("V1 Max", 255, 255),
            ("Range2 Enabled", 0, 1), ("H2 Min", 0, 179), ("H2 Max", 179, 179), ("S2 Min", 0, 255), ("S2 Max", 255, 255), ("V2 Min", 0, 255), ("V2 Max", 255, 255),
            ("Blur Kernel", 3, 9), ("Open Kernel", 3, 9), ("Open Iterations", 1, 5), ("Close Kernel", 3, 9), ("Close Iterations", 1, 5),
            ("Sample Scale X", 55, 100), ("Sample Scale Y", 55, 100), ("Min Coverage", 25, 100), ("Min Margin", 10, 100), ("Min Pixels", 100, 50000),
        ]
        for name, value, maximum in specs:
            cv2.createTrackbar(name, CONTROL_WINDOW, value, maximum, self._on_trackbar)
        self.refresh_trackbars()

    @staticmethod
    def _odd_slider(value: int) -> int:
        value = max(0, min(9, int(value)))
        return 0 if value == 0 else value if value % 2 else value - 1

    def refresh_trackbars(self) -> None:
        profile = self.state.current_profile()
        ranges = list(profile.ranges)
        first = ranges[0]
        second = ranges[1] if len(ranges) > 1 else HSVRange((0, 0, 0), (0, 0, 0))
        values = {
            "H1 Min": first.lower[0], "H1 Max": first.upper[0], "S1 Min": first.lower[1], "S1 Max": first.upper[1], "V1 Min": first.lower[2], "V1 Max": first.upper[2],
            "Range2 Enabled": int(len(ranges) > 1), "H2 Min": second.lower[0], "H2 Max": second.upper[0], "S2 Min": second.lower[1], "S2 Max": second.upper[1], "V2 Min": second.lower[2], "V2 Max": second.upper[2],
            "Blur Kernel": self.state.working_config.processing.blur_kernel, "Open Kernel": self.state.working_config.processing.open_kernel, "Open Iterations": self.state.working_config.processing.open_iterations, "Close Kernel": self.state.working_config.processing.close_kernel, "Close Iterations": self.state.working_config.processing.close_iterations,
            "Sample Scale X": int(round(self.state.working_config.sampling.scale_x * 100)), "Sample Scale Y": int(round(self.state.working_config.sampling.scale_y * 100)), "Min Coverage": int(round(self.state.working_config.sampling.min_coverage * 100)), "Min Margin": int(round(self.state.working_config.sampling.min_margin * 100)), "Min Pixels": self.state.working_config.sampling.min_pixels,
        }
        self.state.trackbar_refreshing = True
        try:
            for name, value in values.items():
                cv2.setTrackbarPos(name, CONTROL_WINDOW, int(value))
        finally:
            self.state.trackbar_refreshing = False

    def _on_trackbar(self, _value: int) -> None:
        if self.state.trackbar_refreshing:
            return
        first = _safe_range(
            (self._trackbar_value("H1 Min"), self._trackbar_value("S1 Min"), self._trackbar_value("V1 Min")),
            (self._trackbar_value("H1 Max"), self._trackbar_value("S1 Max"), self._trackbar_value("V1 Max")),
        )
        second = _safe_range(
            (self._trackbar_value("H2 Min"), self._trackbar_value("S2 Min"), self._trackbar_value("V2 Min")),
            (self._trackbar_value("H2 Max"), self._trackbar_value("S2 Max"), self._trackbar_value("V2 Max")),
        )
        old_ranges = list(self.state.current_profile().ranges)
        extras = old_ranges[2:]
        ranges = [first]
        if self._trackbar_value("Range2 Enabled"):
            ranges.append(second)
        ranges.extend(extras)
        processing = replace(
            self.state.working_config.processing,
            blur_kernel=self._odd_slider(self._trackbar_value("Blur Kernel")),
            open_kernel=self._odd_slider(self._trackbar_value("Open Kernel")),
            open_iterations=self._trackbar_value("Open Iterations"),
            close_kernel=self._odd_slider(self._trackbar_value("Close Kernel")),
            close_iterations=self._trackbar_value("Close Iterations"),
        )
        sampling = replace(
            self.state.working_config.sampling,
            scale_x=max(0.01, self._trackbar_value("Sample Scale X") / 100.0),
            scale_y=max(0.01, self._trackbar_value("Sample Scale Y") / 100.0),
            min_pixels=max(0, self._trackbar_value("Min Pixels")),
            min_coverage=self._trackbar_value("Min Coverage") / 100.0,
            min_margin=self._trackbar_value("Min Margin") / 100.0,
        )
        self.state.working_config = replace(_config_with_profile(self.state.working_config, replace(self.state.current_profile(), ranges=tuple(ranges))), processing=processing, sampling=sampling)
        self.state.dirty = True

    def _frame_point(self, x: int, y: int) -> Optional[Tuple[int, int]]:
        tile_width, tile_height = self.tile_size
        if x >= tile_width or y >= tile_height or self.state.image_bgr is None:
            return None
        height, width = self.state.image_bgr.shape[:2]
        return max(0, min(width - 1, int(x / self.display_scale))), max(0, min(height - 1, int(y / self.display_scale)))

    def _mouse_callback(self, event: int, x: int, y: int, flags: int, _param: object) -> None:
        point = self._frame_point(x, y)
        if event == cv2.EVENT_RBUTTONDOWN:
            self.state.roi = None
            self.state.drag_start = None
            self.state.drag_current = None
            return
        if point is None:
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            self.state.dragging = True
            self.state.drag_start = point
            self.state.drag_current = point
        elif event == cv2.EVENT_MOUSEMOVE and self.state.dragging:
            self.state.drag_current = point
        elif event == cv2.EVENT_LBUTTONUP and self.state.dragging:
            self.state.dragging = False
            self.state.drag_current = point
            if self.state.drag_start is not None:
                self.state.roi = normalize_roi(self.state.drag_start, point, self.state.image_bgr.shape[1], self.state.image_bgr.shape[0]) if self.state.image_bgr is not None else None
            self.state.point = point

    def _classification_text(self, classification: HSVClassification) -> List[str]:
        lines = [
            f"mask pixels: {classification.valid_pixel_count}",
            f"coverage={classification.coverage:.3f} purity={classification.purity:.3f} margin={classification.margin:.3f}",
            f"winner: {classification.color_name or 'uncertain'} type={classification.type_id}",
            f"threshold: {'PASS' if classification.type_id is not None else 'NO'}",
        ]
        lines.extend(f"{name}: {count}" for name, count in classification.counts.items())
        return lines

    def render(self) -> np.ndarray:
        if self.state.image_bgr is None or self.state.image_hsv is None:
            return np.zeros((720, 1280, 3), dtype=np.uint8)
        image = self.state.image_bgr
        hsv_image = self.state.image_hsv
        profile = self.state.current_profile()
        current_mask = build_color_mask_from_hsv(hsv_image, profile, self.state.working_config.processing)
        masked = cv2.bitwise_and(image, image, mask=current_mask)
        masks: Dict[str, np.ndarray] = {}
        for item in self.state.working_config.colors:
            if item.enabled:
                masks[item.name] = build_color_mask_from_hsv(hsv_image, item, self.state.working_config.processing)
        composite, conflicts = build_composite_preview(image, masks, self.state.working_config.colors)
        if not self.state.show_all:
            composite = np.zeros_like(image)
            composite[current_mask > 0] = PREVIEW_COLORS.get(self.state.color_name, (255, 255, 255))
            conflicts = 0
        if self.state.roi is not None:
            x1, y1, x2, y2 = self.state.roi
            classification = classify_bbox_hsv(image, self.state.roi, self.state.working_config)
        else:
            classification = HSVClassification(None, None, 0.0, 0.0, 0.0, 0.0, 0, 0, {}, (0, 0, 0, 0))
        original, self.display_scale = scale_preview(image, self.tile_size[0], self.tile_size[1])
        mask_tile = cv2.resize(cv2.cvtColor(current_mask, cv2.COLOR_GRAY2BGR), (original.shape[1], original.shape[0]), interpolation=cv2.INTER_NEAREST)
        masked_tile = cv2.resize(masked, (original.shape[1], original.shape[0]), interpolation=cv2.INTER_AREA)
        composite_tile = cv2.resize(composite, (original.shape[1], original.shape[0]), interpolation=cv2.INTER_NEAREST)
        canvas = np.zeros((self.tile_size[1] * 2, self.tile_size[0] * 2, 3), dtype=np.uint8)
        tile_h, tile_w = original.shape[:2]
        canvas[:tile_h, :tile_w] = original
        canvas[:tile_h, self.tile_size[0]:self.tile_size[0] + tile_w] = mask_tile
        canvas[self.tile_size[1]:self.tile_size[1] + tile_h, :tile_w] = masked_tile
        canvas[self.tile_size[1]:self.tile_size[1] + tile_h, self.tile_size[0]:self.tile_size[0] + tile_w] = composite_tile
        if self.state.roi is not None:
            x1, y1, x2, y2 = self.state.roi
            cv2.rectangle(canvas, (int(x1 * self.display_scale), int(y1 * self.display_scale)), (int((x2 - 1) * self.display_scale), int((y2 - 1) * self.display_scale)), (255, 255, 255), 2)
        if self.state.dragging and self.state.drag_start and self.state.drag_current:
            sx, sy = self.state.drag_start
            ex, ey = self.state.drag_current
            cv2.rectangle(canvas, (int(sx * self.display_scale), int(sy * self.display_scale)), (int(ex * self.display_scale), int(ey * self.display_scale)), (0, 255, 255), 1)
        height, width = image.shape[:2]
        cv2.putText(canvas, f"{self.state.image_paths[self.state.index].name}  {self.state.index + 1}/{len(self.state.image_paths)}  {width}x{height}", (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(canvas, f"color: {self.state.color_name}  ROI: {self.state.roi}", (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.putText(canvas, f"Mask: {self.state.color_name}", (self.tile_size[0] + 12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(canvas, f"Masked preview: {self.state.color_name}", (12, self.tile_size[1] + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(canvas, f"{'All enabled colors' if self.state.show_all else 'Current color preview'}  conflicts={conflicts}", (self.tile_size[0] + 12, self.tile_size[1] + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        stats_x, stats_y = self.tile_size[0] + 12, self.tile_size[1] + 56
        for line in self._classification_text(classification):
            cv2.putText(canvas, line, (stats_x, stats_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            stats_y += 22
        if self.state.point is not None:
            px, py = self.state.point
            pixel_bgr = image[py, px].tolist()
            pixel_hsv = hsv_image[py, px].tolist()
            patch = hsv_image[max(0, py - 2):py + 3, max(0, px - 2):px + 3].reshape(-1, 3)
            quantiles = np.percentile(patch, (10, 50, 90), axis=0).astype(int).tolist()
            cv2.putText(canvas, f"point BGR={pixel_bgr} HSV={pixel_hsv} q10/50/90={quantiles}", (12, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        self._last_render = canvas
        return canvas

    def _save_outputs(self, kind: str) -> None:
        if self.output_dir is None or self.state.image_bgr is None or self._last_render is None:
            print("output directory is not configured")
            return
        self.output_dir.mkdir(parents=True, exist_ok=True)
        image_path = self.state.image_paths[self.state.index]
        profile = self.state.current_profile()
        mask = build_color_mask_from_hsv(self.state.image_hsv, profile, self.state.working_config.processing) if self.state.image_hsv is not None else None
        if kind == "mask" and mask is not None:
            path = self.output_dir / output_filename(image_path, "mask", self.state.color_name)
            cv2.imwrite(str(path), mask)
        elif kind == "preview":
            path = self.output_dir / output_filename(image_path, "preview")
            cv2.imwrite(str(path), self._last_render)
        else:
            return
        print(f"saved HSV output: {path}")

    def _restore_current(self) -> None:
        self.state.working_config = _config_with_profile(self.state.working_config, _profile(self.state.startup_config, self.state.color_name))
        self.state.dirty = self.state.working_config != self.state.disk_config
        self.refresh_trackbars()

    def _restore_all(self) -> None:
        self.state.working_config = self.state.startup_config
        self.state.dirty = self.state.working_config != self.state.disk_config
        self.refresh_trackbars()

    def _reload(self) -> None:
        loaded = load_hsv_config(self.state.config_path)
        self.state.disk_config = loaded
        self.state.working_config = loaded
        self.state.dirty = False
        self.refresh_trackbars()
        print(f"loaded HSV config: {self.state.config_path}")

    def _save(self) -> None:
        save_hsv_config(self.state.working_config, self.state.config_path)
        self.state.disk_config = self.state.working_config
        self.state.dirty = False
        print(f"saved HSV config: {self.state.config_path}")

    def run(self) -> int:
        if not self.state.load_current_image():
            return 2
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WINDOW_NAME, self._mouse_callback)
        self._create_trackbars()
        while True:
            cv2.imshow(WINDOW_NAME, self.render())
            key = cv2.waitKey(30)
            if key < 0:
                continue
            if key in (27, ord("q"), ord("Q")):
                break
            if key in (ord("n"), ord("N"), KEY_RIGHT):
                self.state.set_index(self.state.index + 1)
            elif key in (ord("p"), ord("P"), KEY_LEFT):
                self.state.set_index(self.state.index - 1)
            elif key in (36, 71):  # Home
                self.state.set_index(0)
            elif key in (35, 79):  # End
                self.state.set_index(len(self.state.image_paths) - 1)
            elif ord("1") <= key <= ord("6"):
                candidate = COLOR_ORDER[key - ord("1")]
                if any(item.name == candidate for item in self.state.working_config.colors):
                    self.state.color_name = candidate
                    self.refresh_trackbars()
            elif key in (ord("s"), ord("S")):
                self._save()
            elif key in (ord("l"), ord("L")):
                try:
                    self._reload()
                except ValueError as exc:
                    print(f"load HSV config failed: {exc}")
            elif key == ord("r"):
                self._restore_current()
            elif key == ord("R"):
                self._restore_all()
            elif key in (ord("c"), ord("C"), 255):
                self.state.roi = None
            elif key in (ord("m"), ord("M")):
                self._save_outputs("mask")
            elif key in (ord("o"), ord("O")):
                self._save_outputs("preview")
            elif key == ord(" "):
                self.state.show_all = not self.state.show_all
        cv2.destroyAllWindows()
        if self.state.dirty:
            print("warning: HSV configuration has unsaved changes")
        return 0
