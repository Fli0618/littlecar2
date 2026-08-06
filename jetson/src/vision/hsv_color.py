"""可复用的 HSV 配置、Mask 构建和颜色分类能力。

旧版 Hough HSV 检测接口保留在本模块底部，供比赛服务兼容使用；新的
配置化接口不依赖 Hough、YOLO 或任何模型。
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple, Union

import cv2
import numpy as np


SUPPORTED_CONFIG_VERSION = 1
RED = 0
YELLOW = 1
BLUE = 2
GREEN = 3


def _is_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _validate_triplet(value: object, path: str, upper: Tuple[int, int, int]) -> Tuple[int, int, int]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{path} must contain exactly 3 integers")
    result: List[int] = []
    for index, item in enumerate(value):
        if not _is_int(item):
            raise ValueError(f"{path}[{index}] must be an integer")
        if item < 0 or item > upper[index]:
            raise ValueError(f"{path}[{index}] must be in [0, {upper[index]}]")
        result.append(int(item))
    return tuple(result)  # type: ignore[return-value]


def _validate_kernel(value: object, path: str) -> int:
    if not _is_int(value) or int(value) < 0 or (int(value) > 0 and int(value) % 2 == 0):
        raise ValueError(f"{path} must be 0 or a positive odd integer")
    return int(value)


def _validate_processing(processing: HSVProcessingConfig) -> HSVProcessingConfig:
    if not isinstance(processing, HSVProcessingConfig):
        raise TypeError("processing must be an HSVProcessingConfig")
    blur_kernel = _validate_kernel(processing.blur_kernel, "processing.blur_kernel")
    open_kernel = _validate_kernel(processing.open_kernel, "processing.open_kernel")
    close_kernel = _validate_kernel(processing.close_kernel, "processing.close_kernel")
    for name, value in (("open_iterations", processing.open_iterations), ("close_iterations", processing.close_iterations)):
        if not _is_int(value) or value < 0:
            raise ValueError(f"processing.{name} must be a non-negative integer")
    return HSVProcessingConfig(blur_kernel, open_kernel, int(processing.open_iterations), close_kernel, int(processing.close_iterations))


@dataclass(frozen=True)
class HSVRange:
    lower: Tuple[int, int, int]
    upper: Tuple[int, int, int]


@dataclass(frozen=True)
class HSVColorProfile:
    name: str
    type_id: int
    enabled: bool
    ranges: Tuple[HSVRange, ...]


@dataclass(frozen=True)
class HSVProcessingConfig:
    blur_kernel: int
    open_kernel: int
    open_iterations: int
    close_kernel: int
    close_iterations: int


@dataclass(frozen=True)
class HSVSamplingConfig:
    scale_x: float
    scale_y: float
    min_pixels: int
    min_coverage: float
    min_margin: float


@dataclass(frozen=True)
class HSVConfig:
    version: int
    colors: Tuple[HSVColorProfile, ...]
    processing: HSVProcessingConfig
    sampling: HSVSamplingConfig


@dataclass(frozen=True)
class HSVClassification:
    type_id: Optional[int]
    color_name: Optional[str]
    confidence: float
    coverage: float
    purity: float
    margin: float
    valid_pixel_count: int
    sample_pixel_count: int
    counts: Dict[str, int]
    sample_bbox: Tuple[int, int, int, int]


@dataclass(frozen=True)
class HSVSampleRegion:
    """正式 HSV 分类使用的中心采样区域。

    ``sample_bbox`` 为半开区间，``ellipse_mask`` 与该区域同尺寸，只有椭圆
    内像素为 255。GUI、离线工具和 Jetson 分类都通过此对象保持采样语义一致。
    """

    sample_bbox: Tuple[int, int, int, int]
    ellipse_mask: np.ndarray
    sample_pixel_count: int


def _validate_range_object(value: HSVRange, path: str) -> HSVRange:
    lower = _validate_triplet(value.lower, f"{path}.lower", (179, 255, 255))
    upper = _validate_triplet(value.upper, f"{path}.upper", (179, 255, 255))
    for index, (low, high) in enumerate(zip(lower, upper)):
        if low > high:
            raise ValueError(f"{path}.lower[{index}] must not be greater than {path}.upper[{index}]")
    return HSVRange(lower, upper)


def _validate_config(config: HSVConfig) -> HSVConfig:
    if not _is_int(config.version) or config.version != SUPPORTED_CONFIG_VERSION:
        raise ValueError(f"version must be {SUPPORTED_CONFIG_VERSION}")
    if not config.colors:
        raise ValueError("colors must not be empty")
    types: set[int] = set()
    names: set[str] = set()
    profiles: List[HSVColorProfile] = []
    for index, profile in enumerate(config.colors):
        path = f"colors[{index}]"
        if not isinstance(profile.name, str) or not profile.name:
            raise ValueError(f"{path}.name must be a non-empty string")
        if profile.name in names:
            raise ValueError(f"{path}.name duplicates {profile.name!r}")
        names.add(profile.name)
        if not _is_int(profile.type_id) or not 0 <= profile.type_id <= 255:
            raise ValueError(f"{path}.type must be in [0, 255]")
        if profile.type_id in types:
            raise ValueError(f"{path}.type duplicates another color")
        types.add(profile.type_id)
        if not isinstance(profile.enabled, bool):
            raise ValueError(f"{path}.enabled must be a boolean")
        if not profile.ranges:
            raise ValueError(f"{path}.ranges must not be empty")
        ranges = tuple(_validate_range_object(item, f"{path}.ranges[{range_index}]") for range_index, item in enumerate(profile.ranges))
        profiles.append(HSVColorProfile(profile.name, int(profile.type_id), profile.enabled, ranges))

    sampling = config.sampling
    for name, value in (("scale_x", sampling.scale_x), ("scale_y", sampling.scale_y), ("min_coverage", sampling.min_coverage), ("min_margin", sampling.min_margin)):
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not 0.0 <= float(value) <= 1.0:
            raise ValueError(f"sampling.{name} must be in [0, 1]")
    if not _is_int(sampling.min_pixels) or sampling.min_pixels < 0:
        raise ValueError("sampling.min_pixels must be a non-negative integer")

    processing = _validate_processing(config.processing)

    return HSVConfig(
        version=SUPPORTED_CONFIG_VERSION,
        colors=tuple(profiles),
        processing=processing,
        sampling=HSVSamplingConfig(float(sampling.scale_x), float(sampling.scale_y), int(sampling.min_pixels), float(sampling.min_coverage), float(sampling.min_margin)),
    )


def _config_from_mapping(data: Mapping[str, Any]) -> HSVConfig:
    if not isinstance(data, Mapping):
        raise ValueError("configuration root must be an object")
    version = data.get("version")
    if not _is_int(version):
        raise ValueError("version must be an integer")
    colors_data = data.get("colors")
    if not isinstance(colors_data, Mapping) or not colors_data:
        raise ValueError("colors must be a non-empty object")
    profiles: List[HSVColorProfile] = []
    for name, raw in colors_data.items():
        path = f"colors.{name}"
        if not isinstance(name, str) or not name:
            raise ValueError("colors keys must be non-empty strings")
        if not isinstance(raw, Mapping):
            raise ValueError(f"{path} must be an object")
        type_id = raw.get("type")
        if not _is_int(type_id) or not 0 <= type_id <= 255:
            raise ValueError(f"{path}.type must be in [0, 255]")
        enabled = raw.get("enabled")
        if not isinstance(enabled, bool):
            raise ValueError(f"{path}.enabled must be a boolean")
        ranges_data = raw.get("ranges")
        if not isinstance(ranges_data, list) or not ranges_data:
            raise ValueError(f"{path}.ranges must be a non-empty list")
        ranges: List[HSVRange] = []
        for index, range_data in enumerate(ranges_data):
            range_path = f"{path}.ranges[{index}]"
            if not isinstance(range_data, Mapping):
                raise ValueError(f"{range_path} must be an object")
            lower = _validate_triplet(range_data.get("lower"), f"{range_path}.lower", (179, 255, 255))
            upper = _validate_triplet(range_data.get("upper"), f"{range_path}.upper", (179, 255, 255))
            for channel, (low, high) in enumerate(zip(lower, upper)):
                if low > high:
                    raise ValueError(f"{range_path}.lower[{channel}] must not be greater than {range_path}.upper[{channel}]")
            ranges.append(HSVRange(lower, upper))
        profiles.append(HSVColorProfile(name, int(type_id), enabled, tuple(ranges)))

    sampling_data = data.get("sampling")
    processing_data = data.get("processing")
    if not isinstance(sampling_data, Mapping):
        raise ValueError("sampling must be an object")
    if not isinstance(processing_data, Mapping):
        raise ValueError("processing must be an object")
    config = HSVConfig(
        version=int(version),
        colors=tuple(profiles),
        processing=HSVProcessingConfig(
            _validate_kernel(processing_data.get("blur_kernel"), "processing.blur_kernel"),
            _validate_kernel(processing_data.get("open_kernel"), "processing.open_kernel"),
            int(processing_data.get("open_iterations")) if _is_int(processing_data.get("open_iterations")) else -1,
            _validate_kernel(processing_data.get("close_kernel"), "processing.close_kernel"),
            int(processing_data.get("close_iterations")) if _is_int(processing_data.get("close_iterations")) else -1,
        ),
        sampling=HSVSamplingConfig(
            sampling_data.get("scale_x"),
            sampling_data.get("scale_y"),
            sampling_data.get("min_pixels"),
            sampling_data.get("min_coverage"),
            sampling_data.get("min_margin"),
        ),
    )
    return _validate_config(config)


def load_hsv_config(path: Union[str, Path]) -> HSVConfig:
    """读取并严格校验 HSV JSON 配置。"""
    config_path = Path(path)
    try:
        with config_path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
    except OSError as exc:
        raise ValueError(f"failed to read HSV config {config_path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid HSV config JSON {config_path}: {exc}") from exc
    return _config_from_mapping(data)


def _config_to_mapping(config: HSVConfig) -> Dict[str, Any]:
    checked = _validate_config(config)
    return {
        "version": checked.version,
        "sampling": {
            "scale_x": checked.sampling.scale_x,
            "scale_y": checked.sampling.scale_y,
            "min_pixels": checked.sampling.min_pixels,
            "min_coverage": checked.sampling.min_coverage,
            "min_margin": checked.sampling.min_margin,
        },
        "processing": {
            "blur_kernel": checked.processing.blur_kernel,
            "open_kernel": checked.processing.open_kernel,
            "open_iterations": checked.processing.open_iterations,
            "close_kernel": checked.processing.close_kernel,
            "close_iterations": checked.processing.close_iterations,
        },
        "colors": {
            profile.name: {
                "type": profile.type_id,
                "enabled": profile.enabled,
                "ranges": [{"lower": list(item.lower), "upper": list(item.upper)} for item in profile.ranges],
            }
            for profile in checked.colors
        },
    }


def save_hsv_config(config: HSVConfig, path: Union[str, Path]) -> None:
    """校验并以临时文件原子保存 HSV 配置。"""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + ".tmp")
    payload = json.dumps(_config_to_mapping(config), ensure_ascii=False, indent=2) + "\n"
    try:
        with temporary.open("w", encoding="utf-8", newline="\n") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(str(temporary), str(target))
    except OSError as exc:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise ValueError(f"failed to save HSV config {target}: {exc}") from exc


def _validate_frame_bgr(frame_bgr: np.ndarray) -> None:
    if not isinstance(frame_bgr, np.ndarray) or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3 or frame_bgr.size == 0 or frame_bgr.dtype != np.uint8:
        raise TypeError("frame_bgr must be a non-empty uint8 BGR numpy.ndarray")


def _validate_hsv_image(hsv_image: np.ndarray) -> None:
    if not isinstance(hsv_image, np.ndarray) or hsv_image.ndim != 3 or hsv_image.shape[2] != 3 or hsv_image.size == 0 or hsv_image.dtype != np.uint8:
        raise TypeError("hsv_image must be a non-empty uint8 HSV numpy.ndarray")


def _morphology(mask: np.ndarray, processing: HSVProcessingConfig) -> np.ndarray:
    result = mask
    for kernel_size, operation, iterations in (
        (processing.open_kernel, cv2.MORPH_OPEN, processing.open_iterations),
        (processing.close_kernel, cv2.MORPH_CLOSE, processing.close_iterations),
    ):
        if kernel_size and iterations:
            kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
            result = cv2.morphologyEx(result, operation, kernel, iterations=iterations)
    return result.astype(np.uint8, copy=False)


def _build_mask_from_hsv(hsv_image: np.ndarray, profile: HSVColorProfile, processing: HSVProcessingConfig, prepared: bool = False) -> np.ndarray:
    _validate_hsv_image(hsv_image)
    if not isinstance(profile, HSVColorProfile) or not profile.ranges:
        raise ValueError("profile.ranges must be a non-empty tuple")
    checked_profile = HSVColorProfile(profile.name, profile.type_id, profile.enabled, tuple(_validate_range_object(item, "profile.ranges") for item in profile.ranges))
    checked_processing = _validate_processing(processing)
    source = hsv_image
    if not prepared and checked_processing.blur_kernel:
        source = cv2.GaussianBlur(source, (checked_processing.blur_kernel, checked_processing.blur_kernel), 0)
    mask = np.zeros(source.shape[:2], dtype=np.uint8)
    for hsv_range in checked_profile.ranges:
        current = cv2.inRange(source, np.asarray(hsv_range.lower, dtype=np.uint8), np.asarray(hsv_range.upper, dtype=np.uint8))
        mask = cv2.bitwise_or(mask, current)
    return _morphology(mask, checked_processing)


def build_color_mask_from_hsv(hsv_image: np.ndarray, profile: HSVColorProfile, processing: HSVProcessingConfig) -> np.ndarray:
    """使用已缓存的 HSV 图像构建单色 Mask。"""
    return _build_mask_from_hsv(hsv_image, profile, processing)


def build_color_mask(frame_bgr: np.ndarray, profile: HSVColorProfile, processing: HSVProcessingConfig) -> np.ndarray:
    """将 BGR 图像转换为 HSV 并构建单色 uint8 Mask。"""
    _validate_frame_bgr(frame_bgr)
    checked_processing = _validate_processing(processing)
    source = frame_bgr
    if checked_processing.blur_kernel:
        source = cv2.GaussianBlur(source, (checked_processing.blur_kernel, checked_processing.blur_kernel), 0)
    hsv_image = cv2.cvtColor(source, cv2.COLOR_BGR2HSV)
    return _build_mask_from_hsv(hsv_image, profile, checked_processing, prepared=True)


def build_all_color_masks(frame_bgr: np.ndarray, config: HSVConfig) -> Dict[str, np.ndarray]:
    """为所有启用颜色构建独立 Mask。"""
    _validate_frame_bgr(frame_bgr)
    checked = _validate_config(config)
    hsv_image = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    if checked.processing.blur_kernel:
        hsv_image = cv2.GaussianBlur(hsv_image, (checked.processing.blur_kernel, checked.processing.blur_kernel), 0)
    return {profile.name: _build_mask_from_hsv(hsv_image, profile, checked.processing, prepared=True) for profile in checked.colors if profile.enabled}


def compute_sample_bbox(
    bbox: Sequence[int], frame_width: int, frame_height: int, scale_x: float, scale_y: float,
) -> Optional[Tuple[int, int, int, int]]:
    """在 bbox 中心生成缩小采样框，并裁剪到图像边界。"""
    try:
        bbox_length = len(bbox)
    except TypeError:
        return None
    if bbox_length != 4 or frame_width <= 0 or frame_height <= 0 or not 0.0 < scale_x <= 1.0 or not 0.0 < scale_y <= 1.0:
        return None
    if not all(_is_int(value) for value in bbox):
        return None
    raw_x1, raw_y1, raw_x2, raw_y2 = (int(value) for value in bbox)
    x1, x2 = sorted((raw_x1, raw_x2))
    y1, y2 = sorted((raw_y1, raw_y2))
    x1, x2 = max(0, min(frame_width, x1)), max(0, min(frame_width, x2))
    y1, y2 = max(0, min(frame_height, y1)), max(0, min(frame_height, y2))
    if x2 <= x1 or y2 <= y1:
        return None
    width = max(1, int(round((x2 - x1) * scale_x)))
    height = max(1, int(round((y2 - y1) * scale_y)))
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    sample_x1 = int(math.floor(center_x - width / 2.0))
    sample_y1 = int(math.floor(center_y - height / 2.0))
    sample_x2 = sample_x1 + width
    sample_y2 = sample_y1 + height
    if sample_x1 < 0:
        sample_x2 -= sample_x1
        sample_x1 = 0
    if sample_y1 < 0:
        sample_y2 -= sample_y1
        sample_y1 = 0
    if sample_x2 > frame_width:
        sample_x1 -= sample_x2 - frame_width
        sample_x2 = frame_width
    if sample_y2 > frame_height:
        sample_y1 -= sample_y2 - frame_height
        sample_y2 = frame_height
    if sample_x2 - sample_x1 <= 0 or sample_y2 - sample_y1 <= 0:
        return None
    return sample_x1, sample_y1, sample_x2, sample_y2


def build_hsv_sample_region(
    bbox: Sequence[int],
    frame_width: int,
    frame_height: int,
    sampling: HSVSamplingConfig,
) -> Optional[HSVSampleRegion]:
    """根据 bbox 和采样配置构造正式分类使用的椭圆区域。"""
    if not isinstance(sampling, HSVSamplingConfig):
        raise TypeError("sampling must be an HSVSamplingConfig")
    sample_bbox = compute_sample_bbox(
        bbox,
        frame_width,
        frame_height,
        float(sampling.scale_x),
        float(sampling.scale_y),
    )
    if sample_bbox is None:
        return None
    x1, y1, x2, y2 = sample_bbox
    sample_height = y2 - y1
    sample_width = x2 - x1
    ellipse_mask = np.zeros((sample_height, sample_width), dtype=np.uint8)
    cv2.ellipse(
        ellipse_mask,
        (sample_width // 2, sample_height // 2),
        (max(1, sample_width // 2), max(1, sample_height // 2)),
        0,
        0,
        360,
        255,
        -1,
    )
    sample_pixel_count = int(cv2.countNonZero(ellipse_mask))
    if sample_pixel_count <= 0:
        return None
    ellipse_mask.setflags(write=False)
    return HSVSampleRegion(sample_bbox, ellipse_mask, sample_pixel_count)


def _empty_classification(sample_bbox: Tuple[int, int, int, int] = (0, 0, 0, 0), counts: Optional[Dict[str, int]] = None, sample_pixel_count: int = 0) -> HSVClassification:
    return HSVClassification(None, None, 0.0, 0.0, 0.0, 0.0, 0, sample_pixel_count, counts or {}, sample_bbox)


def _classify_region_hsv(frame_bgr: np.ndarray, bbox: Sequence[int], config: HSVConfig) -> HSVClassification:
    _validate_frame_bgr(frame_bgr)
    checked = _validate_config(config)
    height, width = frame_bgr.shape[:2]
    sample_region = build_hsv_sample_region(bbox, width, height, checked.sampling)
    if sample_region is None:
        return _empty_classification()
    sample_bbox = sample_region.sample_bbox
    x1, y1, x2, y2 = sample_bbox
    ellipse = sample_region.ellipse_mask
    sample_pixel_count = sample_region.sample_pixel_count
    masks = build_all_color_masks(frame_bgr, checked)
    counts: Dict[str, int] = {}
    for name, mask in masks.items():
        counts[name] = int(cv2.countNonZero(cv2.bitwise_and(mask[y1:y2, x1:x2], ellipse)))
    ordered = sorted(counts.items(), key=lambda item: (-item[1], item[0]))
    first_count = ordered[0][1] if ordered else 0
    second_count = ordered[1][1] if len(ordered) > 1 else 0
    valid_pixel_count = sum(counts.values())
    coverage = first_count / sample_pixel_count
    purity = first_count / valid_pixel_count if valid_pixel_count else 0.0
    margin = (first_count - second_count) / sample_pixel_count
    confidence = max(0.0, min(1.0, (coverage + purity + max(0.0, margin)) / 3.0))
    if not ordered or first_count <= 0 or sample_pixel_count < checked.sampling.min_pixels or coverage < checked.sampling.min_coverage or margin < checked.sampling.min_margin:
        return HSVClassification(None, None, confidence, coverage, purity, margin, valid_pixel_count, sample_pixel_count, counts, sample_bbox)
    winner = next(profile for profile in checked.colors if profile.name == ordered[0][0])
    return HSVClassification(winner.type_id, winner.name, confidence, coverage, purity, margin, valid_pixel_count, sample_pixel_count, counts, sample_bbox)


def classify_bbox_hsv(frame_bgr: np.ndarray, bbox: Sequence[int], config: HSVConfig) -> HSVClassification:
    """按 bbox 中心椭圆区域统计颜色并分类。"""
    return _classify_region_hsv(frame_bgr, bbox, config)


def classify_roi_hsv(roi_bgr: np.ndarray, config: HSVConfig) -> HSVClassification:
    """将完整 ROI 作为 bbox，复用 bbox 分类统计逻辑。"""
    _validate_frame_bgr(roi_bgr)
    height, width = roi_bgr.shape[:2]
    return _classify_region_hsv(roi_bgr, (0, 0, width, height), config)


# Legacy Hough HSV API -----------------------------------------------------

def classify_color_hsv(roi_bgr: np.ndarray) -> Optional[int]:
    """按旧版 BGR 中值规则识别红、黄、蓝、绿。"""
    if not isinstance(roi_bgr, np.ndarray) or roi_bgr.size == 0:
        return None
    if roi_bgr.ndim != 3 or roi_bgr.shape[2] != 3:
        return None
    median_bgr = np.median(roi_bgr.reshape(-1, 3), axis=0).astype(np.uint8)
    hsv_pixel = cv2.cvtColor(median_bgr.reshape(1, 1, 3), cv2.COLOR_BGR2HSV)[0, 0]
    hue, saturation, value = (int(channel) for channel in hsv_pixel)
    if saturation < 25 and value > 200:
        return None
    if value < 70 or saturation < 45:
        return None
    if hue < 8 or hue >= 165:
        return RED
    if 10 <= hue < 34:
        return YELLOW
    if 35 <= hue < 85:
        return GREEN
    if 104 <= hue < 140:
        return BLUE
    return None


def detect_color_hsv(frame_bgr: np.ndarray) -> Dict[str, List[Dict[str, Any]]]:
    """保留旧版 Hough 圆检测和 HSV 分类接口。"""
    _validate_frame_bgr(frame_bgr)
    height, width = frame_bgr.shape[:2]
    radius_scale = 0.5 * (height + width)
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    denoised = cv2.medianBlur(gray, 7)
    blurred = cv2.GaussianBlur(denoised, (9, 9), 2)
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=85, param1=50, param2=32, minRadius=max(1, int(radius_scale * 0.05)), maxRadius=max(1, int(radius_scale * 0.35)))
    if circles is None:
        return {"detections": []}
    detections: List[Dict[str, Any]] = []
    for center_x, center_y, radius in np.round(circles[0]).astype(int):
        center_x = int(np.clip(center_x, 0, width - 1))
        center_y = int(np.clip(center_y, 0, height - 1))
        radius = max(1, int(radius))
        roi_radius = max(5, int(radius * 0.6))
        roi_y1, roi_y2 = max(0, center_y - roi_radius), min(height, center_y + roi_radius)
        roi_x1, roi_x2 = max(0, center_x - roi_radius), min(width, center_x + roi_radius)
        color_type = classify_color_hsv(frame_bgr[roi_y1:roi_y2, roi_x1:roi_x2])
        if color_type is None:
            continue
        detections.append({"type": color_type, "center": [center_x, center_y], "bbox": [max(0, center_x - radius), max(0, center_y - radius), min(width - 1, center_x + radius), min(height - 1, center_y + radius)], "confidence": 1.0})
    detections.sort(key=lambda detection: detection["bbox"][2] - detection["bbox"][0], reverse=True)
    return {"detections": detections[:3]}
