"""基于正式中心椭圆采样的 HSV 参数推荐与区间诊断。"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np

from vision.hsv_color import (
    HSVColorProfile,
    HSVConfig,
    HSVProcessingConfig,
    HSVRange,
    HSVSamplingConfig,
    build_color_mask_from_hsv,
    build_hsv_sample_region,
)


@dataclass(frozen=True)
class HSVRecommendation:
    color_name: str
    ranges: Tuple[HSVRange, ...]
    foreground_coverage: float
    background_leakage: float
    conflict_ratio: float
    confidence: float
    sample_pixel_count: int
    foreground_pixel_count: int
    background_pixel_count: int
    warnings: Tuple[str, ...]


@dataclass(frozen=True)
class HSVRangeOverlap:
    duplicate: bool
    contains: bool
    volume_overlap_ratio: float
    new_pixel_count: int
    warning: Optional[str]


def _profile(config: HSVConfig, color_name: str) -> HSVColorProfile:
    for profile in config.colors:
        if profile.name == color_name:
            return profile
    raise ValueError(f"unknown HSV color: {color_name}")


def _range_volume(item: HSVRange) -> int:
    return int(np.prod([upper - lower + 1 for lower, upper in zip(item.lower, item.upper)]))


def _contains(outer: HSVRange, inner: HSVRange) -> bool:
    return all(a <= c and b >= d for a, b, c, d in zip(outer.lower, outer.upper, inner.lower, inner.upper))


def _intersection_volume(first: HSVRange, second: HSVRange) -> int:
    lengths = [max(0, min(a, b) - max(c, d) + 1) for a, b, c, d in zip(first.upper, second.upper, first.lower, second.lower)]
    return int(np.prod(lengths)) if all(lengths) else 0


def analyze_hsv_range_overlap(
    hsv_image: np.ndarray,
    profile: HSVColorProfile,
    processing: Optional[HSVProcessingConfig] = None,
) -> HSVRangeOverlap:
    """分析当前颜色前两组区间的几何交集和实际新增像素。"""
    if len(profile.ranges) < 2:
        return HSVRangeOverlap(False, False, 0.0, 0, None)
    first, second = profile.ranges[:2]
    duplicate = first == second
    contains = _contains(first, second) or _contains(second, first)
    overlap_ratio = _intersection_volume(first, second) / float(max(1, min(_range_volume(first), _range_volume(second))))
    first_profile = HSVColorProfile(profile.name, profile.type_id, True, (first,))
    second_profile = HSVColorProfile(profile.name, profile.type_id, True, (second,))
    first_mask = build_color_mask_from_hsv(hsv_image, first_profile, processing or HSVProcessingConfig(0, 0, 0, 0, 0))
    second_mask = build_color_mask_from_hsv(hsv_image, second_profile, processing or HSVProcessingConfig(0, 0, 0, 0, 0))
    new_pixel_count = int(cv2.countNonZero(cv2.bitwise_and(second_mask, cv2.bitwise_not(first_mask))))
    warning = None
    if duplicate or contains or overlap_ratio >= 0.70 or new_pixel_count == 0:
        warning = "两个HSV区间高度重叠，第二组未提供有效新增覆盖。"
    return HSVRangeOverlap(duplicate, contains, float(overlap_ratio), new_pixel_count, warning)


def merge_duplicate_ranges(profile: HSVColorProfile) -> HSVColorProfile:
    """保留 A 和第 3 组以后区间，并关闭第二组。"""
    if len(profile.ranges) < 2:
        return profile
    return replace(profile, ranges=(profile.ranges[0],) + tuple(profile.ranges[2:]))


def _weighted_hue_histogram(hues: np.ndarray, saturation: np.ndarray) -> np.ndarray:
    return np.bincount(hues.astype(np.uint8), weights=saturation.astype(np.float64), minlength=180).astype(np.float64)


def _shortest_circular_interval(histogram: np.ndarray, target_ratio: float) -> Tuple[int, int]:
    total = float(histogram.sum())
    if total <= 0:
        raise ValueError("没有有效色相像素")
    target = total * target_ratio
    doubled = np.concatenate((histogram, histogram))
    best_start, best_length, best_mass = 0, 180, -1.0
    for start in range(180):
        mass = 0.0
        for length in range(1, 181):
            mass += doubled[start + length - 1]
            if mass >= target:
                if length < best_length or (length == best_length and mass > best_mass):
                    best_start, best_length, best_mass = start, length, mass
                break
    return best_start % 180, (best_start + best_length - 1) % 180


def _expand_hue_interval(start: int, end: int, margin: int) -> Tuple[HSVRange, ...]:
    length = (end - start) % 180 + 1
    if length + margin * 2 >= 180:
        return (HSVRange((0, 0, 0), (179, 255, 255)),)
    expanded_start = start - margin
    expanded_end = end + margin
    if start > end or expanded_start < 0 or expanded_end >= 180:
        low_end = expanded_end % 180
        high_start = expanded_start % 180
        return (
            HSVRange((0, 0, 0), (low_end, 255, 255)),
            HSVRange((high_start, 0, 0), (179, 255, 255)),
        )
    return (HSVRange((expanded_start, 0, 0), (expanded_end, 255, 255)),)


def _candidate_mask(hsv_image: np.ndarray, ranges: Sequence[HSVRange]) -> np.ndarray:
    mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
    for item in ranges:
        current = cv2.inRange(hsv_image, np.asarray(item.lower, dtype=np.uint8), np.asarray(item.upper, dtype=np.uint8))
        mask = cv2.bitwise_or(mask, current)
    return mask


def _background_mask(bbox: Sequence[int], sample_bbox: Tuple[int, int, int, int], ellipse_mask: np.ndarray, width: int, height: int) -> np.ndarray:
    if len(bbox) != 4:
        return np.zeros((height, width), dtype=np.uint8)
    x1, x2 = sorted((max(0, min(width, int(bbox[0]))), max(0, min(width, int(bbox[2])))))
    y1, y2 = sorted((max(0, min(height, int(bbox[1]))), max(0, min(height, int(bbox[3])))))
    result = np.zeros((height, width), dtype=np.uint8)
    if x2 <= x1 or y2 <= y1:
        return result
    result[y1:y2, x1:x2] = 255
    sx1, sy1, sx2, sy2 = sample_bbox
    result[sy1:sy2, sx1:sx2][ellipse_mask > 0] = 0
    return result


def _score_candidate(
    candidate: np.ndarray,
    foreground: np.ndarray,
    background: np.ndarray,
    other_masks: Sequence[np.ndarray],
) -> Tuple[float, float, float, float]:
    fg_hits = (candidate > 0) & (foreground > 0)
    bg_hits = (candidate > 0) & (background > 0)
    foreground_coverage = float(np.count_nonzero(fg_hits)) / max(1, int(np.count_nonzero(foreground)))
    background_leakage = float(np.count_nonzero(bg_hits)) / max(1, int(np.count_nonzero(background)))
    conflict = np.zeros(candidate.shape, dtype=bool)
    for mask in other_masks:
        conflict |= (mask > 0) & fg_hits
    conflict_ratio = float(np.count_nonzero(conflict)) / max(1, int(np.count_nonzero(fg_hits)))
    score = foreground_coverage - 1.5 * background_leakage - conflict_ratio
    return score, foreground_coverage, background_leakage, conflict_ratio


def recommend_hsv_profile(
    frame_bgr: np.ndarray,
    bbox: Sequence[int],
    config: HSVConfig,
    color_name: str,
) -> HSVRecommendation:
    """根据当前 ROI 的正式中心椭圆生成 HSV 初始参数。"""
    if not isinstance(frame_bgr, np.ndarray) or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3 or frame_bgr.size == 0:
        raise ValueError("frame_bgr must be a non-empty BGR image")
    profile = _profile(config, color_name)
    height, width = frame_bgr.shape[:2]
    region = build_hsv_sample_region(bbox, width, height, config.sampling)
    if region is None:
        raise ValueError("ROI 无效，无法构造正式中心椭圆采样区域")
    if region.sample_pixel_count < max(1, config.sampling.min_pixels):
        raise ValueError(f"正式采样像素不足：{region.sample_pixel_count} < {config.sampling.min_pixels}")
    hsv_image = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    x1, y1, x2, y2 = region.sample_bbox
    foreground = np.zeros((height, width), dtype=np.uint8)
    foreground[y1:y2, x1:x2] = region.ellipse_mask
    values = hsv_image[foreground > 0]
    is_black = color_name == "black"
    if is_black:
        valid = values
    else:
        valid = values[(values[:, 1] >= 40) & (values[:, 2] >= 30)]
    if valid.shape[0] < max(1, min(config.sampling.min_pixels, region.sample_pixel_count)):
        raise ValueError(f"有效颜色像素不足：{valid.shape[0]}")
    background = _background_mask(bbox, region.sample_bbox, region.ellipse_mask, width, height)
    other_masks = []
    for other in config.colors:
        if other.enabled and other.name != color_name:
            other_masks.append(build_color_mask_from_hsv(hsv_image, other, config.processing))
    candidates = []
    if is_black:
        v95 = int(math.ceil(float(np.percentile(valid[:, 2], 95))))
        ranges = (HSVRange((0, 0, 0), (179, 255, min(255, v95 + 8))),)
        candidates.append(ranges)
    else:
        hue_hist = _weighted_hue_histogram(valid[:, 0], valid[:, 1])
        s05, s95 = np.percentile(valid[:, 1], (5, 95))
        v05, v95 = np.percentile(valid[:, 2], (5, 95))
        s_lower = max(0, int(math.floor(s05)) - 5)
        s_upper = min(255, int(math.ceil(s95)) + 5)
        v_lower = max(0, int(math.floor(v05)) - 8)
        v_upper = min(255, int(math.ceil(v95)) + 8)
        for target_ratio in (0.90, 0.95, 0.98):
            start, end = _shortest_circular_interval(hue_hist, target_ratio)
            for hue_margin in (2, 3, 4):
                base_ranges = _expand_hue_interval(start, end, hue_margin)
                ranges = tuple(HSVRange((item.lower[0], s_lower, v_lower), (item.upper[0], s_upper, v_upper)) for item in base_ranges)
                candidates.append(ranges)
    best = None
    for ranges in candidates:
        candidate_mask = _candidate_mask(hsv_image, ranges)
        score = _score_candidate(candidate_mask, foreground, background, other_masks)
        if best is None or score[0] > best[0]:
            best = (score[0], ranges, score[1], score[2], score[3])
    assert best is not None
    _, ranges, foreground_coverage, background_leakage, conflict_ratio = best
    warnings = []
    if foreground_coverage < 0.80:
        warnings.append("前景覆盖率偏低")
    if background_leakage > 0.08:
        warnings.append("背景泄漏率偏高")
    if conflict_ratio > 0.10:
        warnings.append("与其他启用颜色存在冲突")
    if not is_black and np.count_nonzero(_weighted_hue_histogram(valid[:, 0], valid[:, 1])) >= 2:
        warnings.append("ROI 内存在多个色相峰，请切换图片验证")
    if foreground_coverage < 0.65 or background_leakage > 0.20:
        raise ValueError(
            f"推荐质量不足：前景覆盖率={foreground_coverage:.3f}，背景泄漏率={background_leakage:.3f}"
        )
    confidence = max(0.0, min(1.0, foreground_coverage - 1.5 * background_leakage - conflict_ratio))
    return HSVRecommendation(
        color_name,
        tuple(ranges),
        foreground_coverage,
        background_leakage,
        conflict_ratio,
        confidence,
        region.sample_pixel_count,
        int(valid.shape[0]),
        int(np.count_nonzero(background)),
        tuple(warnings),
    )
