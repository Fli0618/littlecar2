import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.hsv_tuner.recommendation import (
    analyze_hsv_range_overlap,
    merge_duplicate_ranges,
    recommend_hsv_profile,
)
from vision.hsv_color import HSVColorProfile, HSVConfig, HSVProcessingConfig, HSVRange, HSVSamplingConfig


def _config(*profiles, min_pixels=20):
    return HSVConfig(
        1,
        tuple(profiles),
        HSVProcessingConfig(0, 0, 0, 0, 0),
        HSVSamplingConfig(1.0, 1.0, min_pixels, 0.1, 0.1),
    )


def _ellipse_frame(hues, saturation=220, value=220):
    hsv = np.zeros((100, 100, 3), dtype=np.uint8)
    ellipse = np.zeros((100, 100), dtype=np.uint8)
    cv2.ellipse(ellipse, (50, 50), (30, 30), 0, 0, 360, 255, -1)
    ys, xs = np.where(ellipse > 0)
    for index, (y, x) in enumerate(zip(ys, xs)):
        hsv[y, x] = (hues[index % len(hues)], saturation, value)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def test_continuous_hue_recommends_one_range_and_disables_second():
    profile = HSVColorProfile("yellow", 1, True, (HSVRange((0, 0, 0), (179, 255, 255)),))
    result = recommend_hsv_profile(_ellipse_frame(range(25, 35)), (20, 20, 80, 80), _config(profile), "yellow")

    assert len(result.ranges) == 1
    assert result.ranges[0].lower[0] <= 25 <= result.ranges[0].upper[0]
    assert result.foreground_coverage > 0.65


def test_red_wrap_recommends_low_h_as_a_and_high_h_as_b():
    profile = HSVColorProfile("red", 0, True, (HSVRange((0, 0, 0), (179, 255, 255)),))
    result = recommend_hsv_profile(_ellipse_frame((0, 1, 2, 177, 178, 179)), (20, 20, 80, 80), _config(profile), "red")

    assert len(result.ranges) == 2
    assert result.ranges[0].lower[0] == 0
    assert result.ranges[1].upper[0] == 179
    assert result.ranges[0].upper[0] < result.ranges[1].lower[0]


def test_black_recommendation_uses_full_hue_and_value_upper_bound():
    hsv = np.full((100, 100, 3), (0, 0, 220), dtype=np.uint8)
    ellipse = np.zeros((100, 100), dtype=np.uint8)
    cv2.ellipse(ellipse, (50, 50), (30, 30), 0, 0, 360, 255, -1)
    hsv[ellipse > 0] = (37, 15, 45)
    frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    profile = HSVColorProfile("black", 4, False, (HSVRange((0, 0, 0), (179, 255, 255)),))

    result = recommend_hsv_profile(frame, (20, 20, 80, 80), _config(profile), "black")

    assert result.ranges[0].lower[:2] == (0, 0)
    assert result.ranges[0].upper[:2] == (179, 255)
    assert result.ranges[0].upper[2] < 100


def test_invalid_roi_and_insufficient_sampling_are_rejected():
    profile = HSVColorProfile("red", 0, True, (HSVRange((0, 0, 0), (179, 255, 255)),))
    frame = np.zeros((20, 20, 3), dtype=np.uint8)

    with pytest.raises(ValueError, match="ROI"):
        recommend_hsv_profile(frame, (2, 2, 2, 5), _config(profile), "red")
    with pytest.raises(ValueError, match="采样像素不足"):
        recommend_hsv_profile(frame, (1, 1, 19, 19), _config(profile, min_pixels=10000), "red")


def test_duplicate_and_contained_ranges_are_reported_and_merge_keeps_extras():
    first = HSVRange((0, 100, 100), (10, 255, 255))
    second = HSVRange((2, 120, 120), (8, 230, 230))
    third = HSVRange((100, 100, 100), (110, 255, 255))
    profile = HSVColorProfile("red", 0, True, (first, second, third))
    hsv = np.zeros((30, 30, 3), dtype=np.uint8)

    report = analyze_hsv_range_overlap(hsv, profile)
    merged = merge_duplicate_ranges(profile)

    assert report.contains is True
    assert report.warning is not None
    assert merged.ranges == (first, third)
