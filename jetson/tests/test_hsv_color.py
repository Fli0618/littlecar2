import json

import cv2
import numpy as np
import pytest

from vision.hsv_color import (
    HSVColorProfile,
    HSVConfig,
    HSVProcessingConfig,
    HSVRange,
    HSVSamplingConfig,
    build_hsv_sample_region,
    build_all_color_masks,
    build_color_mask,
    build_color_mask_from_hsv,
    classify_bbox_hsv,
    classify_roi_hsv,
    compute_sample_bbox,
    load_hsv_config,
    save_hsv_config,
)


def _config(*, min_coverage=0.5, min_margin=0.2):
    return HSVConfig(
        version=1,
        colors=(
            HSVColorProfile("red", 0, True, (HSVRange((0, 150, 150), (10, 255, 255)),)),
            HSVColorProfile("blue", 2, True, (HSVRange((100, 150, 150), (130, 255, 255)),)),
            HSVColorProfile("disabled", 9, False, (HSVRange((40, 150, 150), (80, 255, 255)),)),
        ),
        processing=HSVProcessingConfig(0, 0, 0, 0, 0),
        sampling=HSVSamplingConfig(1.0, 1.0, 1, min_coverage, min_margin),
    )


def test_config_round_trip_preserves_values_and_writes_utf8_json(tmp_path):
    target = tmp_path / "config" / "hsv_colors.json"

    save_hsv_config(_config(), target)

    assert load_hsv_config(target) == _config()
    assert json.loads(target.read_text(encoding="utf-8"))["colors"]["red"]["type"] == 0


@pytest.mark.parametrize(
    "payload, message",
    [
        ({"version": 2}, "colors"),
        ({"version": 1, "colors": {}}, "colors"),
        ({"version": 1, "colors": {"red": {"type": 0, "enabled": True, "ranges": [{"lower": [11, 0, 0], "upper": [10, 255, 255]}]}}, "sampling": {}, "processing": {}}, "lower"),
    ],
)
def test_load_config_rejects_invalid_schema(tmp_path, payload, message):
    target = tmp_path / "invalid.json"
    target.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        load_hsv_config(target)


def test_color_mask_builders_match_for_bgr_and_cached_hsv_input():
    frame = np.zeros((8, 8, 3), dtype=np.uint8)
    frame[:, :4] = (0, 0, 255)
    profile = _config().colors[0]

    from_bgr = build_color_mask(frame, profile, _config().processing)
    from_hsv = build_color_mask_from_hsv(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), profile, _config().processing)

    assert np.array_equal(from_bgr, from_hsv)
    assert np.all(from_bgr[:, :4] == 255)
    assert not from_bgr[:, 4:].any()


def test_build_all_color_masks_returns_enabled_profiles_only():
    masks = build_all_color_masks(np.zeros((6, 6, 3), dtype=np.uint8), _config())

    assert set(masks) == {"red", "blue"}
    assert all(mask.dtype == np.uint8 and mask.shape == (6, 6) for mask in masks.values())


@pytest.mark.parametrize(
    ("bbox", "expected"),
    [
        ((10, 20, 30, 60), (15, 30, 25, 50)),
        ((-8, -4, 8, 4), (2, 1, 6, 3)),
        ((0, 0, 0, 4), None),
        ((0, 0, 4), None),
    ],
)
def test_compute_sample_bbox_clips_and_rejects_invalid_input(bbox, expected):
    assert compute_sample_bbox(bbox, 100, 80, 0.5, 0.5) == expected


def test_classify_bbox_returns_winning_color_and_metrics():
    frame = np.full((20, 20, 3), (0, 0, 255), dtype=np.uint8)

    result = classify_bbox_hsv(frame, (2, 2, 18, 18), _config())

    assert result.type_id == 0
    assert result.color_name == "red"
    assert result.coverage == pytest.approx(1.0)
    assert result.purity == pytest.approx(1.0)
    assert result.margin == pytest.approx(1.0)
    assert result.counts == {"red": result.sample_pixel_count, "blue": 0}


def test_classify_roi_rejects_ambiguous_or_low_coverage_samples():
    frame = np.zeros((20, 20, 3), dtype=np.uint8)
    frame[:, :10] = (0, 0, 255)
    frame[:, 10:] = (255, 0, 0)

    result = classify_roi_hsv(frame, _config(min_coverage=0.1, min_margin=0.2))

    assert result.type_id is None
    assert result.color_name is None
    assert result.counts["red"] > 0
    assert result.counts["blue"] > 0


def test_classification_returns_empty_result_for_invalid_bbox():
    result = classify_bbox_hsv(np.zeros((8, 8, 3), dtype=np.uint8), (2, 2, 2, 6), _config())

    assert result.type_id is None
    assert result.sample_bbox == (0, 0, 0, 0)
    assert result.sample_pixel_count == 0


def test_build_hsv_sample_region_matches_classification_sampling_ellipse():
    sampling = HSVSamplingConfig(0.5, 0.5, 1, 0.0, 0.0)
    region = build_hsv_sample_region((2, 4, 18, 20), 24, 24, sampling)

    assert region is not None
    assert region.sample_bbox == (6, 8, 14, 16)
    assert region.ellipse_mask.shape == (8, 8)
    assert region.sample_pixel_count == int(cv2.countNonZero(region.ellipse_mask))
    assert not region.ellipse_mask.flags.writeable
