from copy import deepcopy

import cv2
import numpy as np
import pytest

import vision.advance_yolo as advance_yolo
import vision.hybrid_color as hybrid_color
from vision.hsv_color import HSVClassification


def _classification(type_id=2, color_name="blue", confidence=0.82):
    return HSVClassification(
        type_id=type_id,
        color_name=color_name,
        confidence=confidence,
        coverage=0.91,
        purity=0.93,
        margin=0.76,
        valid_pixel_count=90,
        sample_pixel_count=100,
        counts={color_name: 90},
        sample_bbox=(12, 14, 32, 34),
    )


def test_hybrid_keeps_yolo_geometry_and_adds_hsv_classification(monkeypatch):
    frame = np.zeros((80, 100, 3), dtype=np.uint8)
    raw_detection = {"type": 0, "center": [22, 24], "bbox": [10, 12, 34, 36], "confidence": 0.64}
    original = deepcopy(raw_detection)
    monkeypatch.setattr(hybrid_color, "detect_color", lambda _frame: {"detections": [raw_detection]})
    monkeypatch.setattr(hybrid_color, "classify_bbox_hsv", lambda *_args: _classification())

    result = hybrid_color.detect_color_hybrid(frame, config=object())

    assert raw_detection == original
    refined = result["detections"][0]
    assert refined["type"] == 2 and refined["confidence"] == pytest.approx(0.82)
    assert refined["center"] == [22, 24] and refined["bbox"] == [10, 12, 34, 36]
    assert refined["yolo_type"] == 0 and refined["yolo_confidence"] == pytest.approx(0.64)
    assert refined["hsv_color"] == "blue" and refined["classification_source"] == "hsv"


def test_hybrid_does_not_call_hough_circles(monkeypatch):
    frame = np.zeros((80, 100, 3), dtype=np.uint8)
    detection = {"type": 0, "center": [22, 24], "bbox": [10, 12, 34, 36], "confidence": 0.64}
    monkeypatch.setattr(hybrid_color, "detect_color", lambda _frame: {"detections": [detection]})
    monkeypatch.setattr(hybrid_color, "classify_bbox_hsv", lambda *_args: _classification())
    monkeypatch.setattr(cv2, "HoughCircles", lambda *_args, **_kwargs: pytest.fail("hybrid path must not use HoughCircles"))

    assert hybrid_color.detect_color_hybrid(frame, config=object())["detections"][0]["type"] == 2


@pytest.mark.parametrize("policy, expected_count, expected_source", [("reject", 0, None), ("keep_yolo", 1, "yolo_fallback")])
def test_hybrid_uncertain_hsv_honors_policy(monkeypatch, policy, expected_count, expected_source):
    frame = np.zeros((80, 100, 3), dtype=np.uint8)
    raw_detection = {"type": 1, "center": [22, 24], "bbox": [10, 12, 34, 36], "confidence": 0.64}
    monkeypatch.setattr(hybrid_color, "detect_color", lambda _frame: {"detections": [raw_detection]})
    monkeypatch.setattr(hybrid_color, "classify_bbox_hsv", lambda *_args: _classification(None, None, 0.0))

    result = hybrid_color.detect_color_hybrid(frame, config=object(), uncertain_policy=policy)

    assert len(result["detections"]) == expected_count
    if expected_source is not None:
        refined = result["detections"][0]
        assert refined["type"] == 1 and refined["confidence"] == pytest.approx(0.64)
        assert refined["classification_source"] == expected_source


def test_hybrid_keeps_empty_slot_as_yolo_result(monkeypatch):
    frame = np.zeros((80, 100, 3), dtype=np.uint8)
    detection = {"type": hybrid_color.EMPTY_SLOT_TYPE, "center": [22, 24], "bbox": [10, 12, 34, 36], "confidence": 0.64}
    monkeypatch.setattr(hybrid_color, "detect_color", lambda _frame: {"detections": [detection]})
    monkeypatch.setattr(hybrid_color, "classify_bbox_hsv", lambda *_args: pytest.fail("EmptySlot must not enter HSV classification"))

    refined = hybrid_color.detect_color_hybrid(frame, config=object())["detections"][0]

    assert refined["type"] == hybrid_color.EMPTY_SLOT_TYPE
    assert refined["classification_source"] == "yolo_empty_slot"
    assert refined["hsv_color"] is None


def test_hybrid_tracking_preserves_debug_fields(monkeypatch):
    frame = np.zeros((80, 100, 3), dtype=np.uint8)
    detection = {
        "type": 2,
        "center": [22, 24],
        "bbox": [10, 12, 34, 36],
        "confidence": 0.82,
        "yolo_type": 0,
        "yolo_confidence": 0.64,
        "hsv_color": "blue",
        "hsv_coverage": 0.91,
        "hsv_purity": 0.93,
        "hsv_margin": 0.76,
        "classification_source": "hsv",
    }
    monkeypatch.setattr(advance_yolo, "detect_color_hybrid", lambda _frame: {"detections": [detection]})
    advance_yolo.reset_advance_tracking()
    try:
        assert advance_yolo.advance_detect_color_hybrid(frame) == {"detections": []}
        tracked = advance_yolo.advance_detect_color_hybrid(frame)["detections"][0]
    finally:
        advance_yolo.reset_advance_tracking()

    for field in ("yolo_type", "yolo_confidence", "hsv_color", "hsv_coverage", "hsv_purity", "hsv_margin", "classification_source"):
        assert tracked[field] == detection[field]
