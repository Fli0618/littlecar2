import sys
from pathlib import Path
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import vision.advance_yolo as advance_yolo
from vision.materials import advance_detect_disk_center, detect_disk_center
from vision import yolo


def detection(kind, x, y, confidence=0.9):
    return {"type": kind, "center": [x, y], "confidence": confidence}


def test_yolo_exposes_raw_class_id_and_uses_cached_model():
    box = type("Box", (), {"cls": np.asarray([4]), "xyxy": np.asarray([[10, 20, 30, 40]]), "conf": np.asarray([0.8])})()
    model = type("Model", (), {"names": {4: "Blue"}, "predict": lambda self, **kw: [type("Result", (), {"boxes": [box], "names": self.names})()]})()
    yolo._get_model.cache_clear()
    with patch.object(yolo, "_create_yolo_model", return_value=model) as create:
        result = yolo.detect_color(np.zeros((100, 100, 3), dtype=np.uint8))
        yolo.detect_color(np.zeros((100, 100, 3), dtype=np.uint8))
    assert result["detections"][0]["type"] == 4
    assert create.call_count == 1


def test_advanced_tracking_reports_measurement_and_support_count_and_resets():
    frame = np.zeros((100, 200, 3), dtype=np.uint8)
    advance_yolo.reset_advance_tracking()
    with patch.object(advance_yolo, "detect_circle", side_effect=[{"detections": [detection(1, 30, 40)]}, {"detections": [detection(1, 32, 40)]}, {"detections": []}]):
        assert advance_yolo.advance_detect_circle(frame) == {"detections": []}
        measured = advance_yolo.advance_detect_circle(frame)["detections"][0]
        predicted = advance_yolo.advance_detect_circle(frame)["detections"][0]
    assert measured["measured"] and measured["support_count"] == 2
    assert not predicted["measured"] and predicted["support_count"] == 2
    advance_yolo.reset_advance_tracking()
    assert advance_yolo._TRACKERS == {"color": {}, "circle": {}}


def test_disk_center_zero_support_is_not_image_center():
    result = detect_disk_center(np.zeros((400, 640, 3), dtype=np.uint8), {"detections": []})
    assert result == {"center": [0, 0], "status": 0, "support_count": 0, "measured_count": 0, "support_points": []}


def test_advanced_disk_center_returns_measured_count():
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    with patch.object(advance_yolo, "advance_detect_color", return_value={"detections": [dict(detection(1, 20, 30), measured=True), dict(detection(2, 50, 30), measured=False)]}):
        result = advance_detect_disk_center(frame)
    assert result["support_count"] == 2 and result["measured_count"] == 1
