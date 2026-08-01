from unittest.mock import patch

import numpy as np
import pytest

import vision.advance_yolo as advance_yolo
import vision.qr as qr
from vision import yolo
from vision.materials import advance_detect_disk_center, detect_disk_center


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


def test_advance_qr_confirms_repeats_disappears_and_reappears():
    frame = np.zeros((20, 20, 3), dtype=np.uint8)
    code = "156+123+516+231"
    qr.reset_qr_tracking()
    with patch.object(qr, "detect_qr", side_effect=[code, code, code, code, None, None, None, None, None, code, code, code]):
        results = [qr.advance_detect_qr(frame) for _ in range(12)]
    assert [item["status"] for item in results[:4]] == ["CONFIRMING", "CONFIRMING", "FIRST_DETECTED", "REPEATED"]
    assert results[2]["code"] == code and results[3]["code"] is None
    assert [item["status"] for item in results[4:9]] == ["MISSING", "MISSING", "MISSING", "MISSING", "DISAPPEARED"]
    assert [item["status"] for item in results[9:]] == ["CONFIRMING", "CONFIRMING", "REAPPEARED"]
    assert results[-1]["code"] == code


def test_advance_qr_rejects_invalid_and_counts_it_as_missing():
    frame = np.zeros((20, 20, 3), dtype=np.uint8)
    code = "156+123+516+231"
    qr.reset_qr_tracking()
    with patch.object(qr, "detect_qr", side_effect=["bad", code, code, code, "invalid", "invalid", "invalid", "invalid", "invalid"]):
        results = [qr.advance_detect_qr(frame) for _ in range(9)]
    assert results[0] == {"raw_code": "bad", "code": None, "status": "INVALID"}
    assert results[3]["status"] == "FIRST_DETECTED"
    assert [item["status"] for item in results[4:8]] == ["INVALID"] * 4
    assert results[8]["status"] == "DISAPPEARED"


def test_advance_qr_changes_code_and_reset_clears_latch():
    frame = np.zeros((20, 20, 3), dtype=np.uint8)
    first = "156+123+516+231"
    second = "111+222+333+444"
    qr.reset_qr_tracking()
    with patch.object(qr, "detect_qr", side_effect=[first, first, first, second, second, second]):
        results = [qr.advance_detect_qr(frame) for _ in range(6)]
    assert results[2]["status"] == "FIRST_DETECTED"
    assert results[-1] == {"raw_code": second, "code": second, "status": "CHANGED"}
    qr.reset_qr_tracking()
    with patch.object(qr, "detect_qr", return_value=first):
        results = [qr.advance_detect_qr(frame) for _ in range(3)]
    assert results[-1]["status"] == "FIRST_DETECTED"


def test_model_backend_rejects_unknown_values():
    try:
        yolo.configure_model_backend("pt")
        with pytest.raises(ValueError):
            yolo.configure_model_backend("onnx")
    finally:
        yolo.configure_model_backend("engine")


def test_default_backend_is_engine():
    assert yolo.get_model_backend() == "engine"


def test_engine_backend_requires_both_engine_files(tmp_path, monkeypatch):
    engine_paths = {
        "color": tmp_path / "color.engine",
        "circle": tmp_path / "circle.engine",
    }
    monkeypatch.setitem(yolo._MODEL_PATHS, "engine", engine_paths)
    try:
        yolo.configure_model_backend("pt")
        with pytest.raises(FileNotFoundError):
            yolo.configure_model_backend("engine")

        engine_paths["color"].touch()
        engine_paths["circle"].touch()
        yolo.configure_model_backend("engine")
        assert yolo.get_model_backend() == "engine"
    finally:
        yolo.configure_model_backend("pt")


def test_switching_backend_clears_model_cache(tmp_path, monkeypatch):
    pt_path = tmp_path / "model.pt"
    engine_path = tmp_path / "model.engine"
    pt_path.touch()
    engine_path.touch()
    paths = {"color": pt_path, "circle": pt_path}
    monkeypatch.setitem(yolo._MODEL_PATHS, "pt", paths)
    monkeypatch.setitem(yolo._MODEL_PATHS, "engine", {"color": engine_path, "circle": engine_path})
    fake_model = object()
    with patch.object(yolo, "_create_yolo_model", return_value=fake_model):
        yolo.configure_model_backend("pt")
        yolo._get_model(pt_path)
        assert yolo._get_model.cache_info().currsize == 1
        yolo.configure_model_backend("engine")
        assert yolo._get_model.cache_info().currsize == 0
    yolo.configure_model_backend("engine")


def test_pt_cma_allocation_error_has_engine_recovery_hint(tmp_path, monkeypatch):
    model_path = tmp_path / "model.pt"
    model_path.touch()
    monkeypatch.setitem(yolo._MODEL_PATHS, "pt", {"color": model_path, "circle": model_path})
    model = type("Model", (), {"predict": lambda self, **kw: (_ for _ in ()).throw(RuntimeError("CUBLAS_STATUS_ALLOC_FAILED"))})()
    try:
        yolo.configure_model_backend("pt")
        with patch.object(yolo, "_create_yolo_model", return_value=model), pytest.raises(RuntimeError, match="configure_model_backend\\('engine'\\)"):
            yolo.detect_color(np.zeros((100, 100, 3), dtype=np.uint8))
    finally:
        yolo.configure_model_backend("engine")
