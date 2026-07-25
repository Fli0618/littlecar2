import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from vision import detect_circle, detect_color, detect_disk_center, detect_qr
from vision import yolo
import vision.advance_yolo as advance_yolo
from vision.materials import advance_detect_disk_center


class FakeBox:
    def __init__(self, class_id, xyxy, confidence):
        self.cls = np.asarray([class_id])
        self.xyxy = np.asarray([xyxy])
        self.conf = np.asarray([confidence])


class FakeModel:
    def __init__(self, names, boxes):
        self.names = names
        self.boxes = boxes
        self.predict_calls = []

    def predict(self, **kwargs):
        self.predict_calls.append(kwargs)
        return [type("Result", (), {"boxes": self.boxes, "names": self.names})()]


def detection(kind, x, y, confidence=0.9):
    return {"type": kind, "center": [x, y], "confidence": confidence}


class VisionTest(unittest.TestCase):
    def setUp(self):
        yolo._get_model.cache_clear()
        advance_yolo._reset_advance_tracking()

    def tearDown(self):
        yolo._get_model.cache_clear()
        advance_yolo._reset_advance_tracking()

    def test_color_and_circle_return_unified_multi_target_structure(self):
        color_model = FakeModel(
            {0: "Red", 6: "EmptySlot"},
            [FakeBox(0, [10, 20, 30, 40], 0.95), FakeBox(6, [100, 200, 140, 260], 0.8)],
        )
        circle_model = FakeModel({0: "1"}, [FakeBox(0, [40, 60, 80, 100], 0.88)])

        with patch.object(yolo, "_create_yolo_model", side_effect=[color_model, circle_model]):
            color_result = detect_color(np.zeros((300, 400, 3), dtype=np.uint8))
            circle_result = detect_circle(np.zeros((300, 400, 3), dtype=np.uint8))

        self.assertEqual(
            color_result,
            {
                "detections": [
                    {"type": "Red", "center": [20, 30], "confidence": 0.95},
                    {"type": "EmptySlot", "center": [120, 230], "confidence": 0.8},
                ]
            },
        )
        self.assertEqual(circle_result, {"detections": [{"type": "1", "center": [60, 80], "confidence": 0.88}]})

    def test_each_model_is_loaded_once_and_empty_detection_is_supported(self):
        color_model = FakeModel({0: "Red"}, None)
        circle_model = FakeModel({0: "1"}, None)
        frame = np.zeros((300, 400, 3), dtype=np.uint8)

        with patch.object(yolo, "_create_yolo_model", side_effect=[color_model, circle_model]) as create_model:
            self.assertEqual(detect_color(frame), {"detections": []})
            self.assertEqual(detect_color(frame), {"detections": []})
            self.assertEqual(detect_circle(frame), {"detections": []})
            self.assertEqual(detect_circle(frame), {"detections": []})

        self.assertEqual(create_model.call_count, 2)
        self.assertEqual(len(color_model.predict_calls), 2)
        self.assertEqual(len(circle_model.predict_calls), 2)

    def test_detection_rejects_non_bgr_frame(self):
        with self.assertRaises(TypeError):
            detect_color(np.zeros((10, 10), dtype=np.uint8))

    def test_disk_center_uses_top_three_targets_including_empty_slot(self):
        frame = np.zeros((400, 640, 3), dtype=np.uint8)
        color_result = {
            "detections": [
                detection("Red", 100, 100, 0.7),
                detection("EmptySlot", 150, 200, 0.99),
                detection("Blue", 200, 100, 0.8),
                detection("Green", 500, 500, 0.1),
            ]
        }

        result = detect_disk_center(frame, color_result)

        self.assertEqual(result, {"center": [150, 133], "status": 3, "support_points": [[150, 200], [200, 100], [100, 100]]})

    def test_disk_center_two_point_geometry_chooses_nearest_candidate(self):
        result = detect_disk_center(
            np.zeros((400, 640, 3), dtype=np.uint8),
            {"detections": [detection("Red", 200, 200), detection("Blue", 400, 200)]},
        )

        self.assertEqual(result["status"], 2)
        self.assertEqual(result["center"], [300, 142])

    def test_disk_center_single_and_zero_point_fallbacks(self):
        frame = np.zeros((400, 640, 3), dtype=np.uint8)

        self.assertEqual(
            detect_disk_center(frame, {"detections": [detection("EmptySlot", 123, 234)]}),
            {"center": [123, 234], "status": 1, "support_points": [[123, 234]]},
        )
        self.assertEqual(
            detect_disk_center(frame, {"detections": []}),
            {"center": [320, 200], "status": 0, "support_points": []},
        )

    def test_disk_center_validates_result_shape(self):
        with self.assertRaises(ValueError):
            detect_disk_center(np.zeros((10, 10, 3), dtype=np.uint8), {})

    def test_advance_circle_requires_multiple_frames_and_predicts_short_misses(self):
        frame = np.zeros((360, 640, 3), dtype=np.uint8)
        detection_frames = [
            {"detections": [detection("1", 100, 100)]},
            {"detections": [detection("1", 104, 100)]},
            {"detections": []},
            {"detections": []},
            {"detections": []},
            {"detections": []},
        ]

        with patch.object(advance_yolo, "detect_circle", side_effect=detection_frames):
            self.assertEqual(advance_yolo.advance_detect_circle(frame), {"detections": []})
            confirmed = advance_yolo.advance_detect_circle(frame)
            predicted = advance_yolo.advance_detect_circle(frame)
            advance_yolo.advance_detect_circle(frame)
            advance_yolo.advance_detect_circle(frame)
            expired = advance_yolo.advance_detect_circle(frame)

        self.assertEqual(confirmed["detections"][0]["tracking_id"], 0)
        self.assertEqual(confirmed["detections"][0]["type"], "1")
        self.assertTrue(confirmed["detections"][0]["measured"])
        self.assertFalse(predicted["detections"][0]["measured"])
        self.assertEqual(expired, {"detections": []})

    def test_advance_circle_tracks_same_type_targets_separately(self):
        frame = np.zeros((360, 640, 3), dtype=np.uint8)
        raw_result = {
            "detections": [
                detection("1", 100, 100, 0.9),
                detection("1", 400, 100, 0.8),
            ]
        }

        with patch.object(advance_yolo, "detect_circle", side_effect=[raw_result, raw_result]):
            advance_yolo.advance_detect_circle(frame)
            result = advance_yolo.advance_detect_circle(frame)

        self.assertEqual([item["tracking_id"] for item in result["detections"]], [0, 1])

    def test_advance_disk_center_uses_advanced_color_detection(self):
        frame = np.zeros((400, 640, 3), dtype=np.uint8)
        raw_result = {"detections": [detection("Red", 123, 234)]}

        with patch.object(advance_yolo, "detect_color", side_effect=[raw_result, raw_result]):
            self.assertEqual(advance_detect_disk_center(frame)["status"], 0)
            result = advance_detect_disk_center(frame)

        self.assertEqual(result["status"], 1)
        self.assertEqual(result["center"], [123, 234])

    def test_qr_without_code_returns_none(self):
        frame = np.full((360, 640, 3), 127, dtype=np.uint8)
        self.assertIsNone(detect_qr(frame))

    def test_qr_returns_task_code(self):
        image_path = ROOT / "assets" / "二维码" / "145+634+312+132.png"
        image = cv2.imdecode(np.fromfile(str(image_path), dtype=np.uint8), cv2.IMREAD_COLOR)
        self.assertIsNotNone(image)
        self.assertEqual(detect_qr(image), "145+634+312+132")

    @unittest.skipUnless(os.environ.get("RUN_YOLO_SMOKE") == "1", "set RUN_YOLO_SMOKE=1 to run model smoke test")
    def test_color_model_smoke_test(self):
        image_path = next((ROOT / "assets" / "物料盘").glob("*.jpg"))
        image = cv2.imdecode(np.fromfile(str(image_path), dtype=np.uint8), cv2.IMREAD_COLOR)
        result = detect_color(image)
        self.assertIn("detections", result)

    @unittest.skipUnless(os.environ.get("RUN_YOLO_SMOKE") == "1", "set RUN_YOLO_SMOKE=1 to run model smoke test")
    def test_circle_model_smoke_test(self):
        image_path = next((ROOT / "assets" / "circle_with_number").glob("*.jpg"))
        image = cv2.imdecode(np.fromfile(str(image_path), dtype=np.uint8), cv2.IMREAD_COLOR)
        result = detect_circle(image)
        self.assertIn("detections", result)


if __name__ == "__main__":
    unittest.main()
