"""YOLO 候选框与 HSV 颜色分类的混合检测链路。"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

from .hsv_color import HSVConfig, HSVClassification, classify_bbox_hsv, load_hsv_config
from .yolo import detect_color


EMPTY_SLOT_TYPE = 6
HSV_UNCERTAIN_POLICY = "reject"
_VALID_POLICIES = {"reject", "keep_yolo"}
DEFAULT_HSV_CONFIG_PATH = Path(__file__).resolve().parents[2] / "assets" / "config" / "hsv_colors.json"


@lru_cache(maxsize=1)
def get_hsv_config(path: Optional[Path] = None) -> HSVConfig:
    """加载并缓存正式 HSV 配置；文件不存在或非法时直接抛出异常。"""
    config_path = DEFAULT_HSV_CONFIG_PATH if path is None else Path(path)
    return load_hsv_config(config_path)


def _classification_fields(classification: HSVClassification) -> Dict[str, Any]:
    return {
        "hsv_color": classification.color_name,
        "hsv_coverage": classification.coverage,
        "hsv_purity": classification.purity,
        "hsv_margin": classification.margin,
    }


def _base_debug_detection(detection: Dict[str, Any]) -> Dict[str, Any]:
    result = dict(detection)
    result["yolo_type"] = detection.get("type")
    result["yolo_confidence"] = detection.get("confidence")
    return result


def refine_yolo_detection_with_hsv(
    frame_bgr: np.ndarray,
    detection: Dict[str, Any],
    config: HSVConfig,
    *,
    uncertain_policy: str = HSV_UNCERTAIN_POLICY,
) -> Optional[Dict[str, Any]]:
    """使用 HSV 细化单个 YOLO detection，不改变其位置字段。"""
    if uncertain_policy not in _VALID_POLICIES:
        raise ValueError(f"unsupported HSV uncertain policy: {uncertain_policy!r}")
    if not isinstance(detection, dict):
        raise TypeError("detection must be a dictionary")

    result = _base_debug_detection(detection)
    yolo_type = detection.get("type")
    yolo_confidence = detection.get("confidence")
    if yolo_type == EMPTY_SLOT_TYPE:
        result.update(
            type=EMPTY_SLOT_TYPE,
            confidence=yolo_confidence,
            classification_source="yolo_empty_slot",
            **{"hsv_color": None, "hsv_coverage": 0.0, "hsv_purity": 0.0, "hsv_margin": 0.0},
        )
        return result

    bbox = detection.get("bbox")
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
        if uncertain_policy == "keep_yolo":
            result["classification_source"] = "yolo_fallback"
            return result
        return None

    classification = classify_bbox_hsv(frame_bgr, bbox, config)
    result.update(_classification_fields(classification))
    if classification.type_id is None:
        if uncertain_policy == "keep_yolo":
            result["type"] = yolo_type
            result["confidence"] = yolo_confidence
            result["classification_source"] = "yolo_fallback"
            return result
        return None

    result.update(
        type=classification.type_id,
        confidence=classification.confidence,
        classification_source="hsv",
    )
    return result


def detect_color_hybrid(
    frame_bgr: np.ndarray,
    *,
    config: Optional[HSVConfig] = None,
    uncertain_policy: str = HSV_UNCERTAIN_POLICY,
) -> Dict[str, list[dict[str, Any]]]:
    """YOLO 负责定位，HSV 负责 bbox 内颜色分类。"""
    active_config = get_hsv_config() if config is None else config
    raw_result = detect_color(frame_bgr)
    raw_detections = raw_result.get("detections", [])
    if not isinstance(raw_detections, list):
        return {"detections": []}
    refined: list[dict[str, Any]] = []
    for detection in raw_detections:
        result = refine_yolo_detection_with_hsv(
            frame_bgr,
            detection,
            active_config,
            uncertain_policy=uncertain_policy,
        )
        if result is not None:
            refined.append(result)
    return {"detections": refined}
