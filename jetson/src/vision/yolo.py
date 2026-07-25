"""面向业务的 YOLO 视觉接口与共享推理实现。"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Any

import numpy as np


_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_COLOR_MODEL_PATH = _PROJECT_ROOT / "assets" / "models" / "6color-circle-v3.pt"
_CIRCLE_MODEL_PATH = _PROJECT_ROOT / "assets" / "models" / "circle-with-number-v3.pt"
_CONFIDENCE_THRESHOLD = 0.5
_IOU_THRESHOLD = 0.45


def _create_yolo_model(model_path: Path) -> Any:
    """构造一个 YOLO 模型，供缓存加载函数调用。"""
    from ultralytics import YOLO

    return YOLO(str(model_path))


@lru_cache(maxsize=2)
def _get_model(model_path: Path) -> Any:
    """按权重路径延迟加载模型；当前进程内同一权重只加载一次。"""
    if not model_path.is_file():
        raise FileNotFoundError(f"YOLO model not found: {model_path}")
    return _create_yolo_model(model_path)


def _validate_frame(frame_bgr: np.ndarray) -> None:
    if not isinstance(frame_bgr, np.ndarray) or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
        raise TypeError("frame_bgr must be a BGR numpy.ndarray")


def _class_name(names: Any, class_id: int) -> str:
    if isinstance(names, dict):
        return str(names.get(class_id, class_id))
    if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
        return str(names[class_id])
    return str(class_id)


def _to_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)


def _detect(frame_bgr: np.ndarray, model_path: Path) -> dict[str, list[dict[str, Any]]]:
    _validate_frame(frame_bgr)
    model = _get_model(model_path)
    result = model.predict(
        source=frame_bgr,
        conf=_CONFIDENCE_THRESHOLD,
        iou=_IOU_THRESHOLD,
        verbose=False,
    )[0]
    if result.boxes is None:
        return {"detections": []}

    names = getattr(result, "names", model.names)
    detections: list[dict[str, Any]] = []
    for box in result.boxes:
        class_id = int(_to_numpy(box.cls).reshape(-1)[0])
        x1, y1, x2, y2 = _to_numpy(box.xyxy).reshape(-1)[:4].astype(int)
        detections.append(
            {
                "type": _class_name(names, class_id),
                "center": [int((x1 + x2) / 2), int((y1 + y2) / 2)],
                "confidence": float(_to_numpy(box.conf).reshape(-1)[0]),
            }
        )
    return {"detections": detections}


def detect_color(frame_bgr: np.ndarray) -> dict[str, list[dict[str, Any]]]:
    """识别彩色物料和 EmptySlot，返回类型、中心点及置信度。"""
    return _detect(frame_bgr, _COLOR_MODEL_PATH)


def detect_circle(frame_bgr: np.ndarray) -> dict[str, list[dict[str, Any]]]:
    """识别带数字的同心圆，返回类型、中心点及置信度。"""
    return _detect(frame_bgr, _CIRCLE_MODEL_PATH)


def load_yolo_model(model_path: str | Path) -> Any:
    """加载指定 YOLO 模型，供历史研究脚本使用。"""
    return _create_yolo_model(Path(model_path))


def detect_yolo(
    frame_bgr: np.ndarray,
    model: Any,
    conf_thres: float = _CONFIDENCE_THRESHOLD,
    iou_thres: float = _IOU_THRESHOLD,
    device: str | None = None,
) -> list[dict[str, Any]]:
    """低层 YOLO 推理兼容接口，供历史研究脚本使用。"""
    _validate_frame(frame_bgr)
    if not 0.0 <= conf_thres <= 1.0 or not 0.0 <= iou_thres <= 1.0:
        raise ValueError("conf_thres and iou_thres must be in [0, 1]")

    result = model.predict(source=frame_bgr, conf=conf_thres, iou=iou_thres, device=device, verbose=False)[0]
    if result.boxes is None:
        return []

    names = getattr(result, "names", model.names)
    detections: list[dict[str, Any]] = []
    for box in result.boxes:
        class_id = int(_to_numpy(box.cls).reshape(-1)[0])
        x1, y1, x2, y2 = _to_numpy(box.xyxy).reshape(-1)[:4].astype(int)
        detections.append(
            {
                "class_id": class_id,
                "class_name": _class_name(names, class_id),
                "confidence": float(_to_numpy(box.conf).reshape(-1)[0]),
                "x1": int(x1),
                "y1": int(y1),
                "x2": int(x2),
                "y2": int(y2),
                "center_x": int((x1 + x2) / 2),
                "center_y": int((y1 + y2) / 2),
            }
        )
    return detections
