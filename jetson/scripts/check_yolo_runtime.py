"""Read-only TensorRT/YOLO runtime check for the deployed Jetson service."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

from vision import configure_model_backend, detect_circle, detect_color, get_model_backend, validate_engine_runtime


PROJECT_ROOT = Path(__file__).resolve().parents[1]
WARMUP_SIZE = 640


def _meminfo_value(name: str) -> str:
    try:
        for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
            if line.startswith(f"{name}:"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return "unavailable"


def main() -> None:
    import torch

    print(f"python={sys.executable}")
    print(f"torch={torch.__version__} torch_cuda={torch.version.cuda} cuda_available={torch.cuda.is_available()}")
    print(f"tensorrt={validate_engine_runtime()}")
    print(f"cma_free={_meminfo_value('CmaFree')} mem_available={_meminfo_value('MemAvailable')}")

    configure_model_backend("engine")
    print(f"backend={get_model_backend()}")
    frame = np.zeros((WARMUP_SIZE, WARMUP_SIZE, 3), dtype=np.uint8)
    for name, detector in (("color", detect_color), ("circle", detect_circle)):
        result = detector(frame)
        print(f"{name}_engine_warmup=PASS detections={len(result['detections'])}")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        raise SystemExit(
            f"YOLO TensorRT runtime check failed: {exc}\n"
            "If this follows NvMap/CMA errors, close desktop or remote graphical applications using /dev/nvmap, "
            "then retry; reboot the Jetson if contiguous memory is not recovered."
        ) from exc
