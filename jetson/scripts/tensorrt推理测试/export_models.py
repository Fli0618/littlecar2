"""Export the production YOLO weights to Jetson-compatible TensorRT engines."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys

# TensorRT is installed by JetPack for the system Python, while the project
# uses a Conda environment. Make the JetPack binding visible to this script.
SYSTEM_DIST_PACKAGES = Path(f"/usr/lib/python{sys.version_info.major}.{sys.version_info.minor}/dist-packages")
if SYSTEM_DIST_PACKAGES.is_dir() and str(SYSTEM_DIST_PACKAGES) not in sys.path:
    sys.path.append(str(SYSTEM_DIST_PACKAGES))
os.environ.setdefault("YOLO_AUTOINSTALL", "false")

from ultralytics import YOLO


PROJECT_ROOT = Path(__file__).resolve().parents[2]
MODEL_DIR = PROJECT_ROOT / "assets" / "models"
EXPORT_SPECS = (
    MODEL_DIR / "6color-circle-v3.pt",
    MODEL_DIR / "circle-with-number-v3.pt",
)
IMAGE_SIZE = 640
BATCH_SIZE = 1
WORKSPACE_MIB = 256
TRTEXEC = Path("/usr/src/tensorrt/bin/trtexec")


def export_model(model_path: Path) -> Path:
    onnx_path = model_path.with_suffix(".onnx")
    engine_path = model_path.with_suffix(".engine")
    if not TRTEXEC.is_file():
        raise FileNotFoundError(f"TensorRT trtexec was not found: {TRTEXEC}")

    print(f"Exporting {model_path.name} -> {onnx_path.name}")
    exported_path = Path(
        YOLO(str(model_path)).export(
            format="onnx",
            imgsz=IMAGE_SIZE,
            batch=BATCH_SIZE,
            half=True,
            device=0,
            dynamic=False,
        )
    )
    if exported_path.resolve() != onnx_path.resolve():
        exported_path.replace(onnx_path)

    print(f"Building {engine_path.name} with {TRTEXEC.name}")
    subprocess.run(
        [
            str(TRTEXEC),
            f"--onnx={onnx_path}",
            f"--saveEngine={engine_path}",
            "--fp16",
            f"--memPoolSize=workspace:{WORKSPACE_MIB}",
            "--builderOptimizationLevel=0",
            "--avgTiming=1",
            "--skipInference",
        ],
        check=True,
        cwd=PROJECT_ROOT,
    )
    onnx_path.unlink()
    print(f"Created {engine_path} ({engine_path.stat().st_size / 1024 / 1024:.2f} MiB)")
    if engine_path.stat().st_size < 10 * 1024 * 1024:
        print("  Size is below 10 MiB; it may be committed to Git.")
    else:
        print("  Size is at least 10 MiB; keep it local and do not commit it.")
    return engine_path


def main() -> None:
    missing = [path for path in EXPORT_SPECS if not path.is_file()]
    if missing:
        raise FileNotFoundError("Missing PT model files: " + ", ".join(map(str, missing)))
    for model_path in EXPORT_SPECS:
        export_model(model_path)


if __name__ == "__main__":
    main()
