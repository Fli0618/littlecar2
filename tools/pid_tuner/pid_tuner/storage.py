"""Local PID profile, telemetry CSV and C-default export helpers."""

from __future__ import annotations

import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import re

from .models import PidConfig, Telemetry
from .protocol import telemetry_csv_row

DEFAULT_PROFILES_DIR = Path(__file__).resolve().parents[1] / "profiles"
DEFAULT_LOGS_DIR = Path(__file__).resolve().parents[1] / "logs"
_PROFILE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")


def profile_path(name: str, directory: Path = DEFAULT_PROFILES_DIR) -> Path:
    if not _PROFILE_NAME.fullmatch(name):
        raise ValueError("profile name must use letters, digits, _ or -, up to 64 characters")
    return directory / f"{name}.json"


def save_profile(name: str, pid: PidConfig, note: str = "", firmware_revision: int | None = None,
                 directory: Path = DEFAULT_PROFILES_DIR) -> Path:
    path = profile_path(name, directory)
    path.parent.mkdir(parents=True, exist_ok=True)
    document = {
        "name": name,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "firmware_revision": firmware_revision,
        "note": note,
        "pid": pid.to_dict(),
    }
    path.write_text(json.dumps(document, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path


def load_profile(name: str, directory: Path = DEFAULT_PROFILES_DIR) -> tuple[PidConfig, dict[str, object]]:
    path = profile_path(name, directory)
    document = json.loads(path.read_text(encoding="utf-8"))
    try:
        pid = PidConfig(**document["pid"])
    except (KeyError, TypeError) as error:
        raise ValueError(f"invalid PID profile: {path}") from error
    return pid, document


def list_profiles(directory: Path = DEFAULT_PROFILES_DIR) -> list[str]:
    if not directory.exists():
        return []
    return sorted(path.stem for path in directory.glob("*.json") if _PROFILE_NAME.fullmatch(path.stem))


def export_c_defaults(pid: PidConfig) -> str:
    values = (
        ("ADVANCE_MOTION_DEFAULT_KP_POS", pid.kp_pos),
        ("ADVANCE_MOTION_DEFAULT_KI_POS", pid.ki_pos),
        ("ADVANCE_MOTION_DEFAULT_KD_POS", pid.kd_pos),
        ("ADVANCE_MOTION_DEFAULT_KP_YAW", pid.kp_yaw),
        ("ADVANCE_MOTION_DEFAULT_KI_YAW", pid.ki_yaw),
        ("ADVANCE_MOTION_DEFAULT_KD_YAW", pid.kd_yaw),
    )
    return "\n".join(f"#define {name} ({value:.9g}f)" for name, value in values) + "\n"


def write_telemetry_csv(path: Path, telemetry: list[Telemetry]) -> None:
    header = [
        "tick", "pid_revision", "telemetry_overwritten", "state", "flags",
        "target_x_mm", "target_y_mm", "target_yaw_deg",
        "actual_x_mm", "actual_y_mm", "actual_yaw_deg",
        "error_x_mm", "error_y_mm", "error_yaw_deg",
        "command_vx_mm_s", "command_vy_mm_s", "command_wz_deg_s",
        "measured_vx_mm_s", "measured_vy_mm_s", "measured_wz_deg_s",
        "integral_x_mm_s", "integral_y_mm_s", "integral_yaw_deg_s",
        "wit_yaw_deg", "ops_yaw_deg",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(telemetry_csv_row(item) for item in telemetry)
