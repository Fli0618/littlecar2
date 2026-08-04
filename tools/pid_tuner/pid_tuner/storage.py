"""Local PID profile, telemetry CSV and C-default export helpers."""

from __future__ import annotations

import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re

from .models import GotoStrategySnapshot, PathConfigState, PidConfig, PidConfigState, Telemetry
from .protocol import telemetry_csv_row

DEFAULT_PROFILES_DIR = Path(__file__).resolve().parents[1] / "profiles"
DEFAULT_LOGS_DIR = Path(__file__).resolve().parents[1] / "logs"
_PROFILE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")

PID_C_MACROS = (
    ("ADVANCE_MOTION_DEFAULT_KP_POS", "kp_pos"),
    ("ADVANCE_MOTION_DEFAULT_KI_POS", "ki_pos"),
    ("ADVANCE_MOTION_DEFAULT_KD_POS", "kd_pos"),
    ("ADVANCE_MOTION_DEFAULT_KP_YAW", "kp_yaw"),
    ("ADVANCE_MOTION_DEFAULT_KI_YAW", "ki_yaw"),
    ("ADVANCE_MOTION_DEFAULT_KD_YAW", "kd_yaw"),
)

PATH_C_MACROS = (
    ("ADVANCE_MOTION_PATH_KP_POS", "kp_cross_track"),
    ("ADVANCE_MOTION_PATH_KD_VEL", "kd_cross_track_velocity"),
    ("ADVANCE_MOTION_PATH_KP_YAW", "kp_yaw"),
    ("ADVANCE_MOTION_PATH_KD_YAW", "kd_yaw_rate"),
    ("ADVANCE_MOTION_PATH_CRUISE_SPEED_MM_S", "cruise_speed_mm_s"),
    ("ADVANCE_MOTION_PATH_MAX_WZ_DEG_S", "max_yaw_rate_deg_s"),
    ("ADVANCE_MOTION_PATH_ACCEL_MM_S2", "accel_mm_s2"),
    ("ADVANCE_MOTION_PATH_DECEL_MM_S2", "decel_mm_s2"),
    ("ADVANCE_MOTION_PATH_MAX_LATERAL_ACC_MM_S2", "max_lateral_accel_mm_s2"),
    ("ADVANCE_MOTION_PATH_CURVATURE_PREVIEW_MM", "curvature_preview_mm"),
    ("ADVANCE_MOTION_PATH_CURVATURE_FF_TIME_S", "curvature_ff_time_s"),
    ("ADVANCE_MOTION_PATH_LOOKAHEAD_MIN_MM", "lookahead_min_mm"),
    ("ADVANCE_MOTION_PATH_LOOKAHEAD_BASE_MM", "lookahead_base_mm"),
    ("ADVANCE_MOTION_PATH_LOOKAHEAD_SPEED_GAIN_S", "lookahead_speed_gain_s"),
    ("ADVANCE_MOTION_PATH_LOOKAHEAD_CURVE_GAIN_MM", "lookahead_curve_gain_mm"),
    ("ADVANCE_MOTION_PATH_LOOKAHEAD_MAX_MM", "lookahead_max_mm"),
    ("ADVANCE_MOTION_PATH_LOOKAHEAD_RATE_MM_S", "lookahead_rate_mm_s"),
    ("ADVANCE_MOTION_PATH_INITIAL_LOOKAHEAD_MM", "initial_lookahead_mm"),
    ("ADVANCE_MOTION_PATH_FINAL_CAPTURE_DISTANCE_MM", "final_capture_distance_mm"),
    ("ADVANCE_MOTION_PATH_FINAL_CAPTURE_SPEED_MM_S", "final_capture_speed_mm_s"),
)


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


def _format_c_float(value: float) -> str:
    """Return one finite float32-compatible C floating literal."""
    if not math.isfinite(value):
        raise ValueError("motion configuration values must be finite")
    if value == 0.0:
        return "0.0f"
    text = format(value, ".9g")
    if "e" not in text and "." not in text:
        text += ".0"
    return f"{text}f"


def _macro_lines(config: object, mappings: tuple[tuple[str, str], ...]) -> list[str]:
    return [f"#define {macro} ({_format_c_float(getattr(config, field))})"
            for macro, field in mappings]


def export_motion_config_header(
    pid_state: PidConfigState,
    path_state: PathConfigState,
    goto_strategy: GotoStrategySnapshot,
) -> str:
    """Export board-confirmed motion values as a replaceable C configuration header."""
    strategy_value = "1U" if goto_strategy.large_yaw_align_enabled else "0U"
    lines = [
        "#ifndef __ADVANCE_MOTION_CONFIG_H__",
        "#define __ADVANCE_MOTION_CONFIG_H__",
        "",
        "#include <stdint.h>",
        "",
        "/*",
        " * 由 motion_workbench 从 STM32 当前活动参数导出。",
        " *",
        f" * PID revision: {pid_state.revision}",
        f" * Path revision: {path_state.revision}",
        " *",
        " * 将本文件复制到 Core/Inc/advance_motion_config.h，",
        " * 然后重新编译并烧录 STM32。",
        " */",
        "",
        "#define ADVANCE_MOTION_CONFIG_SCHEMA_VERSION ((uint32_t)1U)",
        "",
        "/* 单点位姿 PID。 */",
        *_macro_lines(pid_state.config, PID_C_MACROS),
        "",
        "/* 连续路径横向与航向 PD。 */",
        *_macro_lines(path_state.config, PATH_C_MACROS[:4]),
        "",
        "/* 连续路径速度规划。 */",
        *_macro_lines(path_state.config, PATH_C_MACROS[4:9]),
        "",
        "/* 曲率预览与法向前馈。 */",
        *_macro_lines(path_state.config, PATH_C_MACROS[9:11]),
        "",
        "/* 动态前视。 */",
        *_macro_lines(path_state.config, PATH_C_MACROS[11:18]),
        "",
        "/* 末段捕获。 */",
        *_macro_lines(path_state.config, PATH_C_MACROS[18:]),
        "",
        "/* 组合 GOTO 默认策略。 */",
        f"#define ADVANCE_MOTION_DEFAULT_LARGE_YAW_ALIGN_ENABLE ((uint8_t){strategy_value})",
        "",
        "#endif",
        "",
    ]
    return "\n".join(lines)


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
