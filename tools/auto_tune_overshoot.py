#!/usr/bin/env python3
"""Multi-round automated tuning script using CONTINUOUS STEERING MODE (Interpolate Mode)."""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

# Add pid_tuner to sys.path
sys.path.insert(0, str(Path(__file__).parent / "pid_tuner"))

from pid_tuner.models import PathConfigSnapshot, PathPointSnapshot, PathStartCommand
from pid_tuner.protocol import build_path_upload
from pid_tuner.serial_client import SerialClient

PORT = "COM5"
BAUD = 115200

# Target position from 启停区1:
# Δ启1 X=-143 mm, Y=+1684 mm, Yaw=90.0°
TARGET_X = -143.0
TARGET_Y = 1684.0
TARGET_YAW = 90.0

ORIGIN_X = 0.0
ORIGIN_Y = 0.0
ORIGIN_YAW = 0.0

NUM_POINTS = 35


def generate_interpolate_path(start_x: float, start_y: float, start_yaw: float,
                               end_x: float, end_y: float, end_yaw: float,
                               count: int = NUM_POINTS) -> list[PathPointSnapshot]:
    """Generates continuous steering waypoints where both position and yaw interpolate smoothly."""
    waypoints = []
    for i in range(count):
        t = i / (count - 1)
        x = start_x + t * (end_x - start_x)
        y = start_y + t * (end_y - start_y)
        # Wrap yaw delta to shortest turn angle
        yaw_diff = (end_yaw - start_yaw + 180.0) % 360.0 - 180.0
        yaw = start_yaw + t * yaw_diff
        waypoints.append(PathPointSnapshot(x, y, yaw))
    return waypoints


FORWARD_WAYPOINTS = generate_interpolate_path(ORIGIN_X, ORIGIN_Y, ORIGIN_YAW, TARGET_X, TARGET_Y, TARGET_YAW)
RETURN_WAYPOINTS = generate_interpolate_path(TARGET_X, TARGET_Y, TARGET_YAW, ORIGIN_X, ORIGIN_Y, ORIGIN_YAW)


def upload_and_start_path(client: SerialClient, path_id: int, waypoints: list[PathPointSnapshot]) -> None:
    """Uploads continuous path waypoints and starts path execution."""
    begin, chunks, commit = build_path_upload(path_id, waypoints)
    client.path_begin(begin)
    for chunk in chunks:
        client.path_chunk(chunk)
    client.path_commit(commit)
    client.path_start(PathStartCommand(path_id))


def run_path_and_record(client: SerialClient, path_id: int, waypoints: list[PathPointSnapshot], monitor_duration_s: float = 12.0) -> tuple[float, float, float, list]:
    """Uploads path, monitors telemetry, and records max Y position and final yaw error."""
    telemetry_list = []

    def on_telemetry(t):
        telemetry_list.append(t)

    client.add_telemetry_callback(on_telemetry)
    try:
        upload_and_start_path(client, path_id, waypoints)
        target_point = waypoints[-1]
        start_time = time.monotonic()
        max_y = -99999.0
        final_yaw_err = 0.0
        
        while time.monotonic() - start_time < monitor_duration_s:
            time.sleep(0.05)
            if telemetry_list:
                latest = telemetry_list[-1]
                actual_x, actual_y, actual_yaw = latest.actual
                if actual_y > max_y: max_y = actual_y

                dist_to_goal = math.hypot(actual_x - target_point.x_mm, actual_y - target_point.y_mm)
                vx, vy, wz = latest.measured_velocity
                speed = math.hypot(vx, vy)
                final_yaw_err = abs((actual_yaw - target_point.yaw_deg + 180.0) % 360.0 - 180.0)

                if dist_to_goal < 40.0 and speed < 25.0 and (time.monotonic() - start_time > 2.5):
                    print(f"  --> Path arrived! (dist={dist_to_goal:.1f}mm, speed={speed:.1f}mm/s, yaw_err={final_yaw_err:.1f}°)")
                    break

        return target_point.x_mm, max_y, final_yaw_err, telemetry_list
    finally:
        pass


def execute_test_run(client: SerialClient, round_num: int, path_id_base: int, decel: float, capture_dist: float, capture_speed: float, lookahead_base: float) -> dict:
    """Updates path config over serial, resets origin, executes forward continuous steering, measures overshoot, then returns."""
    print(f"\n============================================================")
    print(f"  ROUND {round_num}: Setting Serial Params (decel={decel:.0f}, capture_dist={capture_dist:.0f}, capture_speed={capture_speed:.0f}, lookahead={lookahead_base:.0f})")
    print(f"============================================================")
    
    print("  [0/2] Resetting odometer origin at 启停区1 (0,0)...")
    client.reset_origin()
    time.sleep(0.3)

    # Modify target tuning parameters via serial
    current_state = client.get_path_config()
    cfg_dict = current_state.config.to_dict()
    cfg_dict["decel_mm_s2"] = decel
    cfg_dict["final_capture_distance_mm"] = capture_dist
    cfg_dict["final_capture_speed_mm_s"] = capture_speed
    cfg_dict["lookahead_base_mm"] = lookahead_base
    cfg_dict["lookahead_min_mm"] = min(lookahead_base, cfg_dict.get("lookahead_min_mm", 80.0))

    new_config = PathConfigSnapshot(**cfg_dict)
    client.set_path_config(new_config)
    time.sleep(0.2)

    # Run forward path with continuous steering (0° -> 90°)
    print(f"  [1/2] CONTINUOUS STEERING (0° -> 90°) to target (X={TARGET_X}, Y={TARGET_Y})...")
    _, max_y, yaw_err, _ = run_path_and_record(client, path_id_base, FORWARD_WAYPOINTS, monitor_duration_s=10.0)

    # Overshoot calculation along Y
    overshoot_y = max(0.0, max_y - TARGET_Y)
    print(f"  --> Max Y reached: {max_y:.1f} mm | Y Overshoot: {overshoot_y:.1f} mm | Final Yaw Error: {yaw_err:.1f}°")

    time.sleep(1.2)

    # Run return path with continuous steering back to origin (90° -> 0°)
    print(f"  [2/2] CONTINUOUS STEERING return to 启停区1 origin (X=0, Y=0, Yaw=0°)...")
    run_path_and_record(client, path_id_base + 1, RETURN_WAYPOINTS, monitor_duration_s=10.0)
    time.sleep(1.2)

    return {
        "round": round_num,
        "decel": decel,
        "capture_dist": capture_dist,
        "capture_speed": capture_speed,
        "lookahead_base": lookahead_base,
        "max_y": max_y,
        "overshoot_y": overshoot_y,
        "yaw_err": yaw_err,
    }


def main():
    print(f"Connecting to STM32 on {PORT} @ {BAUD}...")
    with SerialClient.open_port(PORT, BAUD, request_timeout_s=0.5, max_attempts=3) as client:
        print("Connected successfully!")
        
        # 4 Rounds of serial parameter candidates to test
        test_candidates = [
            (2200.0, 100.0, 60.0, 80.0),   # Round 1: Moderate decel, 100mm capture
            (2800.0, 140.0, 50.0, 80.0),   # Round 2: Stronger decel, 140mm capture
            (3200.0, 160.0, 40.0, 70.0),   # Round 3: Fast braking, 160mm capture
            (3500.0, 180.0, 30.0, 60.0),   # Round 4: Extra sharp braking, 180mm capture
        ]

        results = []
        path_id = 10
        for r_num, (decel, capture_dist, capture_speed, lookahead) in enumerate(test_candidates, 1):
            try:
                res = execute_test_run(client, r_num, path_id, decel, capture_dist, capture_speed, lookahead)
                results.append(res)
            except Exception as e:
                print(f"  [!] Round {r_num} error: {e}")
            path_id += 2

        print("\n" + "=" * 70)
        print("         COMPARISON TABLE OF CONTINUOUS STEERING MULTI-ROUND TESTS")
        print("=" * 70)
        print(f"{'Round':<6} | {'Decel':<8} | {'CaptDist':<10} | {'CaptSpd':<10} | {'Lookahead':<10} | {'Max Y (mm)':<12} | {'Overshoot (mm)':<14}")
        print("-" * 70)
        for r in results:
            print(f"R{r['round']:<5} | {r['decel']:<8.0f} | {r['capture_dist']:<10.0f} | {r['capture_speed']:<10.0f} | {r['lookahead_base']:<10.0f} | {r['max_y']:<12.1f} | {r['overshoot_y']:<14.1f}")
        print("=" * 70)

        best = min(results, key=lambda x: x['overshoot_y'])
        print(f"\n[RECOMMENDED BEST SERIAL PARAMS]:")
        print(f"  - Round {best['round']}: decel={best['decel']:.0f} mm/s², capture_dist={best['capture_dist']:.0f} mm, capture_speed={best['capture_speed']:.0f} mm/s, lookahead={best['lookahead_base']:.0f} mm")
        print(f"  - Y Overshoot = {best['overshoot_y']:.1f} mm")
        print("\n(Note: No source code files were modified. These parameters were evaluated purely via serial port.)")


if __name__ == "__main__":
    main()
