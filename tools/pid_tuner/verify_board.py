#!/usr/bin/env python3
"""Verify the STM32 tuner protocol on a real USART1 connection."""

from __future__ import annotations

import argparse
import csv
from collections import deque
import struct
import sys
import time
from pathlib import Path

from pid_tuner.protocol import (
    CMD_ACK,
    CMD_ERROR,
    CMD_GET_PID,
    CMD_GOTO_POSE,
    CMD_HEARTBEAT,
    CMD_PID,
    CMD_RESTORE_PID,
    CMD_SET_PID,
    CMD_STOP,
    CMD_TELEMETRY,
    Frame,
    PidConfig,
    ProtocolError,
    StreamDecoder,
    decode_pid,
    decode_telemetry,
    encode_frame,
    encode_pid,
    telemetry_csv_row,
)

HEARTBEAT_PERIOD_S = 0.5
SAFE_VMAX_MM_S = 250.0
SAFE_WMAX_DEG_S = 90.0
SAFE_TIMEOUT_MS = 15000


class BoardClient:
    def __init__(self, serial_port: object) -> None:
        self.serial_port = serial_port
        self.decoder = StreamDecoder()
        self.frames: deque[Frame] = deque()
        self.sequence = 0
        self.telemetry = []
        self.telemetry_sequence_losses = 0
        self._last_telemetry_sequence: int | None = None

    def _next_sequence(self) -> int:
        value = self.sequence
        self.sequence = (self.sequence + 1) & 0xFF
        return value

    def _read_frames(self) -> None:
        waiting = self.serial_port.in_waiting
        data = self.serial_port.read(waiting or 1)
        for frame in self.decoder.feed(data):
            if frame.command == CMD_TELEMETRY:
                self._record_telemetry(frame)
            else:
                self.frames.append(frame)

    def _record_telemetry(self, frame: Frame) -> None:
        telemetry = decode_telemetry(frame)
        if self._last_telemetry_sequence is not None:
            expected = (self._last_telemetry_sequence + 1) & 0xFF
            if frame.sequence != expected:
                self.telemetry_sequence_losses += (frame.sequence - expected) & 0xFF
        self._last_telemetry_sequence = frame.sequence
        self.telemetry.append(telemetry)

    def request(self, command: int, payload: bytes = b"", expected_command: int = CMD_ACK,
                timeout_s: float = 1.0) -> Frame:
        sequence = self._next_sequence()
        self.serial_port.write(encode_frame(command, sequence, payload))
        self.serial_port.flush()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._read_frames()
            while self.frames:
                frame = self.frames.popleft()
                if frame.sequence != sequence:
                    continue
                if frame.command == CMD_ERROR:
                    code = frame.payload[1] if len(frame.payload) >= 2 else None
                    raise ProtocolError(f"board rejected command 0x{command:02X}, error={code}")
                if frame.command != expected_command:
                    if frame.command == 0x85: # CMD_PATH_TELEMETRY can also be ignored or handled
                        continue
                    raise ProtocolError(
                        f"unexpected response 0x{frame.command:02X} for command 0x{command:02X}"
                    )
                return frame
            time.sleep(0.005)
        raise TimeoutError(f"timeout waiting for command 0x{command:02X}")

    def verify_bad_crc(self) -> None:
        sequence = self._next_sequence()
        raw = bytearray(encode_frame(CMD_GET_PID, sequence))
        raw[-1] ^= 0xFF
        self.serial_port.write(raw)
        self.serial_port.flush()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            self._read_frames()
            while self.frames:
                frame = self.frames.popleft()
                if frame.sequence == sequence and frame.command == CMD_ERROR:
                    return
            time.sleep(0.005)
        raise TimeoutError("timeout waiting for bad-CRC ERROR response")

    def monitor(self, duration_s: float, send_heartbeats: bool) -> None:
        deadline = time.monotonic() + duration_s
        next_heartbeat = time.monotonic() + HEARTBEAT_PERIOD_S
        while time.monotonic() < deadline:
            self._read_frames()
            if send_heartbeats and time.monotonic() >= next_heartbeat:
                self.request(CMD_HEARTBEAT)
                next_heartbeat += HEARTBEAT_PERIOD_S
            time.sleep(0.002)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="STM32 PID tuner board verifier")
    parser.add_argument("--port", required=True, help="Windows serial port, for example COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=30.0, help="telemetry listen duration in seconds")
    parser.add_argument("--write-pid", action="store_true", help="also validate SET_PID and RESTORE_PID")
    parser.add_argument("--exercise-motion", action="store_true", help="send one explicitly supplied low-speed GOTO")
    parser.add_argument("--x", type=float, help="absolute target X in mm")
    parser.add_argument("--y", type=float, help="absolute target Y in mm")
    parser.add_argument("--yaw", type=float, help="absolute target yaw in deg")
    parser.add_argument("--csv", type=Path, help="optional telemetry CSV output path")
    return parser


def write_csv(path: Path, telemetry: list) -> None:
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


def main() -> int:
    args = build_parser().parse_args()
    if args.duration <= 0:
        raise SystemExit("--duration must be positive")
    if args.exercise_motion and any(value is None for value in (args.x, args.y, args.yaw)):
        raise SystemExit("--exercise-motion requires --x, --y and --yaw")

    try:
        import serial
    except ImportError as error:
        raise SystemExit("pyserial is required; run: conda run -n low_numpy pip install -e .") from error

    client: BoardClient | None = None
    try:
        with serial.Serial(args.port, args.baud, timeout=0.02) as serial_port:
            client = BoardClient(serial_port)
            response = client.request(CMD_GET_PID, expected_command=CMD_PID)
            revision, pid = decode_pid(response.payload)
            print(f"GET_PID OK: revision={revision}, pid={pid}")
            client.verify_bad_crc()
            print("bad CRC recovery OK")

            if args.write_pid:
                print("warning: --write-pid writes the current values, then restores firmware defaults")
                client.request(CMD_SET_PID, encode_pid(pid))
                time.sleep(0.05)
                client.request(CMD_RESTORE_PID)
                print("SET_PID and RESTORE_PID OK")

            if args.exercise_motion:
                payload = struct.pack(
                    "<5fIB", args.x, args.y, args.yaw, SAFE_VMAX_MM_S, SAFE_WMAX_DEG_S,
                    SAFE_TIMEOUT_MS, 0x03, # ADVANCE_MOTION_GOAL_USE_POSITION (0x02) | ADVANCE_MOTION_GOAL_USE_YAW (0x01)
                )
                client.request(CMD_GOTO_POSE, payload)
                print(f"GOTO_POSE accepted; collecting telemetry for up to {args.duration:g} seconds")
                try:
                    client.monitor(args.duration, send_heartbeats=True)
                finally:
                    client.request(CMD_STOP)
                    print("STOP sent")
            else:
                print("read-only verification complete; use --exercise-motion to collect telemetry")
    except (OSError, TimeoutError, ProtocolError) as error:
        print(f"verification failed: {error}", file=sys.stderr)
        return 1
    assert client is not None
    if args.csv is not None:
        write_csv(args.csv, client.telemetry)
        print(f"CSV written: {args.csv}")
    latest = client.telemetry[-1] if client.telemetry else None
    print(
        "telemetry summary: "
        f"frames={len(client.telemetry)}, crc_errors={client.decoder.crc_errors}, "
        f"sequence_losses={client.telemetry_sequence_losses}, "
        f"board_overwritten={(latest.overwritten_count if latest else 0)}"
    )
    if latest is not None:
        print(f"latest: state={latest.state}, flags=0x{latest.flags:02X}, error={latest.error}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
