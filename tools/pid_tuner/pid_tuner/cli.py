"""Subcommand CLI for safe STM32 PID tuning."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from queue import Empty
import sys
import time

from .models import MotionGoal, PidConfig, TunerError
from .serial_client import SerialClient
from .storage import export_c_defaults, list_profiles, load_profile, save_profile, write_telemetry_csv

PID_OPTIONS = (("kp_pos", "kp-pos"), ("ki_pos", "ki-pos"), ("kd_pos", "kd-pos"),
               ("kp_yaw", "kp-yaw"), ("ki_yaw", "ki-yaw"), ("kd_yaw", "kd-yaw"))


def _add_pid_options(parser: argparse.ArgumentParser) -> None:
    for field, option in PID_OPTIONS:
        parser.add_argument(f"--{option}", dest=field, type=float)


def _add_connection_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", required=True, help="serial port, for example COM4")
    parser.add_argument("--baud", type=int, default=115200)


def _pid_from_args(args: argparse.Namespace) -> PidConfig:
    values = {field: getattr(args, field) for field, _ in PID_OPTIONS}
    if any(value is None for value in values.values()):
        raise ValueError("all six PID options are required when --profile is not used")
    return PidConfig(**values)


def _print(value: object, as_json: bool = False) -> None:
    if as_json:
        if hasattr(value, "to_dict"):
            value = value.to_dict()
        print(json.dumps(value, ensure_ascii=False, default=str))
    else:
        print(value)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="pid-tuner", description="STM32 PID tuner command line")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("ports", help="list serial ports").add_argument("--json", action="store_true")

    get_pid = commands.add_parser("get-pid", help="read active PID")
    _add_connection_options(get_pid); get_pid.add_argument("--json", action="store_true")

    set_pid = commands.add_parser("set-pid", help="write six PID values")
    _add_connection_options(set_pid); _add_pid_options(set_pid)
    set_pid.add_argument("--profile"); set_pid.add_argument("--apply", action="store_true")
    set_pid.add_argument("--json", action="store_true")

    restore = commands.add_parser("restore-pid", help="restore firmware defaults")
    _add_connection_options(restore); restore.add_argument("--apply", action="store_true")
    restore.add_argument("--json", action="store_true")

    goto = commands.add_parser("goto", help="run one remote motion session")
    _add_connection_options(goto)
    goto.add_argument("--x", type=float, required=True); goto.add_argument("--y", type=float, required=True)
    goto.add_argument("--yaw", type=float, required=True); goto.add_argument("--vmax", type=float, required=True)
    goto.add_argument("--wmax", type=float, required=True); goto.add_argument("--timeout", type=int, required=True)
    goto.add_argument("--no-yaw", action="store_true", help="disable yaw constraint for position-loop tuning")
    goto.add_argument("--csv", type=Path); goto.add_argument("--json", action="store_true")

    monitor = commands.add_parser("monitor", help="passively display telemetry")
    _add_connection_options(monitor); monitor.add_argument("--duration", type=float, default=30.0)
    monitor.add_argument("--csv", type=Path); monitor.add_argument("--json", action="store_true")

    profile = commands.add_parser("profile", help="manage local PID profiles")
    profile_commands = profile.add_subparsers(dest="profile_command", required=True)
    profile_commands.add_parser("list").add_argument("--json", action="store_true")
    show = profile_commands.add_parser("show"); show.add_argument("name"); show.add_argument("--json", action="store_true")
    save = profile_commands.add_parser("save"); save.add_argument("name"); _add_pid_options(save)
    save.add_argument("--note", default=""); save.add_argument("--firmware-revision", type=int)
    save.add_argument("--from-device", action="store_true")
    save.add_argument("--port", help="serial port required with --from-device")
    save.add_argument("--baud", type=int, default=115200); save.add_argument("--json", action="store_true")
    export = profile_commands.add_parser("export-c"); export.add_argument("name"); export.add_argument("--output", type=Path)
    export.add_argument("--json", action="store_true")
    return parser


def _open(args: argparse.Namespace) -> SerialClient:
    return SerialClient.open_port(args.port, args.baud)


def _collect(client: SerialClient, duration_s: float, keepalive: bool, as_json: bool) -> list:
    telemetry = []
    deadline = time.monotonic() + duration_s
    heartbeat_at = time.monotonic() + 0.5
    last_telemetry = time.monotonic()
    while time.monotonic() < deadline:
        try:
            item = client.get_telemetry(timeout_s=0.1)
            telemetry.append(item); last_telemetry = time.monotonic()
            _print(item.to_dict() if as_json else f"tick={item.tick} state={item.state} error={item.error}", as_json)
        except Empty:
            pass
        if keepalive and time.monotonic() >= heartbeat_at:
            client.heartbeat(); heartbeat_at += 0.5
        if keepalive and telemetry and (time.monotonic() - last_telemetry) >= 1.2:
            break
    return telemetry


def _run_goto(args: argparse.Namespace) -> int:
    goal = MotionGoal(args.x, args.y, args.yaw, args.vmax, args.wmax, args.timeout,
                      use_yaw=not args.no_yaw)
    telemetry = []
    with _open(args) as client:
        client.goto(goal)
        try:
            telemetry = _collect(client, (args.timeout / 1000.0) + 1.0, True, args.json)
        finally:
            try:
                client.stop()
            except TunerError as error:
                print(f"STOP failed: {error}", file=sys.stderr)
    if args.csv:
        write_telemetry_csv(args.csv, telemetry)
    _print({"telemetry_frames": len(telemetry), "csv": str(args.csv) if args.csv else None}, args.json)
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "ports":
            from serial.tools import list_ports
            ports = [{"port": item.device, "description": item.description, "hwid": item.hwid}
                     for item in sorted(list_ports.comports(), key=lambda item: item.device)]
            _print(ports if args.json else "\n".join(f"{item['port']}: {item['description']}" for item in ports), args.json)
            return 0
        if args.command == "get-pid":
            with _open(args) as client:
                state = client.get_pid()
            _print({"revision": state.revision, "pid": state.config.to_dict()} if args.json else
                   f"revision={state.revision}, pid={state.config}", args.json)
            return 0
        if args.command == "set-pid":
            if args.profile and any(getattr(args, field) is not None for field, _ in PID_OPTIONS):
                raise ValueError("use either --profile or six PID options, not both")
            pid = load_profile(args.profile)[0] if args.profile else _pid_from_args(args)
            if not args.apply:
                _print({"pending": pid.to_dict()} if args.json else f"dry run: {pid}; add --apply to write", args.json)
                return 0
            with _open(args) as client: response = client.set_pid(pid)
            _print({"revision": response.revision, "pid": pid.to_dict()} if args.json else
                   f"PID submitted, revision={response.revision}", args.json)
            return 0
        if args.command == "restore-pid":
            if not args.apply:
                _print("dry run: add --apply to restore firmware defaults", args.json); return 0
            with _open(args) as client: response = client.restore_pid()
            _print({"revision": response.revision} if args.json else
                   f"firmware defaults submitted, revision={response.revision}", args.json); return 0
        if args.command == "goto": return _run_goto(args)
        if args.command == "monitor":
            with _open(args) as client: telemetry = _collect(client, args.duration, False, args.json)
            if args.csv: write_telemetry_csv(args.csv, telemetry)
            _print({"telemetry_frames": len(telemetry)} if args.json else f"telemetry frames={len(telemetry)}", args.json); return 0
        if args.profile_command == "list": _print(list_profiles(), args.json); return 0
        if args.profile_command == "show":
            pid, document = load_profile(args.name); _print(document if args.json else pid, args.json); return 0
        if args.profile_command == "save":
            if args.from_device:
                if not args.port:
                    raise ValueError("profile save --from-device requires --port")
                with _open(args) as client: state = client.get_pid()
                revision, pid = state.revision, state.config
            else:
                revision, pid = args.firmware_revision, _pid_from_args(args)
            path = save_profile(args.name, pid, args.note, revision)
            _print({"path": str(path), "pid": pid.to_dict()} if args.json else f"saved {path}", args.json); return 0
        if args.profile_command == "export-c":
            text = export_c_defaults(load_profile(args.name)[0])
            if args.output: args.output.write_text(text, encoding="utf-8")
            _print({"output": str(args.output) if args.output else None, "text": text} if args.json else text, args.json); return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, TunerError, TimeoutError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    return 2
