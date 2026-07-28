#!/usr/bin/env python3
"""Print serial ports currently available to the PID tuner."""

from serial.tools import list_ports


def main() -> int:
    ports = sorted(list_ports.comports(), key=lambda port: port.device)
    if not ports:
        print("No serial ports found.")
        return 0

    print(f"Found {len(ports)} serial port(s):")
    for port in ports:
        description = port.description or "Unknown device"
        print(f"{port.device}: {description}")
        if port.hwid:
            print(f"  {port.hwid}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
