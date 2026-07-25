import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from protocol.commands import CMD_START_COLOR, CMD_STOP
from protocol.frame import crc16_modbus, pack_frame, parse_frames


def test_crc16_modbus_known_value():
    assert crc16_modbus(b"123456789") == 0x4B37


def test_parser_handles_half_packets_sticky_packets_and_garbage():
    first = pack_frame(CMD_START_COLOR, 3, b"\x28\x00")
    second = pack_frame(CMD_STOP, 3)
    buffer = bytearray()
    assert parse_frames(buffer, b"bad" + first[:4]) == []
    assert parse_frames(buffer, first[4:] + second) == [(CMD_START_COLOR, 3, b"\x28\x00"), (CMD_STOP, 3, b"")]


def test_parser_discards_bad_crc_and_recovers_next_frame():
    bad = bytearray(pack_frame(CMD_START_COLOR, 1, b"\x28\x00"))
    bad[-1] ^= 0xFF
    buffer = bytearray()
    assert parse_frames(buffer, bad + pack_frame(CMD_STOP, 1)) == [(CMD_STOP, 1, b"")]
