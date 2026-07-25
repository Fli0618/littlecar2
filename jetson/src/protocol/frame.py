"""视觉服务协议的函数式帧封装和增量解析。"""

from __future__ import annotations

from .commands import HEADER, MAX_PAYLOAD_LEN


def crc16_modbus(data: bytes | bytearray | memoryview) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if crc & 1 else (crc >> 1)
    return crc & 0xFFFF


def pack_frame(command: int, session: int, payload: bytes = b"") -> bytes:
    if not 0 <= command <= 0xFF or not 0 <= session <= 0xFF:
        raise ValueError("command and session must be uint8")
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ValueError("payload length must be <= 255")
    body = bytes((command, session, len(payload))) + payload
    return HEADER + body + crc16_modbus(body).to_bytes(2, "little")


def parse_frames(buffer: bytearray, data: bytes | bytearray | memoryview) -> list[tuple[int, int, bytes]]:
    """将数据追加到 ``buffer`` 并返回所有完整、CRC 正确的帧。

    ``buffer`` 由调用方持有，避免解析器对象和隐藏通信上下文。
    """
    buffer.extend(data)
    frames: list[tuple[int, int, bytes]] = []
    while True:
        start = buffer.find(HEADER)
        if start < 0:
            if buffer[-1:] == HEADER[:1]:
                del buffer[:-1]
            else:
                buffer.clear()
            break
        if start:
            del buffer[:start]
        if len(buffer) < 7:
            break
        payload_len = buffer[4]
        frame_len = 2 + 3 + payload_len + 2
        if len(buffer) < frame_len:
            break
        body = bytes(buffer[2 : 5 + payload_len])
        received_crc = int.from_bytes(buffer[5 + payload_len : frame_len], "little")
        if received_crc == crc16_modbus(body):
            frames.append((buffer[2], buffer[3], bytes(buffer[5 : 5 + payload_len])))
            del buffer[:frame_len]
        else:
            del buffer[0]
    return frames
