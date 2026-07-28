from __future__ import annotations

from queue import Empty, Queue


class FakeTransport:
    def __init__(self, on_write):
        self._on_write = on_write
        self._reads: Queue[bytes] = Queue()
        self.writes: list[bytes] = []
        self.closed = False

    @property
    def in_waiting(self) -> int:
        return 1

    def read(self, size: int = 1) -> bytes:
        try:
            return self._reads.get(timeout=0.02)
        except Empty:
            return b""

    def write(self, data: bytes) -> int:
        self.writes.append(bytes(data))
        for response in self._on_write(bytes(data), len(self.writes)):
            self._reads.put(response)
        return len(data)

    def flush(self) -> None:
        return None

    def close(self) -> None:
        self.closed = True
