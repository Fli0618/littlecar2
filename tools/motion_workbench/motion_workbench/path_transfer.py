"""路径上传命令兼容入口；具体编码只在协议层完成。"""

from __future__ import annotations

from collections.abc import Iterable

from pid_tuner.models import PathBeginCommand, PathChunkCommand, PathCommitCommand
from pid_tuner.protocol import build_path_upload


def build_path_commands(path_id: int, points: Iterable[object]) -> tuple[
        PathBeginCommand, tuple[PathChunkCommand, ...], PathCommitCommand]:
    return build_path_upload(path_id, points)

