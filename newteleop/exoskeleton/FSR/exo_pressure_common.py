#!/usr/bin/env python3
"""外骨骼 FSR 工具共用的 BLE Broker TCP 帧解析代码。"""

from __future__ import annotations

import re
import socket
import time
from dataclasses import dataclass
from typing import Iterator


FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")
FINGER_NAMES_CN = ("拇指", "食指", "中指", "无名指", "小指")
FRAME_VALUE_COUNT = 18


class CompactFrameParser:
    """缓存 TCP 分片，只返回完整 ``{...}`` 紧凑帧。"""

    _number = re.compile(
        r"^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$"
    )

    def __init__(self, max_buffer: int = 8192):
        self.buffer = ""
        self.max_buffer = max_buffer

    def feed(self, text: str) -> list[list[float]]:
        self.buffer += text
        frames: list[list[float]] = []
        while True:
            start = self.buffer.find("{")
            if start < 0:
                self.buffer = self.buffer[-self.max_buffer :]
                break
            end = self.buffer.find("}", start + 1)
            if end < 0:
                self.buffer = self.buffer[start:][-self.max_buffer :]
                break
            block = self.buffer[start + 1 : end]
            self.buffer = self.buffer[end + 1 :]
            parts = [part.strip() for part in block.split(",")]
            if len(parts) < FRAME_VALUE_COUNT or any(
                not self._number.fullmatch(part) for part in parts
            ):
                continue
            frames.append([float(part) for part in parts])
        return frames


@dataclass(frozen=True)
class PressureSample:
    mono_time: float
    values: tuple[float, float, float, float, float]


def iter_pressure_samples(
    host: str,
    port: int,
    *,
    connect_timeout: float = 5.0,
    data_timeout: float = 3.0,
) -> Iterator[PressureSample]:
    """连接 BLE broker，产生最后五项 ``[拇,食,中,无,小]``。"""

    parser = CompactFrameParser()
    with socket.create_connection((host, port), timeout=connect_timeout) as sock:
        sock.settimeout(data_timeout)
        while True:
            try:
                chunk = sock.recv(4096)
            except socket.timeout as exc:
                raise TimeoutError(f"{data_timeout:.1f}s 内未收到 BLE 数据") from exc
            if not chunk:
                raise ConnectionError("BLE broker 已关闭 TCP 连接")
            for frame in parser.feed(chunk.decode("utf-8", errors="ignore")):
                values = tuple(frame[-5:])
                yield PressureSample(time.monotonic(), values)  # type: ignore[arg-type]
