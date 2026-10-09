#!/usr/bin/env python3
"""解析现有 PC 端发送的 UA2M/UARM 双臂 UDP 指令。"""

from __future__ import annotations

from dataclasses import dataclass
import socket
import struct
import time
from typing import Optional


PACKET_HEADER_V2 = b"UA2M"
PACKET_FMT_V2 = "!4sBIQ20f"
PACKET_SIZE_V2 = struct.calcsize(PACKET_FMT_V2)

# 腰部尾块。PC 端把它追加在 97 字节 UA2M 之后；只按 data[:PACKET_SIZE_V2]
# 切片的消费端（仿真、真机接收端）会忽略它，这里主动读取。
WAIST_HEADER = b"UAWS"
WAIST_FMT = "!4sB3f3f"
WAIST_SIZE = struct.calcsize(WAIST_FMT)

PACKET_HEADER_V1 = b"UARM"
PACKET_FMT_SINGLE = "!4sBffff"
PACKET_FMT_DUAL = "!4sBffffffff"
PACKET_FMT_DUAL_WR = "!4sBffffffffff"
PACKET_SIZE_SINGLE = struct.calcsize(PACKET_FMT_SINGLE)
PACKET_SIZE_DUAL = struct.calcsize(PACKET_FMT_DUAL)
PACKET_SIZE_DUAL_WR = struct.calcsize(PACKET_FMT_DUAL_WR)


@dataclass(frozen=True)
class ArmCommand:
    """统一后的双臂指令，顺序始终是右臂 5 轴、左臂 5 轴。"""

    mode: int
    seq: Optional[int]
    sender_timestamp_us: Optional[int]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    protocol: str
    received_monotonic: float
    # 腰部 yaw/roll/pitch，单位 rad。发送端未附带尾块或未启用时为 None，
    # 此时消费端应把腰保持在默认参考，等于退回无腰行为。
    waist: Optional[tuple[float, float, float]] = None

    @property
    def right_positions(self) -> tuple[float, ...]:
        return self.positions[:5]

    @property
    def left_positions(self) -> tuple[float, ...]:
        return self.positions[5:]

    @property
    def right_velocities(self) -> tuple[float, ...]:
        return self.velocities[:5]

    @property
    def left_velocities(self) -> tuple[float, ...]:
        return self.velocities[5:]


def parse_arm_packet(data: bytes, received_monotonic: Optional[float] = None) -> ArmCommand:
    """解析一帧数据；无效包抛出 ValueError。"""
    received_monotonic = (
        time.monotonic() if received_monotonic is None else received_monotonic
    )

    if len(data) >= PACKET_SIZE_V2 and data[:4] == PACKET_HEADER_V2:
        values = struct.unpack(PACKET_FMT_V2, data[:PACKET_SIZE_V2])
        mode = int(values[1])
        positions = tuple(float(v) for v in values[4:14])
        velocities = tuple(float(v) for v in values[14:24])
        waist = parse_waist_block(data[PACKET_SIZE_V2:])
        return _validated_command(
            mode=mode,
            seq=int(values[2]),
            sender_timestamp_us=int(values[3]),
            positions=positions,
            velocities=velocities,
            protocol="UA2M+W" if waist is not None else "UA2M",
            received_monotonic=received_monotonic,
            waist=waist,
        )

    if data[:4] != PACKET_HEADER_V1:
        raise ValueError("未知数据包头")

    if len(data) >= PACKET_SIZE_DUAL_WR:
        values = struct.unpack(PACKET_FMT_DUAL_WR, data[:PACKET_SIZE_DUAL_WR])
        return _validated_command(
            mode=int(values[1]),
            seq=None,
            sender_timestamp_us=None,
            positions=tuple(float(v) for v in values[2:12]),
            velocities=(0.0,) * 10,
            protocol="UARM-10",
            received_monotonic=received_monotonic,
        )

    if len(data) >= PACKET_SIZE_DUAL:
        values = struct.unpack(PACKET_FMT_DUAL, data[:PACKET_SIZE_DUAL])
        # 历史 8 轴包缺少两个腕部 roll，补零后仍保持右 5、左 5。
        right = tuple(float(v) for v in values[2:6]) + (0.0,)
        left = tuple(float(v) for v in values[6:10]) + (0.0,)
        return _validated_command(
            mode=int(values[1]),
            seq=None,
            sender_timestamp_us=None,
            positions=right + left,
            velocities=(0.0,) * 10,
            protocol="UARM-8",
            received_monotonic=received_monotonic,
        )

    if len(data) >= PACKET_SIZE_SINGLE:
        values = struct.unpack(PACKET_FMT_SINGLE, data[:PACKET_SIZE_SINGLE])
        arm = tuple(float(v) for v in values[2:6]) + (0.0,)
        mode = int(values[1])
        right = arm if mode == 1 else (0.0,) * 5
        left = arm if mode == 2 else (0.0,) * 5
        return _validated_command(
            mode=mode,
            seq=None,
            sender_timestamp_us=None,
            positions=right + left,
            velocities=(0.0,) * 10,
            protocol="UARM-4",
            received_monotonic=received_monotonic,
        )

    raise ValueError("数据包长度不足")


def parse_waist_block(tail: bytes) -> Optional[tuple[float, float, float]]:
    """解析腰部尾块；缺失、包头不符或未启用时返回 None。

    尾块无效不影响双臂：这里只返回 None，让调用方保持腰部默认参考。
    """
    if len(tail) < WAIST_SIZE or tail[:4] != WAIST_HEADER:
        return None
    values = struct.unpack(WAIST_FMT, tail[:WAIST_SIZE])
    if not int(values[1]):
        return None
    waist = tuple(float(v) for v in values[2:5])
    if any(not (-3.2 <= value <= 3.2) for value in waist):
        return None
    return waist


def _validated_command(
    *,
    mode: int,
    seq: Optional[int],
    sender_timestamp_us: Optional[int],
    positions: tuple[float, ...],
    velocities: tuple[float, ...],
    protocol: str,
    received_monotonic: float,
    waist: Optional[tuple[float, float, float]] = None,
) -> ArmCommand:
    if mode not in (0, 1, 2, 3):
        raise ValueError(f"无效模式: {mode}")
    if len(positions) != 10 or len(velocities) != 10:
        raise ValueError("双臂数据必须包含 10 路位置和速度")
    if any(not (-20.0 <= value <= 20.0) for value in positions):
        raise ValueError("关节角超出合理解析范围")
    if any(not (-100.0 <= value <= 100.0) for value in velocities):
        raise ValueError("关节速度超出合理解析范围")
    return ArmCommand(
        mode=mode,
        seq=seq,
        sender_timestamp_us=sender_timestamp_us,
        positions=positions,
        velocities=velocities,
        protocol=protocol,
        received_monotonic=received_monotonic,
        waist=waist,
    )


class LatestArmReceiver:
    """非阻塞 UDP 接收器，每次只交付最新一帧，避免积压造成延迟。"""

    def __init__(self, bind: str, port: int) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((bind, port))
        self.socket.setblocking(False)
        self.packet_count = 0
        self.invalid_count = 0
        self.lost_count = 0
        self.last_seq: Optional[int] = None
        self.last_command: Optional[ArmCommand] = None

    def poll_latest(self) -> Optional[ArmCommand]:
        latest: Optional[ArmCommand] = None
        while True:
            try:
                data, _address = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            try:
                command = parse_arm_packet(data)
            except (ValueError, struct.error):
                self.invalid_count += 1
                continue
            self.packet_count += 1
            self._update_sequence(command.seq)
            latest = command
        if latest is not None:
            self.last_command = latest
        return latest

    def _update_sequence(self, seq: Optional[int]) -> None:
        if seq is None:
            return
        if self.last_seq is not None:
            delta = (seq - self.last_seq) & 0xFFFFFFFF
            if 1 < delta < 0x80000000:
                self.lost_count += delta - 1
        self.last_seq = seq

    def close(self) -> None:
        self.socket.close()

