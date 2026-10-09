#!/usr/bin/env python3
"""把现有双臂目标嵌入 TWIST2 的 35 维全身参考。"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Optional

import numpy as np

from .ua2m_protocol import ArmCommand


# HumDex/TWIST2 的 MuJoCo 29 关节顺序：腿 12 + 腰 3 + 左臂 7 + 右臂 7。
DEFAULT_JOINTS = np.array(
    [
        -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,
        -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.4, 0.0, 1.2, 0.0, 0.0, 0.0,
        0.0, -0.4, 0.0, 1.2, 0.0, 0.0, 0.0,
    ],
    dtype=np.float32,
)
DEFAULT_ROOT_REFERENCE = np.array(
    [0.0, 0.0, 0.8, 0.0, 0.0, 0.0], dtype=np.float32
)

LEFT_ARM_INPUT_INDICES = np.array([15, 16, 17, 18, 19])
RIGHT_ARM_INPUT_INDICES = np.array([22, 23, 24, 25, 26])
# 29 关节里腰是 12/13/14 = waist_yaw / waist_roll / waist_pitch。
WAIST_INPUT_INDICES = np.array([12, 13, 14])

# 只驱动 waist_yaw：waist_roll/pitch 限位只有 ±0.52 rad 且直接与平衡策略
# 耦合，PC 端目前恒发 0，这里再夹一次作为独立防线。
WAIST_LOWER = np.array([-2.6180, -0.5200, -0.5200], dtype=np.float32)
WAIST_UPPER = np.array([2.6180, 0.5200, 0.5200], dtype=np.float32)
# 腰带动整个上半身，比手臂重得多，限速单列。
DEFAULT_MAX_WAIST_SPEED = 2.0

# 与 G1 URDF 关节限制一致；仅列出 PC 当前控制的每臂 5 轴。
LEFT_ARM_LOWER = np.array(
    [-3.0892, -1.5882, -2.6180, -1.0472, -1.97222], dtype=np.float32
)
LEFT_ARM_UPPER = np.array(
    [2.6704, 2.2515, 2.6180, 2.0944, 1.97222], dtype=np.float32
)
RIGHT_ARM_LOWER = np.array(
    [-3.0892, -2.2515, -2.6180, -1.0472, -1.97222], dtype=np.float32
)
RIGHT_ARM_UPPER = np.array(
    [2.6704, 1.5882, 2.6180, 2.0944, 1.97222], dtype=np.float32
)


@dataclass(frozen=True)
class ReferenceStatus:
    mode: int
    timed_out: bool
    packet_age_ms: Optional[float]
    seq: Optional[int]
    protocol: Optional[str]


class ArmToWholeBodyReference:
    """只替换双臂 5+5 轴，其余关节保持 HumDex 平衡参考。"""

    def __init__(
        self,
        timeout_seconds: float = 0.35,
        max_arm_speed: float = 5.0,
        prediction_seconds: float = 0.0,
        max_waist_speed: float = DEFAULT_MAX_WAIST_SPEED,
        allow_waist: bool = True,
    ) -> None:
        self.timeout_seconds = timeout_seconds
        self.max_arm_speed = max_arm_speed
        self.max_waist_speed = max_waist_speed
        # 硬门控：为假时无论发送端是否附带 UAWS 尾块，腰都保持默认参考。
        # 真机默认关闭，仿真默认开启 —— 腰对平衡策略是真实扰动，应当先在
        # 仿真里验证过再显式在真机上打开。
        self.allow_waist = bool(allow_waist)
        self.prediction_seconds = prediction_seconds
        self.current_joints = DEFAULT_JOINTS.copy()
        self.desired_joints = DEFAULT_JOINTS.copy()
        self.last_command: Optional[ArmCommand] = None

    def consume(self, command: ArmCommand) -> None:
        """接收一帧；mode=0 不跟手，mode=1/2/3 对应右/左/双臂。"""
        desired = DEFAULT_JOINTS.copy()
        predicted = np.asarray(command.positions, dtype=np.float32)
        if self.prediction_seconds > 0.0:
            predicted = predicted + (
                np.asarray(command.velocities, dtype=np.float32)
                * self.prediction_seconds
            )

        right = np.clip(predicted[:5], RIGHT_ARM_LOWER, RIGHT_ARM_UPPER)
        left = np.clip(predicted[5:], LEFT_ARM_LOWER, LEFT_ARM_UPPER)
        if command.mode in (1, 3):
            desired[RIGHT_ARM_INPUT_INDICES] = right
        if command.mode in (2, 3):
            desired[LEFT_ARM_INPUT_INDICES] = left

        # 腰跟着双臂走：mode=0 说明操作者暂停或已超时，此时腰保持默认
        # 参考而不是继续跟人转。未附带尾块的老发送端 waist 为 None。
        if self.allow_waist and command.mode != 0 and command.waist is not None:
            desired[WAIST_INPUT_INDICES] = np.clip(
                np.asarray(command.waist, dtype=np.float32),
                WAIST_LOWER, WAIST_UPPER)

        # 两侧腕 pitch/yaw（每臂后两轴）暂时保持 HumDex 默认 0。
        self.desired_joints = desired
        self.last_command = command

    def step(
        self, now: Optional[float] = None, dt: float = 0.02
    ) -> tuple[np.ndarray, ReferenceStatus]:
        now = time.monotonic() if now is None else now
        timed_out = True
        packet_age_ms: Optional[float] = None
        mode = 0
        seq: Optional[int] = None
        protocol: Optional[str] = None

        if self.last_command is not None:
            packet_age = max(0.0, now - self.last_command.received_monotonic)
            packet_age_ms = packet_age * 1000.0
            timed_out = packet_age > self.timeout_seconds
            seq = self.last_command.seq
            protocol = self.last_command.protocol
            mode = 0 if timed_out else self.last_command.mode

        if timed_out:
            self.desired_joints = DEFAULT_JOINTS.copy()

        # 腿始终保持 HumDex 默认参考，实际受限的是双臂和腰的切入、
        # 跟随与超时回中速度。腰带动整个上半身，单独给更低的限速。
        max_delta = np.full(
            DEFAULT_JOINTS.shape, max(0.0, self.max_arm_speed * dt),
            dtype=np.float32)
        max_delta[WAIST_INPUT_INDICES] = max(0.0, self.max_waist_speed * dt)
        delta = self.desired_joints - self.current_joints
        self.current_joints += np.clip(delta, -max_delta, max_delta)

        reference = np.concatenate(
            (DEFAULT_ROOT_REFERENCE, self.current_joints)
        ).astype(np.float32, copy=False)
        return reference, ReferenceStatus(
            mode=mode,
            timed_out=timed_out,
            packet_age_ms=packet_age_ms,
            seq=seq,
            protocol=protocol,
        )

