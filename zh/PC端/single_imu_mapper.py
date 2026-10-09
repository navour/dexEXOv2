"""右臂单/双 IMU 的临时诊断映射。

该映射假设 IMU X 轴沿小臂、Y 轴对应屈肘轴，将零位后的相对
旋转分解为 ``Ry(elbow) @ Rx(wrist_roll)``。它只用于检查单个 IMU、
UDP 和仿真链路，不是完整的人体双臂解算。
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


def _normalize_quat(q: np.ndarray) -> np.ndarray:
    value = np.asarray(q, dtype=float)
    norm = float(np.linalg.norm(value))
    if value.shape != (4,) or norm < 1e-10 or not np.all(np.isfinite(value)):
        raise ValueError("四元数必须是有限的 [w, x, y, z] 单位四元数")
    return value / norm


def _quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def _quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = _normalize_quat(q)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z),
         2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x),
         1.0 - 2.0 * (x * x + y * y)],
    ])


def _decompose_yxz(rotation: np.ndarray) -> tuple[float, float, float]:
    """将 ``Ry(y) @ Rx(x) @ Rz(z)`` 分解为 y、x、z。"""
    sin_x = -float(np.clip(rotation[1, 2], -1.0, 1.0))
    x_angle = math.asin(sin_x)
    cos_x = math.cos(x_angle)
    if abs(cos_x) > 1e-8:
        z_angle = math.atan2(float(rotation[1, 0]), float(rotation[1, 1]))
        y_angle = math.atan2(float(rotation[0, 2]), float(rotation[2, 2]))
    else:
        z_angle = 0.0
        y_angle = math.atan2(-float(rotation[2, 0]), float(rotation[0, 0]))
    return y_angle, x_angle, z_angle


def _decompose_yx(rotation: np.ndarray) -> tuple[float, float]:
    """将 ``Ry(y) @ Rx(x)`` 分解为 y、x。"""
    y_angle = math.atan2(-float(rotation[2, 0]), float(rotation[0, 0]))
    x_angle = math.atan2(-float(rotation[1, 2]), float(rotation[1, 1]))
    return y_angle, x_angle


@dataclass
class SingleForearmMapper:
    """以一次静止采样为零位，输出 ``(elbow, wrist_roll)``。"""

    elbow_sign: float = 1.0
    wrist_sign: float = 1.0
    zero_quat: np.ndarray | None = None

    def __post_init__(self) -> None:
        if self.elbow_sign not in (-1.0, 1.0):
            raise ValueError("elbow_sign 必须是 -1 或 1")
        if self.wrist_sign not in (-1.0, 1.0):
            raise ValueError("wrist_sign 必须是 -1 或 1")

    @property
    def calibrated(self) -> bool:
        return self.zero_quat is not None

    def calibrate(self, quat: np.ndarray) -> None:
        self.zero_quat = _normalize_quat(quat).copy()

    def reset(self) -> None:
        self.zero_quat = None

    def compute(self, quat: np.ndarray) -> tuple[float, float]:
        if self.zero_quat is None:
            raise RuntimeError("单 IMU 零位尚未采集")

        current = _normalize_quat(quat)
        relative = _normalize_quat(_quat_mul(_quat_conj(self.zero_quat), current))
        if relative[0] < 0.0:
            relative = -relative
        elbow, wrist_roll = _decompose_yx(_quat_to_matrix(relative))
        return self.elbow_sign * elbow, self.wrist_sign * wrist_roll


@dataclass
class RightArmTwoImuMapper:
    """右大臂+右小臂的无胸部 IMU 诊断映射。"""

    shoulder_pitch_sign: float = 1.0
    shoulder_roll_sign: float = 1.0
    shoulder_yaw_sign: float = 1.0
    elbow_sign: float = 1.0
    wrist_sign: float = 1.0
    zero_upper: np.ndarray | None = None
    zero_forearm_relative: np.ndarray | None = None

    def __post_init__(self) -> None:
        signs = (
            self.shoulder_pitch_sign,
            self.shoulder_roll_sign,
            self.shoulder_yaw_sign,
            self.elbow_sign,
            self.wrist_sign,
        )
        if any(sign not in (-1.0, 1.0) for sign in signs):
            raise ValueError("所有方向参数必须是 -1 或 1")

    @property
    def calibrated(self) -> bool:
        return self.zero_upper is not None and self.zero_forearm_relative is not None

    def calibrate(self, upper_quat: np.ndarray, forearm_quat: np.ndarray) -> None:
        upper = _normalize_quat(upper_quat)
        forearm = _normalize_quat(forearm_quat)
        self.zero_upper = upper.copy()
        self.zero_forearm_relative = _normalize_quat(
            _quat_mul(_quat_conj(upper), forearm))

    def reset(self) -> None:
        self.zero_upper = None
        self.zero_forearm_relative = None

    def compute(
        self,
        upper_quat: np.ndarray,
        forearm_quat: np.ndarray,
    ) -> tuple[float, float, float, float, float]:
        if not self.calibrated:
            raise RuntimeError("右臂双 IMU 零位尚未采集")

        upper = _normalize_quat(upper_quat)
        forearm = _normalize_quat(forearm_quat)
        upper_delta = _normalize_quat(
            _quat_mul(_quat_conj(self.zero_upper), upper))
        shoulder_pitch, shoulder_roll, shoulder_yaw = _decompose_yxz(
            _quat_to_matrix(upper_delta))

        forearm_relative = _normalize_quat(
            _quat_mul(_quat_conj(upper), forearm))
        forearm_delta = _normalize_quat(_quat_mul(
            _quat_conj(self.zero_forearm_relative),
            forearm_relative,
        ))
        elbow, wrist_roll = _decompose_yx(_quat_to_matrix(forearm_delta))

        return (
            self.shoulder_pitch_sign * shoulder_pitch,
            self.shoulder_roll_sign * shoulder_roll,
            self.shoulder_yaw_sign * shoulder_yaw,
            self.elbow_sign * elbow,
            self.wrist_sign * wrist_roll,
        )
