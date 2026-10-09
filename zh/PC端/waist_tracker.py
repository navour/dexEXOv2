#!/usr/bin/env python3
"""从腰部 IMU 的绝对姿态解出腰部偏航角。

手臂的解算全部是相对量（大臂/小臂相对胸部 IMU），漂移会互相抵消一部分；
腰部只有一个 IMU，用的是它相对一个记录零位的绝对转角，没有第二个传感器
可以抵消。实测 VQF 静态偏航漂移约 -0.07°/h，这个量级可以接受，但零位仍
然需要能随时重新捕获。

只解 yaw：G1 的 waist_roll/waist_pitch 限位只有 ±0.52 rad，且直接与全身
平衡策略耦合，暂不驱动。
"""

from __future__ import annotations

import numpy as np

from arm_calibration import (
    normalize_quat, quat_conj, quat_mul, swing_twist_angle,
)


# 人的腰转不到 G1 waist_yaw 的 ±2.618 rad 限位，按人体活动范围收紧。
DEFAULT_MAX_YAW_RAD = 1.0
DEFAULT_MAX_SPEED_RAD_S = 2.0
# 放大系数。两处欠跟踪叠加：腰眼 IMU 测到的转角比操作者体感小，
# TWIST2 策略对腰部参考又只跟到约 0.7（2026-07-27 实测，见
# logs/twist2_dual_arm_20260727_142947.csv）。默认 1.0 保持原样，
# 由操作者按手感调；增益作用在限幅之前，限幅仍是最终防线。
DEFAULT_GAIN = 1.0
MIN_GAIN = 0.5
MAX_GAIN = 3.0


def waist_yaw_from_quaternion(q_zero, q_now):
    """零位姿态到当前姿态之间绕铅垂轴的转角，单位 rad。

    ``q_rel`` 表达在零位坐标系中；捕获零位时人是站直的，此时 IMU 的 Z
    轴与重力上方向重合，因此绕零位系 Z 轴做 swing-twist 分解，等价于绕
    重力轴分解，并且自动吸收了 IMU 佩戴时的安装偏差。
    """
    q_rel = normalize_quat(quat_mul(
        quat_conj(normalize_quat(np.asarray(q_zero, dtype=float))),
        normalize_quat(np.asarray(q_now, dtype=float))))
    return float(swing_twist_angle(q_rel, np.array([0.0, 0.0, 1.0])))


def wrap_to_pi(angle):
    """把角度折回 (-pi, pi]，避免零位附近 ±180° 跳变。"""
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


class WaistTracker:
    """记录腰部零位并输出限幅、限速后的 waist_yaw 目标。"""

    def __init__(self, max_yaw_rad=DEFAULT_MAX_YAW_RAD,
                 max_speed_rad_s=DEFAULT_MAX_SPEED_RAD_S, sign=1.0,
                 gain=DEFAULT_GAIN):
        if max_yaw_rad <= 0.0:
            raise ValueError("max_yaw_rad 必须大于 0")
        if max_speed_rad_s <= 0.0:
            raise ValueError("max_speed_rad_s 必须大于 0")
        self.max_yaw_rad = float(max_yaw_rad)
        self.max_speed_rad_s = float(max_speed_rad_s)
        self.sign = 1.0 if float(sign) >= 0.0 else -1.0
        self.gain = self._clamp_gain(gain)
        self.q_zero = None
        self.raw_yaw = 0.0
        self.yaw = 0.0
        self.clamped = False

    @staticmethod
    def _clamp_gain(gain):
        return float(min(max(float(gain), MIN_GAIN), MAX_GAIN))

    def adjust_gain(self, delta):
        """按步长调整增益并返回新值，便于运行中试手感。"""
        self.gain = self._clamp_gain(self.gain + float(delta))
        return self.gain

    @property
    def calibrated(self):
        return self.q_zero is not None

    def capture_zero(self, q_waist):
        """把当前姿态记为零位；调用者负责确保人正对前方站直。"""
        self.q_zero = normalize_quat(np.asarray(q_waist, dtype=float)).copy()
        self.raw_yaw = 0.0
        self.yaw = 0.0
        self.clamped = False

    def reset(self):
        self.q_zero = None
        self.raw_yaw = 0.0
        self.yaw = 0.0
        self.clamped = False

    def update(self, q_waist, dt):
        """推进一步并返回限幅限速后的 waist_yaw 目标，单位 rad。

        未捕获零位、dt 非正或输入含 NaN 时保持上一次输出不变。
        """
        if self.q_zero is None:
            return self.yaw
        q_waist = np.asarray(q_waist, dtype=float)
        if q_waist.shape != (4,) or not np.all(np.isfinite(q_waist)):
            return self.yaw
        if not np.isfinite(dt) or dt <= 0.0:
            return self.yaw

        # 先折回 (-pi, pi] 再放大：反过来的话大转角会先溢出再被折回，
        # 在 ±180° 附近产生符号翻转。raw_yaw 保留未放大的实测值。
        raw = wrap_to_pi(
            self.sign * waist_yaw_from_quaternion(self.q_zero, q_waist))
        self.raw_yaw = raw
        amplified = raw * self.gain
        target = float(np.clip(
            amplified, -self.max_yaw_rad, self.max_yaw_rad))
        self.clamped = abs(amplified) > self.max_yaw_rad

        max_step = self.max_speed_rad_s * float(dt)
        delta = float(np.clip(target - self.yaw, -max_step, max_step))
        self.yaw += delta
        return self.yaw

    def relax_to_zero(self, dt):
        """腰部关闭或超时时按同一限速回到零位。"""
        if not np.isfinite(dt) or dt <= 0.0:
            return self.yaw
        max_step = self.max_speed_rad_s * float(dt)
        self.yaw += float(np.clip(-self.yaw, -max_step, max_step))
        return self.yaw
