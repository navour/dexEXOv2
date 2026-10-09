#!/usr/bin/env python3
"""生成 Sonic/GR00T 所需的双臂全身参考数据，不在此处运行 Sonic 策略。"""

from __future__ import annotations

import numpy as np


# HumDex pipelines/utils/stages.py 中的 MuJoCo -> IsaacLab 29DOF 顺序。
MUJOCO_TO_ISAACLAB = np.array(
    [
        0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10,
        16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28,
    ],
    dtype=np.int64,
)


def build_sonic_reference(
    body_reference: np.ndarray,
    frame_index: int,
    timestamp: float,
    frame_count: int = 1,
) -> dict[str, np.ndarray | int | float | bool]:
    """把 35 维 TWIST2 参考转成 Sonic 接收侧使用的基础字段。"""
    body_reference = np.asarray(body_reference, dtype=np.float32)
    if body_reference.shape != (35,):
        raise ValueError("body_reference 必须是 35 维")
    if frame_count < 1:
        raise ValueError("frame_count 必须大于 0")

    joints_mujoco = body_reference[6:]
    joints_isaaclab = joints_mujoco[MUJOCO_TO_ISAACLAB]
    joint_pos = np.repeat(joints_isaaclab[None, :], frame_count, axis=0)
    return {
        "joint_pos": joint_pos,
        "joint_vel": np.zeros_like(joint_pos),
        "body_quat": np.repeat(
            np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32),
            frame_count,
            axis=0,
        ),
        "frame_index": int(frame_index),
        "timestamp": float(timestamp),
        "heading_increment": 0.0,
        "catch_up": False,
    }

