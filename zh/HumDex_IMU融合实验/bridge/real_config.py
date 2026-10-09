#!/usr/bin/env python3
"""真机常量：关节限位、PD 增益、默认姿态。

真机端只依赖 numpy / onnxruntime / unitree_sdk2py，不装 mujoco，
因此关节限位在这里以常量形式给出。tests/test_real_config.py 会拿
HumDex 的 g1_sim2sim_29dof.xml 逐项核对，防止两边漂移。

29 关节顺序 = 左腿6 + 右腿6 + 腰3 + 左臂7 + 右臂7，
与 G1 SDK 的 29 电机索引一一对应（上游 g1.yaml 的 joint2motor_idx 是恒等映射）。
"""

from __future__ import annotations

import numpy as np


NUM_MOTORS = 29

JOINT_NAMES = (
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
    "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw",
    "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
)

# 取自 upstream/HumDex/assets/g1/g1_sim2sim_29dof.xml 的 jnt_range。
JOINT_LIMITS = np.array(
    [
        (-2.53070, 2.87980),   #  0 left_hip_pitch
        (-0.52360, 2.96710),   #  1 left_hip_roll
        (-2.75760, 2.75760),   #  2 left_hip_yaw
        (-0.08727, 2.87980),   #  3 left_knee
        (-0.87267, 0.52360),   #  4 left_ankle_pitch
        (-0.26180, 0.26180),   #  5 left_ankle_roll
        (-2.53070, 2.87980),   #  6 right_hip_pitch
        (-2.96710, 0.52360),   #  7 right_hip_roll
        (-2.75760, 2.75760),   #  8 right_hip_yaw
        (-0.08727, 2.87980),   #  9 right_knee
        (-0.87267, 0.52360),   # 10 right_ankle_pitch
        (-0.26180, 0.26180),   # 11 right_ankle_roll
        (-2.61800, 2.61800),   # 12 waist_yaw
        (-0.52000, 0.52000),   # 13 waist_roll
        (-0.52000, 0.52000),   # 14 waist_pitch
        (-3.08920, 2.67040),   # 15 left_shoulder_pitch
        (-1.58820, 2.25150),   # 16 left_shoulder_roll
        (-2.61800, 2.61800),   # 17 left_shoulder_yaw
        (-1.04720, 2.09440),   # 18 left_elbow
        (-1.97222, 1.97222),   # 19 left_wrist_roll
        (-1.61443, 1.61443),   # 20 left_wrist_pitch
        (-1.61443, 1.61443),   # 21 left_wrist_yaw
        (-3.08920, 2.67040),   # 22 right_shoulder_pitch
        (-2.25150, 1.58820),   # 23 right_shoulder_roll
        (-2.61800, 2.61800),   # 24 right_shoulder_yaw
        (-1.04720, 2.09440),   # 25 right_elbow
        (-1.97222, 1.97222),   # 26 right_wrist_roll
        (-1.61443, 1.61443),   # 27 right_wrist_pitch
        (-1.61443, 1.61443),   # 28 right_wrist_yaw
    ],
    dtype=np.float64,
)
JOINT_LOWER = JOINT_LIMITS[:, 0].copy()
JOINT_UPPER = JOINT_LIMITS[:, 1].copy()

# 取自 upstream/HumDex/deploy_real/robot_control/configs/g1.yaml。
# 注意：与 twist2_dynamic_sim 的 STIFFNESS/DAMPING 在腕部三轴不同
# (真机 kp=20/kd=1，仿真 kp=4/kd=0.2)，腿/腰/大臂完全一致。
# 这里以上游真机配置为准——那是在实物 G1 上跑过的一组。
KPS = np.array(
    [
        100, 100, 100, 150, 40, 40,
        100, 100, 100, 150, 40, 40,
        150, 150, 150,
        40, 40, 40, 40, 20, 20, 20,
        40, 40, 40, 40, 20, 20, 20,
    ],
    dtype=np.float64,
)
KDS = np.array(
    [
        2, 2, 2, 4, 2, 2,
        2, 2, 2, 4, 2, 2,
        4, 4, 4,
        5, 5, 5, 5, 1, 1, 1,
        5, 5, 5, 5, 1, 1, 1,
    ],
    dtype=np.float64,
)

# 阻尼释放态：kp 全零，仅保留阻尼让机器人软落而不是硬直或断电瘫。
DAMPING_KD = np.array(
    [
        5, 5, 5, 5, 3, 3,
        5, 5, 5, 5, 3, 3,
        5, 5, 5,
        3, 3, 3, 3, 1, 1, 1,
        3, 3, 3, 3, 1, 1, 1,
    ],
    dtype=np.float64,
)

# G1 各关节力矩上限 (N·m)，用于 tau_est 监视。
TORQUE_LIMITS = np.array(
    [
        88, 139, 88, 139, 50, 50,
        88, 139, 88, 139, 50, 50,
        88, 50, 50,
        25, 25, 25, 25, 25, 5, 5,
        25, 25, 25, 25, 25, 5, 5,
    ],
    dtype=np.float64,
)

MOTOR_MODE_ENABLE = 1
LOWCMD_MODE_PR = 0

LOWCMD_TOPIC = "rt/lowcmd"
LOWSTATE_TOPIC = "rt/lowstate"
WIRELESS_TOPIC = "rt/wirelesscontroller"

# 与 upstream/HumDex/deploy_real/robot_control/g1_wrapper.py 的 ControllerMapping 一致。
CONTROLLER_KEYS = {
    "A": 0x0100,
    "B": 0x0200,
    "X": 0x0400,
    "Y": 0x0800,
    "R1": 0x0001,
    "L1": 0x0002,
    "start": 0x0004,
    "select": 0x0008,
    "R2": 0x0010,
    "L2": 0x0020,
    "F1": 0x0040,
    "F2": 0x0080,
    "up": 0x1000,
    "right": 0x2000,
    "down": 0x4000,
    "left": 0x8000,
}
