#!/usr/bin/env python3
"""双臂解算3D可视化 + 机器人遥操UDP发送端.

使用5个无线IMU实时解算双臂姿态, 以3D方式呈现机器人手臂的IK解算结果。
同时通过UDP将平滑后的关节角度发送到机器人端 (与 robot_arm_receiver.py 对接)。

功能:
  - 自动发现机器人 (监听 UDP 9528 端口的 G1RC 广播)
  - 或通过 --udp-target 直接指定机器人地址
  - 平滑输出 (限速3.5 rad/s) 防止电机突跳
  - 每臂独立跟随状态, SPACE 切换当前活跃臂的跟随

操作:
  R/L       - 切换活跃手臂 (右/左)
  SPACE     - 切换当前臂的跟随/暂停 (每臂独立)
  C         - 开始校准 (4步:下垂/前平举/侧平举/Roll)
  X         - 重置校准
  D         - 录制/停止录制
  鼠标拖拽  - 旋转视角
  滚轮      - 缩放
  Q/ESC     - 退出

依赖: pygame, PyOpenGL, numpy, scipy
"""

import importlib.util
import json
import math
import os
import sys
import threading
import time
from datetime import datetime

import numpy as np
import socket
import struct

# ===== 路径设置 =====
_MY_DIR = os.path.dirname(os.path.abspath(__file__))

# IMU API
_IMU_API_DIR = os.path.join(_MY_DIR, "IMU API")
_IMU_API_FILE = os.path.join(_IMU_API_DIR, "multi_imu_core.py")
if not os.path.exists(_IMU_API_FILE):
    raise FileNotFoundError(f"未找到无线IMU核心文件: {_IMU_API_FILE}")
_spec = importlib.util.spec_from_file_location("multi_imu_core", _IMU_API_FILE)
if _spec is None or _spec.loader is None:
    raise ImportError(f"无法加载IMU API: {_IMU_API_FILE}")
_multi_imu_core = importlib.util.module_from_spec(_spec)
sys.modules["multi_imu_core"] = _multi_imu_core
_spec.loader.exec_module(_multi_imu_core)
MultiImuService = _multi_imu_core.MultiImuService

from arm_calibration import (
    ArmDirectionCalibrator, TwistCalibration, IncrementalTracker,
    CALIB_POSES, T_CHEST2DISP,
    quat_mul as cal_quat_mul, quat_conj as cal_quat_conj,
    quat_rotate as cal_quat_rotate, normalize_quat as cal_normalize_quat,
    normalize_vec as cal_normalize_vec, average_quaternions as cal_average_quaternions,
    swing_twist_angle,
)
from arm_solver import (
    direction_to_joint_angles,
    forward_kinematics_right_arm_full,
    forward_kinematics_left_arm_full,
    clamp_joint_angles,
)
from robot_config import (
    JointAngles,
    RIGHT_UPPER_ARM_FIXED_ID,
    RIGHT_FOREARM_FIXED_ID,
    LEFT_UPPER_ARM_FIXED_ID,
    LEFT_FOREARM_FIXED_ID,
    CHEST_FIXED_ID,
    HUMAN_ARM_DEFAULT_LENGTH,
    get_joint_limits,
    get_motor_index,
)

import pygame
from OpenGL.GL import *
from OpenGL.GLU import *

# ===== UDP 协议 (与 robot_arm_receiver.py / headless_arm_sender.py 一致) =====
UDP_PACKET_HEADER = b'UARM'
UDP_PACKET_FMT = '!4sBffffffffff'
UDP_PACKET_SIZE = struct.calcsize(UDP_PACKET_FMT)
UDP_SEND_HZ = 50
UDP_SEND_DT = 1.0 / UDP_SEND_HZ

# 自动发现 (与 robot_arm_receiver.py 的广播一致)
DISCOVERY_LISTEN_PORT = 9528
DISCOVERY_TIMEOUT = 8.0
DEFAULT_ROBOT_PORT = 9527

# 安全限速
MAX_IK_OUTPUT_SPEED = 3.5   # rad/s
RETURN_SPEED = 0.5          # rad/s (回零)

def _pack_dual_arm(mode, r_sp, r_sr, r_sy, r_el, r_wr, l_sp, l_sr, l_sy, l_el, l_wr):
    return struct.pack(UDP_PACKET_FMT, UDP_PACKET_HEADER, mode,
                       r_sp, r_sr, r_sy, r_el, r_wr,
                       l_sp, l_sr, l_sy, l_el, l_wr)

# ===== 常量 =====
WINDOW_WIDTH = 1600
WINDOW_HEIGHT = 750
FPS = 60

ARM_LENGTH = HUMAN_ARM_DEFAULT_LENGTH["upper"]
FOREARM_LENGTH = HUMAN_ARM_DEFAULT_LENGTH["forearm"]
ARM_THICK = 0.055
FOREARM_THICK = 0.045
ROBOT_ARM_THICK = 0.030
ROBOT_FOREARM_THICK = 0.025
PALM_ARROW_LENGTH = 0.10

TORSO_HALF = (0.18, 0.10, 0.25)
HEAD_POS = (0.0, 0.0, 0.65)
HEAD_RADIUS = 0.08

T_CHEST2DISP = np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)

# URDF 帧 (X=forward, Y=left, Z=up) → Display 帧 (X=right, Y=forward, Z=up)
T_URDF2DISP = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)

CALIB_STATIC_DURATION = 3.0
CALIB_ROLL_DURATION = 5.0
WIRELESS_POLL_INTERVAL = 0.01
ROLE_MAP_FILE = os.path.join(_MY_DIR, "wireless_imu_roles.json")

# 肩膀在 display 帧中的位置 (对齐 upper_arm_3pose.py)
RIGHT_SHOULDER_DISP = np.array([0.20, 0.0, 0.48])
LEFT_SHOULDER_DISP = np.array([-0.20, 0.0, 0.48])

# 机器人手臂显示偏移 (与人体臂分开)
ROBOT_ARM_OFFSET_RIGHT = np.array([0.50, 0.0, 0.0])
ROBOT_ARM_OFFSET_LEFT = np.array([-0.50, 0.0, 0.0])

# 机器人手臂长度 (简化版 FK)
ROBOT_ARM_LEN = 0.20  # 上臂 + 前臂合并长度


def _compute_sp_and_sr(upper_dir_chest: np.ndarray, prev_sp: float | None = None, side: str = "right") -> tuple:
    """从上臂方向计算 shoulder_pitch 和 shoulder_roll.

    坐标系转换: chest IMU (X=forward, Y=right, Z=up) → URDF (X=forward, Y=left, Z=up)
    关键: Y 轴方向相反，需翻转 Y。

    返回: (sp, sr)
    """
    from arm_solver import (
        _UPPER_BONE_DIR_LOCAL,
        _LEFT_UPPER_BONE_DIR_LOCAL,
        SHOULDER_PITCH_ORIGIN_RPY_X,
        SHOULDER_ROLL_ORIGIN_RPY_X,
        LEFT_SHOULDER_PITCH_ORIGIN_RPY_X,
        LEFT_SHOULDER_ROLL_ORIGIN_RPY_X,
        rot_x, clamp,
    )
    from robot_config import get_joint_limits

    # 根据 side 选择正确的 origin RPY 和骨骼方向
    if side == "left":
        pitch_origin = LEFT_SHOULDER_PITCH_ORIGIN_RPY_X
        roll_origin = LEFT_SHOULDER_ROLL_ORIGIN_RPY_X
        bx, by, bz = _LEFT_UPPER_BONE_DIR_LOCAL
    else:
        pitch_origin = SHOULDER_PITCH_ORIGIN_RPY_X
        roll_origin = SHOULDER_ROLL_ORIGIN_RPY_X
        bx, by, bz = _UPPER_BONE_DIR_LOCAL

    # Chest → URDF: 翻转 Y (right→left)
    upper_dir_urdf = np.array([upper_dir_chest[0], -upper_dir_chest[1], upper_dir_chest[2]])
    ud = upper_dir_urdf / (np.linalg.norm(upper_dir_urdf) + 1e-12)

    # 补偿 URDF origin: rot_y(SP) @ rot_x(sr_total) @ bone = rot_x(-ORIGIN_SP) @ upper_dir
    target = rot_x(-pitch_origin) @ ud
    tx, ty, tz = float(target[0]), float(target[1]), float(target[2])

    # 从 Y 分量求 sr_total
    r_yz = float(np.sqrt(by * by + bz * bz))
    if r_yz < 1e-6:
        sr_total = 0.0
    else:
        delta = float(np.arctan2(-bz, by))
        acos_arg = float(np.clip(ty / r_yz, -1.0, 1.0))
        acos_val = float(np.arccos(acos_arg))

        sr_total_1 = delta + acos_val
        sr_total_2 = delta - acos_val

        limits = get_joint_limits(side)
        sr_min, sr_max = limits[f"{side}_shoulder_roll"]
        sr_1 = sr_total_1 - roll_origin
        sr_2 = sr_total_2 - roll_origin

        in_limits_1 = sr_min <= sr_1 <= sr_max
        in_limits_2 = sr_min <= sr_2 <= sr_max

        if in_limits_1 and in_limits_2:
            sr_total = sr_total_1 if abs(sr_1) < abs(sr_2) else sr_total_2
        elif in_limits_1:
            sr_total = sr_total_1
        elif in_limits_2:
            sr_total = sr_total_2
        else:
            sr_total = sr_total_1 if abs(sr_1 - (sr_min + sr_max) / 2) < abs(sr_2 - (sr_min + sr_max) / 2) else sr_total_2

    # Clamp SR
    sr = sr_total - roll_origin
    sr_clamped = clamp(sr, sr_min, sr_max)
    if abs(sr_clamped - sr) > 1e-6:
        sr_total = sr_clamped + roll_origin
    sr = sr_clamped

    # 从 XZ 分量求 SP
    vx = bx
    vz = by * np.sin(sr_total) + bz * np.cos(sr_total)
    sp = float(np.arctan2(tx, tz) - np.arctan2(vx, vz))
    sp = float((sp + np.pi) % (2 * np.pi) - np.pi)

    # 连续性处理
    if prev_sp is not None:
        d_sp = sp - prev_sp
        sp -= round(d_sp / (2 * np.pi)) * 2 * np.pi

    sp_min, sp_max = limits[f"{side}_shoulder_pitch"]
    sp = float(np.clip(sp, sp_min, sp_max))
    return sp, sr


def _compute_wrist_roll_from_palm(
    palm_dir_chest: np.ndarray,
    sp: float,
    sr: float,
    sy: float,
    el: float,
    prev_wr: float | None = None,
    side: str = "right",
) -> float:
    """从掌心朝向计算 wrist_roll.

    将人体掌心朝向转换到机器人腕关节局部坐标系,
    用 atan2 提取绕 X 轴的旋转角度.

    URDF 零位掌心方向: 右手 V_mesh=[0,1,0] (+Y向内), 左手 V_mesh=[0,-1,0] (-Y向内).
    wr_ref = atan2(V_mesh[2], V_mesh[1]) = 0 (右) 或 π (左).

    返回: wr (rad)
    """
    from arm_solver import (
        SHOULDER_PITCH_ORIGIN_RPY_X,
        SHOULDER_ROLL_ORIGIN_RPY_X,
        LEFT_SHOULDER_PITCH_ORIGIN_RPY_X,
        LEFT_SHOULDER_ROLL_ORIGIN_RPY_X,
        rot_x, rot_y, rot_z,
    )

    if side == "left":
        pitch_origin = LEFT_SHOULDER_PITCH_ORIGIN_RPY_X
        roll_origin = LEFT_SHOULDER_ROLL_ORIGIN_RPY_X
        wr_ref = np.pi  # V_mesh_left = [0, -1, 0]
    else:
        pitch_origin = SHOULDER_PITCH_ORIGIN_RPY_X
        roll_origin = SHOULDER_ROLL_ORIGIN_RPY_X
        wr_ref = 0.0  # V_mesh_right = [0, 1, 0]

    # Chest → URDF: 翻转 Y
    palm_urdf = np.array([palm_dir_chest[0], -palm_dir_chest[1], palm_dir_chest[2]])
    palm_urdf = palm_urdf / (np.linalg.norm(palm_urdf) + 1e-12)

    # 腕关节之前的旋转链
    R_before_wrist = (
        rot_x(pitch_origin)
        @ rot_y(sp)
        @ rot_x(roll_origin + sr)
        @ rot_z(sy)
        @ rot_y(el)
    )

    # 掌心转换到腕关节局部帧
    palm_local = R_before_wrist.T @ palm_urdf

    # WR = atan2(z, y) - wr_ref (wr_ref 由 URDF 零位掌心方向确定)
    wr = float(np.arctan2(palm_local[2], palm_local[1])) - wr_ref

    # 连续性
    if prev_wr is not None:
        d_wr = wr - prev_wr
        wr -= round(d_wr / (2 * np.pi)) * 2 * np.pi

    return wr


def _compute_sy_and_el(
    forearm_dir_chest: np.ndarray,
    sp: float,
    sr: float,
    prev_sy: float | None = None,
    prev_el: float | None = None,
    side: str = "right",
) -> tuple:
    """从前臂方向计算 shoulder_yaw 和 elbow.

    已知 SP 和 SR (大臂角度)，使用小臂方向求解 SY 和 EL。
    坐标系转换: chest → URDF (翻转 Y).

    返回: (sy, el)
    """
    from arm_solver import (
        _FOREARM_BONE_DIR_LOCAL,
        _LEFT_FOREARM_BONE_DIR_LOCAL,
        SHOULDER_PITCH_ORIGIN_RPY_X,
        SHOULDER_ROLL_ORIGIN_RPY_X,
        LEFT_SHOULDER_PITCH_ORIGIN_RPY_X,
        LEFT_SHOULDER_ROLL_ORIGIN_RPY_X,
        rot_x, rot_y, rot_z,
    )
    from robot_config import get_joint_limits

    # 根据 side 选择正确的 origin RPY 和骨骼方向
    if side == "left":
        pitch_origin = LEFT_SHOULDER_PITCH_ORIGIN_RPY_X
        roll_origin = LEFT_SHOULDER_ROLL_ORIGIN_RPY_X
        bx, by, bz = _LEFT_FOREARM_BONE_DIR_LOCAL
    else:
        pitch_origin = SHOULDER_PITCH_ORIGIN_RPY_X
        roll_origin = SHOULDER_ROLL_ORIGIN_RPY_X
        bx, by, bz = _FOREARM_BONE_DIR_LOCAL

    # Chest → URDF: 翻转 Y
    forearm_dir_urdf = np.array([forearm_dir_chest[0], -forearm_dir_chest[1], forearm_dir_chest[2]])
    forearm_dir_urdf = forearm_dir_urdf / (np.linalg.norm(forearm_dir_urdf) + 1e-12)

    # 肩部旋转矩阵 (SY=0)
    R_base = rot_x(pitch_origin) @ rot_y(sp) @ rot_x(roll_origin + sr)

    # target = 小臂在肩部局部坐标系中的方向
    target = R_base.T @ forearm_dir_urdf
    tx, ty, tz = float(target[0]), float(target[1]), float(target[2])

    # 从 Z 分量求 EL
    r = float(np.sqrt(bx * bx + bz * bz))
    if r < 1e-6:
        # 骨骼沿 Y 轴，用夹角近似
        el = float(np.arccos(np.clip(np.dot(forearm_dir_urdf, forearm_dir_urdf), -1.0, 1.0)))
        el -= 1.39626
        sy = float(np.arctan2(ty, tx) - np.arctan2(by, bx))
    else:
        phase = float(np.arctan2(bx, bz))
        tz_clamped = float(np.clip(tz / r, -1.0, 1.0))
        el_base = float(np.arccos(tz_clamped))

        # 两个候选解
        el_c1 = el_base - phase
        el_c2 = -el_base - phase

        # 肘关节范围
        limits = get_joint_limits(side)
        el_min, el_max = limits[f"{side}_elbow"]

        in_1 = el_min <= el_c1 <= el_max
        in_2 = el_min <= el_c2 <= el_max

        if in_1 and in_2:
            # 都在范围内，选择接近上一帧或接近零位的
            if prev_el is not None:
                el = el_c1 if abs(el_c1 - prev_el) < abs(el_c2 - prev_el) else el_c2
            else:
                el = el_c1 if abs(el_c1) < abs(el_c2) else el_c2
        elif in_1:
            el = el_c1
        elif in_2:
            el = el_c2
        else:
            # 都超限，选择偏离中值较小的
            el = el_c1 if abs(el_c1 - (el_min + el_max) / 2) < abs(el_c2 - (el_min + el_max) / 2) else el_c2

        # Clamp EL
        el = float(np.clip(el, el_min, el_max))

        # 从 XY 分量求 SY
        vx = bx * np.cos(el) + bz * np.sin(el)
        vy = by
        sy = float(np.arctan2(ty, tx) - np.arctan2(vy, vx))

    # SY 连续性处理
    if prev_sy is not None:
        d_sy = sy - prev_sy
        sy -= round(d_sy / (2 * np.pi)) * 2 * np.pi

    # SY 限位
    sy_min, sy_max = limits[f"{side}_shoulder_yaw"]
    sy = float(np.clip(sy, sy_min, sy_max))

    return sy, el


# ========================================================
#  _ArmState — 每臂状态
# ========================================================

class _ArmState:
    """单个手臂的状态管理 — 使用 4 步校准 (对齐 headless_arm_sender.py)."""

    def __init__(self, side):
        self.side = side
        # IMU 节点
        self.upper_node_id = None
        self.forearm_node_id = None
        self.upper_connected = False
        self.forearm_connected = False
        # IMU 四元数
        self.q_upper = np.array([1., 0., 0., 0.])
        self.q_forearm = np.array([1., 0., 0., 0.])
        # 校准器
        self.upper_calibrator = ArmDirectionCalibrator(label=f"{side}大臂")
        self.forearm_calibrator = ArmDirectionCalibrator(label=f"{side}小臂")
        self.upper_calibrated = False
        self.forearm_calibrated = False
        # 扭转
        self.twist_cal = None  # TwistCalibration
        # 增量跟踪
        self.upper_tracker = IncrementalTracker()
        self.forearm_tracker = IncrementalTracker()
        # 方向结果 (胸部坐标系)
        self.arm_dir_chest = np.array([0., 0., -1.])
        self.forearm_dir_chest = np.array([0., 0., -1.])
        self.forearm_twist = 0.0
        self.palm_dir_chest = np.array([0., 0., -1.])
        # WR 零位偏移由 URDF 固定 (wr_ref=0 右, π 左), 不存校准
        # IK 结果
        self.angles = JointAngles(0., 0., 0., 0., 0., side=side)
        # 发送角度 (平滑后, 安全输出)
        self.send_angles = JointAngles(0., 0., 0., 0., 0., side=side)
        # 跟随状态 (每臂独立)
        self.following = False
        # FK 结果
        self.robot_fk_urdf = None
        self.human_fk = None
        # 限位和电机映射
        self.joint_limits = get_joint_limits(side)
        self.motor_index = get_motor_index(side)
        self.limit_prefix = f"{side}_"
        # 显示方向 (IMU驱动的人体手臂方向)
        self.arm_dir_display = np.array([0., 0., -1.])
        self.forearm_dir_display = np.array([0., 0., -1.])
        self.palm_dir_display = np.array([0., 0., -1.])

    def reset(self):
        """重置校准, 保留 IMU 绑定"""
        self.upper_calibrator = ArmDirectionCalibrator(label=f"{self.side}大臂")
        self.forearm_calibrator = ArmDirectionCalibrator(label=f"{self.side}小臂")
        self.upper_calibrated = False
        self.forearm_calibrated = False
        self.twist_cal = None
        self.upper_tracker.reset()
        self.forearm_tracker.reset()
        self.arm_dir_chest = np.array([0., 0., -1.])
        self.forearm_dir_chest = np.array([0., 0., -1.])
        self.forearm_twist = 0.0
        self.palm_dir_chest = np.array([0., 0., -1.])
        self.angles = JointAngles(0., 0., 0., 0., 0., side=self.side)
        self.send_angles = JointAngles(0., 0., 0., 0., 0., side=self.side)
        self.following = False
        self.robot_fk_urdf = None
        self.human_fk = None


# ========================================================
#  DataRecorder — 数据录制
# ========================================================

class DataRecorder:
    """录制每帧的IMU原始数据、校准后数据、IK解算结果"""

    def __init__(self):
        self._frames = []
        self._recording = False
        self._t0 = None

    @property
    def recording(self):
        return self._recording

    @property
    def frame_count(self):
        return len(self._frames)

    def start(self):
        self._frames = []
        self._recording = True
        self._t0 = time.monotonic()
        return self._t0

    def stop(self):
        self._recording = False
        return self._frames

    def snapshot(self, q_chest, right, left):
        if not self._recording:
            return
        t = time.monotonic() - self._t0
        frame = {
            't': t,
            'q_chest': q_chest.copy(),
        }
        for arm in (right, left):
            s = arm.side
            frame[f'{s}_q_upper_raw'] = arm.q_upper.copy()
            frame[f'{s}_q_forearm_raw'] = arm.q_forearm.copy()
            frame[f'{s}_arm_dir_chest'] = arm.arm_dir_chest.copy()
            frame[f'{s}_forearm_dir_chest'] = arm.forearm_dir_chest.copy()
            frame[f'{s}_palm_dir_chest'] = arm.palm_dir_chest.copy()
            frame[f'{s}_angles'] = np.array([
                arm.angles.shoulder_pitch, arm.angles.shoulder_roll,
                arm.angles.shoulder_yaw, arm.angles.elbow, arm.angles.wrist_roll
            ])
            if arm.robot_fk_urdf is not None:
                frame[f'{s}_robot_elbow'] = arm.robot_fk_urdf['elbow'].copy()
                frame[f'{s}_robot_wrist'] = arm.robot_fk_urdf['wrist'].copy()
        self._frames.append(frame)

    def save(self, output_dir=None):
        if not self._frames:
            return None
        out_dir = output_dir or os.path.join(_MY_DIR, "recorded_data")
        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(out_dir, f"record_{ts}.npz")

        keys = list(self._frames[0].keys())
        arrays = {}
        for k in keys:
            arrays[k] = np.array([f[k] for f in self._frames])
        np.savez(path, **arrays)
        return path


# ========================================================
#  OpenGL 绘制函数
# ========================================================

def draw_box(hx, hy, hz, color, alpha=1.0):
    v = [
        (-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
        (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz),
    ]
    faces = [(0,1,2,3),(4,5,6,7),(0,1,5,4),(2,3,7,6),(0,3,7,4),(1,2,6,5)]
    shades = [0.75, 1.0, 0.85, 0.85, 0.65, 0.90]
    glBegin(GL_QUADS)
    for i, face in enumerate(faces):
        s = shades[i]
        glColor4f(color[0]*s, color[1]*s, color[2]*s, alpha)
        for vi in face:
            glVertex3f(*v[vi])
    glEnd()
    edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
    glColor4f(0,0,0, alpha*0.7)
    glLineWidth(1.5)
    glBegin(GL_LINES)
    for a, b in edges:
        glVertex3f(*v[a]); glVertex3f(*v[b])
    glEnd()


def draw_arm_box(length, thick, color, alpha=1.0):
    t = thick / 2.0
    v = [
        (0,-t,-t),(length,-t,-t),(length,t,-t),(0,t,-t),
        (0,-t,t),(length,-t,t),(length,t,t),(0,t,t),
    ]
    faces = [(0,1,2,3),(4,5,6,7),(0,1,5,4),(2,3,7,6),(0,3,7,4),(1,2,6,5)]
    shades = [0.75, 1.0, 0.85, 0.85, 0.65, 0.90]
    glBegin(GL_QUADS)
    for i, face in enumerate(faces):
        s = shades[i]
        glColor4f(color[0]*s, color[1]*s, color[2]*s, alpha)
        for vi in face:
            glVertex3f(*v[vi])
    glEnd()
    edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
    glColor4f(0,0,0, alpha*0.6)
    glLineWidth(1.5)
    glBegin(GL_LINES)
    for a, b in edges:
        glVertex3f(*v[a]); glVertex3f(*v[b])
    glEnd()


def draw_axes(length=0.5):
    glLineWidth(2.5)
    glBegin(GL_LINES)
    glColor3f(1, 0.15, 0.15); glVertex3f(0,0,0); glVertex3f(length,0,0)
    glColor3f(0.15, 1, 0.15); glVertex3f(0,0,0); glVertex3f(0,length,0)
    glColor3f(0.3, 0.3, 1); glVertex3f(0,0,0); glVertex3f(0,0,length)
    glEnd()


def draw_grid(size=1.0, step=0.1):
    glColor4f(0.25, 0.25, 0.30, 0.45)
    glLineWidth(0.5)
    glBegin(GL_LINES)
    n = int(size / step)
    for i in range(-n, n+1):
        x = i * step
        glVertex3f(x,-size,0); glVertex3f(x,size,0)
        glVertex3f(-size,x,0); glVertex3f(size,x,0)
    glEnd()


def draw_sphere(pos, radius, color, alpha=1.0):
    glPushMatrix()
    glTranslatef(*pos)
    glColor4f(*color, alpha)
    quad = gluNewQuadric()
    gluSphere(quad, radius, 14, 14)
    gluDeleteQuadric(quad)
    glPopMatrix()


def draw_arrow(start, direction, length, color, alpha=1.0):
    direction = np.array(direction, dtype=float)
    n = np.linalg.norm(direction)
    if n < 1e-10:
        return
    direction = direction / n
    end = np.array(start) + direction * length
    glLineWidth(3.0)
    glColor4f(*color, alpha)
    glBegin(GL_LINES)
    glVertex3f(*start)
    glVertex3f(*end)
    glEnd()
    head_len = length * 0.2
    head_angle = 25.0
    if abs(direction[0]) < 0.9:
        perp1 = np.cross(direction, [1, 0, 0])
    else:
        perp1 = np.cross(direction, [0, 1, 0])
    perp1 = perp1 / np.linalg.norm(perp1)
    perp2 = np.cross(direction, perp1)
    rad = np.radians(head_angle)
    for perp in [perp1, perp2]:
        offset = (perp * np.sin(rad) - direction * np.cos(rad)) * head_len
        glBegin(GL_LINES)
        glVertex3f(*end)
        glVertex3f(*(end + offset))
        glEnd()


def draw_text_2d(surface, text, pos, font, color=(255,255,255)):
    ts = font.render(text, True, color)
    surface.blit(ts, pos)


# ========================================================
#  DualArmViz — 主应用
# ========================================================

class DualArmViz:

    def __init__(self, udp_target=None):
        self.lock = threading.Lock()
        self._ik_lock = threading.Lock()

        # UDP 发送 + 自动发现
        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if udp_target:
            parts = udp_target.split(':')
            host = parts[0]
            port = int(parts[1]) if len(parts) > 1 else 9527
            self._udp_target = (host, port)
            self._auto_discovery = False
        else:
            self._udp_target = None
            self._auto_discovery = True  # 无指定目标时自动发现
        self._udp_last_send = 0.0

        # 自动发现状态
        self._discovery_running = False
        self._discovery_thread = None
        self.robot_ip = None
        self.robot_port = 9527
        self.robot_discovered = False
        self.robot_last_seen = 0.0

        # IMU
        self.imu_service = MultiImuService()
        self._wireless_running = False
        self._wireless_thread = None
        self.chest_node_id = None
        self.chest_connected = False
        self.q_chest = np.array([1., 0., 0., 0.])
        self.latest_devices = []
        self.sample_rate = 0.0
        self._sample_n = 0
        self._rate_t = time.monotonic()

        # 双臂
        self.right = _ArmState("right")
        self.left = _ArmState("left")
        self.active_arm = "right"

        # IK 线程 (仅用于 FK 计算)
        self._ik_running = False
        self._ik_thread = None
        self._ik_event = threading.Event()
        self._ik_input = None
        self._ik_result_right = None
        self._ik_result_left = None
        self._ik_count = 0
        self._ik_total_time = 0.0
        self._ik_avg_ms = 0.0

        # 4步校准状态
        self._calib_step = -1  # -1=未在校准, >=0=CALIB_POSES 索引
        self._calib_waiting = False  # True=等待用户按C开始采集, False=正在采集
        self._calib_arm = None
        self._calib_start = 0.0
        self._calib_samples_chest = []
        self._calib_samples_upper = []
        self._calib_samples_forearm = []
        self._pose_averages = []
        self._roll_upper_data = []
        self._roll_forearm_data = []

        # 录制
        self.recorder = DataRecorder()

        # 日志
        self._logs = []

        # 相机
        self._cam_rot_x = 20.0
        self._cam_rot_y = -30.0
        self._cam_dist = 2.0
        self._mouse_dragging = False
        self._mouse_last = (0, 0)

    @property
    def active(self):
        return self.right if self.active_arm == "right" else self.left

    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self._logs.append(f"[{ts}] {msg}")
        if len(self._logs) > 50:
            self._logs = self._logs[-50:]
        print(msg)

    # ---------- IMU 管理 ----------

    def _load_role_map(self):
        if not os.path.exists(ROLE_MAP_FILE):
            self._log(f"未找到 IMU 映射文件: {ROLE_MAP_FILE}")
            return
        try:
            with open(ROLE_MAP_FILE, 'r') as f:
                data = json.load(f)
            self.right.upper_node_id = data.get("right_upper_node_id")
            self.right.forearm_node_id = data.get("right_forearm_node_id")
            self.left.upper_node_id = data.get("left_upper_node_id")
            self.left.forearm_node_id = data.get("left_forearm_node_id")
            self.chest_node_id = data.get("chest_node_id")
            self._log(f"IMU映射: 胸={self.chest_node_id} "
                      f"右臂={self.right.upper_node_id}/{self.right.forearm_node_id} "
                      f"左臂={self.left.upper_node_id}/{self.left.forearm_node_id}")
        except Exception as e:
            self._log(f"加载映射失败: {e}")

    def _save_role_map(self):
        data = {
            "chest_node_id": self.chest_node_id,
            "right_upper_node_id": self.right.upper_node_id,
            "right_forearm_node_id": self.right.forearm_node_id,
            "left_upper_node_id": self.left.upper_node_id,
            "left_forearm_node_id": self.left.forearm_node_id,
            "chest_fixed_id": CHEST_FIXED_ID,
            "right_upper_fixed_id": RIGHT_UPPER_ARM_FIXED_ID,
            "right_forearm_fixed_id": RIGHT_FOREARM_FIXED_ID,
            "left_upper_fixed_id": LEFT_UPPER_ARM_FIXED_ID,
            "left_forearm_fixed_id": LEFT_FOREARM_FIXED_ID,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        with open(ROLE_MAP_FILE, 'w') as f:
            json.dump(data, f, indent=2)

    def _try_assign_by_fixed_id(self, devices):
        for d in devices:
            if d.device_id == RIGHT_UPPER_ARM_FIXED_ID and self.right.upper_node_id != d.node_id:
                self.right.upper_node_id = d.node_id
                self._log(f"自动识别右大臂: {d.node_id}")
            if d.device_id == RIGHT_FOREARM_FIXED_ID and self.right.forearm_node_id != d.node_id:
                self.right.forearm_node_id = d.node_id
                self._log(f"自动识别右小臂: {d.node_id}")
            if d.device_id == LEFT_UPPER_ARM_FIXED_ID and self.left.upper_node_id != d.node_id:
                self.left.upper_node_id = d.node_id
                self._log(f"自动识别左大臂: {d.node_id}")
            if d.device_id == LEFT_FOREARM_FIXED_ID and self.left.forearm_node_id != d.node_id:
                self.left.forearm_node_id = d.node_id
                self._log(f"自动识别左小臂: {d.node_id}")
            if d.device_id == CHEST_FIXED_ID and self.chest_node_id != d.node_id:
                self.chest_node_id = d.node_id
                self._log(f"自动识别胸部: {d.node_id}")

    def _read_imu(self, dev_dict, node_id):
        d = dev_dict.get(node_id) if node_id else None
        if d is None or not d.connected:
            return None, False
        q = cal_normalize_quat(np.array(d.quat, dtype=float))
        return q, True

    def _init_imu(self):
        self._load_role_map()
        self._log("启动无线 IMU 服务...")
        self.imu_service.start()
        self._wireless_running = True
        self._wireless_thread = threading.Thread(target=self._wireless_loop, daemon=True)
        self._wireless_thread.start()
        # 启动机器人自动发现
        self._start_discovery()

    def _stop_imu(self):
        self._wireless_running = False
        if self._wireless_thread is not None:
            self._wireless_thread.join(timeout=1.0)
        self.imu_service.stop()
        self._stop_discovery()

    # ---------- UDP 发送角度平滑 ----------

    def _update_send_angles(self, arm, target_angles, dt):
        """跟随时平滑角度更新, 限速防止突跳."""
        if not arm.upper_calibrated:
            return
        max_step = MAX_IK_OUTPUT_SPEED * dt
        sp = arm.send_angles.shoulder_pitch + np.clip(
            target_angles.shoulder_pitch - arm.send_angles.shoulder_pitch, -max_step, max_step)
        sr = arm.send_angles.shoulder_roll + np.clip(
            target_angles.shoulder_roll - arm.send_angles.shoulder_roll, -max_step, max_step)
        sy = arm.send_angles.shoulder_yaw + np.clip(
            target_angles.shoulder_yaw - arm.send_angles.shoulder_yaw, -max_step, max_step)
        el = arm.send_angles.elbow + np.clip(
            target_angles.elbow - arm.send_angles.elbow, -max_step, max_step)
        wr = arm.send_angles.wrist_roll + np.clip(
            target_angles.wrist_roll - arm.send_angles.wrist_roll, -max_step, max_step)
        arm.send_angles = clamp_joint_angles(JointAngles(sp, sr, sy, el, wr, side=arm.side))

    def _update_send_angles_return(self, arm, dt):
        """不跟随时缓慢回零 send_angles (仅本地状态, 不发送)."""
        ret_step = RETURN_SPEED * dt
        sp = arm.send_angles.shoulder_pitch - np.clip(arm.send_angles.shoulder_pitch, -ret_step, ret_step)
        sr = arm.send_angles.shoulder_roll - np.clip(arm.send_angles.shoulder_roll, -ret_step, ret_step)
        sy = arm.send_angles.shoulder_yaw - np.clip(arm.send_angles.shoulder_yaw, -ret_step, ret_step)
        el = arm.send_angles.elbow - np.clip(arm.send_angles.elbow, -ret_step, ret_step)
        wr = arm.send_angles.wrist_roll - np.clip(arm.send_angles.wrist_roll, -ret_step, ret_step)
        arm.send_angles = clamp_joint_angles(JointAngles(sp, sr, sy, el, wr, side=arm.side))

    # ---------- 机器人自动发现 ----------

    def _start_discovery(self):
        if not self._auto_discovery:
            self.robot_discovered = True
            return
        self._discovery_running = True
        self._discovery_thread = threading.Thread(target=self._discovery_loop, daemon=True)
        self._discovery_thread.start()
        self._log("搜索机器人 (UDP 9528)...")

    def _stop_discovery(self):
        self._discovery_running = False
        if self._discovery_thread is not None:
            self._discovery_thread.join(timeout=1.0)

    def _discovery_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(('', DISCOVERY_LISTEN_PORT))
        sock.settimeout(1.0)
        try:
            while self._discovery_running:
                try:
                    data, addr = sock.recvfrom(256)
                except socket.timeout:
                    if self.robot_discovered and time.monotonic() - self.robot_last_seen > DISCOVERY_TIMEOUT:
                        self.robot_discovered = False
                        self._log("机器人已离线")
                    continue
                except OSError:
                    break
                text = data.decode('utf-8', errors='ignore').strip()
                if not text.startswith('G1RC,'):
                    continue
                fields = {}
                for kv in text.split(',')[1:]:
                    if '=' not in kv:
                        continue
                    k, v = kv.split('=', 1)
                    fields[k] = v
                ip = fields.get('ip', addr[0])
                try:
                    port = int(fields.get('port', DEFAULT_ROBOT_PORT))
                except ValueError:
                    port = DEFAULT_ROBOT_PORT
                now = time.monotonic()
                if not self.robot_discovered or self.robot_ip != ip:
                    self._log(f"发现机器人: {ip}:{port}")
                self.robot_ip = ip
                self.robot_port = port
                self.robot_discovered = True
                self.robot_last_seen = now
        finally:
            sock.close()

    def _wireless_loop(self):
        _conn_log_interval = 5.0
        _conn_log_last = time.monotonic()

        while self._wireless_running:
            devices = self.imu_service.list_devices()
            dev_dict = {d.node_id: d for d in devices}
            now = time.monotonic()

            with self.lock:
                self.latest_devices = devices
                self._try_assign_by_fixed_id(devices)

                # 定期打印设备连接状态 (每5秒)
                if now - _conn_log_last >= _conn_log_interval:
                    _conn_log_last = now
                    n_dev = len(devices)
                    n_conn = sum(1 for d in devices if d.connected)
                    if n_dev > 0:
                        parts = []
                        for d in devices:
                            role = ""
                            if d.node_id == self.chest_node_id:
                                role = "(胸)"
                            elif d.node_id == self.right.upper_node_id:
                                role = "(右大臂)"
                            elif d.node_id == self.right.forearm_node_id:
                                role = "(右小臂)"
                            elif d.node_id == self.left.upper_node_id:
                                role = "(左大臂)"
                            elif d.node_id == self.left.forearm_node_id:
                                role = "(左小臂)"
                            status = "ON" if d.connected else "OFF"
                            parts.append(f"{d.node_id}{role}:{status}")
                        self._log(f"设备 [{n_conn}/{n_dev}]: {' '.join(parts)}")

                # 胸部
                q_c, self.chest_connected = self._read_imu(dev_dict, self.chest_node_id)
                if q_c is not None:
                    self.q_chest = q_c

                # 双臂
                for arm in (self.right, self.left):
                    q_u, arm.upper_connected = self._read_imu(dev_dict, arm.upper_node_id)
                    q_f, arm.forearm_connected = self._read_imu(dev_dict, arm.forearm_node_id)
                    if q_u is not None:
                        arm.q_upper = q_u
                    if q_f is not None:
                        arm.q_forearm = q_f

                    # 增量跟踪 + 方向重建
                    if arm.upper_calibrated:
                        arm.upper_tracker.update(self.q_chest, arm.q_upper)
                        if arm.upper_tracker.q_rel_inc is not None:
                            arm.arm_dir_chest = arm.upper_calibrator.reconstruct(
                                arm.upper_tracker.q_rel_inc)

                    if arm.forearm_calibrated:
                        arm.forearm_tracker.update(self.q_chest, arm.q_forearm)
                        if arm.forearm_tracker.q_rel_inc is not None:
                            arm.forearm_dir_chest = arm.forearm_calibrator.reconstruct(
                                arm.forearm_tracker.q_rel_inc)

                    # 扭转 + 掌心
                    if arm.twist_cal is not None and arm.forearm_tracker.q_rel_inc is not None:
                        arm.forearm_twist = arm.twist_cal.compute_forearm_twist(
                            arm.forearm_tracker.q_rel_inc)
                        arm.palm_dir_chest = arm.twist_cal.compute_palm_direction(
                            arm.forearm_tracker.q_rel_inc, arm.forearm_calibrator)

                    # 方向→关节角度 (URDF 坐标系解算)
                    if arm.upper_calibrated:
                        prev_sp = arm.angles.shoulder_pitch if arm.angles is not None else None
                        sp, sr = _compute_sp_and_sr(arm.arm_dir_chest, prev_sp, arm.side)
                        arm.angles.shoulder_pitch = sp
                        arm.angles.shoulder_roll = sr

                        # 小臂方向 → SY, EL
                        if arm.forearm_calibrated:
                            prev_sy = arm.angles.shoulder_yaw
                            prev_el = arm.angles.elbow
                            sy, el = _compute_sy_and_el(
                                arm.forearm_dir_chest, sp, sr, prev_sy, prev_el, arm.side)
                            arm.angles.shoulder_yaw = sy
                            arm.angles.elbow = el
                        else:
                            arm.angles.shoulder_yaw = 0.0
                            arm.angles.elbow = 0.0

                        # wrist_roll: 从掌心朝向计算 URDF 腕关节旋转角
                        # 将掌心转到腕关节局部帧, 用 atan2 提取绕 X 轴的旋转
                        if arm.forearm_calibrated and arm.twist_cal is not None:
                            prev_wr = arm.angles.wrist_roll
                            arm.angles.wrist_roll = _compute_wrist_roll_from_palm(
                                arm.palm_dir_chest, sp, sr, sy, el,
                                prev_wr=prev_wr, side=arm.side)
                        else:
                            arm.angles.wrist_roll = 0.0

                        # URDF FK 计算 (根据 arm.side 选择正确的 FK)
                        if arm.side == "left":
                            arm.robot_fk_urdf = forward_kinematics_left_arm_full(arm.angles)
                        else:
                            arm.robot_fk_urdf = forward_kinematics_right_arm_full(arm.angles)

                    # 显示方向
                    if arm.upper_calibrated:
                        arm.arm_dir_display = T_CHEST2DISP @ arm.arm_dir_chest
                        arm.forearm_dir_display = T_CHEST2DISP @ arm.forearm_dir_chest
                        arm.palm_dir_display = T_CHEST2DISP @ arm.palm_dir_chest

                # 采样率
                self._sample_n += 1
                dt = now - self._rate_t
                if dt >= 2.0:
                    self.sample_rate = self._sample_n / dt
                    self._sample_n = 0
                    self._rate_t = now

                # 平滑角度更新 (每臂独立跟随)
                for arm in (self.right, self.left):
                    if not arm.upper_calibrated:
                        continue
                    if arm.following:
                        self._update_send_angles(arm, arm.angles, UDP_SEND_DT)
                    else:
                        self._update_send_angles_return(arm, UDP_SEND_DT)

                # UDP 发送 (任一臂跟随时发送)
                r_follow = self.right.following and self.right.upper_calibrated
                l_follow = self.left.following and self.left.upper_calibrated
                if r_follow or l_follow:
                    target = self._udp_target or ((self.robot_ip, self.robot_port) if self.robot_discovered else None)
                    if target and (now - self._udp_last_send) >= UDP_SEND_DT:
                        self._udp_last_send = now
                        mode = (3 if r_follow and l_follow else
                                1 if r_follow else
                                2 if l_follow else 0)
                        pkt = _pack_dual_arm(mode,
                            self.right.send_angles.shoulder_pitch, self.right.send_angles.shoulder_roll,
                            self.right.send_angles.shoulder_yaw, self.right.send_angles.elbow,
                            self.right.send_angles.wrist_roll,
                            self.left.send_angles.shoulder_pitch, self.left.send_angles.shoulder_roll,
                            self.left.send_angles.shoulder_yaw, self.left.send_angles.elbow,
                            self.left.send_angles.wrist_roll)
                        try:
                            self._udp_sock.sendto(pkt, target)
                        except OSError:
                            pass

                # 校准数据收集
                if self._calib_step >= 0:
                    self._collect_calib_samples(now)

            time.sleep(WIRELESS_POLL_INTERVAL)

    # ---------- 校准 (4 步: 下垂/前平举/侧平举/Roll) ----------

    def _get_calib_arm_poses(self):
        """返回当前活跃手臂在 CALIB_POSES 中的 (start_idx, end_idx)."""
        if self.active_arm == "right":
            return 0, 3
        else:
            return 4, 7

    def start_calibration(self):
        """进入校准模式, 显示第一步姿态提示, 等待用户按C开始采集."""
        if self._calib_step >= 0:
            return  # 已在校准中
        arm = self.active
        arm.reset()

        # 检查必要 IMU 的连接状态
        missing = []
        if not self.chest_connected:
            missing.append("胸部IMU")
        if not arm.upper_connected:
            missing.append(f"{arm.side}大臂IMU")
        if not arm.forearm_connected:
            missing.append(f"{arm.side}小臂IMU")

        if missing:
            self._log(f"警告: 以下IMU未连接: {', '.join(missing)}")
            self._log("请等待IMU连接后再开始校准")
            # 仍然显示提示, 让用户知道下一步做什么

        start_idx, _ = self._get_calib_arm_poses()
        self._calib_step = start_idx
        self._calib_waiting = True  # 等待用户按C
        self._calib_arm = arm
        self._calib_start = 0.0
        self._calib_samples_chest = []
        self._calib_samples_upper = []
        self._calib_samples_forearm = []
        pose = CALIB_POSES[self._calib_step]
        self._log(f"校准步骤 1/4: 请摆出 [{pose['name']}], 按 C 开始采集")

    def _start_calib_collection(self):
        """用户按C后, 开始采集当前步骤的数据."""
        # 检查必要 IMU
        arm = self._calib_arm
        missing = []
        if not self.chest_connected:
            missing.append("胸部")
        if not arm.upper_connected:
            missing.append("大臂")
        if not arm.forearm_connected:
            missing.append("小臂")
        if missing:
            self._log(f"无法采集: {', '.join(missing)} IMU未连接, 等待连接后重试")
            return  # 不开始采集, 继续等待

        self._calib_waiting = False
        self._calib_start = time.monotonic()
        self._calib_samples_chest = []
        self._calib_samples_upper = []
        self._calib_samples_forearm = []
        pose = CALIB_POSES[self._calib_step]
        start_idx, _ = self._get_calib_arm_poses()
        step_num = self._calib_step - start_idx + 1
        if pose["dir"] is not None:
            self._log(f"  采集中: {pose['name']} ({CALIB_STATIC_DURATION:.0f}s)")
        else:
            self._log(f"  采集中: {pose['name']} ({CALIB_ROLL_DURATION:.0f}s)")

    def _collect_calib_samples(self, now):
        """在 lock 内调用: 收集校准样本. 仅在非等待状态时采集."""
        if self._calib_step < 0 or self._calib_waiting:
            return
        arm = self._calib_arm
        pose = CALIB_POSES[self._calib_step]

        # 收集样本
        if self.chest_connected:
            self._calib_samples_chest.append(self.q_chest.copy())
        if arm.upper_connected:
            self._calib_samples_upper.append(arm.q_upper.copy())
        if arm.forearm_connected:
            self._calib_samples_forearm.append(arm.q_forearm.copy())

        # 检查是否完成当前步骤
        is_roll_step = pose["dir"] is None
        duration = CALIB_ROLL_DURATION if is_roll_step else CALIB_STATIC_DURATION
        if now - self._calib_start >= duration:
            self._advance_calibration()

    def _advance_calibration(self):
        """在 lock 内调用: 完成当前校准步骤, 进入下一步."""
        arm = self._calib_arm
        pose = CALIB_POSES[self._calib_step]

        if pose["dir"] is not None:
            # 静态姿态: 保存平均四元数
            n = min(len(self._calib_samples_chest),
                    len(self._calib_samples_upper))
            if n < 10:
                self._log(f"样本不足 ({n}), 跳过此姿态")
            else:
                q_c = cal_average_quaternions(self._calib_samples_chest)
                q_u = cal_average_quaternions(self._calib_samples_upper)
                q_f = (cal_average_quaternions(self._calib_samples_forearm)
                       if len(self._calib_samples_forearm) >= 10 else q_u.copy())
                self._pose_averages.append((q_c, q_u, q_f))
                self._log(f"  收集 {n} 样本")
        else:
            # Roll 步骤: 收集 q_rel 序列
            if len(self._calib_samples_chest) >= 10:
                self._roll_upper_data = []
                self._roll_forearm_data = []
                for i in range(len(self._calib_samples_chest)):
                    q_c = self._calib_samples_chest[i]
                    q_u = self._calib_samples_upper[i] if i < len(self._calib_samples_upper) else None
                    q_f = self._calib_samples_forearm[i] if i < len(self._calib_samples_forearm) else None
                    if q_u is not None:
                        q_rel = cal_normalize_quat(cal_quat_mul(cal_quat_conj(q_c), q_u))
                        self._roll_upper_data.append(q_rel)
                    if q_f is not None:
                        q_rel = cal_normalize_quat(cal_quat_mul(cal_quat_conj(q_c), q_f))
                        self._roll_forearm_data.append(q_rel)
                self._log(f"  Roll 数据: {len(self._roll_upper_data)} 帧")

        # 下一步
        start_idx, end_idx = self._get_calib_arm_poses()
        next_step = self._calib_step + 1

        if next_step <= end_idx:
            # 进入下一步, 显示提示, 等待用户按C
            self._calib_step = next_step
            self._calib_waiting = True
            self._calib_start = 0.0
            self._calib_samples_chest = []
            self._calib_samples_upper = []
            self._calib_samples_forearm = []
            pose = CALIB_POSES[self._calib_step]
            step_num = self._calib_step - start_idx + 1
            self._log(f"校准步骤 {step_num}/4: 请摆出 [{pose['name']}], 按 C 开始采集")
        else:
            # 全部完成
            self._compute_calibration(arm, start_idx, end_idx)
            self._calib_step = -1

    def _compute_calibration(self, arm, start_idx, end_idx):
        """在 lock 内调用: 计算校准结果."""
        self._log(f"计算 {arm.side} 臂校准...")

        # 收集方向数据
        for i in range(len(self._pose_averages)):
            pose_idx = start_idx + i
            if pose_idx > end_idx or i >= 3:
                break
            q_c, q_u, q_f = self._pose_averages[i]
            pose = CALIB_POSES[pose_idx]
            if q_u is not None:
                arm.upper_calibrator.collect_pose(q_c, q_u, pose["dir"])
            if q_f is not None:
                arm.forearm_calibrator.collect_pose(q_c, q_f, pose["dir"])

        # Procrustes + Roll 联合优化
        try:
            if self._roll_upper_data:
                arm.upper_calibrator.calibrate_with_roll(self._roll_upper_data)
            else:
                arm.upper_calibrator.calibrate()
            arm.upper_calibrated = True
        except Exception as e:
            self._log(f"大臂校准失败: {e}")

        try:
            if self._roll_forearm_data:
                arm.forearm_calibrator.calibrate_with_roll(self._roll_forearm_data)
            else:
                arm.forearm_calibrator.calibrate()
            arm.forearm_calibrated = True
        except Exception as e:
            self._log(f"小臂校准失败: {e}")

        # 扭转校准 (第3个姿态 = 侧平举)
        if arm.forearm_calibrated and len(self._pose_averages) >= 3:
            q_c_3, _, q_f_3 = self._pose_averages[2]
            if q_f_3 is not None:
                q_rel_ref = cal_normalize_quat(cal_quat_mul(cal_quat_conj(q_c_3), q_f_3))
                twist_axis = cal_normalize_vec(
                    cal_quat_rotate(q_rel_ref, arm.forearm_calibrator.arm_body_dir))
                palm_chest_0 = np.array([0.0, 0.0, -1.0])
                palm_raw_0 = arm.forearm_calibrator.R_align.T @ palm_chest_0
                palm_in_imu = cal_normalize_vec(
                    cal_quat_rotate(cal_quat_conj(q_rel_ref), palm_raw_0))
                arm.twist_cal = TwistCalibration(q_rel_ref, twist_axis, palm_in_imu)
                self._log("扭转校准完成")

        # 初始化增量跟踪
        arm.upper_tracker.init(self.q_chest, arm.q_upper)
        arm.forearm_tracker.init(self.q_chest, arm.q_forearm)

        # WR 零位由 URDF 固定 (wr_ref=0 右, π 左), 无需校准计算

        # 清理
        self._pose_averages = []
        self._roll_upper_data = []
        self._roll_forearm_data = []

        self._log(f"{arm.side}臂校准完成!")
        self._save_calibration(arm)

    def _save_calibration(self, arm):
        fname = f"imu_motor_calibration_{arm.side}.json"
        path = os.path.join(_MY_DIR, fname)
        data = {
            "version": 2,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        if arm.upper_calibrated:
            data["upper"] = arm.upper_calibrator.save_dict()
        if arm.forearm_calibrated:
            data["forearm"] = arm.forearm_calibrator.save_dict()
        if arm.twist_cal is not None:
            data["twist"] = arm.twist_cal.save_dict()
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
        self._log(f"校准已保存: {fname}")

    def _load_calibration(self):
        loaded_any = False
        for arm in (self.right, self.left):
            fname = f"imu_motor_calibration_{arm.side}.json"
            path = os.path.join(_MY_DIR, fname)
            if not os.path.exists(path):
                continue
            try:
                with open(path, 'r') as f:
                    data = json.load(f)
            except Exception:
                continue

            if data.get("version") == 2:
                # 新格式 (arm_calibration)
                if "upper" in data:
                    arm.upper_calibrator.load_dict(data["upper"])
                    arm.upper_calibrated = True
                if "forearm" in data:
                    arm.forearm_calibrator.load_dict(data["forearm"])
                    arm.forearm_calibrated = True
                if "twist" in data:
                    arm.twist_cal = TwistCalibration.load_dict(
                        data["twist"], arm.forearm_calibrator)
                ts = data.get("timestamp", "未知")
                self._log(f"{arm.side}臂已加载校准 v2 (保存于 {ts})")
                loaded_any = True
            elif "heading_rot" in data:
                # 旧格式兼容提示
                self._log(f"{arm.side}臂校准文件为旧格式, 请重新校准")
        return loaded_any

    # ---------- IK (已内联到 _wireless_loop) ----------

    def _start_ik_thread(self):
        pass  # 不再需要单独的 IK 线程

    def _stop_ik_thread(self):
        pass

    # ---------- 3D 渲染 ----------

    @staticmethod
    def _apply_dir_rotation(direction):
        ref = np.array([1.0, 0.0, 0.0])
        dot = float(np.clip(np.dot(ref, direction), -1.0, 1.0))
        if dot > 0.9999:
            return
        if dot < -0.9999:
            glRotatef(180.0, 0, 0, 1)
            return
        axis = np.cross(ref, direction)
        axis /= np.linalg.norm(axis)
        angle = np.degrees(np.arccos(dot))
        glRotatef(angle, *axis)

    def _draw_human_arm(self, shoulder_pos, arm, alpha=1.0):
        """绘制 IMU 驱动的人体手臂 (对齐 upper_arm_3pose.py 风格)."""
        arm_color = (0.20, 0.72, 0.35) if arm.upper_calibrated else (0.72, 0.35, 0.20)
        fore_color = (0.25, 0.50, 0.80) if arm.forearm_calibrated else (0.60, 0.40, 0.25)

        draw_sphere(shoulder_pos, 0.04, arm_color, alpha * 0.95)

        arm_n = arm.arm_dir_display / (np.linalg.norm(arm.arm_dir_display) + 1e-12)
        elbow_pos = shoulder_pos + arm_n * ARM_LENGTH
        glPushMatrix()
        glTranslatef(*shoulder_pos)
        self._apply_dir_rotation(arm_n)
        draw_arm_box(ARM_LENGTH, ARM_THICK, arm_color, alpha=0.85)
        glPopMatrix()

        draw_sphere(elbow_pos, 0.025, (0.92, 0.62, 0.20), alpha * 0.9)

        fore_n = arm.forearm_dir_display / (np.linalg.norm(arm.forearm_dir_display) + 1e-12)
        wrist_pos = elbow_pos + fore_n * FOREARM_LENGTH
        glPushMatrix()
        glTranslatef(*elbow_pos)
        self._apply_dir_rotation(fore_n)
        draw_arm_box(FOREARM_LENGTH, FOREARM_THICK, fore_color, alpha=0.85)
        glPopMatrix()

        draw_sphere(wrist_pos, 0.022, (0.92, 0.62, 0.20), alpha * 0.85)

        if arm.twist_cal is not None:
            draw_arrow(wrist_pos, arm.palm_dir_display, PALM_ARROW_LENGTH,
                       (1.0, 0.85, 0.1), alpha=0.95)

    def _draw_robot_arm(self, fk_urdf, side, alpha=1.0):
        """绘制 URDF FK 驱动的机器人手臂.

        fk_urdf 包含 URDF 坐标系下的关节位置，使用 T_URDF2DISP 转换到 display。
        绘制: shoulder_pitch → shoulder_roll → shoulder_yaw → elbow → wrist
        """
        if fk_urdf is None:
            return
        offset = ROBOT_ARM_OFFSET_RIGHT if side == "right" else ROBOT_ARM_OFFSET_LEFT

        # URDF → Display
        sp_pos = T_URDF2DISP @ fk_urdf["shoulder_pitch"][:3] + offset
        sr_pos = T_URDF2DISP @ fk_urdf["shoulder_roll"][:3] + offset
        sy_pos = T_URDF2DISP @ fk_urdf["shoulder_yaw"][:3] + offset
        el_pos = T_URDF2DISP @ fk_urdf["elbow"][:3] + offset
        wr_pos = T_URDF2DISP @ fk_urdf["wrist"][:3] + offset

        # shoulder_pitch 关节球 (红色)
        draw_sphere(sp_pos, 0.022, (0.90, 0.30, 0.20), alpha * 0.95)

        # 连杆: shoulder_pitch → shoulder_roll (第一个电机)
        d1 = sr_pos - sp_pos
        l1 = np.linalg.norm(d1)
        if l1 > 1e-6:
            d1_n = d1 / l1
            glPushMatrix()
            glTranslatef(*sp_pos)
            self._apply_dir_rotation(d1_n)
            draw_arm_box(l1, ROBOT_ARM_THICK * 0.8, (0.60, 0.45, 0.25), alpha * 0.7)
            glPopMatrix()

        # shoulder_roll 关节球 (紫色)
        draw_sphere(sr_pos, 0.020, (0.70, 0.30, 0.80), alpha * 0.95)

        # 连杆: shoulder_roll → shoulder_yaw (第二个电机)
        d2 = sy_pos - sr_pos
        l2 = np.linalg.norm(d2)
        if l2 > 1e-6:
            d2_n = d2 / l2
            glPushMatrix()
            glTranslatef(*sr_pos)
            self._apply_dir_rotation(d2_n)
            draw_arm_box(l2, ROBOT_ARM_THICK * 0.8, (0.60, 0.45, 0.25), alpha * 0.7)
            glPopMatrix()

        # shoulder_yaw 关节球 (橙色)
        draw_sphere(sy_pos, 0.018, (0.95, 0.60, 0.20), alpha * 0.95)

        # 连杆: shoulder_yaw → elbow (大臂骨骼)
        d3 = el_pos - sy_pos
        l3 = np.linalg.norm(d3)
        if l3 > 1e-6:
            d3_n = d3 / l3
            glPushMatrix()
            glTranslatef(*sy_pos)
            self._apply_dir_rotation(d3_n)
            draw_arm_box(l3, ROBOT_ARM_THICK, (0.55, 0.55, 0.60), alpha * 0.75)
            glPopMatrix()

        # elbow 关节球 (绿色)
        draw_sphere(el_pos, 0.020, (0.20, 0.75, 0.30), alpha * 0.9)

        # 连杆: elbow → wrist (小臂骨骼)
        d4 = wr_pos - el_pos
        l4 = np.linalg.norm(d4)
        if l4 > 1e-6:
            d4_n = d4 / l4
            glPushMatrix()
            glTranslatef(*el_pos)
            self._apply_dir_rotation(d4_n)
            draw_arm_box(l4, ROBOT_FOREARM_THICK, (0.50, 0.50, 0.70), alpha * 0.75)
            glPopMatrix()

        # wrist 关节球 (蓝色)
        draw_sphere(wr_pos, 0.018, (0.30, 0.50, 0.90), alpha * 0.85)

        # 掌心朝向箭头 (从 R_forearm 提取)
        if "R_forearm" in fk_urdf:
            R_forearm = fk_urdf["R_forearm"]
            # 机器人掌心方向: R_forearm 的某一列 (取 Z 列作为掌心朝向)
            palm_urdf = R_forearm[:, 2]
            palm_disp = T_URDF2DISP @ palm_urdf
            palm_disp = palm_disp / (np.linalg.norm(palm_disp) + 1e-12)
            draw_arrow(wr_pos, palm_disp, PALM_ARROW_LENGTH * 0.7,
                       (0.3, 0.7, 1.0), alpha * 0.9)

    def _draw_torso(self):
        """绘制躯干 (对齐 upper_arm_3pose.py 尺寸)."""
        draw_box(*TORSO_HALF, (0.45, 0.40, 0.50), alpha=0.40)
        draw_sphere(HEAD_POS, HEAD_RADIUS, (0.70, 0.60, 0.55), 0.70)

    def _render_scene(self, right, left):
        """渲染主3D场景 (全屏)."""
        glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT)
        glClearColor(0.10, 0.10, 0.16, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_PROJECTION); glLoadIdentity()
        gluPerspective(45, WINDOW_WIDTH / WINDOW_HEIGHT, 0.05, 20.0)
        glMatrixMode(GL_MODELVIEW); glLoadIdentity()

        cx = self._cam_dist * math.sin(math.radians(self._cam_rot_y)) * math.cos(math.radians(self._cam_rot_x))
        cy = self._cam_dist * math.cos(math.radians(self._cam_rot_y)) * math.cos(math.radians(self._cam_rot_x))
        cz = self._cam_dist * math.sin(math.radians(self._cam_rot_x))
        gluLookAt(cx, cy, cz + 0.3, 0, 0, 0.30, 0, 0, 1)

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        draw_grid(1.5, 0.1)
        draw_axes(0.35)

        self._draw_torso()

        r_active = self.active_arm == "right"
        r_alpha = 1.0 if r_active else 0.5
        l_alpha = 1.0 if not r_active else 0.5

        # 人体手臂
        self._draw_human_arm(RIGHT_SHOULDER_DISP, right, r_alpha)
        self._draw_human_arm(LEFT_SHOULDER_DISP, left, l_alpha)

        # 机器人手臂 (偏移显示)
        if right.robot_fk_urdf is not None:
            self._draw_robot_arm(right.robot_fk_urdf, "right", r_alpha)
        if left.robot_fk_urdf is not None:
            self._draw_robot_arm(left.robot_fk_urdf, "left", l_alpha)

    def _render_hud(self, screen, font, font_sm, font_lg, right, left):
        """渲染2D叠加文字"""
        surface = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)

        # 状态
        y = 5
        arm_label = "右臂" if self.active_arm == "right" else "左臂"
        r_state = "跟随" if self.right.following else "暂停"
        l_state = "跟随" if self.left.following else "暂停"
        rec_str = f"  [REC {self.recorder.frame_count}帧]" if self.recorder.recording else ""
        draw_text_2d(surface, f"活跃: {arm_label}  |  右:{r_state} 左:{l_state}{rec_str}",
                     (5, y), font, (255, 255, 100))

        # IMU 状态
        y += 25
        n_imu = sum(1 for d in self.latest_devices if d.connected) if self.latest_devices else 0
        imu_str = f"IMU: {n_imu}个 ({self.sample_rate:.0f}Hz)"
        # 机器人连接状态
        target = self._udp_target or ((self.robot_ip, self.robot_port) if self.robot_discovered else None)
        if target:
            robot_str = f"  → 机器人 {target[0]}:{target[1]} [已连接]" if self.robot_discovered or self._udp_target else f"  → 搜索机器人中..."
            robot_color = (100, 255, 100) if (self.robot_discovered or self._udp_target) else (255, 200, 100)
        else:
            robot_str = ""
            robot_color = (200, 200, 200)
        draw_text_2d(surface, imu_str + robot_str, (5, y), font_sm, robot_color)

        # 每臂状态
        y += 20
        for arm in (self.right, self.left):
            label = "右" if arm.side == "right" else "左"
            conn = "O" if (arm.upper_connected and arm.forearm_connected) else "X"
            if self._calib_step >= 0 and self._calib_arm is arm:
                if self._calib_waiting:
                    cal = "校准等待中"
                else:
                    pose = CALIB_POSES[self._calib_step]
                    start_idx, _ = self._get_calib_arm_poses()
                    step_num = self._calib_step - start_idx + 1
                    remaining = max(0, (CALIB_ROLL_DURATION if pose["dir"] is None else CALIB_STATIC_DURATION)
                                    - (time.monotonic() - self._calib_start))
                    cal = f"采集中 {step_num}/4 {remaining:.1f}s"
                color = (255, 255, 100)
            elif arm.upper_calibrated:
                cal = "已校准"
                color = (100, 255, 100)
            else:
                cal = "未校准"
                color = (255, 150, 100)
            draw_text_2d(surface, f"{label}臂 [{conn}] {cal}", (5, y), font_sm, color)
            y += 18

            if arm.upper_calibrated:
                sp_d = np.degrees(arm.angles.shoulder_pitch)
                sr_d = np.degrees(arm.angles.shoulder_roll)
                sy_d = np.degrees(arm.angles.shoulder_yaw)
                el_d = np.degrees(arm.angles.elbow)
                wr_d = np.degrees(arm.angles.wrist_roll)
                draw_text_2d(surface,
                    f"  SP:{sp_d:+.0f}° SR:{sr_d:+.0f}° SY:{sy_d:+.0f}° EL:{el_d:+.0f}° WR:{wr_d:+.0f}°",
                    (5, y), font_sm, (180, 220, 180))
                y += 18

        # 按键提示
        y = WINDOW_HEIGHT - 22
        if self._calib_step >= 0:
            hint = "C:开始采集  X:取消校准"
        else:
            hint = "R:右臂 L:左臂 SPACE:跟随 C:校准(4步) X:重置 D:录制 鼠标拖拽:旋转 滚轮:缩放 Q:退出"
        draw_text_2d(surface, hint, (5, y), font_sm, (130, 130, 140))

        # 日志
        y_log = WINDOW_HEIGHT - 45
        for msg in self._logs[-3:]:
            draw_text_2d(surface, msg, (5, y_log), font_sm, (200, 200, 100))
            y_log -= 16

        # ---- 校准居中提示 ----
        if self._calib_step >= 0 and self._calib_arm is not None:
            pose = CALIB_POSES[self._calib_step]
            start_idx, _ = self._get_calib_arm_poses()
            step_num = self._calib_step - start_idx + 1
            arm = self._calib_arm

            # 检查 IMU 连接状态
            imu_ok = self.chest_connected and arm.upper_connected and arm.forearm_connected

            box_w = 560
            box_h = 200 if not imu_ok else 160
            box_x = (WINDOW_WIDTH - box_w) // 2
            box_y = (WINDOW_HEIGHT - box_h) // 2
            pygame.draw.rect(surface, (0, 0, 0, 200), (box_x, box_y, box_w, box_h), border_radius=12)
            border_color = (100, 180, 255, 220) if imu_ok else (255, 100, 80, 220)
            pygame.draw.rect(surface, border_color, (box_x, box_y, box_w, box_h), 2, border_radius=12)

            ty = box_y + 15
            draw_text_2d(surface, f"校准步骤 {step_num}/4", (box_x + 20, ty), font_lg, (100, 200, 255))
            ty += 40

            draw_text_2d(surface, f"请摆出: {pose['name']}",
                         (box_x + 20, ty), font_lg, (255, 255, 255))
            ty += 40

            if not imu_ok:
                # 显示缺失 IMU
                missing = []
                if not self.chest_connected:
                    missing.append("胸部")
                if not arm.upper_connected:
                    missing.append("大臂")
                if not arm.forearm_connected:
                    missing.append("小臂")
                draw_text_2d(surface, f"等待IMU连接: {', '.join(missing)}",
                             (box_x + 20, ty), font_sm, (255, 120, 80))
                ty += 20
                draw_text_2d(surface, "连接后按 C 开始采集",
                             (box_x + 20, ty), font_sm, (200, 200, 200))
            elif self._calib_waiting:
                # 闪烁提示
                blink = int(time.monotonic() * 2) % 2 == 0
                if blink:
                    draw_text_2d(surface, "按 C 开始采集",
                                 (box_x + 20, ty), font_lg, (255, 255, 100))
            else:
                # 采集进度条
                is_roll = pose["dir"] is None
                duration = CALIB_ROLL_DURATION if is_roll else CALIB_STATIC_DURATION
                elapsed = time.monotonic() - self._calib_start
                progress = min(1.0, elapsed / duration)
                remaining = max(0.0, duration - elapsed)

                bar_x = box_x + 20
                bar_w = box_w - 40
                bar_h = 24
                pygame.draw.rect(surface, (60, 60, 80, 200),
                                 (bar_x, ty, bar_w, bar_h), border_radius=6)
                fill_w = int(bar_w * progress)
                if fill_w > 0:
                    pygame.draw.rect(surface, (80, 200, 120, 220),
                                     (bar_x, ty, fill_w, bar_h), border_radius=6)
                draw_text_2d(surface, f"采集中 {remaining:.1f}s",
                             (bar_x + bar_w // 2 - 50, ty + 2), font_sm, (255, 255, 255))

        # 叠加到 OpenGL
        tex_data = pygame.image.tostring(surface, "RGBA", True)
        w, h = surface.get_size()
        glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT)
        glMatrixMode(GL_PROJECTION); glLoadIdentity()
        glOrtho(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT, -1, 1)
        glMatrixMode(GL_MODELVIEW); glLoadIdentity()
        glDisable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_TEXTURE_2D)

        tex_id = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, tex_id)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, tex_data)

        glColor4f(1, 1, 1, 1)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 0); glVertex2f(0, 0)
        glTexCoord2f(1, 0); glVertex2f(w, 0)
        glTexCoord2f(1, 1); glVertex2f(w, h)
        glTexCoord2f(0, 1); glVertex2f(0, h)
        glEnd()

        glDeleteTextures([tex_id])
        glDisable(GL_TEXTURE_2D)
        glEnable(GL_DEPTH_TEST)

    # ---------- 主循环 ----------

    def run(self):
        pygame.init()
        screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.DOUBLEBUF | pygame.OPENGL)
        pygame.display.set_caption("双臂解算3D可视化")

        font = pygame.font.SysFont("notosanscjksc,notosanscjk,wenquanyimicrohei,simhei,monospace", 18, bold=True)
        font_sm = pygame.font.SysFont("notosanscjksc,notosanscjk,wenquanyimicrohei,simhei,monospace", 14)
        font_lg = pygame.font.SysFont("notosanscjksc,notosanscjk,wenquanyimicrohei,simhei,monospace", 28, bold=True)

        # 初始化
        self._load_calibration()
        self._init_imu()
        self._start_ik_thread()
        self._save_role_map()

        running = True
        clock = pygame.time.Clock()

        try:
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key in (pygame.K_q, pygame.K_ESCAPE):
                            running = False
                        elif event.key == pygame.K_r:
                            self.active_arm = "right"
                            self._log("切换到右臂")
                        elif event.key == pygame.K_l:
                            self.active_arm = "left"
                            self._log("切换到左臂")
                        elif event.key == pygame.K_SPACE:
                            self.active.following = not self.active.following
                            state = "跟随" if self.active.following else "暂停"
                            self._log(f"{self.active.side}臂: {state}")
                        elif event.key == pygame.K_c:
                            if self._calib_step < 0:
                                self.start_calibration()
                            elif self._calib_waiting:
                                self._start_calib_collection()
                        elif event.key == pygame.K_x:
                            with self.lock:
                                self.right.reset()
                                self.left.reset()
                                self._calib_step = -1
                                self._calib_waiting = False
                                self._pose_averages = []
                            self._log("双臂校准已重置")
                        elif event.key == pygame.K_d:
                            if self.recorder.recording:
                                path = self.recorder.save()
                                n = self.recorder.frame_count
                                self._log(f"录制完成: {n}帧 → {path}")
                            else:
                                self.recorder.start()
                                self._log("开始录制...")
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        if event.button == 1:
                            self._mouse_dragging = True
                            self._mouse_last = event.pos
                        elif event.button == 4:
                            self._cam_dist = max(0.5, self._cam_dist - 0.1)
                        elif event.button == 5:
                            self._cam_dist = min(5.0, self._cam_dist + 0.1)
                    elif event.type == pygame.MOUSEBUTTONUP:
                        if event.button == 1:
                            self._mouse_dragging = False
                    elif event.type == pygame.MOUSEMOTION:
                        if self._mouse_dragging:
                            dx = event.pos[0] - self._mouse_last[0]
                            dy = event.pos[1] - self._mouse_last[1]
                            self._cam_rot_y += dx * 0.5
                            self._cam_rot_x = max(-89, min(89, self._cam_rot_x + dy * 0.5))
                            self._mouse_last = event.pos

                # 录制
                with self.lock:
                    self.recorder.snapshot(self.q_chest, self.right, self.left)

                    # 渲染快照
                    right_snap = self.right
                    left_snap = self.left

                self._render_scene(right_snap, left_snap)
                self._render_hud(screen, font, font_sm, font_lg, right_snap, left_snap)

                pygame.display.flip()
                clock.tick(FPS)

        finally:
            self._stop_ik_thread()
            self._stop_imu()
            if self.recorder.recording:
                path = self.recorder.save()
                self._log(f"录制已保存: {path}")
            pygame.quit()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="双臂解算3D可视化 + 机器人遥操发送")
    parser.add_argument("--udp-target", default=None,
                        help="直接指定机器人地址 (如 192.168.1.100:9527), 省略则自动发现")
    args = parser.parse_args()
    app = DualArmViz(udp_target=args.udp_target)
    if args.udp_target:
        print(f"UDP 直连模式 → {args.udp_target}")
    else:
        print("UDP 自动发现模式 (监听机器人广播 G1RC...)")
    app.run()
