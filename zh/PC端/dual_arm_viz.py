#!/usr/bin/env python3
"""双臂解算3D可视化 + 机器人遥操UDP发送端.

使用5个无线IMU实时解算双臂姿态, 以3D方式呈现机器人手臂的IK解算结果。
同时通过UDP将平滑后的关节角度发送到机器人端 (与 robot_arm_receiver.py 对接)。

功能:
  - 自动发现机器人 (监听 UDP 9528 端口的 G1RC 广播)
  - 或通过 --udp-target 直接指定机器人地址
  - 默认 50 Hz 机器人目标输出，与用户指定的原始流畅版一致
  - 每臂独立跟随状态, SPACE 切换当前活跃臂的跟随

操作:
  R/L       - 切换活跃手臂 (右/左)
  SPACE     - 切换当前臂的跟随/暂停 (每臂独立)
  C         - 开始校准 (7步:3个静态姿势/肘屈伸/掌心朝上/朝下/验证)
  X         - 重置校准
  D         - 录制/停止录制
  鼠标拖拽  - 旋转视角
  滚轮      - 缩放
  Q/ESC     - 退出

依赖: pygame, PyOpenGL, numpy, scipy
"""

import importlib.util
import copy
import csv
import json
import math
import os
import queue
import sys
import threading
import time
from datetime import datetime
from types import SimpleNamespace

import numpy as np
import socket

# ===== 路径设置 =====
_MY_DIR = os.path.dirname(os.path.abspath(__file__))
_DEPLOY_DIR = os.path.join(os.path.dirname(_MY_DIR), "部署", "树莓派部署")
sys.path.insert(0, _DEPLOY_DIR)

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
    T_CHEST2DISP,
    quat_mul as cal_quat_mul, quat_conj as cal_quat_conj,
    quat_rotate as cal_quat_rotate, normalize_quat as cal_normalize_quat,
    normalize_vec as cal_normalize_vec,
    average_relative_quaternions, quaternion_angle_deg,
    quaternion_dispersion_deg, fit_hinge_axis, swing_twist_angle,
    palm_pose_consistency_deg,
)
from waist_tracker import (
    WaistTracker,
    DEFAULT_MAX_YAW_RAD as WAIST_DEFAULT_MAX_YAW_RAD,
    DEFAULT_MAX_SPEED_RAD_S as WAIST_DEFAULT_MAX_SPEED,
    DEFAULT_GAIN as WAIST_DEFAULT_GAIN,
)
from arm_solver import (
    direction_to_joint_angles,
    solve_direction_ik,
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
from single_imu_mapper import RightArmTwoImuMapper, SingleForearmMapper
from teleop_protocol import pack_arm_command
import hand_mapping
import inspire_hand_ctrl
from glove_source import GloveSource

import pygame
from OpenGL.GL import *
from OpenGL.GLU import *

from g1_urdf_renderer import G1UrdfRenderer
from human_gltf_renderer import HumanGltfRenderer

# ===== UDP 协议 (机器人端兼容旧 UARM 包，新版发送 UA2M) =====
UDP_SEND_HZ = 50
UDP_SEND_DT = 1.0 / UDP_SEND_HZ
IK_SOLVE_HZ = 100
IK_SOLVE_DT = 1.0 / IK_SOLVE_HZ

# 自动发现 (与 robot_arm_receiver.py 的广播一致)
DISCOVERY_LISTEN_PORT = 9528
DISCOVERY_TIMEOUT = 8.0
DEFAULT_ROBOT_PORT = 9527

# PC 端只拦截不可能的单包突跳，正常速度限制统一由机器人端执行。
PC_PACKET_JUMP_GUARD_RAD = 0.70
COMMAND_VELOCITY_LIMIT = 8.0
COMMAND_VELOCITY_CUTOFF_HZ = 18.0

# ===== 常量 =====
WINDOW_WIDTH = 1600
WINDOW_HEIGHT = 750
# 左列 TRACKING 卡到 y=512，ACTIVITY 贴着底部快捷键栏，两者不能相撞。
MIN_WINDOW_WIDTH = 960
MIN_WINDOW_HEIGHT = 720
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
_MODELS_DIR = os.path.join(os.path.dirname(_MY_DIR), "仿真", "models")
# 全身 29-DOF, 宇树官方 URDF (见同目录 SOURCE.md)。
G1_URDF_PATH = os.path.join(_MODELS_DIR, "g1_29dof", "g1_29dof.urdf")
# 带因时 FTP 灵巧手的同一台机器人, --hand 时使用。除双手外与上面那份一致,
# 同一上游同一修订, 见 仿真/models/g1_29dof/SOURCE.md。
G1_HAND_URDF_PATH = os.path.join(
    _MODELS_DIR, "g1_29dof", "g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf")
# 右手仿真用的 Inspire 配置, 与 mhandpro_diagnostic 读的是同一份文件,
# 端点因此不可能对不上。见 mhandpro/config/inspire_right_sim.cfg 的说明。
DEFAULT_HAND_CFG = os.path.join(
    os.path.dirname(_MY_DIR), "mhandpro", "config", "inspire_right_sim.cfg")
# 左手同理。两侧端口必须不同(9103/9104), 否则两只手会收到同一份数据 ——
# 那份配置里写了原因。
DEFAULT_HAND_LEFT_CFG = os.path.join(
    os.path.dirname(_MY_DIR), "mhandpro", "config", "inspire_left_sim.cfg")
# 旧的上半身模型, --robot-model arms 时使用。
G1_ARMS_URDF_PATH = os.path.join(
    _MODELS_DIR, "g1_description", "g1_dual_arm.urdf")
# 两个模型沿"垂直于默认镜头视线"的方向左右分开, 而不是沿显示系 X 轴。
# 默认方位角 -30° 时, 沿 X 分开会让 +X 那侧的模型远 24%, 透视再缩小 0.81×,
# 两个等高的模型在屏幕上会差出一大截。这样摆两者到镜头都是约 2.07 m。
_LAYOUT_LOOKAT = np.array([0.05, 0.0])
_LAYOUT_PERP = np.array([0.866, 0.5])     # 默认 _cam_rot_y=-30° 视线的水平垂线
_LAYOUT_HALF = 0.465                      # 两个模型间距 0.93 m

def _layout_offset(sign, z):
    xy = _LAYOUT_LOOKAT + sign * _LAYOUT_HALF * _LAYOUT_PERP
    return np.array([xy[0], xy[1], z])

HUMAN_MODEL_OFFSET = _layout_offset(-1, 0.015)
# 人体模型在显示空间的高度 (蒙皮顶点实测), G1 的缩放以它为准。
HUMAN_DISPLAY_HEIGHT = 0.784
# 全身 G1 在 URDF 原生单位下的尺寸 (站姿, 实测)。
G1_NATIVE_HEIGHT = 1.309          # 脚底到头顶
G1_NATIVE_PELVIS_TO_SOLE = 0.780  # pelvis 原点到脚底

# 这里是"看着舒服"优先, 不是物理真实: 按真实身高比 (1.32m vs 1.75m = 0.75)
# 画出来 G1 明显矮一截, 对照双臂姿态时反而费劲。所以直接缩放到与人体等高。
# 想看真实比例, 把这个系数改成 0.75 即可 —— 下面两个量会跟着算。
G1_HEIGHT_RATIO = 1.0
G1_MODEL_SCALE = HUMAN_DISPLAY_HEIGHT * G1_HEIGHT_RATIO / G1_NATIVE_HEIGHT
# 让 G1 站在 z=0 的地面网格上, 与人体模型同一个地面。
G1_MODEL_OFFSET = _layout_offset(
    +1, G1_NATIVE_PELVIS_TO_SOLE * G1_MODEL_SCALE)
# 标签浮在头顶上方 0.06, 与人体标签一致。
G1_LABEL_HEIGHT = (
    (G1_NATIVE_HEIGHT - G1_NATIVE_PELVIS_TO_SOLE) * G1_MODEL_SCALE + 0.06)
G1_ARMS_MODEL_OFFSET = _layout_offset(+1, 0.16)
HUMAN_GLB_PATH = os.path.join(
    _MY_DIR, "assets", "human", "animated_base_character.glb")
HUMAN_MODEL_SCALE = 0.44

CALIB_STATIC_DURATION = 1.8
CALIB_HINGE_DURATION = 9.0
CALIB_HINGE_MIN_DURATION = 4.0
CALIB_HINGE_MIN_RANGE_DEG = 25.0
CALIB_MAX_HINGE_AXIS_DISPERSION_DEG = 25.0
CALIB_MAX_HINGE_UPPER_MOTION_DEG = 30.0
CALIB_VALIDATION_DURATION = 1.8
CALIB_STABILITY_WINDOW = 0.4
CALIB_MAX_STATIC_DISPERSION_DEG = 2.5
CALIB_MAX_PALM_CONSISTENCY_DEG = 30.0
# 第 6 步的实时预览用未经肘轴精修的临时解算，与最终判定会有几度偏差。
# 留出安全余量后才敢显示为合格，避免预览通过但保存时仍被拒。
CALIB_PALM_PREVIEW_MARGIN_DEG = 5.0
CALIB_RECOMMENDED_TRAINING_ERROR_DEG = 20.0
CALIB_RECOMMENDED_VALIDATION_ERROR_DEG = 20.0
CALIB_HARD_TRAINING_ERROR_DEG = 35.0
CALIB_HARD_VALIDATION_ERROR_DEG = 40.0
WIRELESS_POLL_INTERVAL = 0.01
IMU_DATA_TIMEOUT_SEC = 0.10
ROLE_MAP_FILE = os.path.join(_MY_DIR, "wireless_imu_roles.json")


# 日志与 HUD 统一用中文臂别；_ArmState.side 本身是协议侧的英文标识。
SIDE_CN = {"right": "右", "left": "左"}


# PC 端增强标定流程。树莓派端仍保留原四步流程，避免改变部署端操作。
GUIDED_CALIB_POSES = [
    {"name": "右臂自然下垂、肘伸直", "kind": "static", "side": "right",
     "upper_dir": [0, 0, -1], "forearm_dir": [0, 0, -1]},
    {"name": "右臂前平举、肘伸直", "kind": "static", "side": "right",
     "upper_dir": [1, 0, 0], "forearm_dir": [1, 0, 0]},
    {"name": "右臂侧平举、掌心朝下", "kind": "static", "side": "right",
     "upper_dir": [0, 1, 0], "forearm_dir": [0, 1, 0]},
    {"name": "大臂自然下垂，舒适屈伸肘关节2次", "kind": "hinge", "side": "right",
     "hint": "不用夹紧大臂；在舒适范围内屈伸即可，尽量不要旋转小臂"},
    {"name": "右臂前平举、肘伸直、掌心朝上",
     "kind": "palm_up", "side": "right",
     "upper_dir": [1, 0, 0], "forearm_dir": [1, 0, 0],
     "palm_dir": [0, 0, 1],
     "hint": "手臂向前伸直平举，手腕伸直，掌心平稳朝向天花板"},
    {"name": "右臂前平举、肘伸直、掌心朝下",
     "kind": "palm_down", "side": "right",
     "upper_dir": [1, 0, 0], "forearm_dir": [1, 0, 0],
     "palm_dir": [0, 0, -1],
     "hint": "手臂保持前平举，翻掌朝向地面；肩可以跟着转，"
             "只要小臂始终指向正前方"},
    {"name": "验证：大臂下垂、肘90度、小臂向前", "kind": "validation", "side": "right",
     "upper_dir": [0, 0, -1], "forearm_dir": [1, 0, 0]},
    {"name": "左臂自然下垂、肘伸直", "kind": "static", "side": "left",
     "upper_dir": [0, 0, -1], "forearm_dir": [0, 0, -1]},
    {"name": "左臂前平举、肘伸直", "kind": "static", "side": "left",
     "upper_dir": [1, 0, 0], "forearm_dir": [1, 0, 0]},
    {"name": "左臂侧平举、掌心朝下", "kind": "static", "side": "left",
     "upper_dir": [0, -1, 0], "forearm_dir": [0, -1, 0]},
    {"name": "大臂自然下垂，舒适屈伸肘关节2次", "kind": "hinge", "side": "left",
     "hint": "不用夹紧大臂；在舒适范围内屈伸即可，尽量不要旋转小臂"},
    {"name": "左臂前平举、肘伸直、掌心朝上",
     "kind": "palm_up", "side": "left",
     "upper_dir": [1, 0, 0], "forearm_dir": [1, 0, 0],
     "palm_dir": [0, 0, 1],
     "hint": "手臂向前伸直平举，手腕伸直，掌心平稳朝向天花板"},
    {"name": "左臂前平举、肘伸直、掌心朝下",
     "kind": "palm_down", "side": "left",
     "upper_dir": [1, 0, 0], "forearm_dir": [1, 0, 0],
     "palm_dir": [0, 0, -1],
     "hint": "手臂保持前平举，翻掌朝向地面；肩可以跟着转，"
             "只要小臂始终指向正前方"},
    {"name": "验证：大臂下垂、肘90度、小臂向前", "kind": "validation", "side": "left",
     "upper_dir": [0, 0, -1], "forearm_dir": [1, 0, 0]},
]


def _calib_pose_duration(pose):
    return {
        "static": CALIB_STATIC_DURATION,
        "hinge": CALIB_HINGE_DURATION,
        "palm_up": CALIB_STATIC_DURATION,
        "palm_down": CALIB_STATIC_DURATION,
        "validation": CALIB_VALIDATION_DURATION,
    }[pose["kind"]]


def _calibration_quality_grade(static_error_deg, validation_error_deg):
    """返回 (可用, 可上真机, 等级)，把仿真可用与真机安全分开。"""
    usable = (static_error_deg <= CALIB_HARD_TRAINING_ERROR_DEG
              and validation_error_deg <= CALIB_HARD_VALIDATION_ERROR_DEG)
    robot_safe = (usable
                  and static_error_deg
                  <= CALIB_RECOMMENDED_TRAINING_ERROR_DEG
                  and validation_error_deg
                  <= CALIB_RECOMMENDED_VALIDATION_ERROR_DEG)
    grade = "robot_safe" if robot_safe else "simulation_only" if usable else "rejected"
    return usable, robot_safe, grade

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
    """单个手臂的状态管理。"""

    def __init__(self, side, response_mode="original"):
        self.side = side
        self.response_mode = response_mode
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
        self.calibration_quality = {}
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
        # 机器人映射使用自适应低通后的方向；人体显示仍保留原始响应。
        self.mapping_arm_dir_chest = np.array([0., 0., -1.])
        self.mapping_forearm_dir_chest = np.array([0., 0., -1.])
        self._mapping_arm_raw = None
        self._mapping_forearm_raw = None
        self._mapping_arm_speed = 0.0
        self._mapping_forearm_speed = 0.0
        self._mapping_filter_time = None
        self._ik_result_seq = -1
        self.ik_diagnostics = {}

    def _one_euro_direction_filter(self, previous, previous_raw, current,
                                   filtered_speed, dt):
        """按响应模式平滑方向；原始模式复现最早版本的视觉手感。"""
        previous = cal_normalize_vec(np.asarray(previous, dtype=float))
        previous_raw = cal_normalize_vec(np.asarray(previous_raw, dtype=float))
        current = cal_normalize_vec(np.asarray(current, dtype=float))
        if self.response_mode == "original":
            angular_delta = float(np.arccos(np.clip(
                np.dot(previous, current), -1.0, 1.0)))
            angular_speed = angular_delta / max(dt, 1e-3)
            cutoff_hz = 2.0 + min(8.0, 1.4 * angular_speed)
            alpha = 1.0 - np.exp(-2.0 * np.pi * cutoff_hz * dt)
            filtered = cal_normalize_vec(
                previous * (1.0 - alpha) + current * alpha)
            return filtered, current.copy(), angular_speed

        raw_delta = float(np.arccos(np.clip(
            np.dot(previous_raw, current), -1.0, 1.0)))
        raw_speed = raw_delta / max(dt, 1e-3)
        speed_alpha = 1.0 - np.exp(-2.0 * np.pi * 5.0 * dt)
        filtered_speed += speed_alpha * (raw_speed - filtered_speed)
        if self.response_mode == "fast":
            cutoff_hz = min(45.0, 8.0 + 6.0 * filtered_speed)
        else:
            cutoff_hz = min(30.0, 4.0 + 4.0 * filtered_speed)
        alpha = 1.0 - np.exp(-2.0 * np.pi * cutoff_hz * dt)
        filtered = cal_normalize_vec(
            previous * (1.0 - alpha) + current * alpha)
        return filtered, current.copy(), filtered_speed

    def update_mapping_directions(self, now):
        if self._mapping_filter_time is None:
            self.mapping_arm_dir_chest = cal_normalize_vec(
                self.arm_dir_chest.copy())
            self.mapping_forearm_dir_chest = cal_normalize_vec(
                self.forearm_dir_chest.copy())
            self._mapping_arm_raw = self.mapping_arm_dir_chest.copy()
            self._mapping_forearm_raw = (
                self.mapping_forearm_dir_chest.copy())
            self._mapping_filter_time = now
            return
        dt = float(np.clip(now - self._mapping_filter_time, 0.002, 0.05))
        self._mapping_filter_time = now
        (self.mapping_arm_dir_chest, self._mapping_arm_raw,
         self._mapping_arm_speed) = self._one_euro_direction_filter(
            self.mapping_arm_dir_chest, self._mapping_arm_raw,
            self.arm_dir_chest, self._mapping_arm_speed, dt)
        (self.mapping_forearm_dir_chest, self._mapping_forearm_raw,
         self._mapping_forearm_speed) = self._one_euro_direction_filter(
            self.mapping_forearm_dir_chest, self._mapping_forearm_raw,
            self.forearm_dir_chest, self._mapping_forearm_speed, dt)

    def reset(self):
        """重置校准, 保留 IMU 绑定"""
        self.upper_calibrator = ArmDirectionCalibrator(label=f"{self.side}大臂")
        self.forearm_calibrator = ArmDirectionCalibrator(label=f"{self.side}小臂")
        self.upper_calibrated = False
        self.forearm_calibrated = False
        self.twist_cal = None
        self.calibration_quality = {}
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
        self.mapping_arm_dir_chest = np.array([0., 0., -1.])
        self.mapping_forearm_dir_chest = np.array([0., 0., -1.])
        self._mapping_arm_raw = None
        self._mapping_forearm_raw = None
        self._mapping_arm_speed = 0.0
        self._mapping_forearm_speed = 0.0
        self._mapping_filter_time = None
        self._ik_result_seq = -1
        self.ik_diagnostics = {}


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
#  SessionDataLogger — 自动流式采集日志
# ========================================================

class SessionDataLogger:
    """将原始 IMU 包和 PC 解算结果流式写入 CSV，避免长时间占用内存。"""

    IMU_FIELDS = [
        "host_time_iso", "host_unix_s", "receive_monotonic_s",
        "receive_age_ms", "node_id", "device_id", "role", "connected",
        "seq", "sensor_timestamp_us", "qw", "qx", "qy", "qz",
        "gyro_x_dps", "gyro_y_dps", "gyro_z_dps", "rest",
        "pkt_rate_hz", "battery_voltage", "battery_percent",
    ]
    CONTROL_FIELDS = [
        "host_time_iso", "host_unix_s", "monotonic_s",
        "pc_control_hz", "robot_tx_hz", "calibration_step", "active_arm",
        "chest_qw", "chest_qx", "chest_qy", "chest_qz",
        "right_upper_qw", "right_upper_qx", "right_upper_qy",
        "right_upper_qz", "right_forearm_qw", "right_forearm_qx",
        "right_forearm_qy", "right_forearm_qz",
        "right_arm_x", "right_arm_y", "right_arm_z",
        "right_forearm_x", "right_forearm_y", "right_forearm_z",
        "right_palm_x", "right_palm_y", "right_palm_z",
        "right_sp_rad", "right_sr_rad", "right_sy_rad",
        "right_el_rad", "right_wr_rad", "right_following",
        "left_upper_qw", "left_upper_qx", "left_upper_qy", "left_upper_qz",
        "left_forearm_qw", "left_forearm_qx", "left_forearm_qy",
        "left_forearm_qz", "left_arm_x", "left_arm_y", "left_arm_z",
        "left_forearm_x", "left_forearm_y", "left_forearm_z",
        "left_palm_x", "left_palm_y", "left_palm_z",
        "left_sp_rad", "left_sr_rad", "left_sy_rad",
        "left_el_rad", "left_wr_rad", "left_following",
        "ik_avg_ms", "ik_p95_ms",
    ]

    def __init__(self, output_root=None, enabled=True,
                 control_period_sec=0.01):
        self.enabled = bool(enabled)
        self.session_dir = None
        self.error = None
        self._start_monotonic = time.monotonic()
        self._start_wall = time.time()
        self._control_period_sec = max(0.001, float(control_period_sec))
        self._next_control_time = self._start_monotonic
        self._last_flush = self._start_monotonic
        self._last_seq = {}
        self._imu_rows = 0
        self._control_rows = 0
        self._event_rows = 0
        self._dropped_log_records = 0
        self._estimated_missing_packets = {}
        self._imu_file = None
        self._control_file = None
        self._event_file = None
        self._imu_writer = None
        self._control_writer = None
        self._write_queue = queue.Queue(maxsize=20000)
        self._writer_thread = None
        if not self.enabled:
            return

        try:
            root = output_root or os.path.join(_MY_DIR, "采集日志")
            session_name = datetime.now().strftime(
                "session_%Y%m%d_%H%M%S_%f")
            self.session_dir = os.path.join(root, session_name)
            os.makedirs(self.session_dir, exist_ok=False)
            self._imu_file = open(
                os.path.join(self.session_dir, "imu_packets.csv"),
                "w", newline="", encoding="utf-8")
            self._control_file = open(
                os.path.join(self.session_dir, "control.csv"),
                "w", newline="", encoding="utf-8")
            self._event_file = open(
                os.path.join(self.session_dir, "events.log"),
                "w", encoding="utf-8")
            self._imu_writer = csv.DictWriter(
                self._imu_file, fieldnames=self.IMU_FIELDS)
            self._control_writer = csv.DictWriter(
                self._control_file, fieldnames=self.CONTROL_FIELDS)
            self._imu_writer.writeheader()
            self._control_writer.writeheader()
            self._write_metadata(final=False)
            self._writer_thread = threading.Thread(
                target=self._writer_loop,
                name="imu-data-logger", daemon=True)
            self._writer_thread.start()
        except (OSError, ValueError) as exc:
            self.error = str(exc)
            self.enabled = False
            self._close_files()

    @staticmethod
    def _iso_time(wall_time):
        return datetime.fromtimestamp(wall_time).isoformat(
            timespec="milliseconds")

    @staticmethod
    def _values(prefix, values, names):
        return {
            f"{prefix}{name}": float(value)
            for name, value in zip(names, values)
        }

    def _write_metadata(self, final):
        if self.session_dir is None:
            return
        metadata = {
            "format_version": 1,
            "started_at": self._iso_time(self._start_wall),
            "finished_at": (
                self._iso_time(time.time()) if final else None),
            "duration_sec": (
                time.monotonic() - self._start_monotonic
                if final else None),
            "imu_rows": self._imu_rows,
            "control_rows": self._control_rows,
            "event_rows": self._event_rows,
            "dropped_log_records": self._dropped_log_records,
            "estimated_missing_packets":
                self._estimated_missing_packets,
            "files": {
                "imu": "imu_packets.csv",
                "control": "control.csv",
                "events": "events.log",
            },
        }
        with open(
                os.path.join(self.session_dir, "metadata.json"),
                "w", encoding="utf-8") as metadata_file:
            json.dump(metadata, metadata_file, ensure_ascii=False, indent=2)

    def _enqueue(self, record_type, record):
        if not self.enabled:
            return
        try:
            self._write_queue.put_nowait((record_type, record))
        except queue.Full:
            self._dropped_log_records += 1

    def _writer_loop(self):
        try:
            while True:
                record_type, record = self._write_queue.get()
                if record_type == "close":
                    break
                if record_type == "imu":
                    self._imu_writer.writerow(record)
                    self._imu_rows += 1
                elif record_type == "control":
                    self._control_writer.writerow(record)
                    self._control_rows += 1
                elif record_type == "event":
                    self._event_file.write(record)
                    self._event_rows += 1
                self._flush_if_due(time.monotonic())
        except (OSError, ValueError, TypeError) as exc:
            self.error = str(exc)
        finally:
            self._flush_if_due(float("inf"))

    def log_event(self, message):
        if not self.enabled or self._event_file is None:
            return
        wall_time = time.time()
        self._enqueue(
            "event",
            f"[{self._iso_time(wall_time)}] {message}\n")

    def log_imu_devices(self, devices, role_by_node, now_monotonic):
        """每个设备序号只写一次，PC 高频轮询不会制造重复数据。"""
        if not self.enabled or self._imu_writer is None:
            return
        wall_time = time.time()
        try:
            for device in devices:
                seq = int(getattr(device, "seq", -1))
                node_id = str(device.node_id)
                if self._last_seq.get(node_id) == seq:
                    continue
                previous_seq = self._last_seq.get(node_id)
                if previous_seq is not None and seq >= 0:
                    delta = (seq - previous_seq) & 0xFFFF
                    if 1 < delta < 0x8000:
                        self._estimated_missing_packets[node_id] = (
                            self._estimated_missing_packets.get(
                                node_id, 0) + delta - 1)
                self._last_seq[node_id] = seq

                quat = list(getattr(device, "quat", None) or
                            [1.0, 0.0, 0.0, 0.0])
                gyro = list(getattr(device, "gyro_dps", None) or
                            ["", "", ""])
                receive_time = float(
                    getattr(device, "receive_monotonic", 0.0))
                row = {
                    "host_time_iso": self._iso_time(wall_time),
                    "host_unix_s": f"{wall_time:.6f}",
                    "receive_monotonic_s": (
                        f"{receive_time:.6f}"
                        if receive_time > 0.0 else ""),
                    "receive_age_ms": (
                        f"{max(0.0, now_monotonic - receive_time) * 1000.0:.3f}"
                        if receive_time > 0.0 else ""),
                    "node_id": node_id,
                    "device_id": getattr(device, "device_id", ""),
                    "role": role_by_node.get(node_id, ""),
                    "connected": int(bool(device.connected)),
                    "seq": seq,
                    "sensor_timestamp_us": (
                        getattr(device, "sensor_timestamp_us", None)
                        if getattr(
                            device, "sensor_timestamp_us", None)
                        is not None else ""),
                    "qw": quat[0], "qx": quat[1],
                    "qy": quat[2], "qz": quat[3],
                    "gyro_x_dps": gyro[0],
                    "gyro_y_dps": gyro[1],
                    "gyro_z_dps": gyro[2],
                    "rest": int(bool(getattr(device, "rest", False))),
                    "pkt_rate_hz": float(
                        getattr(device, "pkt_rate_hz", 0.0)),
                    "battery_voltage": (
                        getattr(device, "battery_voltage", None)
                        if getattr(device, "battery_voltage", None)
                        is not None else ""),
                    "battery_percent": (
                        getattr(device, "battery_percent", None)
                        if getattr(device, "battery_percent", None)
                        is not None else ""),
                }
                self._enqueue("imu", row)
        except (OSError, ValueError, TypeError) as exc:
            self.error = str(exc)

    def log_control(self, now_monotonic, app):
        if (not self.enabled or self._control_writer is None
                or now_monotonic < self._next_control_time):
            return
        self._next_control_time += self._control_period_sec
        if self._next_control_time <= now_monotonic:
            self._next_control_time = (
                now_monotonic + self._control_period_sec)
        wall_time = time.time()
        try:
            row = {
                "host_time_iso": self._iso_time(wall_time),
                "host_unix_s": f"{wall_time:.6f}",
                "monotonic_s": f"{now_monotonic:.6f}",
                "pc_control_hz": app.sample_rate,
                "robot_tx_hz": app._udp_send_rate,
                "calibration_step": app._calib_step,
                "active_arm": app.active_arm,
                "ik_avg_ms": app._ik_avg_ms,
                "ik_p95_ms": app._ik_p95_ms,
            }
            row.update(self._values(
                "chest_q", app.q_chest, ["w", "x", "y", "z"]))
            for arm in (app.right, app.left):
                prefix = f"{arm.side}_"
                row.update(self._values(
                    prefix + "upper_q", arm.q_upper,
                    ["w", "x", "y", "z"]))
                row.update(self._values(
                    prefix + "forearm_q", arm.q_forearm,
                    ["w", "x", "y", "z"]))
                row.update(self._values(
                    prefix + "arm_", arm.arm_dir_chest,
                    ["x", "y", "z"]))
                row.update(self._values(
                    prefix + "forearm_", arm.forearm_dir_chest,
                    ["x", "y", "z"]))
                row.update(self._values(
                    prefix + "palm_", arm.palm_dir_chest,
                    ["x", "y", "z"]))
                angles = arm.angles
                row.update({
                    prefix + "sp_rad": angles.shoulder_pitch,
                    prefix + "sr_rad": angles.shoulder_roll,
                    prefix + "sy_rad": angles.shoulder_yaw,
                    prefix + "el_rad": angles.elbow,
                    prefix + "wr_rad": angles.wrist_roll,
                    prefix + "following": int(arm.following),
                })
            self._enqueue("control", row)
        except (OSError, ValueError, TypeError) as exc:
            self.error = str(exc)

    def _flush_if_due(self, now_monotonic):
        if now_monotonic - self._last_flush < 1.0:
            return
        for file_obj in (
                self._imu_file, self._control_file, self._event_file):
            if file_obj is not None:
                file_obj.flush()
        self._last_flush = now_monotonic

    def _close_files(self):
        for file_obj in (
                self._imu_file, self._control_file, self._event_file):
            if file_obj is not None and not file_obj.closed:
                file_obj.flush()
                file_obj.close()

    def close(self):
        if not self.enabled:
            self._close_files()
            return self.session_dir
        self.enabled = False
        try:
            self._write_queue.put(("close", None), timeout=2.0)
        except queue.Full:
            self._dropped_log_records += self._write_queue.qsize()
            while True:
                try:
                    self._write_queue.get_nowait()
                except queue.Empty:
                    break
            self._write_queue.put_nowait(("close", None))
        if self._writer_thread is not None:
            self._writer_thread.join(timeout=5.0)
        self._write_metadata(final=True)
        self._close_files()
        return self.session_dir


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


def draw_ellipsoid(pos, radii, color, alpha=1.0, slices=24, stacks=18):
    """绘制平滑椭球，用于人体头部、胸腔和关节外壳。"""
    glPushMatrix()
    glTranslatef(*pos)
    glScalef(*radii)
    glColor4f(*color, alpha)
    quad = gluNewQuadric()
    gluQuadricNormals(quad, GLU_SMOOTH)
    gluSphere(quad, 1.0, slices, stacks)
    gluDeleteQuadric(quad)
    glPopMatrix()


def draw_tapered_limb(length, radius_start, radius_end, color, alpha=1.0):
    """沿局部 X 轴绘制锥形圆柱，端部由关节椭球自然封口。"""
    glPushMatrix()
    glRotatef(90.0, 0.0, 1.0, 0.0)  # GLU 圆柱 Z 轴 → 人体骨段 X 轴
    glColor4f(*color, alpha)
    quad = gluNewQuadric()
    gluQuadricNormals(quad, GLU_SMOOTH)
    gluCylinder(quad, radius_start, radius_end, length, 24, 3)
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

    def __init__(self, udp_target=None, right_forearm_only=False,
                 right_arm_two_imu=False, forearm_elbow_sign=1.0,
                 forearm_wrist_sign=1.0, upper_pitch_sign=1.0,
                 upper_roll_sign=1.0, upper_yaw_sign=1.0,
                 fullscreen=False, data_log=True, data_log_dir=None,
                 response_mode="original", waist_enabled=False,
                 waist_yaw_sign=1.0,
                 waist_max_yaw_rad=WAIST_DEFAULT_MAX_YAW_RAD,
                 waist_max_speed=WAIST_DEFAULT_MAX_SPEED,
                 waist_gain=WAIST_DEFAULT_GAIN, robot_model="full",
                 hand_cfg=None, hand_port=None, real_hand=None,
                 real_hand_transport="modbus", real_hand_ip=None,
                 hand_left_cfg=None, hand_left_port=None,
                 thumb_retarget=True):
        self._g1_full_body = (robot_model == "full")
        # 灵巧手：给了 cfg 才启用，和 --waist 一样是显式开关。数据源是独立
        # 的一条链（手套 → mhandpro_diagnostic → TCP），坏了不影响手臂。
        #
        # 左右手各自一路, 从 cfg 到 TCP 端口到超时降级全程互不相干：一只手套
        # 掉线只让那只手张开, 另一只继续跟随。两只手套共用一个接收器, 所以
        # 上游是同一个 mhandpro_diagnostic 进程 (teleop both), 但下游这两条
        # TCP 是分开的。
        self._hand_cfgs = {"right": hand_cfg, "left": hand_left_cfg}
        self._hand_ports = {"right": hand_port, "left": hand_left_port}
        self.hand_enabled = any(v is not None for v in self._hand_cfgs.values())
        self.gloves = {"right": None, "left": None}
        # 拇指指腹位置重定向, 默认开。关掉就退回线性投影, 方便 A/B 对比 ——
        # "拇指到底有没有变好"要靠同一副手套同一次标定来回切才说得清。
        self._thumb_retarget = thumb_retarget
        # 渲染线程读的手部闭合度，由发送线程写入；None = 没有数据（张开）。
        self._hand_render_closure = {"right": None, "left": None}
        # 真手输出：DDS rt/inspire_hand/ctrl/{l,r}。与仿真那条路完全独立，
        # 起不来只关掉真手，仿真和手臂照跑。
        self._real_hand_side = real_hand
        self._real_hand_transport = real_hand_transport
        self._real_hand_ip = real_hand_ip
        self.real_hand = None
        # 渲染线程读的腰部角度, 由 _waist_snapshot 写入。
        self._waist_render_yaw = 0.0
        self.lock = threading.Lock()
        self._ik_lock = threading.Lock()
        self.right_forearm_only = bool(right_forearm_only)
        self.right_arm_two_imu = bool(right_arm_two_imu)
        self.right_diagnostic_mode = (
            self.right_forearm_only or self.right_arm_two_imu)
        if response_mode not in ("original", "balanced", "fast"):
            raise ValueError(f"未知响应模式: {response_mode}")
        self.response_mode = response_mode
        self._fullscreen = bool(fullscreen)
        self._windowed_size = (WINDOW_WIDTH, WINDOW_HEIGHT)
        self._window_width = WINDOW_WIDTH
        self._window_height = WINDOW_HEIGHT
        self._drawable_width = WINDOW_WIDTH
        self._drawable_height = WINDOW_HEIGHT
        self._drawable_scale_x = 1.0
        self._drawable_scale_y = 1.0
        self.single_forearm_mapper = SingleForearmMapper(
            elbow_sign=forearm_elbow_sign,
            wrist_sign=forearm_wrist_sign,
        )
        self.two_imu_mapper = RightArmTwoImuMapper(
            shoulder_pitch_sign=upper_pitch_sign,
            shoulder_roll_sign=upper_roll_sign,
            shoulder_yaw_sign=upper_yaw_sign,
            elbow_sign=forearm_elbow_sign,
            wrist_sign=forearm_wrist_sign,
        )

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
        self._udp_running = False
        self._udp_thread = None
        self._udp_seq = 0
        self._udp_send_rate = 0.0
        self._udp_send_count = 0

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
        self.right = _ArmState("right", response_mode=response_mode)
        self.left = _ArmState("left", response_mode=response_mode)
        self.active_arm = "right"

        # 腰部：复用胸部 IMU 的绝对姿态，只驱动 waist_yaw。
        self.waist_enabled = bool(waist_enabled)
        self.waist = WaistTracker(
            max_yaw_rad=float(waist_max_yaw_rad),
            max_speed_rad_s=float(waist_max_speed),
            sign=float(waist_yaw_sign),
            gain=float(waist_gain),
        )

        # 固定周期 IK 线程：无线线程只更新姿态和发布最新输入。
        self._ik_running = False
        self._ik_thread = None
        self._ik_event = threading.Event()
        self._ik_input = {}
        self._ik_input_seq = 0
        self._ik_result_right = None
        self._ik_result_left = None
        self._ik_count = 0
        self._ik_total_time = 0.0
        self._ik_avg_ms = 0.0
        self._ik_p95_ms = 0.0
        self._ik_solve_times = []

        # 7步增强校准状态
        self._calib_step = -1  # -1=未在校准, >=0=GUIDED_CALIB_POSES 索引
        self._calib_waiting = False  # True=等待用户按C开始采集, False=正在采集
        self._calib_arm = None
        # True=右臂走完自动接左臂的连续标定; False=只标定当前一条臂。
        self._calib_chain = False
        self._calib_start = 0.0
        self._calib_samples_chest = []
        self._calib_samples_upper = []
        self._calib_samples_forearm = []
        self._pose_averages = []
        self._palm_pose_results = {}
        self._hinge_result = None
        self._hinge_live_result = None
        self._calib_last_quality_update = 0.0
        # 第 6 步翻掌一致性的实时预览及其临时小臂解算。
        self._palm_live_consistency_deg = None
        self._palm_preview_calibrator = None
        self._calib_previous = None
        self._validation_average = None
        self._calib_stability_window = []
        self._calib_stable_since = None
        self._calib_stability_deg = float("inf")

        # 录制
        self.recorder = DataRecorder()

        # 日志
        self._logs = []
        self.session_logger = SessionDataLogger(
            output_root=data_log_dir, enabled=data_log)
        if self.session_logger.enabled:
            self._log(
                f"自动采集日志: {self.session_logger.session_dir}")
        elif self.session_logger.error:
            self._log(
                f"自动采集日志创建失败: {self.session_logger.error}")
        response_labels = {
            "original": "原始平滑",
            "balanced": "平衡",
            "fast": "快速",
        }
        self._log(
            f"响应模式: {response_labels[self.response_mode]} "
            f"(原始版同步 IK 链路)")

        # 相机
        self._cam_rot_x = 20.0
        self._cam_rot_y = -30.0
        self._cam_dist = 2.0
        self._mouse_dragging = False
        self._mouse_last = (0, 0)

        # 官方 G1 URDF 数字孪生；OpenGL 资源在窗口创建后加载。
        self.g1_renderer = None
        self.g1_renderer_ready = False
        self.g1_renderer_error = None
        # 带骨骼的人体数字替身；加载失败时保留原简化人体作为回退。
        self.human_renderer = None
        self.human_renderer_ready = False
        self.human_renderer_error = None

    @property
    def active(self):
        return self.right if self.active_arm == "right" else self.left

    def _display_size(self):
        """返回窗口逻辑尺寸（HUD 坐标系）。"""
        return (
            max(1, int(getattr(self, "_window_width", WINDOW_WIDTH))),
            max(1, int(getattr(self, "_window_height", WINDOW_HEIGHT))),
        )

    def _drawable_size(self):
        """返回 OpenGL 后备缓冲区真实像素尺寸。"""
        return (
            max(1, int(getattr(
                self, "_drawable_width", self._display_size()[0]))),
            max(1, int(getattr(
                self, "_drawable_height", self._display_size()[1]))),
        )

    def _sync_display_size(self, screen=None):
        """同步窗口尺寸，并按创建窗口时测得的 DPI 比例计算 drawable。"""
        if screen is None:
            screen = pygame.display.get_surface()
        try:
            logical_size = pygame.display.get_window_size()
        except pygame.error:
            logical_size = screen.get_size() if screen is not None else None
        if logical_size is not None:
            self._window_width, self._window_height = logical_size
        self._drawable_width = max(1, int(round(
            self._window_width * self._drawable_scale_x)))
        self._drawable_height = max(1, int(round(
            self._window_height * self._drawable_scale_y)))

    def _capture_drawable_scale(self, screen):
        """从新建 OpenGL 上下文的初始视口安全测量高 DPI 比例。"""
        try:
            logical_width, logical_height = pygame.display.get_window_size()
        except pygame.error:
            logical_width, logical_height = screen.get_size()
        try:
            viewport = glGetIntegerv(GL_VIEWPORT)
            drawable_width = int(viewport[2]) if len(viewport) >= 4 else 0
            drawable_height = int(viewport[3]) if len(viewport) >= 4 else 0
        except Exception:
            drawable_width, drawable_height = logical_width, logical_height
        if drawable_width <= 0 or drawable_height <= 0:
            drawable_width, drawable_height = logical_width, logical_height

        scale_x = drawable_width / max(logical_width, 1)
        scale_y = drawable_height / max(logical_height, 1)
        # 异常的旧视口不能作为 DPI 比例使用。
        self._drawable_scale_x = (
            scale_x if 0.75 <= scale_x <= 4.0 else 1.0)
        self._drawable_scale_y = (
            scale_y if 0.75 <= scale_y <= 4.0 else 1.0)
        self._window_width, self._window_height = (
            logical_width, logical_height)
        self._drawable_width = max(1, int(round(
            logical_width * self._drawable_scale_x)))
        self._drawable_height = max(1, int(round(
            logical_height * self._drawable_scale_y)))

    def _set_display_mode(self, fullscreen=None, window_size=None):
        """创建/切换 OpenGL 窗口，并同步渲染器使用的实际分辨率。"""
        if fullscreen is not None:
            self._fullscreen = bool(fullscreen)

        base_flags = pygame.DOUBLEBUF | pygame.OPENGL
        if self._fullscreen:
            info = pygame.display.Info()
            size = (max(1, info.current_w), max(1, info.current_h))
            flags = base_flags | pygame.FULLSCREEN
        else:
            if window_size is not None:
                self._windowed_size = (
                    max(MIN_WINDOW_WIDTH, int(window_size[0])),
                    max(MIN_WINDOW_HEIGHT, int(window_size[1])),
                )
            size = self._windowed_size
            flags = base_flags | pygame.RESIZABLE

        screen = pygame.display.set_mode(size, flags)
        self._capture_drawable_scale(screen)
        glViewport(0, 0, self._drawable_width, self._drawable_height)
        return screen

    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self._logs.append(f"[{ts}] {msg}")
        if len(self._logs) > 50:
            self._logs = self._logs[-50:]
        print(msg)
        session_logger = getattr(self, "session_logger", None)
        if session_logger is not None:
            session_logger.log_event(msg)

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

        # 单 IMU 诊断模式下，如果只有一个设备在线，允许它没有
        # right_forearm 固定 ID，直接将唯一设备作为右小臂。
        if self.right_forearm_only:
            connected = [d for d in devices if d.connected]
            current_online = any(
                d.node_id == self.right.forearm_node_id for d in connected)
            if len(connected) == 1 and not current_online:
                self.right.forearm_node_id = connected[0].node_id
                self._log(f"单IMU模式: 将唯一在线设备识别为右小臂: "
                          f"{connected[0].node_id}")

    def _read_imu(self, dev_dict, node_id):
        d = dev_dict.get(node_id) if node_id else None
        if d is None or not d.connected:
            return None, False
        receive_time = float(getattr(d, "receive_monotonic", 0.0))
        if (receive_time > 0.0
                and time.monotonic() - receive_time > IMU_DATA_TIMEOUT_SEC):
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
        self._start_udp_sender()

    def _stop_imu(self):
        self._stop_udp_sender()
        self._wireless_running = False
        if self._wireless_thread is not None:
            self._wireless_thread.join(timeout=1.0)
        self.imu_service.stop()
        self._stop_discovery()

    # ---------- 50 Hz UDP 目标发送 ----------

    @staticmethod
    def _angles_tuple(arm):
        a = arm.angles
        return (
            float(a.shoulder_pitch), float(a.shoulder_roll),
            float(a.shoulder_yaw), float(a.elbow),
            float(a.wrist_roll),
        )

    @staticmethod
    def _guard_packet_jump(previous, desired):
        """只拦截异常单包突跳，不对正常动作做速度整形。"""
        if previous is None:
            return np.asarray(desired, dtype=float)
        previous = np.asarray(previous, dtype=float)
        desired = np.asarray(desired, dtype=float)
        delta = np.clip(
            desired - previous,
            -PC_PACKET_JUMP_GUARD_RAD,
            PC_PACKET_JUMP_GUARD_RAD,
        )
        return previous + delta

    def _command_snapshot(self):
        """在短锁区内取得目标、模式与机器人地址。"""
        with self.lock:
            r_follow = self.right.following and self.right.upper_calibrated
            if self.right_forearm_only:
                r_follow = r_follow and self.right.forearm_connected
            elif self.right_arm_two_imu:
                r_follow = (r_follow and self.right.upper_connected
                            and self.right.forearm_connected)
            l_follow = self.left.following and self.left.upper_calibrated
            target = self._udp_target or (
                (self.robot_ip, self.robot_port)
                if self.robot_discovered else None)
            if target:
                r_follow = (r_follow
                            and self._quality_allows_target(
                                self.right, target))
                l_follow = (l_follow
                            and self._quality_allows_target(
                                self.left, target))
            mode = (3 if r_follow and l_follow else
                    1 if r_follow else
                    2 if l_follow else 0)
            positions = (
                self._angles_tuple(self.right)
                + self._angles_tuple(self.left))
        return target, mode, positions

    def _start_udp_sender(self):
        if self._udp_running:
            return
        self._udp_running = True
        self._udp_thread = threading.Thread(
            target=self._udp_send_loop, daemon=True, name="robot_udp_50hz")
        self._udp_thread.start()
        self._log("机器人目标发送线程已启动 (50 Hz, V2时间戳协议)")

    def _stop_udp_sender(self):
        self._udp_running = False
        if self._udp_thread is not None:
            self._udp_thread.join(timeout=1.0)
            self._udp_thread = None

    def _udp_send_loop(self):
        next_deadline = time.monotonic()
        previous_positions = None
        filtered_velocities = np.zeros(10, dtype=float)
        previous_time = None
        rate_t = next_deadline
        rate_count = 0

        while self._udp_running:
            now = time.monotonic()
            wait = next_deadline - now
            if wait > 0:
                time.sleep(wait)
                now = time.monotonic()
            elif -wait > 3.0 * UDP_SEND_DT:
                next_deadline = now
            next_deadline += UDP_SEND_DT

            target, mode, desired = self._command_snapshot()
            if target is None:
                previous_time = None
                continue

            guarded = self._guard_packet_jump(
                previous_positions, desired)
            dt = (now - previous_time
                  if previous_time is not None else UDP_SEND_DT)
            dt = float(np.clip(dt, 0.002, 0.05))
            if previous_positions is None or mode == 0:
                raw_velocity = np.zeros(10, dtype=float)
            else:
                raw_velocity = (
                    guarded - np.asarray(previous_positions)) / dt
            raw_velocity = np.clip(
                raw_velocity,
                -COMMAND_VELOCITY_LIMIT,
                COMMAND_VELOCITY_LIMIT,
            )
            alpha = 1.0 - np.exp(
                -2.0 * np.pi * COMMAND_VELOCITY_CUTOFF_HZ * dt)
            filtered_velocities += alpha * (
                raw_velocity - filtered_velocities)
            if mode == 0:
                filtered_velocities[:] = 0.0

            waist_yaw, waist_active = self._waist_snapshot(dt, mode)
            hands = self._hand_snapshot()
            packet = pack_arm_command(
                mode,
                self._udp_seq,
                time.monotonic_ns() // 1000,
                guarded,
                filtered_velocities,
                waist=(waist_yaw, 0.0, 0.0),
                waist_enabled=waist_active,
                right_hand=hands["right"],
                left_hand=hands["left"],
            )
            try:
                self._udp_sock.sendto(packet, target)
            except OSError:
                pass
            self._udp_seq = (self._udp_seq + 1) & 0xFFFFFFFF
            previous_positions = guarded
            previous_time = now
            rate_count += 1
            if now - rate_t >= 2.0:
                self._udp_send_rate = rate_count / (now - rate_t)
                rate_count = 0
                rate_t = now

    def _hand_render_joints(self):
        """每只启用的手 6 通道闭合度 → 展开后的 12 个关节角，供渲染器直接用。

        渲染器不解析 URDF 的 <mimic>，联动关节必须在这里算好；这也是仿真端
        走的同一条路（同一个 hand_mapping），两边画出来的手因此不会不一样。
        没有数据时返回全张开而不是 None —— 手是画出来的，总得有个姿态。

        左右手的关节名不重叠（``left_*`` / ``right_*``），所以两侧直接合成一个
        字典给渲染器，不需要分开传。
        """
        if not self.hand_enabled:
            return None
        joints = {}
        for side, cfg in self._hand_cfgs.items():
            if cfg is None:
                continue
            closure = (self._hand_render_closure[side]
                       or hand_mapping.open_closure())
            joints.update(hand_mapping.expand_mimic(
                hand_mapping.closure_to_angles(closure, side=side),
                side=side))
        return joints

    def _hand_snapshot(self):
        """取本帧左右手闭合度并同步给渲染线程；返回 ``{侧: 闭合度或 None}``。

        每侧独立取数、独立降级：右手套掉线只让右手变 None，左手照常跟随。

        **不跟手臂的模式联动**，这一点和腰相反。腰必须等手臂进入跟随才动，
        是因为扭腰会改变上半身重心、直接扰动运控服务的平衡；手指没有这个
        耦合，握不握拳不影响机器人站得稳不稳。

        真跟着 mode 走的话，不接 IMU 就永远测不了手 —— 手臂没数据就一直是
        安全暂停，手也就永远不动。手部本来就是独立的一条链（独立传感器、
        独立超时、独立降级），保持独立。

        安全性由"没数据即张开"保证，而且有三层：手套断连、200ms 数据超时、
        消费端收不到标志位。返回 None 时协议层不附加手部标志位，消费端据此
        张开 —— "没有数据"和"张开指令"在链路上始终是两回事。
        """
        closures = {"right": None, "left": None}
        if self.hand_enabled:
            for side, glove in self.gloves.items():
                if glove is not None:
                    closures[side] = glove.latest_closure()
        self._hand_render_closure = closures
        if self.real_hand is not None:
            # 真手每帧都要收到指令, closure 为 None 时它自己朝张开推进 ——
            # 不发比发张开更危险, 手会停在最后一个握持姿态上。
            # 真手只驱动 --real-hand 指定的那一侧, 取的必须是同一侧的闭合度。
            try:
                self.real_hand.send(closures.get(self._real_hand_side))
            except Exception as exc:
                self._log(f"真手下发失败, 已关闭真手输出: {exc}")
                self.real_hand = None
        return closures

    def _waist_snapshot(self, dt, mode):
        """在短锁区内推进腰部跟踪并返回 (waist_yaw, 是否启用)。

        腰只在双臂已经进入跟随时才生效：模式 0 说明操作者主动暂停或
        已超时，此时腰应该和手臂一起回中而不是继续跟着人转。
        """
        with self.lock:
            active = (self.waist_enabled and self.waist.calibrated
                      and self.chest_connected and mode != 0)
            if active:
                yaw = self.waist.update(self.q_chest, dt)
            else:
                yaw = self.waist.relax_to_zero(dt)
        # 供渲染线程读取; 单个 float 的读写是原子的, 不必为画面再抢一次锁。
        self._waist_render_yaw = float(yaw)
        return float(yaw), bool(active)

    def capture_waist_zero(self):
        """把当前腰部 IMU 姿态记为零位。调用方负责持有 ``self.lock``。"""
        if not self.chest_connected:
            self._log("腰部零位采集失败: 腰部 IMU 未连接")
            return False
        self.waist.capture_zero(self.q_chest)
        self._log("腰部零位已采集（正对前方站直）")
        return True

    @staticmethod
    def _target_is_local(target):
        if not target:
            return True
        return str(target[0]).strip().lower() in {
            "127.0.0.1", "localhost", "::1"}

    def _quality_allows_target(self, arm, target):
        return (self._target_is_local(target)
                or arm.calibration_quality.get("safe_for_robot", True))

    def _arm_follow_blocker(self, arm, target):
        """返回该臂无法进入跟随的原因，可跟随时返回 None。"""
        side_cn = SIDE_CN[arm.side]
        if not arm.upper_calibrated:
            return f"{side_cn}臂尚未标定"
        if not self.chest_connected:
            return "胸部 IMU 未连接"
        if not (arm.upper_connected and arm.forearm_connected):
            return f"{side_cn}臂 IMU 未全部连接"
        if target and not self._quality_allows_target(arm, target):
            return (f"{side_cn}臂标定仅供仿真；请将目标设为 127.0.0.1，"
                    "或重新标定到≤20°后连接真机")
        return None

    def toggle_dual_follow(self):
        """B 键：两臂都没跟随就尽量全开，否则全部暂停。

        只有一条臂满足条件时开那一条并写明另一条的原因，避免调试时被
        整体阻断。调用方负责持有 ``self.lock``。
        """
        if self.right_diagnostic_mode:
            self._log("右臂诊断模式不支持双臂跟随")
            return
        if self.right.following and self.left.following:
            self.right.following = False
            self.left.following = False
            self._log("双臂: 暂停")
            return

        target = self._udp_target or (
            (self.robot_ip, self.robot_port) if self.robot_discovered else None)
        started = []
        for arm in (self.right, self.left):
            blocker = self._arm_follow_blocker(arm, target)
            if blocker is None:
                arm.following = True
                started.append(f"{SIDE_CN[arm.side]}臂")
            else:
                arm.following = False
                self._log(blocker)
        if started:
            self._log(f"{'、'.join(started)}: 实时跟随")
        else:
            self._log("双臂都不满足跟随条件，未启动")

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
                role_by_node = {
                    node_id: role for node_id, role in (
                        (self.chest_node_id, "chest"),
                        (self.right.upper_node_id, "right_upper"),
                        (self.right.forearm_node_id, "right_forearm"),
                        (self.left.upper_node_id, "left_upper"),
                        (self.left.forearm_node_id, "left_forearm"),
                    ) if node_id
                }
                self.session_logger.log_imu_devices(
                    devices, role_by_node, now)

                # 定期打印设备连接状态 (每5秒)
                if now - _conn_log_last >= _conn_log_interval:
                    _conn_log_last = now
                    n_dev = len(devices)
                    n_conn = sum(1 for d in devices if d.connected)
                    if n_dev > 0:
                        rx_rates = [
                            float(d.pkt_rate_hz) for d in devices
                            if d.connected and float(d.pkt_rate_hz) > 0.0]
                        rx_text = (
                            "IMU RX %.0f/%.0fHz(min/avg)"
                            % (min(rx_rates), sum(rx_rates) / len(rx_rates))
                            if rx_rates else "IMU RX --")
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
                        self._log(
                            f"设备 [{n_conn}/{n_dev}]: {' '.join(parts)} | "
                            f"{rx_text} | PC CTRL {self.sample_rate:.0f}Hz | "
                            f"IK {self._ik_avg_ms:.1f}/"
                            f"{self._ik_p95_ms:.1f}ms(avg/p95) | "
                            f"机器人TX {self._udp_send_rate:.0f}Hz")

                # 胸部
                q_c, self.chest_connected = self._read_imu(dev_dict, self.chest_node_id)
                chest_device = dev_dict.get(self.chest_node_id)
                chest_rest = bool(
                    self.chest_connected and chest_device is not None
                    and chest_device.rest)
                if q_c is not None:
                    self.q_chest = q_c

                # 双臂
                for arm in (self.right, self.left):
                    if self.right_forearm_only:
                        if arm.side == "left":
                            continue
                        q_f, arm.forearm_connected = self._read_imu(
                            dev_dict, arm.forearm_node_id)
                        # HUD 原本要求大臂和小臂同时在线；单 IMU 模式
                        # 中将右小臂的连接状态同步给该标志。
                        arm.upper_connected = arm.forearm_connected
                        if q_f is not None:
                            arm.q_forearm = q_f
                        if (arm.forearm_connected
                                and self.single_forearm_mapper.calibrated):
                            elbow, wrist_roll = self.single_forearm_mapper.compute(
                                arm.q_forearm)
                            arm.angles = clamp_joint_angles(JointAngles(
                                shoulder_pitch=0.0,
                                shoulder_roll=0.0,
                                shoulder_yaw=0.0,
                                elbow=elbow,
                                wrist_roll=wrist_roll,
                                side="right",
                            ))
                            arm.upper_calibrated = True
                            arm.forearm_calibrated = True
                            arm.robot_fk_urdf = forward_kinematics_right_arm_full(
                                arm.angles)
                        continue

                    if self.right_arm_two_imu:
                        if arm.side == "left":
                            continue
                        q_u, arm.upper_connected = self._read_imu(
                            dev_dict, arm.upper_node_id)
                        q_f, arm.forearm_connected = self._read_imu(
                            dev_dict, arm.forearm_node_id)
                        if q_u is not None:
                            arm.q_upper = q_u
                        if q_f is not None:
                            arm.q_forearm = q_f
                        if (arm.upper_connected and arm.forearm_connected
                                and self.two_imu_mapper.calibrated):
                            sp, sr, sy, elbow, wrist_roll = (
                                self.two_imu_mapper.compute(
                                    arm.q_upper, arm.q_forearm))
                            arm.angles = clamp_joint_angles(JointAngles(
                                shoulder_pitch=sp,
                                shoulder_roll=sr,
                                shoulder_yaw=sy,
                                elbow=elbow,
                                wrist_roll=wrist_roll,
                                side="right",
                            ))
                            arm.upper_calibrated = True
                            arm.forearm_calibrated = True
                            arm.robot_fk_urdf = forward_kinematics_right_arm_full(
                                arm.angles)
                        continue

                    q_u, arm.upper_connected = self._read_imu(dev_dict, arm.upper_node_id)
                    q_f, arm.forearm_connected = self._read_imu(dev_dict, arm.forearm_node_id)
                    upper_device = dev_dict.get(arm.upper_node_id)
                    forearm_device = dev_dict.get(arm.forearm_node_id)
                    upper_rest = bool(
                        arm.upper_connected and upper_device is not None
                        and upper_device.rest)
                    forearm_rest = bool(
                        arm.forearm_connected and forearm_device is not None
                        and forearm_device.rest)
                    if q_u is not None:
                        arm.q_upper = q_u
                    if q_f is not None:
                        arm.q_forearm = q_f

                    # 增量跟踪 + 方向重建
                    if arm.upper_calibrated:
                        arm.upper_tracker.update(
                            self.q_chest, arm.q_upper,
                            chest_rest=chest_rest, link_rest=upper_rest)
                        if arm.upper_tracker.q_rel_inc is not None:
                            arm.arm_dir_chest = arm.upper_calibrator.reconstruct(
                                arm.upper_tracker.q_rel_inc)

                    if arm.forearm_calibrated:
                        arm.forearm_tracker.update(
                            self.q_chest, arm.q_forearm,
                            chest_rest=chest_rest, link_rest=forearm_rest)
                        if arm.forearm_tracker.q_rel_inc is not None:
                            arm.forearm_dir_chest = arm.forearm_calibrator.reconstruct(
                                arm.forearm_tracker.q_rel_inc)

                    # 扭转 + 掌心
                    if arm.twist_cal is not None and arm.forearm_tracker.q_rel_inc is not None:
                        arm.forearm_twist = arm.twist_cal.compute_forearm_twist(
                            arm.forearm_tracker.q_rel_inc)
                        arm.palm_dir_chest = arm.twist_cal.compute_palm_direction(
                            arm.forearm_tracker.q_rel_inc, arm.forearm_calibrator)

                    # 方向→关节角度：使用上一帧热启动的连续受限 IK，
                    # 同时匹配官方 G1 FK 中的大臂、小臂方向与腕部位置。
                    if arm.upper_calibrated:
                        arm.update_mapping_directions(now)
                        if arm.forearm_calibrated:
                            solve_start = time.perf_counter()
                            solved, diagnostics = self._solve_arm_mapping(arm)
                            arm.angles = solved
                            arm.ik_diagnostics = diagnostics
                            elapsed_ms = (
                                time.perf_counter() - solve_start) * 1000.0
                            self._ik_solve_times.append(elapsed_ms)
                            if len(self._ik_solve_times) > 200:
                                self._ik_solve_times = (
                                    self._ik_solve_times[-200:])
                            self._ik_count += 1
                            self._ik_total_time += elapsed_ms
                            self._ik_avg_ms = (
                                self._ik_total_time / self._ik_count)
                            self._ik_p95_ms = float(np.percentile(
                                self._ik_solve_times, 95.0))
                        else:
                            prev_sp = arm.angles.shoulder_pitch
                            sp, sr = _compute_sp_and_sr(
                                arm.mapping_arm_dir_chest,
                                prev_sp, arm.side)
                            arm.angles = JointAngles(
                                sp, sr, 0.0, 0.0, 0.0, side=arm.side)

                        # wrist_roll: 从掌心朝向计算 URDF 腕关节旋转角
                        # 将掌心转到腕关节局部帧, 用 atan2 提取绕 X 轴的旋转
                        if arm.forearm_calibrated and arm.twist_cal is not None:
                            prev_wr = arm.angles.wrist_roll
                            arm.angles.wrist_roll = _compute_wrist_roll_from_palm(
                                arm.palm_dir_chest,
                                arm.angles.shoulder_pitch,
                                arm.angles.shoulder_roll,
                                arm.angles.shoulder_yaw,
                                arm.angles.elbow,
                                prev_wr=prev_wr, side=arm.side)
                        else:
                            arm.angles.wrist_roll = 0.0
                        arm.angles = clamp_joint_angles(arm.angles)

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

                # 自动日志独立于 D 键录制，随原始版同步解算周期保存结果。
                self.session_logger.log_control(now, self)

                # 采样率
                self._sample_n += 1
                dt = now - self._rate_t
                if dt >= 2.0:
                    self.sample_rate = self._sample_n / dt
                    self._sample_n = 0
                    self._rate_t = now

                # 校准数据收集
                if self._calib_step >= 0:
                    self._collect_calib_samples(now)

            time.sleep(WIRELESS_POLL_INTERVAL)

    # ---------- 增强校准（静态稳定门控 + 功能轴 + 独立验证） ----------

    # 一次连续标定的姿态区间：右臂 0..6，左臂 7..13。
    CALIB_ARM_POSE_RANGE = {"right": (0, 6), "left": (7, 13)}

    def _get_calib_arm_poses(self):
        """返回正在标定的手臂在增强标定序列中的 (start_idx, end_idx)。

        以 ``self._calib_arm`` 为准而不是 ``self.active_arm``：连续标定
        过程中允许切换显示焦点，索引不能跟着漂到另一条臂的姿态表。
        """
        side = (self._calib_arm.side if self._calib_arm is not None
                else self.active_arm)
        return self.CALIB_ARM_POSE_RANGE[side]

    @staticmethod
    def _snapshot_arm_calibration(arm):
        """保存进入新标定前的可用结果，以便失败时无损回退。"""
        if not (arm.upper_calibrated and arm.forearm_calibrated):
            return None
        return {
            "upper": copy.deepcopy(arm.upper_calibrator.save_dict()),
            "forearm": copy.deepcopy(arm.forearm_calibrator.save_dict()),
            "twist": (copy.deepcopy(arm.twist_cal.save_dict())
                      if arm.twist_cal is not None else None),
            "quality": copy.deepcopy(arm.calibration_quality),
        }

    def _restore_previous_calibration(self, arm):
        previous = self._calib_previous
        if not previous:
            return False
        arm.upper_calibrator.load_dict(previous["upper"])
        arm.forearm_calibrator.load_dict(previous["forearm"])
        arm.upper_calibrated = True
        arm.forearm_calibrated = True
        arm.twist_cal = (
            TwistCalibration.load_dict(
                previous["twist"], arm.forearm_calibrator)
            if previous.get("twist") is not None else None)
        arm.calibration_quality = copy.deepcopy(previous.get("quality", {}))
        arm.upper_tracker.init(self.q_chest, arm.q_upper)
        arm.forearm_tracker.init(self.q_chest, arm.q_forearm)
        arm.following = False
        self._log("已恢复上一次有效标定，可继续仿真跟随")
        return True

    def _clear_calibration_session(self):
        self._pose_averages = []
        self._palm_pose_results = {}
        self._hinge_result = None
        self._validation_average = None
        self._palm_live_consistency_deg = None
        self._palm_preview_calibrator = None

    def _reject_calibration(self, arm, reason):
        """拒绝本次结果，但优先恢复进入流程前的有效标定。"""
        self._log(reason)
        restored = self._restore_previous_calibration(arm)
        if not restored:
            arm.upper_calibrated = False
            arm.forearm_calibrated = False
            arm.twist_cal = None
            self._log("没有可恢复的旧标定，请重新标定后再跟随")
        self._clear_calibration_session()
        self._calib_previous = None

    def _begin_arm_calibration(self, arm):
        """把标定会话状态重置到 ``arm`` 的第一步，等待用户按 C。

        ``start_calibration`` 和右臂走完后的自动接续都走这里，避免两处
        各自清一半状态。
        """
        self._calib_arm = arm
        self._calib_previous = self._snapshot_arm_calibration(arm)
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

        start_idx, end_idx = self._get_calib_arm_poses()
        self._calib_step = start_idx
        self._calib_waiting = True  # 等待用户按C
        self._calib_start = 0.0
        self._calib_samples_chest = []
        self._calib_samples_upper = []
        self._calib_samples_forearm = []
        self._pose_averages = []
        self._palm_pose_results = {}
        self._hinge_result = None
        self._hinge_live_result = None
        self._palm_live_consistency_deg = None
        self._palm_preview_calibrator = None
        self._calib_last_quality_update = 0.0
        self._validation_average = None
        self._calib_stability_window = []
        self._calib_stable_since = None
        self._calib_stability_deg = float("inf")
        pose = GUIDED_CALIB_POSES[self._calib_step]
        self._log(
            f"{SIDE_CN[arm.side]}臂校准步骤 1/{end_idx - start_idx + 1}: "
            f"请完成 [{pose['name']}], 按 C 开始")

    def start_calibration(self, chain=True):
        """进入校准模式, 显示第一步姿态提示, 等待用户按C开始采集.

        ``chain=True`` 时从右臂第一步开始，右臂通过后自动接左臂，一共
        14 步；``chain=False`` 只标定当前活跃的一条臂。
        """
        if self.right_arm_two_imu:
            arm = self.right
            missing = []
            if not arm.upper_connected:
                missing.append("右大臂IMU")
            if not arm.forearm_connected:
                missing.append("右小臂IMU")
            if missing:
                self._log(f"无法采集零位: {', '.join(missing)} 未连接")
                return
            arm.reset()
            self.two_imu_mapper.calibrate(arm.q_upper, arm.q_forearm)
            arm.upper_calibrated = True
            arm.forearm_calibrated = True
            arm.robot_fk_urdf = forward_kinematics_right_arm_full(arm.angles)
            self._log("右臂双IMU零位已采集: 按 SPACE 开始跟随")
            return

        if self.right_forearm_only:
            arm = self.right
            if not arm.forearm_connected:
                self._log("无法采集零位: 右小臂 IMU 未连接")
                return
            arm.reset()
            self.single_forearm_mapper.calibrate(arm.q_forearm)
            arm.upper_calibrated = True
            arm.forearm_calibrated = True
            arm.robot_fk_urdf = forward_kinematics_right_arm_full(arm.angles)
            self._log("单IMU零位已采集: 肩关节固定，按 SPACE 开始跟随")
            return

        if self._calib_step >= 0:
            return  # 已在校准中

        self._calib_chain = bool(chain)
        if self._calib_chain:
            arm = self.right
            self.active_arm = "right"
            self._log("双臂连续标定：右臂 7 步 → 左臂 7 步，共 14 步")
        else:
            arm = self.active
            self._log(f"单臂标定：只标定{SIDE_CN[arm.side]}臂 7 步")
        self._begin_arm_calibration(arm)

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
        self._calib_stability_window = []
        self._calib_stable_since = None
        self._calib_stability_deg = float("inf")
        pose = GUIDED_CALIB_POSES[self._calib_step]
        if pose["kind"] == "hinge":
            self._hinge_live_result = None
            self._calib_last_quality_update = 0.0
        start_idx, _ = self._get_calib_arm_poses()
        step_num = self._calib_step - start_idx + 1
        if pose["kind"] in (
                "static", "palm_up", "palm_down", "validation"):
            self._log(f"  请保持静止，系统检测稳定后自动采集 {_calib_pose_duration(pose):.0f}s")
            if pose["kind"] in ("palm_up", "palm_down"):
                self._log("  小臂保持向前，手腕伸直；不要只弯曲手腕")
        elif pose["kind"] == "hinge":
            self._log("  大臂自然下垂即可，不要用力夹紧身体")
            self._log("  在舒适范围内匀速屈伸2次；合格后自动通过")
            self._log("  若肘轴精修不理想，将自动使用三姿势基础结果")
        else:
            self._log(f"  动作采集中: {pose['name']} ({_calib_pose_duration(pose):.0f}s)")

    def _retry_calib_step(self, reason):
        """当前步骤质量不合格，保留流程并等待用户重新按 C。"""
        self._calib_waiting = True
        self._calib_start = 0.0
        self._calib_samples_chest = []
        self._calib_samples_upper = []
        self._calib_samples_forearm = []
        self._calib_stability_window = []
        self._calib_stable_since = None
        self._hinge_live_result = None
        self._palm_live_consistency_deg = None
        self._calib_last_quality_update = 0.0
        self._log(f"  本步骤未通过: {reason}，请调整后按 C 重试")

    @staticmethod
    def _robust_relative_motion_range(reference_samples, link_samples):
        """返回 link 相对 reference 姿态变化的 P95，忽略偶发无线坏包。"""
        n = min(len(reference_samples), len(link_samples))
        if n < 3:
            return 0.0
        ref_count = min(10, n)
        q_ref = average_relative_quaternions(
            reference_samples[:ref_count], link_samples[:ref_count])
        deviations = []
        for q_reference, q_link in zip(
                reference_samples[:n], link_samples[:n]):
            q_rel = cal_normalize_quat(cal_quat_mul(
                cal_quat_conj(q_reference), q_link))
            deviations.append(quaternion_angle_deg(q_ref, q_rel))
        return float(np.percentile(deviations, 95.0))

    def _continue_calibration_after_step(self):
        """进入下一校准步骤，或在最后一步后计算最终结果。"""
        start_idx, end_idx = self._get_calib_arm_poses()
        next_step = self._calib_step + 1
        if next_step <= end_idx:
            self._calib_step = next_step
            self._calib_waiting = True
            self._calib_start = 0.0
            self._calib_samples_chest = []
            self._calib_samples_upper = []
            self._calib_samples_forearm = []
            self._palm_live_consistency_deg = None
            pose = GUIDED_CALIB_POSES[self._calib_step]
            step_num = self._calib_step - start_idx + 1
            self._log(
                f"校准步骤 {step_num}/{end_idx - start_idx + 1}: "
                f"请完成 [{pose['name']}], 按 C 开始")
        else:
            finished_arm = self._calib_arm
            accepted = self._compute_calibration(
                finished_arm, start_idx, end_idx)
            self._calib_step = -1
            if not (self._calib_chain and accepted):
                # 单臂标定，或本次结果被拒绝：不要带着坏结果继续下一条臂。
                if self._calib_chain and not accepted:
                    self._log(
                        f"{SIDE_CN[finished_arm.side]}臂未通过，"
                        "双臂连续标定已终止；按 C 可重新开始")
                self._calib_chain = False
                return
            if finished_arm is self.right:
                self._log("右臂已完成并存盘，现在换左臂，按 C 开始")
                self.active_arm = "left"
                self._begin_arm_calibration(self.left)
            else:
                self._calib_chain = False
                self._log("双臂 14 步标定全部完成")

    def _collect_calib_samples(self, now):
        """在 lock 内调用: 收集校准样本. 仅在非等待状态时采集."""
        if self._calib_step < 0 or self._calib_waiting:
            return
        arm = self._calib_arm
        pose = GUIDED_CALIB_POSES[self._calib_step]
        if not (self.chest_connected and arm.upper_connected
                and arm.forearm_connected):
            self._retry_calib_step("采集中有 IMU 断开")
            return

        q_c = self.q_chest.copy()
        q_u = arm.q_upper.copy()
        q_f = arm.q_forearm.copy()

        if pose["kind"] in (
                "static", "palm_up", "palm_down", "validation"):
            # 只接受连续稳定的数据。使用短窗口姿态离散度，不依赖原始陀螺仪。
            self._calib_stability_window.append((now, q_c, q_u, q_f))
            cutoff = now - CALIB_STABILITY_WINDOW
            self._calib_stability_window = [s for s in self._calib_stability_window
                                             if s[0] >= cutoff]
            window_span = (self._calib_stability_window[-1][0]
                           - self._calib_stability_window[0][0])
            if window_span < CALIB_STABILITY_WINDOW * 0.8:
                return
            dispersions = [quaternion_dispersion_deg([s[i] for s in self._calib_stability_window])
                           for i in (1, 2, 3)]
            self._calib_stability_deg = max(dispersions)
            if self._calib_stability_deg > CALIB_MAX_STATIC_DISPERSION_DEG:
                self._calib_stable_since = None
                self._calib_samples_chest = []
                self._calib_samples_upper = []
                self._calib_samples_forearm = []
                return
            if self._calib_stable_since is None:
                self._calib_stable_since = now
                self._calib_samples_chest = []
                self._calib_samples_upper = []
                self._calib_samples_forearm = []
            self._calib_samples_chest.append(q_c)
            self._calib_samples_upper.append(q_u)
            self._calib_samples_forearm.append(q_f)
            if pose["kind"] == "palm_down":
                self._update_palm_live_consistency(now)
            if now - self._calib_stable_since >= _calib_pose_duration(pose):
                self._advance_calibration()
            return

        self._calib_samples_chest.append(q_c)
        self._calib_samples_upper.append(q_u)
        self._calib_samples_forearm.append(q_f)
        if pose["kind"] == "hinge":
            elapsed = now - self._calib_start
            # 每 0.25 秒更新一次质量，合格后最早在 6 秒自动结束。
            if (now - self._calib_last_quality_update >= 0.25
                    or elapsed >= _calib_pose_duration(pose)):
                self._hinge_live_result = fit_hinge_axis(
                    self._calib_samples_chest,
                    self._calib_samples_upper,
                    self._calib_samples_forearm,
                    min_motion_range_deg=CALIB_HINGE_MIN_RANGE_DEG,
                    max_axis_dispersion_deg=(
                        CALIB_MAX_HINGE_AXIS_DISPERSION_DEG),
                    max_upper_motion_deg=(
                        CALIB_MAX_HINGE_UPPER_MOTION_DEG))
                self._calib_last_quality_update = now
            if (elapsed >= CALIB_HINGE_MIN_DURATION
                    and self._hinge_live_result is not None
                    and self._hinge_live_result.valid):
                self._advance_calibration()
                return
        if now - self._calib_start >= _calib_pose_duration(pose):
            self._advance_calibration()

    def _build_palm_preview_calibrator(self):
        """用前三个静态姿势临时解算小臂对齐，供翻掌一致性预览使用。

        正式解算在第 7 步之后才做，并且可能带肘轴精修；这里只用基础
        三姿势解，因此预览值与最终值会有几度偏差。
        """
        if len(self._pose_averages) != 3:
            return None
        calibrator = ArmDirectionCalibrator(label="翻掌预览")
        q_identity = np.array([1., 0., 0., 0.])
        for result in self._pose_averages:
            calibrator.collect_pose(
                q_identity, result["q_rel_forearm"], result["forearm_dir"])
        try:
            calibrator.calibrate()
        except Exception:
            return None
        return calibrator

    def _palm_step_consistency(self, q_rel_forearm_down):
        """在 lock 内调用: 第 6 步样本与第 5 步端点的掌心轴夹角。

        条件不足（缺第 5 步端点或三个静态姿势）时返回 None。
        """
        palm_up = self._palm_pose_results.get("palm_up")
        if palm_up is None:
            return None
        if self._palm_preview_calibrator is None:
            self._palm_preview_calibrator = (
                self._build_palm_preview_calibrator())
            if self._palm_preview_calibrator is None:
                return None
        return palm_pose_consistency_deg(
            palm_up["q_rel_forearm"], q_rel_forearm_down,
            self._palm_preview_calibrator)

    def _update_palm_live_consistency(self, now):
        """在 lock 内调用: 刷新第 6 步与第 5 步的翻掌一致性预览。"""
        if now - self._calib_last_quality_update < 0.25:
            return
        self._calib_last_quality_update = now
        n = min(len(self._calib_samples_chest),
                len(self._calib_samples_forearm))
        if n < 5:
            return
        q_rel_down = average_relative_quaternions(
            self._calib_samples_chest[:n], self._calib_samples_forearm[:n])
        self._palm_live_consistency_deg = self._palm_step_consistency(
            q_rel_down)

    def _advance_calibration(self):
        """在 lock 内调用: 完成当前校准步骤, 进入下一步."""
        arm = self._calib_arm
        pose = GUIDED_CALIB_POSES[self._calib_step]
        n = min(len(self._calib_samples_chest), len(self._calib_samples_upper),
                len(self._calib_samples_forearm))
        if n < 10:
            self._retry_calib_step(f"同步样本不足（{n}帧）")
            return

        kind = pose["kind"]
        # 每条臂的第一步是「自然下垂」，此刻人必然站直正对前方，顺带把
        # 腰部零位记下来，省得再单独摆一次姿势。W 键可随时重新捕获。
        start_idx, _ = self._get_calib_arm_poses()
        if self._calib_step == start_idx and self.chest_connected:
            self.waist.capture_zero(self.q_chest)
            self._log("  腰部零位已同步采集")
        if kind in ("static", "palm_up", "palm_down", "validation"):
            dispersions = [quaternion_dispersion_deg(samples)
                           for samples in (self._calib_samples_chest,
                                           self._calib_samples_upper,
                                           self._calib_samples_forearm)]
            max_dispersion = max(dispersions)
            if max_dispersion > CALIB_MAX_STATIC_DISPERSION_DEG:
                self._retry_calib_step(f"静态离散度 {max_dispersion:.2f}° 过大")
                return
            result = {
                "q_rel_upper": average_relative_quaternions(
                    self._calib_samples_chest, self._calib_samples_upper),
                "q_rel_forearm": average_relative_quaternions(
                    self._calib_samples_chest, self._calib_samples_forearm),
                "upper_dir": np.array(pose["upper_dir"], dtype=float),
                "forearm_dir": np.array(pose["forearm_dir"], dtype=float),
                "dispersion_deg": max_dispersion,
                "samples": n,
            }
            if kind == "static":
                self._pose_averages.append(result)
            elif kind in ("palm_up", "palm_down"):
                result["palm_dir"] = np.array(
                    pose["palm_dir"], dtype=float)
                if kind == "palm_down":
                    # 翻掌一致性原本要到第 7 步之后才判定，不合格会作废
                    # 整套 7 步。这里提前用同一份计算拦下本步重做。
                    consistency = self._palm_step_consistency(
                        result["q_rel_forearm"])
                    if (consistency is not None
                            and consistency
                            > CALIB_MAX_PALM_CONSISTENCY_DEG):
                        self._retry_calib_step(
                            "翻掌一致性 %.1f° > %.1f°，"
                            "手臂保持前平举不动，掌心真正翻到朝下"
                            % (consistency,
                               CALIB_MAX_PALM_CONSISTENCY_DEG))
                        return
                self._palm_pose_results[kind] = result
            else:
                self._validation_average = result
            self._log(f"  静态质量通过: {n}帧，离散度 {max_dispersion:.2f}°")
        elif kind == "hinge":
            hinge = fit_hinge_axis(self._calib_samples_chest,
                                   self._calib_samples_upper,
                                   self._calib_samples_forearm,
                                   min_motion_range_deg=(
                                       CALIB_HINGE_MIN_RANGE_DEG),
                                   max_axis_dispersion_deg=(
                                       CALIB_MAX_HINGE_AXIS_DISPERSION_DEG),
                                   max_upper_motion_deg=(
                                       CALIB_MAX_HINGE_UPPER_MOTION_DEG))
            self._hinge_result = hinge
            self._hinge_live_result = hinge
            if not hinge.valid:
                # 三个静态姿势已经足够得到基础映射；功能肘轴只作为精修，
                # 不应因人体动作不够标准而阻塞整套校准。
                self._log(
                    f"  肘轴精修未采用: {hinge.reason}；"
                    "继续使用三姿势基础结果")
            else:
                self._log("  肘轴通过: 范围 %.1f°，轴离散 %.1f°，大臂代偿 %.1f°" %
                          (hinge.motion_range_deg, hinge.axis_dispersion_deg,
                           hinge.upper_motion_deg))
        self._continue_calibration_after_step()

    def _compute_calibration(self, arm, start_idx, end_idx):
        """在 lock 内调用: 计算校准结果。

        返回 True 表示本次结果已被采用并存盘；返回 False 表示已拒绝，
        连续标定链应当就此终止而不是带着坏结果继续下一条臂。
        """
        self._log(f"计算{SIDE_CN[arm.side]}臂校准...")

        if (len(self._pose_averages) != 3 or self._hinge_result is None
                or set(self._palm_pose_results) != {
                    "palm_up", "palm_down"}
                or self._validation_average is None):
            self._reject_calibration(
                arm, "校准失败: 数据不完整，本次结果未采用")
            return False

        # 平均同步相对四元数已经计算完成，这里用单位胸部姿态直接收集。
        q_identity = np.array([1., 0., 0., 0.])
        for result in self._pose_averages:
            arm.upper_calibrator.collect_pose(
                q_identity, result["q_rel_upper"], result["upper_dir"])
            arm.forearm_calibrator.collect_pose(
                q_identity, result["q_rel_forearm"], result["forearm_dir"])

        success = True
        hinge_refinement_used = False
        # 先计算纯三姿势解。肘轴作为可回退的精修约束，
        # 不能因为功能轴精修变差而毁掉原本可用的静态结果。
        try:
            arm.upper_calibrator.calibrate()
        except Exception as e:
            self._log(f"大臂校准失败: {e}")
            success = False

        try:
            arm.forearm_calibrator.calibrate()
        except Exception as e:
            self._log(f"小臂校准失败: {e}")
            success = False

        if not success:
            self._reject_calibration(
                arm, "基础姿势无法求解，本次结果未采用")
            return False

        base_upper = copy.deepcopy(arm.upper_calibrator.save_dict())
        base_forearm = copy.deepcopy(arm.forearm_calibrator.save_dict())
        base_static_max = max(
            arm.upper_calibrator.pose_errors_deg()
            + arm.forearm_calibrator.pose_errors_deg())

        # 功能肘轴提供两条骨段纵轴应与铰链轴正交的约束。
        # 若修正需求过大或静态误差恶化，保留基础解而不是判整套失败。
        if self._hinge_result is not None and self._hinge_result.valid:
            upper_ok, upper_correction = arm.upper_calibrator.refine_with_hinge_axis(
                self._hinge_result.upper_axis)
            forearm_ok, forearm_correction = arm.forearm_calibrator.refine_with_hinge_axis(
                self._hinge_result.forearm_axis)
            refined_static_max = max(
                arm.upper_calibrator.pose_errors_deg()
                + arm.forearm_calibrator.pose_errors_deg())
            if (upper_ok and forearm_ok
                    and refined_static_max <= base_static_max + 3.0):
                hinge_refinement_used = True
            else:
                arm.upper_calibrator.load_dict(base_upper)
                arm.forearm_calibrator.load_dict(base_forearm)
                self._log("  肘轴精修与静态姿势不一致，已自动退回基础解")

        static_max = max(
            arm.upper_calibrator.pose_errors_deg()
            + arm.forearm_calibrator.pose_errors_deg())
        self._log(f"  三姿势拟合最大误差: {static_max:.1f}°")

        # 用未参与拟合的“肘90°”姿势做真正的保存前验证。
        validation_errors = {}
        validation = self._validation_average
        upper_pred = cal_normalize_vec(arm.upper_calibrator.reconstruct(
            validation["q_rel_upper"]))
        forearm_pred = cal_normalize_vec(arm.forearm_calibrator.reconstruct(
            validation["q_rel_forearm"]))
        upper_target = cal_normalize_vec(validation["upper_dir"])
        forearm_target = cal_normalize_vec(validation["forearm_dir"])
        validation_errors = {
            "upper_deg": float(np.degrees(np.arccos(np.clip(
                np.dot(upper_pred, upper_target), -1.0, 1.0)))),
            "forearm_deg": float(np.degrees(np.arccos(np.clip(
                np.dot(forearm_pred, forearm_target), -1.0, 1.0)))),
        }
        self._log("  独立验证: 大臂 %.1f°，小臂 %.1f°" %
                  (validation_errors["upper_deg"],
                   validation_errors["forearm_deg"]))

        # 两个翻掌端点既校准掌心轴，也额外验证小臂是否确实向前。
        palm_pose_direction_errors = {}
        for kind in ("palm_up", "palm_down"):
            palm_pose = self._palm_pose_results[kind]
            forearm_pred = cal_normalize_vec(
                arm.forearm_calibrator.reconstruct(
                    palm_pose["q_rel_forearm"]))
            forearm_target = cal_normalize_vec(palm_pose["forearm_dir"])
            palm_pose_direction_errors[kind] = float(np.degrees(
                np.arccos(np.clip(
                    np.dot(forearm_pred, forearm_target), -1.0, 1.0))))
        validation_errors.update({
            "palm_up_forearm_deg":
                palm_pose_direction_errors["palm_up"],
            "palm_down_forearm_deg":
                palm_pose_direction_errors["palm_down"],
        })
        max_validation = max(validation_errors.values())
        self._log(
            "  翻掌姿势方向: 朝上 %.1f°，朝下 %.1f°" %
            (palm_pose_direction_errors["palm_up"],
             palm_pose_direction_errors["palm_down"]))

        usable, robot_safe, quality_grade = _calibration_quality_grade(
            static_max, max_validation)
        if not usable:
            self._reject_calibration(
                arm,
                "校准误差过大: 静态 %.1f° / 验证 %.1f°，本次结果未采用" %
                (static_max, max_validation))
            return False

        # 用明确的掌心朝上/朝下端点求掌心轴；不再依赖侧平举时
        # “默认掌心朝下”的单点假设。
        try:
            arm.twist_cal = TwistCalibration.from_palm_poses(
                self._palm_pose_results["palm_up"]["q_rel_forearm"],
                self._palm_pose_results["palm_down"]["q_rel_forearm"],
                arm.forearm_calibrator,
                max_consistency_deg=CALIB_MAX_PALM_CONSISTENCY_DEG)
        except ValueError as exc:
            self._reject_calibration(
                arm, f"手掌校准失败: {exc}，本次结果未采用")
            return False
        palm_quality = {
            "consistency_deg":
                arm.twist_cal.palm_pose_consistency_deg,
            "up_error_deg": arm.twist_cal.palm_up_error_deg,
            "down_error_deg": arm.twist_cal.palm_down_error_deg,
        }
        self._log(
            "  手掌校准完成: 朝上误差 %.1f°，朝下误差 %.1f°，"
            "双姿势一致性 %.1f°" %
            (palm_quality["up_error_deg"],
             palm_quality["down_error_deg"],
             palm_quality["consistency_deg"]))

        arm.upper_calibrated = True
        arm.forearm_calibrated = True

        # 初始化增量跟踪
        arm.upper_tracker.init(self.q_chest, arm.q_upper)
        arm.forearm_tracker.init(self.q_chest, arm.q_forearm)

        # WR 零位由 URDF 固定 (wr_ref=0 右, π 左), 无需校准计算

        arm.calibration_quality = {
            "static_max_error_deg": static_max,
            "hinge": self._hinge_result.save_dict(),
            "validation": validation_errors,
            "max_validation_error_deg": max_validation,
            "grade": quality_grade,
            "safe_for_robot": robot_safe,
            "palm": palm_quality,
            "hinge_refinement_used": hinge_refinement_used,
            "recommended_limit_deg": 20.0,
        }

        # 清理并保存可用结果。
        self._clear_calibration_session()
        self._calib_previous = None
        if robot_safe:
            self._log(f"{SIDE_CN[arm.side]}臂校准完成，误差≤20°，可进入真机前验证")
        else:
            self._log(
                f"{SIDE_CN[arm.side]}臂校准完成：仿真可用；误差超过20°，暂不允许真机发送")
        self._save_calibration(arm)
        return True

    def _save_calibration(self, arm):
        fname = f"imu_motor_calibration_{arm.side}.json"
        path = os.path.join(_MY_DIR, fname)
        data = {
            "version": 5,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "quality": arm.calibration_quality,
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

            if data.get("version") in (2, 3, 4, 5):
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
                arm.calibration_quality = data.get("quality", {})
                ts = data.get("timestamp", "未知")
                version = data.get("version")
                self._log(f"{SIDE_CN[arm.side]}臂已加载校准 v{version} (保存于 {ts})")
                loaded_any = True
            elif "heading_rot" in data:
                # 旧格式兼容提示
                self._log(f"{SIDE_CN[arm.side]}臂校准文件为旧格式, 请重新校准")
        return loaded_any

    # ---------- 固定周期 IK ----------

    @staticmethod
    def _copy_joint_angles(angles):
        return JointAngles(
            float(angles.shoulder_pitch),
            float(angles.shoulder_roll),
            float(angles.shoulder_yaw),
            float(angles.elbow),
            float(angles.wrist_roll),
            side=angles.side,
        )

    def _queue_ik_input(self, arm, sample_time):
        with self._ik_lock:
            self._ik_input_seq += 1
            self._ik_input[arm.side] = {
                "seq": self._ik_input_seq,
                "sample_time": float(sample_time),
                "arm_dir": arm.mapping_arm_dir_chest.copy(),
                "forearm_dir": arm.mapping_forearm_dir_chest.copy(),
                "palm_dir": arm.palm_dir_chest.copy(),
                "wrist_roll": float(arm.angles.wrist_roll),
                "seed": self._copy_joint_angles(arm.angles),
            }
        self._ik_event.set()

    def _apply_ik_results_locked(self):
        with self._ik_lock:
            results = {
                "right": self._ik_result_right,
                "left": self._ik_result_left,
            }
        for arm in (self.right, self.left):
            result = results[arm.side]
            if result is None or result["seq"] <= arm._ik_result_seq:
                continue
            arm._ik_result_seq = result["seq"]
            if result.get("angles") is not None:
                arm.angles = result["angles"]
                arm.ik_diagnostics = result["diagnostics"]

    def _start_ik_thread(self):
        # 与用户指定的原始流畅版保持一致：IK 在无线主链路内同步热启动，
        # 避免独立线程的结果到达时序与 60 Hz 渲染互相拍频。
        self._log("IK模式: 原始版同步解算")

    def _stop_ik_thread(self):
        pass

    def _ik_loop(self):
        next_deadline = time.monotonic()
        last_seq = {"right": -1, "left": -1}
        while self._ik_running:
            now = time.monotonic()
            wait = next_deadline - now
            if wait > 0:
                self._ik_event.wait(timeout=wait)
                self._ik_event.clear()
                now = time.monotonic()
                if now < next_deadline:
                    continue
            elif -wait > 3.0 * IK_SOLVE_DT:
                next_deadline = now
            next_deadline += IK_SOLVE_DT

            with self._ik_lock:
                inputs = {
                    side: dict(value)
                    for side, value in self._ik_input.items()
                }

            for side in ("right", "left"):
                item = inputs.get(side)
                if item is None or item["seq"] <= last_seq[side]:
                    continue
                solve_start = time.perf_counter()
                try:
                    snapshot_arm = SimpleNamespace(
                        side=side,
                        mapping_arm_dir_chest=item["arm_dir"],
                        mapping_forearm_dir_chest=item["forearm_dir"],
                        palm_dir_chest=item["palm_dir"],
                        angles=item["seed"],
                    )
                    solved, diagnostics = self._solve_arm_mapping(
                        snapshot_arm)
                    solved = clamp_joint_angles(solved)
                    result = {
                        "seq": item["seq"],
                        "sample_time": item["sample_time"],
                        "angles": solved,
                        "diagnostics": diagnostics,
                    }
                except Exception as exc:
                    result = {
                        "seq": item["seq"],
                        "sample_time": item["sample_time"],
                        "angles": None,
                        "diagnostics": {"error": str(exc)},
                    }
                elapsed_ms = (time.perf_counter() - solve_start) * 1000.0
                self._ik_solve_times.append(elapsed_ms)
                if len(self._ik_solve_times) > 200:
                    self._ik_solve_times = self._ik_solve_times[-200:]
                self._ik_count += 1
                self._ik_total_time += elapsed_ms
                self._ik_avg_ms = self._ik_total_time / self._ik_count
                self._ik_p95_ms = float(np.percentile(
                    self._ik_solve_times, 95.0))
                with self._ik_lock:
                    if side == "right":
                        self._ik_result_right = result
                    else:
                        self._ik_result_left = result
                last_seq[side] = item["seq"]

    def _solve_arm_mapping(self, arm):
        """同步求解入口，保留给诊断和外部适配代码调用。"""
        return solve_direction_ik(
            arm.mapping_arm_dir_chest,
            arm.mapping_forearm_dir_chest,
            wrist_roll=arm.angles.wrist_roll,
            side=arm.side,
            seed=arm.angles,
        )

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

    def _draw_torso(self, offset=None):
        """绘制人体参考躯干（GLB 素材加载失败时的回退）。"""
        offset = HUMAN_MODEL_OFFSET if offset is None else np.asarray(offset)
        glPushMatrix()
        glTranslatef(*offset)
        draw_box(*TORSO_HALF, (0.20, 0.28, 0.38), alpha=0.32)
        draw_sphere(HEAD_POS, HEAD_RADIUS, (0.58, 0.70, 0.78), 0.62)
        glPopMatrix()

    @staticmethod
    def _human_root_transform():
        return HumanGltfRenderer.root_transform(
            HUMAN_MODEL_OFFSET, HUMAN_MODEL_SCALE)

    def _draw_imu_module(self, position, direction, color, scale=1.0):
        """在骨骼表面绘制小型 IMU 模块，明确三传感器安装位置。"""
        direction = np.asarray(direction, dtype=float)
        direction /= max(float(np.linalg.norm(direction)), 1e-12)
        glPushMatrix()
        glTranslatef(*position)
        self._apply_dir_rotation(direction)
        draw_box(0.036 * scale, 0.023 * scale, 0.012 * scale,
                 color, alpha=0.98)
        glPopMatrix()

    def _draw_human_imu_modules(self, right, left, root_transform):
        if not self.human_renderer_ready or self.human_renderer is None:
            return
        renderer = self.human_renderer

        if self.chest_connected:
            chest = renderer.bone_position(root_transform, "DEF-spine.003")
            # 模型正面朝 display +Y，稍微抬离胸腔避免 Z-fighting。
            self._draw_imu_module(
                chest + np.array([0., 0.065, 0.015]),
                np.array([1., 0., 0.]), (0.18, 0.78, 0.88), 1.15)

        for arm, suffix in ((right, "R"), (left, "L")):
            if arm.upper_connected:
                center, direction = renderer.segment_midpoint(
                    root_transform, f"DEF-upper_arm.{suffix}",
                    f"DEF-forearm.{suffix}")
                self._draw_imu_module(
                    center + np.array([0., 0.028, 0.]), direction,
                    (0.24, 0.82, 0.52))
            if arm.forearm_connected:
                center, direction = renderer.segment_midpoint(
                    root_transform, f"DEF-forearm.{suffix}",
                    f"DEF-hand.{suffix}")
                self._draw_imu_module(
                    center + np.array([0., 0.024, 0.]), direction,
                    (0.25, 0.58, 0.95), 0.90)

    def _g1_root_transform(self):
        transform = np.eye(4)
        if self._g1_full_body:
            transform[:3, :3] = T_URDF2DISP * G1_MODEL_SCALE
            transform[:3, 3] = G1_MODEL_OFFSET
        else:
            # 上半身模型按原尺度摆放, 保持换模型之前的画面。
            transform[:3, :3] = T_URDF2DISP
            transform[:3, 3] = G1_ARMS_MODEL_OFFSET
        return transform

    def _render_scene(self, right, left):
        """渲染主3D场景 (全屏)."""
        # 最大化、最小化恢复或跨 DPI 显示器移动时 drawable 会变化。
        self._sync_display_size()
        width, height = self._drawable_size()
        glViewport(0, 0, width, height)
        glClearColor(0.025, 0.037, 0.055, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_PROJECTION); glLoadIdentity()
        gluPerspective(45, width / height, 0.05, 20.0)
        glMatrixMode(GL_MODELVIEW); glLoadIdentity()

        cx = self._cam_dist * math.sin(math.radians(self._cam_rot_y)) * math.cos(math.radians(self._cam_rot_x))
        cy = self._cam_dist * math.cos(math.radians(self._cam_rot_y)) * math.cos(math.radians(self._cam_rot_x))
        cz = self._cam_dist * math.sin(math.radians(self._cam_rot_x))
        gluLookAt(cx, cy, cz + 0.38, 0.05, 0, 0.40, 0, 0, 1)

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        draw_grid(1.5, 0.1)

        r_active = self.active_arm == "right"
        r_alpha = 1.0 if r_active else 0.5
        l_alpha = 1.0 if not r_active else 0.5

        # 带骨骼人体数字替身。在线/已标定手臂由 IMU 方向驱动，
        # 离线手臂保持素材自然站姿。
        if self.human_renderer_ready and self.human_renderer is not None:
            human_arms = {}
            for arm in (right, left):
                if (arm.upper_connected or arm.forearm_connected
                        or arm.upper_calibrated or arm.forearm_calibrated):
                    human_arms[arm.side] = (
                        arm.arm_dir_display,
                        arm.forearm_dir_display,
                        arm.forearm_twist,
                    )
            human_root = self._human_root_transform()
            active = right if self.active_arm == "right" else left
            self.human_renderer.render(
                human_root, human_arms,
                active_side=self.active_arm, following=active.following)
            self._draw_human_imu_modules(right, left, human_root)
        else:
            self._draw_torso()
            if right.upper_connected or right.upper_calibrated:
                self._draw_human_arm(
                    RIGHT_SHOULDER_DISP + HUMAN_MODEL_OFFSET, right, r_alpha)
            if left.upper_connected or left.upper_calibrated:
                self._draw_human_arm(
                    LEFT_SHOULDER_DISP + HUMAN_MODEL_OFFSET, left, l_alpha)

        # 官方 G1 URDF 数字孪生；资源加载失败时退回原简化连杆。
        if self.g1_renderer_ready and self.g1_renderer is not None:
            active = right if self.active_arm == "right" else left
            self.g1_renderer.render(
                self._g1_root_transform(), right.angles, left.angles,
                active_side=self.active_arm, following=active.following,
                waist_yaw=self._waist_render_yaw,
                hand_joints=self._hand_render_joints())
        else:
            if right.robot_fk_urdf is not None:
                self._draw_robot_arm(right.robot_fk_urdf, "right", r_alpha)
            if left.robot_fk_urdf is not None:
                self._draw_robot_arm(left.robot_fk_urdf, "left", l_alpha)

    def _render_hud(self, screen, font, font_sm, font_lg, right, left):
        """渲染现代化控制台 HUD。"""
        width, height = self._display_size()
        drawable_width, drawable_height = self._drawable_size()
        scale_x = width / max(drawable_width, 1)
        scale_y = height / max(drawable_height, 1)
        # 在切换到 2D 投影前保存两个模型标题的屏幕位置，使标题随相机旋转。
        def project_label(world_point):
            try:
                sx, sy, _ = gluProject(
                    *world_point,
                    glGetDoublev(GL_MODELVIEW_MATRIX),
                    glGetDoublev(GL_PROJECTION_MATRIX),
                    glGetIntegerv(GL_VIEWPORT))
                return int(sx * scale_x), int(
                    height - sy * scale_y)
            except Exception:
                return None

        # 标签浮在头顶上方约 0.06, 与人体标签一致。全身模型的高度随
        # G1_MODEL_SCALE 变, 所以那一档是算出来的而不是写死的。
        g1_label_height = G1_LABEL_HEIGHT if self._g1_full_body else 0.62
        g1_label_pos = project_label(
            G1_MODEL_OFFSET + np.array([0., 0., g1_label_height]))
        human_label_pos = project_label(HUMAN_MODEL_OFFSET + np.array([0., 0., 0.84]))
        surface = pygame.Surface((width, height), pygame.SRCALPHA)

        bg = (9, 17, 27, 224)
        bg_soft = (14, 26, 40, 214)
        border = (62, 84, 105, 150)
        text_primary = (232, 240, 247)
        text_secondary = (139, 158, 177)
        cyan = (61, 196, 226)
        green = (67, 211, 151)
        amber = (247, 184, 74)
        red = (245, 99, 99)

        def panel(rect, fill=bg_soft, outline=border, radius=14):
            x, y, w, h = rect
            pygame.draw.rect(surface, (0, 0, 0, 80),
                             (x + 4, y + 6, w, h), border_radius=radius)
            pygame.draw.rect(surface, fill, rect, border_radius=radius)
            pygame.draw.rect(surface, outline, rect, 1, border_radius=radius)

        def pill(text, x, y, color, width=None):
            tw, _ = font_sm.size(text)
            w = width or tw + 30
            pygame.draw.rect(surface, (*color, 34), (x, y, w, 28),
                             border_radius=14)
            pygame.draw.circle(surface, color, (x + 13, y + 14), 4)
            draw_text_2d(surface, text, (x + 23, y + 5), font_sm, text_primary)
            return w

        # 顶部应用栏
        panel((18, 16, width - 36, 64), fill=bg)
        pygame.draw.rect(surface, cyan, (18, 16, 5, 64), border_radius=3)
        draw_text_2d(surface, "G1 MOTION STUDIO", (40, 25), font, text_primary)
        if self.right_forearm_only:
            arm_label = "右小臂 · 单 IMU 诊断"
        elif self.right_arm_two_imu:
            arm_label = "右臂 · 双 IMU 诊断"
        else:
            selected_arm = right if self.active_arm == "right" else left
            side_label = "右臂" if self.active_arm == "right" else "左臂"
            if right.following and left.following:
                side_label = "双臂"
            if selected_arm.ik_diagnostics.get("official_adapter", False):
                arm_label = f"宇树官方结构 · {side_label}三 IMU 适配"
            else:
                arm_label = f"{side_label}追踪"
        draw_text_2d(surface, arm_label, (40, 51), font_sm, text_secondary)

        n_imu = sum(1 for d in self.latest_devices if d.connected) if self.latest_devices else 0
        target = self._udp_target or ((self.robot_ip, self.robot_port) if self.robot_discovered else None)
        right_edge = width - 36
        model_text = "G1 URDF" if self.g1_renderer_ready else "简化模型"
        model_color = cyan if self.g1_renderer_ready else amber
        w_model = font_sm.size(model_text)[0] + 30
        right_edge -= w_model
        pill(model_text, right_edge, 34, model_color, w_model)
        robot_text = (f"UDP {target[0]}:{target[1]}" if target else "UDP 未连接")
        robot_color = green if target else amber
        w_robot = min(235, font_sm.size(robot_text)[0] + 30)
        right_edge -= w_robot + 10
        pill(robot_text, right_edge, 34, robot_color, w_robot)
        rx_rates = [
            float(d.pkt_rate_hz) for d in self.latest_devices
            if d.connected and float(d.pkt_rate_hz) > 0.0]
        rx_rate = min(rx_rates) if rx_rates else 0.0
        imu_text = (
            f"IMU {n_imu} · RX {rx_rate:.0f} · CTRL {self.sample_rate:.0f}"
            if rx_rates else
            f"IMU {n_imu} · CTRL {self.sample_rate:.0f}")
        w_imu = font_sm.size(imu_text)[0] + 30
        right_edge -= w_imu + 10
        pill(imu_text, right_edge, 34, green if n_imu else red, w_imu)

        # 腰是全身量而不是某条臂的，和 IMU/UDP 同级放在顶栏。
        if self.waist_enabled:
            if not self.waist.calibrated:
                waist_text, waist_color = "腰 待零位", amber
            else:
                waist_text = (f"腰 {np.degrees(self.waist.yaw):+.0f}°"
                              f" ×{self.waist.gain:.1f}")
                waist_color = amber if self.waist.clamped else green
            w_waist = font_sm.size(waist_text)[0] + 30
            right_edge -= w_waist + 10
            pill(waist_text, right_edge, 34, waist_color, w_waist)
        if self.recorder.recording:
            rec_text = f"REC {self.recorder.frame_count}"
            w_rec = font_sm.size(rec_text)[0] + 30
            right_edge -= w_rec + 10
            pill(rec_text, right_edge, 34, red, w_rec)

        # 两个模型标题锚定在三维模型上方，拖动相机后仍不会标反。
        if g1_label_pos is not None:
            label = "UNITREE G1 / URDF"
            draw_text_2d(surface, label,
                         (g1_label_pos[0] - font.size(label)[0] // 2,
                          max(94, g1_label_pos[1] - 20)),
                         font, (196, 207, 218))
        if human_label_pos is not None:
            label = "HUMAN / 3-IMU RIG"
            draw_text_2d(surface, label,
                         (human_label_pos[0] - font.size(label)[0] // 2,
                          max(94, human_label_pos[1] - 20)),
                         font, (118, 190, 216))

        # 左侧追踪状态卡：双臂并排，活跃臂用高亮列头标出
        panel((18, 96, 322, 416))
        draw_text_2d(surface, "TRACKING", (38, 116), font, text_primary)
        active = right if self.active_arm == "right" else left
        if right.following and left.following:
            follow_text, follow_color = "双臂跟随", green
        elif right.following:
            follow_text, follow_color = "右臂跟随", green
        elif left.following:
            follow_text, follow_color = "左臂跟随", green
        else:
            follow_text, follow_color = "安全暂停", amber
        pill(follow_text, 208, 111, follow_color, 110)

        imu_rows = [
            ("胸部 IMU", self.chest_connected),
            ("右大臂 IMU", right.upper_connected),
            ("右小臂 IMU", right.forearm_connected),
            ("左大臂 IMU", left.upper_connected),
            ("左小臂 IMU", left.forearm_connected),
        ]
        row_y = 150
        for label, connected in imu_rows:
            pygame.draw.circle(surface, green if connected else red,
                               (43, row_y + 7), 5)
            draw_text_2d(surface, label, (57, row_y), font_sm, text_primary)
            draw_text_2d(surface, "ONLINE" if connected else "OFFLINE",
                         (245, row_y), font_sm, green if connected else red)
            row_y += 26

        pygame.draw.line(surface, border, (38, 284), (320, 284), 1)

        # 双臂两列表格；活跃臂列头加下划线，不依赖特殊字形。
        col_x = {"right": 176, "left": 256}
        for side, label in (("right", "右臂"), ("left", "左臂")):
            is_active = self.active_arm == side
            draw_text_2d(surface, label, (col_x[side], 294), font_sm,
                         cyan if is_active else text_secondary)
            if is_active:
                width_px = font_sm.size(label)[0]
                pygame.draw.line(surface, cyan,
                                 (col_x[side], 311),
                                 (col_x[side] + width_px, 311), 2)

        def _arm_cal_cell(arm):
            if not arm.upper_calibrated:
                return "等待标定", amber
            if arm.calibration_quality.get("grade") == "simulation_only":
                return "仿真可用", amber
            return "标定通过", green

        def _arm_angle_cell(arm, value):
            # 未标定时角度没有物理含义，显示 -- 而不是看似归零的 +0°。
            if not arm.upper_calibrated:
                return "--", text_secondary
            return f"{np.degrees(value):+.0f}°", text_primary

        table_rows = [
            ("标定", lambda arm: _arm_cal_cell(arm)),
            ("跟随", lambda arm: (("实时跟随", green) if arm.following
                                  else ("暂停", text_secondary))),
        ]
        for name, attr in (("SP", "shoulder_pitch"), ("SR", "shoulder_roll"),
                           ("SY", "shoulder_yaw"), ("EL", "elbow"),
                           ("WR", "wrist_roll")):
            table_rows.append(
                (name,
                 lambda arm, attr=attr: _arm_angle_cell(
                     arm, getattr(arm.angles, attr))))

        for index, (label, cell) in enumerate(table_rows):
            y = 318 + index * 24
            draw_text_2d(surface, label, (38, y), font_sm, text_secondary)
            for side, arm in (("right", right), ("left", left)):
                text, color = cell(arm)
                draw_text_2d(surface, text, (col_x[side], y), font_sm, color)

        ik_error = active.ik_diagnostics.get("max_error_deg")
        if ik_error is not None:
            official_6d = bool(
                active.ik_diagnostics.get("official_6d_pose", False))
            posture_assisted = bool(
                active.ik_diagnostics.get("posture_assisted", False))
            acceptable_error = 10.0 if posture_assisted else 3.0
            palm_error = active.ik_diagnostics.get("palm_error_deg")
            palm_ok = palm_error is None or palm_error <= 10.0
            ik_color = (green if ik_error <= acceptable_error and palm_ok
                        else amber)
            side_prefix = "右臂" if self.active_arm == "right" else "左臂"
            if official_6d and palm_error is not None:
                ik_label = f"{side_prefix} 6D 方向 {ik_error:.1f}°  掌心"
                ik_value = f"{palm_error:.1f}°"
            else:
                ik_label = (f"{side_prefix} 自然姿态权衡" if posture_assisted
                            else f"{side_prefix} 映射误差")
                ik_value = f"{ik_error:.1f}°"
            draw_text_2d(
                surface, f"{ik_label}  {ik_value}",
                (38, 482), font_sm, ik_color)

        # 事件日志卡：贴着底部快捷键栏，窗口变高时只拉开与 TRACKING 的间距。
        activity_h = 130
        activity_y = height - 58 - 12 - activity_h
        panel((18, activity_y, 322, activity_h))
        draw_text_2d(surface, "ACTIVITY", (38, activity_y + 18), font,
                     text_primary)
        log_y = activity_y + 48
        # 按像素而不是字符数截断：中英文混排时字符数量与实际宽度无关。
        log_max_px = 322 - (55 - 18) - 18
        for msg in self._logs[-4:]:
            clean = msg.split("] ", 1)[-1]
            if font_sm.size(clean)[0] > log_max_px:
                while clean and font_sm.size(clean + "…")[0] > log_max_px:
                    clean = clean[:-1]
                clean += "…"
            pygame.draw.circle(surface, cyan, (43, log_y + 7), 3)
            draw_text_2d(surface, clean, (55, log_y), font_sm, text_secondary)
            log_y += 20

        # 底部快捷键栏
        if self.right_forearm_only:
            hint = "C:采集当前零位  SPACE:跟随/暂停  X:重置零位  F11:全屏  Q:退出"
        elif self.right_arm_two_imu:
            hint = "C:同时采集大/小臂零位  SPACE:跟随/暂停  X:重置零位  F11:全屏  Q:退出"
        elif self._calib_step >= 0:
            hint = "C:开始采集  X:取消校准"
        else:
            hint = ("R:右臂 L:左臂 SPACE:单臂跟随 B:双臂跟随 "
                    "C:双臂校准 Shift+C:仅当前臂 W:腰部零位 "
                    "Shift+W:腰部开关 [ ]:腰部增益 X:重置 D:录制 Q:退出")
        panel((18, height - 58, width - 36, 40),
              fill=(8, 15, 24, 232), radius=12)
        draw_text_2d(surface, hint, (38, height - 48), font_sm, text_secondary)

        # 校准居中工作流卡片
        if self._calib_step >= 0 and self._calib_arm is not None:
            pose = GUIDED_CALIB_POSES[self._calib_step]
            start_idx, end_idx = self._get_calib_arm_poses()
            step_num = self._calib_step - start_idx + 1
            step_count = end_idx - start_idx + 1
            arm = self._calib_arm

            # 检查 IMU 连接状态
            imu_ok = self.chest_connected and arm.upper_connected and arm.forearm_connected

            # 聚焦场景区域，同时保留顶部连接状态和底部快捷键可见。
            pygame.draw.rect(surface, (2, 7, 12, 92),
                             (0, 82, width, height - 142))
            box_w = min(760, width - 36)
            box_h = 250
            box_x = (width - box_w) // 2
            box_y = (height - box_h) // 2
            panel((box_x, box_y, box_w, box_h), fill=(10, 22, 35, 248),
                  outline=(*cyan, 210), radius=18)
            draw_text_2d(surface, "CALIBRATION WORKFLOW",
                         (box_x + 28, box_y + 22), font_sm, cyan)
            # 连续标定时按 14 步整体计数，进度点分成左右两组便于分辨。
            side_cn = "右臂" if arm.side == "right" else "左臂"
            if self._calib_chain:
                dot_count = len(GUIDED_CALIB_POSES)
                dot_index = self._calib_step
                group_size = step_count
            else:
                dot_count = step_count
                dot_index = step_num - 1
                group_size = 0
            progress_text = (
                f"步骤 {dot_index + 1} / {dot_count} · {side_cn}")
            draw_text_2d(
                surface, progress_text,
                (box_x + box_w - 28 - font_sm.size(progress_text)[0],
                 box_y + 22),
                font_sm, text_secondary)

            def _dot_x(index):
                gap = 28 if group_size and index >= group_size else 0
                return box_x + 34 + index * 34 + gap

            dots_y = box_y + 57
            for i in range(dot_count):
                dot_color = (green if i < dot_index else
                             cyan if i == dot_index else (62, 79, 96))
                pygame.draw.circle(surface, dot_color, (_dot_x(i), dots_y), 6)
                # 两条臂之间断开连线，否则 14 个点看起来仍是一整条。
                group_break = group_size and (i + 1) == group_size
                if i < dot_count - 1 and not group_break:
                    pygame.draw.line(surface, dot_color,
                                     (_dot_x(i) + 6, dots_y),
                                     (_dot_x(i + 1) - 6, dots_y), 2)

            draw_text_2d(surface, pose["name"],
                         (box_x + 28, box_y + 82), font_lg, text_primary)
            ty = box_y + 132
            pose_hint = pose.get("hint")

            if not imu_ok:
                missing = []
                if not self.chest_connected:
                    missing.append("胸部")
                if not arm.upper_connected:
                    missing.append("大臂")
                if not arm.forearm_connected:
                    missing.append("小臂")
                draw_text_2d(surface, f"等待连接：{', '.join(missing)} IMU",
                             (box_x + 28, ty), font, red)
            elif self._calib_waiting:
                waiting_text = "姿势准备好后按 C 开始采集"
                draw_text_2d(surface, waiting_text,
                             (box_x + 28, ty), font, amber)
                if pose_hint:
                    draw_text_2d(surface, pose_hint,
                                 (box_x + 28, ty + 31), font_sm, text_secondary)
            else:
                duration = _calib_pose_duration(pose)
                if pose["kind"] in (
                        "static", "palm_up", "palm_down", "validation"):
                    elapsed = (0.0 if self._calib_stable_since is None else
                               time.monotonic() - self._calib_stable_since)
                else:
                    elapsed = time.monotonic() - self._calib_start
                progress = min(1.0, elapsed / duration)
                remaining = max(0.0, duration - elapsed)

                bar_x = box_x + 28
                bar_w = box_w - 56
                bar_h = 16
                pygame.draw.rect(surface, (39, 55, 70, 230),
                                 (bar_x, ty, bar_w, bar_h), border_radius=6)
                fill_w = int(bar_w * progress)
                if fill_w > 0:
                    pygame.draw.rect(surface, green,
                                     (bar_x, ty, fill_w, bar_h), border_radius=6)
                if pose["kind"] in (
                        "static", "palm_up", "palm_down", "validation"):
                    dispersion = ("--" if not np.isfinite(self._calib_stability_deg)
                                  else f"{self._calib_stability_deg:.2f}°")
                    state = (f"正在检测稳定性 · 姿态离散 {dispersion}"
                             if self._calib_stable_since is None
                             else f"姿态稳定 · 剩余 {remaining:.1f} 秒")
                else:
                    state = f"动作采集中 · 剩余 {remaining:.1f} 秒"
                draw_text_2d(surface, state,
                             (bar_x, ty + 27), font_sm, text_secondary)
                if pose["kind"] == "hinge":
                    quality = self._hinge_live_result
                    if quality is None or quality.sample_count < 10:
                        quality_text = "正在识别肘轴，请继续匀速屈伸…"
                        quality_color = text_secondary
                    else:
                        quality_text = (
                            f"范围 {quality.motion_range_deg:.0f}° / ≥{CALIB_HINGE_MIN_RANGE_DEG:.0f}°   "
                            f"轴离散 {quality.axis_dispersion_deg:.1f}° / ≤{CALIB_MAX_HINGE_AXIS_DISPERSION_DEG:.0f}°   "
                            f"大臂移动 {quality.upper_motion_deg:.1f}° / ≤{CALIB_MAX_HINGE_UPPER_MOTION_DEG:.0f}°")
                        quality_color = green if quality.valid else amber
                    draw_text_2d(surface, quality_text,
                                 (bar_x, ty + 51), font_sm, quality_color)
                elif pose["kind"] == "palm_down":
                    # 与第 5 步的一致性只在第 7 步之后才正式判定，届时
                    # 不合格会作废整套 7 步；这里提前把它显示出来。
                    consistency = self._palm_live_consistency_deg
                    safe_limit = (CALIB_MAX_PALM_CONSISTENCY_DEG
                                  - CALIB_PALM_PREVIEW_MARGIN_DEG)
                    if consistency is None:
                        quality_text = "正在比对与上一步掌心朝上的一致性…"
                        quality_color = text_secondary
                    else:
                        if consistency > CALIB_MAX_PALM_CONSISTENCY_DEG:
                            verdict = "超限，保存时会被拒绝"
                            quality_color = red
                        elif consistency > safe_limit:
                            verdict = "接近上限，建议重做本步"
                            quality_color = amber
                        else:
                            verdict = "合格"
                            quality_color = green
                        quality_text = (
                            f"翻掌一致性 {consistency:.1f}° / "
                            f"≤{CALIB_MAX_PALM_CONSISTENCY_DEG:.0f}°   "
                            f"{verdict}")
                    draw_text_2d(surface, quality_text,
                                 (bar_x, ty + 51), font_sm, quality_color)
                    if consistency is not None and consistency > safe_limit:
                        draw_text_2d(
                            surface,
                            "手臂保持前平举不动，掌心真正翻到朝下",
                            (bar_x, ty + 73), font_sm, text_secondary)

        # 叠加到 OpenGL
        tex_data = pygame.image.tostring(surface, "RGBA", True)
        w, h = surface.get_size()
        glViewport(0, 0, drawable_width, drawable_height)
        glMatrixMode(GL_PROJECTION); glLoadIdentity()
        glOrtho(0, width, 0, height, -1, 1)
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
        screen = self._set_display_mode(self._fullscreen)
        pygame.display.set_caption("Unitree G1 Motion Studio")

        if self.hand_enabled:
            # 手套数据源起不来只关掉手，不拖垮整个上位机 —— 手臂那条链
            # 和它没有任何依赖关系。一侧起不来也只关掉那一侧。
            for side, cfg in self._hand_cfgs.items():
                if cfg is None:
                    continue
                try:
                    self.gloves[side] = GloveSource(
                        cfg_path=cfg, port=self._hand_ports[side], side=side,
                        thumb_retarget=self._thumb_retarget)
                    self._log(f"{side} 手数据源已监听 "
                              f"127.0.0.1:{self.gloves[side].port}"
                              f"（等 mhandpro_diagnostic 的 teleop 连入）")
                    if self.gloves[side].thumb_error is not None:
                        self._log(f"{side} 手拇指指腹重定向未启用, 拇指走线性"
                                  f"投影: {self.gloves[side].thumb_error}")
                except OSError as exc:
                    self._log(f"{side} 手数据源启动失败, 已关闭该侧: {exc}")
            if all(g is None for g in self.gloves.values()):
                self.hand_enabled = False
                self._log("两侧手部数据源都没起来, 已关闭手部")

        if self.hand_enabled and self._real_hand_side is not None:
            try:
                if self._real_hand_transport == "dds":
                    from inspire_hand_ctrl import InspireHandPublisher

                    self.real_hand = InspireHandPublisher(
                        side=self._real_hand_side)
                    self._log(f"真手输出已开启 (DDS): {self.real_hand.topic}")
                    self._log("需要同网段已启动 inspire_hand_sdk 的 Headless "
                              "驱动，否则话题没人消费、手不会动")
                else:
                    from inspire_hand_ctrl import InspireHandModbus

                    self.real_hand = InspireHandModbus(
                        side=self._real_hand_side, ip=self._real_hand_ip)
                    self._log(f"真手输出已开启 (Modbus): "
                              f"{self.real_hand.ip}:{self.real_hand.port}")
                self._log(
                    f"手部行程上限 {hand_mapping.REAL_HAND_MAX_RANGE_SCALE:.0%}"
                    f"，每帧最多变化 {inspire_hand_ctrl.DEFAULT_MAX_STEP} counts")
            except Exception as exc:
                # 缺 SDK、DDS 起不来、网络不通 —— 都只关真手, 不影响仿真。
                self.real_hand = None
                self._log(f"真手输出启动失败, 已关闭 (仿真不受影响): {exc}")

        font = pygame.font.SysFont("notosanscjksc,notosanscjk,wenquanyimicrohei,simhei,monospace", 18, bold=True)
        font_sm = pygame.font.SysFont("notosanscjksc,notosanscjk,wenquanyimicrohei,simhei,monospace", 14)
        font_lg = pygame.font.SysFont("notosanscjksc,notosanscjk,wenquanyimicrohei,simhei,monospace", 28, bold=True)

        try:
            if not self._g1_full_body:
                urdf_path = G1_ARMS_URDF_PATH
            elif self.hand_enabled:
                urdf_path = G1_HAND_URDF_PATH
            else:
                urdf_path = G1_URDF_PATH
            self.g1_renderer = G1UrdfRenderer(urdf_path)
            self.g1_renderer.load_meshes()
            self.g1_renderer_ready = True
            self._log(
                f"G1 {'全身29自由度' if self._g1_full_body else '上半身'}模型已加载: "
                f"{len(self.g1_renderer.model.visuals)}个部件 / "
                f"{self.g1_renderer.triangle_count:,}个三角面")
        except Exception as exc:
            self.g1_renderer_ready = False
            self.g1_renderer_error = str(exc)
            self._log(f"G1 URDF 加载失败，使用简化模型: {exc}")

        try:
            self.human_renderer = HumanGltfRenderer(HUMAN_GLB_PATH)
            self.human_renderer_ready = True
            self._log(
                f"骨骼人体已加载: "
                f"{self.human_renderer.model.triangle_count:,}个三角面 / "
                f"{len(self.human_renderer.model.skin_joints)}个骨骼")
        except Exception as exc:
            self.human_renderer_ready = False
            self.human_renderer_error = str(exc)
            self._log(f"骨骼人体加载失败，使用简化人体: {exc}")

        # 初始化
        if self.right_forearm_only:
            self._log("右小臂单IMU诊断模式: 只需打开一个IMU")
            self._log("保持小臂零位姿态按 C，然后按 SPACE 跟随")
        elif self.right_arm_two_imu:
            self._log("右臂双IMU诊断模式: 右大臂 + 右小臂，不需胸部IMU")
            self._log("保持右臂零位姿态按 C，然后按 SPACE 跟随")
        else:
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
                    elif event.type == pygame.VIDEORESIZE and not self._fullscreen:
                        # pygame 2 会自动调整 RESIZABLE display surface；这里
                        # 不再 set_mode 重建 OpenGL 上下文，只记录窗口尺寸。
                        self._windowed_size = (
                            max(MIN_WINDOW_WIDTH, int(event.w)),
                            max(MIN_WINDOW_HEIGHT, int(event.h)),
                        )
                        self._sync_display_size(screen)
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_q:
                            running = False
                        elif event.key == pygame.K_ESCAPE:
                            if self._fullscreen:
                                screen = self._set_display_mode(False)
                                self._log("已退出全屏")
                            else:
                                running = False
                        elif (event.key == pygame.K_F11
                              or (event.key == pygame.K_RETURN
                                  and event.mod & pygame.KMOD_ALT)):
                            screen = self._set_display_mode(
                                not self._fullscreen)
                            self._log(
                                "已进入全屏" if self._fullscreen
                                else "已退出全屏")
                        elif event.key == pygame.K_r:
                            self.active_arm = "right"
                            self._log("切换到右臂")
                        elif event.key == pygame.K_l:
                            if self.right_diagnostic_mode:
                                self._log("右臂诊断模式不启用左臂")
                            else:
                                self.active_arm = "left"
                                self._log("切换到左臂")
                        elif event.key == pygame.K_SPACE:
                            if self._calib_step >= 0:
                                self._log("标定过程中不能启动跟随，请先完成或按 X 取消")
                                continue
                            single_not_ready = (
                                self.right_forearm_only
                                and not self.single_forearm_mapper.calibrated)
                            two_not_ready = (
                                self.right_arm_two_imu
                                and not self.two_imu_mapper.calibrated)
                            if single_not_ready or two_not_ready:
                                self._log("请先保持零位姿态并按 C 采集零位")
                            else:
                                target = self._udp_target or (
                                    (self.robot_ip, self.robot_port)
                                    if self.robot_discovered else None)
                                enabling = not self.active.following
                                if (enabling and target
                                        and not self._quality_allows_target(
                                            self.active, target)):
                                    self._log(
                                        "当前标定仅供仿真使用；请将目标设为"
                                        "127.0.0.1，或重新标定到≤20°后连接真机")
                                else:
                                    self.active.following = enabling
                                    state = ("跟随" if self.active.following
                                             else "暂停")
                                    self._log(f"{SIDE_CN[self.active.side]}臂: {state}")
                        elif event.key in (pygame.K_LEFTBRACKET,
                                           pygame.K_RIGHTBRACKET):
                            step = (0.1 if event.key == pygame.K_RIGHTBRACKET
                                    else -0.1)
                            with self.lock:
                                gain = self.waist.adjust_gain(step)
                            self._log(f"腰部增益: {gain:.1f}x")
                        elif event.key == pygame.K_w:
                            with self.lock:
                                if event.mod & pygame.KMOD_SHIFT:
                                    self.waist_enabled = not self.waist_enabled
                                    self._log(
                                        "腰部跟随: "
                                        + ("开启" if self.waist_enabled
                                           else "关闭"))
                                else:
                                    self.capture_waist_zero()
                        elif event.key == pygame.K_b:
                            if self._calib_step >= 0:
                                self._log("标定过程中不能启动跟随，请先完成或按 X 取消")
                                continue
                            with self.lock:
                                self.toggle_dual_follow()
                        elif event.key == pygame.K_c:
                            if self._calib_step < 0:
                                # Shift+C 只标定当前臂，C 走双臂 14 步。
                                self.start_calibration(
                                    chain=not (event.mod & pygame.KMOD_SHIFT))
                            elif self._calib_waiting:
                                self._start_calib_collection()
                        elif event.key == pygame.K_s:
                            self._log("新版手掌朝上/朝下校准为必做步骤，请按 C 采集")
                        elif event.key == pygame.K_x:
                            with self.lock:
                                self.right.reset()
                                self.left.reset()
                                self.waist.reset()
                                self.single_forearm_mapper.reset()
                                self.two_imu_mapper.reset()
                                self._calib_step = -1
                                self._calib_waiting = False
                                self._calib_chain = False
                                self._calib_arm = None
                                self._pose_averages = []
                                self._palm_pose_results = {}
                                self._hinge_result = None
                                self._validation_average = None
                                self._calib_previous = None
                                self._calib_stability_window = []
                                self._calib_stable_since = None
                            if self.right_forearm_only:
                                self._log("右小臂单IMU零位已重置")
                            elif self.right_arm_two_imu:
                                self._log("右臂双IMU零位已重置")
                            else:
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
            if self.real_hand is not None:
                # 退出前把手张开, 别让它攥着东西停在那儿。
                try:
                    self.real_hand.open_and_close()
                    self._log("真手已张开")
                except Exception as exc:
                    self._log(f"真手张开失败: {exc}")
            for glove in self.gloves.values():
                if glove is not None:
                    glove.close()
            if self.recorder.recording:
                path = self.recorder.save()
                self._log(f"录制已保存: {path}")
            if self.session_logger.enabled:
                self._log(
                    f"自动采集日志已保存: "
                    f"{self.session_logger.session_dir}")
            self.session_logger.close()
            pygame.quit()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="双臂解算3D可视化 + 机器人遥操发送")
    parser.add_argument("--udp-target", default=None,
                        help="直接指定机器人地址 (如 192.168.1.100:9527), 省略则自动发现")
    parser.add_argument("--fullscreen", action="store_true",
                        help="以桌面原生分辨率全屏启动；运行中可按 F11 切换")
    parser.add_argument(
        "--no-data-log", action="store_true",
        help="关闭默认启用的原始 IMU 与 PC 解算 CSV 自动日志")
    parser.add_argument(
        "--data-log-dir", default=None,
        help="指定自动采集日志根目录，默认保存到 PC端/采集日志")
    parser.add_argument(
        "--response-mode",
        choices=("original", "balanced", "fast"),
        default="original",
        help="姿态响应模式：original原始平滑（默认）、balanced平衡、fast快速")
    diagnostic_group = parser.add_mutually_exclusive_group()
    diagnostic_group.add_argument(
        "--right-forearm-only", action="store_true",
        help="单右小臂IMU诊断：肩关节固定，C键采集零位")
    diagnostic_group.add_argument(
        "--right-arm-two-imu", action="store_true",
        help="右大臂+右小臂双IMU诊断：不需胸部IMU，C键采集零位")
    parser.add_argument("--forearm-elbow-sign", type=float, choices=(-1.0, 1.0),
                        default=1.0, help="屈肘方向，反向时设为 -1")
    parser.add_argument("--forearm-wrist-sign", type=float, choices=(-1.0, 1.0),
                        default=1.0, help="腕roll方向，反向时设为 -1")
    parser.add_argument("--upper-pitch-sign", type=float, choices=(-1.0, 1.0),
                        default=1.0, help="双IMU肩pitch方向，反向时设为 -1")
    parser.add_argument("--upper-roll-sign", type=float, choices=(-1.0, 1.0),
                        default=1.0, help="双IMU肩roll方向，反向时设为 -1")
    parser.add_argument("--upper-yaw-sign", type=float, choices=(-1.0, 1.0),
                        default=1.0, help="双IMU肩yaw方向，反向时设为 -1")
    parser.add_argument("--waist", action="store_true",
                        help="用胸/腰部 IMU 驱动 waist_yaw（仅 HumDex 全身仿真消费）")
    parser.add_argument("--waist-yaw-sign", type=float, choices=(-1.0, 1.0),
                        default=1.0, help="转腰方向，反向时设为 -1")
    parser.add_argument("--waist-max-deg", type=float,
                        default=math.degrees(WAIST_DEFAULT_MAX_YAW_RAD),
                        help="腰部偏航限幅（度），默认按人体活动范围收紧")
    parser.add_argument("--waist-max-speed", type=float,
                        default=WAIST_DEFAULT_MAX_SPEED,
                        help="腰部最大角速度 rad/s")
    parser.add_argument("--waist-gain", type=float,
                        default=WAIST_DEFAULT_GAIN,
                        help="腰部放大系数；实测策略只跟到约 0.7，"
                             "手感偏迟钝时调大，运行中也可用 [ ] 调")
    parser.add_argument("--robot-model", choices=("full", "arms"),
                        default="full",
                        help="G1 显示模型：full=全身29自由度（默认），"
                             "arms=旧的上半身模型")
    parser.add_argument("--hand", nargs="?", const=DEFAULT_HAND_CFG,
                        default=None, metavar="CFG",
                        help="启用右手因时灵巧手：换成带手的模型，并监听 "
                             "mhandpro_diagnostic 的 teleop 流。可给 cfg 路径，"
                             f"默认 {os.path.relpath(DEFAULT_HAND_CFG, _MY_DIR)}")
    parser.add_argument("--hand-port", type=int, default=None,
                        help="右手数据源监听端口，默认取 cfg 里的 port")
    parser.add_argument("--hand-left", nargs="?", const=DEFAULT_HAND_LEFT_CFG,
                        default=None, metavar="CFG",
                        help="同上，但启用左手。左右手各监听一个端口，上游用 "
                             "mhandpro_diagnostic 的 `teleop both` 一个进程同时"
                             "驱动两只手套。可给 cfg 路径，默认 "
                             f"{os.path.relpath(DEFAULT_HAND_LEFT_CFG, _MY_DIR)}")
    parser.add_argument("--hand-left-port", type=int, default=None,
                        help="左手数据源监听端口，默认取 cfg 里的 port")
    parser.add_argument("--no-thumb-retarget", action="store_true",
                        help="关掉拇指指腹位置重定向，拇指退回线性投影。"
                             "用来 A/B 对比拇指手感")
    parser.add_argument("--real-hand", choices=("right", "left"), default=None,
                        help="驱动真机上的因时灵巧手。需要 --hand（手部数据"
                             "来自手套）")
    parser.add_argument("--real-hand-transport", choices=("modbus", "dds"),
                        default="modbus",
                        help="modbus=直连灵巧手（默认，只需 pymodbus）；"
                             "dds=发到 rt/inspire_hand/ctrl/{r,l}，需要 "
                             "cyclonedds 且同网段已起官方驱动进程")
    parser.add_argument("--real-hand-ip", default=None,
                        help="灵巧手 IP，默认按左右手取 "
                             "192.168.123.210 / .211")
    args = parser.parse_args()
    if args.real_hand == "right" and args.hand is None:
        parser.error("--real-hand right 需要同时给 --hand（手部数据来自手套）")
    if args.real_hand == "left" and args.hand_left is None:
        parser.error("--real-hand left 需要同时给 --hand-left（手部数据来自手套）")
    for flag, value in (("--hand", args.hand), ("--hand-left", args.hand_left)):
        if value is None:
            continue
        if not os.path.isfile(value):
            parser.error(f"{flag} 配置不存在: {value}")
        if args.robot_model != "full":
            parser.error(f"{flag} 需要全身模型，不能和 --robot-model arms 一起用")
    if not 0.0 < args.waist_max_deg <= 150.0:
        parser.error("--waist-max-deg 必须在 (0, 150] 之间")
    if args.waist_max_speed <= 0.0:
        parser.error("--waist-max-speed 必须大于 0")
    app = DualArmViz(
        udp_target=args.udp_target,
        right_forearm_only=args.right_forearm_only,
        right_arm_two_imu=args.right_arm_two_imu,
        forearm_elbow_sign=args.forearm_elbow_sign,
        forearm_wrist_sign=args.forearm_wrist_sign,
        upper_pitch_sign=args.upper_pitch_sign,
        upper_roll_sign=args.upper_roll_sign,
        upper_yaw_sign=args.upper_yaw_sign,
        fullscreen=args.fullscreen,
        data_log=not args.no_data_log,
        data_log_dir=args.data_log_dir,
        response_mode=args.response_mode,
        waist_enabled=args.waist,
        waist_yaw_sign=args.waist_yaw_sign,
        waist_max_yaw_rad=math.radians(args.waist_max_deg),
        waist_max_speed=args.waist_max_speed,
        waist_gain=args.waist_gain,
        robot_model=args.robot_model,
        hand_cfg=args.hand,
        hand_port=args.hand_port,
        real_hand=args.real_hand,
        real_hand_transport=args.real_hand_transport,
        real_hand_ip=args.real_hand_ip,
        hand_left_cfg=args.hand_left,
        hand_left_port=args.hand_left_port,
        thumb_retarget=not args.no_thumb_retarget,
    )
    if args.waist:
        print(f"腰部跟随已启用: 增益 {app.waist.gain:.1f}x, "
              f"限幅 ±{args.waist_max_deg:.0f}°, "
              f"限速 {args.waist_max_speed:.1f} rad/s "
              f"(消费端需同样开启腰部: 真机 --waist / HumDex 全身仿真; "
              f"运行中按 [ ] 调增益)")
    if args.right_forearm_only:
        print("右小臂单IMU诊断模式: 肩关节固定为零")
    elif args.right_arm_two_imu:
        print("右臂双IMU诊断模式: 右大臂 + 右小臂，无需胸部IMU")
    if args.udp_target:
        print(f"UDP 直连模式 → {args.udp_target}")
    else:
        print("UDP 自动发现模式 (监听机器人广播 G1RC...)")
    app.run()
