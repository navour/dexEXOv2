#!/usr/bin/env python3
"""
宇树G1 双臂遥操控接收端 (带局域网自动发现)

功能:
  - UDP 广播自身存在, 供树莓派自动发现
  - UDP 接收PC/树莓派发来的电机角度指令
  - 两种真机控制接口 (--mode):
      armsdk (默认) 发布 rt/arm_sdk, 只接管腰+双臂(12~28),
                    双腿与平衡仍由官方运控服务负责 → 机器人可站立
      lowcmd        发布 rt/lowcmd, 接管全部29个电机并把下半身拉到零位,
                    没有任何平衡控制 → 机器人必须吊装悬空
  - 安全保护: mode=0 时缓慢回零, 指令超时1秒回零, 关节限幅
  - 启动渐变: 缓慢从当前位置接管, 避免突跳 (armsdk 另有权重0→1渐入)
  - 退出交还: armsdk 模式退出时权重1→0, 把手臂交还给运控服务
  - 轨迹记录: 记录预期与实际轨迹差值至CSV文件, 便于调试

通信协议:
  新指令包: 'UA2M' + mode + seq + timestamp + 10路q + 10路dq
  旧 UARM 单臂/双臂包继续兼容
  发现广播: 'G1RC,ip=<IP>,port=<PORT>'  (每2秒)

使用:
  python3 robot_arm_receiver.py [端口号] [--mode armsdk|lowcmd] [--iface eth0]
  默认端口: 9527, 默认模式: armsdk
  在机器人板载计算机上运行可省略 --iface;
  在外部PC/树莓派上运行必须指定接入机器人网段的网卡名。
"""

import argparse
import time
import sys
import os
import signal
import threading
import socket
import math

import json
import csv
from datetime import datetime

# 支持从当前目录查找 unitree_sdk2py
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if os.path.isdir(os.path.join(_SCRIPT_DIR, 'unitree_sdk2py')):
    sys.path.insert(0, _SCRIPT_DIR)

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from teleop_protocol import unpack_arm_command
from waist_follow import (
    WAIST_FOLLOW_SPEED, WAIST_JOINTS, WAIST_YAW_JOINT, WAIST_YAW_LIMITS,
    waist_yaw_command)

# ======================== 常量 ========================
G1_NUM_MOTOR = 29

RIGHT_ARM_JOINTS = {
    "right_shoulder_pitch": 22,
    "right_shoulder_roll":  23,
    "right_shoulder_yaw":   24,
    "right_elbow":          25,
    "right_wrist_roll":     26,
}

RIGHT_ARM_JOINT_LIMITS = {
    "right_shoulder_pitch": (-3.0892, 2.6704),
    "right_shoulder_roll":  (-2.2515, 1.5882),
    "right_shoulder_yaw":   (-2.618, 2.618),
    "right_elbow":          (-1.0472, 2.0944),
    "right_wrist_roll":     (-1.9722, 1.9722),
}

LEFT_ARM_JOINTS = {
    "left_shoulder_pitch": 15,
    "left_shoulder_roll":  16,
    "left_shoulder_yaw":   17,
    "left_elbow":          18,
    "left_wrist_roll":     19,
}

LEFT_ARM_JOINT_LIMITS = {
    "left_shoulder_pitch": (-3.0892, 2.6704),
    "left_shoulder_roll":  (-1.5882, 2.2515),
    "left_shoulder_yaw":   (-2.618, 2.618),
    "left_elbow":          (-1.0472, 2.0944),
    "left_wrist_roll":     (-1.9722, 1.9722),
}

WRIST_ZERO_OFFSETS = {
    20: 0.0,   # LeftWristPitch
    21: 0.0,   # LeftWristYaw
    27: 0.0,   # RightWristPitch
    28: 0.0,   # RightWristYaw
}

Kp_full = [
    60, 60, 60, 100, 40, 40,      # 左腿 (0-5)
    60, 60, 60, 100, 40, 40,      # 右腿 (6-11)
    100, 100, 100,                 # 腰 (12-14)
    120, 80, 180, 60, 25, 25, 50,      # 左臂 (v4: 关闭Kv_ff, 适中Kp, 高Kd阻尼, Kd/Kp≥0.04)
    80, 160, 160, 80, 30, 25, 40,      # 右臂 (v4: 关闭Kv_ff, 适中Kp, 高Kd阻尼, Kd/Kp≥0.04)
]

Kd_full = [
    1, 1, 1, 2, 1, 1,
    1, 1, 1, 2, 1, 1,
    2, 2, 2,
    5.0, 3.5, 10.0, 2.5, 1.0, 1.0, 2.0,   # 左臂 (Kd/Kp ≈ 0.04-0.06)
    4.0, 8.0, 8.0, 3.0, 1.5, 1.5, 1.0,    # 右臂 (Kd/Kp ≈ 0.04-0.05)
]

# 速度前馈: v4 全部关闭 (仿真优化值在实机造成严重振荡, delta_q过大导致超调回弹)
Kv_ff_full = [0.0] * G1_NUM_MOTOR

# ---- 控制接口 ----
# armsdk: 官方 rt/arm_sdk 接口, 只接管腰+双臂, 腿和平衡交给运控服务
# lowcmd: 官方 rt/lowcmd 接口, 接管全部29个电机, 无平衡 → 必须吊装
MODE_ARM_SDK = "armsdk"
MODE_LOWCMD = "lowcmd"

TOPIC_ARM_SDK = "rt/arm_sdk"
TOPIC_LOWCMD = "rt/lowcmd"

# rt/arm_sdk 用 motor_cmd[29].q 作为 0~1 的接管权重 (官方 kNotUsedJoint)
ARM_SDK_WEIGHT_JOINT = 29
# armsdk 模式下一并接管的腰部关节 (与官方 g1_arm7_sdk_dds_example 一致);
# 若不下发, 这些关节会以 kp=kd=0 被服务采纳而变软
ARM_SDK_WAIST_JOINTS = WAIST_JOINTS
ARM_SDK_WAIST_KP = 60.0
ARM_SDK_WAIST_KD = 1.5
# 退出时权重 1→0 的渐出时间 (秒), 把手臂交还给运控服务
ARM_SDK_RELEASE_DURATION = 2.0

# 控制参数
CONTROL_DT = 0.002   # 2ms (500Hz)
DEFAULT_PORT = 9527

# 回零速度 (rad/s)
RETURN_SPEED = 0.3

# 跟随平滑
MAX_FOLLOW_SPEED = 5.0  # rad/s (3.5→5.0, 左臂更重需要更快响应)
TARGET_PREDICTION_MAX_SEC = 0.020

# 速度前馈: 已改用 per-joint Kv_ff_full + 位置偏移方式实现 (见控制循环)
# 电机控制: tau = kp*(q_cmd-q) - kd*dq, 其中 q_cmd = q_target + Kv_ff*dq_target/Kp
VELOCITY_FF_GAIN = 0.0  # 不再使用全局增益, 保留字段兼容

# 启动阶段
STARTUP_RAMP_DURATION = 3.0
STARTUP_KP_RATIO = 0.1
STARTUP_MOVE_SPEED = 0.2

# 轨迹记录
LOG_JOINTS = [15, 16, 17, 18, 19, 22, 23, 24, 25, 26]   # 左臂+右臂关节(含腕roll)
LOG_DOWNSAMPLE = 10              # 每10次控制循环记录一次 (500/10=50Hz)

# 超时
TIMEOUT_SEC = 1.0

# 自动发现广播
DISCOVERY_BROADCAST_PORT = 9528
DISCOVERY_INTERVAL = 2.0   # 每2秒广播一次


def get_local_ip():
    """获取本机局域网 IP"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "0.0.0.0"


def hand_driver_range_scale():
    """真手行程上限, 只为启动横幅显示。拿不到就返回 0 而不是抛异常 ——
    横幅少一个数字不该让接收端起不来。"""
    try:
        import hand_mapping
        return hand_mapping.REAL_HAND_MAX_RANGE_SCALE
    except ImportError:
        return 0.0


def clamp_angle(val, limits):
    return max(limits[0], min(limits[1], val))


# ======================== 运控服务查询 ========================
#
# rt/lowcmd 与官方运控服务 (sport_mode / ai_sport / loco) 会同时写同一批电机,
# 必须先 ReleaseMode 停掉运控服务; 而 rt/arm_sdk 恰恰相反, 必须让运控服务
# 保持运行才有平衡。这里只做查询和提示, 不自动停服务 —— 停服务会让机器人
# 瞬间失去平衡, 必须由操作员在确认已吊装后手动确认。

# MotionSwitcherClient 在各版本 unitree_sdk2py 里的位置不一致 (实测机载机上
# 没有 b2.motion_switcher)。逐个试, 全都找不到才放弃 —— 之前只认一个路径,
# 导入失败被静默吞掉, 结果运控服务检查在这台机器上一直等于没做。
_MOTION_SWITCHER_PATHS = (
    "unitree_sdk2py.b2.motion_switcher.motion_switcher_client",
    "unitree_sdk2py.go2.motion_switcher.motion_switcher_client",
    "unitree_sdk2py.g1.motion_switcher.motion_switcher_client",
    "unitree_sdk2py.h1.motion_switcher.motion_switcher_client",
    "unitree_sdk2py.motion_switcher.motion_switcher_client",
)


def _import_motion_switcher():
    """返回 MotionSwitcherClient 类; 都找不到返回 None。"""
    for path in _MOTION_SWITCHER_PATHS:
        try:
            module = __import__(path, fromlist=["MotionSwitcherClient"])
            return getattr(module, "MotionSwitcherClient")
        except Exception:
            continue
    return None


def _make_motion_switcher():
    """构造 MotionSwitcherClient; SDK 不支持时返回 None。"""
    MotionSwitcherClient = _import_motion_switcher()
    if MotionSwitcherClient is None:
        return None
    try:
        msc = MotionSwitcherClient()
        msc.SetTimeout(5.0)
        msc.Init()
        return msc
    except Exception:
        return None


def query_motion_service(msc):
    """查询运控服务状态。

    Returns:
        (True, name)   运控服务正在运行 (name 可能为空串)
        (False, None)  运控服务已停止
        (None, None)   查询失败 / SDK 不支持, 状态未知
    """
    if msc is None:
        return None, None
    try:
        code, result = msc.CheckMode()
    except Exception:
        return None, None
    if code != 0:
        return None, None
    name = ""
    if isinstance(result, dict):
        name = result.get("name") or ""
    if name:
        return True, name
    return False, None


def release_motion_service(msc, retries=3):
    """停止运控服务 (仅 lowcmd 模式使用)。成功返回 True。"""
    if msc is None:
        return False
    for _ in range(retries):
        active, name = query_motion_service(msc)
        if active is False:
            return True
        if active is None:
            return False
        print(f"[*] 正在停止运控服务 ({name}) ...")
        try:
            ret = msc.ReleaseMode()
        except Exception as exc:
            print(f"[!] ReleaseMode 异常: {exc}")
            return False
        if ret != 0:
            print(f"[!] ReleaseMode 失败, 错误码 {ret}")
        time.sleep(5.0)
    return query_motion_service(msc)[0] is False


# ======================== 轨迹记录器 ========================

# 关节名称映射 (电机 ID → 可读名称)
_JOINT_NAMES = {
    15: 'l_sp', 16: 'l_sr', 17: 'l_sy', 18: 'l_el', 19: 'l_wr',
    22: 'r_sp', 23: 'r_sr', 24: 'r_sy', 25: 'r_el', 26: 'r_wr',
}


class TrajectoryLogger:
    """
    增强版轨迹记录器。

    记录每个关节的:
      - cmd  (目标角度, 来自树莓派 UDP 指令)
      - act  (实际角度, 来自电机编码器反馈)
      - err  (cmd - act, 跟踪误差)
      - vel  (电机速度, rad/s)
      - tau  (电机力矩, N·m, 仅记录)

    附加诊断列:
      - udp_gap_ms  (UDP 接收间隔)
      - 各关节 cmd 跳变量
      - mode (当前模式)
      - event (事件标记)
    """

    def __init__(self, joints, downsample=10, log_dir=None):
        self.joints = joints
        self.downsample = downsample
        self._counter = 0
        self._file = None
        self._writer = None
        self._t0 = None
        self._log_dir = log_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'log')
        self._rows_written = 0
        self._prev_cmd = None
        self._last_recv_t = None
        self._udp_gap_ms = 0.0
        self._events = []
        self._lock = threading.Lock()

    def start(self):
        os.makedirs(self._log_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self._log_dir, f"traj_log_{ts}.csv")
        self._file = open(path, 'w', newline='')

        header = ['time_s', 'mode']
        for j in self.joints:
            name = _JOINT_NAMES.get(j, f'j{j}')
            header += [f'{name}_cmd', f'{name}_act', f'{name}_err',
                       f'{name}_vel', f'{name}_tau']
        header += ['udp_gap_ms']
        for j in self.joints:
            name = _JOINT_NAMES.get(j, f'j{j}')
            header.append(f'{name}_jump')
        header.append('event')

        self._writer = csv.writer(self._file)
        self._writer.writerow(header)
        self._t0 = time.monotonic()
        self._rows_written = 0
        print(f"[+] 轨迹记录已启动: {path}  ({len(self.joints)} 关节, "
              f"{500 // self.downsample}Hz)")
        return path

    def log(self, mode, cmd_dict, actual_dict, motor_state):
        """500Hz 控制循环调用, 内部降采样.

        Args:
            mode: 当前控制模式
            cmd_dict: {motor_id: target_angle}
            actual_dict: {motor_id: actual_angle}
            motor_state: low_state.motor_state 列表
        """
        if self._writer is None:
            return
        self._counter += 1
        if self._counter % self.downsample != 0:
            return

        t = time.monotonic() - self._t0
        row = [f'{t:.4f}', mode]

        for j in self.joints:
            c = cmd_dict.get(j, 0.0)
            a = actual_dict.get(j, 0.0)
            ms = motor_state[j]
            row += [f'{c:.5f}', f'{a:.5f}', f'{c - a:.5f}',
                    f'{ms.dq:.5f}', f'{ms.tau_est:.5f}']

        with self._lock:
            row.append(f'{self._udp_gap_ms:.1f}')
            if self._prev_cmd is not None:
                for j in self.joints:
                    cur = cmd_dict.get(j, 0.0)
                    prev = self._prev_cmd.get(j, 0.0)
                    row.append(f'{cur - prev:.5f}')
            else:
                row += ['0.0'] * len(self.joints)
            self._prev_cmd = {j: cmd_dict.get(j, 0.0) for j in self.joints}

            event_str = ';'.join(self._events) if self._events else ''
            self._events.clear()
        row.append(event_str)

        self._writer.writerow(row)
        self._rows_written += 1
        if self._rows_written % 200 == 0:
            self._file.flush()

    def record_recv(self, r_sp, r_sr, r_sy, r_el, r_wr,
                    l_sp, l_sr, l_sy, l_el, l_wr):
        """每收到一个 UDP 包时调用, 记录接收间隔."""
        now = time.monotonic()
        with self._lock:
            if self._last_recv_t is not None:
                self._udp_gap_ms = (now - self._last_recv_t) * 1000.0
            self._last_recv_t = now
            if self._udp_gap_ms > 100:
                self._events.append(f'UDP_GAP:{self._udp_gap_ms:.0f}ms')

    def add_event(self, event_str):
        with self._lock:
            self._events.append(event_str)

    def stop(self):
        if self._file:
            self._file.flush()
            self._file.close()
            print(f"[+] 轨迹记录已保存 ({self._rows_written} 条记录) -> "
                  f"{self._log_dir}")
            self._file = None
            self._writer = None

    @property
    def rows(self):
        return self._rows_written


# ======================== 主控制器 ========================

class RobotArmReceiver:
    def __init__(self, port, control_mode=MODE_ARM_SDK, iface=None,
                 allow_waist=False, hand_sides=(), hand_only=False,
                 hand_range=None, hand_haptic=False,
                 hand_haptic_host="0.0.0.0", hand_touch_hz=5.0):
        self.port = port
        self.control_mode = control_mode
        self.iface = iface
        # 硬门控: 为假时无论发送端是否附带 UAWS 尾块, 腰都保持回中。
        # 默认关闭 —— 扭腰会改变上半身重心, 对运控服务是真实扰动。
        self.allow_waist = bool(allow_waist)
        # 要驱动的灵巧手。默认空 —— 手指会夹人, 必须显式 --hand 打开。
        # 与腰不同的是, 手**不**要求手臂进入跟随: 手指不影响平衡, 而且
        # 跟着 mode 走的话不接 IMU 就永远测不了手。
        self.hand_sides = tuple(hand_sides)
        self.hand_followers = {}
        # 只跑手: 完全不碰 DDS/电机, 用来在运控没起来时验证手的整条链。
        self.hand_only = bool(hand_only)
        # 手部行程比例。None = 用 hand_mapping 的上限 (整程)。
        self.hand_range = hand_range
        self.hand_haptic = bool(hand_haptic)
        self.hand_haptic_host = hand_haptic_host
        self.hand_touch_hz = hand_touch_hz
        self.lock = threading.Lock()

        # UDP 数据接收
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', port))
        self.sock.settimeout(0.05)

        # 发现广播
        self._disc_sock = None
        self._local_ip = get_local_ip()

        # 接收状态 — 右臂
        self._mode = 0
        self._target_r_sp = 0.0
        self._target_r_sr = 0.0
        self._target_r_sy = 0.0
        self._target_r_el = 0.0
        self._target_r_wr = 0.0
        # 接收状态 — 左臂
        self._target_l_sp = 0.0
        self._target_l_sr = 0.0
        self._target_l_sy = 0.0
        self._target_l_el = 0.0
        self._target_l_wr = 0.0
        self._target_velocity = (0.0,) * 10
        # 腰部 yaw 目标; 发送端没带尾块或尾块无效时为 None, 此时腰回中。
        self._target_waist_yaw = None
        self._protocol_version = 1
        self._last_seq = None
        self._last_sender_timestamp_us = None
        self._packet_arrival = 0.0
        self._out_of_order_count = 0
        self._clock_offset_min_us = None
        self._transport_excess_samples_ms = []
        self._last_recv = 0.0
        self._sender_addr = None

        # 当前角度 — 右臂
        self._cmd_r_sp = None
        self._cmd_r_sr = None
        self._cmd_r_sy = None
        self._cmd_r_el = None
        self._cmd_r_wr = None
        # 当前角度 — 左臂
        self._cmd_l_sp = None
        self._cmd_l_sr = None
        self._cmd_l_sy = None
        self._cmd_l_el = None
        self._cmd_l_wr = None
        self._cmd_wrist = {wi: None for wi in WRIST_ZERO_OFFSETS}
        # lowcmd 模式接管腿+腰(0~14); armsdk 模式只接管腰(12~14)
        self._lower_joints = (tuple(range(15)) if control_mode == MODE_LOWCMD
                              else ARM_SDK_WAIST_JOINTS)
        self._cmd_lower = {i: None for i in self._lower_joints}
        self._waist_following = False

        # 启动阶段
        self._startup = True
        self._startup_t = 0.0

        # armsdk 接管权重 (0=完全交给运控服务, 1=完全由本程序接管)
        self._arm_sdk_weight = 0.0
        self._releasing = False
        self._released = False

        # 统计
        self._recv_count = 0
        self._recv_rate = 0.0
        self._recv_timer = time.monotonic()

        # 机器人
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.state_ready = False
        self.mode_machine_ = 0
        self.crc = CRC()

        self._running = True

        # 轨迹记录器
        self._traj_logger = TrajectoryLogger(
            joints=LOG_JOINTS, downsample=LOG_DOWNSAMPLE)

    def init_robot(self):
        topic = (TOPIC_LOWCMD if self.control_mode == MODE_LOWCMD
                 else TOPIC_ARM_SDK)
        self.lowcmd_publisher = ChannelPublisher(topic, LowCmd_)
        self.lowcmd_publisher.Init()
        print(f"[+] 指令发布话题: {topic}")
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self._low_state_handler, 10)

    def _low_state_handler(self, msg: LowState_):
        self.low_state = msg
        if not self.state_ready:
            self.mode_machine_ = msg.mode_machine
            self.state_ready = True

    def wait_for_state(self, timeout=10.0):
        print("[*] 等待机器人状态...")
        t0 = time.time()
        while not self.state_ready:
            if time.time() - t0 > timeout:
                print("[!] 超时: 未收到机器人状态，请检查 DDS 连接")
                sys.exit(1)
            time.sleep(0.1)
        print(f"[+] 已连接到机器人 (mode_machine={self.mode_machine_})")

    # ---------- 自动发现广播 ----------

    def start_discovery_broadcast(self):
        """启动局域网自动发现广播线程"""
        self._disc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._disc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._disc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        t = threading.Thread(target=self._discovery_loop, daemon=True)
        t.start()
        print(f"[+] 发现广播已启动 (UDP {DISCOVERY_BROADCAST_PORT}, 每 {DISCOVERY_INTERVAL:.0f}s)")

    def _discovery_loop(self):
        """周期性广播自身存在"""
        msg = f"G1RC,ip={self._local_ip},port={self.port}"
        while self._running:
            try:
                self._disc_sock.sendto(
                    msg.encode('utf-8'),
                    ('<broadcast>', DISCOVERY_BROADCAST_PORT)
                )
            except OSError:
                pass
            time.sleep(DISCOVERY_INTERVAL)

    # ---------- 控制线程 ----------

    def start_control_thread(self):
        self._ctrl_thread = RecurrentThread(
            interval=CONTROL_DT, target=self._control_loop, name="ctrl"
        )
        self._ctrl_thread.Start()
        print("[+] 控制线程已启动 (500Hz)")

    def _control_loop(self):
        if self.control_mode == MODE_LOWCMD:
            self.low_cmd.mode_pr = 0
            self.low_cmd.mode_machine = self.mode_machine_
        dt = CONTROL_DT

        # 首次: 从当前位置初始化
        if self._cmd_r_sp is None:
            self._cmd_r_sp = self.low_state.motor_state[22].q
            self._cmd_r_sr = self.low_state.motor_state[23].q
            self._cmd_r_sy = self.low_state.motor_state[24].q
            self._cmd_r_el = self.low_state.motor_state[25].q
            self._cmd_r_wr = self.low_state.motor_state[26].q
            self._cmd_l_sp = self.low_state.motor_state[15].q
            self._cmd_l_sr = self.low_state.motor_state[16].q
            self._cmd_l_sy = self.low_state.motor_state[17].q
            self._cmd_l_el = self.low_state.motor_state[18].q
            self._cmd_l_wr = self.low_state.motor_state[19].q
            for wi in self._cmd_wrist:
                self._cmd_wrist[wi] = self.low_state.motor_state[wi].q
            for li in self._lower_joints:
                self._cmd_lower[li] = self.low_state.motor_state[li].q
            self._startup = True
            self._startup_t = 0.0
            print(f"[*] 右臂初始位置: SP={self._cmd_r_sp:+.3f} SR={self._cmd_r_sr:+.3f} "
                  f"SY={self._cmd_r_sy:+.3f} EL={self._cmd_r_el:+.3f} WR={self._cmd_r_wr:+.3f}")
            print(f"[*] 左臂初始位置: SP={self._cmd_l_sp:+.3f} SR={self._cmd_l_sr:+.3f} "
                  f"SY={self._cmd_l_sy:+.3f} EL={self._cmd_l_el:+.3f} WR={self._cmd_l_wr:+.3f}")
            print(f"[*] 启动阶段: {STARTUP_RAMP_DURATION:.1f}s 缓慢接管...")

        # 启动渐变
        if self._startup:
            self._startup_t += dt
            if self._startup_t >= STARTUP_RAMP_DURATION:
                self._startup = False
                print("\n[+] 启动完成, 进入正常控制")

        kp_scale = 1.0
        if self._startup:
            ramp = self._startup_t / STARTUP_RAMP_DURATION
            kp_scale = STARTUP_KP_RATIO + (1.0 - STARTUP_KP_RATIO) * ramp

        # armsdk 用权重而非 kp 缩放实现渐入/渐出, 电机增益始终为整定值
        if self.control_mode == MODE_ARM_SDK:
            kp_scale = 1.0
            if self._releasing:
                self._arm_sdk_weight = max(
                    0.0,
                    self._arm_sdk_weight - dt / ARM_SDK_RELEASE_DURATION)
                if self._arm_sdk_weight <= 0.0:
                    self._released = True
            elif self._startup:
                self._arm_sdk_weight = min(
                    1.0, self._startup_t / STARTUP_RAMP_DURATION)
            else:
                self._arm_sdk_weight = 1.0

        with self.lock:
            mode = self._mode
            tgt_r_sp = self._target_r_sp
            tgt_r_sr = self._target_r_sr
            tgt_r_sy = self._target_r_sy
            tgt_r_el = self._target_r_el
            tgt_r_wr = self._target_r_wr
            tgt_l_sp = self._target_l_sp
            tgt_l_sr = self._target_l_sr
            tgt_l_sy = self._target_l_sy
            tgt_l_el = self._target_l_el
            tgt_l_wr = self._target_l_wr
            target_velocity = self._target_velocity
            tgt_waist_yaw = self._target_waist_yaw
            protocol_version = self._protocol_version
            packet_arrival = self._packet_arrival
            last_recv = self._last_recv

        now = time.monotonic()
        if now - last_recv > TIMEOUT_SEC and last_recv > 0:
            mode = 0

        if self._startup:
            mode = 0

        # V2 包在100 Hz目标之间利用发送端速度做最长20 ms短时外推，
        # 让500 Hz控制目标连续变化；旧包保持原有位置跟随行为。
        if protocol_version >= 2 and packet_arrival > 0.0:
            horizon = min(
                TARGET_PREDICTION_MAX_SEC,
                max(0.0, now - packet_arrival),
            )
            targets = [
                tgt_r_sp, tgt_r_sr, tgt_r_sy, tgt_r_el, tgt_r_wr,
                tgt_l_sp, tgt_l_sr, tgt_l_sy, tgt_l_el, tgt_l_wr,
            ]
            targets = [
                q + dq * horizon
                for q, dq in zip(targets, target_velocity)
            ]
            (tgt_r_sp, tgt_r_sr, tgt_r_sy, tgt_r_el, tgt_r_wr,
             tgt_l_sp, tgt_l_sr, tgt_l_sy, tgt_l_el, tgt_l_wr) = targets

        # mode: 0=双臂回零, 1=右臂跟随, 2=左臂跟随, 3=双臂跟随
        right_follow = mode in (1, 3)
        left_follow = mode in (2, 3)
        max_step = MAX_FOLLOW_SPEED * dt
        ret_spd = (STARTUP_MOVE_SPEED if self._startup else RETURN_SPEED) * dt

        # 右臂
        if right_follow:
            self._cmd_r_sp = self._step_toward(self._cmd_r_sp, tgt_r_sp, max_step)
            self._cmd_r_sr = self._step_toward(self._cmd_r_sr, tgt_r_sr, max_step)
            self._cmd_r_sy = self._step_toward(self._cmd_r_sy, tgt_r_sy, max_step)
            self._cmd_r_el = self._step_toward(self._cmd_r_el, tgt_r_el, max_step)
            self._cmd_r_wr = self._step_toward(self._cmd_r_wr, tgt_r_wr, max_step)
        else:
            self._cmd_r_sp = self._step_toward(self._cmd_r_sp, 0.0, ret_spd)
            self._cmd_r_sr = self._step_toward(self._cmd_r_sr, 0.0, ret_spd)
            self._cmd_r_sy = self._step_toward(self._cmd_r_sy, 0.0, ret_spd)
            self._cmd_r_el = self._step_toward(self._cmd_r_el, 0.0, ret_spd)
            self._cmd_r_wr = self._step_toward(self._cmd_r_wr, 0.0, ret_spd)

        # 左臂
        if left_follow:
            self._cmd_l_sp = self._step_toward(self._cmd_l_sp, tgt_l_sp, max_step)
            self._cmd_l_sr = self._step_toward(self._cmd_l_sr, tgt_l_sr, max_step)
            self._cmd_l_sy = self._step_toward(self._cmd_l_sy, tgt_l_sy, max_step)
            self._cmd_l_el = self._step_toward(self._cmd_l_el, tgt_l_el, max_step)
            self._cmd_l_wr = self._step_toward(self._cmd_l_wr, tgt_l_wr, max_step)
        else:
            self._cmd_l_sp = self._step_toward(self._cmd_l_sp, 0.0, ret_spd)
            self._cmd_l_sr = self._step_toward(self._cmd_l_sr, 0.0, ret_spd)
            self._cmd_l_sy = self._step_toward(self._cmd_l_sy, 0.0, ret_spd)
            self._cmd_l_el = self._step_toward(self._cmd_l_el, 0.0, ret_spd)
            self._cmd_l_wr = self._step_toward(self._cmd_l_wr, 0.0, ret_spd)

        # 腕部 (仅 pitch/yaw 保持零位)
        wrist_spd = (STARTUP_MOVE_SPEED if self._startup else RETURN_SPEED) * dt
        for wi in self._cmd_wrist:
            self._cmd_wrist[wi] = self._step_toward(
                self._cmd_wrist[wi], WRIST_ZERO_OFFSETS[wi], wrist_spd)

        # 下半身 (lowcmd: 腿+腰; armsdk: 仅腰)。
        # waist_yaw 单独排除: 它的回中分支已经在 waist_yaw_command 里, 放在
        # 这里会让它每帧被步进两次, 跟随速度变成不对称的 1.2/1.8 rad/s,
        # 且与 test_waist_follow 测的函数对不上。
        lower_spd = (STARTUP_MOVE_SPEED if self._startup else RETURN_SPEED) * dt
        for li in self._lower_joints:
            if li == WAIST_YAW_JOINT:
                continue
            self._cmd_lower[li] = self._step_toward(
                self._cmd_lower[li], 0.0, lower_spd)

        # 腰部 yaw: 跟随与回中都由这一处负责; roll/pitch (13/14) 走上面的
        # 回中循环。判据见 waist_follow.waist_yaw_command。
        (self._cmd_lower[WAIST_YAW_JOINT],
         self._waist_following) = waist_yaw_command(
            self._cmd_lower[WAIST_YAW_JOINT],
            tgt_waist_yaw,
            allow_waist=self.allow_waist,
            arm_following=right_follow or left_follow,
            dt=dt,
            return_speed=(STARTUP_MOVE_SPEED if self._startup
                          else RETURN_SPEED))

        # 限位
        cmd_r_sp = clamp_angle(self._cmd_r_sp, RIGHT_ARM_JOINT_LIMITS["right_shoulder_pitch"])
        cmd_r_sr = clamp_angle(self._cmd_r_sr, RIGHT_ARM_JOINT_LIMITS["right_shoulder_roll"])
        cmd_r_sy = clamp_angle(self._cmd_r_sy, RIGHT_ARM_JOINT_LIMITS["right_shoulder_yaw"])
        cmd_r_el = clamp_angle(self._cmd_r_el, RIGHT_ARM_JOINT_LIMITS["right_elbow"])
        cmd_r_wr = clamp_angle(self._cmd_r_wr, RIGHT_ARM_JOINT_LIMITS["right_wrist_roll"])

        cmd_l_sp = clamp_angle(self._cmd_l_sp, LEFT_ARM_JOINT_LIMITS["left_shoulder_pitch"])
        cmd_l_sr = clamp_angle(self._cmd_l_sr, LEFT_ARM_JOINT_LIMITS["left_shoulder_roll"])
        cmd_l_sy = clamp_angle(self._cmd_l_sy, LEFT_ARM_JOINT_LIMITS["left_shoulder_yaw"])
        cmd_l_el = clamp_angle(self._cmd_l_el, LEFT_ARM_JOINT_LIMITS["left_elbow"])
        cmd_l_wr = clamp_angle(self._cmd_l_wr, LEFT_ARM_JOINT_LIMITS["left_wrist_roll"])

        arm_target = {
            # 左臂 (官方 SDK: 15=SP, 16=SR, 17=SY, 18=EL, 19=WR)
            15: cmd_l_sp, 16: cmd_l_sr, 17: cmd_l_sy, 18: cmd_l_el, 19: cmd_l_wr,
            # 右臂
            22: cmd_r_sp, 23: cmd_r_sr, 24: cmd_r_sy, 25: cmd_r_el, 26: cmd_r_wr,
            # 腕部 pitch/yaw (20/21=左, 27/28=右), 全程保持零位
            20: self._cmd_wrist[20], 21: self._cmd_wrist[21],
            27: self._cmd_wrist[27], 28: self._cmd_wrist[28],
        }

        # ── 速度前馈: 估计 arm_target 的变化率 ──
        if not hasattr(self, '_prev_arm_target'):
            self._prev_arm_target = dict(arm_target)
            self._prev_arm_target_time = time.monotonic()

        now_mono = time.monotonic()
        dt_ff = now_mono - self._prev_arm_target_time
        arm_dq = {}
        if dt_ff > 1e-6:
            for jid, q in arm_target.items():
                dq = (q - self._prev_arm_target.get(jid, q)) / dt_ff
                # 限幅: 不超过 MAX_FOLLOW_SPEED
                dq = max(-MAX_FOLLOW_SPEED * 2, min(MAX_FOLLOW_SPEED * 2, dq))
                arm_dq[jid] = dq

        self._prev_arm_target = dict(arm_target)
        self._prev_arm_target_time = now_mono

        follow_active = right_follow or left_follow
        if self.control_mode == MODE_LOWCMD:
            self._write_lowcmd(arm_target, arm_dq, kp_scale, follow_active)
        else:
            self._write_arm_sdk(arm_target, arm_dq, follow_active)

        # 轨迹记录 (记录双臂所有关节)
        actual_dict = {j: self.low_state.motor_state[j].q for j in LOG_JOINTS}
        cmd_dict = {j: arm_target[j] for j in LOG_JOINTS}
        self._traj_logger.log(mode, cmd_dict, actual_dict, self.low_state.motor_state)

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)

    def _q_with_ff(self, i, arm_target, arm_dq, follow_active):
        """速度前馈 (与仿真一致): delta_q = Kv_ff * dq_target / Kp"""
        q_cmd = arm_target[i]
        kv_ff = Kv_ff_full[i]
        if kv_ff > 0.0 and i in arm_dq and follow_active:
            kp_i = max(Kp_full[i], 1.0)
            q_cmd += kv_ff * arm_dq[i] / kp_i
        return q_cmd

    def _write_lowcmd(self, arm_target, arm_dq, kp_scale, follow_active):
        """rt/lowcmd: 接管全部29个电机, 下半身被拉到零位 —— 需机器人吊装。"""
        for i in range(G1_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = 1
            self.low_cmd.motor_cmd[i].tau = 0.0
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = Kp_full[i] * kp_scale
            self.low_cmd.motor_cmd[i].kd = Kd_full[i] * kp_scale

            if i in arm_target:
                self.low_cmd.motor_cmd[i].q = self._q_with_ff(
                    i, arm_target, arm_dq, follow_active)
            elif i in self._cmd_lower:
                self.low_cmd.motor_cmd[i].q = self._cmd_lower[i]
            else:
                self.low_cmd.motor_cmd[i].q = self.low_state.motor_state[i].q

    def _write_arm_sdk(self, arm_target, arm_dq, follow_active):
        """rt/arm_sdk: 只下发腰+双臂, 腿和平衡由官方运控服务负责。

        motor_cmd[29].q 是 0~1 的接管权重, 腿部关节完全不写 —— 与官方
        g1_arm7_sdk_dds_example 一致。
        """
        self.low_cmd.motor_cmd[ARM_SDK_WEIGHT_JOINT].q = self._arm_sdk_weight

        for i in ARM_SDK_WAIST_JOINTS:
            self.low_cmd.motor_cmd[i].q = self._cmd_lower[i]
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].tau = 0.0
            self.low_cmd.motor_cmd[i].kp = ARM_SDK_WAIST_KP
            self.low_cmd.motor_cmd[i].kd = ARM_SDK_WAIST_KD

        for i in arm_target:
            self.low_cmd.motor_cmd[i].q = self._q_with_ff(
                i, arm_target, arm_dq, follow_active)
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].tau = 0.0
            self.low_cmd.motor_cmd[i].kp = Kp_full[i]
            self.low_cmd.motor_cmd[i].kd = Kd_full[i]

    @staticmethod
    def _step_toward(current, target, max_step):
        diff = target - current
        if abs(diff) <= max_step:
            return target
        return current + max_step * (1.0 if diff > 0 else -1.0)

    # ---------- UDP 接收 ----------

    def start_recv_thread(self):
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()
        print(f"[+] UDP 接收线程已启动 (0.0.0.0:{self.port})")

    def _recv_loop(self):
        while self._running:
            try:
                data, addr = self.sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break

            result = unpack_arm_command(data)
            if result is None:
                continue

            mode = result["mode"]
            positions = result["positions"]
            velocities = result["velocities"]
            (r_sp, r_sr, r_sy, r_el, r_wr,
             l_sp, l_sr, l_sy, l_el, l_wr) = positions
            now = time.monotonic()
            with self.lock:
                if addr != self._sender_addr:
                    self._last_seq = None
                    self._clock_offset_min_us = None
                    self._transport_excess_samples_ms = []
                seq = result["seq"]
                if seq is not None and self._last_seq is not None:
                    delta = (seq - self._last_seq) & 0xFFFFFFFF
                    if delta == 0 or delta >= 0x80000000:
                        self._out_of_order_count += 1
                        continue
                if seq is not None:
                    self._last_seq = seq
                self._mode = mode
                self._target_r_sp = r_sp
                self._target_r_sr = r_sr
                self._target_r_sy = r_sy
                self._target_r_el = r_el
                self._target_r_wr = r_wr
                self._target_l_sp = l_sp
                self._target_l_sr = l_sr
                self._target_l_sy = l_sy
                self._target_l_el = l_el
                self._target_l_wr = l_wr
                self._target_velocity = tuple(velocities)
                waist = result.get("waist")
                self._target_waist_yaw = (
                    None if waist is None else waist[0])
                self._protocol_version = result["version"]
                self._last_sender_timestamp_us = result[
                    "sender_timestamp_us"]
                self._packet_arrival = now
                if self._last_sender_timestamp_us is not None:
                    arrival_us = time.monotonic_ns() // 1000
                    offset_us = (
                        arrival_us - self._last_sender_timestamp_us)
                    if (self._clock_offset_min_us is None
                            or offset_us < self._clock_offset_min_us):
                        self._clock_offset_min_us = offset_us
                    excess_ms = max(
                        0.0,
                        (offset_us - self._clock_offset_min_us) / 1000.0,
                    )
                    self._transport_excess_samples_ms.append(excess_ms)
                    if len(self._transport_excess_samples_ms) > 2000:
                        self._transport_excess_samples_ms = (
                            self._transport_excess_samples_ms[-2000:])
                self._last_recv = now
                self._sender_addr = addr
                self._recv_count += 1

            # 手部在 self.lock 之外推给各自的线程: HandFollower 自带锁,
            # 在持有 self.lock 时去拿它的锁就多了一条加锁顺序, 迟早撞上。
            # 尾块缺失/这只手标志位没置位时是 None, 跟随线程据此张开。
            # 取快照再遍历: 退出时主线程会 clear() 这个字典, 直接迭代会撞上
            # "dictionary changed size during iteration"。
            followers = list(self.hand_followers.items())
            if followers:
                hands = result.get("hands") or {}
                for side, follower in followers:
                    follower.update(hands.get(side))

            # 记录接收诊断 (UDP间隔、指令跳变)
            self._traj_logger.record_recv(r_sp, r_sr, r_sy, r_el, r_wr,
                                          l_sp, l_sr, l_sy, l_el, l_wr)

    # ---------- 状态打印 ----------

    def print_status(self):
        with self.lock:
            mode = self._mode
            last = self._last_recv
            sender = self._sender_addr
            recv_count = self._recv_count
            protocol_version = self._protocol_version
            out_of_order = self._out_of_order_count
            transport_samples = list(
                self._transport_excess_samples_ms)

        now = time.monotonic()
        age = now - last if last > 0 else -1
        connected = age >= 0 and age < TIMEOUT_SEC

        # 计算接收速率
        dt_rate = now - self._recv_timer
        if dt_rate >= 2.0:
            self._recv_rate = recv_count / dt_rate
            with self.lock:
                self._recv_count = 0
            self._recv_timer = now

        mode_str = "跟随" if mode == 1 else "回零"
        if mode == 2:
            mode_str = "左臂跟随"
        elif mode == 3:
            mode_str = "双臂跟随"
        if self._startup:
            mode_str = f"启动({self._startup_t:.1f}s)"
        if self._releasing:
            mode_str = "交还中"

        if self.hand_only:
            # 不能显示 armsdk —— 这个模式一个电机都没接管, 照抄会让人以为
            # 手臂正被这个进程控制着。
            ctrl_str = "只跑手"
        elif self.control_mode == MODE_ARM_SDK:
            ctrl_str = f"armsdk w={self._arm_sdk_weight:.2f}"
        else:
            ctrl_str = "lowcmd"

        if not self.allow_waist:
            # 显式写出来。留空的话, 忘了加 --waist 时状态行只是"少了一段",
            # 只能靠察觉缺失来发现, 排查起来非常绕。
            waist_str = "腰:关(--waist)  "
        elif connected and protocol_version < 2:
            # 树莓派端发的是 45B legacy UARM 包, 根本没有尾块字段。这种
            # 组合下腰永远不会动, 直接说明白, 不要让人对着 "(回中)" 猜。
            waist_str = "腰:--(发送端不支持)  "
        else:
            waist_q = self._cmd_lower.get(WAIST_YAW_JOINT) or 0.0
            # 跟随中显示实时角度, 门控开着但没收到尾块时明示原因。
            flag = "" if self._waist_following else "(回中)"
            waist_str = f"腰:{math.degrees(waist_q):+4.0f}°{flag}  "

        sender_str = f"{sender[0]}:{sender[1]}" if sender else "无"
        conn_str = f"已连接 ({age:.1f}s)" if connected else "等待连接"

        w20 = self._cmd_wrist.get(20, 0) or 0
        w27 = self._cmd_wrist.get(27, 0) or 0
        w28 = self._cmd_wrist.get(28, 0) or 0

        log_rows = self._traj_logger.rows
        if transport_samples:
            ordered = sorted(transport_samples)
            p50_index = int(0.50 * (len(ordered) - 1))
            p95_index = int(0.95 * (len(ordered) - 1))
            p99_index = int(0.99 * (len(ordered) - 1))
            transport_str = (
                f"抖动:{ordered[p50_index]:.1f}/"
                f"{ordered[p95_index]:.1f}/"
                f"{ordered[p99_index]:.1f}ms")
        else:
            transport_str = "抖动p50/95/99:--"

        followers = list(self.hand_followers.values())
        if self.hand_sides and not followers:
            # 门控开着但一只都没起来。留空的话只能靠察觉缺失来发现。
            hand_str = "手:启动失败  "
        elif followers:
            hand_str = "  ".join(f.status() for f in followers) + "  "
        else:
            hand_str = ""

        print(f"\r  [{mode_str}] "
              f"右:SP={self._cmd_r_sp or 0:+5.2f} SR={self._cmd_r_sr or 0:+5.2f} "
              f"SY={self._cmd_r_sy or 0:+5.2f} EL={self._cmd_r_el or 0:+5.2f} WR={self._cmd_r_wr or 0:+5.2f}  "
              f"左:SP={self._cmd_l_sp or 0:+5.2f} SR={self._cmd_l_sr or 0:+5.2f} "
              f"SY={self._cmd_l_sy or 0:+5.2f} EL={self._cmd_l_el or 0:+5.2f} WR={self._cmd_l_wr or 0:+5.2f}  "
              f"{waist_str}"
              f"{hand_str}"
              f"{conn_str}  发送端:{sender_str}  [{ctrl_str}] "
              f"RX:{self._recv_rate:.0f}Hz V{protocol_version} "
              f"{transport_str} 乱序:{out_of_order}  LOG:{log_rows}    ",
              end="", flush=True)

    # ---------- 灵巧手 ----------

    def start_hand_followers(self):
        """按 --hand 起手部驱动线程。手起不来绝不能连累手臂。

        ``hand_driver`` 在这里才导入而不是放模块顶上: 它需要 hand_mapping.py
        (从 PC端 拷过来的同一份, 不是副本), 忘了拷的话只应该让手起不来,
        手臂照跑。waist_follow 那次就是顶层导入, 少拷一个文件整个接收端
        起不来 —— 同样的坑不踩第二次。
        """
        if not self.hand_sides:
            return
        try:
            from hand_driver import HandFollower
        except ImportError as exc:
            print(f"[!] 灵巧手模块导入失败, 已跳过手部 (双臂不受影响): {exc}")
            print("    需要 hand_driver.py 和 hand_mapping.py 与本程序同目录")
            return

        for side in self.hand_sides:
            try:
                follower = HandFollower(
                    side, range_scale=self.hand_range,
                    haptic=self.hand_haptic,
                    haptic_host=self.hand_haptic_host,
                    touch_hz=self.hand_touch_hz,
                    log=lambda m: print(f"\n{m}"))
            except Exception as exc:
                print(f"[!] {side}手初始化失败, 已跳过: {exc}")
                continue
            follower.start()
            self.hand_followers[side] = follower
            print(f"[*] {side}手驱动已启动 → {follower.ip}:{follower.port} "
                  f"(连接在后台重试, 手不在也不阻塞启动)")

    def stop_hand_followers(self):
        """张开并停掉所有手部线程。可重入。"""
        for side, follower in list(self.hand_followers.items()):
            try:
                follower.stop()
            except Exception as exc:
                print(f"[!] {side}手退出时出错: {exc}")
        self.hand_followers.clear()

    # ---------- 开机前置检查 ----------

    def preflight(self, assume_yes=False):
        """检查运控服务状态是否与所选控制接口匹配。不匹配则中止。"""
        msc = _make_motion_switcher()
        active, name = query_motion_service(msc)

        if self.control_mode == MODE_ARM_SDK:
            if active is True:
                print(f"[+] 运控服务 ({name}) 正在运行, arm_sdk 可用, "
                      f"双腿平衡由官方服务负责")
            elif active is False:
                print("[!] 运控服务未运行 —— arm_sdk 接口不会生效, 机器人不会动。")
                print("    请先用遥控器把机器人切到站立/运控状态 (R1+X 起身), 再运行本程序。")
                if not self._confirm("仍要继续吗?", assume_yes):
                    sys.exit(1)
            else:
                # armsdk 下查不到状态不危险: 运控没跑的话 arm_sdk 只是不生效,
                # 机器人不动而已, 不会打架。所以只提示, 不拦。
                print("[!] 无法查询运控服务状态 —— 本机 unitree_sdk2py 没有")
                print("    MotionSwitcher, 这项检查在这台机器上始终无法进行。")
                print("    请自行确认机器人已站立、官方运控在跑;")
                print("    若启动后机器人完全不动, 十有八九就是运控没起来。")
            return

        # ---- lowcmd: 全身接管, 无平衡 ----
        print()
        print("!" * 65)
        print("  警告: lowcmd 模式接管全部29个电机, 并把双腿和腰拉到零位。")
        print("  本程序没有任何平衡控制 —— 机器人必须已经吊装/挂在支架上悬空,")
        print("  双脚不得承重。脚踩地面运行此模式机器人一定会摔倒。")
        print("!" * 65)
        if not self._confirm("机器人已确认吊装悬空?", assume_yes):
            sys.exit(1)

        if active is True:
            print(f"[!] 运控服务 ({name}) 仍在运行, 会与 rt/lowcmd 抢同一批电机。")
            if self._confirm("现在停止运控服务 (机器人将立即失去平衡)?", assume_yes):
                if not release_motion_service(msc):
                    print("[!] 停止运控服务失败, 请用遥控器 L2+R2 进入阻尼态后重试。")
                    sys.exit(1)
                print("[+] 运控服务已停止")
            else:
                sys.exit(1)
        elif active is False:
            print("[+] 运控服务已停止, lowcmd 可独占电机")
        else:
            # 这里必须拦。查不到状态时本程序既无法确认、也无法停止运控服务;
            # 万一它还在跑, 就是两个控制器同时写同一批电机 —— 这个失效模式
            # 比"机器人不动"严重得多, 不能只打印一句提示就放行。
            print()
            print("!" * 65)
            print("  无法查询运控服务状态 —— 本机 unitree_sdk2py 没有")
            print("  MotionSwitcher, 本程序既查不到、也停不掉运控服务。")
            print()
            print("  如果运控服务此刻仍在运行, 它会和 rt/lowcmd 同时写同一批")
            print("  电机, 两个控制器互相打架, 后果不可预期。")
            print()
            print("  请先用遥控器让机器人进入调试/阻尼态, 确认官方运控已经")
            print("  交出电机控制权, 再继续。")
            print("!" * 65)
            if not self._confirm(
                    "已确认官方运控服务不在控制电机?", assume_yes):
                sys.exit(1)

    @staticmethod
    def _confirm(prompt, assume_yes=False):
        if assume_yes:
            print(f"  {prompt} -> 已由 --yes 确认")
            return True
        try:
            return input(f"  {prompt} [y/N] ").strip().lower() in ("y", "yes")
        except EOFError:
            return False

    # ---------- 主循环 ----------

    def run(self, assume_yes=False):
        print("=" * 65)
        print("  宇树G1 双臂遥操控接收端 (自动发现版)")
        print(f"  本机IP: {self._local_ip}")
        print(f"  监听端口: {self.port}")
        print(f"  发现广播端口: {DISCOVERY_BROADCAST_PORT}")
        print(f"  控制接口: {self.control_mode} "
              f"({TOPIC_LOWCMD if self.control_mode == MODE_LOWCMD else TOPIC_ARM_SDK})")
        print(f"  DDS 网卡: {self.iface or '默认 (板载机运行时可省略)'}")
        if self.allow_waist:
            print(f"  腰部跟随: 开启 (仅 waist_yaw, 限位 ±"
                  f"{math.degrees(WAIST_YAW_LIMITS[1]):.0f}°, "
                  f"限速 {WAIST_FOLLOW_SPEED:.1f} rad/s)")
            print("  [!] 扭腰会改变上半身重心, 请从小幅度开始")
        else:
            print("  腰部跟随: 关闭 (腰保持回中), 需要时加 --waist")
        if self.hand_sides:
            used = (self.hand_range if self.hand_range is not None
                    else hand_driver_range_scale())
            print(f"  灵巧手: {'+'.join(self.hand_sides)} "
                  f"(行程 {used:.0%}, 断流 0.5s 自动张开)")
            if used > 0.5:
                print("  [!] 大行程: 手指可以完全闭合, 会夹伤; 抓到硬物会"
                      "顶到堵转。第一次动手前请把手移开抓握范围")
            else:
                print("  [!] 手指会夹人, 第一次动手前请把手移开抓握范围")
        else:
            print("  灵巧手: 关闭, 需要时加 --hand right")
        print("=" * 65)

        if self.hand_only:
            # 完全不碰 DDS、不接管任何电机 —— 手臂一动不动, 机器人可以吊着、
            # 趴着、运控服务没起来, 都能验证"手套→PC→板载机→手"整条链。
            # 这才是文档里"先只跑手"真正的意思: 不带这个开关的话, armsdk
            # 权重会从 0 升到 1, 手臂立刻被接管, 那不叫只跑手。
            print("[*] 只跑手模式: 不初始化 DDS, 不接管任何电机")
            # 控制线程不跑, 没人会把这个标志清掉, 状态行会一直卡在"启动"。
            self._startup = False
            self.start_recv_thread()
            self.start_discovery_broadcast()
            self.start_hand_followers()
            print("\n[*] 等待发送端连接...")
            print("    Ctrl+C 退出 (退出时手会先张开)")
            print()
            try:
                while True:
                    self.print_status()
                    time.sleep(0.5)
            except KeyboardInterrupt:
                self.shutdown()
            return

        if self.iface:
            ChannelFactoryInitialize(0, self.iface)
        else:
            ChannelFactoryInitialize(0)

        self.preflight(assume_yes)

        self.init_robot()
        self.wait_for_state()
        self.start_control_thread()
        self.start_recv_thread()
        self.start_discovery_broadcast()
        self.start_hand_followers()

        wrist_str = ", ".join(f"{k}={v:+.4f}" for k, v in WRIST_ZERO_OFFSETS.items())
        print(f"\n[*] 腕部零位偏移: {wrist_str}")
        self._traj_logger.start()
        print("[*] 等待发送端连接 (树莓派自动发现中)...")
        print("    Ctrl+C 安全退出 (双臂缓慢回零)")
        print()

        try:
            while True:
                self.print_status()
                time.sleep(0.5)
        except KeyboardInterrupt:
            self.shutdown()

    def shutdown(self):
        """回零 → (armsdk) 权重渐出交还运控服务 → 停线程。可重入。"""
        if getattr(self, '_shutting_down', False):
            return
        self._shutting_down = True

        # 手先张开: 手指夹着东西比手臂停在半空更急, 而且张手不需要等任何
        # 姿态过渡。stop() 内部会渐变张开而不是一帧弹开。
        if self.hand_followers:
            print("\n\n[*] 灵巧手张开中...")
            self.stop_hand_followers()

        if self.hand_only:
            # 没接管过任何电机, 没有可回零的东西, 也没有权重要交还。
            self._running = False
            print("[+] 已安全退出")
            return

        print("\n\n[*] 收到退出信号, 双臂回零中...")
        with self.lock:
            self._mode = 0
        time.sleep(3.0)

        if self.control_mode == MODE_ARM_SDK:
            print(f"[*] 交还手臂控制权 ({ARM_SDK_RELEASE_DURATION:.0f}s 渐出)...")
            self._releasing = True
            deadline = time.monotonic() + ARM_SDK_RELEASE_DURATION + 1.0
            while not self._released and time.monotonic() < deadline:
                time.sleep(0.05)
            if not self._released:
                print("[!] 权重未降到0 (控制线程可能已停), 请检查机器人状态")

        self._running = False
        self._traj_logger.stop()
        print("[+] 已安全退出")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="宇树G1 双臂遥操控接收端",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
控制接口:
  armsdk (默认)  发布 rt/arm_sdk, 只接管腰+双臂, 双腿平衡由官方运控服务负责。
                 机器人可以站在地上, 是真机遥操的常规选择。
  lowcmd         发布 rt/lowcmd, 接管全部29个电机并把下半身拉到零位, 无平衡。
                 仅用于机器人吊装悬空时的调试。
""")
    parser.add_argument("port", nargs="?", type=int, default=DEFAULT_PORT,
                        help=f"UDP 监听端口 (默认 {DEFAULT_PORT})")
    parser.add_argument("--mode", choices=[MODE_ARM_SDK, MODE_LOWCMD],
                        default=MODE_ARM_SDK, help="控制接口 (默认 armsdk)")
    parser.add_argument("--iface", default=None,
                        help="DDS 网卡名 (如 eth0)。在机器人板载计算机上运行"
                             "可省略; 在外部PC上运行必须指定")
    parser.add_argument("--waist", action="store_true",
                        help="允许发送端的 UAWS 尾块驱动 waist_yaw (腰部左右"
                             "扭转)。默认关闭 —— 扭腰改变上半身重心, 对运控"
                             "服务是真实扰动, 请从小幅度开始")
    parser.add_argument("--hand", choices=("right", "left", "both"),
                        default=None,
                        help="驱动因时 FTP 灵巧手 (板载机直连手的 Modbus, "
                             "192.168.123.211/.210)。默认关闭 —— 手指会夹人。"
                             "闭合度来自发送端的 UHND 尾块, 断流 0.5s 自动张开")
    parser.add_argument("--hand-range", type=float, default=None,
                        metavar="0~1",
                        help="手部行程比例, 默认整程 (1.0)。0.3 是早期空载"
                             "验证值, 抓不住东西但绝对安全。第一次让手动、"
                             "或换了抓取对象时可以先用小值试")
    parser.add_argument("--hand-only", action="store_true",
                        help="只驱动灵巧手, 完全不碰 DDS 和任何电机。手臂一动"
                             "不动, 运控服务没起来也能跑 —— 用来单独验证"
                             "手套→手这条链。必须配合 --hand 使用")
    parser.add_argument("--hand-haptic", action="store_true",
                        help="开启INSPIRE触觉发布和外骨骼力控覆盖："
                             "右手9201/9301，左手9202/9302。默认关闭")
    parser.add_argument("--hand-haptic-host", default="0.0.0.0",
                        help="力反馈TCP监听地址（默认0.0.0.0）")
    parser.add_argument("--hand-touch-hz", type=float, default=5.0,
                        help="五指top_touch读取频率（默认5Hz，范围1..10）")
    parser.add_argument("--yes", action="store_true",
                        help="跳过交互确认 (仅在已知运行环境安全时使用)")
    args = parser.parse_args(argv)
    if args.hand_only and args.hand is None:
        parser.error("--hand-only 需要同时给 --hand right/left/both, "
                     "否则这个进程什么都不驱动")
    if args.hand_haptic and args.hand is None:
        parser.error("--hand-haptic需要同时给--hand right/left/both")
    if not 1.0 <= args.hand_touch_hz <= 10.0:
        parser.error("--hand-touch-hz必须在1..10Hz")
    if args.hand_range is not None and not 0.0 <= args.hand_range <= 1.0:
        parser.error("--hand-range 必须在 0~1 之间")
    return args


def hand_sides_from_arg(value):
    """--hand 的取值 → 要驱动的手。None/未给 → 空元组 (不驱动任何手)。"""
    if value is None:
        return ()
    if value == "both":
        return ("right", "left")
    return (value,)


def main():
    args = parse_args()

    receiver = RobotArmReceiver(args.port, control_mode=args.mode,
                                iface=args.iface, allow_waist=args.waist,
                                hand_sides=hand_sides_from_arg(args.hand),
                                hand_only=args.hand_only,
                                hand_range=args.hand_range,
                                hand_haptic=args.hand_haptic,
                                hand_haptic_host=args.hand_haptic_host,
                                hand_touch_hz=args.hand_touch_hz)

    def signal_handler(sig, frame):
        receiver.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    receiver.run(assume_yes=args.yes)


if __name__ == "__main__":
    main()
