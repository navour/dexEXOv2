#!/usr/bin/env python3
"""
宇树G1 右臂遥操控接收端 (带局域网自动发现)

功能:
  - UDP 广播自身存在, 供树莓派自动发现
  - UDP 接收树莓派发来的电机角度指令
  - 使用 rt/lowcmd 低层级控制执行右臂运动
  - 安全保护: mode=0 时缓慢回零, mode=1 时跟随
  - 启动渐变: 缓慢从当前位置接管, 避免突跳
  - 轨迹记录: 记录预期与实际轨迹差值至CSV文件, 便于调试

通信协议:
  指令包: 'UARM' + mode(1B) + sp(f) + sr(f) + sy(f) + el(f)
  发现广播: 'G1RC,ip=<IP>,port=<PORT>'  (每2秒)

使用:
  python3 robot_arm_receiver.py [端口号]
  默认端口: 9527
"""

import time
import sys
import os
import signal
import threading
import socket
import struct

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
    27: 0.0,   # RightWristPitch
    28: 0.0,   # RightWristYaw
    20: 0.0,   # LeftWristPitch
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

# 控制参数
CONTROL_DT = 0.002   # 2ms (500Hz)
DEFAULT_PORT = 9527

# 回零速度 (rad/s)
RETURN_SPEED = 0.3

# 跟随平滑
MAX_FOLLOW_SPEED = 5.0  # rad/s (3.5→5.0, 左臂更重需要更快响应)

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

# ======================== 通信协议 ========================
PACKET_HEADER = b'UARM'
PACKET_FMT_SINGLE = '!4sBffff'          # 单臂 (旧协议)
PACKET_FMT_DUAL = '!4sBffffffff'        # 双臂 (旧双臂协议)
PACKET_FMT_DUAL_WR = '!4sBffffffffff'   # 双臂+腕roll (最新协议)
PACKET_SIZE_SINGLE = struct.calcsize(PACKET_FMT_SINGLE)
PACKET_SIZE_DUAL = struct.calcsize(PACKET_FMT_DUAL)
PACKET_SIZE_DUAL_WR = struct.calcsize(PACKET_FMT_DUAL_WR)

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


def unpack_arm_cmd(data):
    """解析双臂/单臂指令包。返回 (mode, r_sp, r_sr, r_sy, r_el, r_wr, l_sp, l_sr, l_sy, l_el, l_wr)。"""
    n = len(data)
    if n >= PACKET_SIZE_DUAL_WR:
        header, mode, r_sp, r_sr, r_sy, r_el, r_wr, l_sp, l_sr, l_sy, l_el, l_wr = struct.unpack(
            PACKET_FMT_DUAL_WR, data[:PACKET_SIZE_DUAL_WR])
        if header != PACKET_HEADER:
            return None
        return mode, r_sp, r_sr, r_sy, r_el, r_wr, l_sp, l_sr, l_sy, l_el, l_wr
    elif n >= PACKET_SIZE_DUAL:
        header, mode, r_sp, r_sr, r_sy, r_el, l_sp, l_sr, l_sy, l_el = struct.unpack(
            PACKET_FMT_DUAL, data[:PACKET_SIZE_DUAL])
        if header != PACKET_HEADER:
            return None
        return mode, r_sp, r_sr, r_sy, r_el, 0.0, l_sp, l_sr, l_sy, l_el, 0.0
    elif n >= PACKET_SIZE_SINGLE:
        header, mode, r_sp, r_sr, r_sy, r_el = struct.unpack(
            PACKET_FMT_SINGLE, data[:PACKET_SIZE_SINGLE])
        if header != PACKET_HEADER:
            return None
        return mode, r_sp, r_sr, r_sy, r_el, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    return None


def clamp_angle(val, limits):
    return max(limits[0], min(limits[1], val))


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
    def __init__(self, port):
        self.port = port
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
        self._cmd_wrist = {20: None, 27: None, 28: None}
        self._cmd_lower = {i: None for i in range(15)}

        # 启动阶段
        self._startup = True
        self._startup_t = 0.0

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
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()
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
            for wi in (20, 27, 28):
                self._cmd_wrist[wi] = self.low_state.motor_state[wi].q
            for li in range(15):
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
            last_recv = self._last_recv

        now = time.monotonic()
        if now - last_recv > TIMEOUT_SEC and last_recv > 0:
            mode = 0

        if self._startup:
            mode = 0

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
        for wi in (20, 27, 28):
            self._cmd_wrist[wi] = self._step_toward(
                self._cmd_wrist[wi], WRIST_ZERO_OFFSETS[wi], wrist_spd)

        # 下半身
        lower_spd = (STARTUP_MOVE_SPEED if self._startup else RETURN_SPEED) * dt
        for li in range(15):
            self._cmd_lower[li] = self._step_toward(
                self._cmd_lower[li], 0.0, lower_spd)

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
            # 腕部 pitch/yaw
            20: self._cmd_wrist[20], 27: self._cmd_wrist[27], 28: self._cmd_wrist[28],
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

        for i in range(G1_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = 1
            self.low_cmd.motor_cmd[i].tau = 0.0
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = Kp_full[i] * kp_scale
            self.low_cmd.motor_cmd[i].kd = Kd_full[i] * kp_scale

            if i in arm_target:
                # 速度前馈 (与仿真一致): delta_q = Kv_ff * dq_target / Kp
                q_cmd = arm_target[i]
                kv_ff = Kv_ff_full[i]
                if kv_ff > 0.0 and i in arm_dq and (right_follow or left_follow):
                    kp_i = max(Kp_full[i], 1.0)
                    delta_q = kv_ff * arm_dq[i] / kp_i
                    q_cmd += delta_q
                self.low_cmd.motor_cmd[i].q = q_cmd
            elif i in self._cmd_lower:
                self.low_cmd.motor_cmd[i].q = self._cmd_lower[i]
            else:
                self.low_cmd.motor_cmd[i].q = self.low_state.motor_state[i].q

        # 轨迹记录 (记录双臂所有关节)
        actual_dict = {j: self.low_state.motor_state[j].q for j in LOG_JOINTS}
        cmd_dict = {j: arm_target[j] for j in LOG_JOINTS}
        self._traj_logger.log(mode, cmd_dict, actual_dict, self.low_state.motor_state)

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)

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

            result = unpack_arm_cmd(data)
            if result is None:
                continue

            mode, r_sp, r_sr, r_sy, r_el, r_wr, l_sp, l_sr, l_sy, l_el, l_wr = result
            with self.lock:
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
                self._last_recv = time.monotonic()
                self._sender_addr = addr
                self._recv_count += 1

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

        sender_str = f"{sender[0]}:{sender[1]}" if sender else "无"
        conn_str = f"已连接 ({age:.1f}s)" if connected else "等待连接"

        w20 = self._cmd_wrist.get(20, 0) or 0
        w27 = self._cmd_wrist.get(27, 0) or 0
        w28 = self._cmd_wrist.get(28, 0) or 0

        log_rows = self._traj_logger.rows

        print(f"\r  [{mode_str}] "
              f"右:SP={self._cmd_r_sp or 0:+5.2f} SR={self._cmd_r_sr or 0:+5.2f} "
              f"SY={self._cmd_r_sy or 0:+5.2f} EL={self._cmd_r_el or 0:+5.2f} WR={self._cmd_r_wr or 0:+5.2f}  "
              f"左:SP={self._cmd_l_sp or 0:+5.2f} SR={self._cmd_l_sr or 0:+5.2f} "
              f"SY={self._cmd_l_sy or 0:+5.2f} EL={self._cmd_l_el or 0:+5.2f} WR={self._cmd_l_wr or 0:+5.2f}  "
              f"{conn_str}  发送端:{sender_str}  "
              f"RX:{self._recv_rate:.0f}Hz  LOG:{log_rows}    ",
              end="", flush=True)

    # ---------- 主循环 ----------

    def run(self):
        print("=" * 65)
        print("  宇树G1 双臂遥操控接收端 (自动发现版)")
        print(f"  本机IP: {self._local_ip}")
        print(f"  监听端口: {self.port}")
        print(f"  发现广播端口: {DISCOVERY_BROADCAST_PORT}")
        print("=" * 65)

        ChannelFactoryInitialize(0)
        self.init_robot()
        self.wait_for_state()
        self.start_control_thread()
        self.start_recv_thread()
        self.start_discovery_broadcast()

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
            print("\n\n[*] 收到退出信号, 双臂回零中...")
            with self.lock:
                self._mode = 0
            time.sleep(3.0)
            self._running = False
            self._traj_logger.stop()
            print("[+] 已安全退出")


def main():
    port = DEFAULT_PORT
    if len(sys.argv) >= 2:
        port = int(sys.argv[1])

    receiver = RobotArmReceiver(port)

    def signal_handler(sig, frame):
        print("\n[*] 收到退出信号, 双臂回零中...")
        with receiver.lock:
            receiver._mode = 0
        time.sleep(3.0)
        receiver._running = False
        receiver._traj_logger.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    receiver.run()


if __name__ == "__main__":
    main()
