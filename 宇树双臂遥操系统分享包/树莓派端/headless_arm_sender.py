#!/usr/bin/env python3
# coding: UTF-8
"""
无头树莓派 双臂遥操控发送端 (curses TUI)

功能:
  - 自动发现局域网中的无线 IMU (MultiImuService)
  - 侧平举校准 (C 键, 按当前活跃手臂)
  - IK 逆运动学解算 → 电机角度
  - UDP 发送双臂角度到机器人
  - R 键控制右臂, L 键控制左臂, SPACE 切换跟随

操作:
  R     - 切换到右臂控制
  L     - 切换到左臂控制
  SPACE - 按住跟随, 松开回零
  C     - 校准当前活跃手臂 (保持3秒侧平举)
  X     - 重置校准
  Q/ESC - 退出

使用:
  python3 headless_arm_sender.py [机器人IP] [端口]
  默认: 192.168.244.220:9527
"""
import sys
import os
import time
import threading
import json
import socket
import struct
import curses
import csv
import importlib.util
from datetime import datetime

import numpy as np

# ===== 路径设置 =====
_MY_DIR = os.path.dirname(os.path.abspath(__file__))

# 无线 IMU API
_IMU_API_DIR = os.path.join(_MY_DIR, 'IMU_API')
_IMU_API_FILE = os.path.join(_IMU_API_DIR, 'multi_imu_core.py')
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
    ArmDirectionCalibrator,
    TwistCalibration,
    IncrementalTracker,
    CALIB_POSES,
    T_CHEST2DISP,
    quat_mul as cal_quat_mul,
    quat_conj as cal_quat_conj,
    quat_rotate as cal_quat_rotate,
    normalize_quat as cal_normalize_quat,
    normalize_vec as cal_normalize_vec,
    average_quaternions as cal_average_quaternions,
    swing_twist_angle,
)
from arm_solver import (
    direction_to_joint_angles,
    compute_wrist_roll_from_palm,
    forward_kinematics_right_arm_full,
    forward_kinematics_left_arm_full,
    clamp_joint_angles,
)
from robot_config import (
    JointAngles,
    RIGHT_ARM_JOINT_LIMITS,
    RIGHT_ARM_MOTOR_INDEX,
    LEFT_ARM_JOINT_LIMITS,
    LEFT_ARM_MOTOR_INDEX,
    RIGHT_UPPER_ARM_FIXED_ID,
    RIGHT_FOREARM_FIXED_ID,
    LEFT_UPPER_ARM_FIXED_ID,
    LEFT_FOREARM_FIXED_ID,
    CHEST_FIXED_ID,
    get_joint_limits,
    get_motor_index,
)

# ========================================================
#  配置
# ========================================================
DEFAULT_ROBOT_IP = ""  # 留空则自动发现
DEFAULT_ROBOT_PORT = 9527

# 自动发现: 监听机器人广播
DISCOVERY_LISTEN_PORT = 9528
DISCOVERY_TIMEOUT = 8.0

ROLE_MAP_FILE = os.path.join(_MY_DIR, "wireless_imu_roles.json")
WIRELESS_POLL_INTERVAL = 0.01
CALIB_STATIC_DURATION = 3.0   # 静态姿态收集时间
CALIB_ROLL_DURATION = 5.0     # Roll轴收集时间

# UDP 发送频率
UDP_SEND_HZ = 50
UDP_SEND_DT = 1.0 / UDP_SEND_HZ

# 回零速度
RETURN_SPEED = 0.5

# ========================================================
#  UDP 通信协议 (双臂)
# ========================================================
PACKET_HEADER = b'UARM'
PACKET_FMT = '!4sBffffffffff'  # header(4B) + mode(1B) + 10 floats (8 arm + 2 wrist_roll)
PACKET_SIZE = struct.calcsize(PACKET_FMT)

# mode: 0=双臂回零, 1=右臂跟随, 2=左臂跟随, 3=双臂跟随

# 诊断日志配置
DIAG_LOG_HZ = 50
JUMP_THRESHOLD_RAD = 0.15
IMU_STALE_THRESHOLD = 0.1


def pack_dual_arm_cmd(mode, r_sp, r_sr, r_sy, r_el, r_wr, l_sp, l_sr, l_sy, l_el, l_wr):
    return struct.pack(PACKET_FMT, PACKET_HEADER, mode,
                       r_sp, r_sr, r_sy, r_el, r_wr,
                       l_sp, l_sr, l_sy, l_el, l_wr)


# ========================================================
#  单臂状态 (使用 ArmDirectionCalibrator)
# ========================================================

class _ArmState:
    """单个手臂的状态管理 — 使用 4 步校准 (对齐 upper_arm_3pose.py)."""

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
        # IK 结果
        self.angles = JointAngles(0., 0., 0., 0., 0., side=side)
        self.send_angles = JointAngles(0., 0., 0., 0., 0., side=side)
        # 限位和电机映射
        self.joint_limits = get_joint_limits(side)
        self.motor_index = get_motor_index(side)
        self.limit_prefix = f"{side}_"

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


# ========================================================
#  诊断日志记录器
# ========================================================

class DiagnosticLogger:

    HEADER = [
        'time', 'mode', 'active_arm',
        'r_sp', 'r_sr', 'r_sy', 'r_el', 'r_wr',
        'l_sp', 'l_sr', 'l_sy', 'l_el', 'l_wr',
        'send_r_sp', 'send_r_sr', 'send_r_sy', 'send_r_el', 'send_r_wr',
        'send_l_sp', 'send_l_sr', 'send_l_sy', 'send_l_el', 'send_l_wr',
        'event',
    ]

    def __init__(self, log_dir=None):
        self._log_dir = log_dir or os.path.dirname(os.path.abspath(__file__))
        self._file = None
        self._writer = None
        self._t0 = None
        self._rows = 0
        self._events = []

    def start(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self._log_dir, f"diag_log_{ts}.csv")
        self._file = open(path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADER)
        self._t0 = time.monotonic()
        self._rows = 0
        return path

    def log(self, mode, active_arm, right, left, event_str=''):
        if self._writer is None:
            return
        t = time.monotonic() - self._t0
        events = list(self._events)
        self._events.clear()
        if event_str:
            events.append(event_str)
        ev = ';'.join(events)

        row = [f'{t:.4f}', mode, active_arm]
        # IK angles
        for arm in (right, left):
            row += [f'{arm.angles.shoulder_pitch:.5f}', f'{arm.angles.shoulder_roll:.5f}',
                    f'{arm.angles.shoulder_yaw:.5f}', f'{arm.angles.elbow:.5f}',
                    f'{arm.angles.wrist_roll:.5f}']
        # Send angles
        for arm in (right, left):
            row += [f'{arm.send_angles.shoulder_pitch:.5f}', f'{arm.send_angles.shoulder_roll:.5f}',
                    f'{arm.send_angles.shoulder_yaw:.5f}', f'{arm.send_angles.elbow:.5f}',
                    f'{arm.send_angles.wrist_roll:.5f}']
        row.append(ev)
        self._writer.writerow(row)
        self._rows += 1
        if self._rows % 100 == 0:
            self._file.flush()

    def add_event(self, event_str):
        self._events.append(event_str)

    def stop(self):
        if self._file:
            self._file.flush()
            self._file.close()
            self._file = None
            self._writer = None

    @property
    def rows(self):
        return self._rows


# ========================================================
#  主控制器
# ========================================================

class HeadlessArmSender:
    def __init__(self, robot_ip, robot_port):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.lock = threading.Lock()

        # UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 无线 IMU 服务
        self.imu_service = MultiImuService()

        # 胸部 IMU
        self.chest_node_id = None
        self.q_chest = np.array([1., 0., 0., 0.])
        self.chest_connected = False

        # 双臂状态
        self.right = _ArmState("right")
        self.left = _ArmState("left")

        # 活跃手臂
        self.active_arm = "right"

        # IMU 设备列表
        self.latest_devices = []
        self.latest_device_dict = {}

        # 跟随状态
        self.following = False

        # 统计
        self._sample_n = 0
        self._rate_t = time.monotonic()
        self.sample_rate = 0.0

        # 机器人自动发现
        self.robot_discovered = False
        self.robot_last_seen = 0.0
        self._discovery_running = False
        self._discovery_thread = None

        # IK 后台线程 (方向→关节角度, 已内联到无线循环, 但保留用于 FK 可视化)
        self._ik_lock = threading.Lock()
        self._ik_count = 0
        self._ik_total_time = 0.0
        self._ik_avg_ms = 0.0

        # 线程
        self._wireless_running = False
        self._wireless_thread = None

        # 日志
        self._log_lines = []
        self._max_log = 100

        # 诊断日志
        self._diag_logger = DiagnosticLogger()
        self._last_imu_update = time.monotonic()

        # 4步校准状态
        self._calib_step = -1  # -1=未在校准, >=0=CALIB_POSES 索引
        self._calib_waiting = False  # True=等待用户按 C 确认开始采集
        self._calib_arm = None
        self._calib_start = 0.0
        self._calib_samples_chest = []
        self._calib_samples_upper = []
        self._calib_samples_forearm = []
        self._pose_averages = []
        self._roll_upper_data = []
        self._roll_forearm_data = []

    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self._log_lines.append(f"[{ts}] {msg}")
        if len(self._log_lines) > self._max_log:
            self._log_lines = self._log_lines[-self._max_log:]

    @property
    def active(self):
        return self.right if self.active_arm == "right" else self.left

    # ---------- 机器人自动发现 ----------

    def _start_discovery(self):
        if not self.robot_ip:
            self._discovery_running = True
            self._discovery_thread = threading.Thread(
                target=self._discovery_loop, daemon=True)
            self._discovery_thread.start()
            self._log("正在搜索机器人...")
        else:
            self.robot_discovered = True
            self._log(f"使用指定机器人: {self.robot_ip}:{self.robot_port}")

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

        while self._discovery_running:
            try:
                data, addr = sock.recvfrom(256)
            except socket.timeout:
                if (self.robot_discovered and
                        time.monotonic() - self.robot_last_seen > DISCOVERY_TIMEOUT):
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

        sock.close()

    # ---------- IK (已内联到 _wireless_loop) ----------

    def _start_ik_thread(self):
        pass  # 不再需要单独的 IK 线程

    def _stop_ik_thread(self):
        pass

    # ---------- 无线 IMU ----------

    def _load_role_map(self):
        if not os.path.exists(ROLE_MAP_FILE):
            return
        try:
            with open(ROLE_MAP_FILE, 'r') as f:
                data = json.load(f)
            self.right.upper_node_id = data.get("right_upper_node_id")
            self.right.forearm_node_id = data.get("right_forearm_node_id")
            self.left.upper_node_id = data.get("left_upper_node_id")
            self.left.forearm_node_id = data.get("left_forearm_node_id")
            self.chest_node_id = data.get("chest_node_id")
            self._log(f"加载映射: chest={self.chest_node_id}, "
                      f"右臂={self.right.upper_node_id}/{self.right.forearm_node_id}, "
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
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        with open(ROLE_MAP_FILE, 'w') as f:
            json.dump(data, f, indent=2)
        self._log("IMU 映射已保存")

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

    def _init_imu(self):
        self._load_role_map()
        self._log("启动无线 IMU 服务...")
        self.imu_service.start()
        self._wireless_running = True
        self._wireless_thread = threading.Thread(target=self._wireless_loop, daemon=True)
        self._wireless_thread.start()

    def _stop_imu(self):
        self._wireless_running = False
        if self._wireless_thread is not None:
            self._wireless_thread.join(timeout=1.0)
        self.imu_service.stop()

    def _read_imu(self, dev_dict, node_id):
        """读取并归一化一个 IMU 的四元数"""
        d = dev_dict.get(node_id) if node_id else None
        if d is None or not d.connected:
            return None, False, None
        q = cal_normalize_quat(np.array(d.quat, dtype=float))
        return q, True, d

    def _wireless_loop(self):
        while self._wireless_running:
            devices = self.imu_service.list_devices()
            dev_dict = {d.node_id: d for d in devices}
            now = time.monotonic()

            with self.lock:
                self.latest_devices = devices
                self.latest_device_dict = dev_dict
                self._try_assign_by_fixed_id(devices)

                # 胸部
                q_c, self.chest_connected, _ = self._read_imu(dev_dict, self.chest_node_id)
                if q_c is not None:
                    self.q_chest = q_c

                # 读取双臂 IMU
                for arm in (self.right, self.left):
                    q_u, arm.upper_connected, _ = self._read_imu(dev_dict, arm.upper_node_id)
                    q_f, arm.forearm_connected, _ = self._read_imu(dev_dict, arm.forearm_node_id)
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

                    # 方向→关节角度 (先用 forearm_twist=0 计算 SP, SR, SY, EL)
                    if arm.upper_calibrated:
                        # 第一步: 从方向计算 SP, SR, SY, EL
                        angles_temp = direction_to_joint_angles(
                            arm.arm_dir_chest, arm.forearm_dir_chest,
                            0.0, arm.side,
                            seed=arm.angles)

                        # 第二步: 掌心匹配计算 WR
                        if arm.twist_cal is not None and arm.forearm_tracker.q_rel_inc is not None:
                            prev_wr = arm.angles.wrist_roll if arm.angles is not None else None
                            wr = compute_wrist_roll_from_palm(
                                arm.palm_dir_chest,
                                angles_temp.shoulder_pitch,
                                angles_temp.shoulder_roll,
                                angles_temp.shoulder_yaw,
                                angles_temp.elbow,
                                arm.side,
                                prev_wr)
                        else:
                            wr = 0.0

                        arm.angles = clamp_joint_angles(JointAngles(
                            angles_temp.shoulder_pitch,
                            angles_temp.shoulder_roll,
                            angles_temp.shoulder_yaw,
                            angles_temp.elbow,
                            wr,
                            side=arm.side,
                        ))

                # 采样率统计
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

    # ---------- 校准 (4 步: 下垂/前平举/侧平举/Roll) ----------

    def _get_calib_arm_poses(self):
        """返回当前活跃手臂在 CALIB_POSES 中的 (start_idx, end_idx)."""
        if self.active_arm == "right":
            return 0, 3
        else:
            return 4, 7

    def start_calibration(self):
        """开始 4 步校准序列 - 先提示用户, 等待按 C 确认."""
        if self._calib_step >= 0:
            return  # 已在校准中
        arm = self.active
        arm.reset()
        start_idx, _ = self._get_calib_arm_poses()
        self._calib_step = start_idx
        self._calib_waiting = True  # 等待用户确认
        self._calib_arm = arm
        self._calib_samples_chest = []
        self._calib_samples_upper = []
        self._calib_samples_forearm = []
        pose = CALIB_POSES[self._calib_step]
        self._log(f"校准步骤 1/4: {pose['name']} - 摆好姿势后按 C 开始")

    def confirm_calib_step(self):
        """用户按 C 确认已摆好姿势, 开始采集."""
        if self._calib_step < 0 or not self._calib_waiting:
            return
        self._calib_waiting = False
        self._calib_start = time.monotonic()
        self._calib_samples_chest = []
        self._calib_samples_upper = []
        self._calib_samples_forearm = []
        pose = CALIB_POSES[self._calib_step]
        is_roll_step = pose["dir"] is None
        duration = CALIB_ROLL_DURATION if is_roll_step else CALIB_STATIC_DURATION
        self._log(f"  开始采集 ({duration:.0f}s)...")

    def _collect_calib_samples(self, now):
        """在 lock 内调用: 收集校准样本."""
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
            self._calib_step = next_step
            self._calib_waiting = True  # 等待用户确认
            self._calib_samples_chest = []
            self._calib_samples_upper = []
            self._calib_samples_forearm = []
            pose = CALIB_POSES[self._calib_step]
            step_num = self._calib_step - start_idx + 1
            if pose["dir"] is not None:
                self._log(f"校准步骤 {step_num}/4: {pose['name']} - 摆好姿势后按 C 开始")
            else:
                self._log(f"校准步骤 {step_num}/4: {pose['name']} - 摆好后按 C 开始旋前/旋后")
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

    # ---------- UDP ----------

    def _send_udp(self, mode, right, left):
        if not self.robot_ip:
            return
        pkt = pack_dual_arm_cmd(
            mode,
            right.send_angles.shoulder_pitch, right.send_angles.shoulder_roll,
            right.send_angles.shoulder_yaw, right.send_angles.elbow,
            right.send_angles.wrist_roll,
            left.send_angles.shoulder_pitch, left.send_angles.shoulder_roll,
            left.send_angles.shoulder_yaw, left.send_angles.elbow,
            left.send_angles.wrist_roll,
        )
        try:
            self.sock.sendto(pkt, (self.robot_ip, self.robot_port))
        except OSError:
            pass

    def _update_send_angles(self, arm, target_angles, dt):
        MAX_IK_OUTPUT_SPEED = 3.5
        if self.following and arm.upper_calibrated:
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
        else:
            max_step = RETURN_SPEED * dt
            sp = arm.send_angles.shoulder_pitch
            sr = arm.send_angles.shoulder_roll
            sy = arm.send_angles.shoulder_yaw
            el = arm.send_angles.elbow
            wr = arm.send_angles.wrist_roll
            sp = sp - np.clip(sp, -max_step, max_step)
            sr = sr - np.clip(sr, -max_step, max_step)
            sy = sy - np.clip(sy, -max_step, max_step)
            el = el - np.clip(el, -max_step, max_step)
            wr = wr - np.clip(wr, -max_step, max_step)
            arm.send_angles = clamp_joint_angles(JointAngles(sp, sr, sy, el, wr, side=arm.side))

    # ---------- curses UI ----------

    def _safe_addstr(self, win, y, x, text, attr=0):
        h, w = win.getmaxyx()
        if y < 0 or y >= h or x < 0:
            return
        max_len = w - x - 1
        if max_len <= 0:
            return
        win.addnstr(y, x, text, max_len, attr)

    def _draw_bar(self, win, y, x, width, ratio, color_pair):
        h, w = win.getmaxyx()
        if y >= h or x >= w:
            return
        bar_w = min(width, w - x - 1)
        if bar_w <= 0:
            return
        filled = int(bar_w * max(0.0, min(1.0, ratio)))
        bar = "█" * filled + "░" * (bar_w - filled)
        win.addnstr(y, x, bar, bar_w, color_pair)

    def _render_arm_joints(self, stdscr, y, arm, angles, colors):
        """渲染一个手臂的关节角度"""
        C_GREEN, C_RED, C_MAG = colors
        if arm.upper_calibrated:
            joints = [
                ("SP(pitch)", angles.shoulder_pitch,
                 f"{arm.limit_prefix}shoulder_pitch"),
                ("SR(roll) ", angles.shoulder_roll,
                 f"{arm.limit_prefix}shoulder_roll"),
                ("SY(yaw)  ", angles.shoulder_yaw,
                 f"{arm.limit_prefix}shoulder_yaw"),
                ("EL(elbow)", angles.elbow,
                 f"{arm.limit_prefix}elbow"),
                ("WR(wrist)", angles.wrist_roll,
                 f"{arm.limit_prefix}wrist_roll"),
            ]
            for name, val, key in joints:
                limits = arm.joint_limits[key]
                in_limit = limits[0] <= val <= limits[1]
                c = C_GREEN if in_limit else C_RED
                deg = np.degrees(val)
                rng = limits[1] - limits[0]
                ratio = (val - limits[0]) / rng if rng > 0 else 0.5
                idx = arm.motor_index[key]
                line = f"  [{idx:2d}] {name}: {val:+7.3f} rad ({deg:+7.1f}deg)"
                self._safe_addstr(stdscr, y, 0, line, c)
                self._draw_bar(stdscr, y, 48, 15, ratio, c)
                y += 1
        else:
            self._safe_addstr(stdscr, y, 0, "  (需要校准后显示)", curses.A_DIM)
            y += 1
        return y

    def _render_tui(self, stdscr):
        """主 curses 渲染"""
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(33)

        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN, -1)
        curses.init_pair(2, curses.COLOR_RED, -1)
        curses.init_pair(3, curses.COLOR_YELLOW, -1)
        curses.init_pair(4, curses.COLOR_CYAN, -1)
        curses.init_pair(5, curses.COLOR_MAGENTA, -1)
        curses.init_pair(6, curses.COLOR_WHITE, -1)

        C_GREEN = curses.color_pair(1) | curses.A_BOLD
        C_RED = curses.color_pair(2) | curses.A_BOLD
        C_YELLOW = curses.color_pair(3) | curses.A_BOLD
        C_CYAN = curses.color_pair(4) | curses.A_BOLD
        C_MAG = curses.color_pair(5)
        C_NORM = curses.color_pair(6)
        C_DIM = curses.A_DIM

        self._load_calibration()
        self._init_imu()
        self._start_ik_thread()
        self._start_discovery()

        diag_path = self._diag_logger.start()
        self._log(f"诊断日志: {os.path.basename(diag_path)}")

        running = True
        last_send = time.monotonic()
        last_frame = time.monotonic()
        frame_count = 0
        fps = 0.0
        fps_timer = time.monotonic()

        self._log("系统已启动 (双臂模式)")

        while running:
            now = time.monotonic()
            dt = now - last_frame
            if dt <= 0:
                dt = 0.033
            last_frame = now

            frame_count += 1
            if now - fps_timer >= 1.0:
                fps = frame_count / (now - fps_timer)
                frame_count = 0
                fps_timer = now

            # --- 按键处理 ---
            try:
                key = stdscr.getch()
            except Exception:
                key = -1

            if key == ord('q') or key == ord('Q') or key == 27:
                running = False
                continue
            elif key == ord('r') or key == ord('R'):
                self.active_arm = "right"
                self._log("切换到右臂控制")
                self._diag_logger.add_event("SWITCH_RIGHT")
            elif key == ord('l') or key == ord('L'):
                self.active_arm = "left"
                self._log("切换到左臂控制")
                self._diag_logger.add_event("SWITCH_LEFT")
            elif key == ord('c') or key == ord('C'):
                if self._calib_step < 0:
                    # 未在校准 → 开始校准
                    self.start_calibration()
                elif self._calib_waiting:
                    # 等待确认 → 开始采集
                    self.confirm_calib_step()
            elif key == ord('x') or key == ord('X'):
                with self.lock:
                    self.right.reset()
                    self.left.reset()
                    self._calib_step = -1
                    self._pose_averages = []
                self._log("校准已重置")
            elif key == ord(' '):
                self.following = not self.following
                if self.following:
                    self._log("跟随开启")
                    self._diag_logger.add_event("FOLLOW_ON")
                else:
                    self._log("跟随关闭")
                    self._diag_logger.add_event("FOLLOW_OFF")

            # --- 方向→角度已内联到 _wireless_loop ---

            # --- 更新发送角度 ---
            self._update_send_angles(self.right, self.right.angles, dt)
            self._update_send_angles(self.left, self.left.angles, dt)

            # --- UDP + 诊断 ---
            if now - last_send >= UDP_SEND_DT:
                if self.following:
                    if self.active_arm == "right":
                        mode = 1
                    else:
                        mode = 2
                else:
                    mode = 0
                self._send_udp(mode, self.right, self.left)
                last_send = now

                self._diag_logger.log(
                    mode=mode,
                    active_arm=self.active_arm,
                    right=self.right,
                    left=self.left,
                )

            # ===== 渲染 TUI =====
            try:
                stdscr.erase()
                h, w = stdscr.getmaxyx()
                if h < 10 or w < 40:
                    self._safe_addstr(stdscr, 0, 0, "终端太小,请调大窗口", C_RED)
                    stdscr.refresh()
                    continue

                y = 0

                # 标题栏
                title = " 宇树G1 双臂遥操控 (树莓派无头版) "
                self._safe_addstr(stdscr, y, 0, "=" * min(w-1, 70), C_CYAN)
                self._safe_addstr(stdscr, y, max(0, (min(w, 70) - len(title)) // 2), title, C_CYAN)
                y += 1

                # 状态总览
                arm = self.active
                arm_label = "右臂" if self.active_arm == "right" else "左臂"
                if self._calib_step >= 0:
                    pose = CALIB_POSES[self._calib_step]
                    start_idx, _ = self._get_calib_arm_poses()
                    step_num = self._calib_step - start_idx + 1
                    if self._calib_waiting:
                        status = f">>> {arm_label}校准 {step_num}/4: {pose['name']} - 按 C 开始 <<<"
                        self._safe_addstr(stdscr, y, 0, status, C_YELLOW)
                    else:
                        remaining = max(0, (CALIB_ROLL_DURATION if pose["dir"] is None else CALIB_STATIC_DURATION)
                                        - (time.monotonic() - self._calib_start))
                        status = f">>> {arm_label}校准 {step_num}/4: {pose['name']} 采集中 {remaining:.1f}s <<<"
                        self._safe_addstr(stdscr, y, 0, status, C_CYAN)
                elif arm.upper_calibrated:
                    self._safe_addstr(stdscr, y, 0, f"● {arm_label}已校准", C_GREEN)
                else:
                    self._safe_addstr(stdscr, y, 0, f"○ {arm_label}未校准 (按 C 开始4步校准)", C_RED)

                if self.following:
                    self._safe_addstr(stdscr, y, 30, f"▶ {arm_label}跟随中", C_GREEN)
                else:
                    self._safe_addstr(stdscr, y, 30, "■ 停止", C_RED)

                self._safe_addstr(stdscr, y, 45, f"FPS:{fps:.0f}", C_DIM)
                y += 1

                # 连接信息
                self._safe_addstr(stdscr, y, 0, "-" * min(w-1, 70), C_DIM)
                y += 1

                self._safe_addstr(stdscr, y, 0, "目标:", C_NORM)
                if self.robot_discovered:
                    self._safe_addstr(stdscr, y, 6, f"{self.robot_ip}:{self.robot_port}", C_GREEN)
                elif self.robot_ip:
                    self._safe_addstr(stdscr, y, 6, f"{self.robot_ip}:{self.robot_port}", C_CYAN)
                else:
                    self._safe_addstr(stdscr, y, 6, "搜索中...", C_YELLOW)

                with self.lock:
                    rate = self.sample_rate
                    dev_count = len(self.latest_devices)
                    devices = list(self.latest_devices)
                    r_up = self.right.upper_connected
                    r_fo = self.right.forearm_connected
                    l_up = self.left.upper_connected
                    l_fo = self.left.forearm_connected
                    chest_conn = self.chest_connected
                    r_un = self.right.upper_node_id or "未绑定"
                    r_fn = self.right.forearm_node_id or "未绑定"
                    l_un = self.left.upper_node_id or "未绑定"
                    l_fn = self.left.forearm_node_id or "未绑定"
                    c_un = self.chest_node_id or "未绑定"

                self._safe_addstr(stdscr, y, 35, f"IMU:{rate:.0f}Hz", C_NORM)
                y += 1

                # IMU 设备
                self._safe_addstr(stdscr, y, 0, f"--- 无线 IMU ({dev_count}台) ---", C_CYAN)
                y += 1

                def _show_imu(label, connected, node_id, y_pos):
                    c = C_GREEN if connected else C_RED
                    s = "在线" if connected else "离线"
                    self._safe_addstr(stdscr, y_pos, 0, f"  {label}: {node_id} [{s}]", c)
                    for d in devices:
                        if d.node_id == node_id and d.battery_percent is not None:
                            self._safe_addstr(stdscr, y_pos, 38, f"{d.battery_percent}%", C_NORM)
                    return y_pos + 1

                y = _show_imu("胸部", chest_conn, c_un, y)
                y = _show_imu("右大臂", r_up, r_un, y)
                y = _show_imu("右小臂", r_fo, r_fn, y)
                y = _show_imu("左大臂", l_up, l_un, y)
                y = _show_imu("左小臂", l_fo, l_fn, y)

                # 活跃手臂关节角度
                self._safe_addstr(stdscr, y, 0, f"--- {arm_label}关节角度 (IK) ---", C_CYAN)
                y += 1
                y = self._render_arm_joints(stdscr, y, arm, arm.angles, (C_GREEN, C_RED, C_MAG))

                # 发送角度
                self._safe_addstr(stdscr, y, 0, f"--- 发送角度 ---", C_CYAN)
                y += 1
                for label, arm_obj in [("右臂", self.right), ("左臂", self.left)]:
                    sa = arm_obj.send_angles
                    deg_sp = np.degrees(sa.shoulder_pitch)
                    deg_sr = np.degrees(sa.shoulder_roll)
                    deg_sy = np.degrees(sa.shoulder_yaw)
                    deg_el = np.degrees(sa.elbow)
                    c = C_GREEN if self.following else C_DIM
                    self._safe_addstr(stdscr, y, 0,
                        f"  {label}: SP={sa.shoulder_pitch:+6.2f}({deg_sp:+6.1f}) "
                        f"SR={sa.shoulder_roll:+6.2f}({deg_sr:+6.1f}) "
                        f"EL={sa.elbow:+6.2f}({deg_el:+6.1f})", c)
                    y += 1

                # 方向信息
                if arm.upper_calibrated:
                    disp = T_CHEST2DISP @ arm.arm_dir_chest
                    self._safe_addstr(stdscr, y, 0,
                        f"  方向: [{disp[0]:+.2f} {disp[1]:+.2f} {disp[2]:+.2f}] "
                        f"twist={np.degrees(arm.forearm_twist):+.1f}deg", C_DIM)
                    y += 1

                # 日志
                y += 1
                log_max_lines = h - y - 2
                if log_max_lines > 0:
                    self._safe_addstr(stdscr, y, 0, "--- 日志 ---", C_CYAN)
                    y += 1
                    for line in self._log_lines[-(log_max_lines):]:
                        self._safe_addstr(stdscr, y, 0, line, C_DIM)
                        y += 1
                        if y >= h - 1:
                            break

                help_line = "R:右臂 L:左臂 SPACE:跟随 C:校准/确认 X:重置 Q:退出"
                self._safe_addstr(stdscr, h - 1, 0, help_line, C_CYAN | curses.A_REVERSE)

                stdscr.refresh()

            except curses.error:
                pass

        # 清理: 发送停止
        for _ in range(10):
            self._send_udp(0, self.right, self.left)
            time.sleep(0.02)

        self._diag_logger.stop()
        self._log(f"诊断日志已保存 ({self._diag_logger.rows} 条)")
        self._stop_ik_thread()
        self._stop_discovery()
        self._stop_imu()
        self.sock.close()

    def run(self):
        curses.wrapper(self._render_tui)


def main():
    robot_ip = DEFAULT_ROBOT_IP
    robot_port = DEFAULT_ROBOT_PORT

    if len(sys.argv) >= 2:
        robot_ip = sys.argv[1]
    if len(sys.argv) >= 3:
        robot_port = int(sys.argv[2])

    sender = HeadlessArmSender(robot_ip, robot_port)
    try:
        sender.run()
    except KeyboardInterrupt:
        pass
    print("程序已退出")


if __name__ == "__main__":
    main()
