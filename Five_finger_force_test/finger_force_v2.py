#!/usr/bin/env python3
"""
五指力反馈控制 v2 — 简化状态机版本
控制逻辑（每根手指独立）：
  1. 收到目标力 → 解算为驱动电流 → 舵机拉紧绳索（DRIVING）
  2. 位置不再变化（速度≈0）→ 进入锁紧状态（LOCKED），切换到位置保持模式
  3. 目标力归零 → 退出锁紧，舵机回归初始位置（RELEASING → IDLE）

硬件：5× Dynamixel XL330-M288T @ /dev/ttyAMA0 1Mbps
BLE数据：通过 ble_broker.py TCP 9001 接收，格式同原 finger_force.py
"""

import sys
import time
import threading
import socket
import json
import re
import asyncio
import signal
from typing import List, Tuple

from dynamixel_sdk import *
from bleak import BleakClient

# =====================================================================
# ======================== 用户可调参数区 =============================
# =====================================================================

# ===== 手指名称映射 =====
FINGER_NAMES = {1: "大拇指", 2: "食指", 3: "中指", 4: "无名指", 5: "小指"}

# ===== BLE Broker 配置 =====
USE_BROKER = True
BROKER_HOST = "127.0.0.1"
BROKER_PORT = 9001

# ===== BLE 直连配置（USE_BROKER=False 时使用）=====
DEVICE_ADDRESSES = [
    "F0:FD:45:02:85:B3",
    "F0:FD:45:02:67:3B",
]
TX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
RX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
BLE_KEEPALIVE_INTERVAL = 3.0
BLE_KEEPALIVE_DATA = b"PING"
BLE_DATA_TIMEOUT = 15.0
BLE_RECONNECT_DELAY = 1.0

# ===== BLE 力零偏校准 =====
# 固件在"未施力"时发送约 4.903N，减去此基线后才是真实力值
BLE_FORCE_BASELINE = 4.903

# ===== 力→电流转换 =====
# 驱动电流(mA) = 目标力(N) × FORCE_TO_CURRENT_GAIN
# 根据实际舵机+绳索机械比标定此值
FORCE_TO_CURRENT_GAIN = 60.0    # mA/N，默认值，可通过 CAL:ID:力值 标定
MAX_DRIVE_CURRENT_MA = 250.0    # 单舵机最大驱动电流(mA)
TOTAL_CURRENT_LIMIT_MA = 800.0  # 五指总电流上限(mA)，防止电源过载

# ===== 位置锁定检测 =====
# 连续 LOCK_DETECT_WINDOW 秒内位置变化量 ≤ LOCK_POSITION_THRESHOLD → 判定锁紧
LOCK_POSITION_THRESHOLD = 8     # 位置变化阈值(tick)
LOCK_DETECT_WINDOW = 0.4        # 检测窗口(秒)
LOCK_MIN_DRIVE_TIME = 0.3       # 最短驱动时间(秒)，防止刚启动就误判锁紧

# ===== 锁紧保持电流 =====
# 进入 LOCKED 后切换到位置控制模式，以下参数用于位置保持
LOCK_PROFILE_VELOCITY = 0       # 0=最快响应
LOCK_PROFILE_ACCELERATION = 0

# ===== 主动释放参数 =====
RELEASE_CURRENT_MA = -150.0     # 释放驱动电流(负值=松绳方向)
RELEASE_POSITION_THRESHOLD = 15 # 归位判定阈值(tick)
RELEASE_TIMEOUT = 4.0           # 释放超时(秒)
RELEASE_SLOWDOWN_RANGE = 50     # 减速区间(tick)，接近目标时降低电流
RELEASE_MIN_TIME = 0.3          # 释放最短运行时间(秒)
RELEASE_MIN_CURRENT = -30.0     # 释放最小电流，防止distance≈0时停住

# ===== 硬件串口 & 舵机参数 =====
DEVICENAME = "/dev/ttyAMA0"
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
DXL_IDS = [1, 2, 3, 4, 5]
NUM_FINGERS = 5
CURRENT_LIMIT_PER_SERVO_MA = 300

# XL330 Control Table Addresses
ADDR_OPERATING_MODE      = 11
ADDR_CURRENT_LIMIT       = 38
ADDR_TORQUE_ENABLE       = 64
ADDR_GOAL_CURRENT        = 102
ADDR_PROFILE_ACCELERATION= 108
ADDR_PROFILE_VELOCITY    = 112
ADDR_GOAL_POSITION       = 116
ADDR_PRESENT_CURRENT     = 126
ADDR_PRESENT_POSITION    = 132

OP_CURRENT_BASED_POSITION = 5
OP_CURRENT_CONTROL        = 0

# ===== 调试 =====
DEBUG_PRINT_INTERVAL = 2.0   # 状态打印间隔(秒)
HW_ERROR_PRINT_INTERVAL = 3.0

# =====================================================================
# ======================== 全局变量 ===================================
# =====================================================================

_hw_error_last_print = {}
running = True
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
dxl_lock = threading.Lock()


# =====================================================================
# ======================== 状态机定义 =================================
# =====================================================================

class FingerState:
    IDLE      = "IDLE"       # 空闲，等待目标力
    DRIVING   = "DRIVING"    # 施加驱动电流，拉紧绳索
    LOCKED    = "LOCKED"     # 位置锁紧，保持位置
    RELEASING = "RELEASING"  # 归位中


class ServoController:
    """单舵机/手指控制器"""

    def __init__(self, servo_id: int):
        self.servo_id     = servo_id
        self.finger_name  = FINGER_NAMES.get(servo_id, f"手指{servo_id}")

        # 状态
        self.state        = FingerState.IDLE
        self.target_force = 0.0          # 当前目标力(N)，已去除基线

        # 位置记录
        self.init_pos     = None         # 初始化时记录的位置
        self.is_initialized = False
        self.drive_start_time = 0.0      # 进入 DRIVING 的时刻
        self.pos_history  = []           # (time, pos) 用于锁紧检测

        # 释放状态
        self.release_start_time = 0.0

        # 标定系数 (mA/N)
        self.force_to_current_gain = FORCE_TO_CURRENT_GAIN

        # 供总电流限制使用的暂存
        self._pending_current = 0.0

        # 上次打印电流（减少重复读取）
        self.last_print_current = 0


# =====================================================================
# ======================== BLE 数据存储 ================================
# =====================================================================

class BLEDataStore:
    """线程安全的 BLE 目标力存储：[拇指, 食指, 中指, 无名指, 小指] (N, 已去基线)"""

    def __init__(self):
        self._lock = threading.Lock()
        self._values = [0.0] * NUM_FINGERS
        self._last_update = 0.0

    def update_raw(self, raw_values: List[float]) -> None:
        """传入原始力值列表，自动减去基线并截断到0"""
        calibrated = []
        for v in raw_values[:NUM_FINGERS]:
            c = v - BLE_FORCE_BASELINE
            calibrated.append(max(0.0, c))
        with self._lock:
            for i in range(len(calibrated)):
                self._values[i] = calibrated[i]
            self._last_update = time.time()

    def get(self) -> Tuple[List[float], float]:
        """返回 (values, age_seconds)"""
        with self._lock:
            v = list(self._values)
            t = self._last_update
        age = time.time() - t if t > 0 else float("inf")
        return v, age


ble_store = BLEDataStore()


# =====================================================================
# ======================== JSON 解析工具 ==============================
# =====================================================================

class JsonStreamParser:
    def __init__(self):
        self._buf = ""
        self._dec = json.JSONDecoder()

    def feed(self, text: str) -> List[dict]:
        self._buf += text
        out = []
        while self._buf:
            self._buf = self._buf.lstrip()
            if not self._buf:
                break
            if not (self._buf.startswith("{") or self._buf.startswith("[")):
                idx = self._buf.find("{")
                if idx == -1:
                    self._buf = ""
                    break
                self._buf = self._buf[idx:]
                continue
            try:
                obj, end = self._dec.raw_decode(self._buf)
                self._buf = self._buf[end:]
                if isinstance(obj, list):
                    obj = {"touch_sensors": obj}
                if isinstance(obj, dict):
                    out.append(obj)
            except json.JSONDecodeError:
                if "\n" in self._buf:
                    line, self._buf = self._buf.split("\n", 1)
                    try:
                        o = json.loads(line.strip())
                        if isinstance(o, list):
                            o = {"touch_sensors": o}
                        if isinstance(o, dict):
                            out.append(o)
                    except Exception:
                        pass
                else:
                    break
        return out


def extract_from_payload(payload: dict) -> List[float]:
    """从解析后的 dict 提取力值列表"""
    sensors = payload.get("touch_sensors") or payload.get("force")
    if isinstance(sensors, (int, float)):
        sensors = [sensors]
    if isinstance(sensors, list):
        try:
            return [float(v) for v in sensors]
        except (TypeError, ValueError):
            pass
    return []


def extract_from_raw_text(text: str) -> List[float]:
    """兜底：从原始文本提取花括号/方括号数值，取最后5位"""
    for pattern in [r'\{([0-9.,\s]+)\}', r'\[([0-9.,\s]+)\]']:
        for m in re.findall(pattern, text):
            parts = [p.strip() for p in m.split(",") if p.strip()]
            try:
                vals = [float(p) for p in parts]
                if len(vals) >= 5:
                    return vals[-5:]
            except ValueError:
                pass
    return []


# =====================================================================
# ======================== 舵机底层 I/O ================================
# =====================================================================

def _hw_err(servo_id: int, msg: str) -> None:
    now = time.time()
    if now - _hw_error_last_print.get(servo_id, 0) >= HW_ERROR_PRINT_INTERVAL:
        print(msg)
        _hw_error_last_print[servo_id] = now


def read4Signed(sid: int, addr: int) -> int:
    with dxl_lock:
        v, r, e = packetHandler.read4ByteTxRx(portHandler, sid, addr)
    if r != COMM_SUCCESS:
        _hw_err(sid, f"[舵机{sid}] 读取失败: {packetHandler.getTxRxResult(r)}")
    elif e != 0:
        _hw_err(sid, f"[舵机{sid}] 硬件错误: {packetHandler.getRxPacketError(e)}")
    return v - 4294967296 if v > 2147483647 else v


def read2Signed(sid: int, addr: int) -> int:
    with dxl_lock:
        v, r, e = packetHandler.read2ByteTxRx(portHandler, sid, addr)
    if r != COMM_SUCCESS:
        _hw_err(sid, f"[舵机{sid}] 读取失败: {packetHandler.getTxRxResult(r)}")
    return v - 65536 if v > 32767 else v


def write1(sid: int, addr: int, val: int) -> None:
    with dxl_lock:
        packetHandler.write1ByteTxRx(portHandler, sid, addr, val)


def write2(sid: int, addr: int, val: int) -> None:
    with dxl_lock:
        packetHandler.write2ByteTxRx(portHandler, sid, addr, val)


def write2s(sid: int, addr: int, val: int) -> None:
    if val < 0:
        val = (1 << 16) + val
    write2(sid, addr, val)


def write4(sid: int, addr: int, val: int) -> None:
    with dxl_lock:
        packetHandler.write4ByteTxRx(portHandler, sid, addr, int(val))


def torque_on(sid: int) -> None:
    write1(sid, ADDR_TORQUE_ENABLE, 1)


def torque_off(sid: int) -> None:
    write1(sid, ADDR_TORQUE_ENABLE, 0)


def set_mode(sid: int, mode: int) -> None:
    torque_off(sid)
    write1(sid, ADDR_OPERATING_MODE, mode)
    torque_on(sid)


def write_current(sid: int, mA: float) -> None:
    write2s(sid, ADDR_GOAL_CURRENT, int(round(mA)))


def write_position(sid: int, pos: int) -> None:
    write4(sid, ADDR_GOAL_POSITION, pos)


# =====================================================================
# ======================== 硬件初始化 ==================================
# =====================================================================

servo_controllers: List[ServoController] = [ServoController(i) for i in DXL_IDS]


def setup() -> None:
    if not portHandler.openPort():
        print("[错误] 串口打开失败")
        sys.exit(1)
    if not portHandler.setBaudRate(BAUDRATE):
        print("[错误] 波特率设置失败")
        sys.exit(1)
    print(f"[硬件] 串口 {DEVICENAME} @ {BAUDRATE}bps 已开启")

    for ctrl in servo_controllers:
        sid = ctrl.servo_id
        _, r, _ = packetHandler.ping(portHandler, sid)
        if r != COMM_SUCCESS:
            print(f"[舵机{sid}/{ctrl.finger_name}] Ping 失败，跳过")
            continue
        print(f"[舵机{sid}/{ctrl.finger_name}] Ping 成功")
        torque_off(sid)
        write1(sid, ADDR_OPERATING_MODE, OP_CURRENT_CONTROL)
        write2(sid, ADDR_CURRENT_LIMIT, CURRENT_LIMIT_PER_SERVO_MA)
        write4(sid, ADDR_PROFILE_VELOCITY, 100)
        write4(sid, ADDR_PROFILE_ACCELERATION, 50)
        torque_on(sid)
        time.sleep(0.05)

    print("\n===== 五指力反馈 v2 就绪 =====")
    print("手指映射: 1=大拇指 2=食指 3=中指 4=无名指 5=小指")
    print("输入 INIT 记录初始位置，HELP 查看指令")


# =====================================================================
# ======================== 主控制循环 ==================================
# =====================================================================

def loop() -> None:
    last_print = time.time()

    while running:
        try:
            now = time.time()
            ble_forces, ble_age = ble_store.get()
            ble_ok = ble_age < 2.0

            for idx, ctrl in enumerate(servo_controllers):
                sid = ctrl.servo_id

                # ---- 更新目标力 ----
                if ble_ok:
                    ctrl.target_force = ble_forces[idx] if idx < len(ble_forces) else 0.0
                elif ble_age > 5.0:
                    # BLE 超时超过 5s → 强制归零
                    ctrl.target_force = 0.0

                _tick_finger(ctrl, now)

            # ---- 五指总电流限制 ----
            total_pos = sum(max(0.0, c._pending_current) for c in servo_controllers)
            if total_pos > TOTAL_CURRENT_LIMIT_MA and total_pos > 0:
                scale = TOTAL_CURRENT_LIMIT_MA / total_pos
                for c in servo_controllers:
                    if c._pending_current > 0:
                        c._pending_current *= scale

            # ---- 统一写入电流（DRIVING 状态舵机）----
            for ctrl in servo_controllers:
                if ctrl.state == FingerState.DRIVING:
                    write_current(ctrl.servo_id, ctrl._pending_current)
                elif ctrl.state == FingerState.RELEASING:
                    write_current(ctrl.servo_id, ctrl._pending_current)

            # ---- 状态打印 ----
            if now - last_print >= DEBUG_PRINT_INTERVAL:
                _print_status()
                last_print = now

            time.sleep(0.005)  # 5ms 控制周期

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"[循环错误] {e}")


def _tick_finger(ctrl: ServoController, now: float) -> None:
    """每个控制周期对单个手指执行状态机转换"""
    ctrl._pending_current = 0.0
    sid = ctrl.servo_id

    # ================================================================
    if ctrl.state == FingerState.IDLE:
        # 有目标力且已初始化 → DRIVING
        if ctrl.target_force > 0.0 and ctrl.is_initialized:
            _enter_driving(ctrl, now)
        elif ctrl.target_force > 0.0 and not ctrl.is_initialized:
            print(f"[舵机{sid}/{ctrl.finger_name}] 未初始化，请先输入 INIT")

    # ================================================================
    elif ctrl.state == FingerState.DRIVING:
        # 目标力归零 → 归位
        if ctrl.target_force <= 0.0:
            _enter_releasing(ctrl, now)
            return

        # 计算驱动电流
        drive_mA = min(ctrl.target_force * ctrl.force_to_current_gain,
                       MAX_DRIVE_CURRENT_MA)
        ctrl._pending_current = drive_mA

        # 锁紧检测
        pos = read4Signed(sid, ADDR_PRESENT_POSITION)
        _update_pos_history(ctrl, now, pos)
        if _is_locked(ctrl, now):
            _enter_locked(ctrl, pos)

    # ================================================================
    elif ctrl.state == FingerState.LOCKED:
        # 目标力归零 → 归位
        if ctrl.target_force <= 0.0:
            _enter_releasing(ctrl, now)
            return
        # 目标力变化时，若超出当前位置控制能力可选择重新进入 DRIVING
        # 此版本：保持位置不变，等待力归零
        # （位置保持由舵机内部位置环实现，无需额外电流命令）

    # ================================================================
    elif ctrl.state == FingerState.RELEASING:
        _tick_releasing(ctrl, now)


# ------------------------------------------------------------------ #
# 状态转换辅助函数
# ------------------------------------------------------------------ #

def _enter_driving(ctrl: ServoController, now: float) -> None:
    set_mode(ctrl.servo_id, OP_CURRENT_CONTROL)
    write2(ctrl.servo_id, ADDR_CURRENT_LIMIT, CURRENT_LIMIT_PER_SERVO_MA)
    ctrl.state = FingerState.DRIVING
    ctrl.drive_start_time = now
    ctrl.pos_history = []
    print(f"[舵机{ctrl.servo_id}/{ctrl.finger_name}] → DRIVING  目标力={ctrl.target_force:.2f}N")


def _enter_locked(ctrl: ServoController, pos: int) -> None:
    """切换到位置控制模式，锁死在当前位置"""
    sid = ctrl.servo_id
    write_current(sid, 0)          # 先清零电流
    time.sleep(0.02)
    torque_off(sid)
    write1(sid, ADDR_OPERATING_MODE, OP_CURRENT_BASED_POSITION)
    write2(sid, ADDR_CURRENT_LIMIT, CURRENT_LIMIT_PER_SERVO_MA)
    write4(sid, ADDR_PROFILE_VELOCITY, LOCK_PROFILE_VELOCITY)
    write4(sid, ADDR_PROFILE_ACCELERATION, LOCK_PROFILE_ACCELERATION)
    torque_on(sid)
    write_position(sid, pos)       # 保持当前位置
    ctrl.state = FingerState.LOCKED
    print(f"[舵机{sid}/{ctrl.finger_name}] → LOCKED   锁定位置={pos}")


def _enter_releasing(ctrl: ServoController, now: float) -> None:
    if not ctrl.is_initialized or ctrl.init_pos is None:
        # 未初始化则直接清零电流回 IDLE
        if ctrl.state == FingerState.DRIVING:
            write_current(ctrl.servo_id, 0)
        ctrl.state = FingerState.IDLE
        ctrl.target_force = 0.0
        print(f"[舵机{ctrl.servo_id}/{ctrl.finger_name}] → IDLE (未初始化)")
        return

    # 切换到电流控制模式驱动归位
    set_mode(ctrl.servo_id, OP_CURRENT_CONTROL)
    write2(ctrl.servo_id, ADDR_CURRENT_LIMIT, CURRENT_LIMIT_PER_SERVO_MA)
    ctrl.state = FingerState.RELEASING
    ctrl.release_start_time = now
    ctrl.target_force = 0.0
    print(f"[舵机{ctrl.servo_id}/{ctrl.finger_name}] → RELEASING 归位到 {ctrl.init_pos}")


def _tick_releasing(ctrl: ServoController, now: float) -> None:
    sid = ctrl.servo_id
    pos = read4Signed(sid, ADDR_PRESENT_POSITION)
    distance = pos - ctrl.init_pos
    elapsed = now - ctrl.release_start_time

    # 到达初始位置 or 超时
    if elapsed >= RELEASE_MIN_TIME and (abs(distance) <= RELEASE_POSITION_THRESHOLD
                                         or elapsed > RELEASE_TIMEOUT):
        write_current(sid, 0)
        time.sleep(0.02)
        # 切回位置控制模式，写入 init_pos 让舵机稳定
        torque_off(sid)
        write1(sid, ADDR_OPERATING_MODE, OP_CURRENT_BASED_POSITION)
        write2(sid, ADDR_CURRENT_LIMIT, CURRENT_LIMIT_PER_SERVO_MA)
        write4(sid, ADDR_PROFILE_VELOCITY, 50)
        write4(sid, ADDR_PROFILE_ACCELERATION, 30)
        torque_on(sid)
        write_position(sid, ctrl.init_pos)
        ctrl.state = FingerState.IDLE
        reason = "到达初始位置" if abs(distance) <= RELEASE_POSITION_THRESHOLD \
                                 else f"超时({elapsed:.1f}s)"
        print(f"[舵机{sid}/{ctrl.finger_name}] → IDLE   释放完成 ({reason})")
        return

    # 计算释放电流（接近目标时降速）
    if abs(distance) < RELEASE_SLOWDOWN_RANGE and abs(distance) > 0:
        ratio = abs(distance) / RELEASE_SLOWDOWN_RANGE
        mA = RELEASE_CURRENT_MA * max(ratio, 0.2)
    else:
        mA = RELEASE_CURRENT_MA

    if abs(mA) < abs(RELEASE_MIN_CURRENT):
        mA = RELEASE_MIN_CURRENT

    if distance < 0:
        mA = -mA   # 若初始位置在当前位置正方向则反转

    ctrl._pending_current = mA


# ------------------------------------------------------------------ #
# 位置锁紧检测
# ------------------------------------------------------------------ #

def _update_pos_history(ctrl: ServoController, now: float, pos: int) -> None:
    ctrl.pos_history.append((now, pos))
    # 只保留 LOCK_DETECT_WINDOW 窗口内的历史
    cutoff = now - LOCK_DETECT_WINDOW * 2
    ctrl.pos_history = [(t, p) for t, p in ctrl.pos_history if t >= cutoff]


def _is_locked(ctrl: ServoController, now: float) -> bool:
    """在 LOCK_DETECT_WINDOW 窗口内位置变化量 ≤ 阈值 → 判定锁紧"""
    if now - ctrl.drive_start_time < LOCK_MIN_DRIVE_TIME:
        return False
    window = [p for t, p in ctrl.pos_history if now - t <= LOCK_DETECT_WINDOW]
    if len(window) < 3:
        return False
    return (max(window) - min(window)) <= LOCK_POSITION_THRESHOLD


# ------------------------------------------------------------------ #
# 状态打印
# ------------------------------------------------------------------ #

def _print_status() -> None:
    active = [c for c in servo_controllers if c.state != FingerState.IDLE]
    if not active:
        return
    parts = []
    for c in active:
        cur = read2Signed(c.servo_id, ADDR_PRESENT_CURRENT)
        pos = read4Signed(c.servo_id, ADDR_PRESENT_POSITION)
        parts.append(
            f"{c.finger_name}[{c.state}] T={c.target_force:.1f}N "
            f"pos={pos} I={cur}mA"
        )
    print(" | ".join(parts))


# =====================================================================
# ======================== 命令解析 ===================================
# =====================================================================

def parse_command(raw: str) -> None:
    global running
    buf = raw.replace("：", ":").strip().upper()

    try:
        # ---- INIT ----
        if buf == "INIT":
            print("\n=== 初始化所有手指 ===")
            for ctrl in servo_controllers:
                pos = read4Signed(ctrl.servo_id, ADDR_PRESENT_POSITION)
                ctrl.init_pos = pos
                ctrl.is_initialized = True
                print(f"  {ctrl.finger_name}: 初始位置={pos}")
            print("初始化完成！目标力=0 时将归回此位置")

        elif buf.startswith("INIT:"):
            idx = int(buf[5:]) - 1
            if 0 <= idx < len(servo_controllers):
                ctrl = servo_controllers[idx]
                pos = read4Signed(ctrl.servo_id, ADDR_PRESENT_POSITION)
                ctrl.init_pos = pos
                ctrl.is_initialized = True
                print(f"[{ctrl.finger_name}] 初始位置={pos}")

        # ---- 手动设置目标力（测试用，正常由 BLE 驱动）----
        elif buf.startswith("N:"):
            parts = buf.split(":")
            if len(parts) >= 3:
                idx = int(parts[1]) - 1
                force = float(parts[2])
                if 0 <= idx < len(servo_controllers):
                    ctrl = servo_controllers[idx]
                    # 模拟 BLE 数据更新
                    values = list(ble_store.get()[0])
                    # 手动命令不经过基线处理，直接写入
                    with ble_store._lock:
                        ble_store._values[idx] = max(0.0, force)
                        ble_store._last_update = time.time()
                    print(f"[{ctrl.finger_name}] 手动目标力={force:.2f}N")

        elif buf.startswith("NALL:"):
            force = float(buf[5:])
            with ble_store._lock:
                ble_store._values = [max(0.0, force)] * NUM_FINGERS
                ble_store._last_update = time.time()
            print(f"[全部] 手动目标力={force:.2f}N")

        # ---- 标定力→电流增益 ----
        elif buf.startswith("CAL:"):
            parts = buf.split(":")
            if len(parts) >= 3:
                idx = int(parts[1]) - 1
                real_N = float(parts[2])
                if 0 <= idx < len(servo_controllers):
                    ctrl = servo_controllers[idx]
                    cur = read2Signed(ctrl.servo_id, ADDR_PRESENT_CURRENT)
                    if abs(cur) >= 5 and abs(real_N) > 0:
                        new_gain = abs(cur) / abs(real_N)
                        old = ctrl.force_to_current_gain
                        ctrl.force_to_current_gain = new_gain
                        print(f"[{ctrl.finger_name}] 增益 {old:.1f} → {new_gain:.1f} mA/N")
                    else:
                        print(f"[{ctrl.finger_name}] 电流太小或力值无效")

        # ---- STATUS ----
        elif buf == "STATUS":
            forces, age = ble_store.get()
            print(f"\n=== 五指力反馈 v2 状态 (BLE age={age:.1f}s) ===")
            print(f"  BLE目标力: {[f'{v:.2f}N' for v in forces]}")
            for ctrl in servo_controllers:
                pos = read4Signed(ctrl.servo_id, ADDR_PRESENT_POSITION)
                cur = read2Signed(ctrl.servo_id, ADDR_PRESENT_CURRENT)
                init_s = f" init={ctrl.init_pos}" if ctrl.is_initialized else " (未初始化)"
                print(f"  {ctrl.finger_name}: [{ctrl.state}] pos={pos} I={cur}mA "
                      f"T={ctrl.target_force:.2f}N{init_s}")

        # ---- HELP ----
        elif buf == "HELP":
            print("""
=== 五指力反馈 v2 指令 ===
INIT           - 记录所有手指初始位置
INIT:ID        - 记录单个手指初始位置 (ID=1~5)
N:ID:力值      - 手动设置目标力(N) (例: N:1:3)
NALL:力值      - 手动设置所有手指目标力
CAL:ID:力值   - 用实测力标定力→电流增益
STATUS         - 查看当前状态
EXIT           - 退出程序

手指映射: 1=大拇指 2=食指 3=中指 4=无名指 5=小指

状态说明:
  IDLE      - 待机，等待目标力
  DRIVING   - 拉紧绳索（电流控制）
  LOCKED    - 位置锁紧（位置控制）
  RELEASING - 归位中
""")

        elif buf == "EXIT":
            running = False

        else:
            print(f"[未知指令] {buf}  (输入 HELP 查看指令)")

    except Exception as e:
        print(f"[命令错误] {e}")


# =====================================================================
# ======================== TCP 服务器 =================================
# =====================================================================

def tcp_server_thread() -> None:
    HOST, PORT = "0.0.0.0", 8888
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        srv.bind((HOST, PORT))
        srv.listen(1)
        print(f"[TCP] 监听 {HOST}:{PORT}")
        while running:
            try:
                srv.settimeout(1.0)
                conn, addr = srv.accept()
                print(f"[TCP] 连接自 {addr}")
                buf = ""
                try:
                    while running:
                        data = conn.recv(4096)
                        if not data:
                            break
                        buf += data.decode("utf-8", errors="ignore")
                        while "\n" in buf:
                            line, buf = buf.split("\n", 1)
                            cmd = line.strip()
                            if cmd:
                                print(f"[TCP] {cmd}")
                                parse_command(cmd)
                        try:
                            conn.sendall(b"OK\n")
                        except Exception:
                            break
                except Exception as e:
                    print(f"[TCP] 连接错误: {e}")
                finally:
                    conn.close()
            except socket.timeout:
                continue
    except Exception as e:
        print(f"[TCP] 服务器错误: {e}")
    finally:
        srv.close()


# =====================================================================
# ======================== BLE 接收线程 ================================
# =====================================================================

def _process_ble_text(text: str, parser: JsonStreamParser, raw_buf_ref: list) -> None:
    raw_buf_ref[0] += text
    payloads = parser.feed(text)
    for p in payloads:
        vals = extract_from_payload(p)
        if vals:
            ble_store.update_raw(vals)
    # 兜底
    vals = extract_from_raw_text(raw_buf_ref[0])
    if vals:
        ble_store.update_raw(vals)
        raw_buf_ref[0] = ""
    if len(raw_buf_ref[0]) > 512:
        raw_buf_ref[0] = raw_buf_ref[0][-512:]


def ble_broker_client() -> None:
    parser = JsonStreamParser()
    raw_buf = [""]
    while running:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((BROKER_HOST, BROKER_PORT))
            sock.settimeout(5.0)
            print(f"[BLE] 已连接 Broker {BROKER_HOST}:{BROKER_PORT}")
            while running:
                try:
                    chunk = sock.recv(1024)
                    if not chunk:
                        print("[BLE] Broker 断开")
                        break
                    _process_ble_text(chunk.decode("utf-8", errors="ignore"),
                                      parser, raw_buf)
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"[BLE] 接收错误: {e}")
                    break
        except Exception as e:
            if running:
                print(f"[BLE] 连接失败: {e}，3s 后重试…")
                time.sleep(3)
        finally:
            try:
                sock.close()
            except Exception:
                pass


async def _ble_session_direct(address: str, parser: JsonStreamParser) -> None:
    raw_buf = [""]
    disc_evt = asyncio.Event()

    def on_disc(_):
        disc_evt.set()

    async with BleakClient(address, disconnected_callback=on_disc, timeout=15.0) as c:
        if not c.is_connected:
            raise RuntimeError(f"无法连接 {address}")
        print(f"[BLE] 已连接 {address}")
        last_data = time.time()

        def notify(_, data):
            nonlocal last_data
            last_data = time.time()
            _process_ble_text(bytes(data).decode("utf-8", errors="ignore"),
                              parser, raw_buf)

        await c.start_notify(RX_CHAR_UUID, notify)
        last_ka = time.time()
        while running:
            if disc_evt.is_set() or not c.is_connected:
                break
            now = time.time()
            if now - last_ka >= BLE_KEEPALIVE_INTERVAL:
                try:
                    await c.write_gatt_char(TX_CHAR_UUID, BLE_KEEPALIVE_DATA, response=False)
                except Exception:
                    break
                last_ka = now
            if now - last_data > BLE_DATA_TIMEOUT:
                print(f"[BLE] 数据超时，断开 {address}")
                break
            await asyncio.sleep(0.5)
        try:
            await c.stop_notify(RX_CHAR_UUID)
        except Exception:
            pass


async def _ble_main_direct() -> None:
    parser = JsonStreamParser()
    retry = 0
    while running:
        for addr in DEVICE_ADDRESSES:
            if not running:
                break
            try:
                await _ble_session_direct(addr, parser)
                retry = 0
            except Exception as e:
                retry += 1
                delay = min(1.0 * retry, 10.0)
                print(f"[BLE] {addr} 连接失败 ({retry}次): {e}，{delay:.1f}s 后重试")
                await asyncio.sleep(delay)
        if not running:
            break


def ble_thread() -> None:
    if USE_BROKER:
        ble_broker_client()
    else:
        asyncio.run(_ble_main_direct())


# =====================================================================
# ======================== 输入线程 ====================================
# =====================================================================

def input_thread() -> None:
    print("=== 命令就绪 (HELP 查看指令) ===")
    while running:
        try:
            line = sys.stdin.readline()
            if not line:
                break
            line = line.strip()
            if line:
                parse_command(line)
        except Exception as e:
            print(f"[输入错误] {e}")


# =====================================================================
# ======================== 程序入口 ====================================
# =====================================================================

def signal_handler(sig, frame) -> None:
    global running
    if running:
        print("\n接收到 Ctrl+C，正在停止…")
    running = False


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    try:
        setup()

        threading.Thread(target=input_thread,    daemon=True).start()
        threading.Thread(target=tcp_server_thread, daemon=True).start()
        threading.Thread(target=ble_thread,      daemon=True).start()

        loop()

    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"[致命错误] {e}")
    finally:
        print("\n关闭所有舵机…")
        for ctrl in servo_controllers:
            try:
                write_current(ctrl.servo_id, 0)
                torque_off(ctrl.servo_id)
            except Exception:
                pass
        portHandler.closePort()
        print("程序已退出")
