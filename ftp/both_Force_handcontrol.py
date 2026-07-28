#!/usr/bin/env python3
"""
both_Force_handcontrol.py — 双手力反馈双边遥操作主程序

在 Force_handcontrol.py 单右手版本基础上扩展，支持左右手同时控制。

数据流（每手独立）:
  BLE broker (TCP 9001=右手, 9002=左手)
    └─ 前13位 → 数据手套弯曲角度
    └─ 末5位  → 外骨骼触觉传感器（目标力）

  TCP touch (from both_hand_bridge.py on G1)
    右手: 9101   左手: 9103

  TCP ctrl (to both_hand_bridge.py on G1)
    右手: 9100   左手: 9102

每指独立状态机（左右手各5指，共10状态机）:
  GLOVE      → 手套角度控制灵巧手，外骨骼舵机0电流
  FORCE_ENTRY → 灵巧手触觉>阈值 → 冻结手套，灵巧手触觉→解算电流→驱动外骨骼舵机
  LOCKED     → 舵机位置稳定 → 舵机切位置控制锁死
               同时：PID(目标力=外骨骼触觉, 反馈力=灵巧手触觉) → 灵巧手位置增量
  RELEASE    → 外骨骼触觉归零持续300ms → 舵机归位，恢复手套控制

启动方式:
  python3 both_Force_handcontrol.py
"""

import sys
import time
import re
import json
import threading
import socket
import asyncio
import numpy as np
from enum import Enum
from typing import List, Tuple, Optional, Dict

# ========== Dynamixel SDK ==========
from dynamixel_sdk import (
    PortHandler, PacketHandler, COMM_SUCCESS
)

# ========== BLE ==========
from bleak import BleakClient

# =====================================================================
# ======================== 用户可调参数 ================================
# =====================================================================

# ===== G1 桥接地址 =====
BRIDGE_HOST         = "192.168.6.146"   # G1 的 WiFi IP

# ===== 每手独立配置 =====
HAND_CONFIGS: Dict[str, dict] = {
    "r": {
        "label"            : "右手",
        "bridge_ctrl_port" : 9100,     # TCP 角度指令（Pi→G1）
        "bridge_touch_port": 9101,     # TCP 触觉反馈（G1→Pi）
        "ble_broker_port"  : 9001,     # BLE Broker 端口（Pi 本地）
        "dxl_ids"          : [1, 2, 3, 4, 5],   # 外骨骼舵机 ID
        "dxl_device"       : "/dev/ttyAMA0",
    },
    "l": {
        "label"            : "左手",
        "bridge_ctrl_port" : 9102,
        "bridge_touch_port": 9103,
        "ble_broker_port"  : 9002,
        "dxl_ids"          : [6, 7, 8, 9, 10],  # 左手舵机 ID（同一总线）
        "dxl_device"       : "/dev/ttyAMA0",     # 同一串口
    },
}

BROKER_HOST   = "127.0.0.1"   # BLE Broker 监听地址（本机）

# ===== BLE 数据解析 =====
# 18 通道原始数据：前 13 位为弯曲传感器，末 5 位为外骨骼触觉
# 末5位顺序：[拇指, 食指, 中指, 无名指, 小指]
SENSOR_INDEX_R     = [1, 2, 3, 5, 9, 8]  # 右手弯曲通道映射 → [小指,无名,中指,食指,拇弯,拇旋]
SENSOR_INDEX_L     = [9, 8, 5, 3, 1, 2]  # 左手弯曲通道映射 → [小指,无名,中指,食指,拇弯,拇旋]
SENSOR_INDEX_MAP   = {"r": SENSOR_INDEX_R, "l": SENSOR_INDEX_L}
EXO_FORCE_BASELINE = 4.903                 # 外骨骼BLE触觉零偏(N)，减去后才是真实力

# ===== 灵巧手角度范围 =====
HAND_ANGLE_MIN     = 150    # 伸直（小值=伸直，参考 hand_control.py pos=0=伸直）
HAND_ANGLE_MAX     = 850    # 握拳（大值=握拳，参考 hand_control.py pos=1000=握拳）
HAND_ANGLE_EXTEND  = 150    # 伸直安全值（损坏传感器使用，应发伸直=150）
DEFAULT_BEND_MIN   = 1000
DEFAULT_BEND_MAX   = 4000
CONTROL_HZ         = 20     # 灵巧手控制频率

# ===== EMA 滤波 =====
EMA_ALPHA          = 0.2
USE_FILTER         = True

# 触觉 raw → N 转换系数（与 both_hand_bridge.py / hand_bridge.py 保持一致）
CALIBRATION_K      = 0.00292650244415058227
CALIBRATION_B      = -0.6037947156125716
THUMB_CALIBRATION_K = 0.004420145759358057
THUMB_CALIBRATION_B = -1.0701492398616255
FORCE_MAX_N        = 10.0

# ===== 状态机阈值 =====
HAND_CONTACT_ON_N  = 0.50   # 灵巧手触觉超过此值 → 触发 FORCE_ENTRY
RELEASE_HOLD_SEC   = 2.00   # 灵巧手触觉+外骨骼触觉同时低于阈值后持续此时间 → 切回 GLOVE
EXO_ZERO_THR_N     = 0.10   # 外骨骼触觉判零阈值（N）

# ===== 外骨骼→灵巧手位置 PID（LOCKED 阶段）=====
FORCE_KP           = 35.0   # 比例增益 (tick/N)
FORCE_STEP_MAX     = 3.0    # 单周期最大位置修正 (tick)
FORCE_DEADZONE_N   = 0.10   # 死区 (N)
LOCK_ANGLE_RETREAT = 80     # tick，进入 LOCKED 时基准角度向伸直方向回退量

# ===== 手指→角度索引映射 =====
# 灵巧手 angle_set 顺序: [小指, 无名, 中指, 食指, 拇弯, 拇旋]
FINGER_TO_ANGLE_INDEX = {
    0: [4],   # 拇指 → 槽4（拇弯）
    1: [3],   # 食指 → 槽3
    2: [2],   # 中指 → 槽2
    3: [1],   # 无名 → 槽1
    4: [0],   # 小指 → 槽0
}

# ===== Dynamixel 舵机参数 =====
BAUDRATE                = 1000000
PROTOCOL_VERSION        = 2.0
NUM_FINGERS             = 5
CURRENT_LIMIT_MA        = 300               # 单舵机最大电流(mA)
TOTAL_CURRENT_LIMIT_MA  = 800.0             # 五指总电流上限(mA)
HAND_TO_SERVO_GAIN      = 60.0              # mA/N（灵巧手触觉→舵机电流）

# LOCKED 阶段 force_set 参数
HAND_FORCE_BASE         = 200
HAND_FORCE_GAIN         = 40.0
HAND_FORCE_MAX          = 800

# 位置锁定检测
LOCK_POSITION_THR       = 8
LOCK_DETECT_WINDOW      = 0.4
LOCK_MIN_DRIVE_TIME     = 0.3

# 主动释放
RELEASE_CURRENT_MA      = -150.0
RELEASE_POSITION_THR    = 15
RELEASE_TIMEOUT         = 4.0
RELEASE_SLOWDOWN_RANGE  = 50
RELEASE_MIN_TIME        = 0.3
RELEASE_MIN_CURRENT     = -30.0

# Dynamixel Control Table
ADDR_OPERATING_MODE     = 11
ADDR_CURRENT_LIMIT      = 38
ADDR_TORQUE_ENABLE      = 64
ADDR_GOAL_CURRENT       = 102
ADDR_PROFILE_ACCEL      = 108
ADDR_PROFILE_VEL        = 112
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_CURRENT    = 126
ADDR_PRESENT_POSITION   = 132

OP_CURRENT_CONTROL      = 0
OP_CURRENT_BASED_POS    = 5

HW_ERROR_PRINT_INTERVAL = 3.0
DEBUG_PRINT_INTERVAL    = 2.0

# =====================================================================
# ======================== 状态机枚举 ==================================
# =====================================================================

class FingerMode(Enum):
    GLOVE       = "GLOVE"
    FORCE_ENTRY = "FORCE_ENTRY"
    LOCKED      = "LOCKED"
    RELEASE     = "RELEASE"


# =====================================================================
# ======================== 全局状态 ====================================
# =====================================================================

_hw_error_last_print: Dict[int, float] = {}
running = True
FINGER_NAMES = {0: "拇指", 1: "食指", 2: "中指", 3: "无名指", 4: "小指"}

# 共享 Dynamixel 串口（左右手舵机可在同一总线上，通过 ID 区分）
portHandler  = PortHandler(HAND_CONFIGS["r"]["dxl_device"])
packetHandler = PacketHandler(PROTOCOL_VERSION)
dxl_lock     = threading.Lock()


# =====================================================================
# ======================== 线程安全数据存储 ============================
# =====================================================================

class SharedData:
    """每手独立的跨线程共享数据"""

    def __init__(self, label: str):
        self.label = label
        self._lock = threading.Lock()

        self.bend_raw: List[float] = [0.0] * 18
        self.bend_updated = False
        self.exo_force: List[float] = [0.0] * NUM_FINGERS
        self.hand_force: List[float] = [0.0] * NUM_FINGERS
        self.ble_last_update = 0.0
        self.touch_last_update = 0.0

    def update_ble(self, all_values: List[float]) -> None:
        with self._lock:
            if len(all_values) >= 18:
                self.bend_raw = list(all_values[:18])
                for fi in range(NUM_FINGERS):
                    raw = all_values[13 + fi]
                    self.exo_force[fi] = 0.0 if raw <= EXO_FORCE_BASELINE else raw
            elif len(all_values) >= 5:
                for fi in range(NUM_FINGERS):
                    raw = all_values[-(NUM_FINGERS - fi)]
                    self.exo_force[fi] = 0.0 if raw <= EXO_FORCE_BASELINE else raw
            self.ble_last_update = time.time()
            self.bend_updated = True

    def update_hand_force(self, forces: List[float]) -> None:
        with self._lock:
            for i in range(min(NUM_FINGERS, len(forces))):
                self.hand_force[i] = forces[i]
            self.touch_last_update = time.time()

    def get_snapshot(self):
        with self._lock:
            return (
                list(self.bend_raw),
                list(self.exo_force),
                list(self.hand_force),
                self.ble_last_update,
                self.touch_last_update,
            )


# 每手一个 SharedData
shared_r = SharedData("右手")
shared_l = SharedData("左手")
SHARED_MAP = {"r": shared_r, "l": shared_l}


# =====================================================================
# ======================== BLE JSON 解析 ==============================
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
                    obj = {"values": obj}
                if isinstance(obj, dict):
                    out.append(obj)
            except json.JSONDecodeError:
                if "\n" in self._buf:
                    line, self._buf = self._buf.split("\n", 1)
                    try:
                        o = json.loads(line.strip())
                        if isinstance(o, list):
                            o = {"values": o}
                        if isinstance(o, dict):
                            out.append(o)
                    except Exception:
                        pass
                else:
                    break
        return out


def extract_all_values(text: str) -> List[float]:
    """
    从完整的 {a,b,...} 块中提取数值。
    使用 float() 同时兼容整数和小数（如 4.90）。
    硬件格式: {0,4090,...,4.90,4.90,4.90,4.90,4.90} 共18个值
    """
    m = re.search(r'\{([^}]+)\}', text)
    if m:
        parts = [p.strip() for p in m.group(1).split(",") if p.strip()]
        try:
            vals = [float(p) for p in parts]
            if len(vals) >= 5:
                return vals
        except ValueError:
            pass
    return []


# 每手独立的 TCP 接收缓冲区（模块级，跨调用保留）
_ble_recv_buf: Dict[str, str] = {"r": "", "l": ""}


def process_ble_text(text: str, parser: JsonStreamParser,
                     raw_buf: list, shared: SharedData,
                     hand: str = "r") -> None:
    """
    基于完整 {} 块的缓冲解析。
    TCP 可能分片，将数据追加到缓冲区，只处理收到完整 {} 块的数据，
    避免分片导致只解析到部分值（如只解析到13个整数，缺少后5个4.90触觉值）。
    """
    global _ble_recv_buf
    _ble_recv_buf[hand] += text

    while True:
        start = _ble_recv_buf[hand].find('{')
        end   = _ble_recv_buf[hand].find('}')
        if start == -1:
            # 没有起始符，清空脏数据
            _ble_recv_buf[hand] = ""
            break
        if end == -1 or end < start:
            # 没有完整块，等待下次数据；丢弃起始符之前的脏数据
            _ble_recv_buf[hand] = _ble_recv_buf[hand][start:]
            break

        # 提取完整块并消费
        block = _ble_recv_buf[hand][start:end + 1]
        _ble_recv_buf[hand] = _ble_recv_buf[hand][end + 1:]

        vals = extract_all_values(block)
        if vals and len(vals) >= 13:
            shared.update_ble(vals)

    # 防止缓冲区无限增长
    if len(_ble_recv_buf[hand]) > 2048:
        _ble_recv_buf[hand] = _ble_recv_buf[hand][-512:]


# =====================================================================
# ======================== TCP 触觉接收（每手独立）====================
# =====================================================================

class TcpTouchReceiver:
    """通过 TCP 从 both_hand_bridge.py 接收指定手的触觉数据"""

    def __init__(self, hand: str):
        cfg = HAND_CONFIGS[hand]
        self._host  = BRIDGE_HOST
        self._port  = cfg["bridge_touch_port"]
        self._label = cfg["label"]
        self._shared = SHARED_MAP[hand]
        self._thread: Optional[threading.Thread] = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name=f"touch_{self._label}")
        self._thread.start()

    def _run(self):
        buf = ""
        while running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((self._host, self._port))
                sock.settimeout(5.0)
                print(f"[{self._label} TCP触觉] 已连接 {self._host}:{self._port}")
                while running:
                    try:
                        chunk = sock.recv(4096)
                        if not chunk:
                            print(f"[{self._label} TCP触觉] 连接断开")
                            break
                        buf += chunk.decode("utf-8", errors="ignore")
                        while "\n" in buf:
                            line, buf = buf.split("\n", 1)
                            line = line.strip()
                            if not line:
                                continue
                            try:
                                pkt = json.loads(line)
                                if pkt.get("type") == "touch":
                                    forces_raw = pkt.get("forces", [0.0] * 6)
                                    self._shared.update_hand_force(forces_raw[:5])
                            except json.JSONDecodeError:
                                pass
                    except socket.timeout:
                        continue
                    except Exception as e:
                        print(f"[{self._label} TCP触觉] 接收错误: {e}")
                        break
            except Exception as e:
                if running:
                    print(f"[{self._label} TCP触觉] 连接失败({e})，3s 后重试…")
                    time.sleep(3)
            finally:
                try:
                    sock.close()
                except Exception:
                    pass


# =====================================================================
# ======================== EMA 滤波器 ==================================
# =====================================================================

class MultiEMA:
    def __init__(self, channels: int, alpha: float):
        self.alpha = alpha
        self.values: Optional[np.ndarray] = None

    def update(self, x: np.ndarray) -> np.ndarray:
        if self.values is None:
            self.values = x.copy()
        else:
            self.values = self.alpha * x + (1 - self.alpha) * self.values
        return self.values.copy()

    def reset(self):
        self.values = None


# =====================================================================
# ======================== Dynamixel 底层 I/O ==========================
# =====================================================================

def _hw_err(sid: int, msg: str) -> None:
    now = time.time()
    if now - _hw_error_last_print.get(sid, 0) >= HW_ERROR_PRINT_INTERVAL:
        print(msg)
        _hw_error_last_print[sid] = now


def dxl_read4(sid: int, addr: int) -> int:
    with dxl_lock:
        v, r, e = packetHandler.read4ByteTxRx(portHandler, sid, addr)
    if r != COMM_SUCCESS:
        _hw_err(sid, f"[舵机{sid}] 读4失败: {packetHandler.getTxRxResult(r)}")
    return v - 4294967296 if v > 2147483647 else v


def dxl_read2(sid: int, addr: int) -> int:
    with dxl_lock:
        v, r, _ = packetHandler.read2ByteTxRx(portHandler, sid, addr)
    return v - 65536 if v > 32767 else v


def dxl_write1(sid: int, addr: int, val: int) -> None:
    with dxl_lock:
        packetHandler.write1ByteTxRx(portHandler, sid, addr, val)


def dxl_write2(sid: int, addr: int, val: int) -> None:
    with dxl_lock:
        packetHandler.write2ByteTxRx(portHandler, sid, addr, val)


def dxl_write2s(sid: int, addr: int, val: int) -> None:
    if val < 0:
        val = (1 << 16) + val
    dxl_write2(sid, addr, val)


def dxl_write4(sid: int, addr: int, val: int) -> None:
    with dxl_lock:
        packetHandler.write4ByteTxRx(portHandler, sid, addr, int(val))


def torque_on(sid: int) -> None:
    dxl_write1(sid, ADDR_TORQUE_ENABLE, 1)


def torque_off(sid: int) -> None:
    dxl_write1(sid, ADDR_TORQUE_ENABLE, 0)


def set_servo_mode(sid: int, mode: int) -> None:
    torque_off(sid)
    dxl_write1(sid, ADDR_OPERATING_MODE, mode)
    torque_on(sid)


def write_servo_current(sid: int, mA: float) -> None:
    dxl_write2s(sid, ADDR_GOAL_CURRENT, int(round(mA)))


def write_servo_position(sid: int, pos: int) -> None:
    dxl_write4(sid, ADDR_GOAL_POSITION, pos)


# =====================================================================
# ======================== 舵机控制器 ==================================
# =====================================================================

class ServoCtrl:
    def __init__(self, finger_idx: int, sid: int, label_prefix: str):
        self.fi           = finger_idx
        self.sid          = sid
        self.name         = f"{label_prefix}{FINGER_NAMES[finger_idx]}"
        self.init_pos: Optional[int] = None
        self.is_initialized = False
        self.drive_start  = 0.0
        self.pos_history: List[Tuple[float, int]] = []
        self.release_start = 0.0
        self._pending_mA  = 0.0

    def record_init(self) -> None:
        pos = dxl_read4(self.sid, ADDR_PRESENT_POSITION)
        self.init_pos = pos
        self.is_initialized = True
        print(f"  [{self.name}] 舵机{self.sid} 初始位置={pos}")


# =====================================================================
# ======================== 每指独立状态机 ==============================
# =====================================================================

class FingerStateMachine:
    def __init__(self, hand: str):
        cfg = HAND_CONFIGS[hand]
        self.hand         = hand
        self.label        = cfg["label"]
        self.modes        = [FingerMode.GLOVE] * NUM_FINGERS
        self.servos       = [
            ServoCtrl(fi, cfg["dxl_ids"][fi], cfg["label"])
            for fi in range(NUM_FINGERS)
        ]
        self.force_ref_angles = [500] * 6
        self.release_timer    = [None] * NUM_FINGERS  # type: List[Optional[float]]

    def record_init(self) -> None:
        print(f"\n=== [{self.label}] 记录外骨骼舵机初始位置 ===")
        for s in self.servos:
            s.record_init()
        print(f"[{self.label}] 记录完成")

    def tick(
        self,
        glove_angles: List[int],
        exo_force: List[float],
        hand_force: List[float],
        now: float,
    ) -> Tuple[List[int], List[int]]:
        output = list(glove_angles)
        forces = [200] * 6

        for fi in range(NUM_FINGERS):
            mode     = self.modes[fi]
            srv      = self.servos[fi]
            target   = exo_force[fi]
            feedback = hand_force[fi]
            joint_ids = FINGER_TO_ANGLE_INDEX.get(fi, [])

            # ---- GLOVE ----
            if mode == FingerMode.GLOVE:
                srv._pending_mA = 0.0
                if feedback >= HAND_CONTACT_ON_N:
                    self.modes[fi] = FingerMode.FORCE_ENTRY
                    srv.drive_start = now
                    srv.pos_history = []
                    for ji in joint_ids:
                        self.force_ref_angles[ji] = glove_angles[ji]
                    set_servo_mode(srv.sid, OP_CURRENT_CONTROL)
                    dxl_write2(srv.sid, ADDR_CURRENT_LIMIT, CURRENT_LIMIT_MA)
                    print(f"[{srv.name}] GLOVE → FORCE_ENTRY  手触觉={feedback:.2f}N")

            # ---- FORCE_ENTRY ----
             elif mode == FingerMode.FORCE_ENTRY:
                drive_mA = min(feedback * HAND_TO_SERVO_GAIN, float(CURRENT_LIMIT_MA))
                srv._pending_mA = drive_mA
                for ji in joint_ids:
                    output[ji] = self.force_ref_angles[ji]
                pos = dxl_read4(srv.sid, ADDR_PRESENT_POSITION)
                _update_pos_history(srv, now, pos)
                if _is_position_stable(srv, now):
                    _enter_locked(srv, pos)
                    self.modes[fi] = FingerMode.LOCKED
                    # 向伸直方向（850）回退，与 Force_handcontrol.py 一致
                    for ji in joint_ids:
                        self.force_ref_angles[ji] = int(np.clip(
                            self.force_ref_angles[ji] + LOCK_ANGLE_RETREAT,  # ← +，向850(伸直)方向
                            HAND_ANGLE_MIN, HAND_ANGLE_MAX
                        ))
                    print(f"[{srv.name}] FORCE_ENTRY → LOCKED  位置={pos}  "
                          f"基准角={self.force_ref_angles[joint_ids[0]] if joint_ids else '?'}")

            # ---- LOCKED ----
            elif mode == FingerMode.LOCKED:
                srv._pending_mA = 0.0
                err = target - feedback
                if abs(err) > FORCE_DEADZONE_N:
                    step = float(np.clip(FORCE_KP * err, -FORCE_STEP_MAX, FORCE_STEP_MAX))
                    for ji in joint_ids:
                        self.force_ref_angles[ji] = int(np.clip(
                            self.force_ref_angles[ji] - step,
                            HAND_ANGLE_MIN, HAND_ANGLE_MAX
                        ))
                for ji in joint_ids:
                    output[ji] = self.force_ref_angles[ji]
                locked_force = int(np.clip(
                    HAND_FORCE_BASE + target * HAND_FORCE_GAIN,
                    HAND_FORCE_BASE, HAND_FORCE_MAX
                ))
                for ji in joint_ids:
                    forces[ji] = locked_force

                if target <= EXO_ZERO_THR_N:
                    if self.release_timer[fi] is None:
                        self.release_timer[fi] = now
                    elif now - self.release_timer[fi] >= RELEASE_HOLD_SEC:
                        self.release_timer[fi] = None
                        self.modes[fi] = FingerMode.RELEASE
                        _enter_releasing(srv, now)
                        for ji in joint_ids:
                            output[ji] = glove_angles[ji]
                            self.force_ref_angles[ji] = glove_angles[ji]
                        print(f"[{srv.name}] LOCKED → RELEASE  exo={target:.2f}N")
                else:
                    self.release_timer[fi] = None

            # ---- RELEASE ----
            elif mode == FingerMode.RELEASE:
                for ji in joint_ids:
                    output[ji] = glove_angles[ji]
                done = _tick_releasing(srv, now)
                if done:
                    self.modes[fi] = FingerMode.GLOVE
                    print(f"[{srv.name}] RELEASE → GLOVE")

        return output, forces

    def flush_servo_currents(self) -> None:
        total_pos = sum(
            max(0.0, s._pending_mA) for s in self.servos
            if self.modes[s.fi] == FingerMode.FORCE_ENTRY
        )
        scale = 1.0
        if total_pos > TOTAL_CURRENT_LIMIT_MA and total_pos > 0:
            scale = TOTAL_CURRENT_LIMIT_MA / total_pos
        for s in self.servos:
            mode = self.modes[s.fi]
            if mode == FingerMode.FORCE_ENTRY:
                write_servo_current(s.sid, s._pending_mA * scale)
            elif mode == FingerMode.RELEASE:
                write_servo_current(s.sid, s._pending_mA)


# =====================================================================
# ======================== 舵机辅助函数 ================================
# =====================================================================

def _update_pos_history(srv: ServoCtrl, now: float, pos: int) -> None:
    srv.pos_history.append((now, pos))
    cutoff = now - LOCK_DETECT_WINDOW * 2
    srv.pos_history = [(t, p) for t, p in srv.pos_history if t >= cutoff]


def _is_position_stable(srv: ServoCtrl, now: float) -> bool:
    if now - srv.drive_start < LOCK_MIN_DRIVE_TIME:
        return False
    window = [p for t, p in srv.pos_history if now - t <= LOCK_DETECT_WINDOW]
    if len(window) < 3:
        return False
    return (max(window) - min(window)) <= LOCK_POSITION_THR


def _enter_locked(srv: ServoCtrl, pos: int) -> None:
    write_servo_current(srv.sid, 0)
    time.sleep(0.02)
    torque_off(srv.sid)
    dxl_write1(srv.sid, ADDR_OPERATING_MODE, OP_CURRENT_BASED_POS)
    dxl_write2(srv.sid, ADDR_CURRENT_LIMIT, CURRENT_LIMIT_MA)
    dxl_write4(srv.sid, ADDR_PROFILE_VEL, 0)
    dxl_write4(srv.sid, ADDR_PROFILE_ACCEL, 0)
    torque_on(srv.sid)
    write_servo_position(srv.sid, pos)


def _enter_releasing(srv: ServoCtrl, now: float) -> None:
    set_servo_mode(srv.sid, OP_CURRENT_CONTROL)
    dxl_write2(srv.sid, ADDR_CURRENT_LIMIT, CURRENT_LIMIT_MA)
    srv.release_start = now


def _tick_releasing(srv: ServoCtrl, now: float) -> bool:
    if not srv.is_initialized or srv.init_pos is None:
        write_servo_current(srv.sid, 0)
        return True
    pos = dxl_read4(srv.sid, ADDR_PRESENT_POSITION)
    dist = pos - srv.init_pos
    elapsed = now - srv.release_start
    if elapsed >= RELEASE_MIN_TIME and (
        abs(dist) <= RELEASE_POSITION_THR or elapsed > RELEASE_TIMEOUT
    ):
        write_servo_current(srv.sid, 0)
        time.sleep(0.02)
        torque_off(srv.sid)
        dxl_write1(srv.sid, ADDR_OPERATING_MODE, OP_CURRENT_BASED_POS)
        dxl_write2(srv.sid, ADDR_CURRENT_LIMIT, CURRENT_LIMIT_MA)
        dxl_write4(srv.sid, ADDR_PROFILE_VEL, 50)
        dxl_write4(srv.sid, ADDR_PROFILE_ACCEL, 30)
        torque_on(srv.sid)
        write_servo_position(srv.sid, srv.init_pos)
        reason = "到位" if abs(dist) <= RELEASE_POSITION_THR else f"超时{elapsed:.1f}s"
        print(f"  [{srv.name}] 归位完成 ({reason})")
        return True
    if abs(dist) < RELEASE_SLOWDOWN_RANGE and abs(dist) > 0:
        ratio = abs(dist) / RELEASE_SLOWDOWN_RANGE
        mA = RELEASE_CURRENT_MA * max(ratio, 0.2)
    else:
        mA = RELEASE_CURRENT_MA
    if abs(mA) < abs(RELEASE_MIN_CURRENT):
        mA = RELEASE_MIN_CURRENT
    if dist < 0:
        mA = -mA
    srv._pending_mA = mA
    return False


# =====================================================================
# ======================== 角度转换 ====================================
# =====================================================================

def bend_to_angle(raw_values: np.ndarray,
                  bend_min: np.ndarray, bend_max: np.ndarray,
                  sensor_index: list) -> List[int]:
    """
    bend_min = 伸直(伸直极限)采集值
    bend_max = 握拳(弯曲极限)采集值
    与 Force_handcontrol.py 保持一致：
      ratio=0(伸直) → HAND_ANGLE_MAX=850
      ratio=1(握拳) → HAND_ANGLE_MIN=150
    """
    angles = []
    for i, ch in enumerate(sensor_index):
        raw = float(raw_values[ch]) if ch < len(raw_values) else 0.0
        # 通道值为0或标定范围<50视为传感器损坏，发送伸直(150)
        if raw == 0.0:
            angles.append(HAND_ANGLE_EXTEND)   # 150=伸直
            continue
        rng = float(bend_max[i] - bend_min[i])
        if abs(rng) < 50:
            angles.append(HAND_ANGLE_EXTEND)   # 150=伸直
            continue
        ratio = float(np.clip((raw - bend_min[i]) / rng, 0.0, 1.0))
        # 与 Force_handcontrol.py 公式完全一致
        # ratio=0(伸直)→850, ratio=1(握拳)→150
        angles.append(int(HAND_ANGLE_MAX - ratio * (HAND_ANGLE_MAX - HAND_ANGLE_MIN)))
    return angles

def calibrate_glove(shared: SharedData, sensor_index: list,
                    duration_sec: float = 2.0,
                    bend_min_ref: np.ndarray = None,
                    bend_max_ref: np.ndarray = None
                    ) -> Tuple[np.ndarray, np.ndarray, bool]:
    """
    交互式数据手套标定，返回 (bend_min, bend_max, calib_done)
    """
    label = shared.label

    print(f"\n[{label} 标定] 等待 BLE 数据…（最多 30s）")
    t0 = time.time()
    while time.time() - t0 < 30.0:
        _, _, _, ble_t, _ = shared.get_snapshot()
        if ble_t > 0:
            break
        time.sleep(0.1)
    else:
        print(f"[{label} 标定] 超时未收到 BLE 数据，使用默认标定值")
        bmin = np.full(len(sensor_index), DEFAULT_BEND_MIN, dtype=np.float64)
        bmax = np.full(len(sensor_index), DEFAULT_BEND_MAX, dtype=np.float64)
        return bmin, bmax, False

    def collect(pose_name: str) -> np.ndarray:
        try:
            input(f"\n[{label}] 请做出【{pose_name}】姿态，然后按回车开始采集…")
        except EOFError:
            print(f"\n[{label} 标定] 无交互输入，立即采集 {pose_name}")
        print(f"[{label} 标定] 正在采集 {pose_name} {duration_sec:.1f}s…")
        samples: List[np.ndarray] = []
        t_start = time.time()
        while time.time() - t_start < duration_sec:
            bend_raw, _, _, ble_t2, _ = shared.get_snapshot()
            if ble_t2 > 0:
                raw_arr = np.array(bend_raw, dtype=np.float64)
                values = raw_arr[list(sensor_index)]
                samples.append(values.copy())
                print(f"\r[{label} 标定] {pose_name}: {values.astype(int).tolist()}", end="", flush=True)
            time.sleep(0.02)
        print()
        if not samples:
            raise RuntimeError(f"[{label} 标定] 失败：{pose_name} 没有采集到数据")
        ref = np.median(np.vstack(samples), axis=0)
        print(f"[{label} 标定] {pose_name} 参考值: {ref.astype(int).tolist()}")
        return ref

    bmax = collect("握拳(弯曲极限)")
    bmin = collect("伸直(伸直极限)")
    print(f"[{label} 标定] 完成 ✓")
    return bmin, bmax, True


# =====================================================================
# ======================== Dynamixel 硬件初始化 ========================
# =====================================================================

def setup_servos(fsm_r: FingerStateMachine, fsm_l: FingerStateMachine) -> None:
    if not portHandler.openPort():
        print("[错误] 舵机串口打开失败，跳过舵机初始化")
        return
    if not portHandler.setBaudRate(BAUDRATE):
        print("[错误] 波特率设置失败")
        return
    device = HAND_CONFIGS["r"]["dxl_device"]
    print(f"[舵机] 串口 {device} @ {BAUDRATE}bps 已开启")

    for fsm in (fsm_r, fsm_l):
        for s in fsm.servos:
            _, r, _ = packetHandler.ping(portHandler, s.sid)
            if r != COMM_SUCCESS:
                print(f"  [{s.name}] 舵机{s.sid} Ping 失败，跳过")
                continue
            print(f"  [{s.name}] 舵机{s.sid} Ping 成功")
            s.is_initialized = True
            torque_off(s.sid)
            dxl_write1(s.sid, ADDR_OPERATING_MODE, OP_CURRENT_CONTROL)
            dxl_write2(s.sid, ADDR_CURRENT_LIMIT, CURRENT_LIMIT_MA)
            dxl_write4(s.sid, ADDR_PROFILE_VEL, 100)
            dxl_write4(s.sid, ADDR_PROFILE_ACCEL, 50)
            torque_on(s.sid)
            time.sleep(0.05)

    print("[舵机] 初始化完成，请输入 INIT 记录初始位置")


# =====================================================================
# ======================== 主控制循环（每手一个线程）==================
# =====================================================================

def control_loop(hand: str, fsm: FingerStateMachine,
                 bend_min_ref: np.ndarray, bend_max_ref: np.ndarray) -> None:
    """以 CONTROL_HZ 频率运行，处理指定手的灵巧手控制和舵机状态机"""
    cfg    = HAND_CONFIGS[hand]
    label  = cfg["label"]
    shared = SHARED_MAP[hand]
    sensor_index = SENSOR_INDEX_MAP[hand]
    period = 1.0 / CONTROL_HZ
    last_print = time.time()
    ema    = MultiEMA(channels=18, alpha=EMA_ALPHA)

    ctrl_host = BRIDGE_HOST
    ctrl_port = cfg["bridge_ctrl_port"]
    tcp_ctrl_sock: Optional[socket.socket] = None

    def _get_tcp_ctrl() -> Optional[socket.socket]:
        nonlocal tcp_ctrl_sock
        if tcp_ctrl_sock is not None:
            return tcp_ctrl_sock
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2.0)
            s.connect((ctrl_host, ctrl_port))
            tcp_ctrl_sock = s
            print(f"[{label} TCP控制] 已连接 {ctrl_host}:{ctrl_port}")
        except Exception as e:
            print(f"[{label} TCP控制] 连接失败: {e}")
            tcp_ctrl_sock = None
        return tcp_ctrl_sock

    print(f"[{label} 控制循环] 启动，TCP→{ctrl_host}:{ctrl_port}")

    while running:
        t0 = time.time()
        try:
            bend_raw, exo_force, hand_force, ble_t, _ = shared.get_snapshot()
            ble_age = time.time() - ble_t if ble_t > 0 else float("inf")

            if ble_age > 5.0:
                time.sleep(period)
                continue

            raw_arr = np.array(bend_raw, dtype=np.float64)
            filtered = ema.update(raw_arr) if USE_FILTER else raw_arr
            glove_angles = bend_to_angle(filtered, bend_min_ref, bend_max_ref, sensor_index)

            now = t0
            output_angles, hand_force_set = fsm.tick(
                glove_angles, exo_force, hand_force, now
            )
            fsm.flush_servo_currents()

            # TCP → G1 both_hand_bridge.py
            s = _get_tcp_ctrl()
            if s is not None:
                try:
                    pkt = json.dumps({
                        "type"      : "ctrl",
                        "angle_set" : [int(a) for a in output_angles],
                        "force_set" : list(hand_force_set),
                        "speed_set" : [500] * 6,
                        "mode"      : 1,
                    }) + "\n"
                    s.sendall(pkt.encode("utf-8"))
                except Exception as e:
                    print(f"[{label} TCP控制] 发送失败({e})，重连…")
                    try:
                        tcp_ctrl_sock.close()
                    except Exception:
                        pass
                    tcp_ctrl_sock = None

            # 状态打印
            if time.time() - last_print >= DEBUG_PRINT_INTERVAL:
                modes_str = " ".join(
                    f"{FINGER_NAMES[i]}:{fsm.modes[i].value}" for i in range(NUM_FINGERS)
                )
                exo_str  = " ".join(f"{v:.2f}" for v in exo_force)
                hand_str = " ".join(f"{v:.2f}" for v in hand_force)
                ang_str  = " ".join(str(a) for a in output_angles)
                print(f"[{label}] {modes_str}")
                print(f"  angle=[{ang_str}]  exo=[{exo_str}]N  hand=[{hand_str}]N")
                last_print = time.time()

        except Exception as e:
            print(f"[{label} 控制循环错误] {e}")

        elapsed = time.time() - t0
        sleep_t = period - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)


# =====================================================================
# ======================== BLE Broker 接收（每手独立）=================
# =====================================================================

def ble_broker_client(hand: str) -> None:
    cfg    = HAND_CONFIGS[hand]
    label  = cfg["label"]
    port   = cfg["ble_broker_port"]
    shared = SHARED_MAP[hand]
    parser = JsonStreamParser()
    raw_buf = [""]
    while running:
        # 重连时清空缓冲区，避免上次残留半包污染新连接
        _ble_recv_buf[hand] = ""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((BROKER_HOST, port))
            sock.settimeout(5.0)
            print(f"[{label} BLE] 已连接 Broker {BROKER_HOST}:{port}")
            while running:
                try:
                    chunk = sock.recv(1024)
                    if not chunk:
                        print(f"[{label} BLE] Broker 断开")
                        break
                    process_ble_text(
                        chunk.decode("utf-8", errors="ignore"),
                        parser, raw_buf, shared, hand
                    )
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"[{label} BLE] 接收错误: {e}")
                    break
        except Exception as e:
            if running:
                print(f"[{label} BLE] 连接失败: {e}，3s 后重试…")
                time.sleep(3)
        finally:
            try:
                sock.close()
            except Exception:
                pass


# =====================================================================
# ======================== 命令解析 ====================================
# =====================================================================

def parse_cmd(raw: str, fsm_r: FingerStateMachine,
              fsm_l: FingerStateMachine) -> None:
    global running
    buf = raw.replace("：", ":").strip().upper()
    try:
        if buf == "INIT":
            fsm_r.record_init()
            fsm_l.record_init()
            print("  [提示] 双手 INIT 完成")

        elif buf == "INIT R":
            fsm_r.record_init()

        elif buf == "INIT L":
            fsm_l.record_init()

        elif buf == "STATUS":
            for fsm in (fsm_r, fsm_l):
                shared = SHARED_MAP[fsm.hand]
                bend_raw, exo_force, hand_force, ble_t, touch_t = shared.get_snapshot()
                ble_age   = time.time() - ble_t   if ble_t   > 0 else float("inf")
                touch_age = time.time() - touch_t if touch_t > 0 else float("inf")
                print(f"\n=== [{fsm.label}] BLE age={ble_age:.1f}s  触觉 age={touch_age:.1f}s ===")
                for fi in range(NUM_FINGERS):
                    s    = fsm.servos[fi]
                    mode = fsm.modes[fi]
                    pos  = dxl_read4(s.sid, ADDR_PRESENT_POSITION) if s.is_initialized else -1
                    cur  = dxl_read2(s.sid, ADDR_PRESENT_CURRENT) if s.is_initialized else 0
                    print(
                        f"  {s.name}: [{mode.value:12s}] "
                        f"pos={pos:5d} I={cur:4d}mA "
                        f"exo={exo_force[fi]:.2f}N hand={hand_force[fi]:.2f}N"
                    )

        elif buf == "HELP":
            print("""
=== both_Force_handcontrol 指令 ===
INIT      - 记录双手外骨骼舵机初始位置
INIT R    - 仅记录右手
INIT L    - 仅记录左手
STATUS    - 查看双手详细状态
EXIT      - 退出程序
HELP      - 显示此帮助
""")

        elif buf == "EXIT":
            running = False

        else:
            print(f"[未知指令] {buf}  (输入 HELP)")

    except Exception as e:
        print(f"[命令错误] {e}")


def input_thread_fn(fsm_r: FingerStateMachine,
                    fsm_l: FingerStateMachine) -> None:
    print("=== 命令就绪 (输入 HELP 查看指令) ===")
    while running:
        try:
            line = sys.stdin.readline()
            if not line:
                break
            line = line.strip()
            if not line:
                continue
            parse_cmd(line, fsm_r, fsm_l)
        except Exception as e:
            print(f"[输入错误] {e}")


# =====================================================================
# ======================== 程序入口 ====================================
# =====================================================================

import signal

def signal_handler(sig, frame) -> None:
    global running
    if running:
        print("\n接收到 Ctrl+C，正在停止…")
    running = False


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    print("=" * 60)
    print("   both_Force_handcontrol.py  双手力反馈双边遥操作")
    print("   版本: V1   日期: 2026-05-01")
    print(f"   右手: 控制:{HAND_CONFIGS['r']['bridge_ctrl_port']}  "
          f"触觉:{HAND_CONFIGS['r']['bridge_touch_port']}  "
          f"BLE:{HAND_CONFIGS['r']['ble_broker_port']}")
    print(f"   左手: 控制:{HAND_CONFIGS['l']['bridge_ctrl_port']}  "
          f"触觉:{HAND_CONFIGS['l']['bridge_touch_port']}  "
          f"BLE:{HAND_CONFIGS['l']['ble_broker_port']}")
    print("=" * 60)

    # ── 第一步：舵机初始化 ───────────────────────────────────────────
    print("\n[步骤 1/4] 舵机初始化…")
    fsm_r = FingerStateMachine("r")
    fsm_l = FingerStateMachine("l")
    setup_servos(fsm_r, fsm_l)

    # ── 第二步：启动 BLE Broker 接收线程（双手） ────────────────────
    print("\n[步骤 2/4] 启动 BLE Broker 接收线程…")
    print(f"  右手 BLE → {BROKER_HOST}:{HAND_CONFIGS['r']['ble_broker_port']}")
    print(f"  左手 BLE → {BROKER_HOST}:{HAND_CONFIGS['l']['ble_broker_port']}")
    threading.Thread(target=ble_broker_client, args=("r",), daemon=True,
                     name="ble_r").start()
    threading.Thread(target=ble_broker_client, args=("l",), daemon=True,
                     name="ble_l").start()

    # ── 第三步：等待 INIT，然后进行双手数据手套标定 ─────────────────
    print("\n[步骤 3/4] 等待舵机 INIT…")
    print("  请佩戴外骨骼手套，然后输入 INIT 记录舵机初始位置。\n")
    while running:
        try:
            line = sys.stdin.readline()
        except EOFError:
            break
        if not line:
            break
        cmd = line.strip().upper()
        if cmd in ("INIT", "INIT R", "INIT L"):
            if cmd in ("INIT", "INIT R"):
                fsm_r.record_init()
            if cmd in ("INIT", "INIT L"):
                fsm_l.record_init()
            print("\n[步骤 3/4] INIT 完成，开始数据手套标定…")
            # 分别标定左右手手套（左右手通道映射不同）
            bmin_r = np.full(len(SENSOR_INDEX_R), DEFAULT_BEND_MIN, dtype=np.float64)
            bmax_r = np.full(len(SENSOR_INDEX_R), DEFAULT_BEND_MAX, dtype=np.float64)
            bmin_l = np.full(len(SENSOR_INDEX_L), DEFAULT_BEND_MIN, dtype=np.float64)
            bmax_l = np.full(len(SENSOR_INDEX_L), DEFAULT_BEND_MAX, dtype=np.float64)

            bmin_r, bmax_r, calib_r = calibrate_glove(shared_r, SENSOR_INDEX_R, duration_sec=2.0)
            bmin_l, bmax_l, calib_l = calibrate_glove(shared_l, SENSOR_INDEX_L, duration_sec=2.0)
            break
        elif cmd == "EXIT":
            running = False
            break
        elif cmd == "HELP":
            print("  此阶段只接受 INIT / INIT R / INIT L 指令")
        else:
            print(f"  [提示] 请先输入 INIT（当前输入: {cmd}）")

    if not running:
        sys.exit(0)

    # ── 第四步：启动 TCP 触觉接收 + 控制线程 ────────────────────────
    print("\n[步骤 4/4] TCP 桥接模式初始化…")
    print(f"  G1 地址: {BRIDGE_HOST}")

    TcpTouchReceiver("r").start()
    TcpTouchReceiver("l").start()
    print("  [TCP] 双手触觉接收线程已启动，等待 G1 连接…")

    # 控制循环（双手并行，各自独立线程）
    threading.Thread(
        target=control_loop,
        args=("r", fsm_r, bmin_r, bmax_r),
        daemon=True, name="ctrl_r"
    ).start()
    threading.Thread(
        target=control_loop,
        args=("l", fsm_l, bmin_l, bmax_l),
        daemon=True, name="ctrl_l"
    ).start()

    threading.Thread(
        target=input_thread_fn,
        args=(fsm_r, fsm_l),
        daemon=True, name="input"
    ).start()

    # ── 启动摘要 ─────────────────────────────────────────────────────
    init_r = [s.name for s in fsm_r.servos if s.is_initialized]
    init_l = [s.name for s in fsm_l.servos if s.is_initialized]
    print("\n" + "=" * 60)
    print("  ✓ 双手系统就绪")
    print(f"  右手已初始化舵机: {', '.join(init_r) if init_r else '无'}")
    print(f"  左手已初始化舵机: {', '.join(init_l) if init_l else '无'}")
    print(f"  控制频率: {CONTROL_HZ} Hz  |  外骨骼力基线: {EXO_FORCE_BASELINE} N")
    print("=" * 60)
    print("\n输入 STATUS / HELP / EXIT 进行操作\n")

    try:
        while running:
            time.sleep(0.5)
    except KeyboardInterrupt:
        running = False
    finally:
        print("\n关闭所有舵机…")
        for fsm in (fsm_r, fsm_l):
            for s in fsm.servos:
                try:
                    write_servo_current(s.sid, 0)
                    torque_off(s.sid)
                except Exception:
                    pass
        try:
            portHandler.closePort()
        except Exception:
            pass
        print("程序已退出")
