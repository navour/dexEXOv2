#!/usr/bin/env python3
"""
Force_handcontrol.py — 灵巧手力反馈双边遥操作主程序

数据流：
  BLE (ble_broker.py TCP 9001)
    └─ 前13位 → 数据手套弯曲角度
    └─ 末5位  → 外骨骼触觉传感器（目标力）

  DDS rt/inspire_hand/touch/r
    └─ 灵巧手触觉传感器（反馈力）

每指独立状态机：
  GLOVE      → 手套角度控制灵巧手，外骨骼舵机0电流
  FORCE_ENTRY → 灵巧手触觉>阈值 → 冻结手套，灵巧手触觉→解算电流→驱动外骨骼舵机
  LOCKED     → 舵机位置稳定 → 舵机切位置控制锁死
               同时：PID(目标力=外骨骼触觉, 反馈力=灵巧手触觉) → 灵巧手位置增量
  RELEASE    → 外骨骼触觉归零持续300ms → 舵机归位，恢复手套控制
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
from typing import List, Tuple, Optional

# ========== Dynamixel SDK ==========
from dynamixel_sdk import (
    PortHandler, PacketHandler, COMM_SUCCESS
)

# ========== BLE ==========
from bleak import BleakClient

# ========== DDS 路径（必须在 import 之前插入）==========
sys.path.insert(0, '/home/pi/dexEXO/ftp/inspire_hand_ws/inspire_hand_sdk/inspire_sdkpy')
sys.path.insert(0, '/home/pi/dexEXO/ftp/inspire_hand_ws/unitree_sdk2_python')
sys.path.insert(0, '/home/pi/dexEXO/ftp/inspire_hand_ws')

# ========== DDS ==========
try:
    from unitree_sdk2py.core.channel import (
        ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
    )
    from inspire_dds import inspire_hand_ctrl
    from inspire_dds._inspire_hand_touch import inspire_hand_touch
    DDS_AVAILABLE = True
except ImportError as _dds_err:
    DDS_AVAILABLE = False
    print(f"[警告] DDS 库未找到({_dds_err})，将以模拟模式运行")

# =====================================================================
# ======================== 用户可调参数 ================================
# =====================================================================

# ===== 通信模式选择 =====
# USE_TCP_BRIDGE = False  → 原始模式：树莓派直连灵巧手（网线），DDS 通过 eth0
# USE_TCP_BRIDGE = True   → 桥接模式：树莓派通过 WiFi 连 G1，G1 上运行 hand_bridge.py
USE_TCP_BRIDGE  = True
BRIDGE_HOST     = "192.168.6.146"   # G1 的 WiFi IP
BRIDGE_CTRL_PORT  = 9100           # 角度指令端口（树莓派→G1）
BRIDGE_TOUCH_PORT = 9101           # 触觉反馈端口（G1→树莓派）

# ===== BLE Broker =====
USE_BROKER    = True
BROKER_HOST   = "127.0.0.1"
BROKER_PORT   = 9001

# ===== BLE 直连（USE_BROKER=False 时使用）=====
DEVICE_ADDRESSES   = ["F0:FD:45:02:85:B3", "F0:FD:45:02:67:3B"]
TX_CHAR_UUID       = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
RX_CHAR_UUID       = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
BLE_KEEPALIVE_INTERVAL = 3.0
BLE_KEEPALIVE_DATA = b"PING"
BLE_DATA_TIMEOUT   = 15.0
BLE_RECONNECT_DELAY = 1.0

# ===== BLE 数据解析 =====
# 18 通道原始数据：前 13 位为弯曲传感器，末 5 位为外骨骼触觉
# 末5位顺序：[拇指, 食指, 中指, 无名指, 小指]
SENSOR_INDEX       = [1, 2, 3, 5, 9, 8]   # 弯曲通道映射 → [小指,无名,中指,食指,拇弯,拇旋]
EXO_FORCE_BASELINE = 4.903                 # 外骨骼BLE触觉零偏(N)，减去后才是真实力

# ===== 灵巧手角度范围 =====
HAND_ANGLE_MIN     = 150    # 伸直（最小值）
HAND_ANGLE_MAX     = 850    # 握拳（最大值）
DEFAULT_BEND_MIN   = 1500
DEFAULT_BEND_MAX   = 4000
CONTROL_HZ         = 20     # 灵巧手控制频率

# ===== EMA 滤波 =====
EMA_ALPHA          = 0.2
USE_FILTER         = True

# ===== DDS 触觉话题（USE_TCP_BRIDGE=False 时使用）=====
HAND_CTRL_TOPIC    = "rt/inspire_hand/ctrl/r"
HAND_TOUCH_TOPIC   = "rt/inspire_hand/touch/r"
# 触觉 raw → N 转换系数
CALIBRATION_K      = 0.00292650244415058227
CALIBRATION_B      = -0.6037947156125716
THUMB_CALIBRATION_K = 0.004420145759358057
THUMB_CALIBRATION_B = -1.0701492398616255
FORCE_MAX_N        = 10.0
# DDS 字段顺序: [拇指, 食指, 中指, 无名指, 小指]
TOUCH_FIELDS       = [
    "fingerfive_top_touch",   # 拇指
    "fingerfour_top_touch",   # 食指
    "fingerthree_top_touch",  # 中指
    "fingertwo_top_touch",    # 无名指
    "fingerone_top_touch",    # 小指
]

# ===== 状态机阈值 =====
HAND_CONTACT_ON_N  = 0.50   # 灵巧手触觉超过此值 → 触发 FORCE_ENTRY
RELEASE_HOLD_SEC   = 2.00   # 灵巧手触觉+外骨骼触觉同时低于阈值后持续此时间 → 切回 GLOVE
EXO_ZERO_THR_N     = 0.10   # 外骨骼触觉判零阈值（N）

# ===== 外骨骼→灵巧手位置 PID（LOCKED 阶段）=====
# 误差 = 外骨骼触觉 - 灵巧手触觉
# Δangle = KP × 误差 (tick/N)，每个控制周期叠加
FORCE_KP           = 35.0   # 比例增益 (tick/N)
FORCE_STEP_MAX     = 3.0    # 单周期最大位置修正 (tick)，避免过冲导致物体滑出
FORCE_DEADZONE_N   = 0.10   # 死区 (N)
# 进入 LOCKED 时将灵巧手角度从当前基准向伸直方向（+850方向）回退此量，为 PID 留出向握拳方向（-150方向）的调节空间
# 150=握拳，850=伸直；若基准是 600，回退后变为 680，PID 可再减小 80tick 到 600 继续握紧
LOCK_ANGLE_RETREAT = 80     # tick（约占全程 700tick 的 11%）

# ===== 手指→角度索引映射 =====
# 灵巧手 angle_set 顺序: [小指, 无名, 中指, 食指, 拇弯, 拇旋]
# 每指对应 angle_set 中的哪个槽位
FINGER_TO_ANGLE_INDEX = {
    0: [4],   # 拇指 → 槽4（拇弯）
    1: [3],   # 食指 → 槽3
    2: [2],   # 中指 → 槽2
    3: [1],   # 无名 → 槽1
    4: [0],   # 小指 → 槽0
}

# ===== Dynamixel 舵机参数 =====
DEVICENAME              = "/dev/ttyAMA0"
BAUDRATE                = 1000000
PROTOCOL_VERSION        = 2.0
DXL_IDS                 = [1, 2, 3, 4, 5]   # 拇/食/中/无/小
NUM_FINGERS             = 5
CURRENT_LIMIT_MA        = 300               # 单舵机最大电流(mA)
TOTAL_CURRENT_LIMIT_MA  = 800.0             # 五指总电流上限(mA)

# 灵巧手触觉→外骨骼舵机电流 (FORCE_ENTRY 阶段)
# 电流(mA) = 灵巧手触觉(N) × HAND_TO_SERVO_GAIN
HAND_TO_SERVO_GAIN      = 60.0  # mA/N

# LOCKED 阶段：外骨骼触觉 → 灵巧手 force_set 映射
# force_set 范围 0~1000，对应灵巧手内部力控强度
# force_set = HAND_FORCE_BASE + exo_force * HAND_FORCE_GAIN（clipped 到 0~1000）
HAND_FORCE_BASE         = 200   # 无外骨骼力时的基础保持力
HAND_FORCE_GAIN         = 40.0  # N → force_set 增益 (units/N)
HAND_FORCE_MAX          = 800   # force_set 上限

# 位置锁定检测
LOCK_POSITION_THR       = 8     # 位置变化阈值(tick)
LOCK_DETECT_WINDOW      = 0.4   # 检测窗口(秒)
LOCK_MIN_DRIVE_TIME     = 0.3   # 最短驱动时间(秒)

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
    GLOVE       = "GLOVE"        # 手套控制灵巧手
    FORCE_ENTRY = "FORCE_ENTRY"  # 触发力控，舵机拉绳
    LOCKED      = "LOCKED"       # 舵机锁死，PID调节灵巧手
    RELEASE     = "RELEASE"      # 舵机归位，恢复手套

# =====================================================================
# ======================== 全局状态 ====================================
# =====================================================================

_hw_error_last_print = {}
running = True

# Dynamixel 硬件
portHandler  = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
dxl_lock     = threading.Lock()

FINGER_NAMES = {0: "拇指", 1: "食指", 2: "中指", 3: "无名指", 4: "小指"}

# 数据手套标定参数（主线程 calibrate_glove() 写入，control_loop 读取）
_bend_min_ref: np.ndarray = np.full(6, DEFAULT_BEND_MIN, dtype=np.float64)
_bend_max_ref: np.ndarray = np.full(6, DEFAULT_BEND_MAX, dtype=np.float64)
_bend_calib_done: bool = False

# =====================================================================
# ======================== 线程安全数据存储 ============================
# =====================================================================

class SharedData:
    """所有跨线程共享数据的统一存储"""

    def __init__(self):
        self._lock = threading.Lock()

        # BLE 弯曲传感器（18通道原始值）
        self.bend_raw: List[float] = [0.0] * 18
        self.bend_updated = False

        # 外骨骼触觉目标力（末5位，已去基线） [拇,食,中,无,小]
        self.exo_force: List[float] = [0.0] * NUM_FINGERS

        # 灵巧手触觉反馈力（DDS） [拇,食,中,无,小]
        self.hand_force: List[float] = [0.0] * NUM_FINGERS

        # BLE 数据时间戳
        self.ble_last_update = 0.0

        # DDS 数据时间戳
        self.dds_last_update = 0.0

    def update_ble(self, all_values: List[float]) -> None:
        """更新 BLE 全部通道数据"""
        with self._lock:
            # 弯曲传感器 (全部18位)
            for i in range(min(18, len(all_values))):
                self.bend_raw[i] = all_values[i]
            # 末5位 → 外骨骼触觉，≤基线置0，>基线保留原始值
            if len(all_values) >= 18:
                for fi in range(NUM_FINGERS):
                    raw = all_values[13 + fi]
                    self.exo_force[fi] = 0.0 if raw <= EXO_FORCE_BASELINE else raw
            elif len(all_values) >= 5:
                for fi in range(NUM_FINGERS):
                    raw = all_values[-(NUM_FINGERS - fi)]
                    self.exo_force[fi] = 0.0 if raw <= EXO_FORCE_BASELINE else raw
            self.bend_raw = list(all_values) if len(all_values) == 18 else self.bend_raw
            self.ble_last_update = time.time()
            self.bend_updated = True

    def update_hand_force(self, forces: List[float]) -> None:
        """更新灵巧手触觉反馈力"""
        with self._lock:
            for i in range(min(NUM_FINGERS, len(forces))):
                self.hand_force[i] = forces[i]
            self.dds_last_update = time.time()

    def get_snapshot(self):
        """原子读取所有数据快照"""
        with self._lock:
            return (
                list(self.bend_raw),
                list(self.exo_force),
                list(self.hand_force),
                self.ble_last_update,
                self.dds_last_update,
            )


shared = SharedData()


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
    从原始BLE文本提取所有数值，支持：
      格式1: {"bend_sensors": [...18个值...]}
      格式2: {v0,v1,...,v17}  花括号逗号分隔
      格式3: [v0,v1,...,v17]  方括号
    返回全部数值列表（不截断），由调用者判断前13/末5
    """
    # 格式1: JSON 标准 bend_sensors
    m = re.search(r'"bend_sensors"\s*:\s*\[([^\]]+)\]', text)
    if m:
        try:
            vals = [float(v.strip()) for v in m.group(1).split(",") if v.strip()]
            if len(vals) >= 5:
                return vals
        except ValueError:
            pass

    # 格式2/3: 花括号或方括号数值序列
    for pattern in [r'\{([0-9.,\s\-]+)\}', r'\[([0-9.,\s\-]+)\]']:
        for m in re.findall(pattern, text):
            parts = [p.strip() for p in m.split(",") if p.strip()]
            try:
                vals = [float(p) for p in parts]
                if len(vals) >= 5:
                    return vals
            except ValueError:
                pass
    return []


def process_ble_text(text: str, parser: JsonStreamParser, raw_buf: list) -> None:
    """处理BLE接收文本，解析并更新 shared"""
    raw_buf[0] += text

    # 先尝试 JSON 解析
    for payload in parser.feed(text):
        # 可能包含 bend_sensors / values / touch_sensors
        for key in ("bend_sensors", "values", "touch_sensors"):
            if key in payload:
                v = payload[key]
                if isinstance(v, list) and len(v) >= 5:
                    try:
                        shared.update_ble([float(x) for x in v])
                        raw_buf[0] = ""
                        return
                    except (TypeError, ValueError):
                        pass

    # 兜底：从原始文本提取
    vals = extract_all_values(raw_buf[0])
    if vals:
        shared.update_ble(vals)
        raw_buf[0] = ""

    if len(raw_buf[0]) > 1024:
        raw_buf[0] = raw_buf[0][-512:]


# =====================================================================
# ======================== DDS 灵巧手触觉接收 ========================
# =====================================================================

def raw_to_force_n(raw: float, is_thumb: bool = False) -> float:
    if is_thumb:
        f = THUMB_CALIBRATION_K * raw + THUMB_CALIBRATION_B
    else:
        f = CALIBRATION_K * raw + CALIBRATION_B
    return float(np.clip(f, 0.0, FORCE_MAX_N))


class HandTouchReceiver:
    """原始模式（USE_TCP_BRIDGE=False）：通过 DDS 直接接收灵巧手触觉"""
    def __init__(self):
        self._sub = None

    def start(self):
        if not DDS_AVAILABLE:
            return
        self._sub = ChannelSubscriber(HAND_TOUCH_TOPIC, inspire_hand_touch)
        self._sub.Init(self._on_touch, 10)

    def _on_touch(self, msg):
        try:
            forces = []
            for i, field in enumerate(TOUCH_FIELDS):
                raw_seq = getattr(msg, field, None)
                if raw_seq is None or len(raw_seq) == 0:
                    raw = 0.0
                else:
                    raw = float(max(raw_seq))   # 取序列最大值作为代表值
                forces.append(raw_to_force_n(raw, is_thumb=(i == 0)))
            shared.update_hand_force(forces)
        except Exception as e:
            print(f"[DDS触觉] 解析错误: {e}")


class TcpTouchReceiver:
    """桥接模式（USE_TCP_BRIDGE=True）：通过 TCP 从 hand_bridge.py 接收触觉数据"""
    def __init__(self):
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        buf = ""
        while running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((BRIDGE_HOST, BRIDGE_TOUCH_PORT))
                sock.settimeout(5.0)
                print(f"[TCP触觉] 已连接 {BRIDGE_HOST}:{BRIDGE_TOUCH_PORT}")
                while running:
                    try:
                        chunk = sock.recv(4096)
                        if not chunk:
                            print("[TCP触觉] 连接断开")
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
                                    forces_raw = pkt.get("forces", [0.0]*6)
                                    # TCP 传来的已经是牛顿值，直接写入
                                    shared.update_hand_force(forces_raw[:5])
                            except json.JSONDecodeError:
                                pass
                    except socket.timeout:
                        continue
                    except Exception as e:
                        print(f"[TCP触觉] 接收错误: {e}")
                        break
            except Exception as e:
                if running:
                    print(f"[TCP触觉] 连接失败({e})，3s 后重试…")
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
        self.values = None

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
    elif e != 0:
        _hw_err(sid, f"[舵机{sid}] 硬件错误: {packetHandler.getRxPacketError(e)}")
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
    """每根手指对应的外骨骼舵机控制器"""

    def __init__(self, finger_idx: int):
        self.fi           = finger_idx
        self.sid          = DXL_IDS[finger_idx]   # 舵机ID (1~5)
        self.name         = FINGER_NAMES[finger_idx]

        self.init_pos: Optional[int] = None
        self.is_initialized = False

        # DRIVING 阶段
        self.drive_start  = 0.0
        self.pos_history: List[Tuple[float, int]] = []  # (time, pos)

        # RELEASE 阶段
        self.release_start = 0.0

        # 待写入电流暂存
        self._pending_mA  = 0.0

    def record_init(self) -> None:
        pos = dxl_read4(self.sid, ADDR_PRESENT_POSITION)
        self.init_pos = pos
        self.is_initialized = True
        print(f"  [{self.name}] 舵机初始位置={pos}")


# =====================================================================
# ======================== 每指独立状态机 ==============================
# =====================================================================

class FingerStateMachine:
    """5根手指的完整状态机"""

    def __init__(self):
        self.modes       = [FingerMode.GLOVE] * NUM_FINGERS
        self.servos      = [ServoCtrl(i) for i in range(NUM_FINGERS)]
        # 力控模式下灵巧手的参考角度（冻结手套时的角度）
        self.force_ref_angles = [500] * 6   # angle_set 6个槽
        # 退出力控计时
        self.release_timer = [None] * NUM_FINGERS

    def init_all_servos(self) -> None:
        print("\n=== 初始化外骨骼舵机 ===")
        for s in self.servos:
            if not s.is_initialized:
                print(f"  [{s.name}] 舵机未 Ping 成功，跳过")
                continue
            s.record_init()
        print("初始化完成")

    def record_init(self) -> None:
        """记录所有舵机当前位置为初始位置"""
        print("\n=== 记录外骨骼舵机初始位置 ===")
        for s in self.servos:
            s.record_init()
        print("记录完成")

    def tick(
        self,
        glove_angles: List[int],
        exo_force: List[float],
        hand_force: List[float],
        now: float,
    ) -> Tuple[List[int], List[int]]:
        """
        每控制周期调用，返回 (angle_set 6槽, force_set 6槽)
        LOCKED 阶段 force_set 按外骨骼力比例增大，其余阶段使用默认值 200
        """
        output  = list(glove_angles)
        forces  = [200] * 6   # 默认灵巧手保持力

        for fi in range(NUM_FINGERS):
            mode    = self.modes[fi]
            srv     = self.servos[fi]
            target  = exo_force[fi]    # 外骨骼触觉（目标力）
            feedback = hand_force[fi]  # 灵巧手触觉（反馈力）
            joint_ids = FINGER_TO_ANGLE_INDEX.get(fi, [])

            # ---- GLOVE ----
            if mode == FingerMode.GLOVE:
                srv._pending_mA = 0.0
                # 灵巧手接触物体 → 进入 FORCE_ENTRY
                if feedback >= HAND_CONTACT_ON_N:
                    self.modes[fi] = FingerMode.FORCE_ENTRY
                    srv.drive_start = now
                    srv.pos_history = []
                    # 冻结当前角度作为力控基准
                    for ji in joint_ids:
                        self.force_ref_angles[ji] = glove_angles[ji]
                    # 切换舵机到电流控制模式
                    set_servo_mode(srv.sid, OP_CURRENT_CONTROL)
                    dxl_write2(srv.sid, ADDR_CURRENT_LIMIT, CURRENT_LIMIT_MA)
                    print(f"[{srv.name}] GLOVE → FORCE_ENTRY  手触觉={feedback:.2f}N")

            # ---- FORCE_ENTRY ----
            elif mode == FingerMode.FORCE_ENTRY:
                # 灵巧手触觉 → 解算外骨骼舵机电流，拉紧绳索
                drive_mA = min(feedback * HAND_TO_SERVO_GAIN, float(CURRENT_LIMIT_MA))
                srv._pending_mA = drive_mA
                # 灵巧手冻结
                for ji in joint_ids:
                    output[ji] = self.force_ref_angles[ji]

                # 检测舵机位置是否稳定 → LOCKED
                pos = dxl_read4(srv.sid, ADDR_PRESENT_POSITION)
                _update_pos_history(srv, now, pos)
                if _is_position_stable(srv, now):
                    _enter_locked(srv, pos)
                    self.modes[fi] = FingerMode.LOCKED
                    # 将力控基准角度向伸直方向（850）回退 LOCK_ANGLE_RETREAT，
                    # 为 PID 留出向握拳方向（150）的调节空间
                    for ji in joint_ids:
                        self.force_ref_angles[ji] = int(np.clip(
                            self.force_ref_angles[ji] + LOCK_ANGLE_RETREAT,
                            HAND_ANGLE_MIN, HAND_ANGLE_MAX
                        ))
                    print(f"[{srv.name}] FORCE_ENTRY → LOCKED  位置={pos}  "
                          f"基准角度回退至{self.force_ref_angles[joint_ids[0]] if joint_ids else '?'}")

            # ---- LOCKED ----
            elif mode == FingerMode.LOCKED:
                srv._pending_mA = 0.0  # 位置控制模式，无需写电流
                # PID：灵巧手实际力(feedback) < 人期望力(target) → err>0 → 继续握紧 → 角度减小（向 150 握拳方向）
                # 150=握拳，850=伸直；err>0 → step>0 → 角度减小 → 握紧；err<0 → 松开
                err = target - feedback
                if abs(err) > FORCE_DEADZONE_N:
                    step = float(np.clip(FORCE_KP * err, -FORCE_STEP_MAX, FORCE_STEP_MAX))
                    for ji in joint_ids:
                        self.force_ref_angles[ji] = int(np.clip(
                            self.force_ref_angles[ji] - step,   # 减法：err>0 → 角度减小 → 握拳
                            HAND_ANGLE_MIN, HAND_ANGLE_MAX
                        ))
                for ji in joint_ids:
                    output[ji] = self.force_ref_angles[ji]

                # LOCKED 阶段：外骨骼触觉 → 灵巧手 force_set（内部力控持续施力）
                # exo=0N → 200, exo=5N → 400, exo=15N → 800（上限）
                locked_force = int(np.clip(
                    HAND_FORCE_BASE + target * HAND_FORCE_GAIN,
                    HAND_FORCE_BASE, HAND_FORCE_MAX
                ))
                for ji in joint_ids:
                    forces[ji] = locked_force

                # 外骨骼触觉 < 阈值 → 开始计时，持续 RELEASE_HOLD_SEC 后释放
                if target <= EXO_ZERO_THR_N:
                    if self.release_timer[fi] is None:
                        self.release_timer[fi] = now
                    elif now - self.release_timer[fi] >= RELEASE_HOLD_SEC:
                        # 触发释放
                        self.release_timer[fi] = None
                        self.modes[fi] = FingerMode.RELEASE
                        _enter_releasing(srv, now)
                        # 本周期先跟手套
                        for ji in joint_ids:
                            output[ji] = glove_angles[ji]
                            self.force_ref_angles[ji] = glove_angles[ji]
                        print(f"[{srv.name}] LOCKED → RELEASE  exo={target:.2f}N < {EXO_ZERO_THR_N}N")
                else:
                    self.release_timer[fi] = None

            # ---- RELEASE ----
            elif mode == FingerMode.RELEASE:
                # 手套角度正常输出
                for ji in joint_ids:
                    output[ji] = glove_angles[ji]
                # 舵机归位
                done = _tick_releasing(srv, now)
                if done:
                    self.modes[fi] = FingerMode.GLOVE
                    print(f"[{srv.name}] RELEASE → GLOVE")

        return output, forces

    def flush_servo_currents(self) -> None:
        """统一写入所有舵机电流（含总电流限制）"""
        # FORCE_ENTRY 阶段舵机才有正电流，其余为0
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
                mA = s._pending_mA * scale
                write_servo_current(s.sid, mA)
            elif mode == FingerMode.RELEASE:
                write_servo_current(s.sid, s._pending_mA)
            # LOCKED: 位置控制模式，不写电流
            # GLOVE: 0电流由进入GLOVE时已写入


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
    """切换到位置控制模式，锁死在当前位置"""
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
    """归位控制，返回 True 表示归位完成"""
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
        # 切回位置模式稳定
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

    # 减速区
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
# ======================== 角度转换函数 ================================
# =====================================================================

def calibrate_glove(duration_sec: float = 2.0) -> None:
    """
    交互式数据手套标定：采集握拳和伸直参考值。
    阻塞主线程，需在 BLE 线程启动后、control_loop 启动前调用。
    """
    global _bend_min_ref, _bend_max_ref, _bend_calib_done

    # 等待 BLE 数据就绪
    print("\n[标定] 等待 BLE 手套数据…（最多 30s）")
    t0 = time.time()
    while time.time() - t0 < 30.0:
        _, _, _, ble_t, _ = shared.get_snapshot()
        if ble_t > 0:
            break
        time.sleep(0.1)
    else:
        print("[标定] 超时未收到 BLE 数据，使用默认标定值（min=1500 max=4000）")
        return

    def collect(pose_name: str) -> np.ndarray:
        try:
            input(f"\n请做出【{pose_name}】姿态，然后按回车开始采集…")
        except EOFError:
            print(f"\n[标定] 无交互输入，立即采集 {pose_name}")
        print(f"[标定] 正在采集 {pose_name} {duration_sec:.1f}s…")
        samples: List[np.ndarray] = []
        t_start = time.time()
        while time.time() - t_start < duration_sec:
            bend_raw, _, _, ble_t, _ = shared.get_snapshot()
            if ble_t > 0:
                raw_arr = np.array(bend_raw, dtype=np.float64)
                values = raw_arr[list(SENSOR_INDEX)]
                samples.append(values.copy())
                print(f"\r[标定] {pose_name}: {values.astype(int).tolist()}", end="", flush=True)
            time.sleep(0.02)
        print()
        if not samples:
            raise RuntimeError(f"标定失败：{pose_name} 没有采集到数据")
        ref = np.median(np.vstack(samples), axis=0)
        print(f"[标定] {pose_name} 参考值: {ref.astype(int).tolist()}")
        return ref

    bend_max = collect("握拳(弯曲极限)")
    bend_min = collect("伸直(伸直极限)")

    for i in range(len(SENSOR_INDEX)):
        if abs(bend_max[i] - bend_min[i]) < 100:
            print(
                f"[标定][WARN] 通道 {SENSOR_INDEX[i]} 范围太小: "
                f"min={bend_min[i]:.0f} max={bend_max[i]:.0f}，该通道映射可能不准确"
            )

    _bend_min_ref = bend_min
    _bend_max_ref = bend_max
    _bend_calib_done = True
    print("[标定] 数据手套标定完成 ✓")


def bend_to_angle(raw_values: np.ndarray, bend_min: np.ndarray, bend_max: np.ndarray) -> List[int]:
    """弯曲传感器原始值 → 灵巧手 angle_set（6个槽）"""
    angles = []
    for i, ch in enumerate(SENSOR_INDEX):
        raw = raw_values[ch] if ch < len(raw_values) else 0.0
        rng = float(bend_max[i] - bend_min[i])
        if rng < 1:
            angles.append(int((HAND_ANGLE_MIN + HAND_ANGLE_MAX) / 2))
            continue
        # 传感器值越大 → 手指伸直 → 角度越小（150=伸直，850=握拳）
        ratio = (raw - bend_min[i]) / rng
        ratio = float(np.clip(ratio, 0.0, 1.0))
        angle = int(HAND_ANGLE_MAX - ratio * (HAND_ANGLE_MAX - HAND_ANGLE_MIN))
        angles.append(angle)
    return angles


# =====================================================================
# ======================== Dynamixel 硬件初始化 ========================
# =====================================================================

def setup_servos(fsm: FingerStateMachine) -> None:
    if not portHandler.openPort():
        print("[错误] 舵机串口打开失败，跳过舵机初始化")
        return
    if not portHandler.setBaudRate(BAUDRATE):
        print("[错误] 波特率设置失败")
        return
    print(f"[舵机] 串口 {DEVICENAME} @ {BAUDRATE}bps 已开启")

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
# ======================== 主控制循环（线程）==========================
# =====================================================================

def control_loop(fsm: FingerStateMachine) -> None:
    """以 CONTROL_HZ 频率运行，处理灵巧手控制和舵机状态机"""
    period = 1.0 / CONTROL_HZ
    last_print = time.time()

    # 使用主线程标定结果（或默认值）
    bend_min_ref = _bend_min_ref.copy()
    bend_max_ref = _bend_max_ref.copy()
    ema = MultiEMA(channels=18, alpha=EMA_ALPHA)

    # ── 原始模式（USE_TCP_BRIDGE=False）：DDS 直接发布 ──────────────
    pubr = None
    if not USE_TCP_BRIDGE and DDS_AVAILABLE:
        pubr = ChannelPublisher(HAND_CTRL_TOPIC, inspire_hand_ctrl)
        pubr.Init()
        print("[控制循环] 使用 DDS 直接发布模式")

    # ── 桥接模式（USE_TCP_BRIDGE=True）：TCP 发送到 hand_bridge.py ──
    tcp_ctrl_sock = None
    if USE_TCP_BRIDGE:
        print(f"[控制循环] 使用 TCP 桥接模式 → {BRIDGE_HOST}:{BRIDGE_CTRL_PORT}")

    def _get_tcp_ctrl():
        """获取或重建 TCP 控制连接"""
        nonlocal tcp_ctrl_sock
        if tcp_ctrl_sock is not None:
            return tcp_ctrl_sock
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2.0)
            s.connect((BRIDGE_HOST, BRIDGE_CTRL_PORT))
            tcp_ctrl_sock = s
            print(f"[TCP控制] 已连接 {BRIDGE_HOST}:{BRIDGE_CTRL_PORT}")
        except Exception as e:
            print(f"[TCP控制] 连接失败: {e}")
            tcp_ctrl_sock = None
        return tcp_ctrl_sock

    while running:
        t0 = time.time()
        try:
            bend_raw, exo_force, hand_force, ble_t, dds_t = shared.get_snapshot()
            ble_age = time.time() - ble_t if ble_t > 0 else float("inf")

            if ble_age > 5.0:
                # BLE 超时，维持当前状态，不推进
                time.sleep(period)
                continue

            # EMA 滤波弯曲值
            raw_arr = np.array(bend_raw, dtype=np.float64)
            if USE_FILTER:
                filtered = ema.update(raw_arr)
            else:
                filtered = raw_arr

            # 弯曲 → 灵巧手角度（手套模式基准）
            glove_angles = bend_to_angle(filtered, bend_min_ref, bend_max_ref)

            # 每指状态机
            now = t0
            output_angles, hand_force_set = fsm.tick(glove_angles, exo_force, hand_force, now)

            # 写入舵机电流
            fsm.flush_servo_currents()

            # ── 发送灵巧手指令 ──────────────────────────────────────
            if USE_TCP_BRIDGE:
                # 桥接模式：JSON 通过 TCP 发给 G1 上的 hand_bridge.py
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
                        print(f"[TCP控制] 发送失败({e})，重连…")
                        try:
                            tcp_ctrl_sock.close()
                        except Exception:
                            pass
                        tcp_ctrl_sock = None
            else:
                # 原始模式：DDS 直接发布（树莓派网线直连灵巧手）
                if pubr is not None:
                    msg = inspire_hand_ctrl(
                        pos_set   = [0] * 6,
                        angle_set = tuple(int(a) for a in output_angles),
                        force_set = hand_force_set,
                        speed_set = [500] * 6,
                        mode      = 1,
                    )
                    pubr.Write(msg)

            # 状态打印
            if time.time() - last_print >= DEBUG_PRINT_INTERVAL:
                modes_str = " ".join(
                    f"{FINGER_NAMES[i]}:{fsm.modes[i].value}" for i in range(NUM_FINGERS)
                )
                exo_str   = " ".join(f"{v:.2f}" for v in exo_force)
                hand_str  = " ".join(f"{v:.2f}" for v in hand_force)
                ang_str   = " ".join(str(a) for a in output_angles)
                force_str = " ".join(str(f) for f in hand_force_set)
                dds_ok    = "发送中" if pubr is not None else "未连接(DDS不可用)"
                print(f"[状态] {modes_str}")
                print(f"  手套角度(angle_set): [{ang_str}]  DDS: {dds_ok}")
                print(f"  外骨骼目标力: [{exo_str}] N")
                print(f"  灵巧手反馈力: [{hand_str}] N")
                print(f"  灵巧手力控指令(force_set): [{force_str}]")
                last_print = time.time()

        except Exception as e:
            print(f"[控制循环错误] {e}")

        elapsed = time.time() - t0
        sleep_t = period - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)


# =====================================================================
# ======================== BLE 接收 ====================================
# =====================================================================

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
                    process_ble_text(chunk.decode("utf-8", errors="ignore"), parser, raw_buf)
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
    disc = asyncio.Event()

    def on_disc(_):
        disc.set()

    async with BleakClient(address, disconnected_callback=on_disc, timeout=15.0) as c:
        if not c.is_connected:
            raise RuntimeError(f"无法连接 {address}")
        print(f"[BLE] 已连接 {address}")
        last_data = time.time()

        def notify(_, data):
            nonlocal last_data
            last_data = time.time()
            process_ble_text(bytes(data).decode("utf-8", errors="ignore"), parser, raw_buf)

        await c.start_notify(RX_CHAR_UUID, notify)
        last_ka = time.time()
        while running:
            if disc.is_set() or not c.is_connected:
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
                print(f"[BLE] {addr} 失败({retry}): {e}，{delay:.1f}s 后重试")
                await asyncio.sleep(delay)
        if not running:
            break


def ble_thread_fn() -> None:
    if USE_BROKER:
        ble_broker_client()
    else:
        asyncio.run(_ble_main_direct())


# =====================================================================
# ======================== 命令输入 ====================================
# =====================================================================

def input_thread_fn(fsm: FingerStateMachine) -> None:
    print("=== 命令就绪 (输入 HELP 查看指令) ===")
    while running:
        try:
            line = sys.stdin.readline()
            if not line:
                break
            line = line.strip()
            if not line:
                continue
            parse_cmd(line, fsm)
        except Exception as e:
            print(f"[输入错误] {e}")


def parse_cmd(raw: str, fsm: FingerStateMachine) -> None:
    global running
    buf = raw.replace("：", ":").strip().upper()
    try:
        if buf == "INIT":
            fsm.record_init()
            print("  [提示] INIT 已重新记录舵机位置（可随时重新标定位置）")

        elif buf == "STATUS":
            bend_raw, exo_force, hand_force, ble_t, dds_t = shared.get_snapshot()
            ble_age = time.time() - ble_t if ble_t > 0 else float("inf")
            dds_age = time.time() - dds_t if dds_t > 0 else float("inf")
            print(f"\n=== 系统状态 (BLE age={ble_age:.1f}s  DDS age={dds_age:.1f}s) ===")
            for fi in range(NUM_FINGERS):
                s = fsm.servos[fi]
                mode = fsm.modes[fi]
                pos = dxl_read4(s.sid, ADDR_PRESENT_POSITION) if s.is_initialized else -1
                cur = dxl_read2(s.sid, ADDR_PRESENT_CURRENT) if s.is_initialized else 0
                print(
                    f"  {s.name}: [{mode.value:12s}] "
                    f"pos={pos:5d} I={cur:4d}mA "
                    f"exo={exo_force[fi]:.2f}N hand={hand_force[fi]:.2f}N"
                )

        elif buf == "HELP":
            print("""
=== Force_handcontrol 指令 ===
INIT      - 记录所有外骨骼舵机初始位置（程序启动后必须执行一次）
STATUS    - 查看每指详细状态
EXIT      - 退出程序
HELP      - 显示此帮助

状态说明:
  GLOVE       - 手套控制灵巧手，舵机自由
  FORCE_ENTRY - 灵巧手触觉驱动舵机拉绳
  LOCKED      - 舵机锁死，PID调节灵巧手抓力
  RELEASE     - 舵机归位，等待回到GLOVE
""")

        elif buf == "EXIT":
            running = False

        else:
            print(f"[未知指令] {buf}  (输入 HELP)")

    except Exception as e:
        print(f"[命令错误] {e}")


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
    print("   Force_handcontrol.py  灵巧手力反馈双边遥操作")
    print("   版本: V2   日期: 2026-04-25")
    print("=" * 60)

    # ── 第一步：舵机初始化（硬件优先） ──────────────────────────────
    print("\n[步骤 1/4] 舵机初始化…")
    fsm = FingerStateMachine()
    setup_servos(fsm)

    # ── 第二步：数据手套 BLE 初始化 ─────────────────────────────────
    print("\n[步骤 2/4] 数据手套 BLE 初始化…")
    if USE_BROKER:
        print(f"  模式: Broker TCP  {BROKER_HOST}:{BROKER_PORT}")
    else:
        print(f"  模式: BLE 直连    {DEVICE_ADDRESSES}")
    ble_thr = threading.Thread(target=ble_thread_fn, daemon=True)
    ble_thr.start()

    # ── 第三步：等待用户输入 INIT，然后进行数据手套标定 ──────────────
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
        if cmd == "INIT":
            fsm.record_init()
            print("\n[步骤 3/4] INIT 完成，开始数据手套标定…")
            calibrate_glove(duration_sec=2.0)
            break
        elif cmd == "EXIT":
            running = False
            break
        elif cmd == "HELP":
            print("  此阶段只接受 INIT 指令（记录舵机初始位置后继续启动）")
        else:
            print(f"  [提示] 请先输入 INIT 完成初始化（当前输入: {cmd}）")

    if not running:
        sys.exit(0)

    # ── 第四步：通信初始化 + 启动控制线程 ───────────────────────────
    if USE_TCP_BRIDGE:
        # 桥接模式：TCP 连接 G1 hand_bridge.py，不需要本地 DDS
        print("\n[步骤 4/4] TCP 桥接模式初始化…")
        print(f"  G1 地址: {BRIDGE_HOST}")
        print(f"  控制端口: {BRIDGE_CTRL_PORT}  触觉端口: {BRIDGE_TOUCH_PORT}")
        touch_rx = TcpTouchReceiver()
        touch_rx.start()
        print("  [TCP] 触觉接收线程已启动，等待 G1 连接…")
    else:
        # 原始模式：DDS 直连（树莓派网线直连灵巧手）
        print("\n[步骤 4/4] DDS 直连模式初始化…")
        if DDS_AVAILABLE:
            if len(sys.argv) > 1:
                ChannelFactoryInitialize(0, sys.argv[1])
            else:
                ChannelFactoryInitialize(0)
            print("  [DDS] ChannelFactory 初始化完成")
        else:
            print("  [DDS] 库未找到，以模拟模式运行（灵巧手触觉不可用）")
        touch_rx = HandTouchReceiver()
        touch_rx.start()

    threading.Thread(target=control_loop, args=(fsm,), daemon=True).start()
    threading.Thread(target=input_thread_fn, args=(fsm,), daemon=True).start()

    # ── 打印启动摘要 ─────────────────────────────────────────────────
    initialized_servos = [s.name for s in fsm.servos if s.is_initialized]
    print("\n" + "=" * 60)
    print("  ✓ 系统就绪（舵机已初始化 + 数据手套已标定）")
    print(f"  ✓ 已初始化舵机   : {', '.join(initialized_servos) if initialized_servos else '无（检查串口）'}")
    print(f"  ✓ BLE 手套模式   : {'Broker' if USE_BROKER else '直连'}")
    print(f"  ✓ DDS 触觉       : {'可用' if DDS_AVAILABLE else '模拟模式'}")
    print(f"  ✓ 控制频率       : {CONTROL_HZ} Hz")
    print(f"  ✓ 外骨骼力基线   : {EXO_FORCE_BASELINE} N（≤基线视为0）")
    print(f"  ✓ 手套标定       : {'已完成' if _bend_calib_done else '使用默认值'}")
    print("=" * 60)
    bend_raw, exo_force, hand_force, _, _ = shared.get_snapshot()
    print("\n[状态] " + "  ".join(
        f"{FINGER_NAMES[i]}:{fsm.modes[i].value}" for i in range(NUM_FINGERS)
    ))
    print(f"  外骨骼目标力: [{' '.join(f'{v:.2f}' for v in exo_force)}] N")
    print(f"  灵巧手反馈力: [{' '.join(f'{v:.2f}' for v in hand_force)}] N")
    print("\n输入 STATUS / HELP / EXIT 进行操作\n")

    try:
        while running:
            time.sleep(0.5)
    except KeyboardInterrupt:
        running = False
    finally:
        print("\n关闭所有舵机…")
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
