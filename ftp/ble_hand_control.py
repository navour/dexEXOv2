#!/usr/bin/env python3
"""
BLE 直连灵巧手控制
跳过 HDF5，蓝牙收到 bend_sensors → 直接映射角度 → DDS 控制灵巧手
"""

import asyncio
import json
import numpy as np
import time
import sys

# ==================== BLE 配置 ====================
DEVICE_ADDRESSES = [
    "F0:FD:45:02:85:B3",  # 设备0 MAC地址
]
TX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
RX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
DEBUG_RAW_DATA = False
MAX_JSON_BUFFER_BYTES = 32768
BLE_KEEPALIVE_INTERVAL = 3.0
BLE_KEEPALIVE_DATA = b"PING"

# ===== BLE Broker 配置 =====
# USE_BROKER=True: 连接本地 ble_broker.py 中转（与 finger_force.py 共用蓝牙时使用）
# USE_BROKER=False: 直接连接 BLE 设备（单独运行时使用）
USE_BROKER = True
BROKER_HOST = "127.0.0.1"
BROKER_PORT = 9001
# ==================================================

# ==================== 手部控制配置 ====================
HAND_ANGLE_MIN = 150          # 灵巧手伸直
HAND_ANGLE_MAX = 850          # 灵巧手弯曲
DEFAULT_BEND_MIN = 1500       # 未标定时后备参数
DEFAULT_BEND_MAX = 4000       # 未标定时后备参数
CONTROL_HZ = 20               # 控制频率 20Hz
CALIBRATION_DURATION = 2.0    # 每个姿态采样时长(秒)

# ========== 滤波器配置 ==========
EMA_ALPHA = 0.2               # EMA滤波系数 (0.1~0.5)，越小越平滑，越大越灵敏
USE_FILTER = True             # 是否启用滤波
# ==================================================

# ========== 传感器通道映射 ==========
# bend_sensors 有 18 个通道 (0~17)，从中选择 6 个映射到灵巧手 6 个关节
# 格式: [小指, 无名指, 中指, 食指, 拇指弯曲, 拇指旋转]
SENSOR_INDEX = [2,1,3,5,9,8]
# ==================================================


# ---------- DDS 导入 ----------
sys.path.insert(0, '/home/pi/dexEXO/ftp/inspire_hand_ws/inspire_hand_sdk/inspire_sdkpy')
sys.path.insert(0, '/home/pi/dexEXO/ftp/inspire_hand_ws/unitree_sdk2_python')
sys.path.insert(0, '/home/pi/dexEXO/ftp/inspire_hand_ws')

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from inspire_dds import inspire_hand_ctrl


# ==================== 数据解析 ====================
def extract_complete_json_frames(buffer):
    """从字节缓冲区中提取完整JSON对象（支持嵌套括号），保留未完成尾包。
    与 gatt_blu_251202.py 中的 _extract_complete_json_frames 逻辑一致。"""
    frames = []
    start = None
    depth = 0
    in_string = False
    escape = False

    for idx, byte_val in enumerate(buffer):
        if start is None:
            if byte_val == ord('{'):
                start = idx
                depth = 1
                in_string = False
                escape = False
            continue

        if in_string:
            if escape:
                escape = False
            elif byte_val == ord('\\'):
                escape = True
            elif byte_val == ord('"'):
                in_string = False
            continue

        if byte_val == ord('"'):
            in_string = True
        elif byte_val == ord('{'):
            depth += 1
        elif byte_val == ord('}'):
            depth -= 1
            if depth == 0:
                frames.append(bytes(buffer[start:idx + 1]))
                start = None

    remaining = bytearray(buffer[start:]) if start is not None else bytearray()
    return frames, remaining


def parse_bend_sensors(raw_data):
    """
    从原始数据中提取 bend_sensors 数组（支持两种格式）：
    1. JSON 格式: {"bend_sensors": [18个数据]}
    2. 纯数据格式: {18个数据} 或 value1,value2,...
    """
    # 尝试解析为 JSON（格式1）
    if isinstance(raw_data, dict):
        json_data = raw_data
    else:
        try:
            json_data = json.loads(raw_data) if isinstance(raw_data, str) else None
        except (json.JSONDecodeError, TypeError):
            json_data = None

    # 格式1: JSON 格式 {"bend_sensors": [...]}
    if json_data and 'bend_sensors' in json_data:
        if isinstance(json_data['bend_sensors'], list) and len(json_data['bend_sensors']) == 18:
            arr = np.asarray(json_data['bend_sensors'], dtype=np.int64)
            arr = np.mod(arr, 65536).astype(np.uint16)
            return arr
        return None

    # 格式2: 纯数据格式 {val1,val2,...} 或 [val1,val2,...]
    if isinstance(raw_data, (str, bytes, bytearray)):
        # 去掉花括号，提取数字
        text = raw_data.decode('utf-8') if isinstance(raw_data, (bytes, bytearray)) else raw_data
        text = text.strip()
        if text.startswith('{') and text.endswith('}'):
            text = text[1:-1]
        if text.startswith('[') and text.endswith(']'):
            text = text[1:-1]

        parts = text.replace(' ', '').split(',')
        if len(parts) == 18:
            # 支持整数和浮点数
            def to_int(val):
                try:
                    return int(float(val))  # 处理浮点数如 "4.9"
                except ValueError:
                    return 0
            values = [to_int(p) for p in parts]
            arr = np.asarray(values, dtype=np.int64)
            arr = np.mod(arr, 65536).astype(np.uint16)
            return arr

    # 如果已经是 list（直接传入的数组）
    if isinstance(raw_data, (list, np.ndarray)) and len(raw_data) == 18:
        arr = np.asarray(raw_data, dtype=np.int64)
        arr = np.mod(arr, 65536).astype(np.uint16)
        return arr

    return None


# ==================== EMA 滤波器 ====================
class EMAFilter:
    """指数加权平均滤波器，用于平滑传感器数据"""
    def __init__(self, alpha=0.3):
        self.alpha = alpha  # 滤波系数，越小越平滑
        self.value = None   # 滤波后的值

    def update(self, new_value):
        """更新滤波值"""
        if self.value is None:
            self.value = np.asarray(new_value, dtype=np.float64)
        else:
            self.value = self.alpha * np.asarray(new_value, dtype=np.float64) + \
                         (1 - self.alpha) * self.value
        return self.value

    def reset(self):
        """重置滤波器"""
        self.value = None


class MultiEMAFilter:
    """多通道 EMA 滤波器"""
    def __init__(self, channels=6, alpha=0.3):
        self.filters = [EMAFilter(alpha) for _ in range(channels)]

    def update(self, values):
        """批量更新所有通道，返回 numpy 数组"""
        values = np.asarray(values, dtype=np.float64)
        for f, v in zip(self.filters, values):
            f.update(v)
        return np.array([f.value for f in self.filters], dtype=np.float64)

    def reset(self):
        """重置所有滤波器"""
        for f in self.filters:
            f.reset()
# ==================================================


def bend_to_angle(values, bend_min_ref, bend_max_ref):
    """弯曲传感器值 → 灵巧手角度"""
    values = np.asarray(values, dtype=np.float64)
    bend_min_ref = np.asarray(bend_min_ref, dtype=np.float64)
    bend_max_ref = np.asarray(bend_max_ref, dtype=np.float64)

    def map_value(val, vmin, vmax):
        span = float(vmax - vmin)
        if abs(span) < 1e-6:
            # 标定失败（min≈max），用当前值相对于传感器量程映射
            vmin = 0.0
            vmax = 10000.0
            span = vmax - vmin
        norm = (float(val) - float(vmin)) / span
        norm = float(np.clip(norm, 0.0, 1.0))
        # 方向取反：传感器值大 → 伸直(200)，传感器值小 → 弯曲(800)
        norm = 1.0 - norm
        return int(HAND_ANGLE_MIN + norm * (HAND_ANGLE_MAX - HAND_ANGLE_MIN))

    return [map_value(v, mn, mx) for v, mn, mx in zip(values, bend_min_ref, bend_max_ref)]


def build_cmd(angles, mode=0b0001):
    """构建 DDS 控制命令（inspire_hand_ctrl 是 dataclass）"""
    return inspire_hand_ctrl(
        pos_set=[0, 0, 0, 0, 0, 0],
        angle_set=tuple(angles),
        force_set=[200, 200, 200, 200, 200, 200],
        speed_set=[500, 500, 500, 500, 500, 500],
        mode=mode
    )


# ==================== 标定 ====================
async def calibrate(bend_callback, duration_sec):
    """
    标定：采集握拳和伸直的 bend_sensors 参考值（async 版本，不阻塞事件循环）
    bend_callback: callable，返回最新的 bend_sensors (array[18]) 或 None
    """
    loop = asyncio.get_event_loop()

    async def collect(pose_name):
        try:
            # input() 必须放到线程池执行，否则会阻塞事件循环导致 BLE 通知无法接收
            await loop.run_in_executor(None, input, f"\n请做出【{pose_name}】姿态，然后按回车开始采集...")
        except EOFError:
            print(f"\n[CALIB] 无交互输入，立即开始采集 {pose_name}")

        print(f"[CALIB] 正在采集 {pose_name} 数据 {duration_sec:.1f}s")
        samples = []
        start = time.time()

        while time.time() - start < duration_sec:
            bend = bend_callback()
            if bend is not None:
                values = bend[SENSOR_INDEX]
                samples.append(values)
                print(f"\r[CALIB] {pose_name}: {values.astype(int).tolist()}", end='', flush=True)
            else:
                print(f"\r[CALIB] {pose_name}: 等待数据...", end='', flush=True)
            # 必须用 await asyncio.sleep，让事件循环有机会处理 BLE 通知
            await asyncio.sleep(0.02)

        print()
        if not samples:
            raise RuntimeError(f"标定失败：{pose_name} 没有采集到数据")
        arr = np.vstack(samples)
        ref = np.median(arr, axis=0)
        print(f"[CALIB] {pose_name} 标定值: {ref.astype(int).tolist()}")
        return ref

    bend_max_ref = await collect("握拳(弯曲极限)")
    bend_min_ref = await collect("伸直(伸直极限)")

    # 校验标定结果：对 min≈max 的通道发出警告
    for i in range(6):
        if abs(bend_max_ref[i] - bend_min_ref[i]) < 100:
            print(f"[WARN] 通道 {SENSOR_INDEX[i]} 标定范围太小: "
                  f"min={bend_min_ref[i]:.0f} max={bend_max_ref[i]:.0f}，该通道映射可能不准确")

    return bend_min_ref, bend_max_ref


# ==================== BLE 数据接收器 ====================
class BendSensorReceiver:
    """BLE 通知接收器，解析 bend_sensors"""

    def __init__(self):
        self.latest_bend = None
        self.byte_buffer = bytearray()
        self.packet_count = 0

    def on_notification(self, device_id, data):
        """BLE 通知回调"""
        self.byte_buffer.extend(data)
        self._try_parse(device_id)

    def _try_parse(self, device_id):
        """尝试从缓冲区解析数据（支持 JSON 和纯数据格式）"""
        frames, remaining = extract_complete_json_frames(self.byte_buffer)
        self.byte_buffer = remaining

        for frame in frames:
            try:
                text = frame.decode('utf-8').strip()
                # 尝试解析为纯数据格式 {val1,val2,...}
                if text.startswith('{') and text.endswith('}'):
                    inner = text[1:-1]
                    parts = inner.replace(' ', '').split(',')
                    if len(parts) == 18:
                        # 纯数据格式，支持整数和浮点数
                        def to_int(val):
                            try:
                                return int(float(val))  # 处理浮点数如 "4.9"
                            except ValueError:
                                return 0
                        values = [to_int(p) for p in parts]
                        bend = np.asarray(values, dtype=np.int64)
                        bend = np.mod(bend, 65536).astype(np.uint16)
                        self.latest_bend = bend
                        self.packet_count += 1
                        if DEBUG_RAW_DATA:
                            print(f"[BLE] 纯数据格式: {values}")
                        continue

                # 尝试 JSON 格式
                json_data = json.loads(text)
                if DEBUG_RAW_DATA:
                    print(f"[BLE] 收到数据: {list(json_data.keys())}")

                bend = parse_bend_sensors(json_data)
                if bend is not None:
                    self.latest_bend = bend
                    self.packet_count += 1

            except json.JSONDecodeError:
                continue
            except (ValueError, KeyError) as e:
                print(f"[BLE] 解析错误: {e}")
                continue
            except Exception as e:
                print(f"[BLE] 解析错误: {e}")

        # 防止缓冲区过大
        if len(self.byte_buffer) > MAX_JSON_BUFFER_BYTES and b'{' not in self.byte_buffer:
            self.byte_buffer.clear()

    def get_latest(self):
        """获取最新的 bend_sensors"""
        return self.latest_bend


# ==================== 主程序 ====================
async def main():
    import socket as _socket

    # 1. 初始化 DDS
    print("[DDS] 正在初始化...")
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    pubr = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_hand_ctrl)
    pubr.Init()
    print("[DDS] 已连接右手控制器")

    # 2. 连接 BLE（Broker 模式或直连模式）
    receiver = BendSensorReceiver()

    if USE_BROKER:
        # ===== Broker 模式：连接本地 TCP 代理 =====
        print(f"[BLE] 连接 Broker {BROKER_HOST}:{BROKER_PORT}...")

        broker_sock = None
        for attempt in range(10):
            try:
                broker_sock = _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM)
                broker_sock.connect((BROKER_HOST, BROKER_PORT))
                broker_sock.settimeout(5.0)
                print(f"[BLE] 已连接 Broker")
                break
            except Exception as e:
                print(f"[BLE] 连接 Broker 失败 (第{attempt+1}次): {e}，3s 后重试...")
                await asyncio.sleep(3)
                broker_sock = None
        if broker_sock is None:
            print("[BLE] 无法连接 Broker，请先启动 ble_broker.py")
            return

        # 在后台线程持续接收 Broker 数据并喂给 receiver
        def broker_recv_loop():
            buf = bytearray()
            while True:
                try:
                    chunk = broker_sock.recv(1024)
                    if not chunk:
                        break
                    buf.extend(chunk)
                    # 将原始字节喂给 BendSensorReceiver（与直连模式完全一致）
                    receiver.on_notification(0, bytearray(chunk))
                    if len(buf) > MAX_JSON_BUFFER_BYTES:
                        buf = buf[-MAX_JSON_BUFFER_BYTES:]
                except _socket.timeout:
                    continue
                except Exception:
                    break

        import threading as _threading
        t_recv = _threading.Thread(target=broker_recv_loop, daemon=True)
        t_recv.start()

        # 等待第一笔数据到达
        print("[BLE] 等待传感器数据（来自 Broker）...")
        wait_start = time.time()
        while receiver.latest_bend is None:
            if time.time() - wait_start > 30:
                print("[BLE] 超时：30秒内未收到数据，请检查 ble_broker.py 是否已连接蓝牙")
                broker_sock.close()
                return
            await asyncio.sleep(0.1)
        print(f"[BLE] 数据到达！通道数: {len(receiver.latest_bend)}")

        # Broker 模式下心跳由 ble_broker.py 负责，client 设为 None
        client = None

    else:
        # ===== 直连模式（单独运行时使用）=====
        from bleak import BleakClient
        address = DEVICE_ADDRESSES[0]
        print(f"[BLE] 直连模式，正在连接 {address}...")
        try:
            client = BleakClient(address, timeout=30.0)
            await client.connect()
            await client.start_notify(RX_CHAR_UUID,
                                      lambda sender, data: receiver.on_notification(0, data))
            print(f"[BLE] 连接成功，已订阅通知")
        except Exception as e:
            print(f"[BLE] 连接失败: {e}")
            return

        # 等待第一笔数据到达
        print("[BLE] 等待传感器数据...")
        wait_start = time.time()
        while receiver.latest_bend is None:
            if time.time() - wait_start > 30:
                print("[BLE] 超时：30秒内未收到数据，请检查蓝牙设备")
                await client.stop_notify(RX_CHAR_UUID)
                await client.disconnect()
                return
            await asyncio.sleep(0.1)
        print(f"[BLE] 数据到达！通道数: {len(receiver.latest_bend)}")

    # 4. 标定（握拳 → 伸直）
    print("\n===== bend 自动标定开始 =====")
    print(f"[INFO] 使用传感器通道: {SENSOR_INDEX}")
    bend_min_ref, bend_max_ref = await calibrate(receiver.get_latest, CALIBRATION_DURATION)
    print(f"[CALIB] bend_min(伸直): {bend_min_ref.astype(int).tolist()}")
    print(f"[CALIB] bend_max(握拳): {bend_max_ref.astype(int).tolist()}")
    print("===== 标定完成，进入控制循环 =====\n")

    # 5. 初始化滤波器
    bend_filter = MultiEMAFilter(channels=6, alpha=EMA_ALPHA)
    if USE_FILTER:
        print(f"[FILTER] EMA滤波已启用，alpha={EMA_ALPHA} (越小越平滑)\n")
    else:
        print("[FILTER] 滤波已禁用\n")

    # 6. 控制循环
    control_interval = 1.0 / CONTROL_HZ
    keepalive_time = time.time()
    last_print = 0

    print("按 Ctrl+C 停止\n")

    try:
        while True:
            loop_start = time.time()

            bend = receiver.get_latest()
            if bend is not None:
                # 提取原始传感器值
                raw_values = bend[SENSOR_INDEX].astype(np.float64)

                # 应用 EMA 滤波
                if USE_FILTER:
                    filtered_values = bend_filter.update(raw_values)
                else:
                    filtered_values = raw_values

                angles = bend_to_angle(filtered_values, bend_min_ref, bend_max_ref)
                new_cmd = build_cmd(angles)
                pubr.Write(new_cmd)

                # 每0.5秒打印一次状态
                now = time.time()
                if now - last_print > 0.5:
                    raw6 = raw_values.astype(int).tolist()
                    filtered6 = filtered_values.astype(int).tolist()
                    print(f"raw={raw6} | filt={filtered6} -> angles={angles}")
                    last_print = now

            # BLE 心跳保活（仅直连模式）
            now = time.time()
            if not USE_BROKER and now - keepalive_time > BLE_KEEPALIVE_INTERVAL:
                try:
                    await client.write_gatt_char(TX_CHAR_UUID, BLE_KEEPALIVE_DATA)
                except Exception:
                    pass
                keepalive_time = now

            # 精确控制循环频率
            elapsed = time.time() - loop_start
            sleep_time = control_interval - elapsed
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\n已停止")
    finally:
        if USE_BROKER:
            try:
                broker_sock.close()
            except Exception:
                pass
            print("[BLE] Broker 连接已关闭")
        else:
            try:
                await client.stop_notify(RX_CHAR_UUID)
                await client.disconnect()
            except Exception:
                pass
            print("[BLE] 已断开")


if __name__ == '__main__':
    asyncio.run(main())