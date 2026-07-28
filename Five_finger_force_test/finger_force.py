#!/usr/bin/env python3
"""
五指力反馈控制 — 五个 Dynamixel XL330-M288T 舵机独立 PID 闭环
通过 BLE 接收 STM32 发送的 JSON: {"touch_sensors": [大拇指, 食指, 中指, 无名指, 小指]}
每个力值分别驱动对应舵机的 PID 力反馈闭环

舵机映射:
  舵机1 → 大拇指 (touch_sensors[0])
  舵机2 → 食指   (touch_sensors[1])
  舵机3 → 中指   (touch_sensors[2])
  舵机4 → 无名指 (touch_sensors[3])
  舵机5 → 小指   (touch_sensors[4])
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

# ===== 手指名称映射 =====
FINGER_NAMES = {1: "大拇指", 2: "食指", 3: "中指", 4: "无名指", 5: "小指"}

# ===== BLE 配置 =====
DEVICE_ADDRESSES = [
	"F0:FD:45:02:85:B3",  # 设备1 MAC地址
	"F0:FD:45:02:67:3B",  # 设备2 MAC地址
]
TX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
RX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

# ===== BLE Broker 配置 =====
# USE_BROKER=True: 连接本地 ble_broker.py 中转（推荐，可与 ble_hand_control.py 共用同一蓝牙）
# USE_BROKER=False: 直接连接 BLE 设备（仅单独运行时使用）
USE_BROKER = True
BROKER_HOST = "127.0.0.1"
BROKER_PORT = 9001

# ===== BLE 保活配置 =====
BLE_KEEPALIVE_INTERVAL = 3.0
BLE_KEEPALIVE_DATA = b"PING"
BLE_DATA_TIMEOUT = 15.0
BLE_RECONNECT_DELAY = 1.0

# ===== 调试开关 =====
BLE_DEBUG = False
BLE_DEBUG_INTERVAL = 1.0
PID_DEBUG = False         # PID 力控调试打印（五指模式建议关闭，减少串口占用）
PID_DEBUG_INTERVAL = 1.0  # PID 调试打印间隔(秒)

# ===== 目标力缩放 =====
FORCE_SCALE = 1.0  # 目标力比例映射，默认1:1。如设0.5则目标力减半，设2.0则翻倍

# ===== PID 力闭环参数 =====
PID_KP = 50.0           # 比例系数 (mA/N)
PID_KI = 20.0           # 积分系数 (mA/(N·s))
PID_KD = 5.0            # 微分系数 (mA/(N/s))
PID_DEADZONE = 0.15     # 死区 (N) — 与单指版一致
PID_OUTPUT_MAX = 250    # 输出电流上限 (mA) — 降低防止5指同时过流
PID_OUTPUT_MIN = -150.0 # 输出电流下限 (mA)
PID_INTEGRAL_MAX = 200.0
TOTAL_CURRENT_LIMIT = 800  # 五指总电流上限 (mA)，防止电源过载

# ===== 滤波参数 =====
TARGET_FILTER_ALPHA = 0.3    # 与单指版一致
FEEDBACK_FILTER_ALPHA = 0.4  # 与单指版一致

# ===== BLE 反馈力零点校准 =====
BLE_FORCE_BASELINE = 4.903

# ===== 顺应模式参数（与单指版一致）=====
COMPLY_THRESHOLD = 0.3       # 反馈>目标+此值时进入顺应释放
COMPLY_RELEASE_GAIN = 20     # 顺应释放增益

# ===== 主动释放参数 =====
RELEASE_CURRENT_MA = -150.0
RELEASE_POSITION_THRESHOLD = 15
RELEASE_TIMEOUT = 3.0
RELEASE_SLOWDOWN_RANGE = 50
RELEASE_MIN_TIME = 0.3        # 释放最小运行时间(秒)，防止刚进入就判定到达
RELEASE_MIN_CURRENT = -30.0   # 释放最小电流(mA)，防止distance=0时电流=0不动

# ===== 硬件串口 & 舵机参数 =====
DEVICENAME = "/dev/ttyAMA0"
BAUDRATE = 1000000        # 1Mbps，舵机已通过change_baudrate.py修改
PROTOCOL_VERSION = 2.0
DXL_IDS = [1, 2, 3, 4, 5]
NUM_FINGERS = 5

# XL330 Control Table Addresses
ADDR_OPERATING_MODE = 11
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_CURRENT_LIMIT = 38
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_CURRENT = 102
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132
ADDR_PROFILE_VELOCITY = 112
ADDR_PROFILE_ACCELERATION = 108

# Operating Modes
OP_CURRENT_BASED_POSITION = 5
OP_CURRENT_CONTROL = 0

# 硬件错误打印限流
_hw_error_last_print = {}  # servo_id -> last_print_time
HW_ERROR_PRINT_INTERVAL = 3.0  # 同一舵机硬件错误最多每3秒打印一次


class TouchDataStore:
	"""保存 BLE 五指触摸数据的线程安全存储"""

	def __init__(self, size: int) -> None:
		self._lock = threading.Lock()
		self._values = [0.0] * size
		self._last_update = 0.0

	def update_from_json(self, payload: dict) -> None:
		if not isinstance(payload, dict):
			return
		sensors = payload.get("touch_sensors")
		if sensors is None:
			sensors = payload.get("force")
			if sensors is not None:
				sensors = [sensors]
		if not isinstance(sensors, list) or len(sensors) == 0:
			return

		try:
			value_list = [float(v) for v in sensors]
		except (TypeError, ValueError):
			return

		with self._lock:
			if len(value_list) == 1:
				# 单值广播到所有手指
				self._values = [value_list[0]] * len(self._values)
			else:
				# 五个值一一对应: [大拇指, 食指, 中指, 无名指, 小指]
				for i in range(min(len(self._values), len(value_list))):
					self._values[i] = value_list[i]
			self._last_update = time.time()

	def get_values(self) -> Tuple[List[float], float]:
		with self._lock:
			values = list(self._values)
			last_update = self._last_update
		age = time.time() - last_update if last_update > 0 else float("inf")
		return values, age


class JsonStreamParser:
	"""处理 BLE 分包/粘包 JSON 数据"""

	def __init__(self) -> None:
		self._buffer = ""
		self._decoder = json.JSONDecoder()

	def feed(self, text: str) -> List[dict]:
		self._buffer += text
		outputs: List[dict] = []

		while self._buffer:
			self._buffer = self._buffer.lstrip()
			if not self._buffer:
				break
			if not self._buffer.startswith("{") and not self._buffer.startswith("["):
				next_obj = self._buffer.find("{")
				if next_obj == -1:
					break
				self._buffer = self._buffer[next_obj:]
				continue
			try:
				obj, idx = self._decoder.raw_decode(self._buffer)
				if isinstance(obj, dict):
					outputs.append(obj)
				elif isinstance(obj, list):
					# BLE 发送的是裸数组 [4.9,4.9,...] → 包装成 dict
					outputs.append({"touch_sensors": obj})
				self._buffer = self._buffer[idx:]
				continue
			except json.JSONDecodeError:
				if "\n" in self._buffer:
					line, self._buffer = self._buffer.split("\n", 1)
					line = line.strip()
					if not line:
						continue
					try:
						parsed = json.loads(line)
						if isinstance(parsed, list):
							parsed = {"touch_sensors": parsed}
						if isinstance(parsed, dict):
							outputs.append(parsed)
					except json.JSONDecodeError:
						continue
				else:
					break
		return outputs


def extract_touch_sensors(text: str) -> List[float]:
	"""从原始文本中兜底提取 touch_sensors 数值或裸数组
	支持格式：
	  1. {"touch_sensors": [4.9, 4.9, ...]}
	  2. {0,3399,4089,...,4.90,4.90,4.90,4.90,4.90}  → 取最后5位
	  3. [4.9, 4.9, 4.9, 4.9, 4.9]                   → 取最后5位
	"""
	results: List[float] = []

	# 格式1: {"touch_sensors": [...]} 标准JSON
	pattern = re.compile(r'"touch_sensors"\s*:\s*\[([^\]]+)\]')
	for match in pattern.findall(text):
		parts = [p.strip() for p in match.split(",") if p.strip()]
		try:
			values = [float(p) for p in parts]
		except ValueError:
			continue
		if len(values) >= 2:
			results = values[-5:] if len(values) >= 5 else values

	# 格式2: {val1,val2,...,val18} 花括号混合数据，取最后5位
	if not results:
		pattern2 = re.compile(r'\{([0-9.,\s]+)\}')
		for match in pattern2.findall(text):
			parts = [p.strip() for p in match.split(",") if p.strip()]
			try:
				values = [float(p) for p in parts]
			except ValueError:
				continue
			if len(values) >= 5:
				results = values[-5:]  # 取最后5位

	# 格式3: 裸方括号数组 [4.9,4.9,...]，取最后5位
	if not results:
		pattern3 = re.compile(r'\[([0-9.,\s]+)\]')
		for match in pattern3.findall(text):
			parts = [p.strip() for p in match.split(",") if p.strip()]
			try:
				values = [float(p) for p in parts]
			except ValueError:
				continue
			if len(values) >= 2:
				results = values[-5:] if len(values) >= 5 else values

	return results


class ServoController:
	"""单个舵机/手指的控制器"""

	def __init__(self, servo_id: int):
		self.servo_id = servo_id
		self.finger_name = FINGER_NAMES.get(servo_id, f"手指{servo_id}")
		self.targetForceN = 0.0
		self.kN_per_mA = 0.8
		self.currentLimit_mA = 300  # 降低单舵机硬件电流上限，防止串联时电源过载
		self.touchMode = False
		self.holdMode = False
		self.freeModeActive = False
		self.goal_pos = 0
		self.home_pos = 0
		self.init_pos = None
		self.is_initialized = False
		self.state = "IDLE"
		self.command_updated = False

		# PID 闭环状态
		self.pid_integral = 0.0
		self.pid_last_error = 0.0
		self.pid_last_time = 0.0
		self.filtered_target = 0.0
		self.filtered_feedback = 0.0
		self.pid_output = 0.0

		# 主动释放状态
		self.releasing = False
		self.release_start_time = 0.0


servo_controllers = [ServoController(servo_id) for servo_id in DXL_IDS]
touch_store = TouchDataStore(NUM_FINGERS)
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
dxl_lock = threading.Lock()
running = True


# ===== 舵机底层读写 =====

def _hw_error_throttled_print(servo_id: int, msg: str) -> None:
	"""硬件错误限流打印，同一舵机每3秒最多打印一次"""
	now = time.time()
	last = _hw_error_last_print.get(servo_id, 0)
	if now - last >= HW_ERROR_PRINT_INTERVAL:
		print(msg)
		_hw_error_last_print[servo_id] = now


def read2ByteSigned(servo_id: int, addr: int) -> int:
	with dxl_lock:
		val, res, err = packetHandler.read2ByteTxRx(portHandler, servo_id, addr)
	if res != COMM_SUCCESS:
		_hw_error_throttled_print(servo_id, f"[舵机{servo_id}] 读取错误: {packetHandler.getTxRxResult(res)}")
	elif err != 0:
		_hw_error_throttled_print(servo_id, f"[舵机{servo_id}] 硬件错误: {packetHandler.getRxPacketError(err)}")
	if val > 32767:
		val -= 65536
	return val


def read4ByteSigned(servo_id: int, addr: int) -> int:
	with dxl_lock:
		val, res, err = packetHandler.read4ByteTxRx(portHandler, servo_id, addr)
	if res != COMM_SUCCESS:
		_hw_error_throttled_print(servo_id, f"[舵机{servo_id}] 读取错误: {packetHandler.getTxRxResult(res)}")
	elif err != 0:
		_hw_error_throttled_print(servo_id, f"[舵机{servo_id}] 硬件错误: {packetHandler.getRxPacketError(err)}")
	if val > 2147483647:
		val -= 4294967296
	return val


def write1Byte(servo_id: int, addr: int, val: int) -> None:
	with dxl_lock:
		res, _ = packetHandler.write1ByteTxRx(portHandler, servo_id, addr, val)
	if res != COMM_SUCCESS:
		print(f"[舵机{servo_id}] 写入错误: {packetHandler.getTxRxResult(res)}")


def write2Byte(servo_id: int, addr: int, val: int) -> None:
	with dxl_lock:
		res, _ = packetHandler.write2ByteTxRx(portHandler, servo_id, addr, val)
	if res != COMM_SUCCESS:
		print(f"[舵机{servo_id}] 写入错误: {packetHandler.getTxRxResult(res)}")


def write2ByteSigned(servo_id: int, addr: int, val: int) -> None:
	if val < 0:
		val = (1 << 16) + val
	write2Byte(servo_id, addr, val)


def write4Byte(servo_id: int, addr: int, val: int) -> None:
	with dxl_lock:
		res, _ = packetHandler.write4ByteTxRx(portHandler, servo_id, addr, int(val))
	if res != COMM_SUCCESS:
		print(f"[舵机{servo_id}] 写入错误: {packetHandler.getTxRxResult(res)}")


def torqueOn(servo_id: int) -> None:
	write1Byte(servo_id, ADDR_TORQUE_ENABLE, 1)


def torqueOff(servo_id: int) -> None:
	write1Byte(servo_id, ADDR_TORQUE_ENABLE, 0)


def set_operating_mode(servo_id: int, mode: int) -> None:
	torqueOff(servo_id)
	write1Byte(servo_id, ADDR_OPERATING_MODE, mode)
	torqueOn(servo_id)


def write_goal_current(servo_id: int, current_mA: float) -> None:
	current_cmd = int(round(current_mA))
	write2ByteSigned(servo_id, ADDR_GOAL_CURRENT, current_cmd)


def enterFreeMode(controller: ServoController) -> None:
	torqueOff(controller.servo_id)
	controller.freeModeActive = True
	print(f"[舵机{controller.servo_id}/{controller.finger_name}] 自由模式")


def exitFreeMode(controller: ServoController) -> None:
	ppos = read4ByteSigned(controller.servo_id, ADDR_PRESENT_POSITION)
	torqueOn(controller.servo_id)
	controller.goal_pos = ppos
	write4Byte(controller.servo_id, ADDR_GOAL_POSITION, controller.goal_pos)
	controller.freeModeActive = False
	print(f"[舵机{controller.servo_id}/{controller.finger_name}] 退出自由模式")


# ===== 命令解析 =====

def parse_command(buf: str) -> None:
	global running

	try:
		buf = buf.replace("：", ":").strip().upper()

		if buf.startswith("N:"):
			parts = buf.split(":")
			if len(parts) >= 3:
				try:
					servo_idx = int(parts[1]) - 1
					force_val = float(parts[2])

					if 0 <= servo_idx < len(servo_controllers):
						controller = servo_controllers[servo_idx]
						if force_val == 0:
							if controller.targetForceN == 0 and not controller.touchMode and not controller.releasing:
								pass  # 已经是空闲状态，忽略重复零力
							elif controller.releasing:
								pass  # 已在释放中，忽略重复零力
							elif controller.command_updated:
								pass  # 刚收到正力命令还未执行，不要立即释放
							else:
								controller.targetForceN = 0
								controller.pid_integral = 0.0
								controller.command_updated = False

								if controller.is_initialized and controller.init_pos is not None:
									controller.releasing = True
									controller.release_start_time = time.time()
									controller.state = "RELEASING"
									print(f"[舵机{controller.servo_id}/{controller.finger_name}] 目标力=0，开始释放回到初始位置 {controller.init_pos}")
								else:
									controller.touchMode = False
									controller.releasing = False
									controller.state = "IDLE"
									write_goal_current(controller.servo_id, 0)
									print(f"[舵机{controller.servo_id}/{controller.finger_name}] 目标力=0 (未初始化，输入INIT)")
						else:
							if controller.freeModeActive:
								exitFreeMode(controller)
							if controller.releasing:
								controller.releasing = False
								controller.state = "IDLE"
								print(f"[舵机{controller.servo_id}/{controller.finger_name}] 取消释放，切换到力控")
							controller.targetForceN = abs(force_val) * FORCE_SCALE
							controller.touchMode = True
							controller.command_updated = True
							print(f"[舵机{controller.servo_id}/{controller.finger_name}] 目标力={controller.targetForceN:.2f}N (原始={abs(force_val):.2f}×{FORCE_SCALE})")
					else:
						print(f"[错误] 舵机索引超出范围: {servo_idx+1}")
				except ValueError:
					print("[错误] 无效的数值格式")
			else:
				print("[错误] 格式: N:舵机ID:力值  例如: N:1:5")

		elif buf.startswith("NALL:"):
			try:
				force_val = float(buf[5:])
				for controller in servo_controllers:
					if force_val == 0:
						if controller.targetForceN == 0 and not controller.touchMode and not controller.releasing:
							continue
						if controller.releasing:
							continue
						controller.targetForceN = 0
						controller.pid_integral = 0.0
						controller.command_updated = False
						if controller.is_initialized and controller.init_pos is not None:
							controller.releasing = True
							controller.release_start_time = time.time()
							controller.state = "RELEASING"
						else:
							controller.touchMode = False
							controller.releasing = False
							controller.state = "IDLE"
							write_goal_current(controller.servo_id, 0)
					else:
						if controller.freeModeActive:
							exitFreeMode(controller)
						if controller.releasing:
							controller.releasing = False
						controller.targetForceN = abs(force_val) * FORCE_SCALE
						controller.touchMode = True
						controller.command_updated = True
				print(f"[全部] 目标力={abs(force_val) * FORCE_SCALE:.2f}N (原始={abs(force_val):.2f}×{FORCE_SCALE})")
			except ValueError:
				print("[错误] 无效的数值格式")

		elif buf.startswith("HOLD:"):
			try:
				servo_idx = int(buf[5:]) - 1
				if 0 <= servo_idx < len(servo_controllers):
					servo_controllers[servo_idx].holdMode = True
					c = servo_controllers[servo_idx]
					print(f"[舵机{c.servo_id}/{c.finger_name}] 保持模式")
			except ValueError:
				print("[错误] 无效的舵机ID")

		elif buf.startswith("FREE:"):
			try:
				servo_idx = int(buf[5:]) - 1
				if 0 <= servo_idx < len(servo_controllers):
					controller = servo_controllers[servo_idx]
					controller.targetForceN = 0
					controller.touchMode = False
					controller.holdMode = False
					enterFreeMode(controller)
			except ValueError:
				print("[错误] 无效的舵机ID")

		elif buf == "STATUS":
			values, age = touch_store.get_values()
			print("\n=== 五指力反馈状态 ===")
			print(f"[BLE] 反馈力={[f'{v:.2f}N' for v in values]}")
			for controller in servo_controllers:
				ppos = read4ByteSigned(controller.servo_id, ADDR_PRESENT_POSITION)
				pcur = read2ByteSigned(controller.servo_id, ADDR_PRESENT_CURRENT)
				init_str = f", 初始位置={controller.init_pos}" if controller.is_initialized else ""
				print(
					f"  舵机{controller.servo_id}/{controller.finger_name}: "
					f"状态={controller.state}, 位置={ppos}, 电流={pcur}mA, "
					f"目标力={controller.targetForceN:.2f}N{init_str}"
				)

		elif buf.startswith("CAL:"):
			parts = buf.split(":")
			if len(parts) >= 3:
				try:
					servo_idx = int(parts[1]) - 1
					real_force = float(parts[2])
					if 0 <= servo_idx < len(servo_controllers):
						controller = servo_controllers[servo_idx]
						pcur = read2ByteSigned(controller.servo_id, ADDR_PRESENT_CURRENT)
						if abs(pcur) >= 5:
							new_k = abs(real_force / float(pcur))
							if new_k > 0.1:
								print(f"[错误] 舵机{controller.servo_id} 标定系数过大 ({new_k:.6f})!")
							else:
								old_k = controller.kN_per_mA
								controller.kN_per_mA = new_k
								print(f"[舵机{controller.servo_id}] kN_per_mA: {old_k:.6f} -> {new_k:.6f}")
						else:
							print(f"[舵机{controller.servo_id}] 电流太小，请加大外力")
				except ValueError:
					print("[错误] 无效的数值格式")

		elif buf == "INIT":
			print("\n=== 佩戴初始化 (全部手指) ===")
			for controller in servo_controllers:
				pos = read4ByteSigned(controller.servo_id, ADDR_PRESENT_POSITION)
				controller.init_pos = pos
				controller.is_initialized = True
				controller.goal_pos = pos
				print(f"  舵机{controller.servo_id}/{controller.finger_name}: 初始位置={pos}")
			print("初始化完成! 目标力=0时舵机将回到此位置")

		elif buf.startswith("INIT:"):
			try:
				servo_idx = int(buf[5:]) - 1
				if 0 <= servo_idx < len(servo_controllers):
					controller = servo_controllers[servo_idx]
					pos = read4ByteSigned(controller.servo_id, ADDR_PRESENT_POSITION)
					controller.init_pos = pos
					controller.is_initialized = True
					controller.goal_pos = pos
					print(f"[舵机{controller.servo_id}/{controller.finger_name}] 初始位置={pos}")
			except ValueError:
				print("[错误] 格式: INIT:舵机ID  例如: INIT:2")

		elif buf == "HELP":
			print("\n=== 五指力反馈控制指令 ===")
			print("INIT         - 佩戴初始化 (记录所有舵机位置)")
			print("INIT:ID      - 初始化单个舵机 (例: INIT:2=食指)")
			print("N:ID:力值    - 设定单个舵机目标力 (例: N:1:5=大拇指5N)")
			print("NALL:力值    - 设定所有舵机目标力 (例: NALL:5)")
			print("HOLD:ID      - 保持舵机位置 (例: HOLD:1)")
			print("FREE:ID      - 自由模式 (例: FREE:1)")
			print("CAL:ID:力值  - 标定 (例: CAL:1:3)")
			print("STATUS       - 查看所有舵机状态")
			print("EXIT         - 退出程序")
			print("\n手指映射: 1=大拇指 2=食指 3=中指 4=无名指 5=小指")

		elif buf == "EXIT":
			running = False
			print("正在退出...")

		else:
			print(f"[未知指令] {buf}")

	except Exception as e:
		print(f"[命令错误] {e}")


# ===== TCP 服务器 (接收 dds_to_force.py 的命令) =====

def wifi_server_thread() -> None:
	HOST = "0.0.0.0"
	PORT = 8888

	server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

	try:
		server_socket.bind((HOST, PORT))
		server_socket.listen(1)
		print(f"=== TCP Server Listening on {HOST}:{PORT} ===")

		while running:
			try:
				server_socket.settimeout(1.0)
				conn, addr = server_socket.accept()
				print(f"[TCP] Connected by {addr}")
				try:
					buf = ""
					while running:
						data = conn.recv(4096)
						if not data:
							break
						buf += data.decode("utf-8", errors="ignore")
						# 按换行分割处理多条命令
						cmd_count = 0
						while "\n" in buf:
							line, buf = buf.split("\n", 1)
							cmd = line.strip()
							if cmd:
								print(f"[TCP 收到] {cmd}")
								parse_command(cmd)
								cmd_count += 1
						# 每次 recv 处理完后发送一次确认
						if cmd_count > 0:
							try:
								conn.sendall(b"CMD_RECEIVED\n")
							except Exception:
								break
				except Exception as e:
					print(f"[TCP] Connection Error: {e}")
				finally:
					conn.close()
					print(f"[TCP] Disconnected {addr}")
			except socket.timeout:
				continue
			except Exception as e:
				if running:
					print(f"[TCP] Server Error: {e}")
	except Exception as e:
		print(f"[TCP] Fatal Error: {e}")
	finally:
		server_socket.close()


def input_thread() -> None:
	print("=== 命令输入就绪 (输入 HELP 查看指令) ===")
	while running:
		try:
			line = sys.stdin.readline()
			if not line:
				break
			buf = line.strip()
			if not buf:
				continue
			parse_command(buf)
		except Exception as e:
			print(f"[输入错误] {e}")


# ===== BLE 连接 =====

async def ble_session(address: str, parser: JsonStreamParser) -> None:
	print(f"[BLE] 尝试连接: {address}")

	disconnected_event = asyncio.Event()

	def on_disconnect(client) -> None:
		print(f"[BLE] 设备断开连接: {address}")
		disconnected_event.set()

	async with BleakClient(address, disconnected_callback=on_disconnect, timeout=15.0) as client:
		if not client.is_connected:
			raise RuntimeError(f"无法连接到 {address}")
		print(f"[BLE] 已连接: {address}")
		raw_buffer = ""
		last_debug = 0.0
		last_data_time = time.time()
		last_ble_print = 0.0  # BLE接收打印节流

		def notification_handler(_, data: bytearray) -> None:
			nonlocal last_data_time, last_ble_print
			last_data_time = time.time()
			try:
				text = bytes(data).decode("utf-8", errors="ignore")
				nonlocal last_debug
				nonlocal raw_buffer
				raw_buffer += text
				payloads = parser.feed(text)
				for payload in payloads:
					touch_store.update_from_json(payload)
				# 兜底提取
				values = extract_touch_sensors(raw_buffer)
				if values:
					touch_store.update_from_json({"touch_sensors": values})
					raw_buffer = ""
				if len(raw_buffer) > 512:
					raw_buffer = raw_buffer[-512:]
			except Exception as e:
				print(f"[BLE] 解析错误: {e}")

		await client.start_notify(RX_CHAR_UUID, notification_handler)
		print(f"[BLE] 已启动通知监听: {address}")
		try:
			last_keepalive = time.time()
			while running:
				if disconnected_event.is_set():
					print(f"[BLE] 检测到断连: {address}")
					break
				if not client.is_connected:
					print(f"[BLE] 连接已丢失: {address}")
					break

				now = time.time()
				if now - last_keepalive >= BLE_KEEPALIVE_INTERVAL:
					try:
						await client.write_gatt_char(TX_CHAR_UUID, BLE_KEEPALIVE_DATA, response=False)
						last_keepalive = now
					except Exception as e:
						print(f"[BLE] 心跳失败: {e}")
						break

				data_age = now - last_data_time
				if data_age > BLE_DATA_TIMEOUT:
					print(f"[BLE] 数据超时 ({data_age:.1f}s)，主动断开: {address}")
					break
				await asyncio.sleep(0.5)
		finally:
			try:
				if client.is_connected:
					await client.stop_notify(RX_CHAR_UUID)
			except Exception:
				pass


async def ble_main() -> None:
	parser = JsonStreamParser()
	retry_count = 0
	MAX_RETRY_DELAY = 10.0
	last_good_addr = None

	while running:
		if last_good_addr:
			addr_order = [last_good_addr] + [a for a in DEVICE_ADDRESSES if a != last_good_addr]
		else:
			addr_order = list(DEVICE_ADDRESSES)

		for addr in addr_order:
			if not running:
				break
			try:
				await ble_session(addr, parser)
				last_good_addr = addr
				retry_count = 0
				print(f"[BLE] 会话结束，{BLE_RECONNECT_DELAY}秒后重连...")
				await asyncio.sleep(BLE_RECONNECT_DELAY)
				break
			except Exception as e:
				retry_count += 1
				delay = min(1.0 * retry_count, MAX_RETRY_DELAY)
				if "NotPermitted" in str(e) or "Notify acquired" in str(e):
					delay = max(delay, 3.0)
				print(f"[BLE] 连接失败 {addr} (第{retry_count}次): {e}")
				print(f"[BLE] {delay:.1f}秒后重试...")
				await asyncio.sleep(delay)
		if not running:
			break


def ble_broker_client() -> None:
	"""连接本地 BLE Broker，接收广播数据，替代直连 BLE"""
	parser = JsonStreamParser()
	buf = ""
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
						print("[BLE] Broker 连接关闭")
						break
					text = chunk.decode("utf-8", errors="ignore")
					buf += text
					# 与直连模式完全相同的解析逻辑
					payloads = parser.feed(text)
					for payload in payloads:
						touch_store.update_from_json(payload)
					values = extract_touch_sensors(buf)
					if values:
						touch_store.update_from_json({"touch_sensors": values})
						buf = ""
					if len(buf) > 512:
						buf = buf[-512:]
				except socket.timeout:
					continue
				except Exception as e:
					print(f"[BLE] 接收错误: {e}")
					break
		except Exception as e:
			if running:
				print(f"[BLE] 连接 Broker 失败: {e}，3s 后重试...")
				time.sleep(3)
		finally:
			try:
				sock.close()
			except Exception:
				pass


def ble_thread() -> None:
	if USE_BROKER:
		ble_broker_client()
	else:
		try:
			asyncio.run(ble_main())
		except Exception as e:
			print(f"[BLE] 线程退出: {e}")


# ===== 硬件初始化 =====

def setup() -> None:
	if portHandler.openPort():
		print("串口打开成功")
	else:
		print("串口打开失败")
		sys.exit(1)

	if portHandler.setBaudRate(BAUDRATE):
		print("波特率设置成功")
	else:
		print("波特率设置失败")
		sys.exit(1)

	for controller in servo_controllers:
		servo_id = controller.servo_id
		model_number, res, err = packetHandler.ping(portHandler, servo_id)
		if res != COMM_SUCCESS:
			print(f"[舵机{servo_id}/{controller.finger_name}] Ping 失败: {packetHandler.getTxRxResult(res)}")
			continue
		print(f"[舵机{servo_id}/{controller.finger_name}] Ping 成功, Model: {model_number}")

		torqueOff(servo_id)
		write1Byte(servo_id, ADDR_OPERATING_MODE, OP_CURRENT_BASED_POSITION)
		write2Byte(servo_id, ADDR_CURRENT_LIMIT, controller.currentLimit_mA)
		write4Byte(servo_id, ADDR_PROFILE_VELOCITY, 100)   # 降低速度，运动更平稳
		write4Byte(servo_id, ADDR_PROFILE_ACCELERATION, 50)  # 降低加速度，减少冲击
		torqueOn(servo_id)

		controller.home_pos = read4ByteSigned(servo_id, ADDR_PRESENT_POSITION)
		controller.goal_pos = controller.home_pos
		write4Byte(servo_id, ADDR_GOAL_POSITION, controller.goal_pos)
		time.sleep(0.05)

	print("\n===== 五指力反馈系统就绪 =====")
	print("手指映射: 舵机1=大拇指, 舵机2=食指, 舵机3=中指, 舵机4=无名指, 舵机5=小指")
	print("BLE 数据格式: {\"touch_sensors\": [大拇指N, 食指N, 中指N, 无名指N, 小指N]}")


# ===== PID 控制 =====

def pid_control(controller: ServoController, target_force: float, feedback_force: float, dt: float) -> float:
	"""PID 力闭环控制（与单指版一致）
	   误差 = 目标 - 反馈
	   正值 → 拉紧（正电流）, 负值 → 释放（负电流）
	"""
	# 一阶低通滤波
	controller.filtered_target += TARGET_FILTER_ALPHA * (target_force - controller.filtered_target)
	controller.filtered_feedback += FEEDBACK_FILTER_ALPHA * (feedback_force - controller.filtered_feedback)

	# 误差: 正=需拉紧, 负=需释放
	error = controller.filtered_target - controller.filtered_feedback

	# 死区
	if abs(error) < PID_DEADZONE:
		error = 0.0
		controller.pid_integral *= 0.95

	# ===== 顺应模式: 反馈力明显大于目标力时快速释放 =====
	if controller.filtered_feedback > controller.filtered_target + COMPLY_THRESHOLD:
		excess = controller.filtered_feedback - controller.filtered_target
		release_current = -COMPLY_RELEASE_GAIN * excess
		release_current = max(release_current, PID_OUTPUT_MIN)
		controller.pid_integral = 0.0
		controller.state = "COMPLY"
		return release_current

	# ===== PID 调节模式 =====
	controller.state = "PID"
	p_term = PID_KP * error

	controller.pid_integral += error * dt
	controller.pid_integral = max(-PID_INTEGRAL_MAX / PID_KI,
								   min(PID_INTEGRAL_MAX / PID_KI, controller.pid_integral))
	i_term = PID_KI * controller.pid_integral

	if dt > 0:
		d_error = (error - controller.pid_last_error) / dt
	else:
		d_error = 0.0
	d_term = PID_KD * d_error
	controller.pid_last_error = error

	output = p_term + i_term + d_term
	output = max(PID_OUTPUT_MIN, min(PID_OUTPUT_MAX, output))

	# 目标和反馈都接近0时归零
	if controller.filtered_target < 0.1 and controller.filtered_feedback < 0.1:
		output = 0.0
		controller.pid_integral = 0.0

	return output


# ===== 主控制循环 =====

def loop() -> None:
	global running
	last_print = time.time()
	last_pid_debug = time.time()
	loop_count = 0
	loop_freq_time = time.time()

	while running:
		try:
			current_time = time.time()
			touch_values, touch_age = touch_store.get_values()
			# touch_values: [大拇指, 食指, 中指, 无名指, 小指] 对应舵机 [1, 2, 3, 4, 5]

			for idx, controller in enumerate(servo_controllers):
				if controller.holdMode or controller.freeModeActive:
					continue

				# ===== 主动释放模式 =====
				if controller.releasing:
					present_pos = read4ByteSigned(controller.servo_id, ADDR_PRESENT_POSITION)
					distance = present_pos - controller.init_pos
					elapsed = current_time - controller.release_start_time

					# 最小运行时间保护：刚进入释放时 present_pos ≈ init_pos，
					# 需要等舵机实际开始移动后再判断是否到达
					if elapsed >= RELEASE_MIN_TIME and (abs(distance) <= RELEASE_POSITION_THRESHOLD or elapsed > RELEASE_TIMEOUT):
						# 先写0电流刹车，避免切换模式时惯性过冲
						write_goal_current(controller.servo_id, 0)
						time.sleep(0.02)  # 等待电流生效
						set_operating_mode(controller.servo_id, OP_CURRENT_BASED_POSITION)
						write2Byte(controller.servo_id, ADDR_CURRENT_LIMIT, controller.currentLimit_mA)
						write4Byte(controller.servo_id, ADDR_PROFILE_VELOCITY, 50)   # 低速归位防震荡
						write4Byte(controller.servo_id, ADDR_PROFILE_ACCELERATION, 30) # 低加速防过冲
						write4Byte(controller.servo_id, ADDR_GOAL_POSITION, controller.init_pos)
						controller.goal_pos = controller.init_pos
						controller.releasing = False
						controller.touchMode = False
						controller.state = "IDLE"
						controller.pid_integral = 0.0
						controller.pid_output = 0.0
						reason = "到达初始位置" if abs(distance) <= RELEASE_POSITION_THRESHOLD else f"超时({elapsed:.1f}s)"
						print(f"[舵机{controller.servo_id}/{controller.finger_name}] 释放完成 ({reason})")
					else:
						if abs(distance) < RELEASE_SLOWDOWN_RANGE:
							ratio = abs(distance) / RELEASE_SLOWDOWN_RANGE
							release_mA = RELEASE_CURRENT_MA * max(ratio, 0.2)  # 最低20%电流
						else:
							release_mA = RELEASE_CURRENT_MA
						# 保证释放电流不低于最小值
						if abs(release_mA) < abs(RELEASE_MIN_CURRENT):
							release_mA = RELEASE_MIN_CURRENT
						if distance < 0:
							release_mA = -release_mA
						write_goal_current(controller.servo_id, release_mA)
						controller.pid_output = release_mA
					continue

				# ===== 正常 PID 力控制 =====
				if not controller.touchMode:
					continue

				# 时间间隔
				if controller.pid_last_time > 0:
					dt = current_time - controller.pid_last_time
				else:
					dt = 0.02
				controller.pid_last_time = current_time
				dt = max(0.001, min(0.1, dt))

				# 获取该手指的反馈力 (BLE)
				# idx 对应: 0=大拇指, 1=食指, 2=中指, 3=无名指, 4=小指
				raw_feedbackN = touch_values[idx] if idx < len(touch_values) else 0.0

				# BLE 数据过期处理
				ble_valid = (touch_age <= 2.0)
				ble_never_connected = (touch_age == float("inf"))
				if not ble_valid:
					if ble_never_connected:
						# BLE 从未连接过 → 用 feedbackN=0 继续开环控制 (不释放)
						raw_feedbackN = 0.0
					elif touch_age > 5.0 and not controller.command_updated:
						# BLE 曾连接但已断开超过5秒 → 触发释放
						if not controller.releasing and controller.is_initialized and controller.init_pos is not None:
							write_goal_current(controller.servo_id, 0)
							controller.targetForceN = 0
							controller.pid_integral = 0.0
							controller.releasing = True
							controller.release_start_time = current_time
							controller.state = "RELEASING"
							print(f"[警告] BLE 断开 ({touch_age:.1f}s)，舵机{controller.servo_id}/{controller.finger_name} 释放")
						elif not controller.is_initialized:
							write_goal_current(controller.servo_id, 0)
							controller.touchMode = False
							controller.state = "IDLE"
							controller.pid_integral = 0.0
						continue
					else:
						# BLE 短暂过期 → 用 feedbackN=0 继续开环控制
						raw_feedbackN = 0.0

				# BLE 力零点校准
				if raw_feedbackN <= BLE_FORCE_BASELINE:
					feedbackN = 0.0
				else:
					feedbackN = raw_feedbackN

				targetN = controller.targetForceN

				# 首次进入 → 切换电流控制模式
				if controller.command_updated:
					controller.command_updated = False
					controller.filtered_target = targetN
					controller.filtered_feedback = feedbackN
					controller.pid_integral = 0.0
					controller.pid_last_error = 0.0
					set_operating_mode(controller.servo_id, OP_CURRENT_CONTROL)
					print(f"[舵机{controller.servo_id}/{controller.finger_name}] PID闭环, 目标={targetN:.2f}N, BLE反馈={feedbackN:.2f}N")

				# PID 计算（先暂存，后面统一做总电流限制再写入）
				output_mA = pid_control(controller, targetN, feedbackN, dt)
				controller.pid_output = output_mA
				output_mA = max(-controller.currentLimit_mA,
							   min(controller.currentLimit_mA, output_mA))
				controller._pending_current = output_mA

				# PID 调试打印 — 默认关闭，避免额外串口读+print开销
				if PID_DEBUG and current_time - last_pid_debug >= PID_DEBUG_INTERVAL:
					present_cur = read2ByteSigned(controller.servo_id, ADDR_PRESENT_CURRENT)
					print(
						f"  [PID] {controller.finger_name}: "
						f"T={targetN:.1f} F={feedbackN:.1f} O={output_mA:.0f}mA "
						f"R={present_cur}mA {controller.state}"
					)

			# ===== 五指总电流限制 =====
			# 收集所有待写入的正电流（拉紧方向），如果总和超限则等比缩放
			active_currents = []
			for c in servo_controllers:
				if hasattr(c, '_pending_current'):
					active_currents.append(c)
			total_positive = sum(max(0, c._pending_current) for c in active_currents)
			if total_positive > TOTAL_CURRENT_LIMIT and total_positive > 0:
				scale = TOTAL_CURRENT_LIMIT / total_positive
				for c in active_currents:
					if c._pending_current > 0:
						c._pending_current *= scale
						c.pid_output = c._pending_current

			# 统一写入电流
			for c in active_currents:
				write_goal_current(c.servo_id, c._pending_current)
				del c._pending_current

			# 更新PID调试打印时间戳 (在for循环外)
			if PID_DEBUG and current_time - last_pid_debug >= PID_DEBUG_INTERVAL:
				last_pid_debug = current_time

			# ===== 循环计数 =====
			loop_count += 1

			# ===== 状态打印 (每2秒一次，减少IO开销) =====
			if current_time - last_print > 2.0:
				# 计算循环频率
				freq_elapsed = current_time - loop_freq_time
				if freq_elapsed > 0:
					freq = loop_count / freq_elapsed
				else:
					freq = 0
				loop_count = 0
				loop_freq_time = current_time

				active_servos = [c for c in servo_controllers if c.touchMode or c.releasing]
				if active_servos:
					# 读取所有活跃舵机的实际电流（集成监测，无需额外monitor程序）
					real_currents = {}
					for c in active_servos:
						real_currents[c.servo_id] = read2ByteSigned(c.servo_id, ADDR_PRESENT_CURRENT)
					parts = [f"[{freq:.0f}Hz]"]
					for c in active_servos:
						rc = real_currents.get(c.servo_id, 0)
						if c.releasing:
							parts.append(f"{c.finger_name}:REL O={c.pid_output:.0f} R={rc}mA")
						else:
							err = c.filtered_target - c.filtered_feedback
							parts.append(f"{c.finger_name}:{c.state} T={c.filtered_target:.1f} F={c.filtered_feedback:.1f} O={c.pid_output:.0f} R={rc}mA")
					total = sum(abs(v) for v in real_currents.values())
					parts.append(f"Σ={total}mA")
					print(" | ".join(parts))
				last_print = current_time

			time.sleep(0.005)  # 5ms — 配合1Mbps波特率可达更高频率

		except KeyboardInterrupt:
			signal_handler(signal.SIGINT, None)
			break
		except Exception as e:
			print(f"[循环错误] {e}")


def signal_handler(signum, frame) -> None:
	global running
	if running:
		print("\n接收到 Ctrl+C，正在停止...")
	running = False


if __name__ == "__main__":
	try:
		signal.signal(signal.SIGINT, signal_handler)
		setup()

		t_input = threading.Thread(target=input_thread, daemon=True)
		t_input.start()

		t_wifi = threading.Thread(target=wifi_server_thread, daemon=True)
		t_wifi.start()

		t_ble = threading.Thread(target=ble_thread, daemon=True)
		t_ble.start()

		loop()

	except KeyboardInterrupt:
		print("\n程序被用户中断")
	except Exception as e:
		print(f"[致命错误] {e}")
	finally:
		print("\n正在关闭所有舵机...")
		for controller in servo_controllers:
			torqueOff(controller.servo_id)
		portHandler.closePort()
		print("程序已退出")
