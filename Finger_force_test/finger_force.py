#!/usr/bin/env python3
"""
五个 Dynamixel XL330-M288T 舵机的力反馈控制（BLE 触摸数据闭环）
通过 BLE 接收 STM32 发送的 JSON: {"touch_sensors": [..]}，用于力反馈闭环
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

# ===== BLE 配置 =====
DEVICE_ADDRESSES = [
	"F0:FD:45:02:85:B3",  # 设备1 MAC地址
	"F0:FD:45:02:67:3B",  # 设备2 MAC地址
]
TX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # 发送数据的特征 UUID
RX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # 接收数据的特征 UUID

# ===== BLE 保活配置 =====
BLE_KEEPALIVE_INTERVAL = 3.0   # 每3秒发送一次心跳
BLE_KEEPALIVE_DATA = b"PING"   # 心跳数据 (STM32端可忽略)
BLE_DATA_TIMEOUT = 15.0        # 数据超时时间 (秒)
BLE_RECONNECT_DELAY = 1.0      # 重连前等待时间

# ===== 调试开关 =====
BLE_DEBUG = False
BLE_DEBUG_INTERVAL = 1.0

# ===== 力反馈方向 =====
# +1: touch>target 时后退，touch<target 时前进
# -1: 反向
FORCE_DIRECTION = 1

# ===== 几何力比例 =====
# 绳索拉力约为指尖力的倍数（例如 2.0 表示拉力约为指尖力的两倍）
GEOMETRY_FORCE_RATIO = 2.0

# ===== 目标力 -> 电流缩放 =====
# 数值越大，寻觅拉力越明显（注意不要超过电流限制）
FORCE_TO_CURRENT_SCALE = 10.0

# ===== PID 力闭环参数 =====
PID_KP = 80.0           # 比例系数 (mA/N)
PID_KI = 20.0           # 积分系数 (mA/(N·s))
PID_KD = 5.0            # 微分系数 (mA/(N/s))
PID_DEADZONE = 0.15     # 死区 (N)，误差小于此值不调节
PID_OUTPUT_MAX = 400.0  # 输出电流上限 (mA)
PID_OUTPUT_MIN = -200.0 # 输出电流下限 (mA)，负值用于释放
PID_INTEGRAL_MAX = 300.0  # 积分项上限 (防饱和)

# ===== 滤波参数 =====
TARGET_FILTER_ALPHA = 0.3   # 目标力一阶滤波系数 (0~1, 越小越平滑)
FEEDBACK_FILTER_ALPHA = 0.4 # 反馈力一阶滤波系数

# ===== BLE 反馈力零点校准 =====
# BLE 传感器特性:
#   原始值 <= BLE_FORCE_BASELINE 时, 传感器未受力, 实际力 = 0N
#   原始值 >  BLE_FORCE_BASELINE 时, 原始值就是准确的实际力 (不减基准)
BLE_FORCE_BASELINE = 4.903  # 传感器零点阈值

# ===== 释放/顺应参数 =====
COMPLY_THRESHOLD = 0.3      # 反馈超过目标多少N时进入顺应 (释放)
COMPLY_RELEASE_GAIN = 20  # 顺应模式下的释放电流增益 (mA/N)

# ===== 主动释放参数 =====
# 当DDS目标力降为0时，舵机需要主动反转释放绳索回到初始位置
RELEASE_CURRENT_MA = -150.0     # 释放电流 (mA, 负值=反向)
RELEASE_POSITION_THRESHOLD = 15 # 到达初始位置的允许误差 (位置单位)
RELEASE_TIMEOUT = 3.0           # 释放超时时间 (秒)
RELEASE_SLOWDOWN_RANGE = 50     # 接近初始位置时减速的范围 (位置单位)

# ===== 旧参数保留(兼容) =====
SEEK_CURRENT_SIGN = 1
SEEK_HOLD_MIN_FORCE = 0.3

# ===== 硬件串口 & 舵机参数 =====
DEVICENAME = "/dev/ttyAMA0"
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0
DXL_IDS = [1, 2, 3, 4, 5]

# XL330 Control Table Addresses
ADDR_OPERATING_MODE = 11
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


class TouchDataStore:
	"""保存 BLE 触摸数据的线程安全存储"""

	def __init__(self, size: int) -> None:
		self._lock = threading.Lock()
		self._values = [0.0] * size
		self._last_update = 0.0
		self._last_payload = {}

	def update_from_json(self, payload: dict) -> None:
		if not isinstance(payload, dict):
			return
		if "touch_sensors" in payload:
			sensors = payload.get("touch_sensors")
		elif "force" in payload:
			sensors = [payload.get("force")]
		else:
			return
		if isinstance(sensors, list):
			if len(sensors) == 0:
				return
			values = sensors
		else:
			values = [sensors]

		try:
			value_list = [float(v) for v in values]
		except (TypeError, ValueError):
			return

		with self._lock:
			if len(value_list) == 1:
				self._values = [value_list[0]] * len(self._values)
			else:
				for i in range(min(len(self._values), len(value_list))):
					self._values[i] = value_list[i]
			self._last_update = time.time()
			self._last_payload = payload

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
				self._buffer = self._buffer[idx:]
				continue
			except json.JSONDecodeError:
				if "\n" in self._buffer:
					line, self._buffer = self._buffer.split("\n", 1)
					line = line.strip()
					if not line:
						continue
					try:
						outputs.append(json.loads(line))
					except json.JSONDecodeError:
						continue
				else:
					break
		return outputs


def extract_touch_sensors(text: str) -> List[float]:
	"""从原始文本中兜底提取 touch_sensors 数值"""
	results: List[float] = []
	pattern = re.compile(r'"touch_sensors"\s*:\s*\[([^\]]+)\]')
	for match in pattern.findall(text):
		parts = [p.strip() for p in match.split(",") if p.strip()]
		try:
			values = [float(p) for p in parts]
		except ValueError:
			continue
		if values:
			results.extend(values)
	return results


def extract_force(text: str) -> List[float]:
	"""兜底提取 force 数值"""
	match = re.search(r'"force"\s*:\s*([0-9.+-eE]+)', text)
	if not match:
		return []
	try:
		return [float(match.group(1))]
	except ValueError:
		return []


def print_payload(payload: dict) -> None:
	"""按 finger.py 的格式输出 JSON"""
	try:
		print(json.dumps(payload, ensure_ascii=False))
	except Exception:
		print(payload)


class ServoController:
	"""单个舵机的控制器"""

	def __init__(self, servo_id: int):
		self.servo_id = servo_id
		self.targetForceN = 0.0
		self.kN_per_mA = 0.8
		self.hysteresisN = 1.5
		self.currentLimit_mA = 450
		self.touchMode = False
		self.holdMode = False
		self.freeModeActive = False
		self.goal_pos = 0
		self.home_pos = 0
		self.init_pos = None          # 佩戴初始化位置 (INIT 命令设置)
		self.is_initialized = False   # 是否已执行佩戴初始化
		self.admittance_gain = 15.0
		self.admittance_gain_positive = 25.0
		self.max_step_per_loop = 25
		self.calMin_mA = 5
		self.mA_window = []
		self.window_size = 10
		self.state = "IDLE"
		self.command_updated = False
		self.target_current_mA = 0.0
		self.last_pos = 0
		self.stable_count = 0
		self.hold_force = 0.0
		self.last_target_force = 0.0
		
		# === PID 闭环状态 ===
		self.pid_integral = 0.0       # 积分累积
		self.pid_last_error = 0.0     # 上次误差 (用于微分)
		self.pid_last_time = 0.0      # 上次时间戳
		self.filtered_target = 0.0    # 滤波后的目标力
		self.filtered_feedback = 0.0  # 滤波后的反馈力
		self.pid_output = 0.0         # PID 输出电流
		
		# === 主动释放状态 ===
		self.releasing = False        # 是否正在主动释放
		self.release_start_time = 0.0 # 释放开始时间


servo_controllers = [ServoController(servo_id) for servo_id in DXL_IDS]
touch_store = TouchDataStore(len(DXL_IDS))
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
dxl_lock = threading.Lock()
running = True


def read2ByteSigned(servo_id: int, addr: int) -> int:
	with dxl_lock:
		val, res, err = packetHandler.read2ByteTxRx(portHandler, servo_id, addr)
	if res != COMM_SUCCESS:
		print(f"[舵机{servo_id}] 读取错误: {packetHandler.getTxRxResult(res)}")
	elif err != 0:
		print(f"[舵机{servo_id}] 硬件错误: {packetHandler.getRxPacketError(err)}")
	if val > 32767:
		val -= 65536
	return val


def read4ByteSigned(servo_id: int, addr: int) -> int:
	with dxl_lock:
		val, res, err = packetHandler.read4ByteTxRx(portHandler, servo_id, addr)
	if res != COMM_SUCCESS:
		print(f"[舵机{servo_id}] 读取错误: {packetHandler.getTxRxResult(res)}")
	elif err != 0:
		print(f"[舵机{servo_id}] 硬件错误: {packetHandler.getRxPacketError(err)}")
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
	print(f"[舵机{controller.servo_id}] 自由模式")


def exitFreeMode(controller: ServoController) -> None:
	ppos = read4ByteSigned(controller.servo_id, ADDR_PRESENT_POSITION)
	torqueOn(controller.servo_id)
	controller.goal_pos = ppos
	write4Byte(controller.servo_id, ADDR_GOAL_POSITION, controller.goal_pos)
	controller.freeModeActive = False
	print(f"[舵机{controller.servo_id}] 退出自由模式")


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
							# 如果已经在空闲状态（非触控、非释放），跳过
							if controller.targetForceN == 0 and not controller.touchMode and not controller.releasing:
								pass
							elif controller.releasing:
								pass  # 正在释放中，不重复触发
							else:
								# 目标力=0：进入主动释放模式
								# 不立即切换模式，而是在电流控制模式下反向释放绳索
								controller.targetForceN = 0
								controller.pid_integral = 0.0
								controller.command_updated = False
								
								if controller.is_initialized and controller.init_pos is not None:
									# 进入主动释放状态，保持在电流控制模式
									controller.releasing = True
									controller.release_start_time = time.time()
									controller.state = "RELEASING"
									# touchMode 保持 True，让主循环继续处理
									print(f"[舵机{controller.servo_id}] 目标力=0，开始主动释放绳索回到初始位置 {controller.init_pos}")
								else:
									# 未初始化，直接停止
									controller.touchMode = False
									controller.releasing = False
									controller.state = "IDLE"
									write_goal_current(controller.servo_id, 0)
									print(f"[舵机{controller.servo_id}] 目标力=0，释放舵机 (未初始化，输入INIT进行佩戴初始化)")
						else:
							if controller.freeModeActive:
								exitFreeMode(controller)
							# 如果正在释放，先取消释放
							if controller.releasing:
								controller.releasing = False
							controller.targetForceN = abs(force_val)
							controller.touchMode = True
							controller.command_updated = True
							print(f"[舵机{controller.servo_id}] 目标力={controller.targetForceN}N")
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
						controller.targetForceN = abs(force_val)
						controller.touchMode = True
						controller.command_updated = True
				print(f"[全部] 目标力={force_val}N")
			except ValueError:
				print("[错误] 无效的数值格式")

		elif buf.startswith("HOLD:"):
			try:
				servo_idx = int(buf[5:]) - 1
				if 0 <= servo_idx < len(servo_controllers):
					servo_controllers[servo_idx].holdMode = True
					print(f"[舵机{servo_idx+1}] 保持模式")
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
			print("\n=== 五舵机状态 ===")
			print(f"[BLE] touch_sensors age={age:.2f}s values={values}")
			for controller in servo_controllers:
				ppos = read4ByteSigned(controller.servo_id, ADDR_PRESENT_POSITION)
				pcur = read2ByteSigned(controller.servo_id, ADDR_PRESENT_CURRENT)
				print(
					f"舵机{controller.servo_id}: 位置={ppos}, 电流={pcur}mA, 目标力={controller.targetForceN:.2f}N"
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
						if abs(pcur) >= controller.calMin_mA:
							new_k = abs(real_force / float(pcur))
							if new_k > 0.1:
								print(
									f"[错误] 舵机{controller.servo_id} 标定系数过大 ({new_k:.6f})! 忽略。"
								)
								print("       原因: 电流相对于力太小。请确保舵机真的在用力。")
							else:
								old_k = controller.kN_per_mA
								controller.kN_per_mA = new_k
								print(
									f"[舵机{controller.servo_id}] kN_per_mA: {old_k:.6f} -> {new_k:.6f}"
								)
						else:
							print(
								f"[舵机{controller.servo_id}] 电流太小(<{controller.calMin_mA}mA)，请加大外力"
							)
				except ValueError:
					print("[错误] 无效的数值格式")

		elif buf.startswith("KN:"):
			parts = buf.split(":")
			if len(parts) >= 3:
				try:
					servo_idx = int(parts[1]) - 1
					new_k = float(parts[2])
					if 0 <= servo_idx < len(servo_controllers):
						servo_controllers[servo_idx].kN_per_mA = new_k
						print(f"[舵机{servo_idx+1}] kN_per_mA set to {new_k:.6f}")
				except ValueError:
					print("Invalid Value")

		elif buf == "INIT":
			# 佩戴初始化：记录所有舵机当前位置作为"佩戴位置"
			print("\n=== 佩戴初始化 ===")
			for controller in servo_controllers:
				pos = read4ByteSigned(controller.servo_id, ADDR_PRESENT_POSITION)
				controller.init_pos = pos
				controller.is_initialized = True
				controller.goal_pos = pos
				print(f"  舵机{controller.servo_id}: 初始位置记录为 {pos}")
			print("初始化完成! 目标力=0时舵机将回到此位置")

		elif buf.startswith("INIT:"):
			# 初始化单个舵机: INIT:2
			try:
				servo_idx = int(buf[5:]) - 1
				if 0 <= servo_idx < len(servo_controllers):
					controller = servo_controllers[servo_idx]
					pos = read4ByteSigned(controller.servo_id, ADDR_PRESENT_POSITION)
					controller.init_pos = pos
					controller.is_initialized = True
					controller.goal_pos = pos
					print(f"[舵机{controller.servo_id}] 初始位置记录为 {pos}")
			except ValueError:
				print("[错误] 格式: INIT:舵机ID  例如: INIT:2")

		elif buf == "HELP":
			print("\n=== 五舵机控制指令 ===")
			print("INIT         - 佩戴初始化 (记录当前位置，先执行)")
			print("INIT:ID      - 初始化单个舵机 (例: INIT:2)")
			print("N:ID:力值    - 设定单个舵机目标力 (例: N:1:5)")
			print("NALL:力值    - 设定所有舵机目标力 (例: NALL:5)")
			print("KN:ID:系数   - 手动设定力系数 (例: KN:1:0.025)")
			print("HOLD:ID      - 保持舵机当前位置 (例: HOLD:1)")
			print("FREE:ID      - 舵机进入自由模式 (例: FREE:1)")
			print("CAL:ID:力值  - 标定舵机力系数 (例: CAL:1:3)")
			print("STATUS       - 查看所有舵机状态")
			print("EXIT         - 退出程序")

		elif buf == "EXIT":
			running = False
			print("正在退出...")

		else:
			print(f"[未知指令] {buf}")

	except Exception as e:
		print(f"[命令错误] {e}")


def wifi_server_thread() -> None:
	HOST = "0.0.0.0"
	PORT = 8888

	server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

	try:
		server_socket.bind((HOST, PORT))
		server_socket.listen(1)
		print(f"=== WiFi Server Listening on {HOST}:{PORT} ===")

		while running:
			try:
				server_socket.settimeout(1.0)
				conn, addr = server_socket.accept()
				print(f"[WiFi] Connected by {addr}")
				try:
					while running:
						data = conn.recv(1024)
						if not data:
							break
						cmd = data.decode("utf-8").strip()
						if cmd:
							print(f"[WiFi] Recv: {cmd}")
							parse_command(cmd)
							conn.sendall(b"CMD_RECEIVED\n")
				except Exception as e:
					print(f"[WiFi] Connection Error: {e}")
				finally:
					conn.close()
					print(f"[WiFi] Disconnected {addr}")
			except socket.timeout:
				continue
			except Exception as e:
				if running:
					print(f"[WiFi] Server Error: {e}")
	except Exception as e:
		print(f"[WiFi] Fatal Error: {e}")
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


async def ble_session(address: str, parser: JsonStreamParser) -> None:
	print(f"[BLE] 尝试连接: {address}")

	# 断连标志
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
		last_data_time = time.time()  # 上次收到数据的时间

		def notification_handler(_, data: bytearray) -> None:
			nonlocal last_data_time
			last_data_time = time.time()  # 每次收到数据都更新
			try:
				text = bytes(data).decode("utf-8", errors="ignore")
				nonlocal last_debug
				if BLE_DEBUG and time.time() - last_debug >= BLE_DEBUG_INTERVAL:
					print(f"[BLE RAW] {text.strip()}")
					last_debug = time.time()
				nonlocal raw_buffer
				raw_buffer += text
				payloads = parser.feed(text)
				for payload in payloads:
					if BLE_DEBUG and time.time() - last_debug >= BLE_DEBUG_INTERVAL:
						print(f"[BLE JSON] {payload}")
						print_payload(payload)
						last_debug = time.time()
					touch_store.update_from_json(payload)
				values = extract_touch_sensors(raw_buffer)
				if not values:
					values = extract_force(raw_buffer)
				if values:
					if BLE_DEBUG and time.time() - last_debug >= BLE_DEBUG_INTERVAL:
						print(f"[BLE FALLBACK] touch_sensors={values}")
					payload = {"touch_sensors": values}
					if BLE_DEBUG and time.time() - last_debug >= BLE_DEBUG_INTERVAL:
						print_payload(payload)
						last_debug = time.time()
					touch_store.update_from_json(payload)
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
				# 检查是否已断连
				if disconnected_event.is_set():
					print(f"[BLE] 检测到断连，退出会话: {address}")
					break
				# 检查连接状态
				if not client.is_connected:
					print(f"[BLE] 连接已丢失: {address}")
					break

				now = time.time()

				# === 心跳保活: 定期向设备写入数据，防止连接空闲断开 ===
				if now - last_keepalive >= BLE_KEEPALIVE_INTERVAL:
					try:
						await client.write_gatt_char(TX_CHAR_UUID, BLE_KEEPALIVE_DATA, response=False)
						last_keepalive = now
					except Exception as e:
						print(f"[BLE] 心跳发送失败，连接可能已断开: {e}")
						break

				# 检查数据超时 - 连接还在但没数据说明通知通道可能失效
				data_age = now - last_data_time
				if data_age > BLE_DATA_TIMEOUT:
					print(f"[BLE] 数据超时 ({data_age:.1f}s 无数据)，主动断开重连: {address}")
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
	last_good_addr = None  # 记住上次成功连接的设备

	while running:
		# 优先尝试上次成功的设备
		if last_good_addr:
			addr_order = [last_good_addr] + [a for a in DEVICE_ADDRESSES if a != last_good_addr]
		else:
			addr_order = list(DEVICE_ADDRESSES)

		for addr in addr_order:
			if not running:
				break
			try:
				await ble_session(addr, parser)
				# 到这里说明连接过并且正常工作过
				last_good_addr = addr
				retry_count = 0
				print(f"[BLE] 会话结束，{BLE_RECONNECT_DELAY}秒后重连 {addr}...")
				await asyncio.sleep(BLE_RECONNECT_DELAY)
				break  # 断开后立即重连同一设备，不轮询其他设备
			except Exception as e:
				retry_count += 1
				delay = min(1.0 * retry_count, MAX_RETRY_DELAY)
				err_msg = str(e)
				if "NotPermitted" in err_msg or "Notify acquired" in err_msg:
					print(f"[BLE] 通知通道被占用，等待 BlueZ 释放资源...")
					delay = max(delay, 3.0)
				print(f"[BLE] 连接失败 {addr} (第{retry_count}次): {e}")
				print(f"[BLE] {delay:.1f}秒后重试...")
				await asyncio.sleep(delay)
		if not running:
			break


def ble_thread() -> None:
	try:
		asyncio.run(ble_main())
	except Exception as e:
		print(f"[BLE] 线程退出: {e}")


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
			print(f"[舵机{servo_id}] Ping 失败: {packetHandler.getTxRxResult(res)}")
			continue
		print(f"[舵机{servo_id}] Ping 成功, Model: {model_number}")

		torqueOff(servo_id)
		write1Byte(servo_id, ADDR_OPERATING_MODE, OP_CURRENT_BASED_POSITION)
		write2Byte(servo_id, ADDR_CURRENT_LIMIT, controller.currentLimit_mA)
		write4Byte(servo_id, ADDR_PROFILE_VELOCITY, 200)
		write4Byte(servo_id, ADDR_PROFILE_ACCELERATION, 100)
		torqueOn(servo_id)

		controller.home_pos = read4ByteSigned(servo_id, ADDR_PRESENT_POSITION)
		controller.goal_pos = controller.home_pos
		write4Byte(servo_id, ADDR_GOAL_POSITION, controller.goal_pos)

		time.sleep(0.05)

	print("\n五舵机力反馈系统就绪")


def pid_control(controller, target_force: float, feedback_force: float, dt: float) -> float:
	"""
	PID 力闭环控制
	
	参数:
		controller: ServoController 实例
		target_force: 目标力 (N)
		feedback_force: 反馈力 (N)
		dt: 时间间隔 (s)
	
	返回:
		输出电流 (mA)
	"""
	# 一阶低通滤波 - 平滑目标和反馈
	controller.filtered_target += TARGET_FILTER_ALPHA * (target_force - controller.filtered_target)
	controller.filtered_feedback += FEEDBACK_FILTER_ALPHA * (feedback_force - controller.filtered_feedback)
	
	# 计算误差: 正值表示需要拉紧，负值表示需要释放
	error = controller.filtered_target - controller.filtered_feedback
	
	# 死区处理
	if abs(error) < PID_DEADZONE:
		error = 0.0
		# 死区内缓慢衰减积分项，防止累积
		controller.pid_integral *= 0.95
	
	# ===== 顺应模式: 反馈力明显大于目标力时快速释放 =====
	if controller.filtered_feedback > controller.filtered_target + COMPLY_THRESHOLD:
		# 反馈>目标，顺着手指释放
		excess = controller.filtered_feedback - controller.filtered_target
		release_current = -COMPLY_RELEASE_GAIN * excess
		release_current = max(release_current, PID_OUTPUT_MIN)
		# 重置积分项避免切换震荡
		controller.pid_integral = 0.0
		controller.state = "COMPLY"
		return release_current
	
	# ===== PID 调节模式: 反馈力 <= 目标力 =====
	controller.state = "PID"
	
	# 比例项
	p_term = PID_KP * error
	
	# 积分项 (带抗饱和)
	controller.pid_integral += error * dt
	controller.pid_integral = max(-PID_INTEGRAL_MAX / PID_KI, 
								   min(PID_INTEGRAL_MAX / PID_KI, controller.pid_integral))
	i_term = PID_KI * controller.pid_integral
	
	# 微分项 (对误差变化率)
	if dt > 0:
		d_error = (error - controller.pid_last_error) / dt
	else:
		d_error = 0.0
	d_term = PID_KD * d_error
	controller.pid_last_error = error
	
	# PID 输出
	output = p_term + i_term + d_term
	
	# 输出限幅
	output = max(PID_OUTPUT_MIN, min(PID_OUTPUT_MAX, output))
	
	# 目标力接近0且反馈力也接近0时，输出0电流
	if controller.filtered_target < 0.1 and controller.filtered_feedback < 0.1:
		output = 0.0
		controller.pid_integral = 0.0
	
	return output


def loop() -> None:
	global running
	last_print = time.time()
	loop_start = time.time()

	while running:
		try:
			current_time = time.time()
			touch_values, touch_age = touch_store.get_values()

			for idx, controller in enumerate(servo_controllers):
				if controller.holdMode or controller.freeModeActive:
					continue
				
				# 当前只控制舵机2
				if controller.servo_id != 2:
					continue

				# ===== 主动释放模式：目标力降为0后，用负电流反转回到初始位置 =====
				if controller.releasing:
					present_pos = read4ByteSigned(controller.servo_id, ADDR_PRESENT_POSITION)
					distance = present_pos - controller.init_pos  # 正值=需要往回退
					elapsed = current_time - controller.release_start_time
					
					# 判断是否已到达初始位置或超时
					if abs(distance) <= RELEASE_POSITION_THRESHOLD or elapsed > RELEASE_TIMEOUT:
						# 释放完成，切换到位置保持模式
						write_goal_current(controller.servo_id, 0)
						set_operating_mode(controller.servo_id, OP_CURRENT_BASED_POSITION)
						write2Byte(controller.servo_id, ADDR_CURRENT_LIMIT, controller.currentLimit_mA)
						write4Byte(controller.servo_id, ADDR_PROFILE_VELOCITY, 200)
						write4Byte(controller.servo_id, ADDR_PROFILE_ACCELERATION, 100)
						write4Byte(controller.servo_id, ADDR_GOAL_POSITION, controller.init_pos)
						controller.goal_pos = controller.init_pos
						controller.releasing = False
						controller.touchMode = False
						controller.state = "IDLE"
						controller.pid_integral = 0.0
						controller.pid_output = 0.0
						reason = "到达初始位置" if abs(distance) <= RELEASE_POSITION_THRESHOLD else f"超时({elapsed:.1f}s)"
						print(f"[舵机{controller.servo_id}] 释放完成 ({reason}), 当前位置={present_pos}, 初始位置={controller.init_pos}")
					else:
						# 正在释放：施加反向电流，靠近初始位置时减速
						if abs(distance) < RELEASE_SLOWDOWN_RANGE:
							# 接近目标，按比例减小电流
							ratio = abs(distance) / RELEASE_SLOWDOWN_RANGE
							release_mA = RELEASE_CURRENT_MA * ratio
						else:
							release_mA = RELEASE_CURRENT_MA
						
						# 确定方向: 如果 present_pos > init_pos, 需要负电流(反转)
						if distance < 0:
							release_mA = -release_mA  # 反向
						
						write_goal_current(controller.servo_id, release_mA)
						controller.pid_output = release_mA
					continue
				
				# ===== 正常 PID 力控制模式 =====
				if not controller.touchMode:
					continue

				# 计算时间间隔
				if controller.pid_last_time > 0:
					dt = current_time - controller.pid_last_time
				else:
					dt = 0.02  # 首次默认50Hz
				controller.pid_last_time = current_time
				
				# 限制dt范围，防止异常
				dt = max(0.001, min(0.1, dt))

				# 获取反馈力 (BLE 已解算成 N)；≤4.903N视为0，超过后保留原值
				raw_feedbackN = touch_values[0] if touch_values else 0.0
				
				# 如果 BLE 数据过期（超过2秒没收到），启动释放
				if touch_age > 2.0:
					if not controller.releasing and controller.is_initialized and controller.init_pos is not None:
						write_goal_current(controller.servo_id, 0)
						controller.targetForceN = 0
						controller.pid_integral = 0.0
						controller.releasing = True
						controller.release_start_time = current_time
						controller.state = "RELEASING"
					elif not controller.is_initialized:
						write_goal_current(controller.servo_id, 0)
						controller.touchMode = False
						controller.state = "IDLE"
						controller.pid_integral = 0.0
					if current_time - last_print > 1.0:
						print(f"[警告] BLE 数据过期 ({touch_age:.1f}s)，舵机{controller.servo_id} 启动释放")
					continue
				
				# BLE 传感器特性: <= 基准值时为0N, > 基准值时原始值即为实际力
				if raw_feedbackN <= BLE_FORCE_BASELINE:
					feedbackN = 0.0
				else:
					feedbackN = raw_feedbackN  # 大于基准值时, 原始值就是准确的
				
				# 目标力 (来自 TCP)
				targetN = controller.targetForceN

				# 首次进入时切换到电流控制模式
				if controller.command_updated:
					controller.command_updated = False
					# 初始化滤波值
					controller.filtered_target = targetN
					controller.filtered_feedback = feedbackN
					controller.pid_integral = 0.0
					controller.pid_last_error = 0.0
					set_operating_mode(controller.servo_id, OP_CURRENT_CONTROL)
					print(f"[舵机{controller.servo_id}] 进入PID闭环, 目标={targetN:.2f}N, BLE基准={BLE_FORCE_BASELINE}N")

				# PID 闭环计算
				output_mA = pid_control(controller, targetN, feedbackN, dt)
				controller.pid_output = output_mA
				
				# 限制在舵机电流范围内
				output_mA = max(-controller.currentLimit_mA, 
							   min(controller.currentLimit_mA, output_mA))

				# 写入目标电流
				write_goal_current(controller.servo_id, output_mA)

			# 状态打印 (每秒一次)
			if current_time - last_print > 1.0:
				active_servos = [c for c in servo_controllers if c.touchMode or c.releasing]
				if active_servos:
					raw_val = touch_values[0] if touch_values else 0.0
					actual_val = 0.0 if raw_val <= BLE_FORCE_BASELINE else raw_val
					print(f"\n[状态] touch_age={touch_age:.2f}s | BLE原始={raw_val:.2f}N, 实际={actual_val:.2f}N (基准≤{BLE_FORCE_BASELINE}N→0)")
					for c in active_servos:
						if c.releasing:
							ppos = read4ByteSigned(c.servo_id, ADDR_PRESENT_POSITION)
							dist = ppos - c.init_pos if c.init_pos is not None else 0
							elapsed = current_time - c.release_start_time
							print(
								f"  舵机{c.servo_id}: state=RELEASING, "
								f"当前位置={ppos}, 初始位置={c.init_pos}, "
								f"距离={dist}, 释放电流={c.pid_output:.0f}mA, "
								f"已用时={elapsed:.1f}s"
							)
						else:
							err = c.filtered_target - c.filtered_feedback
							print(
								f"  舵机{c.servo_id}: state={c.state}, "
								f"目标={c.filtered_target:.2f}N, 反馈={c.filtered_feedback:.2f}N, "
								f"误差={err:.2f}N, 输出={c.pid_output:.0f}mA"
							)
				last_print = current_time

			# 控制频率 ~50Hz
			time.sleep(0.02)

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
		print("\n正在关闭舵机...")
		for controller in servo_controllers:
			torqueOff(controller.servo_id)
		portHandler.closePort()
		print("程序已退出")
