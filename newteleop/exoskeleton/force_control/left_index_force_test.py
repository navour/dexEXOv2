#!/usr/bin/env python3
"""左食指最小力反馈连接测试。

复用旧 ftp/Force_handcontrol.py 的舵机链路：FORCE_ENTRY 在纯电流模式
下收绳，位置稳定后切换到电流限制位置模式并锁定，释放时使用
反向电流回到启动位置。本程序仅控制 ID 7，并保留显式ARM和通信失效
关扭矩。与原程序一样默认不设位置行程限位，但可显式启用额外测试限位。
FSR使用4.903N阈值开关，不减去阈值。
"""

from __future__ import annotations

import argparse
import json
import signal
import socket
import statistics
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

FSR_DIR = Path(__file__).resolve().parent.parent / "FSR"
sys.path.insert(0, str(FSR_DIR))
from exo_pressure_common import iter_pressure_samples  # noqa: E402


ADDR_OPERATING_MODE = 11
ADDR_CURRENT_LIMIT = 38
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70
ADDR_GOAL_CURRENT = 102
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_INPUT_VOLTAGE = 144
ADDR_PRESENT_TEMPERATURE = 146
MODE_CURRENT_BASED_POSITION = 5
MODE_CURRENT_CONTROL = 0
POSITION_TICKS_PER_REV = 4096

INDEX_SERVO_ID = 7
INDEX_FSR_INDEX = 1
INDEX_INSPIRE_TOUCH_INDEX = 1  # top_touch发布顺序 [拇,食,中,无,小]


def signed16(value: int) -> int:
    return value - 65536 if value > 32767 else value


def signed32(value: int) -> int:
    return value - 4294967296 if value > 2147483647 else value


def unwrap_position(position: int, previous: int) -> int:
    """按相邻采样连续跟踪多圈位置，不固定映射到init_pos附近。"""
    turns = round((previous - position) / POSITION_TICKS_PER_REV)
    return position + turns * POSITION_TICKS_PER_REV


@dataclass
class LatestInputs:
    lock: threading.Lock
    fsr_raw: float = 0.0
    fsr_time: float = 0.0
    inspire_raw: int = 0
    inspire_force: float = 0.0
    inspire_time: float = 0.0
    inspire_valid: bool = False

    def set_fsr(self, raw: float) -> None:
        with self.lock:
            self.fsr_raw = raw
            self.fsr_time = time.monotonic()

    def set_inspire(self, raw: int, force: float, valid: bool) -> None:
        with self.lock:
            self.inspire_raw = raw
            self.inspire_force = force
            self.inspire_valid = valid
            self.inspire_time = time.monotonic()

    def snapshot(self) -> tuple[float, float, int, float, float, bool]:
        with self.lock:
            return (self.fsr_raw, self.fsr_time, self.inspire_raw, self.inspire_force,
                    self.inspire_time, self.inspire_valid)


class Controller:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.servo_id = getattr(args, "servo_id", INDEX_SERVO_ID)
        self.fsr_index = getattr(args, "finger_index", INDEX_FSR_INDEX)
        self.inspire_index = getattr(args, "finger_index", INDEX_INSPIRE_TOUCH_INDEX)
        self.finger_name = getattr(args, "finger_name", "食指")
        self.running = True
        self.armed = False
        # 只有Ping成功后才允许STOP向该ID写安全清理指令。
        # 否则启动阶段的原始通信错误会被十条清理失败淹没。
        self.hardware_ready = False
        self.fault_reason = ""
        self.state = "STOP"
        self.inputs = LatestInputs(threading.Lock())
        self.port = PortHandler(args.device)
        self.packet = PacketHandler(2.0)
        self.init_pos: int | None = args.init_pos
        self.last_position: int | None = self.init_pos
        self.last_status = 0.0
        # 按ID错开十个舵机的电流/电压/温度健康读取，避免
        # 每0.5秒同一时刻向半双工TTL总线突发40个读请求。
        self.last_health = time.monotonic() + (self.servo_id - 1) * 0.05
        self.filtered_target = 0.0
        self.filtered_measured = 0.0
        self.filtered_fsr_excess = 0.0
        self.fsr_rest_raw: float | None = None
        self.last_goal_current = 0
        self.commanded_current = 0
        # 五指上层每周期计算的收绳总电流缩放比。单指测试为1。
        self.force_entry_current_scale = 1.0
        self.release_started = 0.0
        self.release_last_dist: int | None = None
        self.release_start_dist: int | None = None
        self.release_best_abs_dist: int | None = None
        self.return_settle_started = 0.0
        self.return_stable_since = 0.0
        self.drive_started = 0.0
        self.position_history: list[tuple[float, int]] = []
        self.locked_position = 0
        self.fsr_loaded_once = False
        self.fsr_release_started = 0.0
        self.locked_release_started = 0.0
        self.low_voltage_samples = 0
        self.haptic_lock = threading.Lock()
        self.haptic_state = "STOP"
        self.haptic_step = 0.0
        self.haptic_connected = False
        # 上位机只读的舵机遥测缓存。这些字段只由已有控制
        # 周期更新，上位机不会为了刷新界面额外访问半双工TTL总线。
        self.operating_mode: int | None = None
        self.torque_enabled: bool | None = None
        self.hardware_error: int | None = None
        self.present_position_raw: int | None = None
        self.present_position: int | None = None
        self.present_current: int | None = None
        self.present_voltage: float | None = None
        self.present_temperature: int | None = None
        self.health_time = 0.0
        self.communication_errors = 0
        self.last_communication_error = ""

    def check(self, result: int, error: int, action: str) -> None:
        if result != COMM_SUCCESS:
            raise RuntimeError(f"{action}: {self.packet.getTxRxResult(result)}")
        if error:
            raise RuntimeError(f"{action}: {self.packet.getRxPacketError(error)}")

    def _retry_txrx(self, operation, action: str):
        """短暂丢包重试：10个舵机共用半双工TTL总线时，不因单次
        no status packet立即联锁。连续3次通信失败仍上报FAULT。
        """
        last = None
        for attempt in range(3):
            result_tuple = operation()
            result, error = result_tuple[-2], result_tuple[-1]
            if result == COMM_SUCCESS:
                self.check(result, error, f"{action} ID{self.servo_id}")
                return result_tuple
            last = result
            if attempt < 2:
                time.sleep(0.003)
        message = (f"{action} ID{self.servo_id}: {self.packet.getTxRxResult(last)}"
                   "（已重试3次）")
        self.communication_errors += 1
        self.last_communication_error = message
        raise RuntimeError(message)

    def read1(self, address: int) -> int:
        value, _result, _error = self._retry_txrx(
            lambda: self.packet.read1ByteTxRx(self.port, self.servo_id, address),
            f"读取地址{address}")
        return value

    def read2(self, address: int) -> int:
        value, _result, _error = self._retry_txrx(
            lambda: self.packet.read2ByteTxRx(self.port, self.servo_id, address),
            f"读取地址{address}")
        return value

    def read4(self, address: int) -> int:
        value, _result, _error = self._retry_txrx(
            lambda: self.packet.read4ByteTxRx(self.port, self.servo_id, address),
            f"读取地址{address}")
        return value

    def write1(self, address: int, value: int) -> None:
        self._retry_txrx(
            lambda: self.packet.write1ByteTxRx(
                self.port, self.servo_id, address, value),
            f"写地址{address}")
        if address == ADDR_OPERATING_MODE:
            self.operating_mode = int(value)
        elif address == ADDR_TORQUE_ENABLE:
            self.torque_enabled = bool(value)

    def write2(self, address: int, value: int) -> None:
        self._retry_txrx(
            lambda: self.packet.write2ByteTxRx(
                self.port, self.servo_id, address, value & 0xFFFF),
            f"写地址{address}")

    def write4(self, address: int, value: int) -> None:
        self._retry_txrx(
            lambda: self.packet.write4ByteTxRx(
                self.port, self.servo_id, address, value & 0xFFFFFFFF),
            f"写地址{address}")

    def start_inputs(self) -> None:
        threading.Thread(target=self._fsr_loop, daemon=True).start()
        threading.Thread(target=self._inspire_loop, daemon=True).start()
        if self.args.enable_mhandpro:
            threading.Thread(target=self._haptic_loop, daemon=True).start()

    def set_haptic(self, state: str, index_step: float = 0.0) -> None:
        if not self.args.enable_mhandpro:
            return
        with self.haptic_lock:
            self.haptic_state = state
            self.haptic_step += index_step

    def _haptic_loop(self) -> None:
        period = 1.0 / self.args.control_hz
        while self.running:
            sock = None
            try:
                sock = socket.create_connection(
                    (self.args.haptic_host, self.args.haptic_port), timeout=3.0)
                sock.settimeout(1.0)
                with self.haptic_lock:
                    self.haptic_connected = True
                print(f"[mHandPro] 已连接力反馈覆盖 "
                      f"{self.args.haptic_host}:{self.args.haptic_port}")
                while self.running:
                    started = time.monotonic()
                    with self.haptic_lock:
                        state = self.haptic_state
                        step = self.haptic_step
                        self.haptic_step = 0.0
                    packet = {"type": "haptic_override", "state": state,
                              "index_step": step}
                    sock.sendall((json.dumps(packet, separators=(",", ":")) + "\n").encode())
                    delay = period - (time.monotonic() - started)
                    if delay > 0:
                        time.sleep(delay)
            except Exception as exc:
                if self.running:
                    print(f"[mHandPro] {exc}；1秒后重连")
                    time.sleep(1.0)
            finally:
                with self.haptic_lock:
                    self.haptic_connected = False
                if sock:
                    sock.close()

    def _fsr_loop(self) -> None:
        while self.running:
            try:
                for sample in iter_pressure_samples(
                    self.args.fsr_host, self.args.fsr_port, data_timeout=1.0):
                    if not self.running:
                        return
                    self.inputs.set_fsr(sample.values[self.fsr_index])
            except Exception as exc:
                if self.running:
                    print(f"[FSR] {exc}；1秒后重连")
                    time.sleep(1.0)

    def _inspire_loop(self) -> None:
        while self.running:
            sock = None
            try:
                sock = socket.create_connection(
                    (self.args.force_host, self.args.force_port), timeout=3.0)
                sock.settimeout(1.0)
                print(f"[Inspire] 已连接力流 {self.args.force_host}:{self.args.force_port}")
                buffer = ""
                while self.running:
                    chunk = sock.recv(4096)
                    if not chunk:
                        raise ConnectionError("力流连接断开")
                    buffer += chunk.decode("utf-8", errors="strict")
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        if not line.strip():
                            continue
                        packet = json.loads(line)
                        raw_values = packet.get("top_touch_raw_max", [])
                        force_values = packet.get("top_touch_force_n", [])
                        valid = (bool(packet.get("touch_valid"))
                                 and len(raw_values) == 5 and len(force_values) == 5)
                        raw = int(raw_values[self.inspire_index]) if valid else 0
                        force = float(force_values[self.inspire_index]) if valid else 0.0
                        self.inputs.set_inspire(raw, force, valid)
            except Exception as exc:
                self.inputs.set_inspire(0, 0.0, False)
                if self.running:
                    print(f"[Inspire] {exc}；1秒后重连")
                    time.sleep(1.0)
            finally:
                if sock:
                    sock.close()

    def open(self) -> None:
        if not self.port.openPort():
            raise RuntimeError(f"无法打开{self.args.device}")
        if not self.port.setBaudRate(self.args.baudrate):
            raise RuntimeError(f"无法设置波特率{self.args.baudrate}")
        model, result, error = self.packet.ping(self.port, self.servo_id)
        self.check(result, error, f"Ping ID{self.servo_id}")
        self.hardware_ready = True
        mode = self.read1(ADDR_OPERATING_MODE)
        torque = self.read1(ADDR_TORQUE_ENABLE)
        hw_error = self.read1(ADDR_HARDWARE_ERROR)
        position_raw = signed32(self.read4(ADDR_PRESENT_POSITION))
        position = position_raw
        self.operating_mode = mode
        self.torque_enabled = bool(torque)
        self.hardware_error = hw_error
        self.present_position_raw = position_raw
        self.present_position = position
        self.health_time = time.monotonic()
        self.last_position = position
        print(f"{self.finger_name} ID{self.servo_id} model={model}, mode={mode}, torque={torque}, "
              f"hw_error=0x{hw_error:02X}, position_raw={position_raw}, "
              f"position_cont={position}, init_pos={self.init_pos if self.init_pos is not None else '未INIT'}")
        if torque:
            raise RuntimeError("启动时扭矩未关闭，拒绝接管")
        if mode != MODE_CURRENT_BASED_POSITION:
            raise RuntimeError(f"ID{self.servo_id}工作模式为{mode}，必须先安全配置为模式5")
        if hw_error:
            raise RuntimeError(f"ID{self.servo_id}存在硬件错误0x{hw_error:02X}")
        if (self.init_pos is not None
                and abs(position - self.init_pos) > self.args.init_start_tolerance):
            raise RuntimeError(
                f"当前位置{position}偏离init_pos={self.init_pos}超过"
                f"{self.args.init_start_tolerance} tick")

    def data_ready(self) -> tuple[bool, str]:
        fsr_raw, fsr_t, _, _, inspire_t, inspire_valid = self.inputs.snapshot()
        now = time.monotonic()
        if not fsr_t or now - fsr_t > self.args.input_timeout:
            return False, "FSR数据未就绪或超时"
        if not inspire_t or now - inspire_t > self.args.input_timeout or not inspire_valid:
            return False, "Inspire力数据未就绪、无效或超时"
        if fsr_raw < 0 or fsr_raw > self.args.fsr_raw_max:
            return False, f"FSR原始值异常: {fsr_raw}"
        if self.args.enable_mhandpro:
            with self.haptic_lock:
                if not self.haptic_connected:
                    return False, "mHandPro力反馈覆盖9302未连接"
        return True, ""

    def arm(self) -> None:
        if not self.args.enable_write:
            print("[只读模式] 未传--enable-write，不能ARM")
            return
        if self.init_pos is None or self.fsr_rest_raw is None:
            print("不能ARM: 请先将食指置于本轮返回起点，"
                  "保持绑带预载稳定后输入 INIT")
            return
        ready, reason = self.data_ready()
        if not ready:
            print(f"不能ARM: {reason}")
            return
        position_raw = signed32(self.read4(ADDR_PRESENT_POSITION))
        reference = self.last_position if self.last_position is not None else self.init_pos
        position = unwrap_position(position_raw, reference)
        self.last_position = position
        self.present_position_raw = position_raw
        self.present_position = position
        if abs(position - self.init_pos) > self.args.init_start_tolerance:
            print(f"不能ARM: position_raw={position_raw}, position_cont={position} "
                  f"偏离init_pos={self.init_pos}")
            return
        self.armed = True
        self.fault_reason = ""
        self.state = "FREE"
        self.filtered_target = 0.0
        self.filtered_measured = 0.0
        self.filtered_fsr_excess = 0.0
        self.last_goal_current = 0
        self.commanded_current = 0
        self.release_started = 0.0
        self.release_last_dist = None
        self.release_start_dist = None
        self.release_best_abs_dist = None
        self.drive_started = 0.0
        self.return_settle_started = 0.0
        self.return_stable_since = 0.0
        self.position_history = []
        self.locked_position = 0
        self.fsr_loaded_once = False
        self.fsr_release_started = 0.0
        self.locked_release_started = 0.0
        self.set_haptic("GLOVE")
        print(f"已ARM：{self.finger_name} ID{self.servo_id}在init_pos待机，"
              f"Inspire{self.finger_name}受力后进入FORCE_ENTRY。")

    def record_init_pos(self) -> None:
        """复用旧Force_handcontrol.py的INIT交互，只读记录本次运行起点。"""
        if self.armed or self.state not in ("STOP", "FAULT"):
            print("不能INIT: 请先STOP，确认舵机已关扭矩")
            return
        if self.read1(ADDR_TORQUE_ENABLE) != 0:
            print(f"不能INIT: ID{self.servo_id}扭矩未关闭")
            return
        ready, reason = self.data_ready()
        if not ready:
            print(f"不能INIT: {reason}")
            return
        samples: list[int] = []
        fsr_samples: list[float] = []
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            samples.append(signed32(self.read4(ADDR_PRESENT_POSITION)))
            fsr_samples.append(self.inputs.snapshot()[0])
            time.sleep(0.02)
        # 0.5秒采样不应跨越回绕；取中位数抵抗单帧抖动。
        span = max(samples) - min(samples)
        if span > 8:
            print(f"不能INIT: 0.5秒位置波动{span} tick超过8，请保持不动")
            return
        fsr_span = max(fsr_samples) - min(fsr_samples)
        fsr_rest = float(statistics.median(fsr_samples))
        if fsr_span > self.args.fsr_init_max_span:
            print(f"不能INIT: 0.5秒FSR预载波动{fsr_span:.3f}N超过"
                  f"{self.args.fsr_init_max_span:.3f}N")
            return
        if fsr_rest > self.args.fsr_rest_max:
            print(f"不能INIT: FSR绑带预载={fsr_rest:.3f}N超过"
                  f"{self.args.fsr_rest_max:.3f}N，请放松绑带")
            return
        self.init_pos = round(statistics.median(samples))
        self.fsr_rest_raw = fsr_rest
        self.last_position = self.init_pos
        self.fault_reason = ""
        self.state = "STOP"
        print(f"[INIT] 本次运行init_pos={self.init_pos}（释放返回点），"
              f"位置波动={span} tick，FSR绑带预载={fsr_rest:.3f}N，"
              f"FSR波动={fsr_span:.3f}N")

    def set_operating_mode(self, mode: int, *, torque: bool) -> None:
        """按旧程序顺序：关扭矩 -> 切模式 -> 设电流限制 -> 可选开扭矩。"""
        self.write2(ADDR_GOAL_CURRENT, 0)
        self.write1(ADDR_TORQUE_ENABLE, 0)
        self.write1(ADDR_OPERATING_MODE, mode)
        self.write2(ADDR_CURRENT_LIMIT, self.args.max_goal_current)
        if torque:
            self.write1(ADDR_TORQUE_ENABLE, 1)

    def enter_force_entry(self, position: int, now: float) -> bool:
        assert self.init_pos is not None
        if abs(position - self.init_pos) > self.args.init_start_tolerance:
            self.stop(
                f"接触时position={position}偏离init_pos={self.init_pos}过大",
                fault=True,
            )
            return False
        self.set_operating_mode(MODE_CURRENT_CONTROL, torque=True)
        self.last_goal_current = 0
        self.commanded_current = 0
        self.drive_started = now
        self.position_history = [(now, position)]
        # 每次新的接触周期重新确认FSR是否真正加载过。坏通道若始终停在
        # INIT基线，不能凭“新增力接近0”直接触发释放。
        self.fsr_loaded_once = False
        self.fsr_release_started = 0.0
        self.locked_release_started = 0.0
        self.state = "FORCE_ENTRY"
        # 为了能完成整手包络，寻触阶段不冻结Inspire；进入LOCKED才保持整手。
        self.set_haptic("FORCE_ENTRY")
        print(f"[FORCE_ENTRY] 按旧程序切到模式0，正电流收绳，起点={position}；"
              "Inspire整手继续跟随mHandPro")
        return True

    def position_is_stable(self, now: float) -> bool:
        if now - self.drive_started < self.args.lock_min_drive_time:
            return False
        window = [position for stamp, position in self.position_history
                  if now - stamp <= self.args.lock_detect_window]
        return (len(window) >= 3
                and max(window) - min(window) <= self.args.lock_position_threshold)

    def lock_condition_met(self, now: float) -> bool:
        """只有外骨骼FSR确实加载且舵机稳定时才允许进入LOCKED。

        仅凭位置稳定会把“低电流带不动/拉绳仍松弛”误判成已经接触，
        界面虽然显示LOCKED，实际机构却没有建立力反馈。
        """
        return self.fsr_loaded_once and self.position_is_stable(now)

    def enter_locked(self, position: int) -> None:
        self.write2(ADDR_GOAL_CURRENT, 0)
        time.sleep(0.02)
        self.set_operating_mode(MODE_CURRENT_BASED_POSITION, torque=True)
        self.write4(ADDR_PROFILE_VELOCITY, 0)
        self.write4(ADDR_PROFILE_ACCELERATION, 0)
        self.write4(ADDR_GOAL_POSITION, position)
        self.last_goal_current = 0
        self.commanded_current = 0
        self.locked_position = position
        self.fsr_release_started = 0.0
        self.locked_release_started = 0.0
        self.state = "LOCKED"
        self.set_haptic("LOCKED")
        print(f"[LOCKED] 位置稳定，按旧程序切模式5锁定当前位置={position}")

    def observe_fsr_load(self) -> None:
        """记录本次接触周期中FSR是否跨过加载阈值。"""
        if (not self.fsr_loaded_once
                and self.filtered_fsr_excess >= self.args.fsr_load_threshold):
            self.fsr_loaded_once = True
            print(f"[FSR加载] {self.finger_name} ID{self.servo_id}相对INIT新增力="
                  f"{self.filtered_fsr_excess:.3f}N，已达到"
                  f"{self.args.fsr_load_threshold:.3f}N加载阈值。")

    def locked_release_reason(self, now: float, inspire_released: bool) -> str:
        """分别消抖INSPIRE释放与FSR两阶段卸载，返回成熟的释放原因。"""
        if inspire_released:
            if not self.locked_release_started:
                self.locked_release_started = now
        else:
            self.locked_release_started = 0.0

        fsr_unloaded = (
            self.fsr_loaded_once
            and self.filtered_fsr_excess <= self.args.exo_zero_threshold
        )
        if fsr_unloaded:
            if not self.fsr_release_started:
                self.fsr_release_started = now
        else:
            self.fsr_release_started = 0.0

        inspire_confirmed = (
            bool(self.locked_release_started)
            and now - self.locked_release_started >= self.args.release_hold_seconds
        )
        fsr_confirmed = (
            bool(self.fsr_release_started)
            and now - self.fsr_release_started >= self.args.fsr_release_hold_seconds
        )
        reasons = []
        if inspire_confirmed:
            reasons.append("Inspire触觉释放")
        if fsr_confirmed:
            reasons.append("FSR加载后卸载")
        return "+".join(reasons)

    def enter_release(self, now: float, reason: str = "") -> None:
        self.set_operating_mode(MODE_CURRENT_CONTROL, torque=True)
        self.release_started = now
        self.release_last_dist = None
        self.release_start_dist = None
        self.release_best_abs_dist = None
        self.last_goal_current = 0
        self.commanded_current = 0
        self.state = "RELEASE"
        self.set_haptic("RELEASE")
        suffix = f"，触发={reason}" if reason else ""
        print(f"[RELEASE] 切模式0，反向电流回init_pos{suffix}。")

    def tick_release(self, now: float, position: int) -> None:
        """复用旧程序的独立RELEASE：归位期间不再受Inspire目标力影响。"""
        assert self.init_pos is not None
        dist = position - self.init_pos
        elapsed = now - self.release_started
        if self.release_start_dist is None:
            self.release_start_dist = dist
            self.release_best_abs_dist = abs(dist)
        else:
            assert self.release_best_abs_dist is not None
            self.release_best_abs_dist = min(self.release_best_abs_dist, abs(dist))
        # 电流模式下不能只靠“当前帧落入小容差”判定到位：
        # 高归位电流可能一帧从INIT一侧跨到另一侧，随后按误差
        # 反向写电流，从而在目标两侧往复振荡直到超时。首次跨过
        # INIT与进入容差等价，都应立即清电流并切模式5保持。
        crossed_init = (
            self.release_last_dist is not None
            and ((self.release_last_dist > 0 >= dist)
                 or (self.release_last_dist < 0 <= dist))
        )
        self.release_last_dist = dist
        if abs(dist) <= self.args.release_position_threshold or crossed_init:
            self.write2(ADDR_GOAL_CURRENT, 0)
            self.commanded_current = 0
            self.write1(ADDR_TORQUE_ENABLE, 0)
            self.write1(ADDR_OPERATING_MODE, MODE_CURRENT_BASED_POSITION)
            self.write2(ADDR_CURRENT_LIMIT, self.args.max_goal_current)
            self.write4(ADDR_PROFILE_VELOCITY, 50)
            self.write4(ADDR_PROFILE_ACCELERATION, 30)
            self.write4(ADDR_GOAL_POSITION, self.init_pos)
            self.write1(ADDR_TORQUE_ENABLE, 1)
            self.state = "RETURN_SETTLE"
            self.release_started = 0.0
            self.release_last_dist = None
            self.release_start_dist = None
            self.release_best_abs_dist = None
            self.return_settle_started = now
            self.return_stable_since = 0.0
            arrival = "跨过init_pos" if crossed_init else "进入init_pos容差"
            print(f"[RETURN_SETTLE] {arrival}，已清电流并切模式5保持；"
                  "等待位置稳定、Inspire接触释放。")
            return
        if elapsed >= self.args.release_timeout:
            self.stop(
                f"释放超时{self.args.release_timeout:.1f}s: "
                f"position={position}, init_pos={self.init_pos}, "
                f"start_dist={self.release_start_dist}, "
                f"best_abs_dist={self.release_best_abs_dist}, "
                f"goal_current={self.commanded_current:+d}", fault=True)
            return

        magnitude = float(self.args.release_goal_current)
        if 0 < abs(dist) < self.args.release_slowdown_range:
            ratio = abs(dist) / self.args.release_slowdown_range
            magnitude = max(self.args.release_min_current, magnitude * ratio)
        # 尚未到达/跨过INIT时，每周期依据当前距离计算归位方向。
        direction = -1 if dist * self.args.drive_current_sign > 0 else 1
        self.commanded_current = int(round(
            self.args.drive_current_sign * direction * magnitude))
        self.write2(ADDR_GOAL_CURRENT, self.commanded_current)

    def tick_return_settle(
        self, now: float, position: int, inspire_released: bool
    ) -> None:
        """归位后先消除惯性/绳索弹性过冲，并等Inspire确实释放。"""
        assert self.init_pos is not None
        self.write4(ADDR_GOAL_POSITION, self.init_pos)
        position_ok = abs(position - self.init_pos) <= self.args.release_position_threshold
        if position_ok and inspire_released:
            if not self.return_stable_since:
                self.return_stable_since = now
            elif now - self.return_stable_since >= self.args.return_settle_time:
                self.state = "FREE"
                self.return_settle_started = 0.0
                self.return_stable_since = 0.0
                self.set_haptic("GLOVE")
                print("[FREE] init_pos已稳定且Inspire已释放，"
                      "等待新的接触上升沿。")
        else:
            self.return_stable_since = 0.0

    def stop(self, reason: str, *, fault: bool = False) -> None:
        was_active = self.armed or self.state not in ("STOP", "FAULT")
        self.armed = False
        self.last_goal_current = 0
        self.commanded_current = 0
        self.fsr_loaded_once = False
        self.fsr_release_started = 0.0
        self.locked_release_started = 0.0
        try:
            if self.args.enable_write and self.hardware_ready and was_active:
                self.write2(ADDR_GOAL_CURRENT, 0)
                self.write1(ADDR_TORQUE_ENABLE, 0)
                self.write1(ADDR_OPERATING_MODE, MODE_CURRENT_BASED_POSITION)
                self.write2(ADDR_CURRENT_LIMIT, self.args.max_goal_current)
        except Exception as exc:
            print(f"[严重] 停止清理失败: {exc}", file=sys.stderr)
        self.state = "FAULT" if fault else "STOP"
        self.fault_reason = reason if fault else ""
        self.set_haptic("STOP")
        if was_active or fault:
            print(f"[{self.state}] {reason}")

    def loop(self) -> None:
        period = 1.0 / self.args.control_hz
        while self.running:
            started = time.monotonic()
            if self.armed:
                try:
                    self.tick(started)
                except Exception as exc:
                    self.stop(str(exc), fault=True)
            delay = period - (time.monotonic() - started)
            if delay > 0:
                time.sleep(delay)

    def tick(self, now: float) -> None:
        ready, reason = self.data_ready()
        if not ready:
            raise RuntimeError(reason)
        fsr_raw, _, inspire_raw, inspire_force, _, _ = self.inputs.snapshot()
        assert self.fsr_rest_raw is not None  # ARM前INIT已采集绑带预载
        # STM32仍使用绝对力原值；INIT另外记录安装绑带带来的静态预载。
        # 闭环使用“4.903N物理底值 + 相对预载的新增力”，不把绑带力当作手指受力。
        fsr_excess = max(0.0, fsr_raw - self.fsr_rest_raw
                         - self.args.fsr_preload_deadband)
        measured = (0.0 if fsr_excess <= 0.0
                    else self.args.fsr_threshold + fsr_excess)
        target_unscaled = max(0.0, inspire_force)
        target_scaled = min(
            self.args.target_max,
            target_unscaled * self.args.comfort_scale,
        )
        # 状态触发使用未经EMA的最新INSPIRE力，以避免接触/释放
        # 额外等待滤波值跨过0.5/0.3N。两阈值之间保持当前状态，
        # 形成滞回；EMA仅用于收绳电流和LOCKED力差PID。
        contact_active = target_scaled >= self.args.contact_on
        contact_released = target_scaled <= self.args.contact_off
        alpha = self.args.filter_alpha
        self.filtered_target += alpha * (target_scaled - self.filtered_target)
        self.filtered_measured += alpha * (measured - self.filtered_measured)
        self.filtered_fsr_excess += alpha * (fsr_excess - self.filtered_fsr_excess)

        position_raw = signed32(self.read4(ADDR_PRESENT_POSITION))
        assert self.init_pos is not None  # ARM前已强制INIT或加载init_pos
        reference = self.last_position if self.last_position is not None else self.init_pos
        position = unwrap_position(position_raw, reference)
        self.last_position = position
        self.present_position_raw = position_raw
        self.present_position = position
        maximum = self.init_pos + self.args.drive_current_sign * self.args.max_travel
        low, high = sorted((self.init_pos, maximum))
        if self.args.max_travel > 0 and self.state in ("FORCE_ENTRY", "LOCKED") and (
            position < low - self.args.position_tolerance
            or position > high + self.args.position_tolerance
        ):
            raise RuntimeError(f"位置越界: {position}, 允许{low}..{high}")

        # RELEASE与旧程序一样是独立状态：不因Inspire再次受力而中断归位。
        if self.state == "RELEASE":
            self.tick_release(now, position)
        elif self.state == "RETURN_SETTLE":
            self.tick_return_settle(now, position, contact_released)
        elif self.state == "LOCKED":
            self.observe_fsr_load()
            self.write4(ADDR_GOAL_POSITION, self.locked_position)
            # 复用原程序LOCKED PID：目标=外骨骼FSR，反馈=Inspire top_touch。
            force_error = self.filtered_measured - self.filtered_target
            index_step = 0.0
            if abs(force_error) > self.args.force_deadzone:
                index_step = max(-self.args.force_step_max, min(
                    self.args.force_step_max, self.args.force_kp * force_error))
            self.set_haptic("LOCKED", index_step)
            # INSPIRE释放仍是主要请求。FSR必须先跨过加载阈值，随后降到
            # 独立的卸载阈值以下并持续足够时间，才可作为备用释放请求。
            release_reason = self.locked_release_reason(now, contact_released)
            if release_reason:
                self.enter_release(now, release_reason)
        else:
            if self.state == "FREE" and contact_active:
                if not self.enter_force_entry(position, now):
                    return
            if self.state == "FORCE_ENTRY" and contact_released:
                self.enter_release(now, "Inspire触觉释放")
            elif self.state == "FORCE_ENTRY":
                self.observe_fsr_load()
                self.release_started = 0.0
                seek_travel = ((position - self.init_pos)
                               * self.args.drive_current_sign)
                # 切到模式0后齿隙、松绳回弹和单帧位置抖动可造成
                # 十几tick的瞬时反向。原程序不做这个检查；本程序只在
                # 最短驱动时间后仍累计反向超过明确阈值时联锁。
                if (now - self.drive_started >= self.args.lock_min_drive_time
                        and seek_travel < -self.args.direction_fault_threshold):
                    raise RuntimeError(
                        f"FORCE_ENTRY运动方向异常: travel={seek_travel} tick")
                if (self.args.seek_max_travel > 0
                        and seek_travel > self.args.seek_max_travel):
                    raise RuntimeError(
                        f"FORCE_ENTRY寻触行程超限: {seek_travel} > "
                        f"{self.args.seek_max_travel} tick")
                if (self.args.seek_timeout > 0
                        and now - self.drive_started > self.args.seek_timeout):
                    raise RuntimeError(
                        f"FORCE_ENTRY寻触超时{self.args.seek_timeout:.1f}s，"
                        f"行程={seek_travel} tick")
                self.position_history.append((now, position))
                cutoff = now - self.args.lock_detect_window * 2.0
                self.position_history = [item for item in self.position_history
                                         if item[0] >= cutoff]
                requested = round(min(
                    self.args.max_goal_current,
                    self.filtered_target * self.args.force_to_current_gain,
                ) * self.force_entry_current_scale)
                current = min(requested, self.last_goal_current + self.args.current_slew)
                self.last_goal_current = current
                self.commanded_current = self.args.drive_current_sign * current
                self.write2(ADDR_GOAL_CURRENT, self.commanded_current)
                if self.lock_condition_met(now):
                    self.enter_locked(position)

        if now - self.last_health >= self.args.health_interval:
            self.last_health = now
            hw_error = self.read1(ADDR_HARDWARE_ERROR)
            current_actual = signed16(self.read2(ADDR_PRESENT_CURRENT))
            voltage = self.read2(ADDR_PRESENT_INPUT_VOLTAGE) / 10.0
            temperature = self.read1(ADDR_PRESENT_TEMPERATURE)
            self.hardware_error = hw_error
            self.present_current = current_actual
            self.present_voltage = voltage
            self.present_temperature = temperature
            self.health_time = now
            if hw_error:
                raise RuntimeError(
                    f"{self.finger_name} ID{self.servo_id}硬件错误0x{hw_error:02X}")
            if abs(current_actual) > self.args.actual_current_limit:
                raise RuntimeError(
                    f"{self.finger_name} ID{self.servo_id}实际电流越限: "
                    f"{current_actual} raw")
            if temperature > self.args.temperature_limit:
                raise RuntimeError(
                    f"{self.finger_name} ID{self.servo_id}温度越限: "
                    f"{temperature}°C")
            # XL330-M288官方工作下限3.7V。3.7~4.0V可能是负载切换时
            # 的瞬态压降，连续3次健康检查才联锁；<3.7V仍立即停止。
            if voltage < self.args.voltage_min:
                self.low_voltage_samples += 1
            else:
                self.low_voltage_samples = 0
            voltage_fault = (
                voltage < 3.7
                or self.low_voltage_samples >= 3
                or voltage > self.args.voltage_max
            )
            if voltage_fault:
                raise RuntimeError(
                    f"{self.finger_name} ID{self.servo_id}输入电压异常: "
                    f"{voltage:.1f}V，软件允许"
                    f"{self.args.voltage_min:.1f}..{self.args.voltage_max:.1f}V，"
                    f"连续低压采样={self.low_voltage_samples}，"
                    f"当前电流={current_actual} raw")

        # 单指程序保留详细周期日志；五指/双手程序由上层按手汇总，
        # 避免十根手指的日志交错后无法识别手别和指别。
        if (getattr(self.args, "periodic_status", True)
                and now - self.last_status >= 0.5):
            self.last_status = now
            print(f"[{self.state}] Inspire top_touch max={inspire_raw} "
                  f"fit={inspire_force:.3f}N target={self.filtered_target:.3f}N "
                  f"FSR raw={fsr_raw:.3f} rest={self.fsr_rest_raw:.3f} "
                  f"excess={self.filtered_fsr_excess:.3f}N "
                  f"feedback={self.filtered_measured:.3f}N "
                          f"goal_current={self.commanded_current:+d} "
                          f"pos_raw={position_raw} pos_cont={position}")

    def telemetry_snapshot(self, now: float | None = None) -> dict:
        """返回一根手指的纯缓存遥测，不触发任何硬件读写。"""
        now = time.monotonic() if now is None else now
        fsr_raw, fsr_time, inspire_raw, inspire_force, inspire_time, valid = (
            self.inputs.snapshot()
        )
        rest = self.fsr_rest_raw
        excess = None if rest is None else max(
            0.0, fsr_raw - rest - self.args.fsr_preload_deadband)
        return {
            "name": self.finger_name,
            "servo_id": self.servo_id,
            "state": self.state,
            "armed": self.armed,
            "fault": self.fault_reason or None,
            "fsr": {
                "raw_n": fsr_raw,
                "rest_n": rest,
                "excess_n": excess,
                "filtered_excess_n": self.filtered_fsr_excess,
                "age_ms": round((now - fsr_time) * 1000) if fsr_time else None,
            },
            "inspire": {
                "raw": inspire_raw,
                "force_n": inspire_force,
                "filtered_target_n": self.filtered_target,
                "valid": valid,
                "age_ms": round((now - inspire_time) * 1000)
                if inspire_time else None,
            },
            "servo": {
                "operating_mode": self.operating_mode,
                "torque_enabled": self.torque_enabled,
                "hardware_error": self.hardware_error,
                "init_position": self.init_pos,
                "position_raw": self.present_position_raw,
                "position": self.present_position,
                "goal_current": self.commanded_current,
                "present_current": self.present_current,
                "voltage_v": self.present_voltage,
                "temperature_c": self.present_temperature,
                "health_age_ms": round((now - self.health_time) * 1000)
                if self.health_time else None,
                "communication_errors": self.communication_errors,
                "last_communication_error": self.last_communication_error or None,
            },
        }

    def close(self) -> None:
        self.running = False
        self.stop("程序退出")
        self.port.closePort()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="左食指ID7、4.903N阈值开关力反馈测试",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--enable-write", action="store_true",
                        help="允许写ID7并ARM；默认只读")
    parser.add_argument("--device", default="/dev/serial0",
                        help="Dynamixel TTL串口设备")
    parser.add_argument("--baudrate", type=int, default=1_000_000,
                        help="Dynamixel总线波特率")
    parser.add_argument("--init-pos", type=int,
                        help="显式指定释放返回位置(tick)；默认由INIT采集")
    parser.add_argument("--fsr-host", default="127.0.0.1",
                        help="FSR BLE Broker主机")
    parser.add_argument("--fsr-port", type=int, default=9002,
                        help="左手FSR BLE Broker TCP端口")
    parser.add_argument("--force-host", default="127.0.0.1",
                        help="INSPIRE触觉力流主机")
    parser.add_argument("--force-port", type=int, default=9202,
                        help="左手INSPIRE触觉力流TCP端口")
    parser.add_argument("--enable-mhandpro", action="store_true",
                        help="通过9302保持整手抓取姿态，并对食指执行力反馈")
    parser.add_argument("--haptic-host", default="127.0.0.1",
                        help="G1力控覆盖主机")
    parser.add_argument("--haptic-port", type=int, default=9302,
                        help="左手G1力控覆盖TCP端口")
    parser.add_argument("--fsr-threshold", type=float, default=4.903,
                        help="STM32最低有效力阈值；≤阈值为0，>阈值保留原值")
    parser.add_argument("--fsr-raw-max", type=float, default=60.0,
                        help="FSR输入值超过此值时判为数据异常(N)")
    parser.add_argument("--fsr-rest-max", type=float, default=8.0,
                        help="INIT允许的最大绑带静态预载")
    parser.add_argument("--fsr-init-max-span", type=float, default=0.20,
                        help="INIT 0.5秒内FSR最大允许波动")
    parser.add_argument("--fsr-preload-deadband", type=float, default=0.05,
                        help="相对INIT预载的FSR噪声死区")
    parser.add_argument("--comfort-scale", type=float, default=1.0,
                        help="INSPIRE力进入状态机前的整体缩放系数")
    parser.add_argument("--target-max", type=float, default=6.0,
                        help="INSPIRE目标力软件上限(N)")
    parser.add_argument("--contact-on", type=float, default=0.50,
                        help="Inspire接触进入阈值；默认恢复旧程序0.50N")
    parser.add_argument("--contact-off", type=float, default=0.30,
                        help="Inspire接触释放阈值，低于contact-on形成滞回")
    parser.add_argument("--force-to-current-gain", type=float, default=60.0,
                        help="复用旧程序HAND_TO_SERVO_GAIN，默60 mA/N")
    parser.add_argument("--current-slew", type=int, default=30,
                        help="Goal Current每个控制周期最大增量")
    parser.add_argument("--max-goal-current", type=int, default=30,
                        help="原程序为300；单指首试默认限制30")
    parser.add_argument("--release-goal-current", type=int, default=30,
                        help="RELEASE归位Goal Current上限(raw)")
    parser.add_argument("--release-min-current", type=int, default=10,
                        help="接近INIT减速时保留的最小归位电流(raw)")
    parser.add_argument("--release-slowdown-range", type=int, default=50,
                        help="距INIT小于此距离时开始按比例减小归位电流(tick)")
    parser.add_argument("--release-min-time", type=float, default=0.3,
                        help="RELEASE允许判定到位前的最小运行时间(秒)")
    parser.add_argument("--release-timeout", type=float, default=12.0,
                        help="原程序150mA/4s；本测试30mA默12s以返回长行程")
    parser.add_argument("--actual-current-limit", type=int, default=45,
                        help="Present Current绝对值故障阈值(raw)")
    parser.add_argument("--max-travel", type=int, default=0,
                        help="0=与原程序一样不设位置限位；>0时启用额外测试限位")
    parser.add_argument("--seek-max-travel", type=int, default=0,
                        help="0=与源程序一样不设FORCE_ENTRY行程上限；>0启用额外保护")
    parser.add_argument("--seek-timeout", type=float, default=0.0,
                        help="0=与源程序一样不设FORCE_ENTRY超时；>0启用额外保护")
    parser.add_argument("--drive-current-sign", type=int, choices=(-1, 1), default=1,
                        help="复用原程序SEEK_CURRENT_SIGN；当前机构正电流收绳")
    parser.add_argument("--direction-fault-threshold", type=int, default=100,
                        help="最短驱动时间后累计反向超过此tick才故障")
    parser.add_argument("--lock-position-threshold", type=int, default=8,
                        help="稳定检测窗口内位置极差不超过此值才LOCKED(tick)")
    parser.add_argument("--lock-detect-window", type=float, default=0.25,
                        help="FORCE_ENTRY舵机位置稳定检测窗口(秒)")
    parser.add_argument("--lock-min-drive-time", type=float, default=0.2,
                        help="FORCE_ENTRY开始后最早允许LOCKED的时间(秒)")
    parser.add_argument("--force-kp", type=float, default=35.0,
                        help="LOCKED中FSR力与INSPIRE力误差的位置比例增益(tick/N)")
    parser.add_argument("--force-step-max", type=float, default=3.0,
                        help="LOCKED PID每周期允许的INSPIRE最大位置修正(tick)")
    parser.add_argument("--force-deadzone", type=float, default=0.10,
                        help="LOCKED力误差死区(N)")
    parser.add_argument("--fsr-load-threshold", type=float, default=0.30,
                        help="FSR相对INIT新增力达到此值后确认本轮加载(N)")
    parser.add_argument("--fsr-release-threshold", "--exo-zero-threshold",
                        dest="exo_zero_threshold", type=float, default=0.15,
                        help="已加载FSR降到此值以下时开始卸载计时(N)")
    parser.add_argument("--fsr-release-hold-seconds", type=float, default=0.30,
                        help="FSR加载后卸载条件必须连续成立的时间(秒)")
    parser.add_argument("--release-hold-seconds", type=float, default=0.15,
                        help="LOCKED中INSPIRE触觉释放请求的消抖时间(秒)")
    parser.add_argument("--position-tolerance", type=int, default=10,
                        help="位置行程检查允许的越界容差(tick)")
    parser.add_argument("--release-position-threshold", type=int, default=15,
                        help="复用原程序RELEASE_POSITION_THR")
    parser.add_argument("--return-settle-time", type=float, default=0.5,
                        help="归位后位置与Inspire释放连续稳定时间")
    parser.add_argument("--init-start-tolerance", type=int, default=40,
                        help="ARM/接触时当前位置与INIT允许的最大偏差(tick)")
    parser.add_argument("--input-timeout", type=float, default=0.5,
                        help="FSR或INSPIRE数据超过此时间未更新则FAULT(秒)")
    parser.add_argument("--filter-alpha", type=float, default=0.2,
                        help="INSPIRE力、FSR力和FSR新增力的EMA系数")
    parser.add_argument("--control-hz", type=float, default=20.0,
                        help="状态机和舵机控制频率(Hz)")
    parser.add_argument("--health-interval", type=float, default=0.5,
                        help="读取舵机错误、电流、电压和温度的周期(秒)")
    parser.add_argument("--temperature-limit", type=int, default=55,
                        help="Present Temperature故障阈值(摄氏度)")
    parser.add_argument("--voltage-min", type=float, default=4.0,
                        help="Present Input Voltage软件下限(V)")
    parser.add_argument("--voltage-max", type=float, default=8.5,
                        help="Present Input Voltage软件上限(V)")
    args = parser.parse_args()
    if not (4.903 < args.target_max <= 6.0 and 0 < args.comfort_scale <= 1.0):
        parser.error("首测target-max必须在4.903..6N，comfort-scale必须在0..1")
    if not (1 <= args.max_goal_current <= 300 and 0 <= args.max_travel <= 4096):
        parser.error("Goal Current必须在1..300，max-travel必须在0..4096 tick")
    if not (1 <= args.release_goal_current <= args.max_goal_current):
        parser.error("release-goal-current必须在1..max-goal-current")
    if not (1 <= args.release_min_current <= args.release_goal_current):
        parser.error("release-min-current必须在1..release-goal-current")
    if not (10 <= args.release_slowdown_range <= 500
            and 0.1 <= args.release_min_time <= 1.0):
        parser.error("释放减速区必须在10..500 tick，最小时间必须在0.1..1秒")
    if not (1.0 <= args.release_timeout <= 30.0):
        parser.error("release-timeout必须在1..30秒")
    if not (1 <= args.current_slew <= 300):
        parser.error("current-slew必须在1..300")
    if args.force_to_current_gain <= 0:
        parser.error("force-to-current-gain必须>0")
    if not (args.seek_max_travel == 0 or 100 <= args.seek_max_travel <= 20000):
        parser.error("seek-max-travel必须为0，或在100..20000 tick")
    if not (args.seek_timeout == 0.0 or 0.5 <= args.seek_timeout <= 30.0):
        parser.error("seek-timeout必须为0，或在0.5..30秒")
    if not (args.fsr_threshold <= args.fsr_rest_max < args.fsr_raw_max):
        parser.error("fsr-rest-max必须在fsr-threshold..fsr-raw-max之间")
    if not (0.01 <= args.fsr_init_max_span <= 1.0
            and 0.0 <= args.fsr_preload_deadband <= 0.5):
        parser.error("FSR INIT波动上限必须在0.01..1N，预载死区必须在0..0.5N")
    if not (1 <= args.lock_position_threshold <= 30):
        parser.error("lock-position-threshold必须在1..30")
    if not (40 <= args.direction_fault_threshold <= 500):
        parser.error("direction-fault-threshold必须在40..500 tick")
    if not (0.2 <= args.lock_detect_window <= 2.0
            and 0.2 <= args.lock_min_drive_time <= 2.0):
        parser.error("位置稳定检测时间必须在0.2..2秒")
    if args.actual_current_limit < args.max_goal_current:
        parser.error("actual-current-limit不能小于max-goal-current")
    if args.force_kp <= 0 or args.force_step_max <= 0 or args.force_deadzone < 0:
        parser.error("LOCKED PID参数必须为正（死区可为0）")
    if not (0.0 <= args.exo_zero_threshold < args.fsr_load_threshold <= 10.0):
        parser.error("FSR卸载阈值必须>=0且小于加载阈值；加载阈值最大10N")
    if not (0.1 <= args.fsr_release_hold_seconds <= 5.0
            and 0.1 <= args.release_hold_seconds <= 5.0):
        parser.error("FSR/INSPIRE释放保持时间必须在0.1..5秒")
    if not (0.2 <= args.return_settle_time <= 2.0):
        parser.error("return-settle-time必须在0.2..2秒")

    controller = Controller(args)

    def stop_handler(_signum, _frame):
        controller.running = False

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    try:
        controller.open()
        controller.start_inputs()
        threading.Thread(target=controller.loop, daemon=True).start()
        print("命令: STATUS | INIT | ARM | STOP | QUIT")
        print("默认STOP；首轮必须未穿戴，并准备物理断电。")
        while controller.running:
            try:
                command = input("> ").strip().upper()
            except EOFError:
                break
            if command == "ARM":
                controller.arm()
            elif command == "INIT":
                controller.record_init_pos()
            elif command == "STOP":
                controller.stop("人工STOP")
            elif command == "STATUS":
                (fsr_raw, fsr_t, inspire_raw, inspire_force,
                 inspire_t, valid) = controller.inputs.snapshot()
                now = time.monotonic()
                print(f"state={controller.state}, armed={controller.armed}, "
                      f"init_pos={controller.init_pos if controller.init_pos is not None else '未INIT'}, "
                      f"fault={controller.fault_reason or '-'}")
                rest = controller.fsr_rest_raw
                excess = (None if rest is None else max(
                    0.0, fsr_raw - rest - controller.args.fsr_preload_deadband))
                print(f"FSR raw={fsr_raw}, "
                      f"rest={rest if rest is not None else '未INIT'}, "
                      f"excess={excess if excess is not None else '未INIT'}, "
                      f"age={now-fsr_t if fsr_t else float('inf'):.3f}s")
                print(f"Inspire index top_touch max={inspire_raw}, "
                      f"fit={inspire_force:.3f}N, valid={valid}, "
                      f"age={now-inspire_t if inspire_t else float('inf'):.3f}s")
                if controller.args.enable_mhandpro:
                    with controller.haptic_lock:
                        print(f"mHandPro override connected={controller.haptic_connected}, "
                              f"state={controller.haptic_state}")
            elif command == "QUIT":
                break
            elif command:
                print("命令: STATUS | INIT | ARM | STOP | QUIT")
    except Exception as exc:
        print(f"启动/运行失败: {exc}", file=sys.stderr)
        return 1
    finally:
        controller.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
