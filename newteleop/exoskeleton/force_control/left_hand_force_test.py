#!/usr/bin/env python3
"""左手五指外骨骼力反馈联调。

复用 left_index_force_test.Controller 的每指状态机，在同一串口上
串行调度 ID 6~10，避免多进程同时占用 /dev/serial0。
"""

from __future__ import annotations

import argparse
from collections import deque
import json
import signal
import socket
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

from dynamixel_sdk import PortHandler

from left_index_force_test import (
    ADDR_CURRENT_LIMIT,
    ADDR_GOAL_CURRENT,
    ADDR_HARDWARE_ERROR,
    ADDR_OPERATING_MODE,
    ADDR_PRESENT_POSITION,
    ADDR_TORQUE_ENABLE,
    Controller,
    MODE_CURRENT_BASED_POSITION,
    signed32,
)
FSR_DIR = Path(__file__).resolve().parent.parent / "FSR"
sys.path.insert(0, str(FSR_DIR))
from exo_pressure_common import iter_pressure_samples  # noqa: E402


FINGER_NAMES = ["拇指", "食指", "中指", "无名指", "小指"]
SERVO_IDS = [6, 7, 8, 9, 10]  # 仅保留给旧外部导入；控制器使用args.servo_ids
HAND_PROFILES = {
    "left": {
        "label": "左手",
        "servo_ids": [6, 7, 8, 9, 10],
        "fsr_port": 9002,
        "force_port": 9202,
        "haptic_port": 9302,
    },
    "right": {
        "label": "右手",
        "servo_ids": [1, 2, 3, 4, 5],
        "fsr_port": 9001,
        "force_port": 9201,
        "haptic_port": 9301,
    },
}


def five_signs(text: str) -> list[int]:
    try:
        values = [int(item.strip()) for item in text.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("必须是5个逗号分隔的+1或-1") from exc
    if len(values) != 5 or any(value not in (-1, 1) for value in values):
        raise argparse.ArgumentTypeError("顺序[拇,食,中,无,小]，每项只能是+1或-1")
    return values


def finger_args(args: argparse.Namespace, index: int) -> SimpleNamespace:
    return SimpleNamespace(
        enable_write=args.enable_write, enable_mhandpro=False,
        device=args.device, baudrate=args.baudrate,
        servo_id=args.servo_ids[index], finger_index=index,
        finger_name=FINGER_NAMES[index], init_pos=None,
        fsr_host=args.fsr_host, fsr_port=args.fsr_port,
        force_host=args.force_host, force_port=args.force_port,
        haptic_host=args.haptic_host, haptic_port=args.haptic_port,
        fsr_threshold=4.903, fsr_raw_max=60.0,
        fsr_rest_max=args.fsr_rest_max,
        fsr_init_max_span=0.20, fsr_preload_deadband=0.05,
        comfort_scale=args.comfort_scale, target_max=args.target_max,
        contact_on=args.contact_on, contact_off=args.contact_off,
        force_to_current_gain=args.force_to_current_gain,
        hand_total_current_limit=args.hand_total_current_limit,
        current_slew=args.current_slew,
        max_goal_current=args.max_goal_current,
        release_goal_current=args.release_goal_current,
        # RELEASE上限提高后提前150 tick减流，避免越过INIT后左右振荡。
        release_min_current=10, release_slowdown_range=150,
        release_min_time=0.3, release_timeout=args.release_timeout,
        actual_current_limit=args.actual_current_limit,
        max_travel=0, seek_max_travel=0, seek_timeout=0.0,
        drive_current_sign=args.drive_current_signs[index],
        direction_fault_threshold=args.direction_fault_threshold,
        lock_position_threshold=8, lock_detect_window=0.25,
        lock_min_drive_time=0.2, force_kp=args.force_kp,
        force_step_max=args.force_step_max, force_deadzone=args.force_deadzone,
        fsr_load_threshold=args.fsr_load_threshold,
        exo_zero_threshold=args.fsr_release_threshold,
        fsr_release_hold_seconds=args.fsr_release_hold_seconds,
        release_hold_seconds=args.release_hold_seconds,
        position_tolerance=10, release_position_threshold=15,
        return_settle_time=args.return_settle_time,
        init_start_tolerance=40, input_timeout=0.5,
        filter_alpha=0.2, control_hz=args.control_hz,
        # 五指/双手程序由手级控制器汇总打印，禁止单指周期日志。
        periodic_status=False,
        health_interval=0.5, temperature_limit=55,
        # XL330-M288官方范围3.7~6.0V（推荐5.0V）。软件用4.0V保留余量。
        voltage_min=4.0, voltage_max=8.5,
    )


class LeftHandController:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.running = True
        self.port = PortHandler(args.device)
        # 五个Controller共享同一个PortHandler。主线程执行INIT/ARM/STOP时，
        # 20 Hz控制线程不得同时访问Dynamixel SDK，否则会报Port is in use。
        self.dxl_lock = threading.RLock()
        self.fingers = [Controller(finger_args(args, fi)) for fi in range(5)]
        for finger in self.fingers:
            finger.port = self.port
            finger.running = True
        self.haptic_lock = threading.Lock()
        self.haptic_connected = not args.enable_mhandpro
        self.haptic_steps = [0.0] * 5
        self.last_summary = 0.0
        self.force_entry_current_scale = 1.0
        self.metrics_lock = threading.Lock()
        self.fsr_connected = False
        self.fsr_last_error = ""
        self.fsr_sample_times = deque(maxlen=200)
        self.inspire_connected = False
        self.inspire_last_error = ""
        self.inspire_sample_times = deque(maxlen=200)
        self._fsr_input_started = False
        self._g1_inputs_started = False
        for fi, finger in enumerate(self.fingers):
            finger.set_haptic = self._make_haptic_sink(fi)  # type: ignore[method-assign]

    def _make_haptic_sink(self, fi: int):
        def sink(_state: str, index_step: float = 0.0) -> None:
            with self.haptic_lock:
                self.haptic_steps[fi] += index_step
        return sink

    def open(self, open_port: bool = True) -> None:
        with self.dxl_lock:
            if open_port:
                if not self.port.openPort():
                    raise RuntimeError(f"无法打开{self.args.device}")
                if not self.port.setBaudRate(self.args.baudrate):
                    raise RuntimeError(f"无法设置波特率{self.args.baudrate}")
            for finger in self.fingers:
                model, _result, _error = finger._retry_txrx(
                    lambda f=finger: f.packet.ping(self.port, f.servo_id),
                    "Ping")
                finger.hardware_ready = True
                mode = finger.read1(ADDR_OPERATING_MODE)
                torque = finger.read1(ADDR_TORQUE_ENABLE)
                hw_error = finger.read1(ADDR_HARDWARE_ERROR)
                position = signed32(finger.read4(ADDR_PRESENT_POSITION))
                finger.last_position = position
                finger.operating_mode = mode
                finger.torque_enabled = bool(torque)
                finger.hardware_error = hw_error
                finger.present_position_raw = position
                finger.present_position = position
                finger.health_time = time.monotonic()
                print(f"{finger.finger_name} ID{finger.servo_id}: model={model}, mode={mode}, "
                      f"torque={torque}, hw_error=0x{hw_error:02X}, position={position}")
                if hw_error:
                    raise RuntimeError(
                        f"ID{finger.servo_id}硬件错误0x{hw_error:02X}，"
                        "请断电检查后重试")
                if torque:
                    if not self.args.enable_write:
                        raise RuntimeError(
                            f"ID{finger.servo_id}扭矩未关闭；请用--enable-write"
                            "启动以执行安全恢复，或断电重启舵机")
                    # 上次异常退出可能来不及关扭矩。先清目标电流，
                    # 再关扭矩并回读，不要求用户为此反复整机断电。
                    finger.write2(ADDR_GOAL_CURRENT, 0)
                    finger.write1(ADDR_TORQUE_ENABLE, 0)
                    torque = finger.read1(ADDR_TORQUE_ENABLE)
                    if torque:
                        raise RuntimeError(f"ID{finger.servo_id}安全关扭矩失败")
                    print(f"  [安全恢复] ID{finger.servo_id}遗留扭矩已关闭。")
                if mode != MODE_CURRENT_BASED_POSITION:
                    if not self.args.enable_write:
                        print(f"  [只读警告] ID{finger.servo_id}遗留在mode={mode}；"
                              "本次不写寄存器，可继续STATUS检查。")
                    else:
                        # 扭矩已确认关闭且无硬件错误，此时切模式是安全的。
                        finger.write1(ADDR_OPERATING_MODE, MODE_CURRENT_BASED_POSITION)
                        finger.write2(ADDR_CURRENT_LIMIT, self.args.max_goal_current)
                        restored = finger.read1(ADDR_OPERATING_MODE)
                        if restored != MODE_CURRENT_BASED_POSITION:
                            raise RuntimeError(
                                f"ID{finger.servo_id}恢复mode5失败，回读={restored}")
                        print(f"  [安全恢复] ID{finger.servo_id}: mode {mode} -> 5，扭矩仍关闭。")

    def start_inputs(self) -> None:
        """命令行入口保持兼容：一次启动FSR和G1输入。"""
        self.start_fsr_input()
        self.start_g1_inputs()

    def start_fsr_input(self) -> bool:
        """启动本手FSR TCP接收线程；重复调用无副作用。"""
        if self._fsr_input_started:
            return False
        self._fsr_input_started = True
        threading.Thread(
            target=self._fsr_loop,
            name=f"{self.args.hand}-fsr-input",
            daemon=True,
        ).start()
        return True

    def start_g1_inputs(self) -> bool:
        """启动INSPIRE触觉接收及可选的力控覆盖线程。"""
        if self._g1_inputs_started:
            return False
        self._g1_inputs_started = True
        threading.Thread(
            target=self._inspire_loop,
            name=f"{self.args.hand}-inspire-input",
            daemon=True,
        ).start()
        if self.args.enable_mhandpro:
            threading.Thread(
                target=self._haptic_loop,
                name=f"{self.args.hand}-haptic-output",
                daemon=True,
            ).start()
        return True

    def _fsr_loop(self) -> None:
        while self.running:
            try:
                for sample in iter_pressure_samples(
                        self.args.fsr_host, self.args.fsr_port, data_timeout=1.0):
                    if not self.running:
                        return
                    if len(sample.values) != 5:
                        raise RuntimeError(f"FSR通道数={len(sample.values)}，应为5")
                    with self.metrics_lock:
                        self.fsr_connected = True
                        self.fsr_last_error = ""
                        self.fsr_sample_times.append(sample.mono_time)
                    for fi, finger in enumerate(self.fingers):
                        finger.inputs.set_fsr(sample.values[fi])
            except Exception as exc:
                with self.metrics_lock:
                    self.fsr_connected = False
                    self.fsr_last_error = str(exc)
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
                with self.metrics_lock:
                    self.inspire_connected = True
                    self.inspire_last_error = ""
                print(f"[Inspire] 已连接五指力流 {self.args.force_host}:{self.args.force_port}")
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
                        with self.metrics_lock:
                            self.inspire_sample_times.append(time.monotonic())
                        raws = packet.get("top_touch_raw_max", [])
                        forces = packet.get("top_touch_force_n", [])
                        valid = bool(packet.get("touch_valid")) and len(raws) == len(forces) == 5
                        for fi, finger in enumerate(self.fingers):
                            finger.inputs.set_inspire(
                                int(raws[fi]) if valid else 0,
                                float(forces[fi]) if valid else 0.0,
                                valid,
                            )
            except Exception as exc:
                with self.metrics_lock:
                    self.inspire_connected = False
                    self.inspire_last_error = str(exc)
                for finger in self.fingers:
                    finger.inputs.set_inspire(0, 0.0, False)
                if self.running:
                    print(f"[Inspire] {exc}；1秒后重连")
                    time.sleep(1.0)
            finally:
                with self.metrics_lock:
                    self.inspire_connected = False
                if sock:
                    sock.close()

    def _haptic_loop(self) -> None:
        period = 1.0 / self.args.control_hz
        while self.running:
            sock = None
            try:
                sock = socket.create_connection(
                    (self.args.haptic_host, self.args.haptic_port), timeout=3.0)
                with self.haptic_lock:
                    self.haptic_connected = True
                print(f"[mHandPro] 已连接五指力反馈覆盖 "
                      f"{self.args.haptic_host}:{self.args.haptic_port}")
                while self.running:
                    started = time.monotonic()
                    with self.haptic_lock:
                        steps = list(self.haptic_steps)
                        self.haptic_steps = [0.0] * 5
                    packet = {"type": "haptic_override", "state": "GLOVE",
                              "finger_states": [finger.state for finger in self.fingers],
                              "finger_steps": steps}
                    sock.sendall((json.dumps(packet, separators=(",", ":")) + "\n").encode())
                    time.sleep(max(0.0, period - (time.monotonic() - started)))
            except Exception as exc:
                with self.haptic_lock:
                    self.haptic_connected = False
                if self.running:
                    print(f"[mHandPro] {exc}；1秒后重连")
                    time.sleep(1.0)
            finally:
                with self.haptic_lock:
                    self.haptic_connected = not self.args.enable_mhandpro
                if sock:
                    sock.close()

    @staticmethod
    def _sample_rate(sample_times: list[float], now: float) -> float:
        recent = [stamp for stamp in sample_times if now - stamp <= 2.0]
        if len(recent) < 2:
            return 0.0
        span = recent[-1] - recent[0]
        return (len(recent) - 1) / span if span > 0 else 0.0

    def telemetry_snapshot(self, now: float | None = None) -> dict:
        """返回手级链路和五指遥测；只读内存缓存。"""
        now = time.monotonic() if now is None else now
        fingers = [finger.telemetry_snapshot(now) for finger in self.fingers]
        fsr_ages = [item["fsr"]["age_ms"] for item in fingers
                    if item["fsr"]["age_ms"] is not None]
        inspire_ages = [item["inspire"]["age_ms"] for item in fingers
                        if item["inspire"]["age_ms"] is not None]
        with self.haptic_lock:
            haptic_connected = self.haptic_connected
        with self.metrics_lock:
            fsr_connected = self.fsr_connected
            fsr_last_error = self.fsr_last_error
            fsr_sample_times = list(self.fsr_sample_times)
            inspire_connected = self.inspire_connected
            inspire_last_error = self.inspire_last_error
            inspire_sample_times = list(self.inspire_sample_times)
        fsr_age = max(fsr_ages) if fsr_ages else None
        inspire_age = max(inspire_ages) if inspire_ages else None
        # 对于上位机，“BLE正常”表示 broker TCP 以及 BLE
        # 通知的端到端数据均是新鲜的，比只看is_connected更有用。
        fsr_fresh = (fsr_connected and fsr_age is not None
                     and fsr_age <= round(self.fingers[0].args.input_timeout * 1000))
        inspire_fresh = (inspire_connected and inspire_age is not None
                         and inspire_age <= round(
                             self.fingers[0].args.input_timeout * 1000))
        return {
            "label": HAND_PROFILES[self.args.hand]["label"],
            "armed": all(finger.armed for finger in self.fingers),
            "faulted": any(finger.state == "FAULT" for finger in self.fingers),
            "ble_fsr": {
                "connected": fsr_fresh,
                "tcp_connected": fsr_connected,
                "host": self.args.fsr_host,
                "port": self.args.fsr_port,
                "data_age_ms": fsr_age,
                "rate_hz": round(self._sample_rate(fsr_sample_times, now), 1),
                "last_error": fsr_last_error or None,
            },
            "inspire_feedback": {
                "connected": inspire_fresh,
                "tcp_connected": inspire_connected,
                "host": self.args.force_host,
                "port": self.args.force_port,
                "data_age_ms": inspire_age,
                "rate_hz": round(
                    self._sample_rate(inspire_sample_times, now), 1),
                "last_error": inspire_last_error or None,
            },
            "haptic_override": {
                "enabled": self.args.enable_mhandpro,
                "connected": haptic_connected,
                "host": self.args.haptic_host,
                "port": self.args.haptic_port,
            },
            "fingers": fingers,
        }

    def data_ready(self) -> tuple[bool, str]:
        if self.args.enable_mhandpro:
            with self.haptic_lock:
                if not self.haptic_connected:
                    return False, f"mHandPro五指覆盖{self.args.haptic_port}未连接"
        for finger in self.fingers:
            ready, reason = finger.data_ready()
            if not ready:
                return False, f"{finger.finger_name}: {reason}"
        return True, ""

    def init_all(self) -> None:
        ready, reason = self.data_ready()
        if not ready:
            print(f"不能INIT: {reason}")
            return
        with self.dxl_lock:
            for finger in self.fingers:
                finger.record_init_pos()

    def arm_all(self) -> None:
        ready, reason = self.data_ready()
        if not ready:
            print(f"不能ARM: {reason}")
            return
        if any(finger.init_pos is None or finger.fsr_rest_raw is None
               for finger in self.fingers):
            print("不能ARM: 五指必须全部INIT成功")
            return
        with self.dxl_lock:
            for finger in self.fingers:
                finger.arm()
                if not finger.armed:
                    self.stop_all(f"{finger.finger_name} ARM失败，取消已ARM的其他手指")
                    return

    def stop_all(self, reason: str) -> None:
        with self.dxl_lock:
            for finger in self.fingers:
                finger.stop(reason)

    def status(self) -> None:
        for finger in self.fingers:
            fsr, fsr_t, raw, force, force_t, valid = finger.inputs.snapshot()
            now = time.monotonic()
            print(f"{finger.finger_name} ID{finger.servo_id}: state={finger.state}, "
                  f"armed={finger.armed}, init_pos={finger.init_pos}, "
                  f"fsr_rest={finger.fsr_rest_raw}, FSR={fsr:.3f} "
                  f"age={now-fsr_t:.3f}s, Inspire={raw}/{force:.3f}N "
                  f"valid={valid} age={now-force_t:.3f}s")

    def print_summary(self, now: float) -> None:
        """参考 ftp/both_Force_handcontrol.py：每只手每周期只输出一组五指摘要。"""
        modes = " ".join(
            f"{finger.finger_name}:{finger.state}" for finger in self.fingers)
        touch_parts = []
        fsr_parts = []
        current_parts = []
        for finger in self.fingers:
            fsr, _fsr_t, raw, force, _force_t, valid = finger.inputs.snapshot()
            touch_parts.append(
                f"{finger.finger_name}={raw}/{force:.3f}N"
                + ("" if valid else "(!)"))
            fsr_parts.append(f"{fsr:.3f}")
            current_parts.append(f"{finger.commanded_current:+d}")
        label = HAND_PROFILES[self.args.hand]["label"]
        # 一次print完成整组输出，尽量避免左右手线程在组内穿插。
        print(
            f"[{label}] {modes}\n"
            f"  Inspire: {' '.join(touch_parts)}\n"
            f"  FSR: [{' '.join(fsr_parts)}]N  "
            f"goal_current: [{' '.join(current_parts)}]  "
            f"entry_scale={self.force_entry_current_scale:.3f}",
            flush=True,
        )
        self.last_summary = now

    def update_force_entry_current_scale(self) -> None:
        """复用旧版每手FORCE_ENTRY总电流800 raw的按比例限幅。

        用本周期EMA的预测值计算五指请求，然后给每指应用相同比例；
        RELEASE归位电流与旧版一样不计入这个收绳额度。
        """
        total_requested = 0.0
        for finger in self.fingers:
            if not finger.armed:
                continue
            _fsr, _fsr_t, _raw, force, _force_t, valid = finger.inputs.snapshot()
            target = min(
                finger.args.target_max,
                max(0.0, force) * finger.args.comfort_scale,
            ) if valid else 0.0
            active = (finger.state == "FORCE_ENTRY"
                      or (finger.state == "FREE"
                          and target >= finger.args.contact_on))
            if not active:
                continue
            predicted = finger.filtered_target + finger.args.filter_alpha * (
                target - finger.filtered_target)
            total_requested += min(
                finger.args.max_goal_current,
                predicted * finger.args.force_to_current_gain,
            )
        limit = float(self.args.hand_total_current_limit)
        scale = min(1.0, limit / total_requested) if total_requested > 0 else 1.0
        self.force_entry_current_scale = scale
        for finger in self.fingers:
            finger.force_entry_current_scale = scale

    def loop(self) -> None:
        period = 1.0 / self.args.control_hz
        while self.running:
            started = time.monotonic()
            try:
                with self.dxl_lock:
                    self.update_force_entry_current_scale()
                    for finger in self.fingers:
                        if finger.armed:
                            finger.tick(started)
                    faulted = next((finger for finger in self.fingers
                                    if finger.state == "FAULT"), None)
                    if faulted:
                        self.stop_all(f"{faulted.finger_name}故障，五指联锁停止")
                    # STOP/INIT阶段不自动刷屏，保留清晰的命令输入行；
                    # ARM后每秒1次汇总，控制循环仍保持原有20Hz。
                    if (any(finger.armed for finger in self.fingers)
                            and started - self.last_summary >= 1.0):
                        self.print_summary(started)
            except Exception as exc:
                print(f"[FAULT] {exc}")
                self.stop_all("五指联锁停止")
            time.sleep(max(0.0, period - (time.monotonic() - started)))

    def close(self, close_port: bool = True) -> None:
        self.running = False
        self.stop_all("程序退出")
        with self.dxl_lock:
            if close_port:
                self.port.closePort()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="左右手通用五指外骨骼力反馈联调（旧左手入口保持兼容）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--hand", choices=tuple(HAND_PROFILES), default="left",
                        help="选择手别并自动设置舵机ID、FSR和INSPIRE端口")
    parser.add_argument("--enable-write", action="store_true",
                        help="允许写Dynamixel和ARM；默认只读")
    parser.add_argument("--enable-mhandpro", action="store_true",
                        help="向G1发送LOCKED状态和INSPIRE力控位置增量")
    parser.add_argument("--device", default="/dev/serial0",
                        help="Dynamixel TTL串口设备")
    parser.add_argument("--baudrate", type=int, default=1_000_000,
                        help="Dynamixel总线波特率")
    parser.add_argument("--fsr-host", default="127.0.0.1",
                        help="FSR BLE Broker所在主机")
    parser.add_argument("--fsr-port", type=int,
                        help="覆盖手别默认值：左9002、右9001")
    parser.add_argument("--force-host", default="127.0.0.1",
                        help="INSPIRE top_touch力流所在主机")
    parser.add_argument("--force-port", type=int,
                        help="覆盖手别默认值：左9202、右9201")
    parser.add_argument("--haptic-host", default="127.0.0.1",
                        help="G1力控覆盖服务所在主机")
    parser.add_argument("--haptic-port", type=int,
                        help="覆盖手别默认值：左9302、右9301")
    parser.add_argument("--drive-current-signs", type=five_signs,
                        help="必须显式给出[拇,食,中,无,小]收绳方向，如1,1,1,1,1")
    parser.add_argument("--max-goal-current", type=int, default=30,
                        help="FORCE_ENTRY收绳Goal Current上限(raw)")
    parser.add_argument("--release-goal-current", type=int, default=30,
                        help="RELEASE归位Goal Current上限(raw)")
    parser.add_argument("--actual-current-limit", type=int, default=45,
                        help="Present Current绝对值故障阈值(raw)")
    parser.add_argument("--current-slew", type=int, default=30,
                        help="每控制周期最大收绳电流增量(raw)")
    parser.add_argument("--force-to-current-gain", type=float, default=60.0,
                        help="Inspire触觉N到FORCE_ENTRY目标电流的比例，默认沿用旧程序60")
    parser.add_argument("--hand-total-current-limit", type=int, default=800,
                        help="每只手五指FORCE_ENTRY收绳总电流上限(raw)")
    parser.add_argument("--direction-fault-threshold", type=int, default=100,
                        help="0.2秒后累计反向运动故障阈值")
    parser.add_argument("--release-timeout", type=float, default=12.0,
                        help="RELEASE归位超时阈值(秒)")
    parser.add_argument("--return-settle-time", type=float, default=0.5,
                        help="归位后位置与INSPIRE释放稳定时间(秒)")
    parser.add_argument("--release-hold-seconds", type=float, default=0.15,
                        help="LOCKED中INSPIRE触觉释放消抖时间(秒)")
    parser.add_argument("--fsr-load-threshold", type=float, default=0.30,
                        help="FSR相对INIT新增力达到此值后确认本轮加载(N)")
    parser.add_argument("--fsr-release-threshold", type=float, default=0.15,
                        help="已加载FSR降到此值以下时开始卸载计时(N)")
    parser.add_argument("--fsr-release-hold-seconds", type=float, default=0.30,
                        help="FSR加载后卸载条件连续成立时间(秒)")
    parser.add_argument("--fsr-rest-max", type=float, default=8.0,
                        help="INIT允许的最大FSR绑带预载(N)")
    parser.add_argument("--comfort-scale", type=float, default=1.0,
                        help="INSPIRE力进入状态机前的整体缩放系数")
    parser.add_argument("--target-max", type=float, default=6.0,
                        help="INSPIRE目标力软件上限(N)")
    parser.add_argument("--contact-on", type=float, default=0.50,
                        help="Inspire接触进入阈值；默认恢复旧程序0.50N")
    parser.add_argument("--contact-off", type=float, default=0.30,
                        help="Inspire接触释放阈值，低于contact-on形成滞回")
    parser.add_argument("--force-kp", type=float, default=35.0,
                        help="LOCKED力差PID比例增益(tick/N)")
    parser.add_argument("--force-step-max", type=float, default=3.0,
                        help="LOCKED PID每周期INSPIRE最大位置修正(tick)")
    parser.add_argument("--force-deadzone", type=float, default=0.10,
                        help="LOCKED力误差死区(N)")
    parser.add_argument("--control-hz", type=float, default=20.0,
                        help="状态机与舵机控制频率(Hz)")
    args = parser.parse_args()
    profile = HAND_PROFILES[args.hand]
    args.servo_ids = list(profile["servo_ids"])
    args.fsr_port = args.fsr_port or profile["fsr_port"]
    args.force_port = args.force_port or profile["force_port"]
    args.haptic_port = args.haptic_port or profile["haptic_port"]
    if args.enable_write and args.drive_current_signs is None:
        parser.error("五指写入模式必须先逐指验证并显式传--drive-current-signs")
    if args.drive_current_signs is None:
        args.drive_current_signs = [1] * 5
    if not (1 <= args.max_goal_current <= 300):
        parser.error("max-goal-current必须在1..300")
    if not (1 <= args.release_goal_current <= args.max_goal_current):
        parser.error("release-goal-current必须在1..max-goal-current")
    if args.actual_current_limit < args.max_goal_current:
        parser.error("actual-current-limit不能小于max-goal-current")
    if not (1 <= args.current_slew <= 60):
        parser.error("五指首试current-slew必须在1..60")
    if not (40 <= args.direction_fault_threshold <= 500):
        parser.error("direction-fault-threshold必须在40..500 tick")
    if args.force_to_current_gain <= 0:
        parser.error("force-to-current-gain必须>0")
    if not (args.max_goal_current <= args.hand_total_current_limit <= 1500):
        parser.error("hand-total-current-limit必须在max-goal-current..1500")
    if args.force_kp <= 0 or not 0.1 <= args.force_step_max <= 20:
        parser.error("force-kp必须>0，force-step-max必须在0.1..20 tick")
    if args.force_deadzone < 0:
        parser.error("force-deadzone必须>=0")
    if not (0.1 <= args.release_hold_seconds <= 5.0):
        parser.error("release-hold-seconds必须在0.1..5秒")
    if not (0.0 <= args.fsr_release_threshold
            < args.fsr_load_threshold <= 10.0):
        parser.error("FSR卸载阈值必须>=0且小于加载阈值；加载阈值最大10N")
    if not (0.1 <= args.fsr_release_hold_seconds <= 5.0):
        parser.error("fsr-release-hold-seconds必须在0.1..5秒")

    print(f"手别: {profile['label']}  舵机ID={args.servo_ids}  "
          f"FSR={args.fsr_host}:{args.fsr_port}  "
          f"Inspire力流={args.force_host}:{args.force_port}  "
          f"覆盖={args.haptic_host}:{args.haptic_port}")
    controller = LeftHandController(args)
    signal.signal(signal.SIGINT, lambda *_: setattr(controller, "running", False))
    signal.signal(signal.SIGTERM, lambda *_: setattr(controller, "running", False))
    try:
        controller.open()
        controller.start_inputs()
        threading.Thread(target=controller.loop, daemon=True).start()
        print("命令: STATUS | INIT | ARM | STOP | QUIT")
        print("默认STOP；五指首轮必须未穿戴，并准备物理断电。")
        while controller.running:
            try:
                command = input("> ").strip().upper()
            except EOFError:
                break
            if command == "STATUS":
                controller.status()
            elif command == "INIT":
                controller.init_all()
            elif command == "ARM":
                controller.arm_all()
            elif command == "STOP":
                controller.stop_all("人工STOP")
            elif command == "QUIT":
                break
            elif command:
                print("未知命令")
    finally:
        controller.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
