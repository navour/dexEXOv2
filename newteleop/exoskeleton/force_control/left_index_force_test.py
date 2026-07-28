#!/usr/bin/env python3
"""左食指最小力反馈连接测试。

复用旧 Force_handcontrol.py 的 BLE/TCP、Inspire力换算、Dynamixel Protocol 2.0、
电流限制和释放思路，但运动链仍由 mHandPro 独立控制。本程序仅拥有
/dev/serial0，首版只允许 ID 7。FSR使用4.903N阈值开关，不减去阈值。
"""

from __future__ import annotations

import argparse
import json
import signal
import socket
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
POSITION_TICKS_PER_REV = 4096

INDEX_SERVO_ID = 7
INDEX_FSR_INDEX = 1
INDEX_INSPIRE_TOUCH_INDEX = 1  # top_touch发布顺序 [拇,食,中,无,小]


def signed16(value: int) -> int:
    return value - 65536 if value > 32767 else value


def signed32(value: int) -> int:
    return value - 4294967296 if value > 2147483647 else value


def position_near_reference(position: int, reference: int) -> int:
    """将Dynamixel当前位置映射到最接近reference的4096 tick等效圈。"""
    turns = round((reference - position) / POSITION_TICKS_PER_REV)
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
        self.running = True
        self.armed = False
        self.fault_reason = ""
        self.state = "STOP"
        self.inputs = LatestInputs(threading.Lock())
        self.port = PortHandler(args.device)
        self.packet = PacketHandler(2.0)
        self.neutral = args.neutral
        self.last_status = 0.0
        self.last_health = 0.0
        self.filtered_target = 0.0
        self.filtered_measured = 0.0
        self.last_goal_current = 0
        self.release_started = 0.0

    def check(self, result: int, error: int, action: str) -> None:
        if result != COMM_SUCCESS:
            raise RuntimeError(f"{action}: {self.packet.getTxRxResult(result)}")
        if error:
            raise RuntimeError(f"{action}: {self.packet.getRxPacketError(error)}")

    def read1(self, address: int) -> int:
        value, result, error = self.packet.read1ByteTxRx(self.port, INDEX_SERVO_ID, address)
        self.check(result, error, f"读取地址{address}")
        return value

    def read2(self, address: int) -> int:
        value, result, error = self.packet.read2ByteTxRx(self.port, INDEX_SERVO_ID, address)
        self.check(result, error, f"读取地址{address}")
        return value

    def read4(self, address: int) -> int:
        value, result, error = self.packet.read4ByteTxRx(self.port, INDEX_SERVO_ID, address)
        self.check(result, error, f"读取地址{address}")
        return value

    def write1(self, address: int, value: int) -> None:
        result, error = self.packet.write1ByteTxRx(self.port, INDEX_SERVO_ID, address, value)
        self.check(result, error, f"写地址{address}")

    def write2(self, address: int, value: int) -> None:
        result, error = self.packet.write2ByteTxRx(
            self.port, INDEX_SERVO_ID, address, value & 0xFFFF)
        self.check(result, error, f"写地址{address}")

    def write4(self, address: int, value: int) -> None:
        result, error = self.packet.write4ByteTxRx(
            self.port, INDEX_SERVO_ID, address, value & 0xFFFFFFFF)
        self.check(result, error, f"写地址{address}")

    def start_inputs(self) -> None:
        threading.Thread(target=self._fsr_loop, daemon=True).start()
        threading.Thread(target=self._inspire_loop, daemon=True).start()

    def _fsr_loop(self) -> None:
        while self.running:
            try:
                for sample in iter_pressure_samples(
                    self.args.fsr_host, self.args.fsr_port, data_timeout=1.0):
                    if not self.running:
                        return
                    self.inputs.set_fsr(sample.values[INDEX_FSR_INDEX])
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
                        raw = int(raw_values[INDEX_INSPIRE_TOUCH_INDEX]) if valid else 0
                        force = float(force_values[INDEX_INSPIRE_TOUCH_INDEX]) if valid else 0.0
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
        model, result, error = self.packet.ping(self.port, INDEX_SERVO_ID)
        self.check(result, error, "Ping ID7")
        mode = self.read1(ADDR_OPERATING_MODE)
        torque = self.read1(ADDR_TORQUE_ENABLE)
        hw_error = self.read1(ADDR_HARDWARE_ERROR)
        position_raw = signed32(self.read4(ADDR_PRESENT_POSITION))
        position = position_near_reference(position_raw, self.neutral)
        print(f"ID7 model={model}, mode={mode}, torque={torque}, "
              f"hw_error=0x{hw_error:02X}, position_raw={position_raw}, "
              f"position_norm={position}, neutral={self.neutral}")
        if torque:
            raise RuntimeError("启动时扭矩未关闭，拒绝接管")
        if mode != MODE_CURRENT_BASED_POSITION:
            raise RuntimeError(f"ID7工作模式为{mode}，必须先安全配置为模式5")
        if hw_error:
            raise RuntimeError(f"ID7存在硬件错误0x{hw_error:02X}")
        if abs(position - self.neutral) > self.args.neutral_start_tolerance:
            raise RuntimeError(
                f"当前位置{position}偏离neutral={self.neutral}超过"
                f"{self.args.neutral_start_tolerance} tick")

    def data_ready(self) -> tuple[bool, str]:
        fsr_raw, fsr_t, _, _, inspire_t, inspire_valid = self.inputs.snapshot()
        now = time.monotonic()
        if not fsr_t or now - fsr_t > self.args.input_timeout:
            return False, "FSR数据未就绪或超时"
        if not inspire_t or now - inspire_t > self.args.input_timeout or not inspire_valid:
            return False, "Inspire力数据未就绪、无效或超时"
        if fsr_raw < 0 or fsr_raw > self.args.fsr_raw_max:
            return False, f"FSR原始值异常: {fsr_raw}"
        return True, ""

    def arm(self) -> None:
        if not self.args.enable_write:
            print("[只读模式] 未传--enable-write，不能ARM")
            return
        ready, reason = self.data_ready()
        if not ready:
            print(f"不能ARM: {reason}")
            return
        position_raw = signed32(self.read4(ADDR_PRESENT_POSITION))
        position = position_near_reference(position_raw, self.neutral)
        if abs(position - self.neutral) > self.args.neutral_start_tolerance:
            print(f"不能ARM: position_raw={position_raw}, position_norm={position} "
                  f"偏离neutral={self.neutral}")
            return
        self.armed = True
        self.fault_reason = ""
        self.state = "FREE"
        self.filtered_target = 0.0
        self.filtered_measured = 0.0
        self.last_goal_current = 0
        self.release_started = 0.0
        print("已ARM：ID7保持关扭矩待机，Inspire食指受力后才使能力反馈。")

    def engage_at_current_position(self, position: int) -> bool:
        if abs(position - self.neutral) > self.args.neutral_start_tolerance:
            self.stop(
                f"接触时position={position}偏离neutral={self.neutral}过大",
                fault=True,
            )
            return False
        self.write2(ADDR_GOAL_CURRENT, 0)
        self.last_goal_current = 0
        self.write4(ADDR_GOAL_POSITION, position)
        self.write1(ADDR_TORQUE_ENABLE, 1)
        self.state = "HOLD"
        return True

    def stop(self, reason: str, *, fault: bool = False) -> None:
        was_active = self.armed or self.state not in ("STOP", "FAULT")
        self.armed = False
        self.last_goal_current = 0
        try:
            if self.args.enable_write:
                self.write2(ADDR_GOAL_CURRENT, 0)
                self.write1(ADDR_TORQUE_ENABLE, 0)
        except Exception as exc:
            print(f"[严重] 停止清理失败: {exc}", file=sys.stderr)
        self.state = "FAULT" if fault else "STOP"
        self.fault_reason = reason if fault else ""
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
        # STM32固件的查表最低输出约4.903N：阈值及以下视为0，
        # 超过阈值后保留绝对力原值，不减去4.903N。
        measured = 0.0 if fsr_raw <= self.args.fsr_threshold else fsr_raw
        target_unscaled = max(0.0, inspire_force)
        target_scaled = target_unscaled * self.args.comfort_scale
        target = (0.0 if target_scaled < self.args.contact_on
                  else min(self.args.target_max, target_scaled))
        alpha = self.args.filter_alpha
        self.filtered_target += alpha * (target - self.filtered_target)
        self.filtered_measured += alpha * (measured - self.filtered_measured)

        position_raw = signed32(self.read4(ADDR_PRESENT_POSITION))
        position = position_near_reference(position_raw, self.neutral)
        maximum = self.neutral + self.args.reel_in_sign * self.args.max_travel
        low, high = sorted((self.neutral, maximum))
        if self.state in ("HOLD", "RELEASE") and (
            position < low - self.args.position_tolerance
            or position > high + self.args.position_tolerance
        ):
            raise RuntimeError(f"位置越界: {position}, 允许{low}..{high}")

        if self.filtered_target >= self.args.contact_on:
            if self.state == "FREE" and not self.engage_at_current_position(position):
                return
            self.state = "HOLD"
            self.release_started = 0.0
            strength = min(1.0, self.filtered_target / self.args.target_max)
            goal = round(self.neutral + self.args.reel_in_sign * self.args.max_travel * strength)
            error = self.filtered_target - self.filtered_measured
            requested_current = round(max(
                0.0, min(self.args.max_goal_current, self.args.kp * error)))
            current = min(requested_current, self.last_goal_current + self.args.current_slew)
            self.last_goal_current = current
            self.write2(ADDR_GOAL_CURRENT, current)
            self.write4(ADDR_GOAL_POSITION, goal)
        elif self.filtered_target <= self.args.contact_off:
            if self.state == "FREE":
                return
            if self.state != "RELEASE":
                self.release_started = now
            self.state = "RELEASE"
            self.last_goal_current = 0
            self.write2(ADDR_GOAL_CURRENT, self.args.release_goal_current)
            self.write4(ADDR_GOAL_POSITION, self.neutral)
            if abs(position - self.neutral) <= self.args.neutral_tolerance:
                self.write2(ADDR_GOAL_CURRENT, 0)
                self.write1(ADDR_TORQUE_ENABLE, 0)
                self.state = "FREE"
                self.release_started = 0.0
                print("[FREE] 已回neutral并关扭矩，等待下一次Inspire接触。")
            elif now - self.release_started >= self.args.release_timeout:
                self.stop(
                    f"释放超时{self.args.release_timeout:.1f}s: "
                    f"position={position}, neutral={self.neutral}",
                    fault=True,
                )
                return

        if now - self.last_health >= self.args.health_interval:
            self.last_health = now
            hw_error = self.read1(ADDR_HARDWARE_ERROR)
            current_actual = signed16(self.read2(ADDR_PRESENT_CURRENT))
            voltage = self.read2(ADDR_PRESENT_INPUT_VOLTAGE) / 10.0
            temperature = self.read1(ADDR_PRESENT_TEMPERATURE)
            if hw_error:
                raise RuntimeError(f"硬件错误0x{hw_error:02X}")
            if abs(current_actual) > self.args.actual_current_limit:
                raise RuntimeError(f"实际电流越限: {current_actual}")
            if temperature > self.args.temperature_limit:
                raise RuntimeError(f"温度越限: {temperature}°C")
            if voltage < self.args.voltage_min or voltage > self.args.voltage_max:
                raise RuntimeError(f"输入电压异常: {voltage:.1f}V")

        if now - self.last_status >= 0.5:
            self.last_status = now
            print(f"[{self.state}] Inspire top_touch max={inspire_raw} "
                  f"fit={inspire_force:.3f}N target={self.filtered_target:.3f}N "
                  f"FSR raw={fsr_raw:.3f} measured={self.filtered_measured:.3f}N "
                  f"pos_raw={position_raw} pos_norm={position}")

    def close(self) -> None:
        self.running = False
        self.stop("程序退出")
        self.port.closePort()


def load_neutral(path: Path) -> int:
    data = json.loads(path.read_text(encoding="utf-8"))
    return int(data["servos"][str(INDEX_SERVO_ID)]["neutral_position"])


def main() -> int:
    parser = argparse.ArgumentParser(description="左食指ID7、4.903N阈值开关力反馈测试")
    parser.add_argument("--enable-write", action="store_true")
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--neutral-file", type=Path,
                        default=Path("exoskeleton/config/left_neutral.json"))
    parser.add_argument("--neutral", type=int)
    parser.add_argument("--fsr-host", default="127.0.0.1")
    parser.add_argument("--fsr-port", type=int, default=9002)
    parser.add_argument("--force-host", default="127.0.0.1")
    parser.add_argument("--force-port", type=int, default=9202)
    parser.add_argument("--fsr-threshold", type=float, default=4.903,
                        help="STM32最低有效力阈值；≤阈值为0，>阈值保留原值")
    parser.add_argument("--fsr-raw-max", type=float, default=60.0)
    parser.add_argument("--comfort-scale", type=float, default=1.0)
    parser.add_argument("--target-max", type=float, default=6.0)
    parser.add_argument("--contact-on", type=float, default=4.95)
    parser.add_argument("--contact-off", type=float, default=4.80)
    parser.add_argument("--kp", type=float, default=3.0,
                        help="Goal Current原始单位/N")
    parser.add_argument("--current-slew", type=int, default=1,
                        help="Goal Current每个控制周期最大增量")
    parser.add_argument("--max-goal-current", type=int, default=15)
    parser.add_argument("--release-goal-current", type=int, default=15)
    parser.add_argument("--release-timeout", type=float, default=3.0)
    parser.add_argument("--actual-current-limit", type=int, default=30)
    parser.add_argument("--max-travel", type=int, default=40)
    parser.add_argument("--reel-in-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--position-tolerance", type=int, default=10)
    parser.add_argument("--neutral-tolerance", type=int, default=8)
    parser.add_argument("--neutral-start-tolerance", type=int, default=40)
    parser.add_argument("--input-timeout", type=float, default=0.5)
    parser.add_argument("--filter-alpha", type=float, default=0.2)
    parser.add_argument("--control-hz", type=float, default=20.0)
    parser.add_argument("--health-interval", type=float, default=0.5)
    parser.add_argument("--temperature-limit", type=int, default=55)
    parser.add_argument("--voltage-min", type=float, default=4.5)
    parser.add_argument("--voltage-max", type=float, default=8.5)
    args = parser.parse_args()
    if args.neutral is None:
        args.neutral = load_neutral(args.neutral_file)
    if not (4.903 < args.target_max <= 6.0 and 0 < args.comfort_scale <= 1.0):
        parser.error("首测target-max必须在4.903..6N，comfort-scale必须在0..1")
    if not (1 <= args.max_goal_current <= 30 and 1 <= args.max_travel <= 60):
        parser.error("首测Goal Current必须<=30，行程必须<=60 tick")
    if not (1 <= args.release_goal_current <= args.max_goal_current):
        parser.error("release-goal-current必须在1..max-goal-current")
    if not (1.0 <= args.release_timeout <= 5.0):
        parser.error("release-timeout必须在1..5秒")
    if not (1 <= args.current_slew <= 3):
        parser.error("首测current-slew必须在1..3")

    controller = Controller(args)

    def stop_handler(_signum, _frame):
        controller.running = False

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    try:
        controller.open()
        controller.start_inputs()
        threading.Thread(target=controller.loop, daemon=True).start()
        print("命令: STATUS | ARM | STOP | QUIT")
        print("默认STOP；首轮必须未穿戴，并准备物理断电。")
        while controller.running:
            try:
                command = input("> ").strip().upper()
            except EOFError:
                break
            if command == "ARM":
                controller.arm()
            elif command == "STOP":
                controller.stop("人工STOP")
            elif command == "STATUS":
                (fsr_raw, fsr_t, inspire_raw, inspire_force,
                 inspire_t, valid) = controller.inputs.snapshot()
                now = time.monotonic()
                print(f"state={controller.state}, armed={controller.armed}, fault={controller.fault_reason or '-'}")
                print(f"FSR raw={fsr_raw}, age={now-fsr_t if fsr_t else float('inf'):.3f}s")
                print(f"Inspire index top_touch max={inspire_raw}, "
                      f"fit={inspire_force:.3f}N, valid={valid}, "
                      f"age={now-inspire_t if inspire_t else float('inf'):.3f}s")
            elif command == "QUIT":
                break
            elif command:
                print("命令: STATUS | ARM | STOP | QUIT")
    except Exception as exc:
        print(f"启动/运行失败: {exc}", file=sys.stderr)
        return 1
    finally:
        controller.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
