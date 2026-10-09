#!/usr/bin/env python3
"""左食指手动拉绳位移/FSR联合测量。

硬件全程只读：自动采集 P0，用户在拉绳过程中仅需依次输入
P1、P2、P3、P4。
"""

from __future__ import annotations

import argparse
import queue
import statistics
import sys
import threading
import time
from collections import deque
from pathlib import Path

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler


FSR_DIR = Path(__file__).resolve().parents[1] / "FSR"
sys.path.insert(0, str(FSR_DIR))
from exo_pressure_common import iter_pressure_samples  # noqa: E402


PROTOCOL_VERSION = 2.0
POSITION_TICKS_PER_REV = 4096
INDEX_SERVO_ID = 7
INDEX_FSR_INDEX = 1
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70
ADDR_PRESENT_POSITION = 132


def checked(value: int, result: int, error: int, packet, action: str) -> int:
    if result != COMM_SUCCESS:
        raise RuntimeError(f"{action}: {packet.getTxRxResult(result)}")
    if error:
        raise RuntimeError(f"{action}: {packet.getRxPacketError(error)}")
    return value


def signed32(value: int) -> int:
    return value - 0x100000000 if value & 0x80000000 else value


def unwrap_position(position: int, previous: int) -> int:
    """将当前读数映射到与上一采样连续的圈。

    不能永远相对 P0 取最近圈，否则总行程超过半圈（2048 tick）后
    会翻转方向。在20 Hz手动测量中，相邻样本不可能跨越半圈。
    """
    turns = round((previous - position) / POSITION_TICKS_PER_REV)
    return position + turns * POSITION_TICKS_PER_REV


def input_worker(commands: queue.Queue[str]) -> None:
    while True:
        line = sys.stdin.readline()
        if not line:
            commands.put("EOF")
            return
        commands.put(line.strip().upper())


def fsr_worker(host: str, port: int, history: deque, lock: threading.Lock,
               status: dict, stop: threading.Event) -> None:
    try:
        for sample in iter_pressure_samples(host, port):
            with lock:
                history.append((sample.mono_time, sample.values[INDEX_FSR_INDEX]))
                status["last_fsr_time"] = sample.mono_time
            if stop.is_set():
                return
    except Exception as exc:
        with lock:
            status["fsr_error"] = str(exc)


def recent_median(history: deque, now: float, window: float):
    values = [value for stamp, value in history if now - stamp <= window]
    return statistics.median(values) if values else None


def main() -> int:
    parser = argparse.ArgumentParser(
        description="ID7位置与食指FSR手动拉绳只读测量"
    )
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--fsr-host", default="127.0.0.1")
    parser.add_argument("--fsr-port", type=int, default=9002)
    parser.add_argument("--pulley-diameter", type=float, default=40.0,
                        help="舵盘有效直径(mm)")
    parser.add_argument("--p0-seconds", type=float, default=2.0,
                        help="启动时自动采集P0的时长")
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--print-hz", type=float, default=2.0)
    parser.add_argument("--capture-window", type=float, default=0.3,
                        help="记录P点时使用的中位数窗口(秒)")
    args = parser.parse_args()
    if args.pulley_diameter <= 0 or args.p0_seconds < 1.0:
        parser.error("舵盘直径必须>0，P0采集时间必须>=1秒")
    if args.sample_hz <= 0 or args.print_hz <= 0 or args.capture_window <= 0:
        parser.error("采样、显示频率和捕获窗口必须>0")

    port = PortHandler(args.device)
    packet = PacketHandler(PROTOCOL_VERSION)
    if not port.openPort() or not port.setBaudRate(args.baud):
        print("无法打开串口或设置波特率", file=sys.stderr)
        return 2

    lock = threading.Lock()
    stop = threading.Event()
    fsr_history: deque = deque(maxlen=max(100, round(args.sample_hz * 10)))
    position_history: deque = deque(maxlen=max(100, round(args.sample_hz * 10)))
    fsr_status = {"last_fsr_time": None, "fsr_error": None}
    fsr_thread = threading.Thread(
        target=fsr_worker,
        args=(args.fsr_host, args.fsr_port, fsr_history, lock, fsr_status, stop),
        daemon=True,
    )

    try:
        model, result, error = packet.ping(port, INDEX_SERVO_ID)
        checked(model, result, error, packet, f"ID {INDEX_SERVO_ID} Ping")
        torque = checked(*packet.read1ByteTxRx(port, INDEX_SERVO_ID, ADDR_TORQUE_ENABLE),
                         packet, "读取Torque Enable")
        hw_error = checked(*packet.read1ByteTxRx(port, INDEX_SERVO_ID, ADDR_HARDWARE_ERROR),
                           packet, "读取Hardware Error")
        if torque != 0:
            raise RuntimeError("ID7扭矩仍开启：请先STOP/QUIT力控程序")
        if hw_error != 0:
            raise RuntimeError(f"ID7硬件错误=0x{hw_error:02X}")

        fsr_thread.start()
        print("========== 左食指手动拉绳位移 + FSR测量 ==========")
        print(f"ID7 model={model}，torque=0，hw_error=0x00")
        print(f"FSR={args.fsr_host}:{args.fsr_port}，食指通道=1")
        print("硬件全程只读；出现疼痛或卡滞立即放绳/Ctrl+C。")
        print(f"请保持食指自然伸直和绳索起点，自动采集P0 {args.p0_seconds:.1f}s ...")

        p0_raw = []
        p0_fsr = []
        deadline = time.monotonic() + args.p0_seconds
        while time.monotonic() < deadline:
            raw = checked(*packet.read4ByteTxRx(port, INDEX_SERVO_ID, ADDR_PRESENT_POSITION),
                          packet, "读取Present Position")
            p0_raw.append(signed32(raw))
            with lock:
                latest_fsr = recent_median(fsr_history, time.monotonic(), 0.5)
                fsr_error = fsr_status["fsr_error"]
            if latest_fsr is not None:
                p0_fsr.append(latest_fsr)
            if fsr_error:
                raise RuntimeError(f"FSR连接失败: {fsr_error}")
            time.sleep(1.0 / args.sample_hz)
        if not p0_fsr:
            raise RuntimeError("P0采集期间未收到FSR数据，请检查9002 broker")

        p0 = round(statistics.median(p0_raw))
        p0_span = max(p0_raw) - min(p0_raw)
        p0_force = statistics.median(p0_fsr)
        mm_per_tick = 3.141592653589793 * args.pulley_diameter / POSITION_TICKS_PER_REV
        print(f"P0自动锁定: position={p0}, span={p0_span} tick, "
              f"FSR={p0_force:.4f}N, 1tick={mm_per_tick:.5f}mm")
        if p0_span > 8:
            raise RuntimeError("P0位置波动超过8 tick，请保持机构不动后重试")

        commands: queue.Queue[str] = queue.Queue()
        threading.Thread(target=input_worker, args=(commands,), daemon=True).start()
        expected = ["P1", "P2", "P3", "P4"]
        records = {"P0": (p0, 0, 0.0, p0_force)}
        print("开始缓慢拉绳：到达各点时依次输入 P1、P2、P3、P4 并回车。")
        next_print = 0.0
        previous_position = p0
        while expected:
            now = time.monotonic()
            raw = checked(*packet.read4ByteTxRx(port, INDEX_SERVO_ID, ADDR_PRESENT_POSITION),
                          packet, "读取Present Position")
            position = unwrap_position(signed32(raw), previous_position)
            previous_position = position
            position_history.append((now, float(position)))
            with lock:
                force = recent_median(fsr_history, now, args.capture_window)
                last_fsr_time = fsr_status["last_fsr_time"]
                fsr_error = fsr_status["fsr_error"]
            if fsr_error:
                raise RuntimeError(f"FSR读取失败: {fsr_error}")
            fsr_age = None if last_fsr_time is None else now - last_fsr_time
            if fsr_age is None or fsr_age > 1.0:
                raise RuntimeError("FSR数据超时超过1秒")

            if now >= next_print:
                delta = position - p0
                force_text = "--" if force is None else f"{force:.4f}N"
                print(f"[实时] pos={position} delta={delta:+d}tick "
                      f"rope={abs(delta) * mm_per_tick:.3f}mm FSR={force_text} "
                      f"| 等待{expected[0]}", flush=True)
                next_print = now + 1.0 / args.print_hz

            try:
                command = commands.get_nowait()
            except queue.Empty:
                command = None
            if command:
                if command == "EOF":
                    raise RuntimeError("标准输入已关闭")
                if command != expected[0]:
                    print(f"当前应输入 {expected[0]}，收到 {command or '<空>'}，未记录。")
                else:
                    capture_time = time.monotonic()
                    captured_position = recent_median(
                        position_history, capture_time, args.capture_window
                    )
                    with lock:
                        captured_force = recent_median(
                            fsr_history, capture_time, args.capture_window
                        )
                    if captured_position is None or captured_force is None:
                        print("最近数据不足，请保持当前位置后再输入。")
                    else:
                        captured_position = round(captured_position)
                        delta = captured_position - p0
                        label = expected.pop(0)
                        records[label] = (
                            captured_position,
                            delta,
                            abs(delta) * mm_per_tick,
                            captured_force,
                        )
                        print(f"*** {label}已记录: pos={captured_position}, "
                              f"delta={delta:+d}tick, rope={abs(delta) * mm_per_tick:.3f}mm, "
                              f"FSR={captured_force:.4f}N ***")
            time.sleep(1.0 / args.sample_hz)

        print("\n========== P0～P4汇总 ==========")
        print("点   position   delta(tick)   rope(mm)   FSR(N)")
        for label in ("P0", "P1", "P2", "P3", "P4"):
            position, delta, distance, force = records[label]
            print(f"{label:<4} {position:>8} {delta:>+12} {distance:>10.3f} {force:>8.4f}")
        print("请将上述汇总完整发回，再决定自动行程上限。")
        return 0
    except KeyboardInterrupt:
        print("\n已取消；请放松绳索。")
        return 130
    except Exception as exc:
        print(f"[错误] {exc}", file=sys.stderr)
        return 1
    finally:
        stop.set()
        port.closePort()
        print("串口已关闭；程序未写入任何舵机或STM32寄存器。")


if __name__ == "__main__":
    raise SystemExit(main())
