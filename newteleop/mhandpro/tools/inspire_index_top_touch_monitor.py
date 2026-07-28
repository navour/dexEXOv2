#!/usr/bin/env python3
"""Inspire左食指 fingerfour_top_touch 纯只读监视。

只读Modbus地址4128的96个16位寄存器，不写角度、力、速度或标定寄存器。
数据按官方SDK重排为12x8，并复用旧力反馈程序的食指阵列最大值换算。
"""

from __future__ import annotations

import argparse
import statistics
import struct
import sys
import time

from pymodbus.client import ModbusTcpClient


REGISTER = 4128
ROWS = 12
COLS = 8
COUNT = ROWS * COLS
CALIBRATION_K = 0.00292650244415058227
CALIBRATION_B = -0.6037947156125716
FORCE_MAX_N = 10.0


def signed_words(registers: list[int]) -> list[int]:
    if len(registers) != COUNT:
        raise ValueError(f"期望{COUNT}个寄存器，实际{len(registers)}个")
    packed = struct.pack(">" + "H" * COUNT, *registers)
    return list(struct.unpack(">" + "h" * COUNT, packed))


def raw_to_force_n(raw: float) -> float:
    force = CALIBRATION_K * raw + CALIBRATION_B
    return max(0.0, min(force, FORCE_MAX_N))


def read_touch(client: ModbusTcpClient, device_id: int) -> list[int]:
    response = client.read_holding_registers(
        address=REGISTER, count=COUNT, slave=device_id)
    if response.isError() or not hasattr(response, "registers"):
        raise RuntimeError(f"读取fingerfour_top_touch失败: {response}")
    return signed_words(list(response.registers))


def print_matrix(values: list[int]) -> None:
    print("12x8阵列：")
    for row in range(ROWS):
        line = values[row * COLS:(row + 1) * COLS]
        print("  " + " ".join(f"{value:6d}" for value in line))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Inspire左食指fingerfour_top_touch纯只读监视")
    parser.add_argument("--ip", default="192.168.123.210")
    parser.add_argument("--port", type=int, default=6000)
    parser.add_argument("--device-id", type=int, default=1)
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="0表示持续运行")
    parser.add_argument("--matrix", action="store_true",
                        help="每秒额外打印一次12x8阵列")
    args = parser.parse_args()
    if not 1.0 <= args.hz <= 10.0:
        parser.error("--hz必须在1..10之间")
    if args.seconds < 0.0:
        parser.error("--seconds不能为负数")

    print("========== Inspire左食指指尖阵列只读监视 ==========")
    print(f"目标: {args.ip}:{args.port}, Device ID={args.device_id}")
    print(f"数据: fingerfour_top_touch, 地址={REGISTER}, {ROWS}x{COLS}")
    print("安全声明：只读96个寄存器，不写入任何Inspire寄存器。")
    print("指标：max_raw=旧程序使用的阵列最大值；top5_mean=最高5点均值。")

    client = ModbusTcpClient(args.ip, port=args.port, timeout=2.0)
    started = time.monotonic()
    next_matrix = started
    sequence = 0
    try:
        if not client.connect():
            print("Modbus TCP连接失败。", file=sys.stderr)
            return 2
        period = 1.0 / args.hz
        while args.seconds == 0.0 or time.monotonic() - started < args.seconds:
            cycle = time.monotonic()
            values = read_touch(client, args.device_id)
            sequence += 1
            maximum = max(values)
            maximum_index = values.index(maximum)
            row, column = divmod(maximum_index, COLS)
            top5 = sorted(values, reverse=True)[:5]
            top5_mean = statistics.fmean(top5)
            nonzero = sum(value > 0 for value in values)
            force = raw_to_force_n(maximum)
            print(
                f"seq={sequence:06d} max_raw={maximum:6d} "
                f"cell=({row:02d},{column}) top5_mean={top5_mean:8.1f} "
                f"nonzero={nonzero:2d}/{COUNT} old_fit={force:5.3f}N"
            )
            now = time.monotonic()
            if args.matrix and now >= next_matrix:
                print_matrix(values)
                next_matrix = now + 1.0
            delay = period - (time.monotonic() - cycle)
            if delay > 0:
                time.sleep(delay)
    except KeyboardInterrupt:
        print("\n收到Ctrl+C，停止只读监视。")
    except Exception as exc:
        print(f"读取失败: {exc}", file=sys.stderr)
        return 1
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
