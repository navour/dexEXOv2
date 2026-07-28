#!/usr/bin/env python3
"""外骨骼五舵机零力/放松位置采集。

程序严格只读：不写寄存器、不修改模式、不开启扭矩。
只有 5 个舵机均在线、扭矩均关闭且无硬件错误时才采集。
"""

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler


PROTOCOL_VERSION = 2.0
HAND_IDS = {"right": (1, 2, 3, 4, 5), "left": (6, 7, 8, 9, 10)}
FINGER_ORDER = ("thumb", "index", "middle", "ring", "pinky")
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70
ADDR_PRESENT_POSITION = 132


def check(result, error, packet, action):
    if result != COMM_SUCCESS:
        raise RuntimeError(f"{action}: {packet.getTxRxResult(result)}")
    if error:
        raise RuntimeError(f"{action}: {packet.getRxPacketError(error)}")


def read1(packet, port, sid, address):
    value, result, error = packet.read1ByteTxRx(port, sid, address)
    check(result, error, packet, f"ID {sid} 读取地址{address}")
    return value


def read4(packet, port, sid, address):
    value, result, error = packet.read4ByteTxRx(port, sid, address)
    check(result, error, packet, f"ID {sid} 读取地址{address}")
    return value


def signed32(value):
    return value - 0x100000000 if value & 0x80000000 else value


def main():
    parser = argparse.ArgumentParser(description="外骨骼零力位置只读采集")
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--hand", choices=("left", "right"), default="left")
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    ids = HAND_IDS[args.hand]
    fingers = {sid: FINGER_ORDER[index] for index, sid in enumerate(ids)}
    if args.output is None:
        args.output = f"exoskeleton/config/{args.hand}_neutral.json"
    if not 1.0 <= args.seconds <= 5.0:
        parser.error("--seconds必须在1~5秒之间")

    port = PortHandler(args.device)
    packet = PacketHandler(PROTOCOL_VERSION)
    if not port.openPort() or not port.setBaudRate(args.baud):
        print("无法打开串口或设置波特率", file=sys.stderr)
        return 2

    try:
        print("========== 外骨骼零力基准只读采集 ==========")
        states = {}
        print(f"选择手别：{args.hand}，舵机ID：{list(ids)}")
        for sid in ids:
            model, result, error = packet.ping(port, sid)
            check(result, error, packet, f"ID {sid} Ping")
            mode = read1(packet, port, sid, ADDR_OPERATING_MODE)
            torque = read1(packet, port, sid, ADDR_TORQUE_ENABLE)
            hw_error = read1(packet, port, sid, ADDR_HARDWARE_ERROR)
            position = signed32(read4(packet, port, sid, ADDR_PRESENT_POSITION))
            print(f"ID {sid} {fingers[sid]}: model={model}, mode={mode}, "
                  f"torque={torque}, hw_error=0x{hw_error:02X}, position={position}")
            if torque != 0:
                raise RuntimeError(f"ID {sid}扭矩仍开启，拒绝佩戴采集")
            if hw_error != 0:
                raise RuntimeError(f"ID {sid}存在硬件错误")
            states[sid] = {"mode": mode, "samples": []}

        confirm = input(
            "确认所有拉绳只消除明显松垮、对手指没有拉力，输入 NEUTRAL 采集: "
        )
        if confirm != "NEUTRAL":
            print("已取消。")
            return 0

        sample_period = 0.02
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            for sid in ids:
                states[sid]["samples"].append(
                    signed32(read4(packet, port, sid, ADDR_PRESENT_POSITION))
                )
            time.sleep(sample_period)

        result_data = {
            "device": args.device,
            "baudrate": args.baud,
            "captured_unix_time": time.time(),
            "capture_seconds": args.seconds,
            "servos": {},
        }
        unstable = False
        print("\n采集结果：")
        for sid in ids:
            samples = states[sid]["samples"]
            mean = round(statistics.fmean(samples))
            minimum = min(samples)
            maximum = max(samples)
            span = maximum - minimum
            stddev = statistics.pstdev(samples)
            print(f"ID {sid} {fingers[sid]}: 基准={mean}, 范围={minimum}~{maximum}, "
                  f"波动={span} tick, 标准差={stddev:.2f}")
            if span > 8:
                unstable = True
            result_data["servos"][str(sid)] = {
                "finger": fingers[sid],
                "mode": states[sid]["mode"],
                "neutral_position": mean,
                "minimum_sample": minimum,
                "maximum_sample": maximum,
                "span": span,
                "stddev": round(stddev, 3),
            }

        if unstable:
            print("采集未保存：至少一个舵机波动超过8 tick，请保持手指不动后重试。")
            return 1

        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result_data, ensure_ascii=False, indent=2) + "\n",
                          encoding="utf-8")
        print(f"零力基准已保存：{output}")
        return 0
    except Exception as exc:
        print(f"采集失败：{exc}", file=sys.stderr)
        return 1
    finally:
        port.closePort()
        print("串口已关闭；本程序全程未写入舵机。")


if __name__ == "__main__":
    raise SystemExit(main())
