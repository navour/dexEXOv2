#!/usr/bin/env python3
"""Inspire FORCE_ACT 六通道只读监视：采集空载基线并显示相对变化。"""

import argparse
import statistics
import sys
import time

from pymodbus.client import ModbusTcpClient


FORCE_ACT_REGISTER = 1582
CHANNEL_NAMES = ("小指", "无名指", "中指", "食指", "拇指弯曲", "拇指对掌")


def read_force(client, device_id):
    response = client.read_holding_registers(
        address=FORCE_ACT_REGISTER, count=6, slave=device_id
    )
    if response.isError() or not hasattr(response, "registers"):
        raise RuntimeError(f"读取 FORCE_ACT 失败: {response}")
    return [int(value) for value in response.registers]


def main():
    parser = argparse.ArgumentParser(description="Inspire六路实际力只读监视")
    parser.add_argument("--ip", default="192.168.123.210")
    parser.add_argument("--port", type=int, default=6000)
    parser.add_argument("--device-id", type=int, default=1)
    parser.add_argument("--baseline-seconds", type=float, default=2.0)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--hz", type=float, default=10.0)
    args = parser.parse_args()
    if not 1.0 <= args.baseline_seconds <= 5.0:
        parser.error("--baseline-seconds必须在1~5秒之间")
    if not 1.0 <= args.seconds <= 120.0:
        parser.error("--seconds必须在1~120秒之间")
    if not 1.0 <= args.hz <= 30.0:
        parser.error("--hz必须在1~30之间")

    client = ModbusTcpClient(args.ip, port=args.port, timeout=1.0)
    print("========== Inspire FORCE_ACT 只读监视 ==========")
    print(f"灵巧手: {args.ip}:{args.port}, Device ID={args.device_id}")
    print("安全声明：本程序只读地址1582的6个寄存器，不发送任何运动命令。")
    if not client.connect():
        print("连接 Inspire 失败", file=sys.stderr)
        return 2

    try:
        samples = [[] for _ in range(6)]
        print(f"请让灵巧手完全空载、不接触物体，保持{args.baseline_seconds:.1f}秒...")
        deadline = time.monotonic() + args.baseline_seconds
        while time.monotonic() < deadline:
            values = read_force(client, args.device_id)
            for index, value in enumerate(values):
                samples[index].append(value)
            time.sleep(0.05)

        baselines = [round(statistics.fmean(channel)) for channel in samples]
        noise_spans = [max(channel) - min(channel) for channel in samples]
        print("空载基线 [小指 无名指 中指 食指 拇指弯曲 拇指对掌]:")
        print("  baseline = [" + " ".join(str(v) for v in baselines) + "]")
        print("  noise_span = [" + " ".join(str(v) for v in noise_spans) + "]")
        print("现在可依次让灵巧手的单根手指轻触物体；按 Ctrl+C 可提前结束。")

        period = 1.0 / args.hz
        deadline = time.monotonic() + args.seconds
        next_time = time.monotonic()
        while time.monotonic() < deadline:
            values = read_force(client, args.device_id)
            deltas = [value - base for value, base in zip(values, baselines)]
            strongest = max(range(6), key=lambda i: abs(deltas[i]))
            print(
                "raw=[" + " ".join(f"{v:4d}" for v in values) + "] "
                "delta=[" + " ".join(f"{v:+4d}" for v in deltas) + "] "
                f"最大变化={CHANNEL_NAMES[strongest]}:{deltas[strongest]:+d}"
            )
            next_time += period
            time.sleep(max(0.0, next_time - time.monotonic()))
        return 0
    except KeyboardInterrupt:
        print("\n用户结束监视。")
        return 0
    except Exception as exc:
        print(f"监视失败：{exc}", file=sys.stderr)
        return 1
    finally:
        client.close()
        print("已断开 Inspire；本程序全程只读。")


if __name__ == "__main__":
    raise SystemExit(main())
