#!/usr/bin/env python3
"""只读监视 STM32/BLE 帧末尾五路外骨骼 FSR 值。"""

from __future__ import annotations

import argparse
import statistics
import sys
import queue
import threading
import time
from collections import deque

from exo_pressure_common import FINGER_NAMES_CN, PressureSample, iter_pressure_samples


HAND_PORTS = {"right": 9001, "left": 9002}


def format_hand(side: str, sample: PressureSample, history: deque) -> str:
    columns = list(zip(*(item.values for item in history)))
    fields = []
    for name, value, column in zip(FINGER_NAMES_CN, sample.values, columns):
        mean = statistics.fmean(column)
        std = statistics.pstdev(column) if len(column) > 1 else 0.0
        fields.append(f"{name}={value:.4f}(均值{mean:.4f},σ{std:.4f})")
    label = "右手" if side == "right" else "左手"
    return f"[{label}] " + " | ".join(fields)


def receive_hand(host: str, side: str, port: int,
                 events: queue.Queue, stop: threading.Event) -> None:
    try:
        for sample in iter_pressure_samples(host, port):
            if stop.is_set():
                return
            events.put(("sample", side, sample))
    except Exception as exc:
        if not stop.is_set():
            events.put(("error", side, exc))


def main() -> int:
    parser = argparse.ArgumentParser(description="外骨骼单手/双手薄膜压力只读监视")
    parser.add_argument("--host", default="127.0.0.1", help="BLE broker TCP地址")
    parser.add_argument("--hand", choices=("left", "right", "both"), default="left",
                        help="选择左手、右手或双手；双手默认右9001/左9002")
    parser.add_argument("--port", type=int, help="覆盖手别默认端口")
    parser.add_argument("--right-port", type=int, default=9001,
                        help="--hand both时的右手端口（默认9001）")
    parser.add_argument("--left-port", type=int, default=9002,
                        help="--hand both时的左手端口（默认9002）")
    parser.add_argument("--print-hz", type=float, default=5.0)
    parser.add_argument("--window", type=float, default=2.0, help="统计窗口（秒）")
    parser.add_argument("--seconds", type=float, default=0.0, help="0表示持续运行")
    args = parser.parse_args()
    if args.print_hz <= 0 or args.window <= 0 or args.seconds < 0:
        parser.error("print-hz和window必须>0，seconds必须>=0")
    if args.hand == "both" and args.port is not None:
        parser.error("--hand both不能使用--port，请用--right-port/--left-port")

    sides = ("right", "left") if args.hand == "both" else (args.hand,)
    ports = {
        side: ((args.right_port if side == "right" else args.left_port)
               if args.hand == "both" else (args.port or HAND_PORTS[side]))
        for side in sides
    }
    histories = {side: deque() for side in sides}
    latest = {}
    counts = {side: 0 for side in sides}
    started = time.monotonic()
    next_print = started
    print("========== 外骨骼薄膜压力只读监视 ==========")
    print("数据源: " + ", ".join(
        f"{'右手' if side == 'right' else '左手'}={args.host}:{ports[side]}"
        for side in sides))
    print("通道: " + ", ".join(f"{i}={n}" for i, n in enumerate(FINGER_NAMES_CN)))
    print("本程序不控制舵机，也不向STM32写数据。Ctrl+C退出。")

    events = queue.Queue()
    stop = threading.Event()
    for side in sides:
        threading.Thread(
            target=receive_hand,
            args=(args.host, side, ports[side], events, stop),
            daemon=True,
        ).start()

    exit_code = 0
    try:
        while True:
            if args.seconds and time.monotonic() - started >= args.seconds:
                break
            try:
                kind, side, payload = events.get(timeout=0.2)
            except queue.Empty:
                continue
            if kind == "error":
                print(f"[错误] {'右手' if side == 'right' else '左手'}: {payload}",
                      file=sys.stderr)
                exit_code = 1
                break
            sample = payload
            counts[side] += 1
            latest[side] = sample
            history = histories[side]
            history.append(sample)
            cutoff = sample.mono_time - args.window
            while history and history[0].mono_time < cutoff:
                history.popleft()
            if sample.mono_time >= next_print and all(side in latest for side in sides):
                for output_side in sides:
                    print(format_hand(output_side, latest[output_side],
                                      histories[output_side]), flush=True)
                if len(sides) == 2:
                    print("-" * 80, flush=True)
                next_print = sample.mono_time + 1.0 / args.print_hz
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()

    elapsed = max(time.monotonic() - started, 1e-9)
    summary = "，".join(
        f"{'右手' if side == 'right' else '左手'}{counts[side]}帧/"
        f"{counts[side] / elapsed:.1f}Hz" for side in sides)
    print(f"结束：{summary}。")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
