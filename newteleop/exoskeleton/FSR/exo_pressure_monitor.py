#!/usr/bin/env python3
"""只读监视 STM32/BLE 帧末尾五路外骨骼 FSR 值。"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from collections import deque

from exo_pressure_common import FINGER_NAMES_CN, iter_pressure_samples


def main() -> int:
    parser = argparse.ArgumentParser(description="外骨骼五路薄膜压力只读监视")
    parser.add_argument("--host", default="127.0.0.1", help="BLE broker TCP地址")
    parser.add_argument("--port", type=int, default=9002, help="左手默认9002")
    parser.add_argument("--print-hz", type=float, default=5.0)
    parser.add_argument("--window", type=float, default=2.0, help="统计窗口（秒）")
    parser.add_argument("--seconds", type=float, default=0.0, help="0表示持续运行")
    args = parser.parse_args()
    if args.print_hz <= 0 or args.window <= 0 or args.seconds < 0:
        parser.error("print-hz和window必须>0，seconds必须>=0")

    history = deque()
    started = time.monotonic()
    next_print = started
    count = 0
    print("========== 外骨骼薄膜压力只读监视 ==========")
    print(f"数据源: {args.host}:{args.port}")
    print("通道: " + ", ".join(f"{i}={n}" for i, n in enumerate(FINGER_NAMES_CN)))
    print("本程序不控制舵机，也不向STM32写数据。Ctrl+C退出。")

    try:
        for sample in iter_pressure_samples(args.host, args.port):
            count += 1
            history.append(sample)
            cutoff = sample.mono_time - args.window
            while history and history[0].mono_time < cutoff:
                history.popleft()
            if sample.mono_time >= next_print:
                columns = list(zip(*(item.values for item in history)))
                fields = []
                for name, value, column in zip(FINGER_NAMES_CN, sample.values, columns):
                    mean = statistics.fmean(column)
                    std = statistics.pstdev(column) if len(column) > 1 else 0.0
                    fields.append(f"{name}={value:.4f}(均值{mean:.4f},σ{std:.4f})")
                print(" | ".join(fields), flush=True)
                next_print = sample.mono_time + 1.0 / args.print_hz
            if args.seconds and sample.mono_time - started >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[错误] {exc}", file=sys.stderr)
        return 1

    elapsed = max(time.monotonic() - started, 1e-9)
    print(f"结束：收到{count}帧，平均{count / elapsed:.1f}Hz。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
