#!/usr/bin/env python3
"""逐指按压并用相对基线增量识别 FSR 物理手指与帧通道的对应关系。"""

from __future__ import annotations

import argparse
import statistics
import sys
import time

from exo_pressure_common import FINGER_NAMES, FINGER_NAMES_CN, iter_pressure_samples


def collect(samples, seconds: float) -> list[list[float]]:
    end = time.monotonic() + seconds
    rows = []
    while time.monotonic() < end:
        rows.append(list(next(samples).values))
    if len(rows) < 3:
        raise RuntimeError(f"采样不足：仅{len(rows)}帧")
    return rows


def medians(rows: list[list[float]]) -> list[float]:
    return [statistics.median(column) for column in zip(*rows)]


def main() -> int:
    parser = argparse.ArgumentParser(description="右/左手FSR通道逐指识别（全程只读）")
    parser.add_argument("--hand", choices=("right", "left"), default="right")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, help="默认右手9001、左手9002")
    parser.add_argument("--baseline-seconds", type=float, default=3.0)
    parser.add_argument("--press-seconds", type=float, default=2.0)
    parser.add_argument("--min-delta", type=float, default=0.05,
                        help="最大通道至少高于基线此值才接受")
    args = parser.parse_args()
    port = args.port or (9001 if args.hand == "right" else 9002)
    if args.baseline_seconds <= 0 or args.press_seconds <= 0 or args.min_delta < 0:
        parser.error("采样时间必须>0，min-delta必须>=0")

    print("========== FSR物理手指—数据通道识别 ==========")
    print(f"手={args.hand} 数据源={args.host}:{port}；全程只读，不控制舵机。")
    print("先让五片FSR完全卸载。测试时每次只压指定的一片，其他四片不要触碰。")
    samples = iter(iter_pressure_samples(args.host, port))
    try:
        input("保持全部卸载，按回车采集基线...")
        baseline = medians(collect(samples, args.baseline_seconds))
        print("基线: " + ", ".join(f"ch{i}={v:.4f}" for i, v in enumerate(baseline)))
        result: dict[str, int] = {}
        confidence = True
        for physical, physical_cn in zip(FINGER_NAMES, FINGER_NAMES_CN):
            input(f"只持续按压右/左手的【{physical_cn}】FSR，稳定后按回车...")
            pressed = medians(collect(samples, args.press_seconds))
            delta = [value - base for value, base in zip(pressed, baseline)]
            ranked = sorted(range(5), key=lambda i: delta[i], reverse=True)
            best, second = ranked[:2]
            accepted = delta[best] >= args.min_delta and delta[best] > max(0.0, delta[second]) * 1.5
            confidence &= accepted
            result[physical] = best
            print("  增量: " + ", ".join(f"ch{i}={v:+.4f}" for i, v in enumerate(delta)))
            print(f"  判定: {physical_cn} -> ch{best}" + ("" if accepted else "（不可靠，请重测）"))
            input("松开该片并等待回落，然后按回车继续...")

        duplicates = len(set(result.values())) != 5
        print("\n识别结果（物理手指 -> 帧末五路索引）:")
        for name, cn in zip(FINGER_NAMES, FINGER_NAMES_CN):
            print(f"  {cn} ({name}) -> {result[name]}")
        if duplicates or not confidence:
            print("[未通过] 存在重复通道或信号分离度不足；不要据此接入力控，请重新固定并逐指测试。")
            return 2
        inverse = [next(name for name, channel in result.items() if channel == i) for i in range(5)]
        print("帧顺序: [" + ", ".join(inverse) + "]")
        print("[通过] 五个物理手指分别映射到五个唯一通道。建议断电重启后再重复一次。")
        return 0
    except (KeyboardInterrupt, EOFError):
        print("\n已取消。")
        return 130
    except Exception as exc:
        print(f"[错误] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
