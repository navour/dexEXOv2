#!/usr/bin/env python3
"""通过 BLE broker 和推拉力计交互标定五片外骨骼 FSR。"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

from exo_pressure_common import (
    FINGER_NAMES,
    FINGER_NAMES_CN,
    PressureSample,
    iter_pressure_samples,
)


@dataclass
class SampleStore:
    lock: threading.Lock = field(default_factory=threading.Lock)
    samples: list[PressureSample] = field(default_factory=list)
    error: str | None = None

    def append(self, sample: PressureSample) -> None:
        with self.lock:
            self.samples.append(sample)
            cutoff = sample.mono_time - 120.0
            while self.samples and self.samples[0].mono_time < cutoff:
                self.samples.pop(0)

    def interval(self, start: float, end: float) -> list[PressureSample]:
        with self.lock:
            return [s for s in self.samples if start <= s.mono_time <= end]

    def latest(self) -> PressureSample | None:
        with self.lock:
            return self.samples[-1] if self.samples else None


def stats(values: list[float]) -> dict[str, float | int]:
    return {
        "samples": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "min": min(values),
        "max": max(values),
        "stddev": statistics.pstdev(values) if len(values) > 1 else 0.0,
    }


def fit_channel(points: list[dict], baseline: float) -> dict:
    grouped: dict[float, list[float]] = {}
    for point in points:
        grouped.setdefault(float(point["force_N"]), []).append(float(point["raw_mean"]))
    piecewise = [
        {"force_N": force, "raw": statistics.fmean(raws),
         "raw_delta": statistics.fmean(raws) - baseline}
        for force, raws in sorted(grouped.items())
    ]
    usable = [(p["raw_delta"], p["force_N"]) for p in piecewise if p["force_N"] > 0]
    denominator = sum(delta * delta for delta, _ in usable)
    slope = sum(delta * force for delta, force in usable) / denominator if denominator else None
    predictions = [slope * delta for delta, _ in usable] if slope is not None else []
    rmse = (
        math.sqrt(sum((pred - force) ** 2 for pred, (_, force) in zip(predictions, usable)) / len(usable))
        if usable else None
    )
    return {
        "model": "piecewise_linear",
        "points": piecewise,
        "linear_through_tare": {"N_per_raw_delta": slope, "rmse_N": rmse},
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="外骨骼薄膜传感器交互式推拉力计标定")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9002)
    parser.add_argument("--sample-seconds", type=float, default=3.0)
    parser.add_argument("--zero-seconds", type=float, default=10.0)
    parser.add_argument("--output", type=Path,
                        default=Path("exoskeleton/FSR/left_pressure_calibration.json"))
    parser.add_argument("--hand", choices=("left", "right"), default="left")
    parser.add_argument("--ble-mac", default="F0:FD:45:02:67:3B")
    args = parser.parse_args()
    if args.sample_seconds <= 0 or args.zero_seconds <= 0:
        parser.error("采样时长必须大于0")

    store = SampleStore()
    stop = threading.Event()

    def receiver() -> None:
        try:
            for sample in iter_pressure_samples(args.host, args.port):
                if stop.is_set():
                    return
                store.append(sample)
        except Exception as exc:
            store.error = str(exc)

    threading.Thread(target=receiver, daemon=True).start()
    print("========== 外骨骼薄膜传感器交互标定 ==========")
    print("传感器应平放在稳定桌面夹具中，用推拉力计垂直加载；舵机保持断电。")
    print("通道: " + ", ".join(f"{i}={cn}" for i, cn in enumerate(FINGER_NAMES_CN)))
    print("命令: ZERO | FINGER <名称/0-4> | POINT <N> | STATUS | FIT | SAVE | QUIT")

    deadline = time.monotonic() + 8.0
    while store.latest() is None and time.monotonic() < deadline and store.error is None:
        time.sleep(0.05)
    if store.latest() is None:
        print(f"[错误] 未收到数据: {store.error or '等待超时'}", file=sys.stderr)
        return 1

    baseline_stats: list[dict] | None = None
    selected = 1
    points: dict[str, list[dict]] = {name: [] for name in FINGER_NAMES}

    def collect(seconds: float) -> list[PressureSample]:
        if store.error:
            raise RuntimeError(store.error)
        start = time.monotonic()
        print(f"采集中，请保持稳定 {seconds:.1f}s ...")
        time.sleep(seconds)
        result = store.interval(start, time.monotonic())
        if len(result) < 3:
            raise RuntimeError(f"有效样本不足：仅{len(result)}帧")
        return result

    try:
        while True:
            raw = input(f"标定[{FINGER_NAMES_CN[selected]}]> ").strip()
            if not raw:
                continue
            parts = raw.split()
            command = parts[0].upper()
            if command == "QUIT":
                break
            if command == "ZERO":
                samples = collect(args.zero_seconds)
                columns = list(zip(*(s.values for s in samples)))
                baseline_stats = [stats(list(column)) for column in columns]
                for item, column in zip(baseline_stats, columns):
                    item["raw_values"] = list(column)
                for cn, item in zip(FINGER_NAMES_CN, baseline_stats):
                    print(f"  {cn}: median={item['median']:.6f}, mean={item['mean']:.6f}, "
                          f"σ={item['stddev']:.6f}")
            elif command == "FINGER" and len(parts) == 2:
                key = parts[1].lower()
                aliases = {str(i): i for i in range(5)}
                aliases.update({name: i for i, name in enumerate(FINGER_NAMES)})
                aliases.update({name: i for i, name in enumerate(FINGER_NAMES_CN)})
                if key not in aliases:
                    print("未知手指；使用 thumb/index/middle/ring/pinky 或 0～4。")
                    continue
                selected = aliases[key]
            elif command == "POINT" and len(parts) == 2:
                if baseline_stats is None:
                    print("请先执行 ZERO。")
                    continue
                force = float(parts[1])
                if force < 0:
                    print("参考力不能为负。")
                    continue
                samples = collect(args.sample_seconds)
                values = [s.values[selected] for s in samples]
                item = stats(values)
                point = {
                    "force_N": force,
                    "raw_mean": item["mean"],
                    "raw_median": item["median"],
                    "raw_stddev": item["stddev"],
                    "samples": item["samples"],
                    "raw_values": values,
                    "phase": "manual",
                }
                points[FINGER_NAMES[selected]].append(point)
                delta = float(item["mean"]) - float(baseline_stats[selected]["median"])
                print(f"已记录 {FINGER_NAMES_CN[selected]} {force:.3f}N: "
                      f"raw={item['mean']:.6f}, delta={delta:.6f}, σ={item['stddev']:.6f}")
            elif command == "STATUS":
                latest = store.latest()
                age = time.monotonic() - latest.mono_time if latest else float("inf")
                print(f"数据年龄={age:.3f}s，当前值={list(latest.values) if latest else None}")
                print("记录点数: " + ", ".join(
                    f"{cn}={len(points[name])}" for name, cn in zip(FINGER_NAMES, FINGER_NAMES_CN)))
            elif command in ("FIT", "SAVE"):
                if baseline_stats is None:
                    print("请先执行 ZERO。")
                    continue
                channels = {}
                for i, (name, cn) in enumerate(zip(FINGER_NAMES, FINGER_NAMES_CN)):
                    baseline = float(baseline_stats[i]["median"])
                    channels[name] = {
                        "name_cn": cn,
                        "baseline": baseline,
                        "baseline_statistics": baseline_stats[i],
                        "measurements": points[name],
                        "fit": fit_channel(points[name], baseline),
                    }
                    linear = channels[name]["fit"]["linear_through_tare"]
                    print(f"  {cn}: 点数={len(points[name])}, "
                          f"N/raw_delta={linear['N_per_raw_delta']}, RMSE={linear['rmse_N']}")
                if command == "SAVE":
                    document = {
                        "version": 1,
                        "created_at": datetime.now(timezone.utc).isoformat(),
                        "hand": args.hand,
                        "ble_mac": args.ble_mac,
                        "source": f"tcp://{args.host}:{args.port}",
                        "frame_tail_order": list(FINGER_NAMES),
                        "reference_instrument": "push_pull_force_gauge",
                        "calibration_setup": "sensor_flat_on_table_fixture, perpendicular_loading",
                        "output_unit": "N",
                        "channels": channels,
                    }
                    args.output.parent.mkdir(parents=True, exist_ok=True)
                    args.output.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n",
                                           encoding="utf-8")
                    print(f"已保存: {args.output}")
            else:
                print("命令格式: ZERO | FINGER <名称/0-4> | POINT <N> | STATUS | FIT | SAVE | QUIT")
    except (KeyboardInterrupt, EOFError):
        print()
    except Exception as exc:
        print(f"[错误] {exc}", file=sys.stderr)
        return 1
    finally:
        stop.set()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
