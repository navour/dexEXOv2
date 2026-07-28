#!/usr/bin/env python3
"""Inspire左手低行程组合测试：缓慢握合、保持、再安全返回张手位。"""

import argparse
import json
import socket
import time


DEFAULT_OPEN = [980, 965, 957, 946, 949, 922]
DEFAULT_CLOSED = [60, 60, 70, 60, 70, 150]


def six_ints(text: str) -> list[int]:
    try:
        values = [int(part.strip()) for part in text.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("必须是逗号分隔的6个整数") from exc
    if len(values) != 6 or any(value < 0 or value > 1000 for value in values):
        raise argparse.ArgumentTypeError("必须是0~1000内的6个整数")
    return values


def ctrl_packet(angles: list[int]) -> bytes:
    return (json.dumps({
        "type": "ctrl",
        "angle_set": angles,
        "force_set": [100] * 6,
        "speed_set": [100] * 6,
        "mode": 1,
    }) + "\n").encode()


def send(sock: socket.socket, angles: list[int]) -> None:
    sock.sendall(ctrl_packet(angles))


def hold(sock: socket.socket, angles: list[int], seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        send(sock, angles)
        time.sleep(0.1)


def ramp(sock: socket.socket, start: list[int], target: list[int], step: int) -> list[int]:
    # 原地更新，确保中途异常时调用方仍知道最后一次已发送的位置。
    current = start
    while current != target:
        for i in range(6):
            delta = target[i] - current[i]
            current[i] += max(-step, min(step, delta))
        send(sock, current)
        time.sleep(0.1)
    return current


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspire左手30%组合握合安全测试")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9102)
    parser.add_argument("--open", dest="open_pose", type=six_ints, default=DEFAULT_OPEN)
    parser.add_argument("--closed", type=six_ints, default=DEFAULT_CLOSED)
    parser.add_argument("--scale", type=float, default=0.30, help="闭合行程比例，硬限制不超过0.30")
    parser.add_argument("--step", type=int, default=10, help="每100ms单通道最大变化，范围1~20")
    parser.add_argument("--hold", type=float, default=3.0, help="目标姿态保持时间/秒")
    args = parser.parse_args()

    if not 0.0 < args.scale <= 0.30:
        parser.error("--scale必须大于0且不得超过0.30")
    if not 1 <= args.step <= 20:
        parser.error("--step必须在1~20内")
    if args.hold < 0.0 or args.hold > 10.0:
        parser.error("--hold必须在0~10秒内")

    target = [round(o + args.scale * (c - o))
              for o, c in zip(args.open_pose, args.closed)]
    print("========== Inspire左手组合安全测试 ==========")
    print(f"安全张手位: {args.open_pose}")
    print(f"安全闭合端: {args.closed}")
    print(f"本次比例: {args.scale:.0%}")
    print(f"组合目标位: {target}")
    print("动作顺序：确认张手 -> 缓慢握合 -> 保持 -> 缓慢返回张手。")
    confirmation = input("确认手内无物体、周围无人，输入 TEST30 开始: ")
    if confirmation != "TEST30":
        print("已取消，未连接桥接器、未发送命令。")
        return 0

    sock: socket.socket | None = None
    current = list(args.open_pose)
    try:
        sock = socket.create_connection((args.host, args.port), timeout=3.0)
        sock.settimeout(None)
        print("已连接桥接器，先保持安全张手位0.5秒。")
        hold(sock, current, 0.5)
        print("正在缓慢运动到30%组合姿态...")
        current = ramp(sock, current, target, args.step)
        print(f"已到达目标 {current}，保持 {args.hold:.1f} 秒。")
        hold(sock, current, args.hold)
        print("正在缓慢返回安全张手位...")
        current = ramp(sock, current, args.open_pose, args.step)
        hold(sock, current, 0.5)
        print("测试完成，已返回安全张手位。")
        return 0
    except KeyboardInterrupt:
        print("\n收到中止信号，尝试分步返回安全张手位...")
        return 130
    except Exception as exc:
        print(f"测试异常: {exc}")
        return 1
    finally:
        if sock is not None:
            try:
                if current != args.open_pose:
                    ramp(sock, current, args.open_pose, args.step)
                    hold(sock, args.open_pose, 0.3)
                    print("已执行异常返回。")
            except Exception as exc:
                print(f"主动返回失败，桥接器看门狗应返回安全位: {exc}")
            sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
