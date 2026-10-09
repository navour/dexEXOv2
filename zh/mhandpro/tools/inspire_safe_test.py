#!/usr/bin/env python3
"""Inspire 六通道固定小步测试器：先预览，显式 --send 才发送。"""

import argparse
import json
import socket
import time


def six_ints(text: str) -> list[int]:
    try:
        values = [int(part.strip()) for part in text.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("必须是逗号分隔的整数") from exc
    if len(values) != 6:
        raise argparse.ArgumentTypeError("必须恰好有6个整数")
    return values


def packet(angles: list[int]) -> bytes:
    payload = {
        "type": "ctrl",
        "angle_set": angles,
        "force_set": [100] * 6,
        "speed_set": [100] * 6,
        "mode": 1,
    }
    return (json.dumps(payload, ensure_ascii=False) + "\n").encode()


def send_repeated(sock: socket.socket, angles: list[int], seconds: float) -> None:
    deadline = time.monotonic() + seconds
    data = packet(angles)
    while time.monotonic() < deadline:
        sock.sendall(data)
        time.sleep(0.1)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="通过G1 both_hand_bridge对Inspire某一通道做低速小步运动")
    parser.add_argument("--host", default="192.168.3.85", help="G1控制IP")
    parser.add_argument("--port", type=int, required=True, help="右手9100，左手9102")
    parser.add_argument("--base", type=six_ints, required=True,
                        help="已确认安全的6路基准，如500,500,500,500,500,500")
    parser.add_argument("--channel", type=int, choices=range(6), required=True,
                        help="0小指 1无名指 2中指 3食指 4拇指弯曲 5拇指对掌")
    parser.add_argument("--delta", type=int, required=True,
                        help="小步增量，首次建议+20或-20，绝对值不得超过50")
    parser.add_argument("--duration", type=float, default=1.0, help="测试姿态保持秒数")
    parser.add_argument("--send", action="store_true", help="真正发送；不带此参数只预览")
    args = parser.parse_args()

    if abs(args.delta) > 50:
        parser.error("为了首次联调安全，|delta|不得超过50")
    target = list(args.base)
    target[args.channel] += args.delta
    names = ["小指", "无名指", "中指", "食指", "拇指弯曲", "拇指对掌"]
    print(f"通道: {args.channel} {names[args.channel]}")
    print(f"基准: {args.base}")
    print(f"目标: {target}")
    print("执行序列：基准0.5s -> 目标 -> 基准0.5s")
    if not args.send:
        print("当前为预览模式，未发送。确认后加 --send。")
        return
    confirmation = input("确认灵巧手周围无人体/障碍物，输入 SEND 继续: ")
    if confirmation != "SEND":
        print("已取消。")
        return
    with socket.create_connection((args.host, args.port), timeout=3.0) as sock:
        send_repeated(sock, args.base, 0.5)
        send_repeated(sock, target, args.duration)
        send_repeated(sock, args.base, 0.5)
    print("测试完成，已返回基准指令。")


if __name__ == "__main__":
    main()
