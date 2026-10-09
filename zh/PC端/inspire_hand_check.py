#!/usr/bin/env python3
"""因时灵巧手真机开工前检查。**只读，绝不下发任何动作。**

明天到现场第一件事跑这个。它把"手能不能通"从"遥操能不能跑"里单独摘出来 ——
链路不通的时候，你需要知道是网络、IP、寄存器，还是我们的代码。

    python3 inspire_hand_check.py [--side right|left] [--ip A.B.C.D]

全部通过再去起遥操。任何一项失败，下面都会写清楚下一步查什么。
"""

import argparse
import socket
import subprocess
import sys

import hand_mapping
import inspire_hand_ctrl as ihc


def check(label, ok, detail=""):
    mark = "[+]" if ok else "[!]"
    print(f"{mark} {label}" + (f"  {detail}" if detail else ""))
    return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("right", "left"), default="right")
    parser.add_argument("--ip", default=None, help="默认按左右手取官方地址")
    parser.add_argument("--port", type=int, default=ihc.DEFAULT_MODBUS_PORT)
    parser.add_argument("--device-id", type=int, default=ihc.DEFAULT_DEVICE_ID)
    args = parser.parse_args()

    ip = args.ip or ihc.HAND_IP[args.side]
    print(f"\n=== 因时灵巧手检查（{args.side}手 {ip}:{args.port}）===")
    print("本工具只读寄存器，不会让手动。\n")

    # 1. ping
    ok_ping = subprocess.run(
        ["ping", "-c", "2", "-W", "2", ip],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
    check("网络可达", ok_ping,
          "" if ok_ping else f"ping 不通 {ip}。检查网线、PC 是否在 "
                             "192.168.123.x 网段、手的 IP 是否被改过")
    if not ok_ping:
        print("\n先解决网络，后面的检查没有意义。\n")
        return 1

    # 2. TCP 端口
    sock = socket.socket()
    sock.settimeout(2.0)
    ok_port = sock.connect_ex((ip, args.port)) == 0
    sock.close()
    check(f"TCP {args.port} 端口开放", ok_port,
          "" if ok_port else "手上电了吗？端口是否被改成别的值？")
    if not ok_port:
        return 1

    # 3. 读实际角度
    try:
        hand = ihc.InspireHandModbus(side=args.side, ip=ip, port=args.port,
                                     device_id=args.device_id)
    except Exception as exc:
        check("建立 Modbus 连接", False, str(exc))
        return 1
    check("建立 Modbus 连接", True)

    try:
        angles = hand.read_angles()
    except Exception as exc:
        check(f"读 ANGLE_ACT (寄存器 {ihc.REGISTER_ANGLE_ACT})", False,
              f"{exc}\n    寄存器地址可能与 FTP 型号不同，"
              f"用 mhandpro/tools/inspire_modbus_probe.py 扫一下")
        return 1
    check(f"读 ANGLE_ACT (寄存器 {ihc.REGISTER_ANGLE_ACT})", True)

    print()
    for name, value in zip(hand_mapping.CHANNEL_NAMES, angles):
        bar = "█" * int(round(value / 1000 * 24))
        print(f"    {name:<6} {value:>5}  {bar}")

    sane = all(ihc.ANGLE_CLOSED <= v <= ihc.ANGLE_OPEN for v in angles)
    print()
    check("读数在 0~1000 量程内", sane,
          "" if sane else "超出量程说明寄存器读错了位置")

    # 手张开时读数应接近 1000；这条只提示不判定，因为手可能本来就攥着。
    if sane:
        average = sum(angles) / len(angles)
        if average > 800:
            print("[*] 读数接近 1000，手当前是张开的 —— 正常起始状态")
        elif average < 200:
            print("[*] 读数接近 0，手当前是握紧的。开工前建议先让它张开，"
                  "遥操启动时会从当前位置渐变过去")
        else:
            print("[*] 手当前是半握状态")

    print(f"\n检查通过。遥操会把行程限制在 "
          f"{hand_mapping.REAL_HAND_MAX_RANGE_SCALE:.0%}，"
          f"每帧最多变化 {ihc.DEFAULT_MAX_STEP} counts。")
    print("下一步：起上位机，加 --real-hand " + args.side + "\n")
    hand._client.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
