#!/usr/bin/env python3
"""板载机上的因时灵巧手开工前检查。**只读，绝不下发任何动作。**

到现场第一件事在**机器人上**跑这个::

    cd ~/teleop && python3 hand_check.py [--side right|left] [--ip A.B.C.D]

它把"手能不能通"从"遥操能不能跑"里单独摘出来 —— 链路不通的时候, 你需要
知道是网络、IP、寄存器, 还是我们的代码。

PC端 那份 inspire_hand_check.py 在 PC 用网线直连手时才能用: 它要
pymodbus, 而且要 PC 自己在 192.168.123.x 上。PC 走 WiFi 时手只有板载机
够得着, 所以检查也必须在板载机上做 —— 这份只依赖标准库。
"""

import argparse
import socket
import subprocess
import sys

import hand_driver as hd
import hand_mapping


def check(label, ok, detail=""):
    mark = "[+]" if ok else "[!]"
    print(f"{mark} {label}" + (f"  {detail}" if detail else ""))
    return ok


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("right", "left"), default="right")
    parser.add_argument("--ip", default=None, help="默认按左右手取官方地址")
    parser.add_argument("--port", type=int, default=hd.DEFAULT_MODBUS_PORT)
    parser.add_argument("--device-id", type=int, default=hd.DEFAULT_DEVICE_ID)
    args = parser.parse_args(argv)

    ip = args.ip or hd.HAND_IP[args.side]
    print(f"\n=== 因时灵巧手检查（{args.side}手 {ip}:{args.port}）===")
    print("本工具只读寄存器，不会让手动。\n")

    ok_ping = subprocess.run(
        ["ping", "-c", "2", "-W", "2", ip],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
    check("网络可达", ok_ping,
          "" if ok_ping else f"ping 不通 {ip}。检查手到机器人的网线、"
                             f"手是否上电、IP 是否被改过")
    if not ok_ping:
        print("\n先解决网络，后面的检查没有意义。\n")
        return 1

    sock = socket.socket()
    sock.settimeout(2.0)
    ok_port = sock.connect_ex((ip, args.port)) == 0
    sock.close()
    check(f"TCP {args.port} 端口开放", ok_port,
          "" if ok_port else "手上电了吗？端口是否被改成别的值？")
    if not ok_port:
        return 1

    hand = hd.ModbusTcpHand(ip, port=args.port, device_id=args.device_id,
                            timeout=2.0)
    try:
        hand.connect()
    except Exception as exc:
        check("建立 Modbus 连接并读通寄存器", False,
              f"{exc}\n    端口通但读不到寄存器: 寄存器地址可能与 FTP 型号"
              f"不同, 或者这个 IP 上是别的设备")
        return 1
    check("建立 Modbus 连接并读通寄存器", True)

    try:
        angles = hand.read_angles()
    except Exception as exc:
        check(f"读 ANGLE_ACT (寄存器 {hd.REGISTER_ANGLE_ACT})", False, str(exc))
        hand.close()
        return 1
    check(f"读 ANGLE_ACT (寄存器 {hd.REGISTER_ANGLE_ACT})", True)

    print()
    for name, value in zip(hand_mapping.CHANNEL_NAMES, angles):
        bar = "#" * int(round(max(0, min(1000, value)) / 1000 * 24))
        print(f"    {name:<6} {value:>5}  {bar}")

    sane = all(hd.COUNT_CLOSED <= v <= hd.COUNT_OPEN for v in angles)
    print()
    check("读数在 0~1000 量程内", sane,
          "" if sane else "超出量程说明寄存器读错了位置")

    if sane:
        average = sum(angles) / len(angles)
        if average > 800:
            print("[*] 读数接近 1000，手当前是张开的 —— 正常起始状态")
        elif average < 200:
            print("[*] 读数接近 0，手当前是握紧的。遥操启动时会从当前位置"
                  "渐变张开")
        else:
            print("[*] 手当前是半握状态")

    hand.close()
    print(f"\n检查通过。遥操会把行程限制在 "
          f"{hand_mapping.REAL_HAND_MAX_RANGE_SCALE:.0%}，"
          f"每帧最多变化 {hd.DEFAULT_MAX_STEP} counts，"
          f"断流 {hd.DEFAULT_TIMEOUT_SEC}s 自动张开。")
    print(f"下一步：起接收端，加 --hand {args.side}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
