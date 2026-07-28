#!/usr/bin/env python3
"""外骨骼单舵机低电流短脉冲测试。

只用于未穿戴、拉绳松弛时确认电流正负号与收放绳方向。
不修改工作模式、ID、波特率或电流上限等 EEPROM 参数。
"""

import argparse
import sys
import time

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler


PROTOCOL_VERSION = 2.0
ADDR_OPERATING_MODE = 11
ADDR_CURRENT_LIMIT = 38
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70
ADDR_GOAL_CURRENT = 102
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132


def check(result, error, packet, action):
    if result != COMM_SUCCESS:
        raise RuntimeError(f"{action}: {packet.getTxRxResult(result)}")
    if error:
        raise RuntimeError(f"{action}: {packet.getRxPacketError(error)}")


def read1(packet, port, sid, address):
    value, result, error = packet.read1ByteTxRx(port, sid, address)
    check(result, error, packet, f"读取地址{address}")
    return value


def read2(packet, port, sid, address):
    value, result, error = packet.read2ByteTxRx(port, sid, address)
    check(result, error, packet, f"读取地址{address}")
    return value


def read4(packet, port, sid, address):
    value, result, error = packet.read4ByteTxRx(port, sid, address)
    check(result, error, packet, f"读取地址{address}")
    return value


def write1(packet, port, sid, address, value):
    result, error = packet.write1ByteTxRx(port, sid, address, value)
    check(result, error, packet, f"写入地址{address}")


def write2_signed(packet, port, sid, address, value):
    result, error = packet.write2ByteTxRx(port, sid, address, value & 0xFFFF)
    check(result, error, packet, f"写入地址{address}")


def signed16(value):
    return value - 0x10000 if value & 0x8000 else value


def signed32(value):
    return value - 0x100000000 if value & 0x80000000 else value


def main():
    parser = argparse.ArgumentParser(description="XL330单舵机低电流短脉冲测试")
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--hand", choices=("left", "right"), required=True)
    parser.add_argument("--id", type=int, required=True)
    parser.add_argument("--current", type=int, required=True,
                        help="有符号原始电流命令，绝对值最大30")
    parser.add_argument("--duration", type=float, default=0.30,
                        help="脉冲时长，0.05~0.50秒")
    args = parser.parse_args()

    allowed_ids = range(6, 11) if args.hand == "left" else range(1, 6)
    if args.id not in allowed_ids:
        parser.error(f"{args.hand}手与ID {args.id}不匹配")
    if args.current == 0 or abs(args.current) > 30:
        parser.error("--current必须非零且绝对值不超过30")
    if not 0.05 <= args.duration <= 0.50:
        parser.error("--duration必须在0.05~0.50秒之间")

    port = PortHandler(args.device)
    packet = PacketHandler(PROTOCOL_VERSION)
    if not port.openPort() or not port.setBaudRate(args.baud):
        print("无法打开串口或设置波特率", file=sys.stderr)
        return 2

    torque_enabled = False
    try:
        model, result, error = packet.ping(port, args.id)
        check(result, error, packet, "Ping")
        mode = read1(packet, port, args.id, ADDR_OPERATING_MODE)
        torque = read1(packet, port, args.id, ADDR_TORQUE_ENABLE)
        hw_error = read1(packet, port, args.id, ADDR_HARDWARE_ERROR)
        current_limit = read2(packet, port, args.id, ADDR_CURRENT_LIMIT)
        start = signed32(read4(packet, port, args.id, ADDR_PRESENT_POSITION))
        print("========== 外骨骼低电流短脉冲 ==========")
        print(f"ID={args.id}, model={model}, mode={mode}, torque={torque}, "
              f"hw_error=0x{hw_error:02X}")
        print(f"当前位置={start}, 硬件电流上限={current_limit}, "
              f"本次命令={args.current}, 时长={args.duration:.2f}秒")
        if mode != 0:
            raise RuntimeError(f"工作模式为{mode}，不是电流模式0，拒绝测试")
        if torque != 0:
            raise RuntimeError("舵机扭矩已开启，拒绝接管")
        if hw_error != 0:
            raise RuntimeError("存在硬件错误，拒绝测试")
        if abs(args.current) > current_limit:
            raise RuntimeError("本次电流命令超过硬件上限")

        confirm = input("确认外骨骼未穿戴、拉绳松弛且可立即断电，输入 CURRENT 开始: ")
        if confirm != "CURRENT":
            print("已取消，未开启扭矩。")
            return 0

        write2_signed(packet, port, args.id, ADDR_GOAL_CURRENT, 0)
        write1(packet, port, args.id, ADDR_TORQUE_ENABLE, 1)
        torque_enabled = True
        write2_signed(packet, port, args.id, ADDR_GOAL_CURRENT, args.current)

        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            position = signed32(read4(packet, port, args.id, ADDR_PRESENT_POSITION))
            current = signed16(read2(packet, port, args.id, ADDR_PRESENT_CURRENT))
            displacement = position - start
            print(f"  position={position}, displacement={displacement:+d}, current_raw={current}")
            if abs(displacement) > 40:
                raise RuntimeError("位移超过40 tick，看门狗中止")
            time.sleep(0.02)

        write2_signed(packet, port, args.id, ADDR_GOAL_CURRENT, 0)
        final = signed32(read4(packet, port, args.id, ADDR_PRESENT_POSITION))
        print(f"脉冲完成：{start}->{final}，变化={final-start:+d} tick")
        return 0
    except Exception as exc:
        print(f"测试失败：{exc}", file=sys.stderr)
        return 1
    finally:
        try:
            write2_signed(packet, port, args.id, ADDR_GOAL_CURRENT, 0)
            if torque_enabled:
                write1(packet, port, args.id, ADDR_TORQUE_ENABLE, 0)
                print("目标电流已清零，扭矩已关闭。")
        except Exception as exc:
            print(f"警告：清零/关扭矩失败，请立即断电：{exc}", file=sys.stderr)
        port.closePort()
        print("串口已关闭。")


if __name__ == "__main__":
    raise SystemExit(main())
