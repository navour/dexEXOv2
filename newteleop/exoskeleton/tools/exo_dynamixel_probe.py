#!/usr/bin/env python3
"""外骨骼Dynamixel只读冒烟检查：仅Ping并读取状态，不写寄存器、不启用扭矩。"""

import argparse
import sys

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler


PROTOCOL_VERSION = 2.0
HAND_IDS = {"right": [1, 2, 3, 4, 5], "left": [6, 7, 8, 9, 10]}

# XL330/X系列Control Table只读状态地址。
ADDR_HARDWARE_ERROR = 70
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_INPUT_VOLTAGE = 144
ADDR_PRESENT_TEMPERATURE = 146


def signed16(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value


def signed32(value: int) -> int:
    return value - 0x100000000 if value & 0x80000000 else value


def read_value(packet, port, servo_id: int, address: int, size: int):
    if size == 1:
        value, result, error = packet.read1ByteTxRx(port, servo_id, address)
    elif size == 2:
        value, result, error = packet.read2ByteTxRx(port, servo_id, address)
    elif size == 4:
        value, result, error = packet.read4ByteTxRx(port, servo_id, address)
    else:
        raise ValueError("不支持的读取长度")
    if result != COMM_SUCCESS:
        return None, packet.getTxRxResult(result)
    if error:
        return None, packet.getRxPacketError(error)
    return value, None


def main() -> int:
    parser = argparse.ArgumentParser(description="XL330外骨骼只读Ping与状态检查")
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--hand", choices=("left", "right", "all"), default="left",
                        help="默认检查左手ID 6~10")
    parser.add_argument("--ids", help="高级覆盖：逗号分隔的ID")
    args = parser.parse_args()
    if args.ids:
        try:
            servo_ids = [int(item.strip()) for item in args.ids.split(",")]
        except ValueError:
            parser.error("--ids必须是逗号分隔的整数")
    elif args.hand == "all":
        servo_ids = HAND_IDS["right"] + HAND_IDS["left"]
    else:
        servo_ids = HAND_IDS[args.hand]

    print("========== 外骨骼Dynamixel只读检查 ==========")
    print(f"串口: {args.device}，波特率: {args.baud}，协议: {PROTOCOL_VERSION}")
    print(f"选择手别: {args.hand}，目标ID: {servo_ids}")
    print("安全声明：本程序不写寄存器、不修改模式、不启用扭矩。")

    port = PortHandler(args.device)
    packet = PacketHandler(PROTOCOL_VERSION)
    if not port.openPort():
        print(f"错误：无法打开串口 {args.device}", file=sys.stderr)
        return 2
    try:
        if not port.setBaudRate(args.baud):
            print(f"错误：无法设置波特率 {args.baud}", file=sys.stderr)
            return 3

        found = 0
        for servo_id in servo_ids:
            model, result, error = packet.ping(port, servo_id)
            if result != COMM_SUCCESS:
                print(f"ID {servo_id}: Ping失败 — {packet.getTxRxResult(result)}")
                continue
            if error:
                print(f"ID {servo_id}: Ping返回设备错误 — {packet.getRxPacketError(error)}")
                continue
            found += 1
            position, pos_err = read_value(packet, port, servo_id, ADDR_PRESENT_POSITION, 4)
            current, cur_err = read_value(packet, port, servo_id, ADDR_PRESENT_CURRENT, 2)
            voltage, volt_err = read_value(packet, port, servo_id, ADDR_PRESENT_INPUT_VOLTAGE, 2)
            temperature, temp_err = read_value(packet, port, servo_id, ADDR_PRESENT_TEMPERATURE, 1)
            hw_error, hw_err = read_value(packet, port, servo_id, ADDR_HARDWARE_ERROR, 1)
            fields = [f"model={model}"]
            fields.append(f"position={signed32(position)}" if pos_err is None else f"position错误={pos_err}")
            fields.append(f"current_raw={signed16(current)}" if cur_err is None else f"current错误={cur_err}")
            fields.append(f"voltage={voltage/10.0:.1f}V" if volt_err is None else f"voltage错误={volt_err}")
            fields.append(f"temperature={temperature}C" if temp_err is None else f"temperature错误={temp_err}")
            fields.append(f"hw_error=0x{hw_error:02X}" if hw_err is None else f"hw_error读取错误={hw_err}")
            print(f"ID {servo_id}: Ping成功，" + "，".join(fields))

        print(f"检查完成：发现 {found}/{len(servo_ids)} 个舵机。")
        return 0 if found == len(servo_ids) else 1
    finally:
        port.closePort()
        print("串口已关闭。")


if __name__ == "__main__":
    raise SystemExit(main())
