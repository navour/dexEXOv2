#!/usr/bin/env python3
"""因时灵巧手 Modbus TCP 只读探测工具。

本程序只调用 read_holding_registers，不会写任何寄存器，不会下发运动命令。
"""

import argparse
import struct
import pathlib
import sys

from pymodbus.client import ModbusTcpClient

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import pymodbus_compat  # noqa: E402  兼容 pymodbus 新旧版的 slave/device_id


REGISTERS = (
    ("实际位置 POS_ACT", 1534, 6, "signed"),
    ("实际角度 ANGLE_ACT", 1546, 6, "signed"),
    ("实际力 FORCE_ACT", 1582, 6, "signed"),
    ("电机电流 CURRENT", 1594, 6, "signed"),
    ("错误码 ERROR", 1606, 3, "bytes"),
    ("状态 STATUS", 1612, 3, "bytes"),
    ("温度 TEMPERATURE", 1618, 3, "bytes"),
)


def signed_words(registers: list[int]) -> list[int]:
    packed = struct.pack(">" + "H" * len(registers), *registers)
    return list(struct.unpack(">" + "h" * len(registers), packed))


def split_bytes(registers: list[int]) -> list[int]:
    result: list[int] = []
    for value in registers:
        result.extend(((value >> 8) & 0xFF, value & 0xFF))
    return result


def read_block(client: ModbusTcpClient, address: int, count: int, device_id: int):
    # pymodbus 3.6.x 使用 slave 传入 Modbus Unit ID。
    response = pymodbus_compat.read_holding(client, address, count, device_id)
    if response.isError():
        raise RuntimeError(f"读取寄存器 {address} 失败: {response}")
    if not hasattr(response, "registers") or len(response.registers) != count:
        raise RuntimeError(f"寄存器 {address} 返回长度异常")
    return list(response.registers)


def main() -> int:
    parser = argparse.ArgumentParser(description="只读检查因时灵巧手 Modbus TCP 寄存器")
    parser.add_argument("--ip", default="192.168.123.210", help="灵巧手IP，左手默认.210")
    parser.add_argument("--port", type=int, default=6000, help="Modbus TCP端口")
    parser.add_argument("--device-id", type=int, default=1, help="Modbus Unit ID")
    args = parser.parse_args()

    print("========== 因时左手只读 Modbus 检查 ==========")
    print(f"目标: {args.ip}:{args.port}, Device ID: {args.device_id}")
    print("安全声明: 本程序只读取寄存器，不会发送运动命令。")

    client = ModbusTcpClient(args.ip, port=args.port, timeout=2.0)
    try:
        if not client.connect():
            print("错误: Modbus TCP 连接失败。", file=sys.stderr)
            return 2
        print("Modbus TCP连接成功。")
        for label, address, count, data_type in REGISTERS:
            raw = read_block(client, address, count, args.device_id)
            values = signed_words(raw) if data_type == "signed" else split_bytes(raw)
            print(f"{label:<24} 地址={address}: {values}")
    except Exception as exc:
        print(f"读取失败: {exc}", file=sys.stderr)
        return 1
    finally:
        client.close()

    print("全部只读寄存器读取完成。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
