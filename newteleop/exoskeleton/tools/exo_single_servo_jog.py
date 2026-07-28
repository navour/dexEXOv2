#!/usr/bin/env python3
"""外骨骼单舵机极小点动：低PWM、慢速、自动回原位并关闭扭矩。"""

import argparse
import sys
import time

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler


PROTOCOL_VERSION = 2.0
ADDR_OPERATING_MODE = 11
ADDR_HARDWARE_ERROR = 70
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_PWM = 100
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132


def check(result, error, packet, action: str) -> None:
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


def write2(packet, port, sid, address, value):
    result, error = packet.write2ByteTxRx(port, sid, address, value)
    check(result, error, packet, f"写入地址{address}")


def write4(packet, port, sid, address, value):
    result, error = packet.write4ByteTxRx(port, sid, address, value)
    check(result, error, packet, f"写入地址{address}")


def signed16(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value


def main() -> int:
    parser = argparse.ArgumentParser(description="XL330单舵机极小安全点动")
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--hand", choices=("left", "right"), required=True)
    parser.add_argument("--id", type=int, default=2, help="默认ID2食指")
    parser.add_argument("--delta", type=int, required=True, help="相对位置tick，绝对值不得超过100")
    parser.add_argument(
        "--pwm",
        type=int,
        default=50,
        help="PWM上限，允许1~100；50约5.7%%，100约11.4%%",
    )
    parser.add_argument(
        "--temporary-position-mode",
        action="store_true",
        help="允许将扭矩关闭的舵机临时切到位置模式3，结束时恢复原模式",
    )
    args = parser.parse_args()
    allowed_ids = range(6, 11) if args.hand == "left" else range(1, 6)
    if args.id not in allowed_ids:
        parser.error(f"{args.hand}手与ID {args.id}不匹配")
    if args.delta == 0 or abs(args.delta) > 100:
        parser.error("--delta必须非零且绝对值不超过100")
    if not 1 <= args.pwm <= 100:
        parser.error("--pwm必须在1~100之间")

    port = PortHandler(args.device)
    packet = PacketHandler(PROTOCOL_VERSION)
    if not port.openPort() or not port.setBaudRate(args.baud):
        print("无法打开串口或设置1 Mbps波特率", file=sys.stderr)
        return 2

    torque_enabled = False
    original_mode = None
    mode_switched = False
    restore = {}
    try:
        model, result, error = packet.ping(port, args.id)
        check(result, error, packet, "Ping")
        mode = read1(packet, port, args.id, ADDR_OPERATING_MODE)
        original_mode = mode
        torque = read1(packet, port, args.id, ADDR_TORQUE_ENABLE)
        hw_error = read1(packet, port, args.id, ADDR_HARDWARE_ERROR)
        position_before_switch = read4(packet, port, args.id, ADDR_PRESENT_POSITION)
        print("========== 外骨骼单舵机极小点动 ==========")
        print(f"ID={args.id}, model={model}, mode={mode}, torque={torque}, hw_error=0x{hw_error:02X}")
        print(f"切换前位置={position_before_switch}")
        print(f"PWM上限={args.pwm}（约{args.pwm/885.0*100.0:.1f}%）")
        if torque != 0:
            raise RuntimeError("舵机扭矩已经开启，拒绝接管")
        if hw_error != 0:
            raise RuntimeError("舵机存在硬件错误，拒绝测试")
        if mode != 3:
            if not args.temporary_position_mode:
                raise RuntimeError(
                    f"工作模式为{mode}，不是位置模式3。"
                    "如需安全点动，显式加 --temporary-position-mode"
                )
            confirmation = input(
                f"将暂时把ID {args.id}从模式{mode}切到位置模式3，"
                "测试后自动恢复。输入 MODE 确认: "
            )
            if confirmation != "MODE":
                print("已取消，未修改模式。")
                return 0
            write1(packet, port, args.id, ADDR_OPERATING_MODE, 3)
            mode_switched = True
            mode = read1(packet, port, args.id, ADDR_OPERATING_MODE)
            if mode != 3:
                raise RuntimeError(f"临时切换失败，回读模式={mode}")
            print(f"已临时切到位置模式3；结束时将恢复模式{original_mode}。")

        # XL330 切换工作模式后位置表示可能重置，必须重新取当前值。
        time.sleep(0.15)
        position = read4(packet, port, args.id, ADDR_PRESENT_POSITION)
        target = position + args.delta
        print(f"位置模式当前位置={position}, 小步目标={target}, "
              f"变化={args.delta} tick（约{args.delta*0.088:.2f}度）")
        if not 0 <= target <= 4095:
            raise RuntimeError("目标超出位置模式0~4095范围")

        confirmation = input("确认外骨骼未穿戴、拉绳松弛且可立即断电，输入 JOG 开始: ")
        if confirmation != "JOG":
            print("已取消，未写入。")
            return 0

        restore[ADDR_GOAL_PWM] = read2(packet, port, args.id, ADDR_GOAL_PWM)
        restore[ADDR_PROFILE_ACCELERATION] = read4(packet, port, args.id, ADDR_PROFILE_ACCELERATION)
        restore[ADDR_PROFILE_VELOCITY] = read4(packet, port, args.id, ADDR_PROFILE_VELOCITY)

        # 先把目标设为当前位置，防止开启扭矩瞬间跳到旧目标。
        write4(packet, port, args.id, ADDR_GOAL_POSITION, position)
        goal_readback = read4(packet, port, args.id, ADDR_GOAL_POSITION)
        # 通信与内部位置同步可能有1~2 tick量化差，不应误判为跳动。
        if abs(goal_readback - position) > 2:
            raise RuntimeError(f"保持目标写入失败：期望{position}，回读{goal_readback}")
        if goal_readback != position:
            print(f"保持目标回读存在{goal_readback-position:+d} tick量化差，在容差内。")
        write2(packet, port, args.id, ADDR_GOAL_PWM, args.pwm)
        write4(packet, port, args.id, ADDR_PROFILE_ACCELERATION, 5)
        write4(packet, port, args.id, ADDR_PROFILE_VELOCITY, 20)
        write1(packet, port, args.id, ADDR_TORQUE_ENABLE, 1)
        torque_enabled = True

        # 开扭矩后先只保持原位；如仍跳动，20 ms级关断扭矩并拒绝点动。
        for _ in range(15):
            held_position = read4(packet, port, args.id, ADDR_PRESENT_POSITION)
            if abs(held_position - position) > 20:
                write1(packet, port, args.id, ADDR_TORQUE_ENABLE, 0)
                torque_enabled = False
                raise RuntimeError(
                    f"开扭矩保持阶段发生异常跳动：{position}->{held_position}，已立即关扭矩"
                )
            time.sleep(0.02)

        print("正在执行极小点动...")
        write4(packet, port, args.id, ADDR_GOAL_POSITION, target)
        for _ in range(10):
            actual = read4(packet, port, args.id, ADDR_PRESENT_POSITION)
            current = signed16(read2(packet, port, args.id, ADDR_PRESENT_CURRENT))
            print(f"  position={actual}, current_raw={current}")
            time.sleep(0.1)

        print("正在返回原位置...")
        write4(packet, port, args.id, ADDR_GOAL_POSITION, position)
        time.sleep(1.0)
        final = read4(packet, port, args.id, ADDR_PRESENT_POSITION)
        print(f"返回后位置={final}，原位置={position}")
        return 0
    except Exception as exc:
        print(f"测试失败：{exc}", file=sys.stderr)
        return 1
    finally:
        try:
            if torque_enabled:
                write1(packet, port, args.id, ADDR_TORQUE_ENABLE, 0)
                print("扭矩已关闭。")
            for address, value in restore.items():
                if address == ADDR_GOAL_PWM:
                    write2(packet, port, args.id, address, value)
                else:
                    write4(packet, port, args.id, address, value)
            if mode_switched and original_mode is not None:
                write1(packet, port, args.id, ADDR_OPERATING_MODE, original_mode)
                restored_mode = read1(packet, port, args.id, ADDR_OPERATING_MODE)
                if restored_mode != original_mode:
                    raise RuntimeError(
                        f"原工作模式恢复失败：期望{original_mode}，实际{restored_mode}"
                    )
                print(f"已恢复原工作模式{original_mode}。")
        except Exception as exc:
            print(f"警告：关闭/恢复设置时发生错误，请立即关闭舵机电源：{exc}", file=sys.stderr)
        port.closePort()
        print("串口已关闭。")


if __name__ == "__main__":
    raise SystemExit(main())
