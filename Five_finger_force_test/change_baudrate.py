#!/usr/bin/env python3
"""
将所有 XL330 舵机的波特率从 57600 改为 1Mbps
运行一次即可，修改后舵机会记住新波特率

Dynamixel XL330 Baud Rate 寄存器 (地址 8):
  0 → 9600
  1 → 57600
  2 → 115200
  3 → 1000000 (1Mbps)
  4 → 2000000
  5 → 3000000
  6 → 4000000
"""
from dynamixel_sdk import *

DEVICENAME = "/dev/ttyAMA0"
OLD_BAUDRATE = 57600
NEW_BAUDRATE = 1000000
NEW_BAUD_INDEX = 3  # 1Mbps
PROTOCOL_VERSION = 2.0
DXL_IDS = [1, 2, 3, 4, 5]          # 右手（已完成，保留备用）
DXL_IDS_LEFT = [6, 7, 8, 9, 10]    # 左手（ID 已改，波特率待改）

ADDR_TORQUE_ENABLE = 64
ADDR_BAUD_RATE = 8

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    print("无法打开串口")
    exit(1)

# 用旧波特率连接
portHandler.setBaudRate(OLD_BAUDRATE)
print(f"当前波特率: {OLD_BAUDRATE}")

# 左手舵机 ID 已改为 6~10，但波特率仍是出厂 57600，需要统一修改
TARGET_IDS = DXL_IDS_LEFT

for sid in TARGET_IDS:
    model, res, err = packetHandler.ping(portHandler, sid)
    if res != 0:
        print(f"  舵机{sid}: Ping失败 (可能已经是新波特率)")
        continue
    print(f"  舵机{sid}: Ping成功, Model={model}")

    # 关闭扭矩 (修改波特率前必须)
    packetHandler.write1ByteTxRx(portHandler, sid, ADDR_TORQUE_ENABLE, 0)

    # 写入新波特率
    res, err = packetHandler.write1ByteTxRx(portHandler, sid, ADDR_BAUD_RATE, NEW_BAUD_INDEX)
    if res == 0:
        print(f"  舵机{sid}: 波特率已修改为 {NEW_BAUDRATE}")
    else:
        print(f"  舵机{sid}: 修改失败 ({packetHandler.getTxRxResult(res)})")

import time
time.sleep(0.5)

# 用新波特率验证
portHandler.setBaudRate(NEW_BAUDRATE)
print(f"\n切换到新波特率 {NEW_BAUDRATE} 验证:")
for sid in TARGET_IDS:
    model, res, err = packetHandler.ping(portHandler, sid)
    if res == 0:
        print(f"  舵机{sid}: ✓ 验证成功 (Model={model})")
    else:
        print(f"  舵机{sid}: ✗ 验证失败")

portHandler.closePort()
print("\n完成! 现在 finger_force.py 可以用 1Mbps 波特率了。")
