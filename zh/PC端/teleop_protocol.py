#!/usr/bin/env python3
"""PC 与机器人之间的低延迟双臂遥控协议。"""

import struct


PACKET_HEADER_V2 = b"UA2M"
PACKET_FMT_V2 = "!4sBIQ20f"
PACKET_SIZE_V2 = struct.calcsize(PACKET_FMT_V2)
JOINT_COUNT = 10

# 腰部尾块。现有三个消费端都按 data[:PACKET_SIZE_V2] 切片解 UA2M，
# 因此追加在后面的字节会被它们静默忽略，仿真和真机接收端无需改动。
# 自带包头是为了不把随机尾部字节误读成腰部数据；启用标志放在这里而
# 不是 mode 字节里，因为所有消费端都校验 mode ∈ {0,1,2,3}，给 mode
# 加位会让它们直接丢包。
WAIST_HEADER = b"UAWS"
WAIST_FMT = "!4sB3f3f"
WAIST_SIZE = struct.calcsize(WAIST_FMT)
WAIST_AXIS_COUNT = 3

# 灵巧手尾块，跟在腰部尾块后面。载荷是 6 通道的闭合度（0=张开，1=握紧），
# 不是关节角也不是因时的角度寄存器值 —— 理由同手臂发 rad 而不是电机计数：
# 换手套或换手时只有端点标定表要改，协议和消费端不动。
# 通道顺序与 PC端/hand_mapping.py 一致：小指,无名指,中指,食指,拇指弯曲,拇指对掌。
#
# **顺序不能反**：现有三个消费端都在固定偏移 PACKET_SIZE_V2 上读腰块并校验
# magic，腰块必须排在手块前面，没升级的消费端才照常拿得到腰。手块排在后面
# 时它们只会看到多余字节并忽略。
HAND_HEADER = b"UHND"
HAND_FMT = "!4sB12f"
HAND_SIZE = struct.calcsize(HAND_FMT)
HAND_CHANNEL_COUNT = 6
# 启用位放在尾块自己的标志字节里，不动 mode 字节 —— 所有消费端都校验
# mode ∈ {0,1,2,3}，给 mode 加位会让它们整包丢弃。
HAND_FLAG_RIGHT = 0x1
HAND_FLAG_LEFT = 0x2


def pack_arm_command(mode, seq, sender_timestamp_us, positions, velocities,
                     waist=None, waist_velocities=None, waist_enabled=False,
                     right_hand=None, left_hand=None):
    """打包带序号、发送时间、目标角和目标速度的 V2 指令。

    ``waist`` 给出时追加腰部尾块（yaw/roll/pitch，单位 rad）。
    ``waist_enabled`` 为假时消费端应把腰归零，等于随时可退回无腰行为。

    ``right_hand`` / ``left_hand`` 给出时再追加灵巧手尾块，各 6 个闭合度。
    只给一只手时另一只手的标志位不置位，消费端把它当"没有数据"张开处理。
    """
    positions = tuple(float(v) for v in positions)
    velocities = tuple(float(v) for v in velocities)
    if len(positions) != JOINT_COUNT or len(velocities) != JOINT_COUNT:
        raise ValueError("positions 和 velocities 必须各包含10个关节值")
    packet = struct.pack(
        PACKET_FMT_V2,
        PACKET_HEADER_V2,
        int(mode),
        int(seq) & 0xFFFFFFFF,
        int(sender_timestamp_us) & 0xFFFFFFFFFFFFFFFF,
        *(positions + velocities),
    )
    if waist is not None:
        packet += pack_waist_block(waist, waist_velocities, waist_enabled)
    if right_hand is not None or left_hand is not None:
        packet += pack_hand_block(right_hand, left_hand)
    return packet


def pack_waist_block(waist, waist_velocities=None, waist_enabled=False):
    """单独打包腰部尾块，便于测试与复用。"""
    waist = tuple(float(v) for v in waist)
    if waist_velocities is None:
        waist_velocities = (0.0,) * WAIST_AXIS_COUNT
    waist_velocities = tuple(float(v) for v in waist_velocities)
    if (len(waist) != WAIST_AXIS_COUNT
            or len(waist_velocities) != WAIST_AXIS_COUNT):
        raise ValueError("腰部数据必须包含 yaw/roll/pitch 三个值")
    return struct.pack(
        WAIST_FMT,
        WAIST_HEADER,
        1 if waist_enabled else 0,
        *(waist + waist_velocities),
    )


def pack_hand_block(right_hand=None, left_hand=None):
    """单独打包灵巧手尾块，便于测试与复用。

    每只手 6 个闭合度，取值 0.0（完全张开）~1.0（按当前行程比例握到底）。
    ``None`` 表示这只手没有数据，对应标志位不置位、载荷填 0（张开），
    这样即使消费端忽略标志位，退化行为也是张手而不是握拳。
    """
    flags = 0
    if right_hand is not None:
        flags |= HAND_FLAG_RIGHT
    if left_hand is not None:
        flags |= HAND_FLAG_LEFT

    def _channels(values, name):
        if values is None:
            return (0.0,) * HAND_CHANNEL_COUNT
        values = tuple(float(v) for v in values)
        if len(values) != HAND_CHANNEL_COUNT:
            raise ValueError(f"{name} 必须包含 {HAND_CHANNEL_COUNT} 个通道")
        # 越界不抛异常也不外推，直接夹住 —— 手部数据坏掉不该让整包丢弃，
        # 那会连带手臂一起超时回零。
        return tuple(min(1.0, max(0.0, v)) for v in values)

    return struct.pack(
        HAND_FMT,
        HAND_HEADER,
        flags,
        *(_channels(right_hand, "right_hand")
          + _channels(left_hand, "left_hand")),
    )
