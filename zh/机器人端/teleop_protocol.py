#!/usr/bin/env python3
"""PC/树莓派与机器人之间的双臂遥控协议解析。"""

import math
import struct


PACKET_HEADER_V2 = b"UA2M"
PACKET_FMT_V2 = "!4sBIQ20f"
PACKET_SIZE_V2 = struct.calcsize(PACKET_FMT_V2)
JOINT_COUNT = 10

# 腰部尾块, 追加在 97 字节 UA2M 之后 (共 126 字节)。自带包头是为了不把
# 随机尾部字节误读成腰部数据; 启用标志放在这里而不是 mode 字节里, 因为
# 所有消费端都校验 mode ∈ {0,1,2,3}, 给 mode 加位会让它们直接丢包。
# 旧版发送端不带尾块, 解析结果 waist 为 None, 行为与加腰之前完全一致。
WAIST_HEADER = b"UAWS"
WAIST_FMT = "!4sB3f3f"
WAIST_SIZE = struct.calcsize(WAIST_FMT)
WAIST_AXIS_COUNT = 3
# 解析阶段的粗筛范围; 真正的关节限位在接收端按 URDF 再夹一次。
WAIST_PARSE_LIMIT = 3.2

# 灵巧手尾块, 跟在腰部尾块后面 (共 179 字节)。载荷是 6 通道闭合度
# (0=张开, 1=按当前行程比例握到底), 不是关节角也不是因时角度寄存器值。
# 通道顺序: 小指,无名指,中指,食指,拇指弯曲,拇指对掌。换算见 hand_mapping。
# 旧发送端不带这个尾块, 解析结果 hands 为 None, 行为与加手之前完全一致。
HAND_HEADER = b"UHND"
HAND_FMT = "!4sB12f"
HAND_SIZE = struct.calcsize(HAND_FMT)
HAND_CHANNEL_COUNT = 6
HAND_FLAG_RIGHT = 0x1
HAND_FLAG_LEFT = 0x2

PACKET_HEADER_LEGACY = b"UARM"
PACKET_FMT_SINGLE = "!4sBffff"
PACKET_FMT_DUAL = "!4sBffffffff"
PACKET_FMT_DUAL_WR = "!4sBffffffffff"
PACKET_SIZE_SINGLE = struct.calcsize(PACKET_FMT_SINGLE)
PACKET_SIZE_DUAL = struct.calcsize(PACKET_FMT_DUAL)
PACKET_SIZE_DUAL_WR = struct.calcsize(PACKET_FMT_DUAL_WR)


def unpack_arm_command(data):
    """解析 V2 或旧版指令，返回统一字典；无效数据返回 None。"""
    if len(data) >= PACKET_SIZE_V2 and data[:4] == PACKET_HEADER_V2:
        values = struct.unpack(PACKET_FMT_V2, data[:PACKET_SIZE_V2])
        result = {
            "version": 2,
            "mode": values[1],
            "seq": values[2],
            "sender_timestamp_us": values[3],
            "positions": tuple(values[4:14]),
            "velocities": tuple(values[14:24]),
            "waist": unpack_waist_block(data[PACKET_SIZE_V2:]),
            "hands": unpack_hand_block(data[PACKET_SIZE_V2:]),
        }
    elif len(data) >= PACKET_SIZE_DUAL_WR:
        values = struct.unpack(
            PACKET_FMT_DUAL_WR, data[:PACKET_SIZE_DUAL_WR])
        if values[0] != PACKET_HEADER_LEGACY:
            return None
        positions = tuple(values[2:12])
        result = None
    elif len(data) >= PACKET_SIZE_DUAL:
        values = struct.unpack(PACKET_FMT_DUAL, data[:PACKET_SIZE_DUAL])
        if values[0] != PACKET_HEADER_LEGACY:
            return None
        positions = (
            values[2], values[3], values[4], values[5], 0.0,
            values[6], values[7], values[8], values[9], 0.0,
        )
        result = None
    elif len(data) >= PACKET_SIZE_SINGLE:
        values = struct.unpack(PACKET_FMT_SINGLE, data[:PACKET_SIZE_SINGLE])
        if values[0] != PACKET_HEADER_LEGACY:
            return None
        positions = (
            values[2], values[3], values[4], values[5], 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0,
        )
        result = None
    else:
        return None

    if result is None:
        result = {
            "version": 1,
            "mode": values[1],
            "seq": None,
            "sender_timestamp_us": None,
            "positions": positions,
            "velocities": (0.0,) * JOINT_COUNT,
            # 旧版包没有腰部和手部数据。
            "waist": None,
            "hands": None,
        }
    if result["mode"] not in (0, 1, 2, 3):
        return None
    if not all(math.isfinite(v) and abs(v) <= 20.0
               for v in result["positions"]):
        return None
    if not all(math.isfinite(v) and abs(v) <= 100.0
               for v in result["velocities"]):
        return None
    return result


def _find_tail_block(tail, header, size):
    """在尾块区里按块走, 找到指定包头的那一块。

    尾块是自带包头、定长、可选的, 所以这里按块前进而不是固定偏移 ——
    发送端将来调整尾块顺序或中间插新块时, 接收端不会因此读错。
    走到不认识的包头就停, 不做搜索: 宁可当作没有, 也不能把随机字节
    当成手部指令。
    """
    known = ((WAIST_HEADER, WAIST_SIZE), (HAND_HEADER, HAND_SIZE))
    offset = 0
    while offset + 4 <= len(tail):
        magic = tail[offset:offset + 4]
        for known_header, known_size in known:
            if magic != known_header:
                continue
            if offset + known_size > len(tail):
                return None
            if magic == header:
                return tail[offset:offset + size]
            offset += known_size
            break
        else:
            return None
    return None


def unpack_waist_block(tail):
    """解析腰部尾块; 缺失、包头不符、未启用或数值离谱时返回 None。

    尾块坏掉绝不能连累双臂 —— 这里只返回 None, 让接收端把腰回中,
    等价于退回加腰之前的行为。
    """
    tail = _find_tail_block(tail, WAIST_HEADER, WAIST_SIZE)
    if tail is None:
        return None
    try:
        values = struct.unpack(WAIST_FMT, tail[:WAIST_SIZE])
    except struct.error:
        return None
    if not int(values[1]):
        return None
    waist = tuple(float(v) for v in values[2:2 + WAIST_AXIS_COUNT])
    if not all(math.isfinite(v) and abs(v) <= WAIST_PARSE_LIMIT
               for v in waist):
        return None
    return waist


def unpack_hand_block(tail):
    """解析灵巧手尾块, 返回 {"right": 6元组或None, "left": ...}; 没有则 None。

    某只手的标志位没置位、通道数值非法时, 这只手返回 None —— 消费端应把
    它当"没有数据"张开处理, 而不是沿用上一帧。手上没数据时张开是安全的,
    握着不放不是。坏掉的手也绝不能连累另一只手和双臂。
    """
    tail = _find_tail_block(tail, HAND_HEADER, HAND_SIZE)
    if tail is None:
        return None
    try:
        values = struct.unpack(HAND_FMT, tail[:HAND_SIZE])
    except struct.error:
        return None

    flags = int(values[1])
    channels = values[2:2 + 2 * HAND_CHANNEL_COUNT]

    def _side(flag, start):
        if not flags & flag:
            return None
        side = tuple(float(v)
                     for v in channels[start:start + HAND_CHANNEL_COUNT])
        # 闭合度天然就是 0~1, 越界说明发送端或链路出了问题, 整只手作废,
        # 不夹紧后照用 —— 夹紧会把一个已知有问题的值当成有效指令。
        if not all(math.isfinite(v) and 0.0 <= v <= 1.0 for v in side):
            return None
        return side

    right = _side(HAND_FLAG_RIGHT, 0)
    left = _side(HAND_FLAG_LEFT, HAND_CHANNEL_COUNT)
    if right is None and left is None:
        return None
    return {"right": right, "left": left}
