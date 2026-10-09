#!/usr/bin/env python3
"""因时 FTP 灵巧手的通道映射, 纯逻辑, 不碰 Modbus 也不碰 MuJoCo。

数据手套给的是 6 个通道, 因时的角度寄存器收的也是 6 个通道, 而上游
URDF (``g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf``) 里每只手正好有
6 个独立转动关节, 其余全是 ``<mimic>`` 联动或固定的力传感器 —— 所以
这里是一一对应, 不需要做 retargeting。

三种表示各管一段:

    手套原始计数 (0~1000, 越大越张开)
        ↓ glove_to_closure()   按逐通道实测端点归一化
    闭合度 (0.0=完全张开, 1.0=完全握紧)   ← 这个上 UDP, 与设备无关
        ↓ closure_to_angles()  → 仿真/渲染
        ↓ closure_to_counts()  → 真手的角度寄存器

上线的是闭合度而不是原始计数, 和手臂那边发 rad 而不是电机计数是同一个
理由: 换一副手套或换一只手时, 只有端点标定表要改, 协议和消费端不用动。

仿真角度和真手寄存器必须分开限幅:

* URDF/MuJoCo 是无接触力的纯运动学显示，闭合度 1.0 应对应模型完整关节行程，
  否则真人握拳时画面永远只能弯到一小截。
* 真手寄存器继续沿用 ``mhandpro/config/inspire_left.cfg`` 已验证过的 0.30
  硬上限；加负载后仍要从低比例重新爬，调用方传 1.0 也不能绕过。
"""

import math


CHANNEL_COUNT = 6
# 顺序与 mhandpro 的六维输出、因时角度寄存器完全一致, 不要重排。
CHANNEL_NAMES = ("小指", "无名指", "中指", "食指", "拇指弯曲", "拇指对掌")

# 因时角度寄存器的物理范围。0 = 完全握紧, 1000 = 完全张开。
COUNT_MIN = 0
COUNT_MAX = 1000

# 因时 FTP 灵巧手的**机械角度范围**, 来自厂商数据手册 PRJ-02-TS-U-010 第 22
# 页。与 CHANNEL_NAMES 同序, 每项是 (闭合端角度, 张开端角度), 单位度。
#
# 这张表描述的是**真手的物理姿态**, 不是寄存器计数, 也不是 URDF 关节角 ——
# 三者是三套不同的量, 不要混用:
#
#   * 寄存器计数 0~1000: 走 closure_to_counts(), 端点由 cfg 逐通道实测
#   * URDF 关节角:       走 closure_to_angles(), 上限见 JOINT_UPPER
#   * 这张物理角度表:     只用来把闭合度翻译成"真手现在大概是多少度",
#                        便于和人手实际动作对照, 不参与任何下发路径
#
# 四指的 ∠α 是手指与掌骨平面的夹角: 伸直时 176°, 握紧时 20°(角度变小)。
# 拇指弯曲 ∠θ 相反, 越弯数值越大: 张开 -13°, 弯到底 70°。
INSPIRE_ANGLE_RANGE_DEG = (
    (20.0, 176.0),    # 小指   ∠α
    (20.0, 176.0),    # 无名指 ∠α
    (20.0, 176.0),    # 中指   ∠α
    (20.0, 176.0),    # 食指   ∠α
    (70.0, -13.0),    # 拇指弯曲 ∠θ, 注意闭合端是较大的那个数
    (165.0, 90.0),    # 拇指对掌 ∠β —— 方向未在实机上核对, 见下
)
# 拇指对掌 ∠β 的量程 90°~165° 是手册给的, 但手册的图没有明确哪一端是"对掌
# 到底"。这里按"β 越大越靠向掌心"取 165° 为闭合端。**这一条没有在真手上核
# 对过**; 它只影响角度显示, 不影响任何下发的计数或关节角, 所以取反了也不会
# 让手动错。要核实: 用 mhandpro/tools/inspire_jog.py 把该通道推到寄存器 0 和
# 1000 两端, 量一下拇指与掌骨平面的实际夹角。
THUMB_OPP_RANGE_VERIFIED = False

# URDF 里每个通道对应的驱动关节 (右手)。左手把 right_ 换成 left_。
# 限位取自上游 URDF, 下限一律是 0 (伸直), 上限见 JOINT_UPPER。
DRIVER_JOINTS_RIGHT = (
    "right_little_1_joint",
    "right_ring_1_joint",
    "right_middle_1_joint",
    "right_index_1_joint",
    "right_thumb_2_joint",   # 拇指弯曲
    "right_thumb_1_joint",   # 拇指对掌, 没有联动关节
)
# 上限, 单位 rad, 与 DRIVER_JOINTS_RIGHT 同序。
JOINT_UPPER = (1.4381, 1.4381, 1.4381, 1.4381, 0.5864, 1.1641)

# URDF 的 <mimic> 链。MuJoCo 不认 mimic, 转 MJCF 时要写成 equality 约束;
# 渲染和自检则直接用这张表把 6 个驱动关节展开成 12 个。
# 形式: 驱动关节 -> ((被动关节, 相对上一级的倍率), ...) 按链序排列。
MIMIC_CHAINS_RIGHT = {
    "right_little_1_joint": (("right_little_2_joint", 1.0843),),
    "right_ring_1_joint": (("right_ring_2_joint", 1.0843),),
    "right_middle_1_joint": (("right_middle_2_joint", 1.0843),),
    "right_index_1_joint": (("right_index_2_joint", 1.0843),),
    "right_thumb_2_joint": (("right_thumb_3_joint", 0.8024),
                            ("right_thumb_4_joint", 0.9487)),
    "right_thumb_1_joint": (),
}

# 纯运动学仿真允许显示URDF完整行程。
SIM_RANGE_SCALE = 1.0

# 真手行程的**硬上限**。调用方传再大也会被夹到这个值。
#
# 这里曾经是 0.30, 来自 mhandpro/config/inspire_left.cfg。那个数的含义是
# "只在空载下验证到 30%", 不是"超过 30% 会损坏" —— 30% 的行程根本抓不住
# 东西, 遥操没法用。2026-07-29 在真机上确认整条链路可控后放开到整程。
#
# 放开的代价要清楚: 手指能完全闭合, 夹到手是真的会夹; 抓到硬物时角度模式
# 会一直顶到堵转 (因时的角度模式不做力限)。真正兜住安全的是另外三条 ——
# 断流张开、逐帧限速、退出张开 —— 它们与这个值无关, 不要因为这里放开了就
# 去动那三条。
#
# 要保守跑就用 CONSERVATIVE_RANGE_SCALE, 或者接收端加 --hand-range 0.3。
REAL_HAND_MAX_RANGE_SCALE = 1.0
CONSERVATIVE_RANGE_SCALE = 0.30
# 兼容旧调用方；安全含义仅限真实灵巧手，仿真请使用 SIM_RANGE_SCALE。
MAX_RANGE_SCALE = REAL_HAND_MAX_RANGE_SCALE


def _mirror(name):
    if name.startswith("right_"):
        return "left_" + name[len("right_"):]
    if name.startswith("left_"):
        return "right_" + name[len("left_"):]
    return name


def driver_joints(side="right"):
    """按通道顺序返回该侧的 6 个驱动关节名。"""
    if side == "right":
        return DRIVER_JOINTS_RIGHT
    if side == "left":
        return tuple(_mirror(n) for n in DRIVER_JOINTS_RIGHT)
    raise ValueError("side 只能是 'right' 或 'left'")


def mimic_chains(side="right"):
    """按侧返回 mimic 链表。"""
    if side == "right":
        return MIMIC_CHAINS_RIGHT
    if side == "left":
        return {_mirror(k): tuple((_mirror(n), m) for n, m in v)
                for k, v in MIMIC_CHAINS_RIGHT.items()}
    raise ValueError("side 只能是 'right' 或 'left'")


def _clamp(value, low, high):
    return max(low, min(high, value))


def _check_len(values, what):
    values = tuple(values)
    if len(values) != CHANNEL_COUNT:
        raise ValueError(f"{what} 必须是 {CHANNEL_COUNT} 个通道")
    return values


def glove_to_closure(raw_counts, open_counts, closed_counts):
    """手套原始计数 → 闭合度 0~1。

    ``open_counts`` / ``closed_counts`` 是逐通道实测的端点 (不是 0/1000,
    见 inspire_left.cfg 里那两行), 所以必须按通道各归一化各的。

    因时是"值越大越张开", 闭合度反过来, 这个方向反转只在这里做一次。
    端点重合的坏通道返回 0.0 (张开) 而不是抛异常 —— 手部数据坏掉不该
    让整只手锁死在某个姿态上。
    """
    raw_counts = _check_len(raw_counts, "raw_counts")
    open_counts = _check_len(open_counts, "open_counts")
    closed_counts = _check_len(closed_counts, "closed_counts")

    closure = []
    for raw, opened, closed in zip(raw_counts, open_counts, closed_counts):
        span = float(opened) - float(closed)
        if not math.isfinite(span) or abs(span) < 1e-6:
            closure.append(0.0)
            continue
        if not math.isfinite(float(raw)):
            closure.append(0.0)
            continue
        closure.append(_clamp((float(opened) - float(raw)) / span, 0.0, 1.0))
    return tuple(closure)


def closure_to_angles(closure, range_scale=SIM_RANGE_SCALE, side="right"):
    """闭合度 → 6 个驱动关节角 (rad), 返回顺序与通道一致。

    这是纯URDF/仿真换算，不会写真实设备；默认使用模型完整行程，并始终夹在
    ``[0, 1]`` 内，不允许越过URDF关节上限。
    """
    closure = _check_len(closure, "closure")
    scale = _clamp(float(range_scale), 0.0, SIM_RANGE_SCALE)
    angles = []
    for value, upper in zip(closure, JOINT_UPPER):
        value = float(value) if math.isfinite(float(value)) else 0.0
        angles.append(_clamp(value, 0.0, 1.0) * scale * upper)
    return tuple(angles)


def closure_to_counts(closure, open_counts, closed_counts,
                      range_scale=REAL_HAND_MAX_RANGE_SCALE):
    """闭合度 → 因时角度寄存器的 6 个整数, 供真手写入。

    真实设备路径无条件把 ``range_scale`` 夹到
    ``REAL_HAND_MAX_RANGE_SCALE``，调用方绕不过这个上限。输出同时夹在
    ``[COUNT_MIN, COUNT_MAX]``。
    """
    closure = _check_len(closure, "closure")
    open_counts = _check_len(open_counts, "open_counts")
    closed_counts = _check_len(closed_counts, "closed_counts")
    scale = _clamp(float(range_scale), 0.0, REAL_HAND_MAX_RANGE_SCALE)

    counts = []
    for value, opened, closed in zip(closure, open_counts, closed_counts):
        value = float(value) if math.isfinite(float(value)) else 0.0
        value = _clamp(value, 0.0, 1.0) * scale
        count = float(opened) + (float(closed) - float(opened)) * value
        counts.append(int(round(_clamp(count, COUNT_MIN, COUNT_MAX))))
    return tuple(counts)


def closure_to_physical_deg(closure):
    """闭合度 → 真手的物理角度 (度), 按 INSPIRE_ANGLE_RANGE_DEG 线性插值。

    **只用于显示和对照**, 不在任何下发路径上 —— 下发走 closure_to_counts()
    (真手) 或 closure_to_angles() (仿真)。存在的理由是: 说"闭合度 0.62"没法
    和自己的手比, 说"食指现在在 79°"可以。
    """
    closure = _check_len(closure, "closure")
    angles = []
    for value, (closed_deg, open_deg) in zip(closure, INSPIRE_ANGLE_RANGE_DEG):
        value = float(value) if math.isfinite(float(value)) else 0.0
        value = _clamp(value, 0.0, 1.0)
        angles.append(open_deg + (closed_deg - open_deg) * value)
    return tuple(angles)


def physical_deg_to_closure(degrees):
    """物理角度 → 闭合度, closure_to_physical_deg 的逆。

    量程为零的通道退化成 0.0 而不是抛异常: 这条路径只用于显示和标定辅助,
    不该因为一个通道的表项写坏就把整只手弄没。
    """
    degrees = _check_len(degrees, "degrees")
    closure = []
    for value, (closed_deg, open_deg) in zip(degrees, INSPIRE_ANGLE_RANGE_DEG):
        span = closed_deg - open_deg
        if abs(span) < 1e-9:
            closure.append(0.0)
            continue
        value = float(value) if math.isfinite(float(value)) else open_deg
        closure.append(_clamp((value - open_deg) / span, 0.0, 1.0))
    return tuple(closure)


def expand_mimic(angles, side="right"):
    """6 个驱动关节角 → 全部 12 个转动关节角, 键是 URDF 关节名。

    mimic 是逐级相乘的链 (thumb_2 → thumb_3 → thumb_4), 不是都乘驱动关节,
    照 URDF 的写法逐级算。固定的力传感器关节不在结果里。
    """
    angles = _check_len(angles, "angles")
    joints = driver_joints(side)
    chains = mimic_chains(side)

    result = {}
    for name, angle in zip(joints, angles):
        angle = float(angle)
        result[name] = angle
        previous = angle
        for passive, multiplier in chains[name]:
            previous = previous * float(multiplier)
            result[passive] = previous
    return result


def open_closure():
    """完全张开的闭合度, 用作零位和失效回退值。"""
    return (0.0,) * CHANNEL_COUNT
