#!/usr/bin/env python3
"""因时 FTP 灵巧手的真机输出：闭合度 → DDS ``inspire_hand_ctrl``。

链路（宇树官方文档 G1_developer/inspire_ftp_dexterity_hand）::

    手套 → 闭合度 0~1
        ↓ 本模块
    DDS rt/inspire_hand/ctrl/{l,r}   inspire_hand_ctrl.angle_set = 6×int16
        ↓ inspire_hand_sdk 的 Headless_driver
    因时手 (左 192.168.123.210 / 右 192.168.123.211)

``angle_set`` 是 0~1000 的**全量程**，0=完全握紧、1000=完全张开 —— 方向与
手套一致（越大越张开），通道顺序也与 hand_mapping 一致（小指…拇指对掌），
所以这里不需要重排，也不需要逐通道实测端点（那是走裸 Modbus 寄存器时才要
的，见 mhandpro/config/inspire_left.cfg）。

DDS 发布被隔离在 ``InspireHandPublisher`` 里、依赖延迟导入，所以换算逻辑在
没有 unitree_sdk2py / inspire_sdkpy 的机器上也能单测 —— 与 waist_follow.py
是同一个理由：这段决定真手会不会握死，必须能在 PC 上测。
"""

from __future__ import annotations

import hand_mapping


# 官方 DDS 话题。左右手是两个独立话题，不是一个话题里的 12 个通道
# （那是 DFX 型号的做法，FTP 不是）。
TOPIC_RIGHT = "rt/inspire_hand/ctrl/r"
TOPIC_LEFT = "rt/inspire_hand/ctrl/l"

# angle_set 的全量程。0=完全握紧，1000=完全张开。
ANGLE_CLOSED = 0
ANGLE_OPEN = 1000
# 与 hand_mapping 共用一套换算，端点就是全量程。
_FULL_OPEN = (ANGLE_OPEN,) * hand_mapping.CHANNEL_COUNT
_FULL_CLOSED = (ANGLE_CLOSED,) * hand_mapping.CHANNEL_COUNT

# mode 的 bit0 = 按角度控制，与 xr_teleoperate 的 robot_hand_inspire.py 一致。
MODE_ANGLE = 0b0001

# 每帧允许的最大变化量（counts）。100Hz 下 60 counts/帧 ≈ 1.7 秒走完全程，
# 手指不会因为一帧跳变而猛地夹上去。真手第一次跑建议再调小。
DEFAULT_MAX_STEP = 60


def topic_for(side):
    if side == "right":
        return TOPIC_RIGHT
    if side == "left":
        return TOPIC_LEFT
    raise ValueError("side 只能是 'right' 或 'left'")


def closure_to_angle_set(closure, range_scale=None):
    """闭合度 → ``angle_set`` 的 6 个 int16。

    ``closure`` 为 None 表示这一帧没有手部数据，返回完全张开 —— 与整条链
    其它环节一致：没数据时张手是安全的，保持上一个握持姿态不是。

    ``range_scale`` 默认走 hand_mapping 的真手硬上限（0.30），而且那个函数
    自己会再夹一次，调用方传 1.0 也绕不过去。
    """
    if closure is None:
        return list(_FULL_OPEN)
    if range_scale is None:
        range_scale = hand_mapping.REAL_HAND_MAX_RANGE_SCALE
    return list(hand_mapping.closure_to_counts(
        closure, _FULL_OPEN, _FULL_CLOSED, range_scale=range_scale))


def step_toward_angle_set(current, target, max_step=DEFAULT_MAX_STEP):
    """把 ``current`` 朝 ``target`` 推进一帧，逐通道限速。

    限速放在这里而不是靠上游平滑：上游断流时下游必须仍然是渐变的，
    否则"没数据→张开"会变成一次瞬间弹开。
    """
    if len(current) != hand_mapping.CHANNEL_COUNT:
        raise ValueError("current 必须是 6 个通道")
    if len(target) != hand_mapping.CHANNEL_COUNT:
        raise ValueError("target 必须是 6 个通道")
    max_step = max(1, int(max_step))

    stepped = []
    for now, goal in zip(current, target):
        now = int(now)
        goal = int(goal)
        diff = goal - now
        if abs(diff) <= max_step:
            stepped.append(goal)
        else:
            stepped.append(now + (max_step if diff > 0 else -max_step))
    return [max(ANGLE_CLOSED, min(ANGLE_OPEN, v)) for v in stepped]


# 因时官方 Modbus 寄存器（与 mhandpro/standalone_inspire_bridge.py 一致，
# 那份在左手上实测跑通过）。DDS 那层是宇树在 Modbus 外面包的一层，直连
# Modbus 少一个驱动进程、少一套 Python 环境，而且**没有功能上的代价** ——
# 触觉也在 Modbus 上（宇树自己的 Headless_driver_r.py 就是用 ModbusDataHandler
# 读触觉阵列再转发到 DDS）。寄存器表见 inspire_hand_ws/inspire_hand_sdk/
# inspire_sdkpy/inspire_hand_defaut.py 的 data_sheet：17 块、3000~5124、
# 1062 个 int16；轻量的抓握力单独在 FORCE_ACT 1582，6 个寄存器。
# 注意 data_sheet 的 length 列是**字节数**不是寄存器数。
REGISTER_ANGLE_SET = 1486
REGISTER_ANGLE_ACT = 1546
DEFAULT_MODBUS_PORT = 6000
DEFAULT_DEVICE_ID = 1
# 官方文档给的默认 IP：左手 210，右手 211。
HAND_IP = {"left": "192.168.123.210", "right": "192.168.123.211"}


class InspireHandModbus:
    """直连因时手的 Modbus TCP 输出，不经过 DDS 和官方驱动进程。

    ``pymodbus`` 是纯 Python，任何版本的解释器都装得上 —— 而 DDS 那条路要
    ``cyclonedds``，它需要编译且对 Python 版本挑剔。所以这条是环境阻力最小
    的路径。

    注意 pymodbus 3.9 起 ``slave=`` 改名为 ``device_id=``；仓库里更早写的
    ``standalone_inspire_bridge.py`` 和 ``tools/inspire_modbus_probe.py``
    还在用旧名，在新版 pymodbus 上会抛 TypeError。
    """

    def __init__(self, side="right", ip=None, port=DEFAULT_MODBUS_PORT,
                 device_id=DEFAULT_DEVICE_ID, max_step=DEFAULT_MAX_STEP,
                 range_scale=None, timeout=1.0):
        from pymodbus.client import ModbusTcpClient

        if side not in HAND_IP:
            raise ValueError("side 只能是 'right' 或 'left'")
        self.side = side
        self.ip = ip or HAND_IP[side]
        self.port = port
        self.device_id = device_id
        self.max_step = max_step
        self.range_scale = range_scale
        self._client = ModbusTcpClient(self.ip, port=port, timeout=timeout)
        if not self._client.connect():
            raise ConnectionError(f"连不上因时手 {self.ip}:{port}")
        # pymodbus 3.14 的 connect() 对着不存在的地址也返回 True（实测），
        # 所以必须真读一次才算连上 —— 否则失败会推迟到第一次下发动作时，
        # 那时候手已经在动了，症状比"起不来"难查得多。
        try:
            self.read_angles()
        except Exception as exc:
            self._client.close()
            raise ConnectionError(
                f"连上了 {self.ip}:{port} 但读不到寄存器: {exc}") from exc
        # 起始姿态是张开，不是零 —— 零在这个量程里是"握死"。
        self.current = list(_FULL_OPEN)

    def read_angles(self):
        """读实际角度，用于开工前确认通信正常。只读，不下发任何动作。"""
        try:
            response = self._client.read_holding_registers(
                REGISTER_ANGLE_ACT, count=hand_mapping.CHANNEL_COUNT,
                device_id=self.device_id)
        except Exception as exc:
            # pymodbus 对超时抛 ModbusIOException 而不是返回错误响应，
            # 两种失败在这里统一成 IOError, 调用方只需要处理一种。
            raise IOError(f"读 ANGLE_ACT 失败: {exc}") from exc
        if response.isError():
            raise IOError(f"读 ANGLE_ACT 失败: {response}")
        return list(response.registers)

    def send(self, closure):
        """发一帧。``closure`` 为 None 时朝张开推进。返回本帧实际下发的值。"""
        target = closure_to_angle_set(closure, self.range_scale)
        self.current = step_toward_angle_set(
            self.current, target, self.max_step)
        try:
            response = self._client.write_registers(
                REGISTER_ANGLE_SET, self.current, device_id=self.device_id)
        except Exception as exc:
            raise IOError(f"写 ANGLE_SET 失败: {exc}") from exc
        if response.isError():
            raise IOError(f"写 ANGLE_SET 失败: {response}")
        return list(self.current)

    def open_and_close(self):
        """退出前把手张开再断开，别让它攥着东西停在那儿。"""
        try:
            for _ in range(int(ANGLE_OPEN / max(1, self.max_step)) + 2):
                if self.current == list(_FULL_OPEN):
                    break
                self.send(None)
        finally:
            self._client.close()


class InspireHandPublisher:
    """把 angle_set 发到 DDS。依赖延迟导入，没装 SDK 时构造会抛 ImportError。

    需要先在同网段起官方驱动进程，否则话题没人消费、手不会动::

        python inspire_hand_sdk/example/Headless_driver_r.py
    """

    def __init__(self, side="right", max_step=DEFAULT_MAX_STEP,
                 range_scale=None):
        from unitree_sdk2py.core.channel import ChannelPublisher
        from inspire_sdkpy import inspire_dds
        # 上游包名就是 defaut（少个 l），不是笔误。
        import inspire_sdkpy.inspire_hand_defaut as inspire_hand_defaut

        self.side = side
        self.topic = topic_for(side)
        self.max_step = max_step
        self.range_scale = range_scale
        self._make_msg = inspire_hand_defaut.get_inspire_hand_ctrl
        self._publisher = ChannelPublisher(self.topic,
                                           inspire_dds.inspire_hand_ctrl)
        self._publisher.Init()
        # 起始姿态是张开，不是零 —— 零在这个量程里是"握死"。
        self.current = list(_FULL_OPEN)

    def send(self, closure):
        """发一帧。``closure`` 为 None 时朝张开推进。返回本帧实际下发的值。"""
        target = closure_to_angle_set(closure, self.range_scale)
        self.current = step_toward_angle_set(
            self.current, target, self.max_step)
        message = self._make_msg()
        message.angle_set = self.current
        message.mode = MODE_ANGLE
        self._publisher.Write(message)
        return list(self.current)

    def open_and_close(self):
        """退出前把手张开再收摊，别让它攥着东西停在那儿。"""
        for _ in range(int(ANGLE_OPEN / max(1, self.max_step)) + 2):
            if self.current == list(_FULL_OPEN):
                break
            self.send(None)
