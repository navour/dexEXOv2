#!/usr/bin/env python3
"""G1 真机 IO：unitree_sdk2py 的 lowstate / lowcmd / 遥控器封装。

对应上游 HumDex deploy_real/robot_control/g1_wrapper.py 的语义，但那份依赖
HumDex 自建的 pybind 模块 unitree_interface（未随代码发布），这里改用
unitree_sdk2py —— 与 机器人端/robot_arm_receiver.py 同一套依赖。

unitree_sdk2py 只在 connect() 时导入，因此没装 SDK 的开发机也能 import
本模块跑单元测试。
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Optional

import numpy as np

from . import real_config as cfg


@dataclass(frozen=True)
class LowStateSnapshot:
    """一帧机器人状态，附带本地接收时刻用于断流判定。"""

    dof_pos: np.ndarray
    dof_vel: np.ndarray
    tau_est: np.ndarray
    quaternion: np.ndarray      # wxyz
    gyroscope: np.ndarray       # 骨盆局部角速度 rad/s
    accelerometer: np.ndarray
    temperature: np.ndarray
    mode_machine: int
    received_monotonic: float


class G1RealIO:
    """rt/lowstate 订阅 + rt/lowcmd 发布 + rt/wirelesscontroller 订阅。"""

    def __init__(self, net: Optional[str] = None, domain: int = 0) -> None:
        self.net = net
        self.domain = domain
        self._lock = threading.Lock()
        self._state: Optional[LowStateSnapshot] = None
        self._mode_machine = 0
        self._state_count = 0
        self._wireless_keys = 0
        self._connected = False

        self._crc = None
        self._low_cmd = None
        self._publisher = None

    # ---------- 连接 ----------

    def connect(self) -> None:
        try:
            from unitree_sdk2py.core.channel import (
                ChannelFactoryInitialize,
                ChannelPublisher,
                ChannelSubscriber,
            )
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_
            from unitree_sdk2py.utils.crc import CRC
        except ImportError as exc:
            raise RuntimeError(
                "缺少 unitree_sdk2py。请在机器人机载电脑上安装宇树官方 Python SDK：\n"
                "  https://github.com/unitreerobotics/unitree_sdk2py"
            ) from exc

        if self.net:
            ChannelFactoryInitialize(self.domain, self.net)
        else:
            ChannelFactoryInitialize(self.domain)

        self._crc = CRC()
        self._low_cmd = unitree_hg_msg_dds__LowCmd_()

        self._publisher = ChannelPublisher(cfg.LOWCMD_TOPIC, LowCmd_)
        self._publisher.Init()

        self._state_subscriber = ChannelSubscriber(cfg.LOWSTATE_TOPIC, LowState_)
        self._state_subscriber.Init(self._on_low_state, 10)

        self._wireless_subscriber = ChannelSubscriber(
            cfg.WIRELESS_TOPIC, WirelessController_
        )
        self._wireless_subscriber.Init(self._on_wireless, 10)
        self._connected = True

    def wait_for_state(self, timeout: float = 10.0) -> LowStateSnapshot:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self.read_state()
            if state is not None:
                return state
            time.sleep(0.02)
        raise RuntimeError(
            "超时未收到 rt/lowstate。请检查：DDS 网卡是否正确、"
            "机器人是否已进入低层控制模式、官方运控是否已关闭。"
        )

    # ---------- 回调 ----------

    def _on_low_state(self, msg) -> None:
        dof_pos = np.empty(cfg.NUM_MOTORS, dtype=np.float64)
        dof_vel = np.empty(cfg.NUM_MOTORS, dtype=np.float64)
        tau_est = np.empty(cfg.NUM_MOTORS, dtype=np.float64)
        temperature = np.zeros(cfg.NUM_MOTORS, dtype=np.float64)
        for i in range(cfg.NUM_MOTORS):
            motor = msg.motor_state[i]
            dof_pos[i] = motor.q
            dof_vel[i] = motor.dq
            tau_est[i] = motor.tau_est
            # temperature 是两元组 (线圈, 外壳)，取较高的一个做监视。
            try:
                temperature[i] = float(max(motor.temperature))
            except TypeError:
                temperature[i] = float(motor.temperature)

        imu = msg.imu_state
        snapshot = LowStateSnapshot(
            dof_pos=dof_pos,
            dof_vel=dof_vel,
            tau_est=tau_est,
            quaternion=np.asarray(imu.quaternion, dtype=np.float64),
            gyroscope=np.asarray(imu.gyroscope, dtype=np.float64),
            accelerometer=np.asarray(imu.accelerometer, dtype=np.float64),
            temperature=temperature,
            mode_machine=int(msg.mode_machine),
            received_monotonic=time.monotonic(),
        )
        with self._lock:
            self._state = snapshot
            self._mode_machine = snapshot.mode_machine
            self._state_count += 1

    def _on_wireless(self, msg) -> None:
        with self._lock:
            self._wireless_keys = int(msg.keys)

    # ---------- 读 ----------

    def read_state(self) -> Optional[LowStateSnapshot]:
        with self._lock:
            return self._state

    @property
    def state_count(self) -> int:
        with self._lock:
            return self._state_count

    def key_pressed(self, name: str) -> bool:
        """遥控器按键是否按下。名称见 real_config.CONTROLLER_KEYS。"""
        mask = cfg.CONTROLLER_KEYS[name]
        with self._lock:
            return bool(self._wireless_keys & mask)

    # ---------- 写 ----------

    def send_targets(
        self,
        target_dof_pos: np.ndarray,
        *,
        kp_scale: float = 1.0,
        kd_scale: float = 1.0,
    ) -> None:
        """位置模式：下发 q/kp/kd，PD 由电机驱动板执行。"""
        target = np.asarray(target_dof_pos, dtype=np.float64)
        if target.shape != (cfg.NUM_MOTORS,):
            raise ValueError(f"目标维数应为 29，实际 {target.shape}")
        if not np.all(np.isfinite(target)):
            raise ValueError("目标包含 NaN/Inf，拒绝下发")
        self._write(target, cfg.KPS * kp_scale, cfg.KDS * kd_scale)

    def send_damping(self) -> None:
        """阻尼释放态：kp=0，仅保留阻尼。q 取当前实测位置避免任何位置误差。"""
        state = self.read_state()
        target = (
            state.dof_pos.copy()
            if state is not None
            else np.zeros(cfg.NUM_MOTORS, dtype=np.float64)
        )
        self._write(target, np.zeros(cfg.NUM_MOTORS), cfg.DAMPING_KD)

    def _write(self, q: np.ndarray, kp: np.ndarray, kd: np.ndarray) -> None:
        if not self._connected:
            raise RuntimeError("尚未 connect()")
        cmd = self._low_cmd
        cmd.mode_pr = cfg.LOWCMD_MODE_PR
        with self._lock:
            cmd.mode_machine = self._mode_machine
        for i in range(cfg.NUM_MOTORS):
            motor = cmd.motor_cmd[i]
            motor.mode = cfg.MOTOR_MODE_ENABLE
            motor.q = float(q[i])
            motor.dq = 0.0
            motor.tau = 0.0
            motor.kp = float(kp[i])
            motor.kd = float(kd[i])
        cmd.crc = self._crc.Crc(cmd)
        self._publisher.Write(cmd)

    def close(self) -> None:
        self._connected = False
