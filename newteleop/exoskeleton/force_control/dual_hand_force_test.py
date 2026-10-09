#!/usr/bin/env python3
"""左右手外骨骼力反馈：单进程共享 /dev/serial0，串行调度 ID 1~10。"""

from __future__ import annotations

import argparse
from pathlib import Path
import signal
import sys
import threading
import time
from types import SimpleNamespace

from dynamixel_sdk import PortHandler

from left_hand_force_test import HAND_PROFILES, LeftHandController, five_signs

EXOSKELETON_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(EXOSKELETON_DIR))


def hand_args(common, side):
    profile = HAND_PROFILES[side]
    prefix = "right" if side == "right" else "left"
    return SimpleNamespace(
        hand=side, servo_ids=list(profile["servo_ids"]),
        enable_write=common.enable_write,
        enable_mhandpro=common.enable_mhandpro,
        device=common.device, baudrate=common.baudrate,
        fsr_host=common.fsr_host,
        fsr_port=getattr(common, f"{prefix}_fsr_port"),
        force_host=common.force_host,
        force_port=getattr(common, f"{prefix}_force_port"),
        haptic_host=common.haptic_host,
        haptic_port=getattr(common, f"{prefix}_haptic_port"),
        drive_current_signs=getattr(common, f"{prefix}_drive_current_signs"),
        max_goal_current=common.max_goal_current,
        release_goal_current=common.release_goal_current,
        actual_current_limit=common.actual_current_limit,
        current_slew=common.current_slew,
        force_to_current_gain=common.force_to_current_gain,
        hand_total_current_limit=common.hand_total_current_limit,
        direction_fault_threshold=common.direction_fault_threshold,
        release_timeout=common.release_timeout,
        return_settle_time=common.return_settle_time,
        release_hold_seconds=common.release_hold_seconds,
        fsr_load_threshold=common.fsr_load_threshold,
        fsr_release_threshold=common.fsr_release_threshold,
        fsr_release_hold_seconds=common.fsr_release_hold_seconds,
        fsr_rest_max=common.fsr_rest_max,
        comfort_scale=common.comfort_scale,
        target_max=common.target_max,
        contact_on=common.contact_on,
        contact_off=common.contact_off,
        force_kp=common.force_kp,
        force_step_max=common.force_step_max,
        force_deadzone=common.force_deadzone,
        control_hz=common.control_hz,
    )


class DualHandController:
    def __init__(self, args):
        self.args = args
        self.running = True
        self.port = PortHandler(args.device)
        self.dxl_lock = threading.RLock()
        self.control_started = False
        self.fsr_inputs_started = False
        self.g1_inputs_started = False
        self.hands = {
            side: LeftHandController(hand_args(args, side))
            for side in ("right", "left")
        }
        for hand in self.hands.values():
            hand.port = self.port
            hand.dxl_lock = self.dxl_lock
            for finger in hand.fingers:
                finger.port = self.port

    def open(self):
        self.hands["right"].open(open_port=True)
        self.hands["left"].open(open_port=False)

    def start(self):
        """命令行入口保持原行为：所有输入与控制一次启动。"""
        self.start_fsr_inputs()
        self.start_g1_inputs()
        self.start_control()

    def start_control(self):
        """只启动舵机调度和十指联锁，不自动连网。"""
        if self.control_started:
            return False
        self.control_started = True
        for hand in self.hands.values():
            threading.Thread(target=hand.loop, daemon=True).start()
        threading.Thread(target=self._interlock_loop, daemon=True).start()
        return True

    def start_fsr_inputs(self):
        """启动右/9001、左/9002的FSR输入；可安全重复调用。"""
        if self.fsr_inputs_started:
            return False
        self.fsr_inputs_started = True
        for hand in self.hands.values():
            hand.start_fsr_input()
        return True

    def start_g1_inputs(self):
        """启动9201/9202触觉和可选9301/9302力控覆盖。"""
        if self.g1_inputs_started:
            return False
        self.g1_inputs_started = True
        for hand in self.hands.values():
            hand.start_g1_inputs()
        return True

    def _interlock_loop(self):
        while self.running:
            fault = next((f for hand in self.hands.values() for f in hand.fingers
                          if f.state == "FAULT"), None)
            if fault is not None:
                self.stop_all(f"{fault.finger_name} ID{fault.servo_id}故障，十指联锁停止")
            time.sleep(0.02)

    def init_all(self):
        for side in ("right", "left"):
            print(f"--- {HAND_PROFILES[side]['label']} INIT ---")
            self.hands[side].init_all()
        missing = [
            f"{HAND_PROFILES[side]['label']}{finger.finger_name}(ID{finger.servo_id})"
            for side, hand in self.hands.items()
            for finger in hand.fingers
            if finger.init_pos is None or finger.fsr_rest_raw is None
        ]
        if missing:
            print("[INIT未完成] 以下手指没有有效的位置/FSR基线: "
                  + ", ".join(missing), flush=True)
        else:
            print("[INIT成功] 右手和左手十指均已记录返回点与FSR基线。",
                  flush=True)

    def arm_all(self):
        blockers = []
        for side, hand in self.hands.items():
            label = HAND_PROFILES[side]["label"]
            ready, reason = hand.data_ready()
            if not ready:
                blockers.append(f"{label}数据未就绪: {reason}")
            missing = [
                f"{f.finger_name}(ID{f.servo_id})"
                for f in hand.fingers
                if f.init_pos is None or f.fsr_rest_raw is None
            ]
            if missing:
                blockers.append(f"{label}未INIT: {', '.join(missing)}")
        if blockers:
            reason = "；".join(blockers)
            # Controller.stop() 在本来就是STOP时不打印，所以必须在
            # 这里明确输出，否则交互表现为输入ARM后“没反应”。
            print(f"[ARM拒绝] {reason}。十指保持STOP。", flush=True)
            self.stop_all(f"双手ARM前置不满足: {reason}")
            return False
        for side in ("right", "left"):
            self.hands[side].arm_all()
            if not all(f.armed for f in self.hands[side].fingers):
                label = HAND_PROFILES[side]["label"]
                print(f"[ARM失败] {label}有手指未能ARM，十指联锁STOP。",
                      flush=True)
                self.stop_all(f"{label}ARM失败")
                return False
        right_on = self.hands["right"].args.contact_on
        left_on = self.hands["left"].args.contact_on
        print("[ARM成功] 右手和左手十指已进入FREE待机；"
              f"INSPIRE单指触觉达到contact-on后才会收绳"
              f"(右{right_on:.2f}N/左{left_on:.2f}N)。", flush=True)
        return True

    def stop_all(self, reason):
        for hand in self.hands.values():
            hand.stop_all(reason)

    def status(self):
        for side in ("right", "left"):
            hand = self.hands[side]
            ready, reason = hand.data_ready()
            print(f"--- {HAND_PROFILES[side]['label']} "
                  f"ARM前置={'OK' if ready else reason} ---")
            hand.status()

    def telemetry_snapshot(self):
        """供上位机读取的十指快照；不访问Dynamixel总线。"""
        now = time.monotonic()
        # 不获取dxl_lock：INIT会连续采样十指数秒，如果显示层
        # 等待总线锁，本地GUI会像卡死一样无法立即按STOP。
        # 每指输入已有自身锁，其余都是控制线程更新的纯缓存标量。
        hands = {
            side: self.hands[side].telemetry_snapshot(now)
            for side in ("right", "left")
        }
        fingers = [finger for hand in hands.values()
                   for finger in hand["fingers"]]
        if any(hand["faulted"] for hand in hands.values()):
            state = "FAULT"
        elif all(hand["armed"] for hand in hands.values()):
            state = "ARMED"
        elif any(finger["armed"] for finger in fingers):
            state = "PARTIAL"
        else:
            state = "STOP"
        serial_ready = all(
            finger["servo"]["health_age_ms"] is not None
            for finger in fingers
        )
        return {
            "schema_version": 1,
            "monotonic_ms": round(now * 1000),
            "system": {
                "state": state,
                "write_enabled": self.args.enable_write,
                "mhandpro_enabled": self.args.enable_mhandpro,
                "serial": {
                    "device": self.args.device,
                    "baudrate": self.args.baudrate,
                    "connected": serial_ready,
                    "owner": "dual_hand_force_test",
                },
            },
            "hands": hands,
        }

    def close(self):
        self.running = False
        for hand in self.hands.values():
            hand.running = False
        self.stop_all("双手程序退出")
        self.port.closePort()


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="双手外骨骼力反馈（一个串口进程）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # 写入开关：默认只读，两个开关的作用不同。
    p.add_argument("--enable-write", action="store_true",
                   help="允许写Dynamixel并执行INIT/ARM；不传时只读")
    p.add_argument("--enable-mhandpro", action="store_true",
                   help="向G1 9301/9302发送LOCKED状态和INSPIRE位置PID增量")

    # Dynamixel TTL总线：右手ID1~5、左手ID6~10共用这一个串口。
    p.add_argument("--device", default="/dev/serial0",
                   help="Dynamixel TTL串口设备")
    p.add_argument("--baudrate", type=int, default=1_000_000,
                   help="Dynamixel总线波特率")

    # FSR数据：树莓派上的两个BLE Broker分别广播右/左手五路FSR。
    p.add_argument("--fsr-host", default="127.0.0.1",
                   help="FSR BLE Broker所在主机")
    p.add_argument("--right-fsr-port", type=int, default=9001,
                   help="右手FSR Broker TCP端口")
    p.add_argument("--left-fsr-port", type=int, default=9002,
                   help="左手FSR Broker TCP端口")

    # G1力流：hand_driver.py从INSPIRE top_touch读力后通过这两个端口发回。
    p.add_argument("--force-host", default="192.168.3.78",
                   help="G1板载机INSPIRE力流IP")
    p.add_argument("--right-force-port", type=int, default=9201,
                   help="G1右手INSPIRE触觉端口")
    p.add_argument("--left-force-port", type=int, default=9202,
                   help="G1左手INSPIRE触觉端口")

    # G1力控覆盖：只在--enable-mhandpro时使用。
    p.add_argument("--haptic-host", default="192.168.3.78",
                   help="G1板载机力控覆盖IP")
    p.add_argument("--right-haptic-port", type=int, default=9301,
                   help="G1右手力控覆盖端口")
    p.add_argument("--left-haptic-port", type=int, default=9302,
                   help="G1左手力控覆盖端口")

    # 机械方向：顺序固定为[拇,食,中,无,小]；+1/-1表示哪个电流方向收绳。
    p.add_argument("--right-drive-current-signs", type=five_signs,
                   help="右手五指收绳电流方向，如1,1,1,1,1")
    p.add_argument("--left-drive-current-signs", type=five_signs,
                   help="左手五指收绳电流方向，如1,1,1,1,1")

    # Dynamixel电流参数：XL330-M288(model 1200)实测约1 raw≈1 mA。
    p.add_argument("--max-goal-current", type=int, default=300,
                   help="FORCE_ENTRY收绳Goal Current上限(raw)")
    p.add_argument("--release-goal-current", type=int, default=100,
                   help="RELEASE放绳/归位Goal Current上限(raw)")
    p.add_argument("--actual-current-limit", type=int, default=320,
                   help="Present Current绝对值超过此值立即FAULT(raw)")
    p.add_argument("--current-slew", type=int, default=20,
                   help="FORCE_ENTRY每个控制周期允许增加的最大电流(raw)")
    p.add_argument("--force-to-current-gain", type=float, default=60.0,
                   help="INSPIRE触觉力到收绳电流的比例(raw/N)")
    p.add_argument("--hand-total-current-limit", type=int, default=800,
                   help="每只手五指FORCE_ENTRY收绳总电流上限(raw)")

    # 状态机与安全时序。
    p.add_argument("--direction-fault-threshold", type=int, default=100,
                   help="FORCE_ENTRY反向移动超过此tick时FAULT")
    p.add_argument("--release-timeout", type=float, default=12.0,
                   help="RELEASE未在此时间内返回INIT则FAULT(秒)")
    p.add_argument("--return-settle-time", type=float, default=0.5,
                   help="归位后位置与INSPIRE释放必须连续稳定的时间(秒)")
    p.add_argument("--release-hold-seconds", type=float, default=0.15,
                   help="LOCKED中INSPIRE触觉释放消抖时间(秒)")
    p.add_argument("--fsr-load-threshold", type=float, default=0.30,
                   help="FSR相对INIT新增力达到此值后确认本轮加载(N)")
    p.add_argument("--fsr-release-threshold", type=float, default=0.15,
                   help="已加载FSR降到此值以下时开始卸载计时(N)")
    p.add_argument("--fsr-release-hold-seconds", type=float, default=0.30,
                   help="FSR加载后卸载条件连续成立时间(秒)")

    # 力传感器与接触阈值。
    p.add_argument("--fsr-rest-max", type=float, default=8.0,
                   help="INIT时允许的最大FSR绑带静态预载(N)")
    p.add_argument("--comfort-scale", type=float, default=1.0,
                   help="INSPIRE力进入状态机前的整体缩放系数")
    p.add_argument("--target-max", type=float, default=6.0,
                   help="INSPIRE目标力在软件中的最大值(N)")
    p.add_argument("--contact-on", type=float, default=0.50,
                   help="Inspire接触进入阈值；默认恢复旧程序0.50N")
    p.add_argument("--contact-off", type=float, default=0.30,
                   help="Inspire接触释放阈值，低于contact-on形成滞回")
    p.add_argument("--force-kp", type=float, default=35.0,
                   help="LOCKED力差PID比例增益(tick/N)")
    p.add_argument("--force-step-max", type=float, default=3.0,
                   help="LOCKED PID每周期INSPIRE最大位置修正(tick)")
    p.add_argument("--force-deadzone", type=float, default=0.10,
                   help="LOCKED力误差死区(N)")
    p.add_argument("--control-hz", type=float, default=20.0,
                   help="每只手的状态机和舵机控制频率(Hz)")
    args = p.parse_args(argv)
    if args.enable_write and (args.right_drive_current_signs is None
                              or args.left_drive_current_signs is None):
        p.error("写入模式必须同时显式给出左右手五指电流方向")
    if args.right_drive_current_signs is None:
        args.right_drive_current_signs = [1] * 5
    if args.left_drive_current_signs is None:
        args.left_drive_current_signs = [1] * 5
    if not 1 <= args.max_goal_current <= 300:
        p.error("max-goal-current必须在1..300")
    if not 1 <= args.release_goal_current <= args.max_goal_current:
        p.error("release-goal-current必须在1..max-goal-current")
    if args.actual_current_limit < args.max_goal_current:
        p.error("actual-current-limit不能小于max-goal-current")
    if not 1 <= args.current_slew <= 60:
        p.error("current-slew必须在1..60")
    if args.force_to_current_gain <= 0:
        p.error("force-to-current-gain必须>0")
    if not args.max_goal_current <= args.hand_total_current_limit <= 1500:
        p.error("hand-total-current-limit必须在max-goal-current..1500")
    if args.force_kp <= 0 or not 0.1 <= args.force_step_max <= 20:
        p.error("force-kp必须>0，force-step-max必须在0.1..20 tick")
    if args.force_deadzone < 0:
        p.error("force-deadzone必须>=0")
    if not 40 <= args.direction_fault_threshold <= 500:
        p.error("direction-fault-threshold必须在40..500 tick")
    if not 0.1 <= args.release_hold_seconds <= 5.0:
        p.error("release-hold-seconds必须在0.1..5秒")
    if not (0.0 <= args.fsr_release_threshold
            < args.fsr_load_threshold <= 10.0):
        p.error("FSR卸载阈值必须>=0且小于加载阈值；加载阈值最大10N")
    if not 0.1 <= args.fsr_release_hold_seconds <= 5.0:
        p.error("fsr-release-hold-seconds必须在0.1..5秒")
    return args


def main():
    args = parse_args()
    ctl = DualHandController(args)
    signal.signal(signal.SIGINT, lambda *_: setattr(ctl, "running", False))
    signal.signal(signal.SIGTERM, lambda *_: setattr(ctl, "running", False))
    try:
        ctl.open()
        ctl.start()
        print("双手端口: FSR右9001/左9002, G1触觉右9201/左9202, 覆盖右9301/左9302")
        print("命令: STATUS | INIT | ARM | STOP | QUIT")
        while ctl.running:
            try:
                command = input("> ").strip().upper()
            except EOFError:
                break
            if command == "STATUS": ctl.status()
            elif command == "INIT": ctl.init_all()
            elif command == "ARM": ctl.arm_all()
            elif command == "STOP": ctl.stop_all("人工STOP")
            elif command == "QUIT": break
            elif command: print("未知命令")
    finally:
        ctl.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
