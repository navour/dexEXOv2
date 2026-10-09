#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
g1_jog.py —— 纯 SSH 的 G1 手臂关节点动工具 (调试模式 / lowcmd)

不需要遥控器, 不需要手机, 不需要 IMU, 不需要 PC 端。
在机器人板载机上跑起来, 直接在命令行里给单个关节下角度。

用途: 验证 lowcmd 链路通不通、电机方向对不对、关节限位对不对 ——
      在接 IMU 跑完整遥操之前, 先用它把最基础的东西确认掉。

===========================================================================
 前提条件 (缺一不可)
===========================================================================
 1. 【机器人必须吊起来】本工具走 rt/lowcmd, 没有任何平衡控制。
 2. 【运控服务必须停止】也就是机器人处于调试模式。运控服务和 lowcmd 会
    同时写同一批电机, 一起跑会打架。本工具启动时会检查, 没停就拒绝运行。
 3. 物理急停按钮伸手能按到。
===========================================================================

用法:
    python3 g1_jog.py                 # 交互式, 推荐
    python3 g1_jog.py --iface eth0    # 在板载机以外的机器上跑时要加

交互式命令 (进去以后 `help` 也能看):
    rsp 0.5       右肩俯 (right shoulder pitch) 转到 0.5 rad
    lel 1.0       左肘转到 1.0 rad
    rsp +0.2      在当前目标基础上 +0.2 rad (相对量, 前面带正负号)
    show          打印所有臂关节的 目标值 / 实测值 / 误差
    home          全部臂关节缓慢回零
    speed 0.3     改点动速度 (rad/s), 默认 0.3
    q / exit      回零后安全退出

关节代号:
    左臂  lsp lsr lsy lel lwr      右臂  rsp rsr rsy rel rwr
          |   |   |   |   |
          肩俯 肩滚 肩偏 肘  腕滚

腕俯(20/27) 和腕偏(21/28) 全程锁在 0 —— 和本项目其它部分保持一致。
"""

import argparse
import math
import signal
import sys
import threading
import time

try:
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber)
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC
except ModuleNotFoundError as exc:
    sys.exit(
        f"[!] 找不到 unitree_sdk2py ({exc}).\n"
        f"    非交互式 ssh 不读 ~/.bashrc, 手动补一句:\n"
        f"    export PYTHONPATH=/home/unitree/workspace/zhw_workspace"
    )

G1_NUM_MOTOR = 29
TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"

CONTROL_DT = 0.002          # 500Hz, 和 robot_arm_receiver.py 一致
STARTUP_RAMP = 3.0          # 启动时 kp 从 10% 爬到 100% 的时间
STARTUP_KP_RATIO = 0.1
DEFAULT_JOG_SPEED = 0.3     # rad/s, 目标值的爬升速度 (不是电机速度)
RELEASE_RAMP = 2.0          # 退出时 kp 降到 0 的时间

# 下半身: 只给阻尼不给位置控制。机器人是吊着的, 让腿自然垂着比拉到零位安全,
# kd 用来防止它荡来荡去。
LOWER_BODY_KP = 0.0
LOWER_BODY_KD = 1.0

# 增益直接沿用 robot_arm_receiver.py 的整定值
Kp_full = [
    60, 60, 60, 100, 40, 40,
    60, 60, 60, 100, 40, 40,
    100, 100, 100,
    120, 80, 180, 60, 25, 25, 50,
    80, 160, 160, 80, 30, 25, 40,
]
Kd_full = [
    1, 1, 1, 2, 1, 1,
    1, 1, 1, 2, 1, 1,
    2, 2, 2,
    5.0, 3.5, 10.0, 2.5, 1.0, 1.0, 2.0,
    4.0, 8.0, 8.0, 3.0, 1.5, 1.5, 1.0,
]

# 代号 -> (电机索引, 中文名, (下限, 上限))  —— 限位取自 robot_arm_receiver.py
JOINTS = {
    "lsp": (15, "左肩俯", (-3.0892, 2.6704)),
    "lsr": (16, "左肩滚", (-1.5882, 2.2515)),
    "lsy": (17, "左肩偏", (-2.618,  2.618)),
    "lel": (18, "左肘",   (-1.0472, 2.0944)),
    "lwr": (19, "左腕滚", (-1.9722, 1.9722)),
    "rsp": (22, "右肩俯", (-3.0892, 2.6704)),
    "rsr": (23, "右肩滚", (-2.2515, 1.5882)),
    "rsy": (24, "右肩偏", (-2.618,  2.618)),
    "rel": (25, "右肘",   (-1.0472, 2.0944)),
    "rwr": (26, "右腕滚", (-1.9722, 1.9722)),
}
ORDER = ["lsp", "lsr", "lsy", "lel", "lwr", "rsp", "rsr", "rsy", "rel", "rwr"]

# 全程锁零的腕关节
LOCKED_WRISTS = (20, 21, 27, 28)
# 本工具会主动控制的所有臂电机
ARM_MOTORS = [JOINTS[k][0] for k in ORDER] + list(LOCKED_WRISTS)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Jogger:
    def __init__(self, iface=None, dry_run=False):
        self.dry_run = dry_run
        if iface:
            ChannelFactoryInitialize(0, iface)
        else:
            ChannelFactoryInitialize(0)

        self.crc = CRC()
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.mode_machine = 0

        self.state = None
        self.state_lock = threading.Lock()

        self.target = {}        # motor_idx -> 目标角度 (rad)
        self.jog_speed = DEFAULT_JOG_SPEED
        self.kp_scale = STARTUP_KP_RATIO
        self._t0 = None
        self._releasing = False
        self._release_t0 = None
        self.running = True

        self.sub = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
        self.sub.Init(self._on_state, 10)

        self.pub = ChannelPublisher(TOPIC_LOWCMD, LowCmd_)
        self.pub.Init()

    # ---------- 状态 ----------
    def _on_state(self, msg: LowState_):
        with self.state_lock:
            self.state = msg
            self.mode_machine = msg.mode_machine

    def wait_state(self, timeout=5.0):
        t = time.time()
        while time.time() - t < timeout:
            with self.state_lock:
                if self.state is not None:
                    return True
            time.sleep(0.05)
        return False

    def q_now(self, idx):
        with self.state_lock:
            if self.state is None:
                return 0.0
            return self.state.motor_state[idx].q

    # ---------- 控制循环 ----------
    def start(self):
        # 目标 = 当前实测位置, 保证接管瞬间不跳
        for key in ORDER:
            idx = JOINTS[key][0]
            self.target[idx] = self.q_now(idx)
        for idx in LOCKED_WRISTS:
            self.target[idx] = 0.0

        self._t0 = time.time()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        # 当前实际下发的角度, 从实测位置开始, 按 jog_speed 爬向 target
        cmd = dict(self.target)
        next_t = time.time()

        while self.running:
            now = time.time()

            if self._releasing:
                elapsed = now - self._release_t0
                self.kp_scale = max(0.0, 1.0 - elapsed / RELEASE_RAMP)
            else:
                elapsed = now - self._t0
                if elapsed < STARTUP_RAMP:
                    r = elapsed / STARTUP_RAMP
                    self.kp_scale = STARTUP_KP_RATIO + (1.0 - STARTUP_KP_RATIO) * r
                else:
                    self.kp_scale = 1.0

            step = self.jog_speed * CONTROL_DT
            for idx, tgt in self.target.items():
                delta = tgt - cmd[idx]
                if abs(delta) <= step:
                    cmd[idx] = tgt
                else:
                    cmd[idx] += math.copysign(step, delta)

            self._write(cmd)

            next_t += CONTROL_DT
            sleep = next_t - time.time()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.time()

    def _write(self, cmd):
        self.low_cmd.mode_pr = 0
        self.low_cmd.mode_machine = self.mode_machine

        for i in range(G1_NUM_MOTOR):
            m = self.low_cmd.motor_cmd[i]
            m.mode = 1
            m.dq = 0.0
            m.tau = 0.0
            if i in cmd:
                m.q = cmd[i]
                m.kp = Kp_full[i] * self.kp_scale
                m.kd = Kd_full[i] * self.kp_scale
            else:
                # 腿 + 腰: 只给阻尼, 不给位置目标
                m.q = 0.0
                m.kp = LOWER_BODY_KP
                m.kd = LOWER_BODY_KD

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        if not self.dry_run:
            self.pub.Write(self.low_cmd)

    # ---------- 用户操作 ----------
    def set_joint(self, key, value, relative=False):
        idx, name, (lo, hi) = JOINTS[key]
        want = self.target[idx] + value if relative else value
        clamped = clamp(want, lo, hi)
        self.target[idx] = clamped
        note = ""
        if abs(clamped - want) > 1e-6:
            note = f"   [!] 超限位, 已夹到 {clamped:+.3f} (限位 {lo:+.3f} ~ {hi:+.3f})"
        print(f"  {key} {name}  目标 -> {clamped:+.3f} rad ({math.degrees(clamped):+.1f}°){note}")

    def home(self):
        print("  全部臂关节回零 ...")
        for key in ORDER:
            self.target[JOINTS[key][0]] = 0.0

    def show(self):
        print("  " + "-" * 62)
        print(f"  {'代号':<6}{'关节':<8}{'目标(rad)':>11}{'实测(rad)':>11}{'误差':>9}{'角度':>9}")
        print("  " + "-" * 62)
        for key in ORDER:
            idx, name, _ = JOINTS[key]
            tgt = self.target[idx]
            act = self.q_now(idx)
            print(f"  {key:<6}{name:<8}{tgt:>11.3f}{act:>11.3f}"
                  f"{act - tgt:>9.3f}{math.degrees(act):>8.1f}°")
        print("  " + "-" * 62)
        print(f"  kp_scale={self.kp_scale:.2f}   点动速度={self.jog_speed:.2f} rad/s")

    def shutdown(self):
        """回零 -> 泄力 -> 停止发布。"""
        print("\n[*] 回零中 ...")
        self.home()
        # 等目标走到位 (最多 15s)
        t = time.time()
        while time.time() - t < 15.0:
            if all(abs(self.q_now(JOINTS[k][0])) < 0.08 for k in ORDER):
                break
            time.sleep(0.1)

        print("[*] 泄力 (2s kp 渐出) ...")
        self._release_t0 = time.time()
        self._releasing = True
        time.sleep(RELEASE_RAMP + 0.3)

        self.running = False
        time.sleep(0.05)
        print("[+] 已安全退出。手臂现在是自由状态。")


HELP = """
  可用命令:
    <代号> <角度>     绝对角度, 如  rsp 0.5
    <代号> +<增量>    相对当前目标, 如  rel +0.2  /  rel -0.2
    show              打印所有关节 目标/实测/误差
    home              全部回零
    speed <rad/s>     改点动速度 (当前可用范围 0.05 ~ 1.0)
    help              这份帮助
    q / exit          回零后安全退出

  关节代号:
    左臂  lsp(肩俯) lsr(肩滚) lsy(肩偏) lel(肘) lwr(腕滚)
    右臂  rsp(肩俯) rsr(肩滚) rsy(肩偏) rel(肘) rwr(腕滚)
"""


def preflight(assume_yes):
    print("=" * 66)
    print("  g1_jog.py —— lowcmd 关节点动")
    print("=" * 66)
    print("  这个工具会直接驱动电机, 且【没有任何平衡控制】。")
    print()
    print("  确认以下三条:")
    print("    1. 机器人已经吊起来 (双脚离地或不承重)")
    print("    2. 机器人处于调试模式 / 运控服务已停止")
    print("    3. 物理急停按钮伸手能按到")
    print("=" * 66)
    if assume_yes:
        print("[!] --yes 已指定, 跳过确认")
        return True
    try:
        ans = input("  输入大写 YES 继续: ")
    except (EOFError, KeyboardInterrupt):
        print("\n  已取消")
        return False
    if ans.strip() != "YES":
        print("  已取消")
        return False
    return True


def check_sport_stopped():
    """运控服务还在跑的话, lowcmd 会和它抢电机。返回 True 表示可以继续。"""
    try:
        from unitree_sdk2py.b2.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient)
        msc = MotionSwitcherClient()
        msc.SetTimeout(3.0)
        msc.Init()
        code, result = msc.CheckMode()
    except Exception:
        print("[*] 运控服务状态查询不可用, 跳过检查")
        return True

    if code != 0:
        print("[*] 运控服务无响应 —— 符合调试模式的预期, 继续")
        return True
    name = result.get("name") if isinstance(result, dict) else ""
    if name:
        print(f"[!] 运控服务 ({name}) 还在运行!")
        print("    它和 rt/lowcmd 会同时写同一批电机, 一起跑会打架。")
        print("    请先让机器人进入调试模式, 或用 --mode armsdk 的接收端。")
        return False
    print("[*] 运控服务已停止, 符合 lowcmd 的前提")
    return True


def main():
    ap = argparse.ArgumentParser(
        description="纯 SSH 的 G1 手臂关节点动 (lowcmd, 需吊装 + 调试模式)")
    ap.add_argument("--iface", default=None, help="DDS 网卡名, 板载机上可省略")
    ap.add_argument("--yes", action="store_true", help="跳过启动确认")
    ap.add_argument("--speed", type=float, default=DEFAULT_JOG_SPEED,
                    help=f"点动速度 rad/s (默认 {DEFAULT_JOG_SPEED})")
    ap.add_argument("--dry-run", action="store_true",
                    help="全流程照跑但【不发布 rt/lowcmd】, 电机不会动。"
                         "用来验证 DDS/导入/限位逻辑")
    args = ap.parse_args()

    if args.dry_run:
        print("[*] --dry-run: 只读状态, 不会发布任何指令, 电机不会动")
    elif not preflight(args.yes):
        sys.exit(1)

    jog = Jogger(iface=args.iface, dry_run=args.dry_run)
    jog.jog_speed = clamp(args.speed, 0.05, 1.0)

    print("[*] 等待 rt/lowstate ...")
    if not jog.wait_state():
        sys.exit("[!] 收不到 rt/lowstate —— DDS 不通。板载机以外的机器要加 --iface")
    print(f"[+] 已连接 (mode_machine={jog.mode_machine})")

    if not check_sport_stopped():
        sys.exit(1)

    print(f"[*] 启动: {STARTUP_RAMP:.0f}s 从当前姿态缓慢接管, 手臂不应该跳动")
    jog.start()
    time.sleep(STARTUP_RAMP + 0.2)
    print("[+] 接管完成")
    print(HELP)

    def on_sig(sig, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    try:
        while True:
            try:
                line = input("g1> ").strip()
            except EOFError:
                break
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()

            if cmd in ("q", "exit", "quit"):
                break
            if cmd == "help":
                print(HELP)
            elif cmd == "show":
                jog.show()
            elif cmd == "home":
                jog.home()
            elif cmd == "speed":
                if len(parts) != 2:
                    print("  用法: speed 0.3")
                    continue
                try:
                    v = clamp(float(parts[1]), 0.05, 1.0)
                except ValueError:
                    print("  速度得是个数")
                    continue
                jog.jog_speed = v
                print(f"  点动速度 -> {v:.2f} rad/s")
            elif cmd in JOINTS:
                if len(parts) != 2:
                    print(f"  用法: {cmd} 0.5   或   {cmd} +0.2")
                    continue
                raw = parts[1]
                relative = raw[0] in "+-"
                try:
                    val = float(raw)
                except ValueError:
                    print("  角度得是个数")
                    continue
                jog.set_joint(cmd, val, relative=relative)
            else:
                print(f"  不认识的命令 '{cmd}', 敲 help 看列表")
    except KeyboardInterrupt:
        print()
    finally:
        jog.shutdown()


if __name__ == "__main__":
    main()
