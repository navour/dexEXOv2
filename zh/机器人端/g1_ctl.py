#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
g1_ctl.py —— 不用遥控器, 纯 SSH 控制 G1 的运控状态机

背后调的就是遥控器按键对应的那套 LocoClient API (DDS 服务名 "sport")。

    python3 g1_ctl.py status            # 只读, 查当前状态
    python3 g1_ctl.py damp              # 阻尼态 == 遥控器 L2+B  <-- 出事先敲这个
    python3 g1_ctl.py prepare           # 站立锁定 == L2+上
    python3 g1_ctl.py start             # 主运控/可行走 == R1+X
    python3 g1_ctl.py zero              # 零力矩, 全身瘫软 (吊着才能用)
    python3 g1_ctl.py sit               # 坐下
    python3 g1_ctl.py squat             # 蹲下
    python3 g1_ctl.py lie2stand         # 趴着起身
    python3 g1_ctl.py stop              # 速度清零
    python3 g1_ctl.py move VX VY VYAW   # 走 (需要先 start)

===========================================================================
 安全须知 —— 没有遥控器的时候
===========================================================================
 1. 本脚本发的 damp 要走 DDS -> 运控服务, 网线一松就到不了。
    它【不能】替代遥控器的 L2+R2, 唯一可靠的保险是【物理急停按钮】。
    动腿之前先确认急停伸手能按到。
 2. 建议开两个 SSH 窗口, 一个专门停在那儿准备敲 `g1_ctl.py damp`。
 3. 会让腿动的指令 (prepare/start/lie2stand/sit/squat/move) 需要输入
    确认字符串, 或者加 --yes 跳过。damp / zero / stop / status 不需要。
 4. 状态机不能乱跳。从零力矩起身的顺序是:
        zero(0) -> damp(1) -> prepare(4) -> start(200)
    直接从 0 跳 200 会被运控服务拒绝。
===========================================================================
"""

import argparse
import json
import sys
import time

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.g1.loco.g1_loco_api import (
        ROBOT_API_ID_LOCO_GET_FSM_ID,
        ROBOT_API_ID_LOCO_GET_FSM_MODE,
        ROBOT_API_ID_LOCO_GET_BALANCE_MODE,
    )
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
except ModuleNotFoundError as e:
    sys.exit(
        f"[!] 找不到 unitree_sdk2py ({e}).\n"
        f"    非交互式 ssh 不读 ~/.bashrc, 手动补一句:\n"
        f"    export PYTHONPATH=/home/unitree/workspace/zhw_workspace"
    )


FSM_NAMES = {
    0: "ZeroTorque 零力矩 (全身瘫软, 必须吊着)",
    1: "Damp 阻尼态 (安全停靠态)",
    2: "Squat 蹲下",
    3: "Sit 坐下",
    4: "StandUp 站立锁定",
    200: "Start 主运控 (可行走, arm_sdk 可用)",
    702: "Lie2StandUp 趴->站 过渡中",
    706: "Squat<->StandUp 过渡中",
}

# 会让腿动、需要确认的动作
DANGEROUS = {"prepare", "start", "lie2stand", "sit", "squat", "move"}


def get_fsm(client):
    """返回当前 FSM ID, 查不到返回 None。"""
    try:
        code, data = client._Call(ROBOT_API_ID_LOCO_GET_FSM_ID, "{}")
        if code != 0 or not data:
            return None
        return json.loads(data).get("data")
    except Exception:
        return None


def cmd_status(client):
    print("=" * 60)
    for label, api in (
        ("FSM ID", ROBOT_API_ID_LOCO_GET_FSM_ID),
        ("FSM MODE", ROBOT_API_ID_LOCO_GET_FSM_MODE),
        ("BALANCE MODE", ROBOT_API_ID_LOCO_GET_BALANCE_MODE),
    ):
        try:
            code, data = client._Call(api, "{}")
            val = json.loads(data).get("data") if data else None
            note = f"   <- {FSM_NAMES.get(val, '未知')}" if label == "FSM ID" else ""
            print(f"{label:>14}: code={code}  value={val}{note}")
        except Exception as exc:
            print(f"{label:>14}: 查询失败 {exc!r}")

    holder = {}
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(lambda msg: holder.setdefault("s", msg), 10)
    for _ in range(50):
        if "s" in holder:
            break
        time.sleep(0.1)

    print("-" * 60)
    if "s" not in holder:
        print("  未收到 rt/lowstate —— DDS 不通, 或运控服务没起来")
        print("=" * 60)
        return

    s = holder["s"]
    r, p, y = s.imu_state.rpy
    posture = "直立" if abs(p) < 0.35 else ("趴着/躺着" if abs(p) > 1.0 else "倾斜")
    print(f"  躯干姿态: roll={r:+.3f} pitch={p:+.3f} yaw={y:+.3f} rad   -> {posture}")
    print(f"  mode_machine={s.mode_machine}  mode_pr={s.mode_pr}")

    tau = [abs(s.motor_state[i].tau_est) for i in range(12)]
    print(f"  腿部力矩: 最大 {max(tau):.2f} N·m, 平均 {sum(tau)/12:.2f} N·m")
    if max(tau) < 1.0:
        print("           -> 力矩接近 0, 机器人没在自己承重 (吊着 / 或已瘫软)")
    print("=" * 60)


def confirm(action, fsm_now, assume_yes):
    if assume_yes:
        print(f"[!] --yes 已指定, 跳过确认")
        return True
    print()
    print("!" * 60)
    print(f"  即将执行: {action}   (当前 FSM={fsm_now} {FSM_NAMES.get(fsm_now, '未知')})")
    print("  这会让【腿】动。确认:")
    print("    - 物理急停按钮伸手能按到")
    print("    - 机器人周围 2 米内没有人和障碍物")
    print("    - 如果机器人是吊着的, 吊具能承重且不会卡住")
    print("!" * 60)
    try:
        ans = input("  输入大写 YES 继续, 其它任何输入都会取消: ")
    except (EOFError, KeyboardInterrupt):
        print("\n  已取消")
        return False
    if ans.strip() != "YES":
        print("  已取消")
        return False
    return True


def main():
    ap = argparse.ArgumentParser(
        description="不用遥控器, 通过 SSH 控制 G1 运控状态机",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("action", choices=[
        "status", "damp", "zero", "prepare", "start",
        "sit", "squat", "lie2stand", "stop", "move",
    ])
    ap.add_argument("vel", nargs="*", type=float, default=[],
                    help="move 用: VX VY VYAW (m/s, m/s, rad/s)")
    ap.add_argument("--iface", default=None,
                    help="DDS 网卡名。在板载机上跑可省略")
    ap.add_argument("--yes", action="store_true",
                    help="跳过交互确认 (危险动作也不再问)")
    args = ap.parse_args()

    if args.iface:
        ChannelFactoryInitialize(0, args.iface)
    else:
        ChannelFactoryInitialize(0)

    client = LocoClient()
    client.SetTimeout(5.0)
    client.Init()

    if args.action == "status":
        cmd_status(client)
        return

    fsm_now = get_fsm(client)
    if fsm_now is None:
        print("[!] 查不到当前 FSM (code=3102) —— sport 运控服务没在跑。")
        print()
        print("    最常见的原因是【机器人处于调试模式】。调试模式会停掉运控服务,")
        print("    而本脚本所有指令都是发给它的 RPC, 所以全都用不了。")
        print("    这时候遥控器的 L2+B / R1+X 同样失效, 走的是同一条链路。")
        print()
        print("    确认一下: ps aux | grep sport   (只有 master_service 就是没跑)")
        print()
        print("    调试模式下要控制机器人, 改用低层接口:")
        print("      python3 robot_arm_receiver.py --mode lowcmd    # 必须吊起来")
        sys.exit(1)

    if args.action in DANGEROUS and not confirm(args.action, fsm_now, args.yes):
        sys.exit(1)

    if args.action == "damp":
        print("[*] -> Damp 阻尼态 (FSM 1)")
        client.Damp()

    elif args.action == "zero":
        if fsm_now not in (0, 1):
            print(f"[!] 当前 FSM={fsm_now}, 从这里切零力矩机器人会直接瘫下去。")
            print("    先 damp。要强行来请自己改代码。")
            sys.exit(1)
        print("[*] -> ZeroTorque 零力矩 (FSM 0)")
        client.ZeroTorque()

    elif args.action == "prepare":
        if fsm_now == 0:
            print("[!] 当前是零力矩(0), 不能直接跳站立锁定。先跑 `g1_ctl.py damp`。")
            sys.exit(1)
        print("[*] -> 站立锁定 (FSM 4) == 遥控器 L2+上")
        client.SetFsmId(4)

    elif args.action == "start":
        if fsm_now not in (1, 2, 4, 706):
            print(f"[!] 当前 FSM={fsm_now}, 不建议直接进主运控。")
            print("    顺序应该是 damp(1) -> prepare(4) -> start(200)。")
            sys.exit(1)
        print("[*] -> 主运控 (FSM 200) == 遥控器 R1+X")
        client.Start()

    elif args.action == "sit":
        print("[*] -> Sit 坐下 (FSM 3)")
        client.Sit()

    elif args.action == "squat":
        print("[*] -> Squat 蹲下 (FSM 706)")
        client.StandUp2Squat()

    elif args.action == "lie2stand":
        print("[*] -> Lie2StandUp 趴着起身 (FSM 702)")
        client.Lie2StandUp()

    elif args.action == "stop":
        print("[*] -> 速度清零")
        client.StopMove()

    elif args.action == "move":
        if len(args.vel) != 3:
            sys.exit("[!] move 需要 3 个参数: VX VY VYAW")
        if fsm_now != 200:
            sys.exit(f"[!] 当前 FSM={fsm_now}, 不在主运控(200), 走不了。先 start。")
        vx, vy, vyaw = args.vel
        if max(abs(vx), abs(vy)) > 0.5 or abs(vyaw) > 0.5:
            sys.exit("[!] 速度超过 0.5 的安全上限, 本脚本拒绝。要更快请自己改代码。")
        print(f"[*] -> Move vx={vx} vy={vy} vyaw={vyaw}")
        client.Move(vx, vy, vyaw)

    time.sleep(1.0)
    fsm_new = get_fsm(client)
    print(f"[+] 完成。FSM: {fsm_now} -> {fsm_new} ({FSM_NAMES.get(fsm_new, '未知')})")


if __name__ == "__main__":
    main()
