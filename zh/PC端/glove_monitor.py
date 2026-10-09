#!/usr/bin/env python3
"""手套 → 闭合度 的最小实时监视器，用来把 PC 这一侧单独验证掉。

完整链路里 `mhandpro_diagnostic` 的 teleop 流后面还接着 pygame 渲染和
MuJoCo 仿真，出问题不好定位。这个工具只起 glove_source，把六个通道打成
条形图，不碰渲染也不发 UDP —— 数字跟着手指动，就说明 PC 这一侧全通了。

    python3 glove_monitor.py [cfg路径]

另一个终端里:
    mhandpro_diagnostic <so路径>
    诊断> load <标定文件>
    诊断> teleop <同一个cfg>
"""

import argparse
import os
import sys
import time

import hand_mapping
from glove_source import GloveSource


_MY_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CFG = os.path.join(
    os.path.dirname(_MY_DIR), "mhandpro", "config", "inspire_right_sim.cfg")

BAR_WIDTH = 24


def bar(value):
    filled = int(round(value * BAR_WIDTH))
    return "█" * filled + "·" * (BAR_WIDTH - filled)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cfg", nargs="?", default=DEFAULT_CFG,
                        help=f"Inspire 配置，默认 {DEFAULT_CFG}")
    parser.add_argument("--port", type=int, default=None,
                        help="监听端口，默认取 cfg 里的 port")
    args = parser.parse_args()

    source = GloveSource(cfg_path=args.cfg, port=args.port)
    print(f"监听 127.0.0.1:{source.port}，等 teleop 连入。Ctrl+C 退出。")
    print(f"端点取自 {args.cfg}")
    print(f"open  ={source.open_counts}")
    print(f"closed={source.closed_counts}\n")

    names = hand_mapping.CHANNEL_NAMES
    try:
        while True:
            closure = source.latest_closure()
            # 光标回到顶部重画，避免刷屏
            sys.stdout.write("\033[H\033[J")
            state = "已连接" if source.connected else "等待 teleop 连入"
            print(f"[{state}]  帧={source.frames}  坏行={source.bad_lines}\n")
            if closure is None:
                print("  没有数据 —— 消费端会张开手（这不是零指令，是「无数据」）")
            else:
                for name, value in zip(names, closure):
                    print(f"  {name:<6} {bar(value)} {value:5.2f}")
                angles = hand_mapping.expand_mimic(
                    hand_mapping.closure_to_angles(closure))
                print(f"\n  仿真显示行程 {hand_mapping.SIM_RANGE_SCALE:.0%}"
                      f"（真手安全上限 "
                      f"{hand_mapping.REAL_HAND_MAX_RANGE_SCALE:.0%}），"
                      f"小指根关节 {angles['right_little_1_joint']:.3f} rad")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n退出。")
    finally:
        source.close()


if __name__ == "__main__":
    main()
