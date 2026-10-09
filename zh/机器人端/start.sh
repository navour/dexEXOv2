#!/usr/bin/env bash
# 板载机上的接收端启动脚本。部署到 ~/zh/ 使用:
#
#     ~/zh/start.sh --hand right --hand-only        # 只跑手, 不碰电机
#     ~/zh/start.sh --hand right --waist            # 手臂 + 腰 + 手
#
# 为什么要这个脚本而不是直接 python3:
#   板载机的 ~/.bashrc 里有一个**交互式**的 ROS 版本选择提示
#   (ros:foxy(1) noetic(2) ?)。选了 ROS 之后它会把 ROS 自带的 cyclonedds
#   挂到 PYTHONPATH 前面, 顶掉宇树 SDK 依赖的那份, unitree_sdk2py 的
#   idl 导入就会在很深的地方炸掉, 报错信息完全看不出是环境问题。
#   非登录 shell 又根本不读 .bashrc, 于是同一条命令在不同的进入方式下
#   表现不一样。这里把环境写死, 谁来跑都一样。
set -euo pipefail

export CYCLONEDDS_URI=/home/unitree/cyclonedds_ws/cyclonedds.xml
# 只挂宇树 SDK。inspire_hand_sdk 是给 DDS 那条路用的, 我们走裸 Modbus,
# 不需要它, 少一个路径就少一次符号冲突的机会。
export PYTHONPATH=/home/unitree/workspace/zhw_workspace

cd "$(dirname "$(readlink -f "$0")")"
exec python3 -u robot_arm_receiver.py "$@"
