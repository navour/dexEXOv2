# G1 双臂 MuJoCo 仿真

本目录是 IMU 双臂遥操的第一阶段仿真：用 MuJoCo 加载全身 29 自由度的 G1
模型，通过现有 UDP 协议驱动双臂和腰部 yaw。仿真器同时兼容新版 `UA2M`
（位置、速度、序号和时间戳）、`UAWS` 腰部尾块与旧版 `UARM` 数据包。

该阶段用于验证 IMU 标定、解算方向、关节零位、限位、平滑性和丢包回零。
**它是纯运动学回放** —— 直接写 qpos，不算力矩、不算平衡。腿只是摆成站姿
造型，不参与任何计算。所以它能告诉你腰部跟随的方向、增益和限速对不对，
但**说不了**官方运控服务能不能容忍这个扰动，那要看真机或 HumDex 全身仿真。

模型说明见 `models/g1_29dof/SOURCE.md`。旧的上半身模型仍保留，用
`--model models/g1_description/g1_dual_arm.urdf` 指定；那条路径针对
MuJoCo 3.10 的重复 `meshes/meshes` 修正只发生在加载时的内存副本中，
磁盘上的官方 URDF 未修改。

## 目录

```text
仿真/
├── g1_mujoco_sim.py
├── requirements.txt
└── models/
    ├── g1_29dof/                  # 全身 29 自由度，默认模型
    │   ├── g1_29dof.xml           # MJCF，与 HumDex 全身仿真同一份
    │   ├── g1_29dof.urdf          # 宇树官方 URDF，供 PC 端渲染器用
    │   ├── LICENSE
    │   ├── SOURCE.md
    │   └── meshes/
    └── g1_description/            # 旧的上半身模型
        ├── g1_dual_arm.urdf       # 宇树官方 URDF，未修改
        ├── g1_dual_arm.xml
        ├── LICENSE
        ├── SOURCE.md
        └── meshes/
```

## 安装

建议在仿真目录使用独立环境：

```bash
cd 仿真
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 先验证 MuJoCo 模型

不连接 IMU，运行内置小幅度双臂动作：

```bash
python3 g1_mujoco_sim.py --demo
```

MuJoCo 窗口中应显示固定的 G1 上半身，左右臂持续平滑运动。

## 连接现有 PC 发送端

终端 1：

```bash
cd 仿真
source .venv/bin/activate
python3 g1_mujoco_sim.py
```

终端 2（从仓库根目录运行）：

```bash
cd ~/unitree
PC端/.venv/bin/python PC端/dual_arm_viz.py \
  --udp-target 127.0.0.1:9527
```

五个 IMU 都在线时，直接按 `C` 走双臂连续标定：右臂 7 步走完自动存盘
并提示换左臂，再走 7 步，共 14 步。中途某条臂未通过会终止整条链，
按 `C` 重来即可。只想补标一条臂时用 `Shift+C`，先用 `R`/`L` 选好臂。

标定完成后按 `B` 一次开启双臂跟随；`SPACE` 仍然只切换当前活跃的那条臂。
如果按 `B` 时只有一条臂满足条件（另一条未标定、IMU 掉线、或标定质量
只允许仿真而目标是真机），仿真器会只跟随可用的那条，另一条在 ACTIVITY
日志里写明原因。

左侧 TRACKING 卡同时显示五个 IMU 的在线状态，以及左右臂各自的标定、
跟随和五个关节角；活跃臂的列头带青色下划线。

仿真器完整支持发送端的模式：

- `0`：双臂缓慢回零
- `1`：右臂跟随，左臂回零
- `2`：左臂跟随，右臂回零
- `3`：双臂跟随

终端状态会显示当前协议、接收频率、序号、累计丢包，以及发送端和
仿真器位于同一台电脑时可计算的单向 UDP 传输时间。

## 只打开右小臂一个 IMU

用于临时检查单个 IMU、UDP 和 MuJoCo 链路：

```bash
PC端/.venv/bin/python PC端/dual_arm_viz.py \
  --right-forearm-only \
  --udp-target 127.0.0.1:9527
```

在发送端窗口中：

1. 保持右小臂为希望的零位，按 `C` 采集当前姿态。
2. 按 `SPACE` 开始跟随。
3. 按 `X` 可清除零位并重新采集。

该诊断模式将 G1 右肩三个关节固定为零，根据小臂 IMU
相对零位的旋转驱动 `right_elbow_joint` 和 `right_wrist_roll_joint`。
如果某个动作方向相反，重启时加上：

```bash
--forearm-elbow-sign -1
```

或：

```bash
--forearm-wrist-sign -1
```

这只是单传感器诊断映射，不代表完整的人体右臂姿态。

## 只打开右大臂和右小臂两个 IMU

该模式用右大臂 IMU 驱动肩部三个关节，用右小臂相对右大臂
的旋转驱动肘部和腕 roll，不需要胸部或左臂 IMU：

```bash
PC端/.venv/bin/python PC端/dual_arm_viz.py \
  --right-arm-two-imu \
  --udp-target 127.0.0.1:9527
```

使用时保持右大臂和右小臂为希望的初始姿态，按 `C` 同时采集
两只 IMU 的零位，再按 `SPACE` 开始跟随。按 `X` 可重新采集。

如果某个关节方向相反，可用以下参数单独反转：

```text
--upper-pitch-sign -1
--upper-roll-sign -1
--upper-yaw-sign -1
--forearm-elbow-sign -1
--forearm-wrist-sign -1
```

双 IMU 模式以采集时的大臂绝对姿态作为临时躯干参考。因此测试过程
中不要转动躯干；需要补偿躯干运动时，仍需接入胸部 IMU。

如果发送端在另一台电脑，仿真器使用：

```bash
python3 g1_mujoco_sim.py --bind 0.0.0.0
```

然后将发送目标改为仿真电脑的 IP 和 `9527` 端口。

## 无界面检查

```bash
python3 g1_mujoco_sim.py --demo --headless --duration 2
```

## 安全行为

- 通过 MuJoCo 关节名称映射，不依赖 URDF 中的关节排列顺序。
- 现有发送端控制每侧 5 个关节；两侧 `wrist_pitch` 和 `wrist_yaw`
  在第一阶段锁定为零。
- 所有输入按官方 URDF 限位截断。
- 跟随默认限速 `5 rad/s`，回零默认限速 `0.3 rad/s`。
- 超过 `1 s` 未收到有效 UDP 包时，自动进入双臂回零。
- 拒绝错误包头、非法模式以及 NaN/Inf 关节角。

查看所有参数：

```bash
python3 g1_mujoco_sim.py --help
```
