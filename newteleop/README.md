# newteleop — G1 双手 INSPIRE 力反馈遥操移交手册

> 文档版本：2026-09-01
>
> 当前硬件：Unitree G1、INSPIRE 左/右灵巧手、左/右 mHandPro、左/右外骨骼、双 STM32/BLE FSR
>
> 首次接手请先通读「安全约束」和「分阶段联调」，不要直接运行全功能 ARM。

`newteleop` 是外骨骼、FSR 和力控状态机的新一代代码包。与同仓库的
`zh` 配合时，Ubuntu 负责 mHandPro 与手臂/腰部上位遥操，G1 板载机负责
DDS 手臂/腰部和两只 INSPIRE，树莓派负责双 FSR 与十个外骨骼舵机。

## 快速导航

1. [当前结论与范围](#1-当前结论与范围)
2. [安全约束](#2-安全约束)
3. [整体架构](#3-整体架构)
4. [设备、端口和通道表](#4-设备端口和通道表)
5. [目录和代码职责](#5-目录和代码职责)
6. [数据协议和力控含义](#6-数据协议和力控含义)
7. [安装和部署](#7-安装和部署)
8. [每次联调前检查](#8-每次联调前检查)
9. [分阶段联调](#9-分阶段联调)
10. [常用命令与参数](#10-常用命令与参数)
11. [停机顺序](#11-停机顺序)
12. [故障排查](#12-故障排查)
13. [验证与代码修改后检查](#13-验证与代码修改后检查)
14. [详细子文档](#14-详细子文档)
15. [移交时必须记录的实物信息](#15-移交时必须随代码记录的实物信息)

---

## 1. 当前结论与范围

已完成：

- 右手 STM32/BLE `F0:FD:45:02:85:B3` → TCP 9001。
- 左手 STM32/BLE `F0:FD:45:02:67:3B` → TCP 9002。
- 双手 FSR 顺序统一为 `[拇指, 食指, 中指, 无名指, 小指]`。
- 右外骨骼 Dynamixel ID 1～5，左外骨骼 ID 6～10。
- 单手通用力控和单进程双手力控。
- G1 板载机直连左/右 INSPIRE Modbus TCP，发布双手 top_touch。
- 不加 mHandPro 时，可用手按压 INSPIRE 指尖独立测试双外骨骼。
- 加 mHandPro 时，可与 `zh` 的 G1 双臂、腰部和双手位置遥操联合运行。

当前试验参数：

- XL330-M288（探测 `model=1200`）Goal Current 单位约为 `1 mA/raw`。
- 双手收绳上限已调整为 `max-goal-current=300`，实物测试时需重点观察供电压降、温升和堵转。
- 左/右手五指已确认正电流收绳，参数均为 `1,1,1,1,1`。
- 电流、方向、FSR 静息值和 INIT 位置都是当前机械实物的结果，换绳路、舵机或外骨骼后必须重新确认。

---

## 2. 安全约束

1. 首次运行、换机构、换舵机或修改电流方向后，必须未穿戴、低电流、单指测试。
2. 测试时必须能立即物理断电。`STOP` 是软件保护，不能代替电源开关。
3. `/dev/serial0` 上的 ID 1～10 只能由一个进程控制。双手必须运行
   `dual_hand_force_test.py`，禁止同时启动两个单手力控。
4. G1 上的 `zh/机器人端/hand_driver.py` 必须是两只 INSPIRE 的唯一 Modbus 所有者。
   联调时禁止再启动 `standalone_inspire_bridge.py`、`inspire_hand_ctrl.py --real-hand`
   或官方 Inspire DDS Headless 驱动。
5. G1 接收端统一用 `~/zh/start.sh` 启动。不要直接运行
   `robot_arm_receiver.py`，也不要传 `--iface wlan0`。
6. 先测手，再测手臂，最后测腰部。全部功能不得在第一次通电时同时开启。
7. 任一 Dynamixel 通信、硬件错误或电流越限会导致十指联锁 STOP。

---

## 3. 整体架构

```text
左/右 mHandPro（共用一个USB接收器）
        │
        ▼
Ubuntu 上位机（zh）
  mhandpro_diagnostic
        │ TCP 9103右 / 9104左
        ▼
  dual_arm_viz.py ── 手套闭合度 + 手臂/IMU + 腰部
        │ UDP 9527（UHND/UAWS尾块）
        ▼
G1 板载机 192.168.3.78（zh）
  robot_arm_receiver.py
    ├─ Unitree DDS/eth0 → G1双臂和腰部
    ├─ Modbus TCP → 右 INSPIRE 192.168.123.211:6000
    ├─ Modbus TCP → 左 INSPIRE 192.168.123.210:6000
    ├─ TCP 9201/9202 → 右/左 INSPIRE top_touch
    └─ TCP 9301/9302 ← 右/左力控位置覆盖
        ▲                         │
        │ Wi-Fi                  │
        │                         ▼
树莓派 192.168.3.76（newteleop）
  右BLE broker → 127.0.0.1:9001 ─┐
  左BLE broker → 127.0.0.1:9002 ─├→ dual_hand_force_test.py
  /dev/serial0 → XL330 ID 1～10 ───┤
  PyQt本地上位机 ← 内存遥测快照 ────┘
```

三台计算机的责任边界：

| 计算机 | 主要代码 | 负责 | 不负责 |
|---|---|---|---|
| Ubuntu PC | `zh/PC端`、`zh/mhandpro` | mHandPro、IMU、可视化、遥操意图 | 不直连G1内网INSPIRE |
| G1板载机 | `zh/机器人端` | DDS、INSPIRE唯一Modbus连接、触觉发布 | 不直连BLE/XL330 |
| 树莓派 | `newteleop/exoskeleton` | BLE FSR、Dynamixel、力控状态机 | 不接管G1手臂DDS |

### 为什么 INSPIRE 不直接 Ubuntu

两只 INSPIRE 位于 G1 机器人内网 `192.168.123.x`。Ubuntu 通过 Wi-Fi
只能到 G1 板载机，通常不能直接 `.210/.211`。因此 Ubuntu 只发送手指闭合意图，
G1 板载机在内网中以 Modbus TCP 写入两只手。

### DDS 和 Modbus 的边界

- G1 手臂和腰部：Unitree DDS。
- `zh` 当前的两只 INSPIRE：G1 板载机原生 socket Modbus TCP。
- 树莓派外骨骼：Dynamixel Protocol 2.0 TTL。
- BLE FSR：Bleak 连接 STM32 BLE，然后在树莓派本机广播 TCP。

---

## 4. 设备、端口和通道表

| 对象 | 右手 | 左手 |
|---|---|---|
| BLE名称 | `RFstar_85B3` | `RFstar_673B` |
| BLE MAC | `F0:FD:45:02:85:B3` | `F0:FD:45:02:67:3B` |
| FSR Broker | `127.0.0.1:9001` | `127.0.0.1:9002` |
| mHandPro → Ubuntu | TCP 9103 | TCP 9104 |
| INSPIRE内网IP | `192.168.123.211:6000` | `192.168.123.210:6000` |
| INSPIRE触觉发布 | G1 TCP 9201 | G1 TCP 9202 |
| INSPIRE力控覆盖 | G1 TCP 9301 | G1 TCP 9302 |
| 外骨骼Dynamixel | ID 1,2,3,4,5 | ID 6,7,8,9,10 |

只读外骨骼上位机默认关闭；启用时使用树莓派 `8080/TCP`。

外骨骼和触觉软件顺序统一为：

```text
0=拇指, 1=食指, 2=中指, 3=无名指, 4=小指
```

INSPIRE 六路角度寄存器顺序不同：

```text
[小指, 无名指, 中指, 食指, 拇指弯曲, 拇指对掌]
```

力控映射为：

```text
[拇,食,中,无,小] → INSPIRE角度槽 [4,3,2,1,0]
```

---

## 5. 目录和代码职责

```text
newteleop/
├── README.md                         # 本移交手册
├── requirements.txt                  # 树莓派力控Python依赖（Qt用apt安装）
├── computer_debug/README.md          # Ubuntu x86_64厂商SDK调试记录
├── mhandpro/                         # 旧的树莓派直连INSPIRE台架路径
│   ├── mhandpro_diagnostic.cpp
│   ├── standalone_inspire_bridge.py
│   ├── config/
│   └── tools/
└── exoskeleton/
    ├── README.md                     # 外骨骼硬件与工具说明
    ├── HAPTIC_FEEDBACK_PLAN_CN.md   # 力反馈设计与安全依据
    ├── UPPER_COMPUTER_PLAN_CN.md    # 双手外骨骼上位机方案
    ├── config/                      # 左右舵机映射/维护参考位
    ├── FSR/
    │   ├── ble_broker.py            # 一个BLE板的唯一连接者，广播原始帧
    │   ├── exo_pressure_common.py   # 18项紧凑帧解析，取最后五路FSR
    │   ├── exo_pressure_monitor.py  # 单手/双手FSR只读监视
    │   ├── fsr_channel_identifier.py# 逐指确认通道顺序
    │   └── exo_pressure_calibrate.py # 历史推拉力计标定工具
    ├── force_control/
    │   ├── left_index_force_test.py # 左食指基础状态机/Controller
    │   ├── left_hand_force_test.py  # 五指共享串口控制器，也支持右手参数
    │   ├── hand_force_test.py       # 单手通用入口 --hand left/right
    │   └── dual_hand_force_test.py  # 双手唯一正式入口，共享ID1～10总线
    ├── supervisor/
    │   ├── local_supervisor.py      # 树莓派PyQt本地上位机/安全按钮
    │   └── supervisor_common.py     # STATUS格式化/BLE子进程管理
    └── tools/
        ├── exo_dynamixel_probe.py       # 只读Ping/模式/错误/位置检查
        ├── exo_single_servo_jog.py      # 未穿戴小步点动
        ├── exo_single_servo_current_test.py # 低电流短脉冲方向检查
        ├── exo_capture_neutral.py       # 维护用参考位采集
        └── left_index_manual_travel_measure.py
```

### `newteleop/mhandpro` 还能不能用

可以用于「树莓派网线直连单只 INSPIRE」的独立台架调试，但它不是 G1 联调主路径。
当 INSPIRE 已装到 G1 时，使用 `zh/mhandpro` + `zh/PC端` + G1 `hand_driver.py`，
不运行 `newteleop/mhandpro/standalone_inspire_bridge.py`。

---

## 6. 数据协议和力控含义

### 6.1 FSR数据

STM32 通过 BLE 发送 18 项紧凑帧：

```text
{前13项旧手套数据, 拇指FSR, 食指FSR, 中指FSR, 无名指FSR, 小指FSR}
```

`ble_broker.py` 不解析数据，只独占 BLE 连接并广播原始字节。
`exo_pressure_common.py` 负责 TCP 分片重组并取最后五项。

### 6.2 G1 → 树莓派 INSPIRE 触觉包

9201/9202 是换行 JSON，核心字段：

```json
{
  "type": "inspire_feedback",
  "hand": "right",
  "top_touch_order": ["拇指", "食指", "中指", "无名指", "小指"],
  "top_touch_raw_max": [0, 0, 0, 0, 0],
  "top_touch_force_n": [0.0, 0.0, 0.0, 0.0, 0.0],
  "touch_valid": true
}
```

G1 仅在自己的 `HandFollower` Modbus 线程中读触觉，避免同一只手并发读写。

### 6.3 树莓派 → G1 力控覆盖

9301/9302 使用换行 JSON：

```json
{
  "type": "haptic_override",
  "state": "GLOVE",
  "finger_states": ["FREE", "LOCKED", "FREE", "FREE", "FREE"],
  "finger_steps": [0.0, 2.0, 0.0, 0.0, 0.0]
}
```

只有传 `--enable-mhandpro` 时，树莓派才会连接 9301/9302。不加 mHandPro 的
外骨骼独立测试不传该参数，只使用 9201/9202 触觉流。

### 6.4 PID 的实际含义

LOCKED 状态的误差是：

```text
force_error = 外骨骼FSR反馈力 - INSPIRE指尖目标力
position_step = clamp(Kp * force_error, -force_step_max, +force_step_max)
```

这个 PID 输出不是外骨骼舵机电流，而是 INSPIRE 已锁定手指的小位置增量。
外骨骼在 FORCE_ENTRY 阶段的收绳电流另由：

```text
requested_current = min(INSPIRE目标力 * force_to_current_gain,
                        max_goal_current)
```

当前双手联调 `force_to_current_gain=60`、`max_goal_current=300`，因此收绳命令为
`min(INSPIRE力×60, 300)`；只有 INSPIRE 力达到 5 N 时才到达 300 上限。

### 6.5 外骨骼状态机

```text
STOP
  │ INIT采集本次返回位置和FSR静息基线，再ARM
  ▼
FREE
  │ INSPIRE触觉 >= contact_on
  ▼
FORCE_ENTRY  模式0，限流收绳
  │ 位置稳定
  ▼
LOCKED       模式5，锁定外骨骼位置；可选向INSPIRE发PID增量
  │ INSPIRE释放持续0.15s；或FSR曾>=0.30N、随后<=0.15N持续0.30s
  ▼
RELEASE      模式0，反向电流回到init_pos
  │ 进入init_pos阈值
  ▼
RETURN_SETTLE 模式5保持，等待位置和触觉稳定
  ▼
FREE
```

`INIT` 不是固定的出厂零点。它采集本次安装/穿戴后的舵机放松位置和
FSR 绑带预载，RELEASE 回到本次 `init_pos`。

---

## 7. 安装和部署

### 7.1 Ubuntu PC

```bash
cd <dexEXO仓库根目录>/zh/PC端
python3 -m venv .venv
./.venv/bin/python -m pip install --upgrade pip
./.venv/bin/python -m pip install -r requirements.txt pymodbus
```

`zh/mhandpro/bin/mhandpro_diagnostic` 已提供时可直接使用。需重编译时：

```bash
cd <dexEXO仓库根目录>/zh/mhandpro
chmod +x build.sh
./build.sh
```

### 7.2 树莓派

开发电脑同步：

```bash
cd <dexEXO仓库根目录>
scp -r newteleop pi@192.168.3.76:/home/pi/cnn/
```

树莓派安装：

```bash
ssh pi@192.168.3.76
cd ~/cnn/newteleop
python3 -m venv .venv
./.venv/bin/python -m pip install --upgrade pip
./.venv/bin/python -m pip install -r requirements.txt
```

确认串口：

```bash
readlink -f /dev/serial0
ls -l /dev/serial0
```

当前为 1000000 baud、Protocol 2.0。

### 7.3 G1 板载机

```bash
cd <dexEXO仓库根目录>/zh
ROBOT=192.168.3.78
ssh unitree@$ROBOT 'mkdir -p ~/zh'
scp 机器人端/{robot_arm_receiver,teleop_protocol,waist_follow,hand_driver,hand_check,g1_ctl,g1_jog}.py \
    机器人端/start.sh \
    PC端/hand_mapping.py \
    unitree@$ROBOT:~/zh/
ssh unitree@$ROBOT 'chmod +x ~/zh/start.sh && rm -rf ~/zh/__pycache__'
```

G1 SSH 登录时如出现 `ros:foxy(1) noetic(2) ?`，需进入 shell 时选 `1`，
但接收端仍只用 `~/zh/start.sh`启动。

---

## 8. 每次联调前检查

### 网络

Ubuntu 和树莓派：

```bash
ping -c 3 192.168.3.78
```

G1：

```bash
ping -c 3 192.168.123.211
ping -c 3 192.168.123.210
```

### INSPIRE

G1：

```bash
cd ~/zh
python3 hand_check.py --side right
python3 hand_check.py --side left
```

### 外骨骼只读检查

树莓派，且其他舵机程序必须全部退出：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/tools/exo_dynamixel_probe.py --hand right
./.venv/bin/python exoskeleton/tools/exo_dynamixel_probe.py --hand left
```

应确认 ID 1～10 全部 Ping 成功、`torque=0`、`hw_error=0`。

---

## 9. 分阶段联调

### 阶段 A：双 FSR 只读

树莓派终端 A1（单进程双BLE）：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/FSR/ble_broker.py --hand both
```

树莓派终端 A2：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/FSR/exo_pressure_monitor.py --hand both
```

逐指按压，确认右手和左手各自五路没有串手、串指或断流。

### 阶段 B：不加 mHandPro 的双外骨骼力反馈

该阶段只用手按压 INSPIRE 指尖，不运行 Ubuntu 的 `dual_arm_viz.py`
和 `mhandpro_diagnostic`。

G1 终端 B1：

```bash
cd ~/zh
./start.sh --hand both --hand-only --hand-haptic
```

正常状态是：

```text
right手: 0%(张开)  left手: 0%(张开)
等待连接  发送端:无  [只跑手]  RX:0Hz
```

`RX:0Hz` 正常，因为没有 Ubuntu 遥操发送端。

G1 终端 B2：

```bash
ss -lnt | grep -E '9201|9202|9301|9302'
```

四个端口应全部 LISTEN。本阶段只会建立 9201/9202 客户端连接，9301/9302
保持 LISTEN 是正常的。

先用 `Ctrl+C` 停止阶段 A 手工启动的两个 BLE broker，由本地上位机
统一管理 BLE。树莓派桌面终端 B3 先只读：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/supervisor/local_supervisor.py \
  --force-host 192.168.3.78
```

G1 的 9201/9202 客户端会随上位机自动启动。点击 `1 连接十舵机`、
`2 连接双 FSR 蓝牙`，等 INSPIRE 状态自动变绿，再用 `STATUS` 确认
十指 FSR 与 INSPIRE 都不超时。点击 `QUIT` 退出后
用当前双手参数进入写入模式：

```bash
./.venv/bin/python exoskeleton/supervisor/local_supervisor.py \
  --enable-write \
  --enable-mhandpro \
  --force-host 192.168.3.78 \
  --haptic-host 192.168.3.78 \
  --right-drive-current-signs 1,1,1,1,1 \
  --left-drive-current-signs 1,1,1,1,1 \
  --max-goal-current 30 \
  --release-goal-current 30 \
  --actual-current-limit 45 \
  --current-slew 5 \
  --control-hz 10
```

树莓派本地桌面将显示双手 FSR、BLE/INSPIRE/覆盖链路以及
ID 1～10 的位置、电流、电压、温度和错误。不再启动网页端口。

在界面中按顺序点击：

```text
1 连接十舵机
2 连接双 FSR 蓝牙
INIT
ARM
```

INIT 时外骨骼放松、FSR 不受额外力、INSPIRE 指尖不按压。ARM 后从
右拇指到右小指，再从左拇指到左小指，每次只按一根 INSPIRE 指尖。
每指应按 `FREE → FORCE_ENTRY → LOCKED → RELEASE → RETURN_SETTLE → FREE`
转移。本阶段传入 `--enable-mhandpro`，因此会使用 9301/9302 和
INSPIRE PID 位置覆盖。

### 阶段 C：双 mHandPro → 双 INSPIRE 位置遥操

先停止阶段 B 的外骨骼写入程序。

G1 终端 C1：

```bash
cd ~/zh
./start.sh --hand both --hand-only
```

Ubuntu 终端 C2：

```bash
cd <dexEXO仓库根目录>/zh/PC端
./.venv/bin/python dual_arm_viz.py \
  --hand --hand-left \
  --udp-target 192.168.3.78:9527
```

Ubuntu 终端 C3：

```bash
cd <dexEXO仓库根目录>/zh/mhandpro
./bin/mhandpro_diagnostic
```

程序内确认显示 `手套已连接：BOTH`，然后输入：

```text
teleop both config/inspire_right_sim.cfg config/inspire_left_sim.cfg
ARM
```

两只手套共用一个 USB 接收器，只能运行一个 `mhandpro_diagnostic`进程。
检查右手套只动右 INSPIRE、左手套只动左 INSPIRE，然后输入 `STOP`。

### 阶段 D：双 mHandPro + 双 INSPIRE + 双外骨骼闭环

G1 终端 D1：

```bash
cd ~/zh
./start.sh --hand both --hand-only --hand-haptic
```

Ubuntu 按阶段 C 启动 `dual_arm_viz.py` 和单个 `mhandpro_diagnostic`，
执行 `teleop both ...` 并 ARM。

树莓派保持右/左 BLE broker，第三终端运行：

```bash
cd ~/cnn/newteleop
mkdir -p logs
./.venv/bin/python exoskeleton/force_control/dual_hand_force_test.py \
  --enable-write --enable-mhandpro \
  --force-host 192.168.3.78 \
  --haptic-host 192.168.3.78 \
  --right-drive-current-signs 1,1,1,1,1 \
  --left-drive-current-signs 1,1,1,1,1 \
  --force-to-current-gain 60 \
  --max-goal-current 300 \
  --release-goal-current 100 \
  --actual-current-limit 320 \
  --hand-total-current-limit 800 \
  --current-slew 20 \
  --force-step-max 4 \
  --control-hz 10 \
  2>&1 | tee "logs/force_test_$(date +%Y%m%d_%H%M%S).log"
```

输入 `STATUS → INIT → STATUS → ARM`。此时：

- FREE/FORCE_ENTRY：INSPIRE 继续跟手套。
- LOCKED：G1 锁定对应 INSPIRE 手指，并叠加树莓派 PID 位置增量。
- RELEASE/RETURN_SETTLE：INSPIRE 平滑恢复手套跟随，外骨骼回 INIT。
- 9301/9302 断开或 0.5 s 无覆盖包：G1 清除锁定并恢复手套跟随。

先逐指，再单手多指，最后双手同时接触。

### 阶段 E：加入 G1 双臂

只有阶段 D 稳定后才进入。G1 不再传 `--hand-only`：

```bash
cd ~/zh
python3 g1_ctl.py status
./start.sh --hand both --hand-haptic
```

Ubuntu 继续：

```bash
cd <dexEXO仓库根目录>/zh/PC端
./.venv/bin/python dual_arm_viz.py \
  --hand --hand-left \
  --udp-target 192.168.3.78:9527
```

保持人员离开双臂运动范围，按 `B` 启用双臂跟随。手和外骨骼仍按阶段 D 启动。

### 阶段 F：最后加入腰部

G1：

```bash
./start.sh --hand both --hand-haptic --waist
```

Ubuntu：

```bash
./.venv/bin/python dual_arm_viz.py \
  --hand --hand-left --waist \
  --udp-target 192.168.3.78:9527
```

双臂已进入跟随后，站直按 `W` 采腰部零位，再按 `Shift+W` 开启。
第一次限制在 ±15° 内，旁边必须有人扶护。

---

## 10. 常用命令与参数

### 单手力控

```bash
./.venv/bin/python exoskeleton/force_control/hand_force_test.py --hand right
./.venv/bin/python exoskeleton/force_control/hand_force_test.py --hand left
```

只用于单手调试。双手时不能同时开两个。

### 双手交互命令

| 命令 | 含义 |
|---|---|
| `STATUS` | 读取十指状态、INIT、FSR、INSPIRE和数据年龄 |
| `INIT` | 只读采集本次舵机放松位置和FSR预载 |
| `ARM` | 双手数据与INIT全部正常时才启用力控 |
| `STOP` | 十指目标电流清零、关扭矩、退出力控 |
| `QUIT` | 执行STOP清理并退出 |

### 当前双手力控命令与参数

`cd ~/cnn/newteleop` 切换到树莓派项目根目录。
`./.venv/bin/python` 使用项目虚拟环境的 Python；
`dual_hand_force_test.py` 是共享 `/dev/serial0` 的双手唯一入口。
完整的默认值、校验范围、计算公式和可省略项见
[`exoskeleton/force_control/README.md`](exoskeleton/force_control/README.md)。

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `--enable-write` | 开启 | 允许写Dynamixel和执行INIT/ARM；不传时只读 |
| `--enable-mhandpro` | 联调时开启 | 通过9301/9302向G1发送逐指力控覆盖 |
| `--force-host` | `192.168.3.78` | G1的INSPIRE指尖触觉发布主机 |
| `--haptic-host` | `192.168.3.78` | G1的mHandPro力控覆盖主机 |
| `--right/left-drive-current-signs` | `1,1,1,1,1` | 左右手`[拇,食,中,无,小]`收绳电流方向 |
| `--force-to-current-gain` | 60 | INSPIRE触觉 N 到请求电流 mA 的比例 |
| `--max-goal-current` | 300 | FORCE_ENTRY收绳电流上限 |
| `--release-goal-current` | 100 | RELEASE返回电流上限，接近INIT时自动减小 |
| `--actual-current-limit` | 320 | 实际电流故障阈值 |
| `--hand-total-current-limit` | 800 | 每只手FORCE_ENTRY五指收绳总电流上限 |
| `--current-slew` | 20 | 每控制周期最大电流增量 |
| `--force-step-max` | 4 | LOCKED中每周期最大位置修正（tick） |
| `--control-hz` | 10 | 树莓派状态机和十舵机读写频率，当前用于降低TTL丢包 |
| `--release-hold-seconds` | 0.15 | INSPIRE触觉释放消抖时间 |
| `--fsr-load-threshold` | 0.30N | FSR相对INIT新增力达到该值才确认本轮加载过 |
| `--fsr-release-threshold` | 0.15N | 已加载FSR降到该值以下才开始卸载计时 |
| `--fsr-release-hold-seconds` | 0.30s | FSR卸载条件必须连续成立的时间 |
| `2>&1` | — | 把错误输出合并进正常输出，确保FAULT被记录 |
| `mkdir -p logs` | — | 首次运行时创建日志目录，已存在时不报错 |
| `tee "logs/force_test_$(date +%Y%m%d_%H%M%S).log"` | — | 终端显示的同时按实验启动日期时间生成独立日志 |

`--control-hz 10` 与 G1 的 `--hand-touch-hz 10` 不同：前者是
Dynamixel力控频率，后者是INSPIRE触觉采样频率。确认供电和TTL总线
在20Hz下长时间无 `no status packet` 后，才建议把前者提高到20。
`logs/force_test_20260827_143205.log` 表示该次实验在
2026-08-27 14:32:05启动。时间戳文件名避免了`tee`覆盖旧日志。

---

## 11. 停机顺序

1. 树莓派双手力控终端输入 `STOP`。
2. 确认十指不再收绳，输入 `QUIT`。
3. Ubuntu `mhandpro_diagnostic` 的 teleop 内输入 `STOP`。
4. Ubuntu 退出 `dual_arm_viz.py`。
5. 树莓派 Ctrl+C 停止两个 BLE broker。
6. G1 的 `start.sh` 终端 Ctrl+C，等待显示安全退出。
7. 最后断开外骨骼电源和传感器电源。

异常时优先物理断开外骨骼电源，不必等待软件顺序。

---

## 12. 故障排查

| 现象 | 检查 |
|---|---|
| `Port is in use` | 有两个进程在访问 `/dev/serial0`；退出所有probe/单手/双手程序后重启 |
| BLE参数报 `unrecognized arguments: --hand` | 树莓派仍是旧版 `ble_broker.py`，重新同步 |
| FSR右手连不上 | 确认MAC `...85:B3`、没有其他Bleak进程占用 |
| FSR左手连不上 | 确认MAC `...67:3B`、供电和蓝牙适配器 |
| `Inspire valid=False` | G1是否加 `--hand-haptic`；9201/9202是否LISTEN/ESTAB；INSPIRE是否可Ping |
| 9301/9302没有ESTAB | 不加mHandPro时正常；全闭环时确认树莓派加了 `--enable-mhandpro` |
| G1显示 `RX:0Hz` | 不加Ubuntu遥操时正常；加mHandPro时检查UDP 9527和`--udp-target` |
| 按右手却驱动左手 | 检查9001/9002、9201/9202、ID表和BLE MAC是否交换 |
| 按某指却驱动另一指 | 运行 `fsr_channel_identifier.py`，核对 `[拇,食,中,无,小]` |
| ARM被拒绝 | 先STATUS；十指都必须有新鲜FSR/INSPIRE数据并全部INIT |
| 输入ARM后立即回到提示符且没有输出 | 旧版双手入口会静默拒绝前置异常；同步新版 `dual_hand_force_test.py` 和 `left_hand_force_test.py`，再用STATUS看具体原因 |
| ARM成功但舵机不立即动 | 正常，此时为FREE待机；按压INSPIRE单指，换算力达到`contact_on`(默认0.50N)后才进入FORCE_ENTRY |
| LOCKED后松开不立即RELEASE | INSPIRE需低于`contact_off`持续0.15s；FSR路径则必须先加载到0.30N，再降至0.15N以下持续0.30s |
| FORCE_ENTRY收绳慢 | 先查绳路/摩擦/电源；不要跳过单指试验直接大幅增流 |
| G1端四端口不LISTEN | 确认新版代码已同步，启动命令包含 `--hand-haptic` |
| G1登录后ROS环境冲突 | 接收端必须用 `~/zh/start.sh`，删除 `~/zh/__pycache__` 后重试 |

---

## 13. 验证与代码修改后检查

语法检查：

```bash
python3 -m py_compile \
  newteleop/exoskeleton/FSR/*.py \
  newteleop/exoskeleton/force_control/*.py \
  zh/机器人端/hand_driver.py \
  zh/机器人端/robot_arm_receiver.py
```

不连硬件验证双手 ARM 前置检查：

```bash
python3 -m unittest \
  newteleop/exoskeleton/force_control/test_dual_hand_force_control.py -v
```

G1 手部纯逻辑测试（在 `zh/PC端` 虚拟环境）：

```bash
cd zh/PC端
./.venv/bin/python -m unittest \
  test_robot_hand_driver.ModbusFrameTest \
  test_robot_hand_driver.ConversionTest \
  test_robot_hand_driver.FollowerTest \
  test_robot_hand_driver.HapticOverrideTest \
  test_robot_hand_driver.ReceiverWiringTest
```

完整 `zh` 测试还需 `numpy/scipy/pygame/PyOpenGL/pymodbus`，并有部分本地 socket 测试。
「没有执行」不等于「测试失败」；移交时应记录测试环境、跳过原因和真正的断言失败。

---

## 14. 详细子文档

- [`raspberry.md`](raspberry.md)：树莓派 SSH、环境、多终端、设备检查和全部树莓派操作步骤。
- [`exoskeleton/README.md`](exoskeleton/README.md)：Dynamixel、硬件工具和安全测试。
- [`exoskeleton/FSR/README.md`](exoskeleton/FSR/README.md)：BLE、FSR帧、双手监视和通道识别。
- [`exoskeleton/force_control/README.md`](exoskeleton/force_control/README.md)：单指/五指/双手状态机和参数。
- [`exoskeleton/HAPTIC_FEEDBACK_PLAN_CN.md`](exoskeleton/HAPTIC_FEEDBACK_PLAN_CN.md)：设计依据、力学量和风险。
- [`exoskeleton/LOCAL_SUPERVISOR_CN.md`](exoskeleton/LOCAL_SUPERVISOR_CN.md)：树莓派本地桌面上位机和安全按钮。
- [`exoskeleton/UPPER_COMPUTER_PLAN_CN.md`](exoskeleton/UPPER_COMPUTER_PLAN_CN.md)：只读上位机、遥测协议、界面和后续路线。
- [`mhandpro/README.md`](mhandpro/README.md)：树莓派单手直连INSPIRE历史台架路径。
- [`../zh/使用说明.md`](../zh/使用说明.md)：G1双臂、腰部、双手的完整操作手册。
- [`../zh/机器人端/部署到真机.md`](../zh/机器人端/部署到真机.md)：G1网络、DDS环境和部署。

---

## 15. 移交时必须随代码记录的实物信息

每次更换硬件或网络后，更新本节和对应子 README：

- G1 板载机 Wi-Fi IP（当前 `192.168.3.78`）。
- 树莓派 Wi-Fi IP（当前 `192.168.3.76`）。
- 两只 INSPIRE 的 IP 与左右手标识。
- 两个 BLE MAC 与左右手标识。
- Dynamixel ID、型号、电流单位和收绳符号。
- FSR 五路顺序和最近一次逐指复测结果。
- 左/右手最近的安全电流档位、机械行程和异常日志。

接手人如果只记住一条：**G1 唯一控制 INSPIRE，树莓派唯一控制 ID 1～10，
任何时候都不要为同一硬件启动第二个驱动进程。**
