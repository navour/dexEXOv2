# newteleop — mHandPro 与 G1 双手 INSPIRE 力反馈遥操

> 更新：2026-10-09
>
> 运行位置：Ubuntu PC + G1 板载机 + 树莓派；旧 ftp 为算法参考，zh 为最新手部实现来源。

---

## 目录

1. [系统架构](#系统架构)
2. [端口分配](#端口分配)
3. [文件清单](#文件清单)
4. [首次部署](#首次部署)
5. [启动顺序](#启动顺序)
6. [配置参数](#配置参数)
7. [状态机说明](#状态机说明)
8. [常见问题](#常见问题)

---

## 系统架构

```text
双 mHandPro / 一个 USB 接收器
  → PC: zh/mhandpro/bin/mhandpro_diagnostic
  → TCP 9103/9104 → zh/PC端/dual_arm_viz.py
  → UDP 9527 → G1: zh/机器人端/robot_arm_receiver.py --hand-only
                         ├─ hand_driver.py → Modbus → 双 INSPIRE
                         ├─ TCP 9201/9202 → Pi 双手触觉输入
                         └─ TCP 9301/9302 ← Pi 力差位置修正
双 STM32 FSR → Pi BLE broker → TCP 9001/9002
                         ↓
Pi: PyQt local_supervisor / DualHandController → XL330 ID 1～10
```

当前代码保持原有分工：`zh/mhandpro` 和 `zh/PC端` 负责手套及上位遥操，`zh/机器人端` 负责 G1 和 INSPIRE，`newteleop/exoskeleton` 负责双手外骨骼反馈。
`newteleop/mhandpro` 保留早期树莓派网线直连台架。没有新增独立整合包，也未修改外骨骼代码。
G1 手部仅有一个 Modbus 所有者，不同时启动 ftp DDS 手桥或 Headless 手驱动。

文档整理不等于真机验证；双机通信、方向和穿戴闭环仍需现场验收。

---

## 端口分配

| 接口 | 右手 | 左手 | 方向与作用 |
|---|---|---|---|
| mHandPro TCP | 9103 | 9104 | 采集程序 → PC dual_arm_viz |
| FSR TCP | 9001 | 9002 | Pi BLE broker → 本地控制器 |
| INSPIRE 触觉 TCP | 9201 | 9202 | G1 → Pi |
| 力控覆盖 TCP | 9301 | 9302 | Pi → G1，仅启用覆盖时连接 |
| INSPIRE Modbus | 192.168.123.211:6000 | 192.168.123.210:6000 | G1 → 实体手 |
| Dynamixel ID | 1～5 | 6～10 | Pi 共用 /dev/serial0，1 Mbps |

PC → G1 统一使用 UDP 9527（UA2M + UHND）。示例 G1 IP 为 `192.168.3.78`，Pi 为 `192.168.3.76`，请按实际网络修改。
五指顺序为 `[拇, 食, 中, 无, 小]`；INSPIRE 六槽为 `[小, 无, 中, 食, 拇弯, 拇对掌]`，力控映射 `[4,3,2,1,0]`。

---

## 文件清单

| 路径 | 运行位置 | 用途 |
|---|---|---|
| `../zh/机器人端/` | G1 | 接收器、Modbus 驱动、触觉与覆盖服务 |
| `../zh/PC端/` | Ubuntu | IMU、双臂/腰部可视化与双手闭合度发送 |
| `../zh/mhandpro/` | Ubuntu | 现有双手采集、标定和SDK |
| `exoskeleton/supervisor/` | Pi 桌面 | PyQt 上位机与 BLE 子进程管理 |
| `exoskeleton/force_control/` | Pi | 双手共享串口、逐指状态机 |
| `exoskeleton/FSR/` | Pi | BLE 转发、只读监视、通道识别 |
| `exoskeleton/tools/` | Pi | 舵机探测、点动、参考位采集 |
| `mhandpro/` | 独立台架 | 旧 Pi 直连 INSPIRE 实验，非 G1 主入口 |
| `computer_debug/` | Ubuntu | 厂商 SDK 示例参考 |
| `test/` | Pi / 开发机 | 拉绳实验与离线测试 |

全仓库目录关系见 [ARCHITECTURE.md](ARCHITECTURE.md)，离线检查结果见 [VALIDATION.md](VALIDATION.md)。

---

## 首次部署

开发电脑从仓库根目录执行：

```bash
ssh unitree@192.168.3.78 'mkdir -p ~/zh'
scp zh/机器人端/*.py zh/机器人端/start.sh zh/PC端/hand_mapping.py \
  unitree@192.168.3.78:~/zh/
ssh pi@192.168.3.76 'mkdir -p ~/cnn/newteleop'
scp -r newteleop/exoskeleton newteleop/requirements.txt pi@192.168.3.76:~/cnn/newteleop/
```

G1 保留已有 Unitree SDK 和 CycloneDDS 配置。`zh/机器人端/start.sh` 使用现有环境隔离，默认 SDK 为 `/home/unitree/workspace/zhw_workspace`，DDS 配置为 `/home/unitree/cyclonedds_ws/cyclonedds.xml`。只测试手也使用该启动脚本。

Ubuntu PC 从仓库根目录执行：

```bash
python3 -m venv zh/PC端/.venv
zh/PC端/.venv/bin/python -m pip install -r zh/PC端/requirements.txt
bash zh/mhandpro/build.sh
```

在树莓派桌面或 VNC 终端执行（首次）：

```bash
sudo apt update
sudo apt install -y python3-venv python3-pyqt5 python3-pyqtgraph
cd ~/cnn/newteleop
python3 -m venv --system-site-packages .venv
./.venv/bin/python -m pip install -r requirements.txt
./.venv/bin/python -c 'import PyQt5, pyqtgraph, bleak, dynamixel_sdk; print("依赖正常")'
```

已有虚拟环境也可执行同一条 `venv --system-site-packages` 更新配置。GUI 需要桌面会话；纯 SSH 可运行命令行控制器。
串口权限不足时检查用户的 `dialout` 组和设备权限；默认串口 `/dev/serial0`，USB 转接器需显式传 `--device /dev/ttyUSB0`。

---

## 启动顺序

### 阶段一：上位机只读检查

G1 终端：

```bash
cd ~/zh
bash start.sh --hand both --hand-only --hand-haptic --hand-range 0.3
```

只开手、不启用手臂或腰部；无 PC 输入时 INSPIRE 保持张开，`RX:0Hz` 正常。
G1 手服务启动会写张手目标，“只读”仅指下面的外骨骼程序。

树莓派桌面终端：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/supervisor/local_supervisor.py --force-host 192.168.3.78
```

### 阶段二：低电流外骨骼测试

退出只读程序后在同一树莓派桌面终端运行：

```bash
./.venv/bin/python exoskeleton/supervisor/local_supervisor.py \
  --enable-write \
  --force-host 192.168.3.78 \
  --right-drive-current-signs 1,1,1,1,1 \
  --left-drive-current-signs 1,1,1,1,1 \
  --max-goal-current 30 --release-goal-current 30 \
  --actual-current-limit 45 --current-slew 5 --control-hz 10
```

此阶段不需要 mHandPro，不启用 INSPIRE 力控覆盖。GUI 自己管理双 BLE broker 和 9201/9202 输入，不另起力控程序。

1. 退出其他外骨骼控制器和手工启动的 BLE broker，避免争用串口与蓝牙。
2. G1 启动手部服务，再在树莓派桌面启动上位机。
3. 点击 `1 连接十舵机`，然后 `2 连接双 FSR 蓝牙`。
4. 等双手 FSR 和 INSPIRE 数据有效；只读模式不需要力控覆盖在线。
5. 先逐指按压 FSR 确认通道，再按压 INSPIRE 确认对应目标力变化。
6. 退出只读程序，用低电流写入命令重启。机构未穿戴、绳索放松、FSR 无额外按压、INSPIRE 无接触时点击 `INIT`。
7. 就绪清单通过后点击 `ARM`；依次只按一根 INSPIRE 指尖，观察对应舵机与 FSR。
8. 正常状态为 `FREE → FORCE_ENTRY → LOCKED → RELEASE → RETURN_SETTLE → FREE`。低电流无法拉紧或 FSR 未加载时不会进入 LOCKED，不应据此直接增大电流。
9. 异常按 `STOP` 或 `Esc`；退出用 `QUIT`。软件停止不能代替物理断电。

### 阶段三：双 mHandPro 位置遥操

先停止外骨骼写入测试。使用现有 `zh/PC端/dual_arm_viz.py` 与 `zh/mhandpro/mhandpro_diagnostic`，操作与标定说明见 [zh 使用说明](../zh/使用说明.md) 和 [mHandPro README](../zh/mhandpro/README.md)。

从仓库根目录启动 PC 上位机：

```bash
zh/PC端/.venv/bin/python zh/PC端/dual_arm_viz.py \
  --hand --hand-left --udp-target 192.168.3.78:9527
```

另一个终端启动现有采集程序：

```bash
cd zh/mhandpro
./bin/mhandpro_diagnostic
```

一个 USB 接收器只运行一个采集进程。采集端 teleop 必须与 PC 读取同一对 cfg；PC 现有默认是 `config/inspire_right_sim.cfg` 与 `config/inspire_left_sim.cfg`。
这些配置包含仿真用的宽松磁状态和断流设置，不能将其视为已经完成真机保护配置；真机操作前按 zh 说明核对标定、配置和行程。

### 阶段四：力控遥操

位置遥操正常后，以阶段二的低电流命令重启上位机，并额外加入：

```text
--enable-mhandpro --haptic-host 192.168.3.78
```

确认 9301/9302 在线后再 `INIT → ARM`。此时 LOCKED 的力差会改变 INSPIRE 指令。
`--enable-mhandpro` 是历史名称，实际含义是“启用 INSPIRE 力控覆盖”，不负责启动手套。

联合双臂/腰部继续使用同一 zh 上位机，G1 按 zh 手册分阶段启用相应功能；同一时间只保留一个接收器和一个 PC 发送端。

---

## 配置参数

| 参数 | 代码默认 | 首测使用 | 含义 |
|---|---:|---:|---|
| `--enable-write` | 关 | 显式开启 | 外骨骼寄存器写入权限 |
| `--enable-mhandpro` | 关 | 外骨骼独测保持关 | INSPIRE 覆盖 |
| `--max-goal-current` | 300 | 30 | 单指收绳电流上限 raw |
| `--release-goal-current` | 100 | 30 | 回位电流上限 raw |
| `--actual-current-limit` | 320 | 45 | 实测电流故障阈值 raw |
| `--current-slew` | 20 | 5 | 每周期电流上升上限 raw |
| `--control-hz` | 20 | 10 | 每手调度频率 Hz |
| `--fsr-rest-max` | 8 | 8 | INIT 预载上限 N |
| `--force-to-current-gain` | 60 | 60 | INSPIRE 力 → 电流 raw/N |

两组 `--*-drive-current-signs` 按五指顺序填写，写模式必填。示例全 +1 来自现有机构记录，换绳路后须重新验证。电流以 raw 为准，现有 XL330-M288 记录约为 1 mA/raw。
不要用 `--fsr-rest-max 55` 放行约 49N 的异常 FSR；先检查传感器和焊接。

---

## 状态机说明

```text
STOP --INIT / ARM--> FREE --INSPIRE接触--> FORCE_ENTRY
FORCE_ENTRY --FSR已加载且位置稳定--> LOCKED
LOCKED --INSPIRE释放或FSR已加载后卸载--> RELEASE
RELEASE --返回本次INIT位置--> RETURN_SETTLE --稳定--> FREE
任意通信/电流/硬件故障 → 十指联锁STOP
```

FORCE_ENTRY 使用限流收绳；LOCKED 使用模式5保持外骨骼位置。
INSPIRE 位置修正是 `clamp(Kp × (FSR反馈力 − INSPIRE目标力), ±force_step_max)`，G1 从对应角度槽减去该增量；正误差使手指更闭合。
当前实现只有比例项，不是带 I、D 的完整 PID。FSR 反馈遵循旧系统的底值与本次 INIT 去皮，不能直接把 GUI 原始 FSR 当作闭环误差。

覆盖断流超过 0.5s 恢复手套跟随。现有 zh 驱动的有效 LOCKED 覆盖仍可能覆盖手套失效后的张手目标，且覆盖修正不受 `--hand-range` 的同等约束；本次仅整理文档，未修改该行为。独测外骨骼时不启用覆盖；结束双边测试先 STOP 外骨骼。

---

## 常见问题

| 现象 | 检查 |
|---|---|
| GUI 无法显示 | 在 Pi 本地桌面/VNC 运行，检查 Qt 与虚拟环境 |
| 十舵机连接失败 | 供电、ID1～10、串口、1Mbps、其他占用进程 |
| BLE 端口被占 | 退出手工 broker，让 GUI 管理唯一进程 |
| INSPIRE 数据无效 | G1 服务、9201/9202、内部 .210/.211 网络 |
| ARM 灰色 | 看就绪清单；只读或缺任一手数据都不能 ARM |
| 不进入 LOCKED | 看 FSR 是否确实加载、位置是否稳定；低电流可能不足以拉紧 |
| 只显示覆盖在线 | TCP 在线不是 G1 实际执行回执 |

停机：先 GUI `STOP → QUIT`，再采集程序 `STOP`，最后 Ctrl+C 停 PC 转发与 G1 服务。
详细操作：[外骨骼 README](exoskeleton/README.md)、[本地上位机](exoskeleton/supervisor/LOCAL_SUPERVISOR_CN.md)、[力控 README](exoskeleton/force_control/README.md)。
