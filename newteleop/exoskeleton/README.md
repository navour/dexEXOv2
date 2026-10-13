# 双手外骨骼与 PyQt 上位机

> 更新：2026-10-09
>
> 运行位置：树莓派桌面；一个进程控制十个 XL330。

---

## 目录

1. [系统架构](#系统架构)
2. [端口分配](#端口分配)
3. [文件清单](#文件清单)
4. [首次安装](#首次安装)
5. [启动顺序](#启动顺序)
6. [配置参数](#配置参数)
7. [常见问题](#常见问题)

---

## 系统架构

`local_supervisor.py` 与 `DualHandController` 同进程，十指共享串口和互斥锁。GUI 自动接收 G1 触觉，点击蓝牙按钮后管理自己的 broker 子进程。
目前没有网页上位机，也不监听 8080。Ubuntu 浏览器不能直接打开此 GUI；请使用树莓派桌面或 VNC。

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

| 路径 | 作用 |
|---|---|
| `supervisor/local_supervisor.py` | 正式双手 PyQt 入口 |
| `force_control/dual_hand_force_test.py` | 双手 CLI，不能与 GUI 同时运行 |
| `FSR/ble_broker.py` | BLE 唯一连接者 |
| `tools/exo_dynamixel_probe.py` | 只读 Ping、模式、错误检查 |
| `tools/exo_single_servo_current_test.py` | 单舵机短脉冲方向检查 |
| `tools/exo_single_servo_jog.py` | 单舵机小步点动 |
| `tools/exo_capture_neutral.py` | 维护参考位，不能代替每轮 INIT |
| `config/` | ID 映射及维护记录 |

历史台架细节保留于 [LEGACY_BENCH_NOTES_CN.md](LEGACY_BENCH_NOTES_CN.md)，其网络和参数不是当前启动依据。

---

## 首次安装

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

G1：

```bash
cd ~/zh
bash start.sh --hand both --hand-only --hand-haptic --hand-range 0.3
```

树莓派先只读：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/supervisor/local_supervisor.py --force-host 192.168.3.78
```

退出只读后再低电流写入：

```bash
./.venv/bin/python exoskeleton/supervisor/local_supervisor.py \
  --enable-write \
  --force-host 192.168.3.78 \
  --right-drive-current-signs 1,1,1,1,1 \
  --left-drive-current-signs 1,1,1,1,1 \
  --max-goal-current 30 --release-goal-current 30 \
  --actual-current-limit 45 --current-slew 5 --control-hz 10
```

1. 退出其他外骨骼控制器和手工启动的 BLE broker，避免争用串口与蓝牙。
2. G1 启动手部服务，再在树莓派桌面启动上位机。
3. 点击 `1 连接十舵机`，然后 `2 连接双 FSR 蓝牙`。
4. 等双手 FSR 和 INSPIRE 数据有效；只读模式不需要力控覆盖在线。
5. 先逐指按压 FSR 确认通道，再按压 INSPIRE 确认对应目标力变化。
6. 退出只读程序，用低电流写入命令重启。机构未穿戴、绳索放松、FSR 无额外按压、INSPIRE 无接触时点击 `INIT`。
7. 就绪清单通过后点击 `ARM`；依次只按一根 INSPIRE 指尖，观察对应舵机与 FSR。
8. 正常状态为 `FREE → FORCE_ENTRY → LOCKED → RELEASE → RETURN_SETTLE → FREE`。低电流无法拉紧或 FSR 未加载时不会进入 LOCKED，不应据此直接增大电流。
9. 异常按 `STOP` 或 `Esc`；退出用 `QUIT`。软件停止不能代替物理断电。

---

## 配置参数

串口默认为 `/dev/serial0`，波特率 1000000。只读探测也必须在其他舵机程序退出后进行：

```bash
./.venv/bin/python exoskeleton/tools/exo_dynamixel_probe.py --hand right
./.venv/bin/python exoskeleton/tools/exo_dynamixel_probe.py --hand left
```

未连接 INSPIRE 时可看串口和 FSR，但不能 ARM 力控。单舵机方向实验使用专用工具的 `--help`，不要绕过力控数据联锁。
正式覆盖联调在上述 GUI 命令加 `--enable-mhandpro --haptic-host 192.168.3.78`，PC 手套遥操按 zh 现有流程启动；该开关本身不检查手套数据是否有效。

---

## 常见问题

`INIT` 记录本次舵机位置和 FSR 静息中位数；正常 RELEASE 返回该位置，不读取固定 neutral JSON。左右手初始化都成功才允许双手 ARM。
低电流收绳但尚无 FSR 加载时保持 FORCE_ENTRY；先检查绳索与 FSR 受力路径。
所有异常停止、退出和链路中断都要在未穿戴时验证。更多排查见 [主 README](../README.md)。
