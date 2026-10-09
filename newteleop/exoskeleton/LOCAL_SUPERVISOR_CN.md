# 树莓派 PyQt 本地外骨骼上位机

> 版本：2026-09-01  
> 入口：`exoskeleton/supervisor/local_supervisor.py`

## 1. 定位

这是运行在树莓派本地桌面/VNC 中的 PyQt5 上位机，不再使用
Tk，正式路径也不需要浏览器。视觉上沿用 `zh` 调试工具的深色工程
HUD，界面包含：

- 顶部全局状态、写入授权和时钟。
- 左侧的硬件连接顺序、ARM 就绪清单和安全命令。
- 右/左手各五指 FSR、INSPIRE 力、状态机和链路状态。
- Dynamixel ID 1～10 的位置、电流、电压、温度、模式和错误表。
- 右/左手 FSR 与舅机电流的 30 s 实时曲线。
- `STATUS / INIT / ARM / STOP / QUIT`、`Esc=STOP`。

GUI 与 `DualHandController` 在同一进程，因此 `/dev/serial0` 仍然只有
一个所有者。Qt 主线程只绘制已缓存遥测；慢速硬件操作在工作线程中运行。
实时卡片以 10 Hz 刷新，表格和曲线以 2 Hz 且仅在对应页签可见时
重绘；曲线仍保留过去 30 s 数据。这些显示频率不改变力控频率。

## 2. 按钮边界

- `1 连接十舅机`：打开 `/dev/serial0`，检查 ID 1～10。
- `2 连接双 FSR 蓝牙`：GUI 启动并且只管理自己的
  `ble_broker.py --hand both` 子进程，然后连接 9001/9002。
- G1 链路不再设置按钮：GUI 启动时自动建立树莓派端 9201/9202
  数据接收；开启 `--enable-mhandpro` 时同时自动连接 9301/9302。
  G1 服务未启动时保持等待并自动重连，不影响界面启动。
- `--enable-mhandpro` 的历史命名容易误解：它实际开启的是
  **G1 INSPIRE 力控覆盖**，并不检查 mHandPro 手套采集程序是否在运行。
  只要 9301/9302 已连接，`LOCKED` 中的力差 PID 就可以直接改变
  INSPIRE 单指位置。若本次只想读取 INSPIRE 触觉而不允许它运动，
  启动树莓派上位机时不要传该参数。
- 界面的 `LOCKED` 是树莓派本地力控状态；`覆盖TCP在线` 只证明
  9301/9302 连接存在。当前协议没有G1执行回执，不能把TCP在线等同于
  INSPIRE已经执行了位置修正。

G1 按钮不会通过 SSH 远程运行机器人程序。G1 上的 `~/zh/start.sh`
必须已经运行，否则界面会显示 INSPIRE/覆盖等待连接。

## 3. 树莓派首次安装

PyQt5 在树莓派 ARM 上通过 Raspberry Pi OS 的预编译包安装。在树莓派执行：

```bash
sudo apt update
sudo apt install -y python3-venv python3-pyqt5 python3-pyqtgraph

cd ~/cnn/newteleop
python3 -m venv --system-site-packages .venv
./.venv/bin/python -m pip install --upgrade pip
./.venv/bin/python -m pip install -r requirements.txt
./.venv/bin/python -c 'import PyQt5, pyqtgraph, bleak, dynamixel_sdk; print("上位机依赖正常")'
```

如果 `.venv` 原来已存在，重新执行上面的
`python3 -m venv --system-site-packages .venv` 会更新其配置，不需要删除虚拟环境。

## 4. 启动

### 4.1 只读看界面

在树莓派本地桌面终端或 VNC 终端执行：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/supervisor/local_supervisor.py
```

可以检查十舅机、BLE 和 G1 数据，但 ARM 保持锁定。

### 4.2 低电流力控启动

```bash
cd ~/cnn/newteleop
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

这个终端只用来承载 GUI 进程；GUI 打开后，不需要再运行
`dual_hand_force_test.py` 或另一个 BLE broker。如果想点图标而不打终端，
可以后续添加 `.desktop` 启动器；无论从哪里启动，程序都必须作为一个
本地进程运行。

## 5. 正式操作顺序

1. 先在 G1 上运行
   `cd ~/zh && ./start.sh --hand both --hand-haptic`，确认 hand driver 已启动。
2. 在树莓派桌面启动 PyQt 上位机。
3. 点击 `1 连接十舅机`。
4. 点击 `2 连接双 FSR 蓝牙`，等右/左 BLE/FSR 均变绿。
5. 等右/左 INSPIRE 和力控覆盖自动变绿。
6. 机构未穿戴、双手完全放松时点击 `INIT`；不再弹出二次确认框。
7. 确认十指 INIT 完成、就绪清单全部通过，点击 `ARM`；不再弹出二次确认框。
8. 任何异常立即点击 `STOP` 或按 `Esc`，必要时物理断电。
9. 退出用 `QUIT`；程序会先 STOP，再关闭串口和由 GUI 启动的 BLE 进程。

`ARM` 只在以下条件全部成立时可用：显式授权写入、十舅机串口
已打开、双手 FSR 与 INSPIRE 数据新鲜、已启用的覆盖链路正常、十指 INIT
完整且无 FAULT。

## 6. 常见问题

| 现象 | 原因与处理 |
|---|---|
| `no display name and no $DISPLAY` | 在纯 SSH 会话里启动了 GUI。请在树莓派本地桌面/VNC 终端运行。 |
| `No module named PyQt5` | 安装 `python3-pyqt5`，并用 `--system-site-packages` 更新 `.venv`。 |
| `Could not load the Qt platform plugin xcb` | 补齐树莓派桌面/Qt 系统包，不要在纯 SSH 终端强制伪造 `DISPLAY`。 |
| BLE/FSR 一直红色 | 检查两块 STM32 上电、MAC 配置和蓝牙适配器；不要另起 broker 抢占 BLE。 |
| 终端按 `Ctrl+C` 后 FSR 拒绝连接 | `Ctrl+C` 会停止 GUI 管理的 broker；新版会把终端中断转换成完整 STOP/退出，不要在旧GUI仍运行时反复中断。 |
| INSPIRE 一直红色 | 检查 G1 `~/zh/start.sh`、`192.168.3.78` 连通性与 9201/9202。 |
| 显示 `LOCKED` 但机构没有建立力 | 新版要求外骨骼FSR相对INIT增加至少加载阈值后才允许LOCKED；同时检查INSPIRE力是否超过contact-on以及FORCE_ENTRY目标/实际电流。 |
| ARM 按钮灰色 | 直接查看左侧“ARM 就绪清单”和其下方的锁定原因。 |

## 7. 安全限制

- 不要同时运行上位机和另一个 `dual_hand_force_test.py`。
- 使用 GUI 管理 BLE 时，不要事先手工运行另一个
  `ble_broker.py --hand both`。
- 软件 STOP 不代替硬件急停和物理断电。
- `--fsr-rest-max` 保持默认 8 N；没有重复标定依据时不要提高到 55 N。
