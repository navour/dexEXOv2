# 树莓派 SSH 与 newteleop 完整操作手册

> 更新：2026-08-06
>
> 树莓派用户：`pi`，Wi-Fi IP：`192.168.3.76`
>
> 代码目录：`/home/pi/cnn/newteleop`
>
> 本文档只说树莓派上的操作。G1 和 Ubuntu 的联调命令见 [`README.md`](README.md)。

树莓派负责：

- 独占连接左/右 STM32 BLE 板。
- 把右手 FSR 发布到 `127.0.0.1:9001`。
- 把左手 FSR 发布到 `127.0.0.1:9002`。
- 通过 `/dev/serial0` 控制 Dynamixel ID 1～10。
- 从 G1 `192.168.3.78:9201/9202` 接收左/右 INSPIRE 触觉。
- 全闭环时向 G1 `9301/9302` 发送力控状态和 INSPIRE 位置增量。

---

## 1. SSH 登录

### 1.1 开发电脑上检查网络

Ubuntu 开发电脑和树莓派应连接同一个 `192.168.3.x` 网络。

```bash
ping -c 3 192.168.3.76
```

如果不通：

1. 确认树莓派已开机。
2. 确认开发电脑与树莓派在同一 Wi-Fi/路由器。
3. 在树莓派本地终端执行 `ip -br addr`，检查 `wlan0` 的实际 IP。
4. IP 变化后，本文所有 `192.168.3.76` 都要替换。

### 1.2 密码登录

```bash
ssh pi@192.168.3.76
```

首次连接出现主机指纹提示时，核对设备后输入：

```text
yes
```

然后输入树莓派密码。密码不应写入 README、脚本或 Git。

登录后提示符应类似：

```text
pi@pi:~ $
```

### 1.3 配置 SSH 免密登录

在 Ubuntu 开发电脑执行：

```bash
ssh-keygen -t ed25519
ssh-copy-id pi@192.168.3.76
```

已有 SSH 密钥时不需重复生成，只执行 `ssh-copy-id`。验证：

```bash
ssh -o BatchMode=yes pi@192.168.3.76 'hostname && whoami'
```

应输出树莓派主机名和 `pi`。

### 1.4 退出 SSH

```bash
exit
```

不要在力控 ARM 状态下直接关闭 SSH 窗口。应先在力控程序中输入
`STOP`、`QUIT`。

---

## 2. 登录后的基础检查

```bash
cd ~/cnn/newteleop
pwd
uname -m
python3 --version
ls -la
```

应确认：

- `pwd` 为 `/home/pi/cnn/newteleop`。
- `uname -m` 为 `aarch64`。
- 存在 `.venv`、`exoskeleton`、`mhandpro`、`README.md`、`raspberry.md`。

检查关键程序：

```bash
ls -l \
  exoskeleton/FSR/ble_broker.py \
  exoskeleton/FSR/exo_pressure_monitor.py \
  exoskeleton/force_control/dual_hand_force_test.py
```

---

## 3. Python 虚拟环境

### 3.1 首次安装（包含 PyQt 上位机）

```bash
sudo apt update
sudo apt install -y python3-venv python3-pyqt5 python3-pyqtgraph

cd ~/cnn/newteleop
python3 -m venv --system-site-packages .venv
./.venv/bin/python -m pip install --upgrade pip
./.venv/bin/python -m pip install -r requirements.txt
```

Qt 使用 Raspberry Pi OS 预编译包，所以虚拟环境必须使用
`--system-site-packages`。已有 `.venv` 时可直接重新执行这条 `venv`
命令更新配置，不用删除原环境。

`requirements.txt` 当前包含：

- `dynamixel-sdk`：外骨骼 TTL 总线。
- `bleak`：STM32 BLE 连接。
- `pymodbus`：保留的树莓派直连 INSPIRE 台架工具。

### 3.2 检查依赖

```bash
./.venv/bin/python -c 'import PyQt5, pyqtgraph, bleak, dynamixel_sdk; print("Python依赖正常")'
```

### 3.3 运行时不必 `source activate`

文档命令统一使用：

```bash
./.venv/bin/python <脚本>
```

这样可避免不同 SSH 终端忘记激活虚拟环境。

---

## 4. 代码备份和更新

### 4.1 从 Ubuntu 安全同步整个 newteleop（推荐）

先在树莓派 GUI 中点击 `STOP`、`QUIT`。然后以下命令在 Ubuntu
开发电脑执行，不是在树莓派上执行。先预览：

```bash
cd '/home/cnn/桌面/dexexo/dexEXO'
ssh pi@192.168.3.76 'mkdir -p /home/pi/cnn/newteleop'
rsync -avn --itemize-changes \
  --exclude='.venv/' \
  --exclude='__pycache__/' \
  --exclude='*.pyc' \
  --exclude='*.log' \
  newteleop/ pi@192.168.3.76:/home/pi/cnn/newteleop/
```

确认列表后去掉 `n` 执行正式同步：

```bash
rsync -av --progress \
  --exclude='.venv/' \
  --exclude='__pycache__/' \
  --exclude='*.pyc' \
  --exclude='*.log' \
  newteleop/ pi@192.168.3.76:/home/pi/cnn/newteleop/
```

此命令不使用 `--delete`，所以不会删除树莓派上额外文件，也不同步
`.venv`。但同名源码仍会被覆盖；树莓派如有未回传的标定或实验记录，
先备份。

### 4.2 只同步外骨骼力控

Ubuntu：

```bash
cd '<dexEXO仓库根目录>'
scp \
  newteleop/exoskeleton/force_control/dual_hand_force_test.py \
  newteleop/exoskeleton/force_control/hand_force_test.py \
  newteleop/exoskeleton/force_control/left_hand_force_test.py \
  newteleop/exoskeleton/force_control/left_index_force_test.py \
  pi@192.168.3.76:/home/pi/cnn/newteleop/exoskeleton/force_control/

scp newteleop/exoskeleton/FSR/exo_pressure_common.py \
  pi@192.168.3.76:/home/pi/cnn/newteleop/exoskeleton/FSR/

scp -r newteleop/exoskeleton/supervisor \
  pi@192.168.3.76:/home/pi/cnn/newteleop/exoskeleton/
```

### 4.3 只同步 FSR 工具

Ubuntu：

```bash
cd '<dexEXO仓库根目录>'
scp \
  newteleop/exoskeleton/FSR/ble_broker.py \
  newteleop/exoskeleton/FSR/exo_pressure_common.py \
  newteleop/exoskeleton/FSR/exo_pressure_monitor.py \
  newteleop/exoskeleton/FSR/fsr_channel_identifier.py \
  pi@192.168.3.76:/home/pi/cnn/newteleop/exoskeleton/FSR/
```

### 4.4 同步后语法检查

树莓派：

```bash
cd ~/cnn/newteleop
./.venv/bin/python -m py_compile \
  exoskeleton/FSR/*.py \
  exoskeleton/force_control/*.py \
  exoskeleton/supervisor/*.py \
  exoskeleton/tools/*.py

./.venv/bin/python -m unittest discover \
  -s exoskeleton/supervisor -p 'test_*.py' -v
```

没有输出表示语法检查通过。

---

## 5. 设备与端口

| 对象 | 右手 | 左手 |
|---|---|---|
| BLE MAC | `F0:FD:45:02:85:B3` | `F0:FD:45:02:67:3B` |
| BLE Broker TCP | `127.0.0.1:9001` | `127.0.0.1:9002` |
| Dynamixel ID | 1～5 | 6～10 |
| G1 INSPIRE触觉 | `192.168.3.78:9201` | `192.168.3.78:9202` |
| G1力控覆盖 | `192.168.3.78:9301` | `192.168.3.78:9302` |

手指顺序统一为：

```text
[拇指, 食指, 中指, 无名指, 小指]
```

---

## 6. 串口和 Dynamixel 检查

### 6.1 检查串口设备

```bash
readlink -f /dev/serial0
ls -l /dev/serial0
```

如果权限不足，检查用户组：

```bash
groups
```

通常 `pi` 需要在 `dialout` 组。如果不在：

```bash
sudo usermod -aG dialout pi
```

执行后退出 SSH 并重新登录，或重启树莓派。

### 6.2 只读检查右手

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/tools/exo_dynamixel_probe.py --hand right
```

应发现 ID 1～5。

### 6.3 只读检查左手

```bash
./.venv/bin/python exoskeleton/tools/exo_dynamixel_probe.py --hand left
```

应发现 ID 6～10。

### 6.4 只读检查十指

```bash
./.venv/bin/python exoskeleton/tools/exo_dynamixel_probe.py --hand all
```

应确认：

- ID 1～10 全部 Ping 成功。
- `hw_error=0x00`。
- 静止时 `current_raw` 接近 0。
- 电压、温度无异常。

运行 probe 时不能同时运行单手或双手力控。

---

## 7. BLE Broker

每块 BLE 板只能被一个 `ble_broker.py` 进程连接。双手需要两个 SSH 终端。

### 终端 1：右手

```bash
ssh pi@192.168.3.76
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/FSR/ble_broker.py --hand right
```

应看到：

```text
手=right BLE=F0:FD:45:02:85:B3 TCP=127.0.0.1:9001
[BLE] 已连接
[TCP] 监听 127.0.0.1:9001
```

### 终端 2：左手

```bash
ssh pi@192.168.3.76
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/FSR/ble_broker.py --hand left
```

应看到：

```text
手=left BLE=F0:FD:45:02:67:3B TCP=127.0.0.1:9002
[BLE] 已连接
[TCP] 监听 127.0.0.1:9002
```

### 检查端口

另开 SSH 终端：

```bash
ss -lnt | grep -E '9001|9002'
```

应显示两个 LISTEN。

---

## 8. 双手 FSR 监视

保持两个 BLE broker 运行，在第三个 SSH 终端执行：

```bash
ssh pi@192.168.3.76
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/FSR/exo_pressure_monitor.py --hand both
```

限定监视 20 秒：

```bash
./.venv/bin/python exoskeleton/FSR/exo_pressure_monitor.py \
  --hand both \
  --seconds 20
```

降低打印频率：

```bash
./.venv/bin/python exoskeleton/FSR/exo_pressure_monitor.py \
  --hand both \
  --print-hz 2
```

输出每轮两行：

```text
[右手] 拇指=... | 食指=... | 中指=... | 无名指=... | 小指=...
[左手] 拇指=... | 食指=... | 中指=... | 无名指=... | 小指=...
```

监视器可以和力控同时连接 9001/9002，因为 broker 支持多个 TCP 客户端。
但输出日志较多，正式力控时可停止监视器以方便观察状态机。

---

## 9. G1 触觉端口连通检查

G1 必须已运行：

```bash
~/zh/start.sh --hand both --hand-only --hand-haptic
```

树莓派先检查 G1 网络：

```bash
ping -c 3 192.168.3.78
```

可用 Bash TCP 只检查端口是否可建立连接：

```bash
timeout 2 bash -c '</dev/tcp/192.168.3.78/9201' && echo '右手9201可连接'
timeout 2 bash -c '</dev/tcp/192.168.3.78/9202' && echo '左手9202可连接'
```

该命令不控制任何硬件，但会短暂建立一次 TCP 连接。

---

## 10. 双手外骨骼只读联调

条件：

- G1 已以 `--hand both --hand-only --hand-haptic` 启动。
- 右/左 BLE broker 已运行。
- 其他 Dynamixel 程序全部退出。

树莓派第三终端：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/force_control/dual_hand_force_test.py \
  --force-host 192.168.3.78
```

程序会打开 `/dev/serial0` 并读取 ID 1～10，但没有 `--enable-write`时不启用外骨骼。

输入：

```text
STATUS
```

应确认十指：

- `state=STOP`。
- FSR 数据年龄正常。
- INSPIRE `valid=True`。
- Inspire 数据年龄小于约 0.5 秒。
- 没有 `FAULT`。

退出：

```text
QUIT
```

---

## 11. 不加 mHandPro：按压 INSPIRE 指尖测试外骨骼

该模式只验证：

```text
INSPIRE指尖触觉 → G1 9201/9202 → 树莓派力控 → 外骨骼
```

不运行 Ubuntu `dual_arm_viz.py`和 `mhandpro_diagnostic`，命令中不加
`--enable-mhandpro`。

运行：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/force_control/dual_hand_force_test.py \
  --enable-write \
  --force-host 192.168.3.78 \
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
  2>&1 | tee force_test.log
```

程序启动后：

命令中各参数的逐项含义见项目主文档 `README.md` 的
“当前双手力控命令与参数”。`force_test.log` 保存在
`/home/pi/cnn/newteleop/force_test.log`。

```text
STATUS
INIT
STATUS
ARM
```

INIT 时：

- 不按压 INSPIRE 指尖。
- 外骨骼保持放松。
- FSR 不受额外力。
- 十根绳不要人为拉紧。

ARM 后按以下顺序测试，每次只按一根 INSPIRE 指尖：

1. 右拇指、食指、中指、无名指、小指。
2. 左拇指、食指、中指、无名指、小指。
3. 最后才测试两根或双手同时按压。

正常状态：

```text
FREE → FORCE_ENTRY → LOCKED → RELEASE → RETURN_SETTLE → FREE
```

本模式不连接 9301/9302，G1 上 9301/9302 保持 LISTEN 是正常的。

---

## 12. 加 mHandPro：双手完整闭环

条件：

- G1 以 `--hand both --hand-only --hand-haptic` 或
  `--hand both --hand-haptic` 启动。
- Ubuntu 已启动 `dual_arm_viz.py --hand --hand-left`。
- Ubuntu 只运行一个 `mhandpro_diagnostic`，并已执行 `teleop both ...`。
- 右/左 BLE broker 已启动。

树莓派：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/force_control/dual_hand_force_test.py \
  --enable-write \
  --enable-mhandpro \
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
  2>&1 | tee force_test.log
```

正常启动信息应包含：

- 右手 FSR 9001、INSPIRE 9201、覆盖 9301。
- 左手 FSR 9002、INSPIRE 9202、覆盖 9302。
- ID 1～10 全部 Ping 成功。

再执行：

```text
STATUS
INIT
STATUS
ARM
```

只有加 `--enable-mhandpro` 时，树莓派才会连接 9301/9302，LOCKED 时才会向 G1
发送 INSPIRE 指位置增量。

---

## 13. SSH 多终端组织

双手测试通常需要三个树莓派 SSH 终端：

| 终端 | 程序 |
|---|---|
| Pi-1 | 右手 `ble_broker.py --hand right` |
| Pi-2 | 左手 `ble_broker.py --hand left` |
| Pi-3 | `dual_hand_force_test.py` 或 FSR monitor |

不要在同一终端用 `&` 后台启动所有程序，否则容易看不到 BLE 断开和力控
FAULT。

### 可选：tmux

需要 SSH 断开后查看日志时可用 `tmux`，但它不能作为无人监控力控的理由。

```bash
sudo apt install tmux
tmux new -s newteleop
```

分窗格：`Ctrl+B` 后按 `%`。脱离：`Ctrl+B` 后按 `D`。恢复：

```bash
tmux attach -t newteleop
```

力控运行时仍必须有人现场并能物理断电。

---

## 14. 停机顺序

### 正常停止

1. 力控终端输入：

   ```text
   STOP
   ```

2. 确认十指不再收绳，再输入：

   ```text
   QUIT
   ```

3. 右手 BLE broker 终端按 `Ctrl+C`。
4. 左手 BLE broker 终端按 `Ctrl+C`。
5. 最后断开外骨骼电源。

### 紧急情况

如果出现持续收绳、方向错误、人手被夹或程序无响应：

1. 立即物理断开外骨骼电源。
2. 然后才在终端按 `Ctrl+C` 或关闭进程。
3. 不要为了保留日志而延迟断电。

---

## 15. 进程与端口排查

### 查看 newteleop 进程

```bash
ps -ef | grep -E '[b]le_broker|[d]ual_hand_force|[h]and_force|[e]xo_pressure'
```

### 查看本地端口

```bash
ss -lntp | grep -E '9001|9002'
```

### 查看与 G1 的连接

```bash
ss -ntp | grep -E '9201|9202|9301|9302'
```

不加 mHandPro 时：

- 9201/9202 应有 `ESTAB`。
- 9301/9302 没有 `ESTAB` 正常。

加 mHandPro 完整闭环时：

- 9201/9202 应有 `ESTAB`。
- 9301/9302 也应有 `ESTAB`。

### 查看谁占用串口

```bash
sudo lsof /dev/serial0
```

如果出现 `Port is in use`，先找出并正常退出占用进程，不要反复 ARM。

---

## 16. 常见错误

| 错误/现象 | 处理 |
|---|---|
| `Permission denied (publickey,password)` | 检查用户是否为`pi`、IP是否正确；使用密码登录后重做`ssh-copy-id` |
| `No route to host` | 开发电脑和树莓派不在同一网络，或IP已变 |
| `ModuleNotFoundError: bleak` | 使用`./.venv/bin/python`，并安装`requirements.txt` |
| `ModuleNotFoundError: dynamixel_sdk` | 在`.venv`中安装`dynamixel-sdk` |
| BLE一直重连 | 检查MAC、供电、距离，并确认没有第二个Bleak进程 |
| `Address already in use` 9001/9002 | 已有broker运行，用`ss -lntp`和`ps`查找 |
| `[TxRxResult] Port is in use!` | 有多个Dynamixel程序同时访问串口 |
| `Inspire valid=False` | 检查G1是否加`--hand-haptic`，以及9201/9202网络 |
| `mHandPro五指覆盖未连接` | 完整闭环时检查G1 9301/9302；独立外骨骼测试不要加`--enable-mhandpro` |
| ARM被拒绝 | 运行STATUS，确保十指FSR/INSPIRE数据新鲜并且全部INIT |
| 按右手驱动左手 | 检查BLE MAC、9001/9002、9201/9202和ID映射 |
| 按某指驱动另一指 | 运行`fsr_channel_identifier.py`重新确认通道 |
| 松开后延迟RELEASE | 默认有0.15s释放消抖；同时检查INSPIRE和FSR是否真正卸载 |

---

## 17. 重启和关机

重启前先 STOP/QUIT 力控并停止 BLE broker：

```bash
sudo reboot
```

关机：

```bash
sudo poweroff
```

等待系统完全关机后再切断树莓派主电源，避免损坏存储卡。

---

## 18. 每次实验建议记录

每次联调建议记录：

- 日期和操作人。
- 树莓派/G1/Ubuntu IP。
- Git commit 或同步时间。
- 左右 BLE MAC 和连接状态。
- ID 1～10 的 Ping、温度、电压和硬件错误。
- `max-goal-current`、`release-goal-current`和电流方向。
- 每指 INIT 位置和 FSR 预载。
- 每指状态转移是否完整。
- 任何 FAULT、串指、延迟释放或物理断电事件。

最重要的原则：**树莓派上同一时刻只能有一个进程控制 `/dev/serial0`；
应该由两个 BLE broker 分别独占两块 BLE 板。**
