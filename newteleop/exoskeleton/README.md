# 外骨骼力反馈测试与集成

完整的力反馈架构、开环/真闭环选择、旧代码复用和分阶段实施顺序见
[`HAPTIC_FEEDBACK_PLAN_CN.md`](HAPTIC_FEEDBACK_PLAN_CN.md)。

本目录专门放置操作者侧外骨骼、Dynamixel XL330 舵机、拉绳和力反馈相关代码。

`mHandpro/` 只保留 mHandPro 手套 SDK、姿态解算、标定及 Inspire 灵巧手映射代码。

## 当前硬件

- Dynamixel XL330-M288-T × 5
- Protocol 2.0，1 Mbps
- 树莓派串口：`/dev/serial0` → `/dev/ttyAMA0`
- 右手舵机 ID：1～5（ID 1 拇指、2 食指、3 中指、4 无名指、5 小指）
- 左手舵机 ID：6～10（ID 6 拇指、7 食指、8 中指、9 无名指、10 小指）
- 当前优先调试左手；工具必须显式选择 `--hand left`。

## 诊断工具

- `tools/exo_dynamixel_probe.py`：只读 Ping 和状态检查，不开启扭矩。
- `tools/exo_single_servo_jog.py`：单舵机小幅点动，低 PWM、低速、自动回原位并关闭扭矩。
- `FSR/exo_pressure_monitor.py`：通过 BLE broker TCP 只读显示五路薄膜压力值和滚动统计。
- `FSR/exo_pressure_calibrate.py`：使用推拉力计交互采集零点和已知力点，保存每片传感器的原始数据与拟合结果。
- `FSR/README.md`：桌面标定、穿戴去皮和旧代码复用边界。

薄膜传感器本体标定时，应将传感器平放在稳定桌面夹具中，用面积稳定的接触头垂直加载；
安装到外骨骼后只需重新采集穿戴零点，并用少量已知力点复核安装影响。两个压力工具都只读取
BLE broker 的 TCP 数据，不控制舵机，也不向 STM32 写数据。

左手 broker 在 `127.0.0.1:9002` 运行后，可执行：

```bash
python exoskeleton/FSR/exo_pressure_monitor.py --port 9002
python exoskeleton/FSR/exo_pressure_calibrate.py --port 9002
```

当旧力控程序已将舵机设为电流模式 0 时，点动必须显式使用
`--temporary-position-mode`。工具只会在扭矩已关闭且硬件无错时临时切换为位置模式 3，
测试结束后自动恢复原模式。

首次使用 `--delta 20 --pwm 50`。若空载下因机械摩擦无法观察，可使用
`--delta 80 --pwm 100`；工具仍会限制在 100 tick 和约 11.4% PWM 以内。

## 与原力控代码的关系

仓库原有五指力控主实现位于 `../Five_finger_force_test/`（即旧 `dexEXO` 仓库），其核心是五指独立 PID 电流闭环。该代码的 BLE 手套输入与现在的 mHandPro 不同，后续仅迁入已验证的执行器、状态机和安全限制。

## 左食指力反馈连接首测

`force_control/left_index_force_test.py` 是最小连接测试：仅接管左食指 ID 7，
保留旧程序的 Inspire 原始值换算、限流、限位和释放思路。FSR 使用 STM32
绝对力输出：`raw <= 4.903N` 视为0，`raw > 4.903N` 保留原值，不减去4.903，
不读取标定 JSON。

在树莓派 `~/cnn/newteleop` 中安装依赖：

```bash
python3 -m venv .venv
./.venv/bin/python -m pip install -r requirements.txt
```

终端 1，启动左手 FSR BLE broker：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/tools/ble_broker.py \
  --devices F0:FD:45:02:67:3B --port 9002
```

终端 2，以只读模式启动 Inspire 桥接。它在 `9202` 发布六路 `FORCE_ACT`
以及五指 `top_touch`摘要；五指顺序为 `[拇指,食指,中指,无名指,小指]`：

```bash
cd ~/cnn/newteleop
./.venv/bin/python mhandpro/standalone_inspire_bridge.py \
  --force-hz 20 --touch-hz 5
```

先确认力数据正在更新：

```bash
timeout 3 nc 127.0.0.1 9202
```

输出JSON中应看到：

```text
top_touch_raw_max     五指96点阵列的各自最大值
top_touch_top5_mean   五指各自最高5点均值
top_touch_force_n     复用旧程序换算后的力
touch_valid           五组阵列本次采集是否有效
```

终端 3，先做只读联通检查：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/force_control/left_index_force_test.py
```

输入 `STATUS`，确认 FSR 和 Inspire 数据年龄均小于 `0.5s`，且
`Inspire index top_touch max`空载约352、右手压食指指尖时明显增加，然后 `QUIT`。

外骨骼未穿戴、
可随时物理断电时运行：

```bash
./.venv/bin/python exoskeleton/force_control/left_index_force_test.py --enable-write
```

输入 `ARM`。ID 7 先保持关扭矩；右手逐渐按压 Inspire 食指指尖，其
`fingerfour_top_touch` 换算力达到 `4.95N` 后才使能，
最大行程 `40 tick`、Goal Current `15`、目标力上限 `6N`。Goal Current 每个20Hz
周期最多增加1，防止FSR从0跳到约4.9N时造成机械冲击。Inspire 撤力后自动回 neutral
并关扭矩。任何异常输入
`STOP`或物理断电。

### 不带数据手套的分阶段测试

1. **Inspire阵列只读**：先单独运行
   `mhandpro/tools/inspire_index_top_touch_monitor.py --matrix --seconds 20`，确认空载、按压、
   释放可分。
2. **全链只读**：broker、Inspire桥接、不带 `--enable-write` 的力控程序同时运行，
   用 `STATUS` 检查两路数据，此时ID 7不动。
3. **未穿戴空载动作**：加 `--enable-write`并 `ARM`。Inspire目标力超过4.95N后，
   确认ID 7收绳方向、电流斜坡、40 tick限位，松开Inspire后回neutral并关扭矩。
4. **未穿戴软垫闭环**：让收绳使外骨骼食指FSR压到软海绵。FSR必须严格按
   `raw <= 4.903N -> 0`、`raw > 4.903N -> raw` 参与闭环，不减去4.903。
5. **穿戴左食指**：只有前四步全部通过后进行；一人穿戴，另一人逐渐按压
   Inspire指尖并准备物理断电，首次只测4.95～6N。

当Inspire指尖换算力低于4.9N时，本轮不启动外骨骼闭环，因为STM32在
`0～4.9N` 范围只输出 `4.903N`，无法提供与目标力可比较的反馈。不应通过
降低树莓派阈值绕过这个物理限制。

## neutral、机械预紧与重新采集

### neutral的含义

`neutral` 是某根手指在“自然伸直、绳索刚好消除明显松垮、对手指/FSR没有主动拉力”
时的 Dynamixel 位置基准。它不是力传感器标定，也不是舵机的出厂零点。

左食指ID 7使用neutral完成：

- 启动时判断机械位置是否安全；
- `ARM`前防止在已经拉紧的位置接管；
- 计算收绳软限位 `neutral + reel_in_sign * max_travel`；
- Inspire撤力后回到neutral，再关闭扭矩。

Dynamixel位置每4096 tick存在等效回绕。例如 `2691` 和 `6787=2691+4096`
可以是同一机械位置，程序会自动映射到最接近neutral的等效圈。但如果
`position_raw=5496`、`neutral=2997`，归一化后为 `1400`，仍相1597 tick，
这不是单纯回绕。

### 什么时候必须重新确定neutral

出现以下任一情况时需要重新确定：

- 拆下并重装舵盘；
- 改变舵盘花键角度；
- 改变绳长、打结位置或卷线轮绕法；
- 手动转动过舵机轴，且新位置被定义为机械起点；
- 自然伸直时的当前位置与旧neutral在做4096回绕后仍相差超过40 tick。

如果没有主动调整却突然偏离数百/数千tick，先检查舵盘螺丝、花键打滑、绳索卡死、
舵机轴被外力转动和供电重启；不能直接把异常位置保存为新neutral。

### 只测ID 7时的临时neutral

确认ID 7当前处于自然伸直、轻微预紧位置后，可以先用命令行覆盖，不改写五指JSON：

```bash
./.venv/bin/python exoskeleton/force_control/left_index_force_test.py \
  --neutral <ID7当前稳定位置>
```

只读启动应显示 `position_norm` 与neutral相差不超过40 tick。不应为了跳过安全检查
随意填写当前位置；该位置必须先通过机械状态确认。

### 最终五指neutral采集

确保左手5个舵机全部在线、扭矩关闭、无硬件错误，且五指都处于各自自然起点：

```bash
./.venv/bin/python exoskeleton/tools/exo_dynamixel_probe.py --hand left
./.venv/bin/python exoskeleton/tools/exo_capture_neutral.py --hand left
```

输入 `NEUTRAL` 后，程序只读采样2秒，位置波动不超过8 tick才保存到
`exoskeleton/config/left_neutral.json`。该操作会更新全部五指，因此任何一指未就位或掉线时都不应采集。

## 从机械安装到穿戴的完整流程

### 0. 固定测试范围

首版仅测左食指：Dynamixel ID 7、FSR食指索引1、Inspire
`fingerfour_top_touch`。其他指保持关扭矩。准备物理断电，首次不穿戴。

### 1. 机械安装与轻微预紧

1. 输入 `STOP`/`QUIT`，关闭舵机电源。
2. 将外骨骼食指置于自然伸直。
3. 重装舵盘或调整绳长，只消除明显松垮，不应主动拉弯关节。
4. 拧紧舵盘螺丝，检查绳索在导向槽内，不磨擦FSR引线。
5. 上电后连续运行3次probe，必须每次发现5/5个舵机。

### 2. 安装独立STM32 FSR

FSR由STM32独立采集并通过BLE发送，但必须被安装在机械受力路径：

```text
外骨骼硬质指尖 -> FSR -> 薄软垫 -> 海绵模拟指/操作者指腹
```

固定FSR边缘，不在感知圆片中央堆胶；引线沿手指侧面固定。空载时应约4.9N，
按压后超过4.903N，松开后回到约4.9N。

### 3. 只读检查三条链路

1. `exo_dynamixel_probe.py --hand left`：ID 6～10全部在线，无错误、零电流。
2. `exo_pressure_monitor.py --port 9002`：食指FSR空载/按压/释放正常。
3. `inspire_index_top_touch_monitor.py --matrix`：Inspire食指指尖阵列空载/按压/释放正常。

### 4. 启动三个长期进程

终端1启动FSR broker，终端2以只读模式启动Inspire桥接，终端3启动左食指力控。
命令参见上文“左食指力反馈连接首测”。首先不带 `--enable-write`，用 `STATUS`
确认FSR、Inspire数据年龄都小于0.5秒。

### 5. 未穿戴、海绵模拟闭环

使用已确认的neutral，先保持低限制：

```bash
./.venv/bin/python exoskeleton/force_control/left_index_force_test.py \
  --enable-write \
  --neutral <已确认的ID7 neutral> \
  --max-travel 40 \
  --max-goal-current 10 \
  --current-slew 1
```

输入 `STATUS`、`ARM`，再逐渐按压Inspire食指指尖。验收：

- Inspire目标力达到4.95N才从FREE进入HOLD；
- ID 7位置向收绳方向平缓变化，不超过40 tick软限位；
- 海绵被压后FSR从4.9跳到超过4.903N，且保留绝对力原值；
- FSR趋近目标力时Goal Current不再增加；
- 松开Inspire后RELEASE回neutral、关扭矩、回FREE；
- `STOP`、FSR断线、Inspire断线都会关扭矩。

释放默认使用Goal Current 15。如果3秒内未回到 `neutral±8 tick`，程序进入
`FAULT`并关闭扭矩，不允许无限期停留在RELEASE。发生超时后必须检查预紧、摩擦和
neutral，不能反复ARM绕过故障。

如果位置不动，先检查是否预紧过度；再将 `max-goal-current` 从10增到15。
不应同时大幅增加电流和行程。如果FSR始终为4.9，说明FSR未进入受力路径，
增大电流不能解决该问题。

### 6. 穿戴前验收

未穿戴闭环连续成功3～5次，且无位置越界、无通信掉线、无快速冲击、无明显发热，
才能进入穿戴测试。

### 7. 首次穿戴

1. 舵机断电穿戴，确认能快速抽出手指。
2. 检查绳索、FSR和软垫位置，空载FSR约4.9N。
3. 一人穿戴，另一人按压Inspire并准备物理断电。
4. 只测左食指，保持4.95～6N目标上限、短时接触，随时 `STOP`。
5. 释放、断线和人工STOP都验证后，再考虑接入mHandPro和扩展其他手指。

## 安全顺序

1. 只读检查 5 个舵机。
2. 外骨骼不穿戴，分别确认每个舵机的收绳/放绳方向。
3. 确认位置软限位、电流限制和断线扭矩关闭。
4. 穿戴后从单指、小力度开始闭环。
5. 再启用五指与 Inspire 触觉反馈联动。
