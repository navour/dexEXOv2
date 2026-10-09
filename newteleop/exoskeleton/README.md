# 外骨骼力反馈测试与集成

> 运行位置：树莓派，Dynamixel总线 `/dev/serial0`  
> 当前状态：左手单指/五指联调；右手先做BLE/FSR只读识别  
> 更新：2026-07-30  
> 安全原则：任何写入测试先未穿戴、单指、低电流，并准备物理断电

---

## 目录

1. [当前硬件](#当前硬件)
2. [诊断工具](#诊断工具)
3. [右手BLE与FSR首测](#右手ble与fsr首测)
4. [左食指力反馈连接首测](#左食指力反馈连接首测)
5. [mHandPro与五指联合测试](#mhandpro抓取--locked联合测试)
6. [机械安装到穿戴流程](#从机械安装到穿戴的完整流程)
7. [安全顺序](#安全顺序)

---

完整的力反馈架构、开环/真闭环选择、旧代码复用和分阶段实施顺序见
[`HAPTIC_FEEDBACK_PLAN_CN.md`](HAPTIC_FEEDBACK_PLAN_CN.md)。
双手 FSR、BLE 数据链和 ID 1～10 遥测上位机见
[`UPPER_COMPUTER_PLAN_CN.md`](UPPER_COMPUTER_PLAN_CN.md)。当前正式实现是与双手力控
同进程的树莓派 PyQt5 本地桌面版，集成 BLE、自动 G1 链路客户端以及
STATUS/INIT/ARM/STOP/QUIT，见 [`LOCAL_SUPERVISOR_CN.md`](LOCAL_SUPERVISOR_CN.md)。

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

## 右手BLE与FSR首测

右手板卡已通过断电消失、上电出现确认：名称 `RFstar_85B3`，MAC
`F0:FD:45:02:85:B3`，本机TCP端口 `9001`。

终端1独占右手BLE，终端2只读监视并逐指识别：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/FSR/ble_broker.py --hand right

# 另一个终端
./.venv/bin/python exoskeleton/FSR/exo_pressure_monitor.py --hand right
./.venv/bin/python exoskeleton/FSR/fsr_channel_identifier.py --hand right
```

识别工具先采集五片卸载基线，再提示每次只压一片，按相对增量判定物理手指对应的末五路索引。结果必须是五个唯一通道，并在板卡断电重启后重复一致，才能写入右手力控。BLE MAC只能确认板卡身份，不能说明ADC插座连接了哪根手指；不能按线色或左手顺序猜测。

## 与原力控代码的关系

当前单指测试的舵机状态机直接对齐 `ftp/Force_handcontrol.py`：
FORCE_ENTRY在模式0中用Inspire触觉力产生收绳电流，位置稳定后进入
LOCKED并切模式5锁定。传入 `--enable-mhandpro` 后，FORCE_ENTRY仍允许
Inspire整手跟随mHandPro；进入LOCKED后才通过9302保持整手抓取姿态，
并对食指执行原程序的力差PID；INSPIRE触觉释放或FSR新增力卸载持续0.30秒后，模式0
反向电流回到init_pos。

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
./.venv/bin/python exoskeleton/FSR/ble_broker.py --hand left
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
./.venv/bin/python exoskeleton/force_control/left_index_force_test.py \
  --enable-write \
  --max-goal-current 30 \
  --release-goal-current 30
```

默认不盲目把任意上电姿态当作init_pos。将食指和绳索摆到本轮希望的返回
起点，绑好FSR且保持本次绑带预载稳定，输入 `INIT`；程序只读采样0.5秒，
同时记录位置和FSR预载，位置波动
不超过8 tick才接受。然后输入 `ARM`。ID 7在init_pos待机；Inspire
`fingerfour_top_touch`换算力达到 `4.95N` 后进入FORCE_ENTRY，模式0正电流
收绳；位置稳定后切模式5锁定。与原程序一样默认无位置限位，但电流首试
仅限30mA；Inspire
撤力后模式0反向电流回init_pos，再切模式5保持。任何异常输入
`STOP`或物理断电。

### 不带数据手套的分阶段测试

1. **Inspire阵列只读**：先单独运行
   `mhandpro/tools/inspire_index_top_touch_monitor.py --matrix --seconds 20`，确认空载、按压、
   释放可分。
2. **全链只读**：broker、Inspire桥接、不带 `--enable-write` 的力控程序同时运行，
   用 `STATUS` 检查两路数据，此时ID 7不动。
3. **未穿戴空载动作**：加 `--enable-write`，先 `INIT`再 `ARM`。Inspire目标力
   超过4.95N后，确认ID 7在FORCE_ENTRY中正电流使position增大、
   稳定后LOCKED；INSPIRE触觉释放或FSR卸载持续0.30秒后反向电流回init_pos并切模式5保持。
4. **未穿戴软垫受力验证**：让收绳使外骨骼食指FSR压到软海绵。FSR必须严格按
   `raw <= 4.903N -> 0`、`raw > 4.903N -> raw` 显示绝对力，不减去4.903。
   不传 `--enable-mhandpro` 时LOCKED不写Inspire，可保留这一单独硬件验证模式。
5. **穿戴左食指**：只有前四步全部通过后进行；一人穿戴，另一人逐渐按压
   Inspire指尖并准备物理断电，首次只测4.95～6N。

当Inspire指尖换算力低于4.9N时，本轮不启动外骨骼闭环，因为STM32在
`0～4.9N` 范围只输出 `4.903N`，无法提供与目标力可比较的反馈。不应通过
降低树莓派阈值绕过这个物理限制。

## mHandPro抓取 + LOCKED联合测试

### 五指力反馈程序

单食指流程验证通过后，使用
`force_control/left_hand_force_test.py` 扩展到左手ID 6～10。五指的
FSR、Inspire top_touch和舵机均按 `[拇,食,中,无,小]` 排列，
每指独立运行状态机；未LOCKED手指仍跟随mHandPro。详细参数和
逐指验收绳方向步骤见 `force_control/README.md`。五指写入模式必须
显式传入 `--drive-current-signs`，防止把食指已验证的方向盲目用于
其他四指。

首轮仍不穿戴外骨骼，用海绵或可快速抽出的模拟指压FSR，并准备舵机
物理断电。四个终端按下列顺序启动。

终端1，FSR：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/FSR/ble_broker.py --hand left
```

终端2，Inspire桥接。9102接收mHandPro角度，9202发布触觉，9302接收
力控状态：

```bash
cd ~/cnn/newteleop
./.venv/bin/python mhandpro/standalone_inspire_bridge.py \
  --enable-write \
  --safe-open 980,965,957,946,949,922 \
  --force-hz 20 --touch-hz 5
```

终端3，mHandPro：

```bash
cd ~/cnn/newteleop/mhandpro
./bin/mhandpro_diagnostic
```

依次输入：

```text
status
load
zero
show
teleop config/inspire_left_pi_smoke_10.cfg
ARM
```

先缓慢弯伸六维，确认Inspire方向与限幅正确。本轮确认低行程效果不明显后，
先退出当前teleop，再使用力反馈专用100%行程配置：

```text
teleop config/inspire_left_haptic_100.cfg
ARM
```

终端4，外骨骼力控：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/force_control/left_index_force_test.py \
  --enable-write --enable-mhandpro \
  --max-goal-current 30 \
  --release-goal-current 30 \
  --release-timeout 12 \
  --actual-current-limit 45
```

输入 `STATUS`，除FSR与Inspire数据新鲜外，必须看到mHandPro覆盖9302已连接。
将外骨骼绳索置于本轮释放返回点，绑好FSR并保持绑带预载稳定，然后输入：

```text
INIT
ARM
```

用mHandPro使Inspire缓慢抓住软物体。正常日志应依次出现：

```text
FREE -> FORCE_ENTRY -> LOCKED -> RELEASE -> RETURN_SETTLE -> FREE
```

- `FREE`：六通道都跟随mHandPro。
- `FORCE_ENTRY`：Inspire食指触觉超过4.95N；六通道继续跟随mHandPro，ID7正电流收绳。
- `LOCKED`：ID7在0.4秒窗口内位置变化不超过8 tick；它切模式5锁位置，
  Inspire食指先朝张开方向退让80 tick，之后根据 `FSR - Inspire`力差每周期
  最多调整3 tick。其他五个通道继续保持抓取姿态。
- `RELEASE`：INSPIRE触觉释放或FSR相对INIT预载的新增力≤0.1N，持续0.30秒；
  ID7反向电流回init_pos，归位期间忽略Inspire再次接触，
  Inspire六通道平滑恢复跟随mHandPro。
- `RETURN_SETTLE`：ID7以模式5保持init_pos；位置在±15 tick内且
  Inspire目标力低于释放阈值连续0.5秒后才进FREE。这会吸收归位
  惯性与绳索弹性造成的过冲，并阻止滤波残留力立即触发下一轮。
  `--return-settle-time` 可在0.2～2.0秒调整，默认0.5秒。
- 任何时候输入 `STOP`，都会关闭ID7扭矩并取消整手姿态覆盖。9302断开或超时
  0.5秒也会让Inspire食指自动恢复跟随mHandPro。

如果始终进不去LOCKED，不要先增大电流：先看FORCE_ENTRY中ID7是否持续移动。
程序默认参考源代码，不用固定行程或时间判定寻触结束，只有位置稳定才进入
LOCKED。`--seek-max-travel 0 --seek-timeout 0` 表示源程序模式；完成正常行程
实测后，可传入非零值作为额外异常保护，但它不参与LOCKED判定。
只有绳索与外骨骼形成反力、位置稳定后才会LOCKED。如果LOCKED后FSR仍恒为
4.900N，说明FSR没有进入受力路径，此时闭环无法进行。

## init_pos与释放返回点

`init_pos` 直接复用 `Force_handcontrol.py` 的名称和含义：它是本轮启动位置和
RELEASE返回点，可以是放松绳位置。FORCE_ENTRY负责从该位置用正电流
收绳并寻找接触，位置稳定后LOCKED。

启动后依次输入：

```text
STATUS
INIT
ARM
```

`INIT`要求ID 7关扭矩、FSR空载且位置保持稳定；程序只读采样0.5秒，
波动不超过8 tick才记录本次 `init_pos`。它不写文件。也可以显式传入：

```bash
./.venv/bin/python exoskeleton/force_control/left_index_force_test.py \
  --init-pos <ID7已确认的释放返回点>
```

拆装舵盘、改变绳长、改变卷线方式或手动转动舵机轴后，都应重新INIT。
不能在手指意外弯曲、绳索已过紧或FSR已受力时记录，因为RELEASE会主动
回到该位置。

位置读数由相邻采样连续跟踪；即使收绳超过4096 tick或跨过0，也不会强制
映射回init_pos附近。

## 从机械安装到穿戴的完整流程

### 0. 固定测试范围

首版仅测左食指：Dynamixel ID 7、FSR食指索引1、Inspire
`fingerfour_top_touch`。其他指保持关扭矩。准备物理断电，首次不穿戴。

### 1. 机械安装与释放返回点

1. 输入 `STOP`/`QUIT`，关闭舵机电源。
2. 将外骨骼食指置于自然伸直。
3. 重装舵盘或调整绳长。原程序的FORCE_ENTRY允许从放松绳位置开始寻触，
   但起点不应主动拉弯关节，且必须确认为本轮RELEASE希望返回的位置。
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

使用已确认的本轮释放返回点，先保持低电流限制：

```bash
./.venv/bin/python exoskeleton/force_control/left_index_force_test.py \
  --enable-write \
  --max-goal-current 30 \
  --release-goal-current 30
```

如果未显式传 `--init-pos`，先输入 `STATUS`、`INIT`，
再输入 `ARM`和逐渐按压Inspire食指指尖。验收：

- Inspire目标力达到4.95N才从FREE进入FORCE_ENTRY；
- ID 7按原 `Force_handcontrol.py` 切到模式0，用正电流收绳，当前机构上
  position应增大；默认 `max_travel=0`，与原程序一样不设位置限位；
- 0.4秒窗口内位置变化不超过8 tick后进入LOCKED，切模式5锁定当前位置；
- 海绵或指腹被压后FSR从4.9跳到超过4.903N，且保留绝对力原值；
- INSPIRE触觉释放或FSR卸载持续0.30秒后RELEASE回init_pos、切模式5保持；
- RETURN_SETTLE确认位置和Inspire释放连续稳定0.5秒后回FREE；
- `STOP`、FSR断线、Inspire断线都会关扭矩。

释放默认使用Goal Current 30。由于低于原程序的150mA，默认释放超时从
原程序的4秒放宽到12秒。如12秒内未回到 `init_pos±15 tick`，程序进入
`FAULT`并关闭扭矩。发生超时后必须检查绳路、摩擦和init_pos。

如果FORCE_ENTRY中位置不动且过早LOCKED，先检查机械卡滞；再将
`max-goal-current` 从30逐级增加，不能直接跳到原程序的300。
不应同时大幅增加电流和行程。如果FSR始终为4.9，说明FSR未进入受力路径，
增大电流不能解决该问题。

### 5.1 40 mm舵盘：手动测量实际所需收绳位移

已知舵盘直径为40 mm，且Dynamixel每圈4096 tick。在绳索不打滑、不叠层的
近似下：

```text
舵盘周长 = pi * 40 = 125.664 mm
1 tick收绳量 = 125.664 / 4096 = 0.03068 mm
收绳位移(mm) = |position - init_pos| * 0.03068
```

因此，40 tick约1.23 mm，42 tick约1.29 mm，60 tick约1.84 mm，
80 tick约2.45 mm，100 tick约3.07 mm。之前从约5891到5933的42 tick只相当于
约1.29 mm收绳，这段位移可能先被绳索松弛、弹性和外骨骼间隙吸收。

下面的联合测量程序只读舵机位置和FSR，不写寄存器。它会自动锁定P0，
用户只需依次输入 `P1`、`P2`、`P3`、`P4`。先退出力控程序，确认ID 7
已关扭矩；保留舵机供电和串口连接，使位置编码器仍可读。不要在力控程序
运行时手动拉绳。

1. 确保BLE broker已在9002运行，然后启动联合测量：

   ```bash
   cd ~/cnn/newteleop
   ./.venv/bin/python exoskeleton/tools/left_index_manual_travel_measure.py
   ```

2. 启动后前2秒保持食指自然伸直和绳索起点，程序会自动取位置中位数作为
   `P0`。如果这段时间位置波动超过8 tick，程序拒绝继续。
3. 一人缓慢沿实际收绳方向拉绳，另一人看实时位置/FSR和随时释放。到达
   `P1`（绳索刚绷紧/刚开始带动机构）、`P2`（食指刚能感到拉力）、
   `P3`（拉力清晰但仍舒适）和 `P4`（FSR首次稳定大于4.903 N）时，
   分别输入对应标签并回车。程序会自动记录最近0.3秒的位置和FSR中位数。
   必须慢拉，不准绕舵盘整圈；出现疼痛、卡滞或FSR快速上升立即放松。
4. 输入P4后，程序自动输出P0～P4的position、相对P0的tick差、40 mm舵盘
   的理论收绳毫米数和食指FSR。如果收绳方向使position变小，脚本仍会正确计算
   位移绝对值。但用户已确认本机构position减小会增加放出的绳长，因此
   力控复用原程序的正电流收绳（`--drive-current-sign 1`）。脚本按相邻采样
   连续跟踪4096 tick回绕，因此总行程超过半圈时也不会翻转正负号。

注意：FSR的5 mm感知区必须被正向、居中压缩。直接将薄片贴在曲面外骨骼
上，DIP指腹的软组织可能将力分散到感知区外，并让传感器发生弯曲而不是
面内压缩。安装时应依次为：平整刚性背板、FSR、对准感知区的3～4 mm小压头、
薄软防滑层。只固定FSR外缘和引线，不在感知区上涂厚胶；小压头不应有尖角。

判定方法：

- `P2-P0` 是“人刚感到力”所需的最小机械行程；
- `P3-P0` 可作为后续渐进行程上限的参考，不应直接一次写入程序；
- `P4`存在且FSR能随放松回到4.9，才说明FSR已进入闭环受力路径；
- 如果已有明显拉力但FSR仍恒为4.9，先修正FSR位置/压力传递结构，
  不应继续增大舵机电流或自动行程。
- 当前程序默认按原程序不设位置限位；如果需要临时额外保护，可显式传
  `--max-travel <tick>`。

### 6. 穿戴前验收

未穿戴闭环连续成功3～5次，且无位置越界、无通信掉线、无快速冲击、无明显发热，
才能进入穿戴测试。

### 7. 首次穿戴

1. 舵机断电穿戴，确认能快速抽出手指。
2. 检查绳索、FSR和软垫位置，空载FSR约4.9N。
3. 一人穿戴，另一人按压Inspire并准备物理断电。
4. 只测左食指，保持4.95～6N目标上限、短时接触，随时 `STOP`。
5. 释放、9302断线恢复手套控制和人工STOP都验证后，再扩展其他手指。

## 安全顺序

1. 只读检查 5 个舵机。
2. 外骨骼不穿戴，分别确认每个舵机的收绳/放绳方向。
3. 确认电流限制、位置稳定锁定和断线扭矩关闭；如显式启用额外位置
   限位，同时验证该限位。
4. 穿戴后从单指、小力度开始闭环。
5. 再启用五指与 Inspire 触觉反馈联动。
