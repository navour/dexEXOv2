# 外骨骼力控 — 左食指与左手五指首测

## G1双手力反馈（Ubuntu + G1板载机 + 树莓派）

双手不能分别启动两个 `hand_force_test.py`：ID 1～10在同一条
`/dev/serial0` 总线上，两个进程会抢占串口并报 `Port is in use`。
双手必须使用 `dual_hand_force_test.py`，由一个进程共享
`PortHandler` 和 `RLock`。

| 手 | FSR（树莓派本机） | 外骨骼ID | G1 INSPIRE触觉 | G1力控覆盖 |
|---|---:|---:|---:|---:|
| 右 | 9001 | 1～5 | 9201 | 9301 |
| 左 | 9002 | 6～10 | 9202 | 9302 |

G1 端先启动（仅测手时保留 `--hand-only`）：

```bash
~/zh/start.sh --hand both --hand-only --hand-haptic
```

树莓派先确认两个 BLE broker 在 9001/9002 都有数据，再启动：

```bash
cd ~/cnn/newteleop
mkdir -p logs
./.venv/bin/python exoskeleton/force_control/dual_hand_force_test.py \
  --enable-write --enable-mhandpro \
  --force-host 192.168.3.78 --haptic-host 192.168.3.78 \
  --right-drive-current-signs 1,1,1,1,1 \
  --left-drive-current-signs 1,1,1,1,1 \
  --force-to-current-gain 60 \
  --max-goal-current 300 --release-goal-current 100 \
  --actual-current-limit 320 --current-slew 20 \
  --hand-total-current-limit 800 \
  --force-step-max 4 --fsr-rest-max 55 --control-hz 10 \
  2>&1 | tee "logs/force_test_$(date +%Y%m%d_%H%M%S).log"
```

### 为什么命令看起来有很多参数

`dual_hand_force_test.py` 是硬件联调入口，而不是参数全部写死的成品服务。
电流方向、舵机电流、FSR预载、G1 IP和控制频率都暴露为命令行参数，
目的是让每次实物实验可复现，并防止换绳路或换舵机后沿用错误方向。

但是，命令中并不是每一项都必须写。当前程序的分类是：

| 类别 | 参数 | 原因 |
|---|---|---|
| 写模式必须显式写 | `--enable-write` | 不写就是只读，不允许ARM |
| 写模式必须显式写 | `--right-drive-current-signs`、`--left-drive-current-signs` | 防止换机构后电流方向错误导致收绳变放绳 |
| 当前硬件需要覆盖 | `--fsr-rest-max 55` | 已知焊接异常通道的静息值约49.03N，默认8N会拒绝INIT |
| 当前建议覆盖 | `--control-hz 10` | 十舵机共享TTL总线，10Hz比默认20Hz更容易稳定 |
| 只有全闭环才需要 | `--enable-mhandpro`、`--force-step-max` | 只在树莓派通过9301/9302修正INSPIRE位置时生效 |
| 可以省略 | 命令中其余与默认值相同的项 | 程序会自动使用默认值 |

因此，当前“不启用mHandPro力控覆盖，只按INSPIRE指尖测外骨骼”时，
可以简化为：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/force_control/dual_hand_force_test.py \
  --enable-write \
  --right-drive-current-signs 1,1,1,1,1 \
  --left-drive-current-signs 1,1,1,1,1 \
  --fsr-rest-max 55 \
  --control-hz 10
```

这条简化命令与原命令在以下项上相同：

```text
force-host=192.168.3.78
force-to-current-gain=60
max-goal-current=300
hand-total-current-limit=800
release-goal-current=100
actual-current-limit=320
current-slew=20
```

原命令中的`--force-step-max 4`与默认3不同，但它调整的是INSPIRE位置修正，
不是外骨骼舵机电流。未传`--enable-mhandpro`时，9301/9302不连接，
这个参数不会对G1上的INSPIRE产生实际位置修正。

### 用户当前命令中每个参数的含义

| 命令项 | 当前值 | 默认值 | 能否省略 | 含义 |
|---|---:|---:|---|---|
| `./.venv/bin/python` | — | — | 不能 | 使用`newteleop/.venv`内已安装`dynamixel-sdk`等依赖的Python |
| `dual_hand_force_test.py` | — | — | 不能 | 双手唯一力控入口，单进程独占`/dev/serial0`并管理ID1～10 |
| `--enable-write` | 开启 | 关闭 | 不能 | 允许写Dynamixel寄存器、INIT后ARM；不传时只读 |
| `--force-host` | `192.168.3.78` | `192.168.3.78` | 可省 | G1板载机IP，右/左INSPIRE触觉数据从9201/9202读入 |
| `--right-drive-current-signs` | `1,1,1,1,1` | 只读时内部为全+1 | 写模式不能 | 右手`[拇,食,中,无,小]`收绳电流符号；每项只能+1或-1 |
| `--left-drive-current-signs` | `1,1,1,1,1` | 只读时内部为全+1 | 写模式不能 | 左手`[拇,食,中,无,小]`收绳电流符号 |
| `--force-to-current-gain` | 60 raw/N | 60 raw/N | 可省 | INSPIRE目标力每增加1N请求60 raw收绳电流 |
| `--max-goal-current` | 300 raw | 300 raw | 可省 | 单指FORCE_ENTRY目标电流上限；XL330-M288在本项目中约1 raw≈1mA |
| `--hand-total-current-limit` | 800 raw | 800 raw | 可省 | 同一只手五指收绳请求之和上限，超过时五指按相同比例缩小 |
| `--release-goal-current` | 100 raw | 100 raw | 可省 | RELEASE放绳并返回INIT位置时的电流上限，接近INIT时还会自动减流 |
| `--actual-current-limit` | 320 raw | 320 raw | 可省 | 回读Present Current绝对值超过此值立即FAULT，必须不小于`max-goal-current` |
| `--current-slew` | 20 raw/周期 | 20 raw/周期 | 可省 | FORCE_ENTRY中每个控制周期最多增加的电流，用于避免电流突跳 |
| `--force-step-max` | 4 tick/周期 | 3 tick/周期 | 未启用mHandPro时可省 | LOCKED中力误差PID每周期对INSPIRE指令的最大位置修正，不是舵机位置或电流 |
| `--fsr-rest-max` | 55N | 8N | 当前不能省 | INIT时0.5秒FSR静息中位数的允许上限；当前为放行已知的49.03N焊接异常通道 |

电流请求的实际公式为：

```text
单指请求电流 = min(INSPIRE目标力 × force_to_current_gain,
                       max_goal_current)
本周期电流 = min(单指请求电流,
                   上周期电流 + current_slew)
```

例如INSPIRE某指为3N：`3×60=180 raw`，未碰到300 raw单指上限；
由于`current-slew=20`，电流会按约0、20、40……180 raw逐步上升，
而不是第一个周期立即到180 raw。

### `dual_hand_force_test.py` 全部参数参考

命令行显示默认值：

```bash
./.venv/bin/python exoskeleton/force_control/dual_hand_force_test.py --help
```

#### 写入、串口与网络

| 参数 | 默认值 | 含义/取值 |
|---|---:|---|
| `--enable-write` | 关 | 无数值开关；开启Dynamixel写入和ARM |
| `--enable-mhandpro` | 关 | 无数值开关；连接9301/9302，向G1发送LOCKED状态和INSPIRE位置修正 |
| `--device` | `/dev/serial0` | 右左十个Dynamixel共用的TTL串口 |
| `--baudrate` | 1000000 | Dynamixel Protocol 2.0总线波特率，必须与ID1～10一致 |
| `--fsr-host` | `127.0.0.1` | BLE Broker所在主机；当前与力控同在树莓派 |
| `--right-fsr-port` | 9001 | 右手FSR Broker TCP端口 |
| `--left-fsr-port` | 9002 | 左手FSR Broker TCP端口 |
| `--force-host` | `192.168.3.78` | G1板载机IP，用于接收INSPIRE指尖触觉 |
| `--right-force-port` | 9201 | G1右INSPIRE触觉端口 |
| `--left-force-port` | 9202 | G1左INSPIRE触觉端口 |
| `--haptic-host` | `192.168.3.78` | G1板载机力控覆盖IP；仅`--enable-mhandpro`时使用 |
| `--right-haptic-port` | 9301 | 右INSPIRE力控覆盖端口；仅`--enable-mhandpro`时使用 |
| `--left-haptic-port` | 9302 | 左INSPIRE力控覆盖端口；仅`--enable-mhandpro`时使用 |

#### 机械方向和电流

| 参数 | 默认值 | 程序校验 | 含义/取值 |
|---|---:|---|---|
| `--right-drive-current-signs` | 无 | 必须是5个`+1/-1` | 右手`[拇,食,中,无,小]`收绳方向；写模式必填 |
| `--left-drive-current-signs` | 无 | 必须是5个`+1/-1` | 左手`[拇,食,中,无,小]`收绳方向；写模式必填 |
| `--max-goal-current` | 300 raw | 1～300 | FORCE_ENTRY单指Goal Current上限 |
| `--release-goal-current` | 100 raw | 1～`max-goal-current` | RELEASE返回INIT时的电流上限 |
| `--actual-current-limit` | 320 raw | ≥`max-goal-current` | Present Current的绝对值故障阈值，不是写入目标 |
| `--current-slew` | 20 raw/周期 | 1～60 | FORCE_ENTRY每周期最大电流增量 |
| `--force-to-current-gain` | 60 raw/N | >0 | INSPIRE目标力到Dynamixel请求电流的比例 |
| `--hand-total-current-limit` | 800 raw | `max-goal-current`～1500 | 每只手五指FORCE_ENTRY请求总和上限 |

#### 状态机与时间

| 参数 | 默认值 | 程序校验 | 含义/取值 |
|---|---:|---|---|
| `--direction-fault-threshold` | 100 tick | 40～500 | FORCE_ENTRY最短驱动0.2秒后，累计反向运动超过此值则FAULT |
| `--release-timeout` | 12s | 双手入口未额外限定 | RELEASE在此时间内未回到INIT则FAULT；建议保持默认 |
| `--return-settle-time` | 0.5s | 双手入口未额外限定 | 已回INIT后，位置和INSPIRE释放连续稳定多久才回FREE |
| `--release-hold-seconds` | 0.15s | 0.1～5.0 | LOCKED中INSPIRE触觉释放需要连续成立的消抖时间 |
| `--fsr-release-hold-seconds` | 0.30s | 0.1～5.0 | FSR确认加载后，卸载条件需要连续成立的时间 |

双手/五指入口当前内部使用0.25秒位置稳定窗口，并要求FORCE_ENTRY至少
驱动0.2秒后才允许进入LOCKED；单指入口对应参数为
`--lock-detect-window 0.25`和`--lock-min-drive-time 0.2`。

#### FSR、接触与LOCKED力差调节

| 参数 | 默认值 | 程序校验 | 含义/取值 |
|---|---:|---|---|
| `--fsr-rest-max` | 8N | 双手入口未额外限定 | INIT时FSR静息中位数上限；当前焊接异常通道使用55N。FSR原始值仍必须在0～60N |
| `--fsr-load-threshold` | 0.30N | 必须大于卸载阈值，最大10N | FSR相对INIT新增力达到该值后，记录本轮确实加载过 |
| `--fsr-release-threshold` | 0.15N | ≥0且小于加载阈值 | 只有已加载FSR降到该值以下，才开始FSR卸载计时 |
| `--comfort-scale` | 1.0 | 双手入口未额外限定 | INSPIRE力进入状态机前的整体缩放；建议0～1 |
| `--target-max` | 6.0N | 双手入口未额外限定 | 缩放后INSPIRE目标力的软件上限 |
| `--contact-on` | 0.50N | 双手入口未额外限定 | FREE中INSPIRE力达到该值后进入FORCE_ENTRY |
| `--contact-off` | 0.30N | 双手入口未额外限定 | 低于该值视为INSPIRE释放；应小于`contact-on`形成滞回 |
| `--force-kp` | 35 tick/N | >0 | LOCKED中`FSR反馈力 - INSPIRE目标力`的比例增益 |
| `--force-step-max` | 3 tick/周期 | 0.1～20 | 上述力差每周期最多修正多少INSPIRE位置；仅力控覆盖链路有实际作用 |
| `--force-deadzone` | 0.10N | ≥0 | LOCKED力误差绝对值不超过该值时不修正INSPIRE位置 |
| `--control-hz` | 20Hz | 双手入口未额外限定 | 每只手的状态机和舵机读写频率；十舵机当前建议10Hz |

上表中“未额外限定”表示双手入口当前只做了类型解析，没有为该参数
再设置独立的CLI取值范围；它不表示任意数值都安全。真机联调优先使用表中默认/建议值。

交互顺序仍是 `STATUS` → `INIT` → `ARM`。任一手未连接或未INIT时，
双手 `ARM` 被拒绝并列出具体的手别/手指/数据原因；任一指进入
`FAULT` 时十指联锁 `STOP`。`ARM成功` 只表示十指进入 `FREE`
待机，不会立即驱动舵机；单指 INSPIRE 换算力达到默认 `0.50N`
后才进入 `FORCE_ENTRY` 收绳。

XL330-M288（model 1200）在本项目中按约 `1 mA/raw` 解释。实际单指
收绳命令是 `min(Inspire力×60, 300)`，不是始终输出300；同一只手
五指 FORCE_ENTRY 请求合计超过800时按相同比例缩小。

`--enable-write` 允许写舵机；`--enable-mhandpro` 启用9301/9302力控覆盖；
`force-host/haptic-host` 指向G1；两组signs依次是`[拇,食,中,无,小]`
收绳方向；`current-slew=20` 限制每周期电流增量；`force-step-max=4`
限制LOCKED位置修正；`control-hz=10` 降低十舵机TTL负载；
`2>&1` 把错误输出合并到日志；`tee` 同时显示并保存日志。
`$(date +%Y%m%d_%H%M%S)` 在启动时生成时间戳，例如
`logs/force_test_20260827_143205.log`，因此每次实验不会覆盖上一次。

不要在 G1 上另起 `standalone_inspire_bridge.py`。`zh/机器人端/hand_driver.py`
必须是左右 INSPIRE Modbus 的唯一所有者。力控TCP断开或超时 0.5 s 后，
G1 清除 LOCKED 覆盖并恢复 mHandPro 跟随；手套指令断流仍自动张手。

> 运行位置：树莓派，独占 `/dev/serial0`  
> 当前范围：左手 ID6～10保留；通用入口支持右手 ID1～5
> 更新：2026-09-01

正式双手上位机是树莓派本地 PyQt 程序，集成 BLE/G1 连接、
STATUS/INIT/ARM/STOP/QUIT、十舵机遥测和实时曲线。它与力控共享
同一个 `PortHandler`，不另起网页服务。详见
[`../LOCAL_SUPERVISOR_CN.md`](../LOCAL_SUPERVISOR_CN.md)。

---

## 目录

1. [左手五指版](#左手五指版)
2. [左食指单指版](#左食指单指版)
3. [启动与安全条件](#启动与安全条件)
4. [状态机与参数](#状态机与参数)
5. [左右手通用入口](#左右手通用入口)

---

## 左手五指版

`left_hand_force_test.py` 复用本文下方已验证的单食指状态机，
在同一串口中串行管理五个通道：

```text
拇指: FSR0 / top_touch0 / ID6 / Inspire角度槽4
食指: FSR1 / top_touch1 / ID7 / Inspire角度槽3
中指: FSR2 / top_touch2 / ID8 / Inspire角度槽2
无名指: FSR3 / top_touch3 / ID9 / Inspire角度槽1
小指: FSR4 / top_touch4 / ID10 / Inspire角度槽0
```

## 左右手通用入口

保留 `left_hand_force_test.py` 和原左手命令。新增的
`hand_force_test.py` 复用同一套已验证状态机，通过 `--hand` 选择配置：

| 手 | FSR端口 | Dynamixel ID | Inspire力流 | mHandPro覆盖 |
|---|---:|---|---:|---:|
| 左 | 9002 | 6,7,8,9,10 | 9202 | 9302 |
| 右 | 9001 | 1,2,3,4,5 | 9201 | 9301 |

右手只读启动：

```bash
./.venv/bin/python exoskeleton/force_control/hand_force_test.py --hand right
```

右手写入首测仍必须显式确认五指电流方向。当前实测五指都与左手一致时：

```bash
./.venv/bin/python exoskeleton/force_control/hand_force_test.py \
  --hand right --enable-write \
  --drive-current-signs 1,1,1,1,1 \
  --max-goal-current 30 --release-goal-current 30 \
  --actual-current-limit 45
```

程序启动后仍依次输入 `STATUS`、`INIT`、`ARM`。`INIT` 是每次安装/佩戴的
运行基准：同时采集五个舵机当前位置和五路FSR静态预载；RELEASE返回该次
`INIT` 位置。`left_neutral.json` / `right_neutral.json` 只用于未穿戴维护、
位置漂移比较和异常恢复参考，不参与正常闭环的返回点计算。

右手程序默认要求 `127.0.0.1:9201` 已有右INSPIRE触觉流；没有有效
`top_touch` 时只允许 `STATUS`，不能完成 `INIT/ARM`。只调试舵机方向时继续使用
`tools/exo_single_servo_jog.py` 和 `exo_single_servo_current_test.py`，不要绕过该联锁。

五指共用一个Dynamixel `PortHandler`。通用控制器使用总线互斥锁，将
`INIT`、`ARM`、`STOP`和20 Hz周期控制串行化。若旧版本在ARM第四/第五指时出现
`[TxRxResult] Port is in use!`，说明主线程和控制线程同时访问了
`/dev/serial0`；应立即断电并同步最新版 `left_hand_force_test.py`，不能反复输入ARM。

每指都有独立的 `FREE -> FORCE_ENTRY -> LOCKED -> RELEASE ->
RETURN_SETTLE -> FREE` 状态。未LOCKED的手指继续跟mHandPro，已LOCKED
的手指才保持对应Inspire角度并叠加该指力差PID。任意一指通信、
硬件或电流检查故障时，五指联锁STOP。

只读启动不需方向参数：

```bash
./.venv/bin/python exoskeleton/force_control/left_hand_force_test.py \
  --enable-mhandpro
```

五指写入前必须未穿戴逐指验证“哪个电流符号收绳”。然后按
`[拇,食,中,无,小]` 顺序显式传入；例如五指都是正电流收绳：

```bash
./.venv/bin/python exoskeleton/force_control/left_hand_force_test.py \
  --enable-write --enable-mhandpro \
  --drive-current-signs 1,1,1,1,1 \
  --max-goal-current 30 --release-goal-current 30 \
  --actual-current-limit 45
```

程序启动后依次输入 `STATUS`、`INIT`、`ARM`。`INIT` 会顺序对
五指各采样0.5秒，整个过程必须保持五指、绳索和FSR预载稳定。
首轮先使用海绵分别只测一指，确认五指方向、状态转移和自动回位，
再做多指同时接触，最后才穿戴。

如果上次力控退出时某个舵机遗留在mode 0，只读启动会显示警告但仍
允许 `STATUS`，不会修改寄存器。传入 `--enable-write` 时，程序只在
`torque=0` 且 `hw_error=0` 的前提下自动将mode 0/3恢复为mode 5，
回读确认成功后才继续。扭矩已开或存在硬件错误时仍会拒绝接管。

FORCE_ENTRY刚切模式0时，齿隙和松绳回弹可能造成约10～20 tick的
瞬时反向变化，不代表电流方向错误。程序只在驱动至少0.2秒后，
累计反向运动超过100 tick才联锁；可用
`--direction-fault-threshold 100` 调整（40～500 tick）。

## 左食指单指版

`left_index_force_test.py` 仅控制左食指 Dynamixel ID 7，用于验证：

```text
Inspire fingerfour_top_touch[96] -> 取最大值 -> 旧系统换算公式
FORCE_ENTRY: Inspire力 x 60mA/N -> 模式0正电流收绳
位置稳定 -> LOCKED: 切模式5锁定，用FSR/Inspire力差调节Inspire食指
FSR相对INIT预载的新增力归零持2s -> RELEASE -> init_pos
-> RETURN_SETTLE -> FREE
STM32 FSR绝对原值 -> INIT绑带预载去皮 -> LOCKED闭环反馈
```

传 `--enable-mhandpro` 时，FORCE_ENTRY仍跟随手套；位置稳定进入LOCKED后，
9302才保持Inspire整手六通道抓取姿态，当前只对食指执行
`ftp/Force_handcontrol.py` 的力差PID。不传该参数
时保留外骨骼单独测试模式。

原程序单舵机电流上限为300mA，没有外骨骼位置行程上限。本测试默认
同样不设位置限位（`--max-travel 0`），但保留更低的30mA电流上限，不直接
套用原程序的300mA。需要额外调试限位时可显式传入，例如
`--max-travel 200`。

STM32输出仍是绝对原值，4.903N仍是物理底值，不是固件零偏。
由于指尖绑带可产生静态预载，程序在INIT额外记录 `fsr_rest_raw`：

```python
fsr_excess = max(0, raw - fsr_rest_raw - 0.05)
feedback = 0 if fsr_excess == 0 else 4.903 + fsr_excess
```

这里只减去“安装绑带的本次预载”，不改变STM32的4.903N换算。
FSR释放采用两阶段滞回：本次接触周期中`fsr_excess`必须先达到0.30N，
随后降到0.15N以下并连续保持0.30秒，才可请求RELEASE。始终停留在
INIT基线的坏通道不会获得“已加载”资格。INSPIRE触觉释放仍独立按
`--release-hold-seconds`消抖后请求RELEASE；
归位期间完全忽略新的Inspire接触。当前RELEASE最高100 raw，
距INIT 150 tick内按距离比例减流。舵机首次进入INIT±15 tick或
首次跨过INIT时，立即清电流并切模式5保持，避免在返回点
两侧反复反转直到释放超时。

## 启动与安全条件

详细四终端启动顺序见上级 `README.md`。首次先不带 `--enable-write` 运行并输入
`STATUS`。写入测试必须外骨骼未穿戴，并准备物理断电：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/force_control/left_index_force_test.py \
  --enable-write --enable-mhandpro \
  --max-goal-current 30 --release-goal-current 30
```

输入 `ARM` 后，ID 7在init_pos待机。Inspire食指换算力达到4.95N才启动；
LOCKED后，未经EMA的INSPIRE力释放持续默认0.15秒，或者已加载过的
外骨骼FSR新增力降至0.15N以下持续0.30秒，程序自动回init_pos，
再与原程序一样切模式5保持。该条件避免位置锁定持续压住FSR时形成
“不放绳就不归零、不归零就不放绳”的闭锁。归位后程序进入
`RETURN_SETTLE`：ID7位置在init_pos±15 tick内，且Inspire目标力
低于释放阈值，两个条件连续稳定0.5秒后才进入FREE。之后只有
新的Inspire接触上升沿才能再次收绳。可用
`--return-settle-time 0.5` 调整稳定时间（0.2～2.0秒）。
INSPIRE释放消抖可用 `--release-hold-seconds` 调整；FSR两阶段判断使用
`--fsr-load-threshold`、`--fsr-release-threshold`和
`--fsr-release-hold-seconds`调整。

旧 `ftp/Force_handcontrol.py` 的FORCE_ENTRY上限为300mA，约5N触觉会按
`5×60=300mA`请求；新程序默认仅30mA，因此克服绳索摩擦的转矩更小、收绳更慢。
不要直接恢复300mA。未穿戴单指验证后建议按30、40、50、60mA逐级增加，
并让 `--actual-current-limit` 至少高于目标上限10～15mA。
`STOP`、`QUIT`、通信故障和运行异常仍会关闭扭矩。

默认不盲目使用任意上电姿态。将食指和绳索摆到本轮返回起点，确认
绑好FSR并保持手指不受额外力，依次输入 `STATUS`、`INIT`、`ARM`。
`INIT`只读采样0.5秒，同时记录 `init_pos` 和本次绑带预载。默认预载
不得超过8N，0.5秒波动不得超过0.2N。

## 状态机与参数

FORCE_ENTRY不设固定寻触行程和超时；开始驱动至少0.2秒后，在0.25秒窗口内
位置变化不超过8 tick时进入LOCKED。未穿戴完成机械验证后，可显式传入
`--seek-max-travel <tick>` 和 `--seek-timeout <s>` 增加额外安全保护；0表示关闭。
也可用 `--init-pos <tick>` 显式传入已确认的返回点。

Dynamixel位置每4096 tick可能回绕。程序使用相邻采样连续跟踪多圈位置，
不会在超过半圈后强制映射回init_pos附近。这保证RELEASE能根据真实方向写入
反向电流。

某些旧probe会把负位置按32位无符号数打印。例如 `4294965108` 应换算为
`4294965108 - 4294967296 = -2188`；更新后的probe和力控脚本会自动做此转换。
