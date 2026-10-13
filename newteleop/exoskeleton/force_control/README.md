# 双手 INSPIRE 力反馈控制器

> 更新：2026-10-09
>
> 运行位置：树莓派；默认只读，写模式必须显式给出双手方向。

---

## 目录

1. [系统架构](#系统架构)
2. [端口分配](#端口分配)
3. [文件清单](#文件清单)
4. [启动顺序](#启动顺序)
5. [配置参数](#配置参数)
6. [状态机说明](#状态机说明)
7. [常见问题](#常见问题)

---

## 系统架构

外骨骼闭环复用旧 ftp 的接触、限流、锁定、释放思路；弯曲动作来自 mHandPro，不再解析旧 BLE 弯曲通道。双手入口共享一条 Dynamixel 总线。

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

| 文件 | 范围 |
|---|---|
| `dual_hand_force_test.py` | 正式双手 ID1～10 |
| `hand_force_test.py` | `--hand right/left` 单手 |
| `left_hand_force_test.py` | 通用五指底层控制器 |
| `left_index_force_test.py` | 单指状态机及左食指早期入口 |

单手/单指属于调试入口；其默认 host/阈值不同，使用前看各自 `--help`。不能同时运行两个单手进程控制同一串口。

---

## 启动顺序

G1 启动 `bash ~/zh/start.sh --hand both --hand-only --hand-haptic --hand-range 0.3`。
不使用 GUI 时，在 Pi 终端一运行：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/FSR/ble_broker.py --hand both
```

Pi 终端二先只读：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/force_control/dual_hand_force_test.py --force-host 192.168.3.78
```

退出后，低电流测试：

```bash
./.venv/bin/python exoskeleton/force_control/dual_hand_force_test.py \
  --enable-write \
  --force-host 192.168.3.78 \
  --right-drive-current-signs 1,1,1,1,1 \
  --left-drive-current-signs 1,1,1,1,1 \
  --max-goal-current 30 --release-goal-current 30 \
  --actual-current-limit 45 --current-slew 5 --control-hz 10
```

依次输入 `STATUS → INIT → STATUS → ARM`，异常 `STOP`，结束 `QUIT`。
需要双边位置修正时再加 `--enable-mhandpro --haptic-host 192.168.3.78`，并按主 README 启动 zh 的 PC 手套链路。

---

## 配置参数

| 参数 | 双手默认 | 含义 |
|---|---:|---|
| `--max-goal-current` | 300 raw | 收绳目标上限，首测显式降为30 |
| `--release-goal-current` | 100 raw | 回位上限，首测30 |
| `--actual-current-limit` | 320 raw | 实测电流故障值，首测45 |
| `--current-slew` | 20 raw/周期 | 收绳电流爬升，首测5 |
| `--hand-total-current-limit` | 800 raw | 每手 FORCE_ENTRY 请求总和限幅 |
| `--force-to-current-gain` | 60 raw/N | INSPIRE 力到电流 |
| `--contact-on / --contact-off` | 0.50 / 0.30 N | INSPIRE 接触滞回 |
| `--force-kp / --force-step-max` | 35 / 3 | LOCKED 比例增益和每周期位置步长 |
| `--force-deadzone` | 0.10 N | 力误差死区 |
| `--fsr-rest-max` | 8 N | INIT 静息预载上限 |
| `--fsr-load-threshold` | 0.30 N | 相对 INIT 的加载确认 |
| `--fsr-release-threshold` | 0.15 N | 已加载后的卸载判据 |
| `--fsr-release-hold-seconds` | 0.30 s | 卸载消抖 |
| `--release-hold-seconds` | 0.15 s | INSPIRE 释放消抖 |
| `--release-timeout` | 12 s | 回位超时故障 |
| `--return-settle-time` | 0.5 s | 返回后的稳定时间 |
| `--control-hz` | 20 Hz | 双手首测显式10 |

完整参数：`python exoskeleton/force_control/dual_hand_force_test.py --help`。
示例方向全 +1 仅对应已有机械记录。不要提高 FSR 静息上限来绕过坏通道。

---

## 状态机说明

`STOP → INIT/ARM → FREE → FORCE_ENTRY → LOCKED → RELEASE → RETURN_SETTLE → FREE`。
LOCKED 需要 FSR 已加载且舵机位置稳定，不再仅凭静止判定。
FSR 原始值先减本轮 INIT 预载及0.05N死区，新增力为零则反馈为零；否则沿用 `4.903 + 新增力`。加载/卸载判断使用新增力，不能混用原始值。
FREE 跟手套；FORCE_ENTRY 限流收绳；LOCKED 固定外骨骼并发送逐指位置增量；RELEASE 回本轮 INIT。当前“PID”实际为 P 项。
同一只手总电流限制针对 FORCE_ENTRY 收绳请求；并非对所有模式的硬件电流总和作保证。
任一指故障十指停止。默认未设置额外寻触行程/超时上限，首测需未穿戴并能立即断电。

---

## 常见问题

ARM 拒绝时看数据新鲜度、INIT 和覆盖连接；ARM 成功仅代表 FREE 待机。
GUI 与 CLI 不能并行。TCP 覆盖在线不是执行回执。
现有 zh 驱动中，仍有效的 LOCKED 覆盖可能覆盖手套断流后的张手目标。结束测试或数据异常时先 STOP 外骨骼，再检查并重新 INIT/ARM。
