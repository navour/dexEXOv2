# 五指力反馈系统 — 参数解释与调参指南

## 系统架构简述

```
DDS触觉数据 → dds_to_force.py → TCP → finger_force.py → PID闭环 → 舵机电流
                                          ↑
                                   BLE反馈力(手套)
```

**核心原理**：`电流 = 力矩`。写入舵机的 goal_current 直接控制输出扭矩，正电流拉紧绳索，负电流反向释放。

---

## 一、PID 力闭环参数

> 这是最核心的参数组，决定力反馈的响应速度和稳定性

| 参数 | 当前值 | 单位 | 含义 |
|---|---|---|---|
| `PID_KP` | 50.0 | mA/N | 比例增益：误差×KP=输出电流 |
| `PID_KI` | 20.0 | mA/(N·s) | 积分增益：消除稳态误差 |
| `PID_KD` | 5.0 | mA/(N/s) | 微分增益：抑制误差变化率 |
| `PID_DEADZONE` | 0.15 | N | 死区：误差小于此值视为0 |
| `PID_OUTPUT_MAX` | 400 | mA | PID输出电流上限（拉紧方向） |
| `PID_OUTPUT_MIN` | -200 | mA | PID输出电流下限（释放方向） |
| `PID_INTEGRAL_MAX` | 300.0 | mA | 积分项输出限幅，防止积分饱和 |

### 调参指南

**公式**：`error = 目标力 - 反馈力`，`输出电流 = KP×error + KI×∫error + KD×d(error)/dt`

| 现象 | 原因 | 调整方法 |
|---|---|---|
| 🔴 舵机力度不够，拉不动 | KP太小，电流输出不足 | **增大 KP**（如50→80），或增大 PID_OUTPUT_MAX |
| 🔴 舵机来回振荡（抖动） | KP太大，过冲后反复修正 | **减小 KP**（如50→30），或**增大 KD**（如5→10） |
| 🔴 舵机慢慢才达到目标力 | KI太小，稳态误差消除慢 | **增大 KI**（如20→40） |
| 🔴 舵机到达目标后缓慢漂移 | KI积分累积过大 | **减小 PID_INTEGRAL_MAX**（如300→150） |
| 🔴 在目标力附近小幅抖动 | 死区太小，微小噪声触发修正 | **增大 PID_DEADZONE**（如0.15→0.3） |
| 🔴 力度到了但不精确（有偏差）| 死区太大，小误差被忽略 | **减小 PID_DEADZONE**（如0.15→0.05） |
| 🔴 电流打满但力还不够 | OUTPUT_MAX限制了输出 | **增大 PID_OUTPUT_MAX**（如400→500，注意不超过舵机极限） |

### 调参顺序（推荐）

```
1. 先调 KP：从小到大增加，直到舵机能快速响应但不振荡
2. 再调 KD：从小到大增加，用来抑制振荡（KD是KP的"刹车"）
3. 最后调 KI：从小到大增加，消除稳态偏差
4. 微调 DEADZONE：根据传感器噪声水平设置
```

---

## 二、滤波参数

> 平滑输入信号，减少噪声引起的抖动

| 参数 | 当前值 | 范围 | 含义 |
|---|---|---|---|
| `TARGET_FILTER_ALPHA` | 0.3 | 0~1 | 目标力低通滤波系数 |
| `FEEDBACK_FILTER_ALPHA` | 0.4 | 0~1 | BLE反馈力低通滤波系数 |

**公式**：`filtered = filtered + alpha × (raw - filtered)`

| 现象 | 调整方法 |
|---|---|
| 🔴 目标力变化时舵机响应太慢（手指按下后延迟明显） | **增大 TARGET_FILTER_ALPHA**（如0.3→0.6），越大越灵敏 |
| 🔴 目标力变化时舵机抖动/跳动 | **减小 TARGET_FILTER_ALPHA**（如0.3→0.15），越小越平滑 |
| 🔴 BLE反馈数据噪声大，PID输出抖动 | **减小 FEEDBACK_FILTER_ALPHA**（如0.4→0.2） |
| 🔴 BLE反馈响应迟钝，顺应模式进入慢 | **增大 FEEDBACK_FILTER_ALPHA**（如0.4→0.7） |

> ⚠️ alpha=1.0 表示不滤波（原始值直通），alpha=0.01 表示极强平滑（几乎不跟随变化）

---

## 三、顺应模式参数

> 当实际触觉力**超过**目标力时，舵机快速释放，防止过度挤压

| 参数 | 当前值 | 单位 | 含义 |
|---|---|---|---|
| `COMPLY_THRESHOLD` | 0.3 | N | 触发顺应的超额阈值 |
| `COMPLY_RELEASE_GAIN` | 20 | mA/N | 超额力→释放电流的增益 |

**触发条件**：`反馈力 > 目标力 + COMPLY_THRESHOLD`
**输出电流**：`-COMPLY_RELEASE_GAIN × (反馈力 - 目标力)`

| 现象 | 调整方法 |
|---|---|
| 🔴 手指碰到物体后舵机不松，挤压感强 | **减小 COMPLY_THRESHOLD**（如0.3→0.1）或**增大 COMPLY_RELEASE_GAIN**（如20→40） |
| 🔴 正常拉紧过程中频繁进入顺应模式 | **增大 COMPLY_THRESHOLD**（如0.3→0.5） |
| 🔴 进入顺应后释放太猛（舵机弹回） | **减小 COMPLY_RELEASE_GAIN**（如20→10） |
| 🔴 进入顺应后释放太慢 | **增大 COMPLY_RELEASE_GAIN**（如20→40） |

---

## 四、主动释放参数

> 当目标力=0时，舵机主动反向转动回到初始位置

| 参数 | 当前值 | 单位 | 含义 |
|---|---|---|---|
| `RELEASE_CURRENT_MA` | -150.0 | mA | 释放时的反向电流（控制回退力矩/速度） |
| `RELEASE_POSITION_THRESHOLD` | 15 | 编码器 tick | 距离初始位置小于此值时视为"到达" |
| `RELEASE_TIMEOUT` | 3.0 | 秒 | 释放超时，超过后强制结束 |
| `RELEASE_SLOWDOWN_RANGE` | 50 | 编码器 tick | 接近初始位置时开始减速的距离范围 |
| `RELEASE_MIN_TIME` | 0.3 | 秒 | 释放最小运行时间，防止刚进入就判定到达 |
| `RELEASE_MIN_CURRENT` | -30.0 | mA | 释放最小电流，防止很近时电流=0不动 |

| 现象 | 调整方法 |
|---|---|
| 🔴 松手后舵机回退太慢 | **增大 RELEASE_CURRENT_MA 绝对值**（如-150→-250） |
| 🔴 松手后舵机回退时冲过初始位置（过冲） | **减小 RELEASE_CURRENT_MA 绝对值**（如-150→-80），或**减小 RELEASE_SLOWDOWN_RANGE**调低减速区 |
| 🔴 回到初始位置附近时卡住不动 | **增大 RELEASE_MIN_CURRENT 绝对值**（如-30→-50） |
| 🔴 释放完成后舵机位置有偏差 | **减小 RELEASE_POSITION_THRESHOLD**（如15→5），精度更高但耗时更久 |
| 🔴 释放刚开始就结束了 | **增大 RELEASE_MIN_TIME**（如0.3→0.5） |

---

## 五、BLE 配置

| 参数 | 当前值 | 含义 |
|---|---|---|
| `BLE_FORCE_BASELINE` | 4.903 | STM32最低有效输出阈值（N），≤此值视为0，超过后保留原值 |
| `BLE_KEEPALIVE_INTERVAL` | 3.0s | 心跳包发送间隔 |
| `BLE_DATA_TIMEOUT` | 15.0s | 数据超时阈值，超时后断开重连 |
| `BLE_RECONNECT_DELAY` | 1.0s | 重连等待时间 |

| 现象 | 调整方法 |
|---|---|
| 🔴 手套没碰东西但舵机有微小输出 | 传感器零漂，**增大 BLE_FORCE_BASELINE**（如4.903→4.95） |
| 🔴 轻触时没反应（力太小检测不到） | STM32当前低端固定输出约4.903N，树莓派不能恢复0～4.9N信息 |
| 🔴 BLE频繁断连重连 | **增大 BLE_DATA_TIMEOUT**（如15→30） |

---

## 六、硬件/运动参数

| 参数 | 当前值 | 含义 |
|---|---|---|
| `BAUDRATE` | 1000000 | 串口波特率，影响PID循环频率 |
| `currentLimit_mA` | 450 | 舵机硬件电流上限（代码中 ServoController 类） |
| `profile_velocity`（setup） | 100 | 位置模式下运动速度（仅电流-位置模式生效） |
| `profile_acceleration`（setup） | 50 | 位置模式下加速度 |
| `profile_velocity`（释放完成） | 50 | 释放完成归位时的低速 |
| `profile_acceleration`（释放完成）| 30 | 释放完成归位时的低加速度 |

| 现象 | 调整方法 |
|---|---|
| 🔴 通信报错 "no status packet" | 波特率不匹配，先运行 `change_baudrate.py` 统一波特率 |
| 🔴 PID循环频率低（<50Hz），控制迟钝 | **提升 BAUDRATE**（已从57600升到1Mbps） |
| 🔴 归位时冲过头 | **减小 profile_velocity/acceleration** |
| 🔴 舵机电流不够用 | **增大 currentLimit_mA**（最大不超过XL330极限1000mA） |

---

## 七、调试开关

| 参数 | 当前值 | 建议 |
|---|---|---|
| `PID_DEBUG` | False | 调参时开启（True），正常运行关闭。开启后每秒打印PID详情，会占用串口带宽 |
| `BLE_DEBUG` | False | BLE连接问题时开启，正常运行关闭 |

---

## 八、快速调参场景速查表

| 场景 | 优先调整 | 方向 |
|---|---|---|
| **力度不够** | KP ↑, PID_OUTPUT_MAX ↑ | 增大 |
| **振荡抖动** | KP ↓, KD ↑, FILTER_ALPHA ↓ | KP减小，KD增大 |
| **响应太慢** | KP ↑, FILTER_ALPHA ↑ | 增大 |
| **松手后回不去** | RELEASE_CURRENT_MA ↑(绝对值), RELEASE_MIN_CURRENT ↑(绝对值) | 增大绝对值 |
| **松手后弹过头** | RELEASE_CURRENT_MA ↓(绝对值), RELEASE_SLOWDOWN_RANGE ↑ | 减小电流，加大减速区 |
| **碰到物体挤压感** | COMPLY_THRESHOLD ↓, COMPLY_RELEASE_GAIN ↑ | 更灵敏的顺应 |
| **传感器噪声大** | DEADZONE ↑, FEEDBACK_FILTER_ALPHA ↓ | 加大死区和滤波 |

---

## 九、电流 vs 力矩 vs 速度 的关系

```
电流(mA) ──控制──→ 力矩(扭矩) ──对抗负载──→ 速度

• 电流直接决定力矩大小和方向
• 正电流 → 正方向扭矩（拉紧绳索）
• 负电流 → 反方向扭矩（释放/回退）
• 速度 = f(力矩, 负载)：负载轻则快，负载重则慢
• 想让舵机"慢慢拉" → 减小电流（减小KP或OUTPUT_MAX）
• 想让舵机"快速拉" → 增大电流（增大KP或OUTPUT_MAX）
```

> **重点**：在电流控制模式下，你无法直接控制速度。速度是力矩和负载平衡的结果。如果想限制速度，需要切换到电流-位置模式（`OP_CURRENT_BASED_POSITION`）并设置 `profile_velocity`。
