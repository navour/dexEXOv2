# Force_handcontrol — 灵巧手力反馈双边遥操作 V2

> 文件：`Force_handcontrol.py`  
> 更新：2026-04-25

---

## 目录

1. [系统架构](#系统架构)
2. [依赖安装](#依赖安装)
3. [硬件连接](#硬件连接)
4. [配置参数](#配置参数)
5. [启动顺序](#启动顺序)
6. [运行时指令](#运行时指令)
7. [状态机说明](#状态机说明)
8. [常见问题](#常见问题)

---

## 系统架构

```
外骨骼（BLE手套）
  └─ BLE → ble_broker.py (TCP 9001)
              └─ Force_handcontrol.py
                    ├─ 前13位 → 弯曲角度 → DDS rt/inspire_hand/ctrl/r → 灵巧手
                    └─ 末5位  → 外骨骼触觉力 → 状态机 → Dynamixel XL330×5

灵巧手 inspire
  └─ DDS rt/inspire_hand/touch/r → Force_handcontrol.py → 状态机反馈
```

**数据流方向：**

```
BLE 手套弯曲 ──────────────────────────► DDS 灵巧手角度指令
BLE 手套触觉(末5位) ──┐
                      ├── 状态机 ──────► Dynamixel 舵机电流/位置
DDS 灵巧手触觉 ────────┘
```

---

## 依赖安装

```bash
# 基础依赖
pip install dynamixel-sdk bleak numpy

# DDS（Unitree SDK2）
pip install unitree_sdk2py

# inspire_dds（本地包，需手动安装）
cd /path/to/inspire_hand_ws
pip install -e .
```

---

## 硬件连接

| 设备 | 接口 | 配置 |
|------|------|------|
| Dynamixel XL330×5 | `/dev/ttyAMA0` | 1 Mbps，ID = 1~5（拇/食/中/无/小） |
| BLE 数据手套 | BLE → `ble_broker.py` | TCP 127.0.0.1:9001 |
| 灵巧手 inspire | DDS（以太网）| `rt/inspire_hand/ctrl/r` / `touch/r` |

**串口权限（Pi 首次使用）：**

```bash
sudo usermod -aG dialout $USER
# 重启或重新登录
```

---

## 配置参数

所有可调参数集中在文件头部 `用户可调参数` 区块（约第 52~160 行）。

### BLE 连接

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `USE_BROKER` | `True` | `True`=TCP Broker，`False`=BLE 直连 |
| `BROKER_HOST` | `127.0.0.1` | Broker 地址 |
| `BROKER_PORT` | `9001` | Broker TCP 端口 |
| `DEVICE_ADDRESSES` | `[...]` | 直连模式 BLE MAC 地址列表 |
| `EXO_FORCE_BASELINE` | `4.903` | STM32最低有效输出阈值(N)，≤此值视为0，>此值保留原值 |

### 灵巧手角度

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `HAND_ANGLE_MIN` | `150` | 灵巧手伸直角度 tick |
| `HAND_ANGLE_MAX` | `850` | 灵巧手握拳角度 tick |
| `CONTROL_HZ` | `20` | 控制循环频率 (Hz) |

### 状态机阈值

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `HAND_CONTACT_ON_N` | `0.30 N` | 灵巧手触觉超过此值 → 进入 FORCE_ENTRY |
| `EXO_ZERO_THR_N` | `0.05 N` | 外骨骼触觉判零阈值 |
| `RELEASE_HOLD_SEC` | `0.30 s` | 触觉归零后保持时间 → 切回 GLOVE |

### 舵机力控

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `HAND_TO_SERVO_GAIN` | `60.0 mA/N` | FORCE_ENTRY：灵巧手触觉→舵机电流增益 |
| `CURRENT_LIMIT_MA` | `300 mA` | 单舵机最大电流 |
| `TOTAL_CURRENT_LIMIT_MA` | `800 mA` | 五指总电流上限 |
| `FORCE_KP` | `35.0 tick/N` | LOCKED 阶段 PID 比例增益 |
| `FORCE_STEP_MAX` | `12.0 tick` | 单周期最大位置修正 |
| `RELEASE_CURRENT_MA` | `-150 mA` | 归位阶段反向电流 |

---

## 快速启动（3 个终端）

> **启动顺序严格按照终端编号依次执行，不可颠倒。**

### 终端 1 — 灵巧手驱动（DDS 数据源）

```bash
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/ftp/inspire_hand_ws
python3 inspire_hand_sdk/example/Headless_driver_r.py
```

等待灵巧手完成自检并进入就绪状态后，再启动后续程序。

### 终端 2 — BLE Broker（**必须在终端 3 之前启动**）

```bash
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/Five_finger_force_test
python3 ble_broker.py
```

等待输出 `[BLE] 已连接 F0:FD:45:02:85:B3，等待客户端...` 后再启动终端 3。

### 终端 3 — 力反馈双边遥操作主程序

```bash
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/ftp
python3 Force_handcontrol.py
```

启动后程序按以下顺序自动完成初始化：

```
[步骤 1/4] 舵机初始化
           ├─ 打开串口 /dev/ttyAMA0 @ 1 Mbps
           ├─ Ping 舵机 ID 1~5（拇/食/中/无/小）
           ├─ 设置电流模式 + 力矩上限
           └─ 使能 Torque

[步骤 2/4] 数据手套 BLE 初始化
           └─ 连接 Broker TCP 127.0.0.1:9001

[步骤 3/4] DDS 初始化
           ├─ ChannelFactoryInitialize
           └─ 订阅 rt/inspire_hand/touch/r

[步骤 4/4] 启动控制线程
           ├─ control_loop  (20 Hz)
           └─ input_thread  (命令解析)
```

启动成功后终端打印摘要：

```
============================================================
  ✓ 系统启动完成
  ✓ 已初始化舵机   : 拇指, 食指, 中指, 无名指, 小指
  ✓ BLE 手套模式   : Broker
  ✓ DDS 触觉       : 可用
  ✓ 控制频率       : 20 Hz
  ✓ 外骨骼力基线   : 4.903 N（≤基线视为0）
============================================================
```

随后执行佩戴初始化：

```
1. 等待出现: [BLE] 已连接 Broker (127.0.0.1:9001)
2. 佩戴外骨骼手套，手自然伸直
3. 输入: INIT       ← 记录舵机初始位置（每次启动必须执行）
4. 开始遥操作
5. 随时输入 STATUS 查看各指状态
6. 输入 EXIT 或按 Ctrl+C 安全退出
```

---

## 运行时指令

| 指令 | 说明 |
|------|------|
| `INIT` | 记录当前外骨骼舵机位置为初始位置（**启动后必须执行一次**） |
| `STATUS` | 打印各指详细状态：模式、位置、电流、外骨骼力、灵巧手力 |
| `HELP` | 显示指令列表 |
| `EXIT` | 安全退出（自动关闭力矩） |

---

## 状态机说明

每根手指独立运行以下状态机：

```
         灵巧手触觉 > 0.30 N
GLOVE ──────────────────────► FORCE_ENTRY
  ▲                                │
  │                    舵机位置稳定(0.4s)
  │                                ▼
  │                            LOCKED
  │                                │
  │          外骨骼触觉≈0 持续 0.30s│
  │                                ▼
  └─────────────────────────── RELEASE
           舵机归位完成
```

| 状态 | 灵巧手 | 舵机 |
|------|--------|------|
| `GLOVE` | 手套弯曲角度控制 | 零电流，自由 |
| `FORCE_ENTRY` | 冻结在接触时角度 | 电流 = 灵巧手触觉 × 60 mA/N |
| `LOCKED` | PID 调节（目标力=外骨骼触觉，反馈=灵巧手触觉） | 位置模式锁死 |
| `RELEASE` | 冻结 | 反向 -150 mA 归位 |

---

## 常见问题

**Q: 舵机 Ping 失败**  
A: 检查 `/dev/ttyAMA0` 权限，确认波特率一致（1 Mbps），确认 ID 编号。

**Q: BLE 长时间未连接**  
A: 确认 `ble_broker.py` 已运行；检查 MAC 地址；重启手套蓝牙。

**Q: 外骨骼触觉始终为 0**  
A: BLE 数据末5位全部 ≤ `EXO_FORCE_BASELINE(4.903)`，属于正常待机状态。
   STM32当前无法分辨0～约4.9N；不应在树莓派端降低阈值来伪造低力测量。

**Q: 状态机不进入 FORCE_ENTRY**  
A: 灵巧手触觉未超过 `HAND_CONTACT_ON_N = 0.30 N`；  
   检查 DDS 话题 `rt/inspire_hand/touch/r` 是否有数据（`STATUS` 命令查看 DDS age）。

**Q: 舵机电流过大/发热**  
A: 降低 `HAND_TO_SERVO_GAIN`（默认 60 mA/N）或 `CURRENT_LIMIT_MA`（默认 300 mA）。
