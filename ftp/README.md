# 灵巧手力反馈控制系统

BLE 弯曲传感器手套 → 实时标定 → EMA 滤波 → DDS 控制因时灵巧手关节角度  
同时通过 `finger_force.py` 驱动 5 路 Dynamixel XL330 提供触觉力反馈。

---

## 系统架构

```
STM32 BLE 手套 (F0:FD:45:02:85:B3)
        │ BLE GATT Notify
        ▼
  ble_broker.py  ─── TCP 9001 广播原始字节 ───┬─────────────────────────────┐
  (唯一 BLE 进程)                              │                             │
                                               ▼                             ▼
                                   ble_hand_control.py          finger_force.py
                                   (BLE→角度→DDS)               (BLE→PID→舵机力)
                                               │                             │
                                               │ DDS                         │ 串口 /dev/ttyAMA0
                                               ▼                             ▼
                                     因时灵巧手右手                  5× XL330-M288T
                                  (192.168.123.210)               拉绳手套力反馈
                                  rt/inspire_hand/ctrl/r
```

> **关键设计**：`ble_broker.py` 独占 BLE 连接，两个控制程序均通过 TCP localhost:9001 接收相同的原始字节流，无需争抢蓝牙资源。

---

## 目录结构

```
ftp/
├── ble_hand_control.py   # 弯曲传感器 → 灵巧手控制（主程序）
├── hand_control.py       # 手动控制工具（伸直/握拳/指定位置）
├── dds_debug_all_fingers.py  # DDS 触觉数据调试工具
├── test_touch.py         # 触觉传感器测试
├── touch_calibration.py  # 触觉标定工具
├── inspire_hand_ws/      # 因时灵巧手 SDK 及 DDS 接口
└── README.md             # 本文档

../Five_finger_force_test/
├── ble_broker.py         # BLE 数据代理（必须最先启动）
├── finger_force.py       # 5 指 PID 力反馈控制
└── dds_to_force.py       # DDS 触觉 → 力值 TCP 桥接
```

---

## 硬件配置

| 组件 | 参数 |
|---|---|
| 树莓派 | RPi 5，IP `10.42.0.174` |
| 灵巧手 | 因时右手，IP `192.168.123.210` |
| BLE 手套 | STM32，MAC `F0:FD:45:02:85:B3` / `F0:FD:45:02:67:3B` |
| 舵机串口 | `/dev/ttyAMA0`，波特率 1,000,000 |
| 虚拟环境 | `/home/pi/dexEXO/dexEXO/bin/activate` |

---

## 安装依赖

```bash
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/ftp
pip install bleak numpy
```

---

## 快速启动（4 个终端）

### 终端 1 — 灵巧手驱动（DDS 数据源）

```bash
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/ftp/inspire_hand_ws
python3 inspire_hand_sdk/example/Headless_driver_r.py
```

### 终端 2 — BLE Broker（**必须最先启动**）

```bash
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/Five_finger_force_test
python3 ble_broker.py
```

等待输出 `[BLE] 已连接 F0:FD:45:02:85:B3，等待客户端...` 后再启动后续程序。

### 终端 3 — 灵巧手姿态控制

```bash
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/ftp
python3 ble_hand_control.py
```

启动后根据提示完成**握拳→伸直**两步标定，之后进入实时控制循环。

### 终端 4 — 五指力反馈控制（可选）

```bash
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/Five_finger_force_test
python3 finger_force.py
```
启动后执行佩戴初始化:
```
> INIT
```
### 终端 5 — DDS → 力值桥接

```bash
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/Five_finger_force_test
python3 dds_to_force.py
```
---

## BLE 数据格式

STM32 发送 18 通道弯曲传感器原始值，两种格式均支持：

```
# 格式1：JSON
{"bend_sensors": [v0, v1, ..., v17]}

# 格式2：纯数据（当前固件）
{v0, v1, v2, ..., v15, 4.90, 4.90, 4.90, 4.90, 4.90}
```

`ble_hand_control.py` 从 18 个通道中按 `SENSOR_INDEX` 选取 6 个通道，映射到灵巧手 6 个关节。

---

## 传感器通道 → 关节映射

灵巧手 6 个关节定义（`pos_set` / `angle_set` 顺序）：

| 索引 | 关节 | 默认传感器通道 (`SENSOR_INDEX`) |
|---|---|---|
| 0 | 小指 | 通道 2 |
| 1 | 无名指 | 通道 3 |
| 2 | 中指 | 通道 4 |
| 3 | 食指 | 通道 6 |
| 4 | 大拇指弯曲 | 通道 10 |
| 5 | 大拇指旋转 | 通道 9 |

修改 `ble_hand_control.py` 顶部的 `SENSOR_INDEX` 来重新映射通道：

```python
SENSOR_INDEX = [2, 3, 4, 6, 10, 9]   # [小指, 无名指, 中指, 食指, 拇指弯, 拇指旋]
```

---

## 关节角度范围

```python
HAND_ANGLE_MIN = 150   # 伸直（弯曲传感器值大 → 映射到此）
HAND_ANGLE_MAX = 850   # 握拳（弯曲传感器值小 → 映射到此）
```

传感器值与角度方向**相反**（传感器值越大 = 手指伸直 = 角度值越小），已在 `bend_to_angle()` 中取反处理。

---

## 标定流程

每次启动 `ble_hand_control.py` 均需完成两步标定：

1. **握拳（弯曲极限）** — 按回车后保持握拳 2 秒，采集弯曲参考值 `bend_max_ref`
2. **伸直（伸直极限）** — 按回车后保持伸直 2 秒，采集伸直参考值 `bend_min_ref`

标定完成后进入实时控制，无需重启。

标定参数：

```python
CALIBRATION_DURATION = 2.0   # 每个姿态采样时长（秒），可增大提高稳定性
DEFAULT_BEND_MIN = 1500       # 未标定时的后备最小值
DEFAULT_BEND_MAX = 4000       # 未标定时的后备最大值
```

---

## EMA 滤波参数

```python
USE_FILTER = True    # 是否启用滤波
EMA_ALPHA  = 0.2     # 平滑系数，范围 (0, 1)
                     # 越小：越平滑，延迟越大
                     # 越大：响应越快，抖动越多
                     # 推荐范围：0.1（极平滑）~ 0.5（快速响应）
```

---

## BLE Broker 配置

```python
# ble_broker.py 和 ble_hand_control.py 中保持一致
BROKER_HOST = "127.0.0.1"
BROKER_PORT = 9001

# ble_hand_control.py 开关
USE_BROKER = True    # True=通过 Broker 共享连接；False=直连 BLE（单独运行时）
```

**两个程序同时运行时必须使用 Broker 模式**，否则两个进程争抢同一 BLE 设备会导致连接失败。

---

## 控制频率

```python
CONTROL_HZ = 20   # 控制循环频率（Hz）
```

提高频率可降低延迟，但过高会增加 CPU 负载和 DDS 消息密度。建议范围：10 ~ 50 Hz。

---

## 手动控制工具（hand_control.py）

不需要 BLE，直接通过 DDS 命令灵巧手：

```bash
python3 hand_control.py
```

```python
from hand_control import HandController
h = HandController('r')   # 'r' 右手, 'l' 左手
h.open_hand()             # 伸直
h.close_hand()            # 握拳
h.set_position([500, 500, 500, 500, 500, 250])   # 指定位置（0=伸直, 1000=握拳）
```

`pos_set` 顺序：`[小指, 无名指, 中指, 食指, 大拇指弯曲, 大拇指旋转]`

---

## 常见问题

### BLE 连接失败

```
[BLE] 连接 Broker 失败: Connection refused
```
**原因**：`ble_broker.py` 未启动或尚未连上蓝牙设备。  
**解决**：先启动 `ble_broker.py`，等待其输出已连接提示后再启动其他程序。

---

### 30 秒内未收到数据

```
[BLE] 超时：30秒内未收到数据，请检查 ble_broker.py 是否已连接蓝牙
```
**原因**：BLE 设备未开机，或 MAC 地址有变化。  
**解决**：检查 `ble_broker.py` 输出；若 MAC 变化，更新 `DEVICE_ADDRESSES`。

---

### 标定范围太小警告

```
[WARN] 通道 X 标定范围太小: min=XXXX max=XXXX
```
**原因**：握拳和伸直时该通道传感器变化不明显（可能电极未接触手指）。  
**解决**：重新标定，确保该手指做出充分的弯曲和伸直动作；或调整 `SENSOR_INDEX` 更换通道。

---

### 灵巧手无响应

检查 DDS 网络：灵巧手 IP `192.168.123.210`，树莓派需配置同网段地址：

```bash
sudo ip addr add 192.168.123.100/24 dev eth0
```

---

### e1000e 网卡断线（重启后无法连接树莓派）

```bash
sudo modprobe e1000e
# 永久修复：
echo 'e1000e' | sudo tee -a /etc/modules
```

---

## DDS 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `rt/inspire_hand/ctrl/r` | `inspire_hand_ctrl` | 右手控制指令（角度/位置/力/速度） |
| `rt/inspire_hand/touch/r` | — | 右手触觉数据（由 Headless_driver_r.py 发布） |

`inspire_hand_ctrl` 字段：

```python
inspire_hand_ctrl(
    pos_set   = [0~1000, ×6],    # 位置（0=伸直, 1000=握拳）
    angle_set = [0~1000, ×6],    # 角度
    force_set = [0~1000, ×6],    # 力（默认 200）
    speed_set = [0~1000, ×6],    # 速度（默认 500）
    mode      = 0b0001            # 0x01=角度控制, 0x02=位置控制
)
```
