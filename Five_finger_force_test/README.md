# 五指力反馈控制系统

五根手指独立 PID 力闭环控制，基于 Dynamixel XL330-M288T 舵机 + BLE 触觉传感器 + DDS 灵巧手触觉数据。

## 系统架构

```
灵巧手 (192.168.123.210)         STM32 BLE 手套
    │ Modbus TCP                       │ BLE
    ▼                                  │
Headless_driver_r.py (终端1)           │
    │ DDS                              │
    ▼                                  ▼
dds_to_force.py (终端3)       finger_force.py (终端2)
    │ TCP N:ID:力值                    │
    └──────────────────────────────────┘
                                       │ 串口 /dev/ttyAMA0
                                       ▼
                            5× XL330-M288T 舵机
                            (拉绳驱动手套力反馈)
```

## 手指 → 舵机 → DDS 映射

| 手指   | 舵机ID | BLE索引 | DDS 字段              |
|--------|--------|---------|----------------------|
| 大拇指 | 1      | [0]     | fingerfive_top_touch  |
| 食指   | 2      | [1]     | fingerfour_top_touch  |
| 中指   | 3      | [2]     | fingerthree_top_touch |
| 无名指 | 4      | [3]     | fingertwo_top_touch   |
| 小指   | 5      | [4]     | fingerone_top_touch   |

## BLE 数据格式

STM32 发送:
```json
[大拇指N, 食指N, 中指N, 无名指N, 小指N]
```
五个力值 (N) 分别对应五根手指，一一映射到舵机 1~5。

## 安装依赖

```bash
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/Five_finger_force_test
pip3 install -r requirements.txt
```

## 运行 (3个终端)

### 终端 1: 灵巧手驱动 (发布 DDS 触觉数据)

```bash
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/ftp/inspire_hand_ws
python3 inspire_hand_sdk/example/Headless_driver_r.py
```

### 终端 2: 舵机控制 + BLE 接收

```bash
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/Five_finger_force_test
python3 finger_force.py
```

启动后执行佩戴初始化:
```
> INIT
```

### 终端 3: DDS → 力值桥接

```bash
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/Five_finger_force_test
python3 dds_to_force.py
```

## 控制指令

在 finger_force.py 终端输入:

| 指令 | 说明 | 示例 |
|------|------|------|
| `INIT` | 佩戴初始化 (记录所有舵机位置) | `INIT` |
| `INIT:ID` | 初始化单个舵机 | `INIT:2` (食指) |
| `N:ID:力值` | 设定目标力 | `N:1:5` (大拇指5N) |
| `NALL:力值` | 全部设定 | `NALL:3` |
| `N:ID:0` | 关闭并释放 | `N:1:0` |
| `STATUS` | 查看状态 | `STATUS` |
| `FREE:ID` | 自由模式 | `FREE:1` |
| `HELP` | 帮助 | `HELP` |
| `EXIT` | 退出 | `EXIT` |

## 工作流程

1. 启动三个终端的程序
2. 在终端2输入 `INIT` 完成佩戴初始化
3. 触摸灵巧手手指 → DDS 检测 → 转为力值 → TCP 发送 → 舵机 PID 拉绳
4. 松开手指 → DDS 力降为0 → 舵机反向释放绳索 → 回到初始位置
5. BLE 手套传感器提供实时反馈力，PID 闭环调节

## 与单指版 (Finger_force_test) 的区别

| 项目 | 单指版 | 五指版 |
|------|--------|--------|
| 控制舵机 | 仅舵机2 (食指) | 全部5个舵机 |
| BLE 数据 | 单个力值广播到所有舵机 | 5个力值一一对应 |
| DDS 字段 | 仅 fingerfour_top_touch | 5个手指各自的 top_touch |
| TCP 命令 | N:2:力值 | N:1~5:力值 (批量发送) |

## PID 参数 (可调)

```python
PID_KP = 80.0           # 比例系数 (mA/N)
PID_KI = 20.0           # 积分系数
PID_KD = 5.0            # 微分系数
PID_DEADZONE = 0.15     # 死区 (N)
PID_OUTPUT_MAX = 400.0  # 电流上限 (mA)
PID_OUTPUT_MIN = -200.0 # 电流下限 (mA)
BLE_FORCE_BASELINE = 4.903  # STM32最低有效输出阈值 (N)；≤阈值为0，>阈值保留原值
```

## 网络配置

灵巧手 IP: `192.168.123.210`，树莓派 eth0 需要配置为 `192.168.123.100/24`。

```bash
# 临时
sudo ip addr add 192.168.123.100/24 dev eth0

# 永久 (写入 /etc/rc.local 的 exit 0 之前)
ip addr add 192.168.123.100/24 dev eth0 2>/dev/null || true
```
