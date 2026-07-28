# Finger Force BLE Controller

该目录包含树莓派端的五舵机力反馈控制脚本，使用 BLE 从 STM32 接收 `touch_sensors` 并作为力反馈闭环的反馈量。

## 硬件要求

- 树莓派 5
- Dynamixel XL330-M288T 舵机 × 5（IDs: 1-5）
- STM32F103 BLE 模块（触觉传感器）
- Inspire 灵巧手右手（可选，用于 DDS 触觉数据）

## 依赖

- Python 3.8+
- `bleak` - BLE 通信
- `DynamixelSDK` - 舵机控制
- `unitree_sdk2py` 和 `inspire_hand_sdk` - DDS 触觉数据（可选）

## 安装依赖

在树莓派上执行：

```bash
# 激活虚拟环境
source /home/pi/dexEXO/dexEXO/bin/activate

# 安装依赖
cd /home/pi/dexEXO/Finger_force_test
pip3 install -r requirements.txt
```

## 网络配置

### 1. 配置树莓派网络连接灵巧手

灵巧手右手的 IP 地址为 `192.168.123.211`。树莓派需要配置在同一网段 `192.168.123.x`。

**在树莓派本地终端执行（需要显示器和键盘）：**

```bash
# 临时配置（仅当前会话）
sudo ip addr add 192.168.123.100/24 dev eth0

# 验证配置
ip addr show eth0
# 应该能看到两个 IP：10.42.0.174 和 192.168.123.100
```

**永久配置（重启后仍生效）：**

```bash
# 编辑 rc.local
sudo nano /etc/rc.local

# 在 exit 0 之前添加这两行：
ip addr add 192.168.123.100/24 dev eth0 2>/dev/null || true

# 保存（Ctrl+O，Ctrl+X）
# 重启树莓派
sudo reboot
```

### 2. 验证网络连接

重启后，在树莓派上测试：

```bash
# 检查 IP 地址
ip addr show eth0

# 测试连接到灵巧手
ping -c 4 192.168.123.211
```

如果 ping 成功，网络配置完成 ✓

## 程序运行

### 启动顺序（需要 3 个终端）

#### 终端 1：启动灵巧手驱动（发布 DDS 触觉数据）

```bash
cd /home/pi/dexEXO/ftp/inspire_hand_ws

# 激活虚拟环境
source /home/pi/dexEXO/dexEXO/bin/activate

# 启动右手驱动
python3 inspire_hand_sdk/example/Headless_driver_r.py
```

预期输出：
```
当前频率: 50.00 Hz, 调用次数: 500, 耗时: 10.000000 秒
```



#### 终端 2：启动舵机控制和 BLE 接收

```bash
# 激活虚拟环境
source /home/pi/dexEXO/dexEXO/bin/activate

cd /home/pi/dexEXO/Finger_force_test
python3 finger_force.py
```

预期输出：
```
五舵机力反馈系统就绪
=== WiFi Server Listening on 0.0.0.0:8888 ===
[BLE] 尝试连接: F0:FD:45:02:85:B3
[BLE] 已连接: F0:FD:45:02:85:B3
[DDS] 正在初始化...
[DDS] 订阅话题: rt/inspire_hand/touch/r
```
#### 终端 3：启动 DDS → 力值转换桥接

```bash
# 激活虚拟环境
source /home/pi/dexEXO/dexEXO/bin/activate

cd /home/pi/dexEXO/Finger_force_test
python3 dds_to_force.py
```

预期输出：
```
==================================================
DDS 触觉数据转力值桥接程序
==================================================
[TCP] 已连接到 127.0.0.1:8888
[DDS] 订阅话题: rt/inspire_hand/touch/r
[就绪] 开始接收 DDS 触觉数据...
[DDS] top_max=5234 -> force=0.950N | sent=0.95N | msgs=10
```

## 佩戴与初始化步骤（非常重要）

系统启动后，**必须**执行初始化才能确保舵机在灵巧手松开时能正确释放绳索并回退：

1. **佩戴手套**：穿戴好力反馈手套，使手指处于自然舒展、绳索刚好拉直但不受力的状态。
2. **发送初始化指令**：在 **终端 2**（运行 `finger_force.py` 的进程）的控制台中，手动输入 `INIT` 并按回车。
   ```text
   > INIT
   ```
3. **确认生效**：终端会打印出记录的初始位置。系统依赖此位置，在失去力反馈目标（力降为0）时主动反转回退，以释放绳索张力。
   ```text
   === 佩戴初始化 ===
     舵机1: 初始位置记录为 0
     舵机2: 初始位置记录为 512
     ...
   初始化完成! 目标力=0时舵机将回到此位置
   ```

## 控制指令

运行 `finger_force.py` 后，在控制台输入以下指令：

| 指令 | 说明 | 示例 |
|------|------|------|
| `INIT` | 佩戴初始化 - 记录所有舵机当前位置作为基准 | `INIT` |
| `INIT:ID` | 初始化单个舵机 | `INIT:2` |
| `N:ID:力值` | 设定单个舵机目标力（牛顿） | `N:2:1.5` |
| `NALL:力值` | 设定所有舵机目标力 | `NALL:1.5` |
| `N:ID:0` | 关闭舵机力反馈，返回到初始位置 | `N:2:0` |
| `STATUS` | 查看所有舵机状态 | `STATUS` |
| `HELP` | 显示帮助信息 | `HELP` |
| `EXIT` | 退出程序 | `EXIT` |

## 典型工作流程

1. **启动所有程序**（按上述顺序）
2. **执行佩戴初始化**：
   ```
   > INIT
   === 佩戴初始化 ===
     舵机1: 初始位置记录为 0
     舵机2: 初始位置记录为 512
     ...
   初始化完成! 目标力=0时舵机将回到此位置
   ```
3. **设定目标力**：
   ```
   > N:2:1.5
   [舵机2] 目标力=1.5N
   ```
4. **触觉反馈开始工作** - 灵巧手接触到物体时，DDS 发送触觉数据 → 转换为力值 → TCP 发送给舵机 → PID 闭环控制舵机

## 故障排查

### DDS 没有接收到数据

**检查灵巧手驱动是否启动：**
```bash
ps aux | grep Headless_driver_r.py
```

**检查网络连接：**
```bash
ping 192.168.123.211
```

**检查 DDS 话题：**
```bash
cd /home/pi/dexEXO/Finger_force_test
python3 dds_diagnostic.py
```

### BLE 连接失败

**检查 BLE 设备是否可见：**
```bash
sudo hcitool scan
```

**修改 `finger_force.py` 中的 `DEVICE_ADDRESSES` 为实际的 MAC 地址**

### 网络配置丢失（重启后又变回 10.42.0.174）

**重新编辑 `/etc/rc.local`：**
```bash
sudo nano /etc/rc.local
```

确保包含这一行（在 `exit 0` 之前）：
```bash
ip addr add 192.168.123.100/24 dev eth0 2>/dev/null || true
```

## 配置说明

### PID 参数（可调）

在 `finger_force.py` 中修改：

```python
PID_KP = 80.0           # 比例系数 (mA/N) - 越大响应越快
PID_KI = 20.0           # 积分系数 (mA/(N·s)) - 消除稳态误差
PID_KD = 5.0            # 微分系数 (mA/(N/s)) - 阻尼防止震荡
PID_DEADZONE = 0.15     # 死区 (N) - 误差小于此值不调节
PID_OUTPUT_MAX = 400.0  # 输出电流上限 (mA)
```

### BLE 参数（可调）

```python
BLE_FORCE_BASELINE = 4.903  # STM32最低有效输出阈值；≤阈值为0，>阈值保留原值
BLE_KEEPALIVE_INTERVAL = 3.0   # 心跳间隔 (秒)
BLE_DATA_TIMEOUT = 15.0        # 数据超时 (秒)
```

### DDS 标定参数（来自 touch_calibration.json）

```python
CALIBRATION_K = 0.00292650244415058227
CALIBRATION_B = -0.6037947156125716
```

## Unitree G1 灵巧手触觉桥接（可选）

如果需要将 G1 上 Inspire FTP 灵巧手触觉数据作为目标力输入，可在 G1 的 PC2 上运行
`unitree_touch_bridge.py`，它会订阅 DDS 话题并通过 TCP 向树莓派发送 `N:2:NUM` 指令。

### 依赖

- `unitree_sdk2_python`
- `inspire_hand_sdk`

### 运行示例

```bash
python3 unitree_touch_bridge.py --pi-ip <树莓派IP> \
  --topic rt/inspire_hand/touch/r --finger index --region tip \
  --scale 1.0 --offset 0.0 --interval 0.1
```

> 提示：`--scale/--offset` 用于把触觉值映射为牛顿（N）。

## 说明

- 修改 `finger_force.py` 中的 `DEVICE_ADDRESSES` 为你的 STM32 BLE MAC 地址。
- STM32 发送 JSON 示例：
  ```json
  {"touch_sensors": [4.903]}
  ```
- 当 `touch_sensors` 只有一个值时，会自动广播到 5 个舵机；若有 5 个值则一一对应。
