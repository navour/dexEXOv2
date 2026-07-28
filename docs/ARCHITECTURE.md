# dexEXO 系统架构说明

## 1. 项目定位

dexEXO 不是一个单一进程，而是一套围绕以下设备逐步演进形成的实验与集成工程：

- 左右手 BLE 数据手套
- Dynamixel XL330 外骨骼力反馈舵机
- Inspire 左右灵巧手
- Unitree G1 人形机器人
- 无线 IMU 双臂遥操作设备
- ROS 2 + Ignition Gazebo 双手仿真
- Pico4U XR 显示链路
- STM32、Arduino 传感器与执行器固件

系统的最终目标是实现双臂、双手的双向遥操作：

1. 操作者的手臂动作驱动 G1 双臂。
2. 操作者的手指弯曲驱动 Inspire 左右灵巧手。
3. 机器人灵巧手接触物体后，将触觉反馈回操作者侧。
4. 操作者侧的 Dynamixel 舵机拉动外骨骼绳索，形成力反馈。
5. Gazebo 用于在不连接全部真实硬件时验证手部模型和控制映射。
6. Pico4U 用于显示机器人头部相机画面和提供 XR 交互入口。

目前手部控制最完整的实现是 `ftp/README_V4.md` 所描述的双手版本：

- 树莓派：`ble_broker.py`、`both_Force_handcontrol.py`
- G1：`both_hand_bridge.py`、`Headless_driver_r.py`、`Headless_driver_l.py`
- 右手 Dynamixel ID：1～5
- 左手 Dynamixel ID：6～10

---

## 2. 系统总体架构

```text
┌──────────────────────────── 操作者侧：树莓派 ────────────────────────────┐
│                                                                          │
│  右手 BLE 手套 ── BLE ──► ble_broker.py:9001 ─┐                         │
│                                                ├─► both_Force_handcontrol.py
│  左手 BLE 手套 ── BLE ──► ble_broker.py:9002 ─┘          │              │
│                                                           │              │
│                    ┌──────────────────────────────────────┼───────────┐  │
│                    │                                      │           │  │
│                    │  TCP 控制 9100/9102                  │           │  │
│                    │  TCP 触觉 9101/9103                  │           │  │
│                    │                                      ▼           │  │
│                    │                           Dynamixel 总线            │  │
│                    │                         /dev/ttyAMA0, 1 Mbps         │  │
│                    │                                      │           │  │
│                    │                         XL330 舵机 ID 1～10          │  │
│                    │                                      │           │  │
│                    │                               双手外骨骼力反馈       │  │
└────────────────────┼──────────────────────────────────────────────────┘  │
                     │                                                     │
                     ▼                                                     │
┌────────────────────────────── G1 机器人侧 ────────────────────────────────┐
│                                                                          │
│                         both_hand_bridge.py                              │
│                     TCP ↔ Unitree DDS 双向桥接                            │
│                         │                  ▲                              │
│          DDS ctrl/r,l   │                  │ DDS touch/r,l                │
│                         ▼                  │                              │
│            Headless_driver_r.py / Headless_driver_l.py                   │
│                         │                  ▲                              │
│          Modbus 控制    │                  │ Modbus 状态/触觉              │
│                         ▼                  │                              │
│                    Inspire 左右灵巧手                                     │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
```

这里存在四种主要通信方式：

| 通信方式 | 连接双方 | 主要数据 |
|---|---|---|
| BLE/GATT | 数据手套 ↔ 树莓派 | 弯曲传感器、外骨骼触觉数据 |
| TCP | BLE Broker、树莓派主控、G1 Bridge | BLE 原始流、灵巧手角度、触觉力 |
| DDS | G1 Bridge ↔ Inspire Headless Driver | 灵巧手控制、触觉、状态 |
| Modbus TCP | Inspire Driver ↔ Inspire 实体手 | 寄存器级角度、位置、力、状态和触觉 |
| Dynamixel TTL 串口 | 树莓派 ↔ XL330 舵机 | 目标电流、目标位置、实际位置和状态 |

---

## 3. 当前双手主链路

### 3.1 BLE 手套到树莓派

入口程序是：

```text
ftp/ble_broker.py
```

它使用 Python `bleak` 库连接 BLE 手套。Bleak 是跨平台的 BLE GATT 客户端库，可以：

- 根据 MAC 地址连接 BLE 设备
- 订阅 GATT Characteristic 通知
- 接收手套持续发送的传感器字节流
- 向手套写入心跳数据
- 检测断开并自动重连

项目使用的主要配置为：

```text
右手 MAC：F0:FD:45:02:85:B3
左手 MAC：F0:FD:45:02:67:3B

TX UUID：6e400002-b5a3-f393-e0a9-e50e24dcca9e
RX UUID：6e400003-b5a3-f393-e0a9-e50e24dcca9e
```

数据方向：

```text
手套
  │ BLE Notification
  ▼
BleakClient.start_notify(RX UUID)
  │ 原始 bytes
  ▼
ble_broker.py
  │ 本机 TCP 广播
  ├── 右手：127.0.0.1:9001
  └── 左手：127.0.0.1:9002
```

Broker 本身不解释具体传感器字段，只保持一条稳定的 BLE 连接并转发原始字节。这样可以避免多个进程同时争用同一个 BLE 设备。

当前双手模式下分别启动两个实例：

```bash
python3 ble_broker.py --port 9001 --devices F0:FD:45:02:85:B3
python3 ble_broker.py --port 9002 --devices F0:FD:45:02:67:3B
```

### 3.2 树莓派双手主控制器

核心程序：

```text
ftp/both_Force_handcontrol.py
```

它承担以下职责：

1. 从 TCP 9001/9002 接收左右手 BLE 数据。
2. 解析手套弯曲通道和外骨骼触觉通道。
3. 对数据进行滤波、标定和范围映射。
4. 计算 Inspire 灵巧手的六路目标角度。
5. 通过 TCP 9100/9102 将目标发送给 G1。
6. 通过 TCP 9101/9103接收 G1 返回的灵巧手触觉。
7. 控制 10 个 Dynamixel 舵机产生双手力反馈。
8. 对每根手指执行独立状态机和安全限制。

BLE 数据按当前代码约定包含 18 个通道：

```text
前 13 路：弯曲、拇指旋转等动作传感器
后 5 路：拇指、食指、中指、无名指、小指外骨骼触觉
```

灵巧手角度数组顺序为：

```text
[小指, 无名指, 中指, 食指, 拇指弯曲, 拇指旋转]
```

外骨骼舵机映射为：

| 手 | 拇指 | 食指 | 中指 | 无名指 | 小指 |
|---|---:|---:|---:|---:|---:|
| 右手 | 1 | 2 | 3 | 4 | 5 |
| 左手 | 6 | 7 | 8 | 9 | 10 |

每根手指有独立状态机：

```text
GLOVE
  手套弯曲角控制灵巧手，外骨骼不主动施力
    │
    │ 灵巧手触觉超过接触阈值
    ▼
FORCE_ENTRY
  暂停直接手套跟随，舵机逐步建立外骨骼接触力
    │
    │ 舵机位置和接触状态稳定
    ▼
LOCKED
  锁定外骨骼位置，并根据两侧触觉误差修正灵巧手位置
    │
    │ 灵巧手和外骨骼触觉持续归零
    ▼
RELEASE
  舵机返回佩戴初始化位置，释放绳索
    │
    └──────────────────────────────► GLOVE
```

这里同时存在两类触觉：

- 灵巧手触觉：机器人端接触物体时产生，由 G1 返回。
- 外骨骼触觉：操作者手套上的传感器测得实际反馈力，由 BLE 返回。

控制器结合两类触觉判断何时建立接触、何时锁定以及何时释放。

### 3.3 树莓派与 G1 之间的 TCP

双手模式端口如下：

| 端口 | 方向 | 数据 |
|---:|---|---|
| 9100 | 树莓派 → G1 | 右手角度控制 |
| 9101 | G1 → 树莓派 | 右手触觉反馈 |
| 9102 | 树莓派 → G1 | 左手角度控制 |
| 9103 | G1 → 树莓派 | 左手触觉反馈 |

TCP 的作用是隔离树莓派环境和 G1 DDS 环境。树莓派不需要直接加载 Unitree DDS 和 Inspire 硬件 SDK。

### 3.4 G1 TCP/DDS 桥接

桥接程序：

```text
ftp/both_hand_bridge.py
```

它为左右手分别创建 DDS Publisher 和 Subscriber：

```text
右手控制：rt/inspire_hand/ctrl/r
右手触觉：rt/inspire_hand/touch/r

左手控制：rt/inspire_hand/ctrl/l
左手触觉：rt/inspire_hand/touch/l
```

控制方向：

```text
both_Force_handcontrol.py
  │ TCP 角度 JSON
  ▼
both_hand_bridge.py
  │ 构造 inspire_hand_ctrl
  ▼
DDS ctrl/r 或 ctrl/l
```

反馈方向：

```text
DDS touch/r 或 touch/l
  │ Inspire 原始触觉阵列
  ▼
both_hand_bridge.py
  │ 取每根手指触觉阵列最大值
  │ raw → N 标定并限幅到 0～10 N
  ▼
TCP 9101 或 9103
  ▼
both_Force_handcontrol.py
```

### 3.5 Inspire 和 Unitree SDK

这两个 SDK 的职责不同。

#### Inspire SDK

目录：

```text
ftp/inspire_hand_ws/inspire_hand_sdk
```

作用是操作 Inspire 灵巧手硬件：

- 建立 Modbus TCP 或 Modbus RTU 连接
- 读写灵巧手寄存器
- 设置六路角度、位置、力和速度
- 读取实际位置、实际角度、电流、错误码、状态和温度
- 读取各手指触觉阵列

Inspire SDK 是“硬件驱动层”，它知道灵巧手具体使用哪些寄存器和数据格式。

#### Unitree SDK

目录：

```text
ftp/inspire_hand_ws/unitree_sdk2_python
```

在手部系统中主要提供 DDS 通信能力：

```python
ChannelFactoryInitialize
ChannelPublisher
ChannelSubscriber
```

它让 G1 上不同进程通过话题交换数据，而不需要互相直接调用。

因此两者的关系为：

```text
Unitree SDK：负责 G1 进程之间传递消息
Inspire SDK：负责把消息转换为真实灵巧手硬件操作
```

### 3.6 Headless Driver

右手驱动：

```text
ftp/inspire_hand_ws/inspire_hand_sdk/example/Headless_driver_r.py
```

左手驱动：

```text
ftp/inspire_hand_ws/inspire_hand_sdk/example/Headless_driver_l.py
```

`Headless` 表示无图形界面，适合在 G1 上通过 SSH 或后台运行。

以右手为例，`Headless_driver_r.py` 创建：

```python
ModbusDataHandler(
    ip="192.168.123.211",
    LR="r",
    device_id=1
)
```

初始化后会：

1. 通过 Modbus TCP 连接右侧 Inspire 实体手。
2. 订阅 `rt/inspire_hand/ctrl/r`。
3. 将 DDS 控制消息写入灵巧手寄存器。
4. 发布 `rt/inspire_hand/touch/r`。
5. 发布 `rt/inspire_hand/state/r`。

控制模式与寄存器关系：

| DDS 字段 | 用途 | Modbus 起始寄存器 |
|---|---|---:|
| `angle_set` | 角度控制 | 1486 |
| `pos_set` | 位置控制 | 1474 |
| `force_set` | 力控制 | 1498 |
| `speed_set` | 速度控制 | 1522 |

状态话题包含：

```text
pos_act       实际位置
angle_act     实际角度
force_act     实际力
current       电机电流
err           错误码
status        状态
temperature   温度
```

`Headless_driver_r.py` 主循环中的 `handler.read()` 不只是普通读取。它会读取实体手触觉和状态寄存器，并把结果发布到 DDS。

完整双向关系：

```text
ctrl/r DDS
  ▼
Headless_driver_r.py
  ▼
Inspire SDK / Modbus
  ▼
右侧 Inspire 实体手

右侧 Inspire 实体手
  ▼
Inspire SDK / Modbus
  ▼
Headless_driver_r.py
  ├── touch/r DDS
  └── state/r DDS
```

---

## 4. 推荐的双手运行进程

### G1 侧

```text
进程 1：Headless_driver_r.py
进程 2：Headless_driver_l.py
进程 3：both_hand_bridge.py
```

### 树莓派侧

```text
进程 1：右手 ble_broker.py，端口 9001
进程 2：左手 ble_broker.py，端口 9002
进程 3：both_Force_handcontrol.py
```

启动顺序建议：

```text
1. 检查 G1、树莓派、Inspire 左右手的网络连接
2. 启动右手 Headless Driver
3. 启动左手 Headless Driver
4. 启动 G1 双手 Bridge
5. 启动右手 BLE Broker
6. 启动左手 BLE Broker
7. 启动树莓派双手主控制器
8. 戴好手套后执行 INIT
```

---

## 5. 根目录各文件夹作用

### 5.1 `ftp/`

当前真实灵巧手遥操作的核心目录。

| 文件或子目录 | 作用 |
|---|---|
| `both_Force_handcontrol.py` | 当前双手遥操作与力反馈主程序 |
| `both_hand_bridge.py` | G1 双手 TCP↔DDS 桥接 |
| `ble_broker.py` | BLE 手套数据到本机 TCP 的代理 |
| `Force_handcontrol.py` | 单右手力反馈版本 |
| `hand_bridge.py` | 单右手 TCP↔DDS 桥接 |
| `ble_hand_control.py` | BLE 弯曲数据直接控制灵巧手的早期程序 |
| `hand_control.py` | 基础 DDS 灵巧手控制工具 |
| `touch_calibration.py` | 灵巧手触觉标定 |
| `dds_debug_all_fingers.py` | 五指 DDS 触觉调试 |
| `inspire_hand_ws/` | Inspire SDK、Unitree SDK 和 Headless Driver |
| `README_V1～V4.md` | 控制系统的版本演进和部署说明 |

该目录同时保存了历史版本。当前双手部署优先参考 `README_V4.md`。

### 5.2 `Finger_force_test/`

早期单手、单指到五舵机力反馈验证目录。

主要数据流：

```text
Inspire 触觉 DDS
  ▼
dds_to_force.py
  │ TCP 目标力
  ▼
finger_force.py ◄── BLE 外骨骼触觉
  ▼
Dynamixel 舵机
```

用于验证：

- DDS 触觉能否转成力目标
- BLE 力传感器能否作为反馈
- 舵机能否完成闭环力控制
- 目标力归零后能否返回佩戴初始位置

### 5.3 `Finger_force_test1.26/`

`Finger_force_test` 的历史快照或归档副本。根目录中还有对应 ZIP。

它不属于当前主运行链路，主要用于追溯旧版本。

### 5.4 `Five_finger_force_test/`

单手五指独立闭环实验，是双手主控制器的重要前身。

主要功能：

- 五根手指独立 PID
- DDS 五指触觉分别映射到舵机 1～5
- BLE 五路力反馈
- 单舵机和总电流限制
- 初始化、锁定、释放和状态查询

主要文件：

| 文件 | 作用 |
|---|---|
| `finger_force.py` | 五指控制器 |
| `finger_force_v2.py` | 另一版五指状态机实现 |
| `dds_to_force.py` | DDS 触觉到 TCP 力目标 |
| `ble_broker.py` | BLE 数据代理 |
| `change_baudrate.py` | 修改 Dynamixel 舵机波特率 |
| `PARAMETER_GUIDE.md` | PID、电流和安全参数说明 |

### 5.5 `sensors/`

早期双 BLE 传感器采集、记录与网络转发实验。

| 文件 | 作用 |
|---|---|
| `gatt_blu_251202.py` | 双 BLE 连接、解析、滤波、HDF5 保存和 Wi-Fi 发送 |
| `wifi_json_server.py` | JSON TCP 数据服务 |
| `sensor_receiver.py` | Ubuntu 端接收树莓派传感器数据 |

`sensor_receiver.py` 中向 Gazebo 发送的接口仍为预留实现，所以该目录主要完成采集、记录和传输。

### 5.6 `gazebos/`

ROS 2 Humble + Ignition Gazebo 双灵巧手仿真系统。

数据流：

```text
ROS 2 手指目标话题
  ▼
joint12_mapping_controller.py
  ├── 四指 joint1 → joint2 多项式映射
  ├── 拇指 joint2 → joint3/joint4 映射
  └── PD 速度控制
  ▼
ros_gz_bridge
  ▼
Ignition Gazebo JointController
  ▼
双手 URDF/SDF 模型
```

子目录：

| 子目录 | 作用 |
|---|---|
| `urdf/` | 左右手 URDF 机器人描述 |
| `models/` | Gazebo SDF 模型 |
| `worlds/` | 双手仿真世界 |
| `meshes/` | 灵巧手、G1 和传感器 STL 几何资源 |
| `config/` | ROS 2 ↔ Gazebo 话题桥接配置 |
| `scripts/` | 映射控制、拟合、传感器桥接和测试程序 |

核心程序为：

```text
gazebos/scripts/joint12_mapping_controller.py
```

`驱动器行程与角度关系表.xls` 为机械联动角度映射的数据来源。

### 5.7 `shiloh/`

主要包含 `gazebos/` 和 `sensors/` 的交付或打包副本，同时存在 `shiloh.zip`。

后续开发应优先修改根目录原始版本，避免两套副本继续分叉。

### 5.8 `宇树双臂遥操系统分享包/`

使用五个无线 IMU 控制 G1 双臂，和手指系统可以并行组合。

数据流：

```text
5 个无线 IMU
  │ UDP 4211，四元数
  ▼
PC 或树莓派
  ├── 设备角色识别
  ├── 姿态标定
  ├── 肩肘角度解算
  ├── IK 与关节限位
  └── UDP 9527，10 个双臂关节角
        ▼
G1 robot_arm_receiver.py
  ▼
500 Hz PD 电机控制
```

子目录：

| 子目录 | 作用 |
|---|---|
| `PC端/` | 带 pygame/PyOpenGL 3D 可视化的双臂控制 |
| `树莓派端/` | curses 无头双臂控制 |
| `机器人端/` | G1 关节目标接收和 PD 控制 |

这套系统主要控制肩、肘等手臂关节；`ftp/` 主链路控制左右灵巧手。

### 5.9 `VR/`

保存 G1 + Pico4U 的部署和启动说明。

当前记录的两个入口：

```text
60001：G1 头部 RealSense WebRTC 预览
8012：Pico4U XR 页面
```

实际 XR 程序位于 G1 上的外部 `xr_teleoperate` 环境，本目录目前不是完整源码模块。

### 5.10 `STM32G431CBT6-0119/`

STM32CubeMX/Keil 固件工程，属于传感器采集最底层。

主要功能：

- ADC + DMA 采集弯曲传感器
- SPI 采集触觉阵列
- MPU6050/DMP 姿态采集
- UART 命令和数据输出
- 打包弯曲、触觉、加速度、陀螺仪和姿态数据

子目录：

| 子目录 | 作用 |
|---|---|
| `Core/Inc/` | 应用和外设头文件 |
| `Core/Src/` | 主程序、ADC、DMA、SPI、定时器和串口代码 |
| `Core/MPU6050/` | MPU6050 与 DMP 驱动 |
| `Drivers/` | STM32 HAL 和 CMSIS |
| `MDK-ARM/` | Keil 工程和构建资源 |
| `RTT/` | 调试输出组件 |

### 5.11 `dxl_bridge/`

Arduino + Dynamixel 最小硬件测试程序。

`dxl_bridge.ino` 用于：

- Ping 指定舵机
- 验证位置模式
- 验证电流模式
- 输出实际位置和电流
- 排查供电、接线、ID、波特率和协议问题

### 5.12 `xl330_force_demo/`

单 XL330 舵机力控实验平台。

用于验证：

- 电流和位置模式切换
- 电流到力的标定
- 自由跟随
- 主动寻触
- 触摸增力
- 行程限制和安全释放

该目录是后续五指和双手力反馈算法的底层实验基础。

### 5.13 `tools/`

通用诊断脚本。

当前 `servo_current_monitor.py` 用于观察 Dynamixel 舵机电流和负载，辅助排查过流、红灯和供电问题。

### 5.14 `build/`

Arduino 编译生成目录，包括：

- ELF、HEX、BIN
- 目标文件
- 编译数据库
- Arduino 库缓存

它是可再生成的构建产物，不属于业务源码。

### 5.15 `source/`

硬件资料目录。当前包含 Inspire 触觉手用户手册 PDF。

### 5.16 `.venv/`

本地 Python 虚拟环境，保存 Python 解释器和依赖，不属于业务代码。

### 5.17 `.vscode/`

VS Code、Arduino 和 C/C++ 开发配置，不参与系统运行。

### 5.18 ZIP 文件

根目录中的 ZIP 是历史快照或交付包，不参与运行：

- `Finger_force_test1.26.zip`
- `shiloh.zip`
- `宇树双臂遥操系统分享包.zip`

---

## 6. 双臂、双手、VR 和仿真的组合关系

完整系统可以分为四层：

| 层级 | 主要模块 | 作用 |
|---|---|---|
| 设备层 | STM32、BLE 手套、无线 IMU、Inspire 手、XL330 | 采集动作、触觉并执行运动 |
| 边缘控制层 | BLE Broker、双手主控制器、IMU Sender | 解析、滤波、状态机、IK 和力控 |
| 机器人适配层 | DDS Bridge、Headless Driver、Arm Receiver | 将网络目标转换为 G1 和 Inspire 硬件操作 |
| 仿真显示层 | Gazebo、PC 3D、Pico4U | 仿真、可视化和机器人第一视角 |

理想组合为：

```text
人体双臂动作
  └── 无线 IMU → 双臂解算 → G1 双臂

人体双手动作
  └── BLE 弯曲传感器 → 双手控制器 → Inspire 左右手

机器人手部接触
  └── Inspire 触觉 → DDS/TCP → 双手控制器 → XL330 外骨骼

机器人头部相机
  └── WebRTC → Pico4U

离线验证
  └── 传感器/ROS 2 → Gazebo 双手模型
```

当前这些功能已经分别存在，但还没有统一的顶层进程负责同时启动、监控、记录和急停所有子系统。

---

## 7. 当前架构注意事项

### 7.1 配置存在硬编码

设备地址、BLE MAC、串口、舵机 ID 和网络端口分散在脚本中。例如：

- 文档中 G1 地址可能为 `192.168.3.85`
- `both_Force_handcontrol.py` 中可能配置为其他 Wi-Fi 地址
- Inspire 右手通常为 `192.168.123.211`
- 左右手 BLE MAC 固定写在 Broker 默认配置中

实际运行前应以当前设备网络为准逐项检查。

### 7.2 存在多代代码和副本

以下内容属于不同研发阶段：

```text
Finger_force_test
Five_finger_force_test
ftp/Force_handcontrol.py
ftp/both_Force_handcontrol.py
shiloh
各类 ZIP
```

它们不需要全部同时运行。真实双手部署应优先使用 `ftp` 中 V4 版本。

### 7.3 初始化是安全步骤

外骨骼戴好、手指自然伸展且绳索刚好拉直后，应执行：

```text
INIT
```

主控制器会记录舵机初始位置。释放状态依赖该位置让舵机回退并解除绳索张力。

### 7.4 当前缺少统一进程管理

双手系统需要至少六个进程；叠加双臂和 XR 后进程更多。当前主要依靠多个 SSH 终端人工启动。

后续适合增加：

- 统一 YAML 配置
- systemd 或 supervisor 服务
- 一键启动和停止脚本
- 连接健康检查
- 全局急停
- 统一日志和数据记录

---

## 8. 一句话理解主要组件

```text
Bleak                 把 BLE 手套数据送入树莓派
ble_broker.py         保持唯一 BLE 连接并向本机程序广播
both_Force_handcontrol.py
                      负责双手动作映射、状态机和外骨骼力反馈
both_hand_bridge.py   在 G1 上进行 TCP 与 DDS 双向转换
Unitree SDK           负责 G1 内部 DDS 消息传输
Inspire SDK           负责真实灵巧手的 Modbus 寄存器读写
Headless_driver_r/l   持续运行左右灵巧手硬件驱动
Gazebo                验证手部模型、关节映射和控制算法
无线 IMU 双臂系统     将人体肩肘动作转换为 G1 双臂关节目标
Pico4U                显示机器人第一视角并提供 XR 入口
STM32/Arduino         完成最底层传感器采集和舵机硬件验证
```
