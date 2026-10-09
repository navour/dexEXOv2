# Codex 项目交接说明

> **历史文档（已被取代）：**本文记录的是 2026-07-28 树莓派直连
> mHandPro/INSPIRE 的左手台架阶段，不是当前 G1 双手联调架构。
> 当前唯一操作入口请看 [`../newteleop/README.md`](../newteleop/README.md)，
> 力控设计看
> [`../newteleop/exoskeleton/HAPTIC_FEEDBACK_PLAN_CN.md`](../newteleop/exoskeleton/HAPTIC_FEEDBACK_PLAN_CN.md)。

更新时间：2026-07-28  
仓库根目录：`/home/cnn/桌面/dexexo/dexEXO`  
当前主开发包：`newteleop/`

> 本文记录本轮 mHandPro 替换原数据手套、Inspire 左手遥操、树莓派迁移、外骨骼舵机测试及薄膜压力反馈链路的实际进展。旧 `dexEXO` 代码仍保留作参考，但新功能应优先进入 `newteleop/`，不要继续扩展旧系统的耦合。

## 1. 项目目标和当前需求

### 1.1 总目标

将原工程中的旧 BLE 弯曲数据手套替换为 mHandPro，在树莓派上形成一套可独立部署的左手遥操与力反馈系统：

```text
mHandPro 姿态
  -> 六维手指命令
  -> Inspire 左手运动
  -> Inspire FORCE_ACT 检测机器人端接触
  -> 左手外骨骼 Dynamixel XL330 收绳
  -> 薄膜压力传感器测量操作者侧实际受力
  -> 外骨骼力闭环
```

最终需要支持左右手，但当前优先完成左手。树莓派是最终控制计算机，开发电脑只负责 SSH、复制代码、日志查看和必要的 x64 调试。

### 1.2 当前阶段需求

当前已经完成 mHandPro 到 Inspire 的左手运动映射和树莓派冒烟测试，正在接入外骨骼力反馈。最近正在处理的是：

1. 使用原 STM32 + BLE 板读取五片薄膜压力传感器。
2. 为左手五个压力通道采集穿戴状态下的独立零点。
3. 将 Inspire 的六路 `FORCE_ACT` 转换为外骨骼五指目标反馈力。
4. 建立左手五个 XL330 的安全、低强度、逐指闭环。
5. 最后再扩展到双手。

## 2. 目录结构和主要模块

### 2.1 新遥操包

```text
newteleop/
├── README.md                         # 新包定位、部署入口
├── requirements.txt                 # 当前含 pymodbus、dynamixel-sdk
├── computer_debug/
│   ├── README.md
│   └── mhandpro_x64_sdk/             # Ubuntu x64 官方SDK与过程文件
├── mhandpro/
│   ├── README.md
│   ├── build.sh
│   ├── mhandpro_diagnostic.cpp       # 手套读取、标定、六维映射、遥操主程序
│   ├── standalone_inspire_bridge.py # JSON/TCP -> Inspire Modbus安全桥
│   ├── sdk/                          # ARM64官方SDK头文件和.so
│   ├── config/
│   │   ├── left_hand.calib
│   │   ├── committed_left_safe_open.txt
│   │   ├── inspire_left.cfg
│   │   ├── inspire_left_pi_smoke_10.cfg
│   │   └── inspire_left_video_100.cfg
│   └── tools/
│       ├── inspire_modbus_probe.py
│       ├── inspire_force_monitor.py
│       ├── inspire_jog.py
│       ├── inspire_safe_test.py
│       └── inspire_combined_test.py
└── exoskeleton/
    ├── README.md
    ├── HAPTIC_FEEDBACK_PLAN_CN.md
    ├── config/
    │   ├── left_exoskeleton.json
    │   ├── right_exoskeleton.json
    │   └── left_neutral.json
    └── tools/
        ├── exo_dynamixel_probe.py
        ├── exo_capture_neutral.py
        ├── exo_single_servo_jog.py
        └── exo_single_servo_current_test.py
```

### 2.2 仍需参考的旧实现

- `Five_finger_force_test/ble_broker.py`：BLE 唯一连接者，将原始通知广播到本机 TCP。
- `Five_finger_force_test/finger_force.py`：五指 PID 电流闭环参考实现，包含 `extract_touch_sensors()` 和 `pid_control()`。
- `Five_finger_force_test/finger_force_v2.py`：另一版五指控制实现。
- `ftp/both_Force_handcontrol.py`：旧双手状态机与双向力反馈参考。
- `ftp/Force_handcontrol.py`：旧单手控制参考。
- `ftp/README_V4.md`、`docs/ARCHITECTURE.md`：旧系统/G1/DDS总体架构。
- `STM32G431CBT6-0119/`：仓库中的 STM32G431 工程。注意它与板卡当前实际烧录固件的数据格式不一致。

旧实现只能抽取已验证的状态机、安全限制和解析思路，不得整包迁入 `newteleop`。

## 3. 已完成的工作

### 3.1 mHandPro 接入和权限

- Ubuntu 22 开发电脑与树莓派均识别 mHandPro 接收器为 `/dev/ttyUSB0`。
- 用户已加入 `dialout`，验证 `/dev/ttyUSB0` 可读可写。
- x64 官方样例最初没有执行位，通过 `chmod +x ./bin/x64/main` 后运行成功。
- 官方 SDK 成功连接左手，持续输出帧数据，P-pose 标定完成。
- ARM64 官方样例在树莓派 `aarch64` 上运行成功；动态库软链接按 SDK 目录要求建立后可加载。

### 3.2 mHandPro 专用诊断程序

`newteleop/mhandpro/mhandpro_diagnostic.cpp` 已实现：

- 通过 `dlopen()` 动态加载官方 mHandPro SDK：
  - ARM64：`sdk/lib/arm64/libVDMocapSDK_mHandProArm64.so`
  - x86_64：`../computer_debug/mhandpro_x64_sdk/lib/x64/libVDMocapSDK_mHandPro.so`
- 注册 SDK 回调 `SetGloveDataCallBackFunc` 和断线回调。
- 读取 20 个节点的四元数和 `_SensorState_`。
- `relative(parent, child)` 实现：

  ```text
  q_relative = inverse(q_parent) * q_child
  ```

  含义是消除父骨段整体姿态，只保留子骨段相对父骨段的关节旋转。
- `compute_relatives()` 生成关节相对旋转。
- `compute_command()` 输出固定六维顺序：

  ```text
  [小指, 无名指, 中指, 食指, 拇指弯曲, 拇指对掌]
  ```

- 支持命令：`status`、`open`、`fist`、`calib ...`、`show`、`monitor`、`save`、`load`、`zero`、`teleop`、`quit`。
- 终端提示和主要注释已中文化。
- 标定格式使用魔数 `MHANDPRO_CALIBRATION_V2`，默认保存到 `config/left_hand.calib`。
- 四指弯曲/侧摆以及拇指弯曲/对掌使用二维方向解耦；当两标定方向太接近或正交有效幅度太小时回退为单轴，避免数值放大。
- 加入 EMA、死区、单周期变化限制和传感器状态看门狗。
- 对 `BAD_MAG` 采用短暂保持、持续异常才中止，而不是单帧立即停止：
  - `sensor_fault_warn_ms=150`
  - `sensor_fault_abort_ms=800`
- 加入 `ARM`/`STOP` 人工确认。
- 修复了“等待输入 ARM 时桥接器先超时关闭连接”的问题：现在 `run_teleop()` 在用户输入 `ARM` 后才建立 TCP，并首先发送安全张手位。
- 针对 OK 手势加入可选 `pinch_assist`，联合食指弯曲和拇指对掌，提高拇指弯曲/对掌目标。

### 3.3 手套标定和六维映射验证

已完成完整标定流程：

```text
open
calib index
calib middle
calib ring
calib pinky
calib spread
calib thumb-flex
calib thumb-opp
save
```

之后每次使用可执行：

```text
load
zero
show
```

不必每次重做全部动作标定；只有换人、改变手套佩戴、SDK姿态明显改变或标定文件失效时才需完整重标。每次佩戴后应重新执行 `zero`。

独立手指测试已确认六维输出总体正确。四指生理联动客观存在，二维解耦后串扰明显降低。拇指原始 OK、最大弯曲和最大对掌数据已经用于 `pinch_assist` 参数设计。

### 3.4 Inspire 左手直连与安全桥

已确认 Inspire 左手硬件：

- IP：`192.168.123.210`
- Modbus TCP：端口 `6000`
- Device/Unit ID：`1`
- 树莓派/电脑以太网静态地址：`192.168.123.100/24`
- `ping 192.168.123.210` 成功。
- `nc -vz -w 3 192.168.123.210 6000` 成功。
- `pymodbus==3.6.9` 已验证可用。

`newteleop/mhandpro/tools/inspire_modbus_probe.py` 已只读验证：

| 数据 | Modbus地址 | 数量 |
|---|---:|---:|
| `POS_ACT` | 1534 | 6 words |
| `ANGLE_ACT` | 1546 | 6 words |
| `FORCE_ACT` | 1582 | 6 words |
| `CURRENT` | 1594 | 6 words |
| `ERROR` | 1606 | 3 words/6 bytes |
| `STATUS` | 1612 | 3 words/6 bytes |
| `TEMPERATURE` | 1618 | 3 words/6 bytes |

`standalone_inspire_bridge.py` 的关键接口：

- JSON监听：`127.0.0.1:9102`
- 写角度寄存器：`ANGLE_SET_REGISTER=1486`
- 读实际角度：`ANGLE_ACT_REGISTER=1546`
- `Bridge.write_angles()` 强制每路在 `0..1000`。
- 相邻命令最大变化默认 `--max-step 50` tick。
- 客户端超时默认 `--watchdog-ms 500`，超时/断开调用 `return_safe()`。
- 必须同时传 `--enable-write` 和 `--safe-open` 才允许写，否则为只读模式。

已实测安全张手和闭合参考：

```text
通道：[小指, 无名指, 中指, 食指, 拇指弯曲, 拇指对掌]
open  = [980, 965, 957, 946, 949, 922]
closed= [ 60,  60,  70,  60,  70, 150]
```

这些 `closed` 是实测后保留余量的遥操端点，不是机械极限。

已通过：

- 10% 树莓派冒烟遥操。
- 30% 连续遥操。
- 空载 100% 映射视频录制。
- 100% 配置中的 OK/捏取协同可勉强实现拇食指接触。

Inspire 手册确认物理范围限制：四指约 `20°..176°`，拇指弯曲 `-13°..70°`，拇指旋转 `90°..165°`。因此人手拇指完全贴掌或精准逐指对齐不能仅靠映射解决，部分偏差是机构自由度和几何极限。

### 3.5 树莓派部署与网络

树莓派已确认：

```text
架构：aarch64
Wi-Fi IP：192.168.3.76
eth0：192.168.123.100/24
Inspire：192.168.123.210:6000
mHandPro：/dev/ttyUSB0
Dynamixel：/dev/serial0 -> /dev/ttyAMA0
```

- 当前电脑通过 `ssh pi@192.168.3.76` 登录。
- 用户决定暂时保留 `pi` 默认密码；这是已知安全风险。
- `newteleop` 被复制到 `/home/pi/cnn/newteleop`。
- mHandPro ARM64 SDK、诊断程序、Inspire只读探测和遥操均已在树莓派运行成功。

### 3.6 外骨骼 Dynamixel 验证

硬件：Dynamixel XL330-M288-T，Protocol 2.0，1 Mbps。

```text
右手：ID 1拇指，2食指，3中指，4无名指，5小指
左手：ID 6拇指，7食指，8中指，9无名指，10小指
```

接线约定：转接板 `G/T/R/V` 中使用树莓派 UART 三线时，只连接共地和交叉串口信号；舵机电源独立供电，不能从树莓派GPIO给舵机供电。具体板卡线序必须以丝印/原线束为准。

已完成：

- `/dev/serial0 -> ttyAMA0` 且 `enable_uart=1`。
- 10个舵机可在树莓派上被读取。
- 左手 ID 6～10 已只读 Ping 成功。
- 位置模式小点动工具验证可临时切到模式3、运动、返回原位并恢复原模式。
- 已观察到 `+80 tick` 为收绳方向；低电流测试也观察到正电流为收绳方向。该结论仍应在每个左手ID上逐一写入配置确认，不能从一个舵机外推所有舵机。
- 左手放松位置已保存到 `newteleop/exoskeleton/config/left_neutral.json`：

  ```text
  ID6=3396, ID7=6787, ID8=3569, ID9=2893, ID10=5243
  ```

- 采集时 ID7 为模式5，ID6/8/9/10 为模式0。模式不一致是后续联调前必须处理的问题。

### 3.7 BLE 薄膜压力链路

通过断电消失、重新上电出现的方法实物确认：

```text
左手 BLE：F0:FD:45:02:67:3B，名称 RFstar_673B
右手 BLE：F0:FD:45:02:85:B3，名称 RFstar_85B3
Nordic UART Service：6e400001-b5a3-f393-e0a9-e50e24dcca9e
TX characteristic：6e400002-b5a3-f393-e0a9-e50e24dcca9e
RX characteristic：6e400003-b5a3-f393-e0a9-e50e24dcca9e
```

`Five_finger_force_test/ble_broker.py` 使用 `BleakClient.start_notify()` 订阅 RX，并每3秒向 TX 写 `PING`；原始字节广播到：

```text
右手 127.0.0.1:9001
左手 127.0.0.1:9002
```

树莓派虚拟环境最初没有 `bleak`，后已手工安装。BLE连接左手成功；`nc 127.0.0.1 9002` 能看到连续数据。

板卡实际发送旧版18项紧凑帧：

```text
{前13项原手套弯曲ADC, 拇指压力, 食指压力, 中指压力, 无名指压力, 小指压力}
```

五片传感器已逐一测试，确认最后五项严格按：

```text
[拇指, 食指, 中指, 无名指, 小指]
```

排列。无压力值约 `4.90`，旧代码基线为 `BLE_FORCE_BASELINE=4.903`。实际增量力初步按：

```text
F_feedback = max(0, raw - baseline_per_sensor)
```

计算，但每片传感器仍需在安装/穿戴状态下独立标定。

## 4. 已修改或新增的文件及原因

以下是本轮主线相关文件；旧仓库还有大量历史代码，不在此逐一列出。

### 4.1 `newteleop/`

- `newteleop/README.md`
  - 建立最小独立部署包边界。
  - 说明树莓派目录、编译和运行入口。
- `newteleop/requirements.txt`
  - 固定 `pymodbus==3.6.9` 并加入 `dynamixel-sdk`。
  - **待修改：还应加入 `bleak`。**
- `newteleop/mhandpro/mhandpro_diagnostic.cpp`
  - 新建专用mHandPro诊断、标定、六维映射和遥操程序。
  - 增加中文输出、保存/加载/回零、滤波、看门狗、ARM/STOP和捏取协同。
- `newteleop/mhandpro/build.sh`
  - 提供 C++17、`-pthread`、`-ldl` 的统一编译命令。
- `newteleop/mhandpro/standalone_inspire_bridge.py`
  - 新建本机JSON/TCP到Inspire Modbus的唯一安全桥。
  - 默认只读，显式授权后才写；加入跳变限制、500 ms看门狗和安全张手。
- `newteleop/mhandpro/config/left_hand.calib`
  - 保存已完成的左手动作标定模型。
- `newteleop/mhandpro/config/committed_left_safe_open.txt`
  - 保存实物确认的安全张手基准。
- `newteleop/mhandpro/config/inspire_left_pi_smoke_10.cfg`
  - 树莓派首次冒烟用10%行程，关闭捏取协同。
- `newteleop/mhandpro/config/inspire_left.cfg`
  - 日常低风险配置，30%行程及传感器看门狗。
- `newteleop/mhandpro/config/inspire_left_video_100.cfg`
  - 空载视频专用100%行程，加入OK/捏取协同；明确禁止直接用于外骨骼联调。
- `newteleop/mhandpro/tools/inspire_modbus_probe.py`
  - 只读检查Inspire关键寄存器。
- `newteleop/mhandpro/tools/inspire_jog.py`
  - 六通道逐通道安全点动，建立方向和安全张手基准。
- `newteleop/mhandpro/tools/inspire_safe_test.py`
  - 显式 `--send` 才真正下发的单通道测试工具。
- `newteleop/mhandpro/tools/inspire_combined_test.py`
  - 组合开合测试，硬限制比例不超过30%。
- `newteleop/mhandpro/tools/inspire_force_monitor.py`
  - 读取 `FORCE_ACT=1582`，用于空载和接触阈值标定。
- `newteleop/exoskeleton/tools/exo_dynamixel_probe.py`
  - 只读Ping、位置、电流、电压、温度和硬件错误。
- `newteleop/exoskeleton/tools/exo_single_servo_jog.py`
  - 单舵机低PWM位置点动；支持临时模式3并恢复；限制 `abs(delta)<=100`、`pwm<=100`。
- `newteleop/exoskeleton/tools/exo_single_servo_current_test.py`
  - 模式0低电流短脉冲；限制 `abs(current)<=30`、`duration<=0.50s`、位移看门狗40 tick。
- `newteleop/exoskeleton/tools/exo_capture_neutral.py`
  - 全程只读采集五个舵机放松位置；扭矩未关或硬件错误时拒绝采集。
- `newteleop/exoskeleton/config/left_neutral.json`
  - 保存左手实际放松位置和采集波动。
- `newteleop/exoskeleton/config/left_exoskeleton.json`、`right_exoskeleton.json`
  - 保存手指-ID和方向验证状态；当前部分标志尚未与最新实测同步。
- `newteleop/exoskeleton/HAPTIC_FEEDBACK_PLAN_CN.md`
  - 记录开环力提示、真正闭环、进程边界、状态机和阶段验收方案。
- `newteleop/computer_debug/`
  - 保存 x64 SDK 和电脑调试过程，不部署到最终树莓派最小包。

### 4.2 旧实现中本轮参考/整理的文件

- `Five_finger_force_test/ble_broker.py`
  - BLE连接与TCP广播；已经复制到树莓派测试，但尚未正式迁入本地 `newteleop`。
- `Five_finger_force_test/finger_force.py`
  - 支持最后五项压力解析和五指PID；输出上限曾降到250以避免五指同时过流，仅作参考。
- `Five_finger_force_test/finger_force_v2.py`
  - 另一版力控实现，保留作比较。
- `docs/ARCHITECTURE.md`
  - 整理旧系统全局架构。

## 5. 已确认的技术方案和重要决策

### 5.1 开发顺序

采用分阶段而不是一次集成：

```text
mHandPro姿态正确
 -> 六维映射正确
 -> Inspire单独遥操正确
 -> 树莓派迁移
 -> 外骨骼单舵机/单指
 -> 五指力反馈
 -> 双手
```

这一路线已证明有效，应继续保持。

### 5.2 最终主机和接线

- mHandPro USB接收器接树莓派。
- Inspire网线接树莓派 `eth0`。
- 10个XL330通过转接板接树莓派 `/dev/serial0`。
- STM32/BLE薄膜压力板通过BLE连接树莓派。
- 电脑只通过Wi-Fi SSH连接树莓派。

### 5.3 进程唯一所有权

- `standalone_inspire_bridge.py` 应是 Modbus TCP 的唯一拥有者。
- 后续 `left_haptic_controller` 应是 `/dev/serial0` 的唯一拥有者。
- 左右BLE各自只运行一个 `ble_broker.py`，其他进程通过9001/9002消费数据。
- 运行主力控时不得同时运行点动、电流测试或第二个串口程序。

### 5.4 mHandPro映射

- 姿态采用父子骨段相对四元数，不直接使用世界四元数。
- 六维顺序固定为 Inspire 的顺序：`[小指,无名指,中指,食指,拇指弯曲,拇指对掌]`。
- 每次佩戴执行 `load -> zero -> show`。
- `BAD_MAG` 是磁力计异常，不应单帧触发停机；持续800 ms才安全张手退出。
- 100%只是到实测安全端点，不是机械极限；加入外骨骼后重新从10%或更低开始。

### 5.5 外骨骼反馈方案

最终采用原 STM32 + 五片薄膜传感器测量操作者侧实际压力，形成闭环，而不是仅根据XL330电流开环估计手指力。

建议闭环：

```text
Inspire FORCE_ACT
 -> 基线扣除、滞回、滤波、舒适缩放
 -> F_target
 -> error = F_target - FSR_measured
 -> P/PI（初期不使用D）
 -> 模式5 Goal Current + 有界 Goal Position
```

初期首选模式5（Current-based Position Control），因为模式0纯电流没有位置终点，空载时也可能持续卷绳。必须同时具备最大电流、最大行程、变化率和超时释放。

### 5.6 左右板卡能否混用

- 相同型号薄膜传感器和STM32板在电气上通常可交换。
- BLE MAC决定软件身份；交换板卡必须更新左右MAC。
- 每片薄膜传感器零点/增益不同，标定文件不能混用。
- 线束通道和插头定义未确认前不得凭颜色交叉插接。
- 最稳妥做法是固定成套：`L-STM32 + L-BLE + L-5 sensors`，右手同理。

## 6. 尝试过但失败的方法及原因

### 6.1 串口和Shell问题

- `ls -l /dev/ttyACM*`、`/dev/ttyXRUSB*` 无结果：实际设备是CH341/Exar枚举出的 `/dev/ttyUSB0`。
- 首次运行SDK报 `Permission denied`：可执行文件没有执行位，使用 `chmod +x` 修复。
- SDK连接时报串口 `Permission denied`：用户当时的shell尚未获得 `dialout`；重新登录或 `newgrp dialout` 后修复。
- 每次新shell出现：

  ```text
  bash: install/local_setup.sh: 没有那个文件或目录
  ```

  原因是 `.bashrc` 中残留了不存在的ROS工作区source语句，与mHandPro无关。尚未清理。
- 普通 `fuser /dev/ttyUSB0` 输出大量 `/proc/... 权限不够`：这是检查其他用户/受限进程文件描述符时的权限问题；`sudo fuser -v /dev/ttyUSB0` 才是有效检查，结果未发现占用。

### 6.2 SDK/遥操失败

- 出现过 `*** stack smashing detected ***`：发生在SDK程序重启阶段，未形成稳定复现；可能与官方二进制SDK、异常退出或运行环境有关。当前程序恢复正常，但仍是未消除风险。
- 遥操出现 `TCP发送失败`：早期程序在等待用户输入 `ARM` 前已经连接桥接器，桥接器500 ms看门狗关闭了空闲连接；输入ARM后向旧socket发送导致失败。已修改为ARM后连接并先发安全张手。
- `BAD_MAG` 导致看门狗频繁STOP：mHandPro磁力计对电脑、舵机、磁性工具和大电流导线敏感。将手套远离干扰后明显改善，并改成短暂保持、持续800 ms才中止。
- 100%时OK手势不理想：不是简单行程上限问题；mHandPro的拇指弯曲/对掌解耦、Inspire拇指两自由度及其机械范围共同造成错位。加入 `pinch_assist` 后改善，但无法突破机构几何极限。

### 6.3 Inspire和网络问题

- 未给灵巧手供电时NetworkManager中看不到有效以太网链路：物理链路未建立；供电后 `enp0s31f6/eth0` 才变为UP。
- `pip install` 曾超时：Wi-Fi下载慢，重试后成功；不是包版本问题。
- 树莓派最初没有 `nc`：安装 `netcat-openbsd` 后 `nc -vz` 可用。
- 在树莓派终端误执行电脑端 `scp ~/桌面/...`：树莓派没有该中文桌面源路径。`scp`应从文件实际所在的开发电脑执行。

### 6.4 外骨骼测试问题

- 第一次舵机Ping 0/5：舵机电源接头松动；重新接牢后5/5成功。
- `exo_single_servo_jog.py` 报“工作模式0，不是位置模式3”：旧力控把舵机留在电流模式。增加 `--temporary-position-mode` 后，在扭矩关闭且无硬件错误时临时切模式3并恢复。
- `delta=20`运动过小难以观察：机械摩擦和绳间隙较大；未穿戴条件下改用 `delta=80,pwm=100` 后可观察且能回原位。
- 测试曾因位移超过40 tick被电流脉冲看门狗中止：这是安全逻辑正常动作，不应删除；需要降低脉冲或在确认机械安全后重新设计分级阈值。

### 6.5 BLE/压力链路误判

- 初看扫描结果设备过多：改用已知MAC并通过断电消失/上电出现确认身份。
- `bleak` 未安装：虚拟环境中手工 `python -m pip install bleak` 后解决。
- Broker显示“正在转发”但终端没数据：Broker本身只转发不打印，需要另一个客户端执行 `nc 127.0.0.1 9002`。
- 根据仓库 `STM32G431CBT6-0119/Core/Src/main.c` 曾判断固件只输出 `BLE_arrayBend[17]` 一路压力；实物板实际发送18项紧凑格式且最后五路独立。结论是仓库源码与当前烧录固件不是同一版本，必须以实测协议为准。

## 7. 当前问题和待办事项

按优先级排序：

1. **薄膜压力零点未保存。** 左手五片传感器通道已确认，但尚未在实际安装和穿戴状态下采集每指均值、噪声、漂移和加载/卸载曲线。
2. **BLE工具未正式迁入 `newteleop`。** 树莓派上已有临时复制的 `ble_broker.py`，本地 `newteleop/exoskeleton/tools/` 尚无对应文件。
3. **`newteleop/requirements.txt` 缺少 `bleak`。** 当前树莓派依赖手工安装状态。
4. **没有新的五指压力只读诊断/标定工具。** 需要正确缓存 `{...}` 完整帧、解析最后五项，并保存左右手独立基线JSON。
5. **Inspire FORCE_ACT尚未完整标定。** 需要用 `inspire_force_monitor.py` 分别记录六路空载、轻触、中等接触和释放。
6. **`standalone_inspire_bridge.py`尚未发布力数据。** 设计中应在 `127.0.0.1:9202` 以20～30 Hz发布含序号、单调时间戳和六路`FORCE_ACT`的JSON。
7. **`left_haptic_controller`尚未实现。** 需要成为 `/dev/serial0` 的唯一拥有者，读取9202与9002，运行状态机并控制ID6～10。
8. **左手舵机模式不统一。** `left_neutral.json` 显示ID7模式5，其余模式0；闭环前需要明确目标模式并安全配置。
9. **方向配置未同步。** `left_exoskeleton.json` 中所有 `*_verified` 仍为false，与部分实测结论不一致；必须逐ID复核后更新，不能猜测。
10. **外骨骼闭环尚未佩戴测试。** 必须从左食指ID7、低目标力、低行程开始。
11. **缺少压力/位置/电流/温度统一日志。** 后续调参必须可回放，不应只看终端瞬时输出。
12. **缺少硬件过力保护。** Python看门狗不能替代物理断电、机械限位或独立过力保护。
13. **右手压力基准和外骨骼零位未采集。** 左手完成后再做。
14. **树莓派使用默认密码。** 用户明确暂不修改，但部署到非隔离网络前必须处理。
15. **`.bashrc`残留无效ROS source。** 不影响系统，但持续制造误导性错误。

## 8. 下一步建议

### 阶段1：左手压力只读标定

1. 把五片薄膜传感器固定到外骨骼对人体手指实际施力的位置，避免剪切和尖点载荷。
2. 迁入并改造BLE只读工具：完整解析18项帧，取最后五项 `[thumb,index,middle,ring,pinky]`。
3. 戴好外骨骼但保持舵机扭矩关闭，采集10秒基线。
4. 为每指保存 `mean/min/max/stddev`，不要共用单个4.903常量。
5. 逐指轻压，确认只有对应通道显著变化，松开能回零。
6. 使用已知砝码或力计做至少0、轻、中三个点的加载/卸载标定。

建议配置文件：`newteleop/exoskeleton/config/left_pressure_calibration.json`。

### 阶段2：Inspire接触力标定

1. 外骨骼保持断电。
2. 运行 `mhandpro/tools/inspire_force_monitor.py`。
3. 记录每通道空载噪声、接触开启阈值、释放阈值和可用满量程。
4. 拇指暂用 `max(channel4, channel5)` 合并，后续再按实验加权。

### 阶段3：桥接器输出力流

扩展 `standalone_inspire_bridge.py`：

- 以20～30 Hz读取地址1582。
- 本机发布到 `127.0.0.1:9202`。
- JSON至少包含：

  ```json
  {"seq":1,"mono_ms":123456,"force_act":[0,0,0,0,0,0],"valid":true}
  ```
- Modbus读取失败时发布 `valid=false`，力控端必须进入RELEASE/FAULT，不能沿用旧值。

### 阶段4：左食指闭环

1. 只启用ID7，其余扭矩关闭。
2. 重新采集当次佩戴 `neutral_position`。
3. 使用模式5，设置很小 `max_travel` 和 Goal Current。
4. 先人工注入目标强度验证 FREE/CONTACT/HOLD/RELEASE/FAULT。
5. 再接 Inspire 食指 `FORCE_ACT[3]` 和左压力 `touch[1]`。
6. 从P控制开始；稳定后才加I，初期不要D。
7. 依次验证停止程序、拔网线、停止BLE、mHandPro断线和物理急停。

### 阶段5：逐指和双手

按ID8、9、10、6扩展；每增加一指都重做方向、限位和断线验证。左手五指稳定后再采集右手零位和压力标定。

## 9. 编译、运行和测试命令

### 9.1 开发电脑：串口权限

```bash
ls -l /dev/ttyUSB0
id
test -r /dev/ttyUSB0 && echo 可读
test -w /dev/ttyUSB0 && echo 可写
sudo usermod -aG dialout "$USER"
newgrp dialout
sudo fuser -v /dev/ttyUSB0
```

### 9.2 树莓派登录与网络

```bash
ssh pi@192.168.3.76
ip -br address
sudo ip link set eth0 up
ping -c 4 192.168.123.210
nc -vz -w 3 192.168.123.210 6000
```

开发电脑直连Inspire时的NetworkManager配置：

```bash
sudo nmcli connection add \
  type ethernet \
  ifname enp0s31f6 \
  con-name inspire-left \
  ipv4.method manual \
  ipv4.addresses 192.168.123.100/24 \
  ipv4.never-default yes \
  ipv6.method disabled
sudo nmcli connection up inspire-left
```

### 9.3 部署和虚拟环境

开发电脑：

```bash
scp -r ~/桌面/dexexo/dexEXO/newteleop pi@192.168.3.76:/home/pi/cnn/
```

树莓派：

```bash
cd ~/cnn/newteleop
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install bleak   # 在requirements补齐前仍需手工执行
```

历史树莓派环境也曾使用：

```bash
source ~/cnn/mHandpro/diagnostic/.venv-inspire/bin/activate
```

新代码应逐步统一到 `~/cnn/newteleop/.venv`。

### 9.4 编译和运行mHandPro

```bash
cd ~/cnn/newteleop/mhandpro
chmod +x build.sh
./build.sh
./bin/mhandpro_diagnostic
```

交互命令：

```text
status
load
zero
show
monitor
teleop config/inspire_left_pi_smoke_10.cfg
teleop config/inspire_left.cfg
teleop config/inspire_left_video_100.cfg
quit
```

### 9.5 Inspire只读检查

```bash
cd ~/cnn/newteleop
source .venv/bin/activate
python mhandpro/tools/inspire_modbus_probe.py --ip 192.168.123.210
python mhandpro/tools/inspire_force_monitor.py \
  --ip 192.168.123.210 --baseline-seconds 2 --seconds 30 --hz 10
```

### 9.6 Inspire遥操

终端1：

```bash
cd ~/cnn/newteleop/mhandpro
../.venv/bin/python standalone_inspire_bridge.py \
  --enable-write \
  --safe-open 980,965,957,946,949,922 \
  --max-step 50 \
  --watchdog-ms 500
```

终端2：

```bash
cd ~/cnn/newteleop/mhandpro
./bin/mhandpro_diagnostic
```

然后输入：

```text
load
zero
show
teleop config/inspire_left_pi_smoke_10.cfg
ARM
```

停止时输入 `STOP`，不要直接关闭终端作为正常停止方式。

### 9.7 外骨骼只读、零位和点动

```bash
cd ~/cnn/newteleop
source .venv/bin/activate

python exoskeleton/tools/exo_dynamixel_probe.py --hand left

python exoskeleton/tools/exo_capture_neutral.py \
  --hand left --seconds 2

python exoskeleton/tools/exo_single_servo_jog.py \
  --hand left --id 7 --delta 20 --pwm 50 \
  --temporary-position-mode
```

只有未穿戴、拉绳松弛时才可提高到已验证的观察档：

```bash
python exoskeleton/tools/exo_single_servo_jog.py \
  --hand left --id 7 --delta 80 --pwm 100 \
  --temporary-position-mode
```

模式0低电流短脉冲只用于未穿戴方向确认：

```bash
python exoskeleton/tools/exo_single_servo_current_test.py \
  --hand left --id 7 --current 15 --duration 0.20
```

### 9.8 BLE压力只读

当前Broker位于旧目录；若已复制到树莓派 `exoskeleton/tools/`：

```bash
cd ~/cnn/newteleop
source .venv/bin/activate

python exoskeleton/tools/ble_broker.py \
  --devices F0:FD:45:02:67:3B \
  --port 9002
```

另一个终端：

```bash
nc 127.0.0.1 9002
```

右手对应：

```bash
python exoskeleton/tools/ble_broker.py \
  --devices F0:FD:45:02:85:B3 \
  --port 9001
```

BLE身份检查：

```bash
bluetoothctl
```

交互输入：

```text
power on
scan on
info F0:FD:45:02:67:3B
scan off
quit
```

不要在 `bluetoothctl` 中长期手动连接设备；`BleakClient`需要自行占用连接。

### 9.9 关机

先停止所有控制程序并确认扭矩清零，再执行：

```bash
sudo poweroff
```

等待树莓派完全关机后再切断电源。

## 10. 特别注意的约束

1. **默认安全。** 所有上电默认必须是STOP/只读/扭矩关闭；运动需要明确ARM或确认词。
2. **单一设备所有者。** 同一时刻只能有一个进程占用 `/dev/ttyUSB0`、`/dev/serial0` 或 Inspire Modbus连接。
3. **禁止带电改线。** 薄膜传感器、Dynamixel线束、UART转接板必须断电后插拔。
4. **舵机独立供电且共地。** 不得用树莓派GPIO为10个舵机供电；UART信号电平和GND必须正确。
5. **先未穿戴，后佩戴。** 方向、模式、限位、回零和异常清理未验证前不得穿戴测试。
6. **不把100%视频配置用于力反馈。** `inspire_left_video_100.cfg` 只用于已完成的空载视频；外骨骼联调重新从10%或更低开始。
7. **不删除看门狗以“通过测试”。** TCP、BLE、mHandPro、位置、位移、电流、温度看门狗是安全边界；触发后应查原因。
8. **磁干扰是实际风险。** 手套远离舵机、扬声器、磁性工具、大电流电源和钢制桌架；`BAD_MAG`持续异常必须释放。
9. **压力基线必须按片、按次采集。** 不能把所有传感器都硬编码成4.903，也不能左右手共用标定。
10. **板上固件版本不等于仓库源码。** 当前实物协议为18项紧凑帧；修改STM32前必须先读出/备份实际固件或取得对应源版本。
11. **Inspire机械极限不可软件突破。** 拇指贴掌和精确OK对齐存在机构限制，不应靠过度扩大命令补偿。
12. **保留物理急停。** 软件STOP、Ctrl+C和异常清理不能代替随手可触达的舵机断电。
13. **保持`newteleop`最小化。** G1、ROS、Gazebo、旧BLE弯曲控制和历史虚拟环境不要无选择迁入。
14. **保留用户现有改动。** 仓库可能包含实验配置和实物标定数据；修改前检查 `git status`，不要重置或覆盖。
15. **当前实测结论优先于过时注释，但必须回写配置。** 特别是左右MAC、舵机方向、模式和压力通道映射。
