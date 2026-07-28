# 宇树 G1 双臂遥操系统

使用 5 个无线 IMU 传感器实时驱动 Unitree G1 人形机器人双臂运动。

---

## 系统架构

```
┌──────────────────┐     UDP 4211      ┌──────────────────┐     UDP 9527      ┌──────────────────┐
│   5 个无线 IMU   │ ───────────────>  │  PC / 树莓派     │ ───────────────>  │  宇树 G1 机器人   │
│  (ESP32 + MPU)   │   四元数数据       │  (解算 + 发送)    │   关节角度指令     │  (PD 电机控制)    │
└──────────────────┘                   └──────────────────┘                   └──────────────────┘
  胸部 / 右上臂 / 右前臂
  左上臂 / 左前臂

通信协议:
  IMU → PC/树莓派:    UDP 4211 (四元数), UDP 4212 (发现), TCP 4210 (配置)
  PC/树莓派 → 机器人:  UDP 9527 (指令), UDP 9528 (机器人广播发现)
  指令包格式:          'UARM' + mode(1B) + 10个float (双臂各5个关节角度)
  频率:                50Hz 发送, 500Hz 机器人控制环
```

## 文件夹结构

```
宇树双臂遥操系统分享包/
├── README.md                    ← 你正在看的文档
├── PC端/                        ← 带 3D 可视化的 PC 端程序
│   ├── dual_arm_viz.py          主程序 (PyOpenGL + pygame 3D 可视化)
│   ├── arm_calibration.py       IMU 校准模块 (4步校准流程)
│   ├── arm_solver.py            逆运动学求解器 (解析 IK + 优化 IK)
│   ├── robot_config.py          机器人配置 (关节限位, 电机映射, 连杆参数)
│   ├── IMU API/                 无线 IMU 通信库
│   │   └── multi_imu_core.py    IMU 发现/连接/数据接收
│   ├── wireless_imu_roles.json  IMU 角色分配配置 (需根据实际设备修改)
│   └── requirements.txt         Python 依赖
├── 机器人端/                     ← 运行在宇树 G1 上的程序
│   └── robot_arm_receiver.py    关节角度接收 + PD 电机控制 (需 unitree_sdk2py)
└── 树莓派端/                     ← 无头模式, 运行在树莓派上
    ├── headless_arm_sender.py   主程序 (curses 终端 UI, 无需显示器)
    ├── arm_calibration.py       IMU 校准模块
    ├── arm_solver.py            逆运动学求解器
    ├── robot_config.py          机器人配置
    ├── IMU_API/                 无线 IMU 通信库
    │   └── multi_imu_core.py    IMU 发现/连接/数据接收
    ├── wireless_imu_roles.json  IMU 角色分配配置
    └── requirements.txt         Python 依赖
```

---

## 运行模式

系统支持两种运行模式:

| 模式 | PC 端 | 树莓派端 | 适用场景 |
|------|-------|---------|---------|
| **PC 可视化模式** | dual_arm_viz.py | 不需要 | 调试、演示、有显示器时 |
| **树莓派无头模式** | 不需要 | headless_arm_sender.py | 现场遥操、无显示器 |

两种模式都需要机器人端运行 `robot_arm_receiver.py`。

---

## 环境要求

### 硬件

- **宇树 G1 人形机器人** (固件已配置为低层级控制模式)
- **5 个无线 IMU 传感器** (ESP32 + MPU6050/9250, 固件支持 UDP 四元数输出)
- **PC** (Linux/Windows, 推荐有独立显卡) 或 **树莓派 4/5**
- **路由器** (组建局域网, 所有设备在同一网段)

### 网络

- 所有设备在同一局域网
- IMU 传感器: 通过 WiFi 连接到路由器
- 机器人: 有线或 WiFi 连接, 默认 IP `192.168.3.85` (根据实际情况修改)
- PC / 树莓派: 有线或 WiFi 连接

---

## 安装步骤

### 一、PC 端安装

```bash
# 1. 将 "PC端" 文件夹复制到你的工作目录

# 2. 创建并激活 Python 虚拟环境
cd PC端
python3 -m venv .venv
source .venv/bin/activate   # Linux/macOS
# .venv\Scripts\activate    # Windows

# 3. 安装依赖
pip install -r requirements.txt

# 4. (可选) 如果需要 PyBullet 仿真 (不需要真机也能测试)
pip install pybullet
```

### 二、树莓派端安装

```bash
# 1. 将 "树莓派端" 文件夹复制到树莓派, 例如 /home/pi/teleop/
scp -r 树莓派端/ pi@<树莓派IP>:/home/pi/teleop/

# 2. SSH 登录树莓派
ssh pi@<树莓派IP>
cd /home/pi/teleop

# 3. 创建并激活虚拟环境
python3 -m venv .venv
source .venv/bin/activate

# 4. 安装依赖
pip install -r requirements.txt
```

### 三、机器人端安装

> **注意**: 机器人端程序依赖 `unitree_sdk2py` (宇树官方 SDK), 需要在机器人上安装。

```bash
# 1. 将 "机器人端" 文件夹复制到机器人的工作目录
scp robot_arm_receiver.py unitree@<机器人IP>:~/workspace/

# 2. SSH 登录机器人
ssh unitree@<机器人IP>

# 3. 安装 unitree_sdk2py (如尚未安装)
# 参考: https://github.com/unitreerobotics/unitree_sdk2py
pip3 install unitree_sdk2py

# 4. 确认 unitree_sdk2py 可导入
python3 -c "from unitree_sdk2py.core.channel import ChannelFactoryInitialize; print('SDK OK')"
```

---

## 配置 IMU 角色

使用前, 需要将 5 个 IMU 传感器分配到身体部位。编辑 `wireless_imu_roles.json`:

```json
{
  "chest_node_id": "949550",
  "right_upper_node_id": "9495B4",
  "right_forearm_node_id": "9495B8",
  "left_upper_node_id": "949534",
  "left_forearm_node_id": "949640",
  "chest_fixed_id": "chest",
  "right_upper_fixed_id": "right_upper_arm",
  "right_forearm_fixed_id": "right_forearm",
  "left_upper_fixed_id": "left_upper_arm",
  "left_forearm_fixed_id": "left_forearm"
}
```

**如何获取 IMU 设备 ID**:
1. 给所有 IMU 上电, 连接到路由器
2. 运行程序后, 系统会自动发现局域网中的 IMU 设备
3. 根据设备 ID 标签 (贴在每个 IMU 模块上) 对应填入

---

## 使用方法

### 方式一: PC 可视化模式

**步骤 1 — 启动机器人端**

```bash
# SSH 到机器人
ssh unitree@<机器人IP>
cd ~/workspace
python3 robot_arm_receiver.py
# 输出: "等待指令中..." 表示就绪
```

**步骤 2 — 启动 PC 端**

```bash
cd PC端
source .venv/bin/activate

# 方式 A: 自动发现机器人 (机器人在同一局域网)
python3 dual_arm_viz.py

# 方式 B: 直接指定机器人 IP
python3 dual_arm_viz.py --udp-target 192.168.3.85:9527
```

**步骤 3 — 操作**

1. 等待 IMU 全部连接 (屏幕左下角显示 5/5)
2. 佩戴 IMU 传感器:
   - 胸部 (IMU 平贴胸口, 朝上)
   - 右上臂 (外侧, 朝上)
   - 右前臂 (外侧, 朝上)
   - 左上臂 (外侧, 朝上)
   - 左前臂 (外侧, 朝上)
3. 按键盘操作:

| 按键 | 功能 |
|------|------|
| `R` | 切换到右臂控制 |
| `L` | 切换到左臂控制 |
| `SPACE` | 切换当前手臂的跟随/暂停 |
| `C` | 开始校准 (4步流程, 见下方) |
| `X` | 重置校准 |
| `D` | 录制/停止数据记录 |
| 鼠标拖拽 | 旋转 3D 视角 |
| 滚轮 | 缩放 |
| `Q` / `ESC` | 退出 |

**校准流程 (按 C 启动, 每步保持 3 秒)**:

| 步骤 | 姿势 | 说明 |
|------|------|------|
| 1 | 自然下垂 | 手臂自然放在身体两侧 |
| 2 | 前平举 | 手臂向前伸直, 与地面平行 |
| 3 | 侧平举 | 手臂向两侧伸直, 与地面平行 |
| 4 | 翻滚 | 前臂沿自身轴旋转 (握拳转动) |

> **重要**: 校准质量直接决定控制精度! 校准时保持姿势稳定, IMU 不要滑动。

### 方式二: 树莓派无头模式

**步骤 1 — 同上, 启动机器人端**

**步骤 2 — 在树莓派上启动**

```bash
# SSH 到树莓派
ssh pi@<树莓派IP>
cd /home/pi/teleop
source .venv/bin/activate

# 启动 (自动发现机器人)
python3 headless_arm_sender.py

# 或指定机器人 IP
python3 headless_arm_sender.py 192.168.3.85 9527
```

**步骤 3 — 操作 (终端 UI)**

| 按键 | 功能 |
|------|------|
| `R` | 切换到右臂 |
| `L` | 切换到左臂 |
| `SPACE` | 切换跟随/暂停 |
| `C` | 校准当前手臂 (侧平举, 保持 3 秒) |
| `X` | 重置校准 |
| `Q` / `ESC` | 退出 |

---

## 安全注意事项

1. **首次使用务必在仿真模式下测试**, 确认关节角度映射正确后再接真机
2. 机器人端内置安全保护:
   - 指令超时 1 秒 → 自动回零
   - 关节角度限幅 (不超过 URDF 定义的物理限位)
   - 启动时 3 秒渐变, 从当前位置缓慢接管
3. 回零速度 0.3 rad/s, 跟随限速 3.5~5.0 rad/s
4. 操作员应随时准备按 `SPACE` 暂停或 `Q` 退出
5. 确保机器人周围有足够的安全距离

---

## 通信协议详解

### IMU → PC/树莓派

| 端口 | 协议 | 用途 |
|------|------|------|
| UDP 4212 | 广播 | IMU 设备发现 |
| TCP 4210 | 连接 | IMU 配置/控制 |
| UDP 4211 | 数据流 | 四元数数据 (w, x, y, z) |

### PC/树莓派 → 机器人

| 端口 | 协议 | 用途 |
|------|------|------|
| UDP 9528 | 监听 | 机器人广播 `G1RC,ip=<IP>,port=<PORT>` |
| UDP 9527 | 发送 | 关节角度指令 |

**指令包格式** (二进制, 小端序):

```
偏移   大小    内容
0      4B     'UARM' (魔数)
4      1B     mode: 0=停止, 1=右臂跟随, 2=左臂跟随, 3=双臂
5      4f     右肩俯仰, 右肩横滚, 右肩偏航, 右肘, 右腕翻滚
25     4f     左肩俯仰, 左肩横滚, 左肩偏航, 左肘, 左腕翻滚
```

总计 45 字节, 50Hz 发送频率。

---

## 故障排除

| 问题 | 解决方法 |
|------|---------|
| IMU 连不上 | 检查 IMU 是否已连上路由器 WiFi; 检查 `wireless_imu_roles.json` 中 ID 是否正确 |
| 机器人没反应 | 确认 `robot_arm_receiver.py` 正在运行; 检查网络连通性 `ping <机器人IP>` |
| 关节运动方向反了 | 重新校准 (按 C); 检查 IMU 佩戴方向是否正确 |
| 运动延迟大 | 检查网络质量; 确保 IMU 数据率正常 (约 100Hz) |
| PyOpenGL 报错 | 确认已安装 OpenGL 驱动; Linux 下安装 `mesa-utils` |
| `ModuleNotFoundError` | 确认在正确的目录下运行程序; 检查虚拟环境是否激活 |

---

## 系统工作原理

1. **数据采集**: 5 个无线 IMU 以 ~100Hz 频率输出四元数 (方向)
2. **坐标对齐**: 通过校准流程, 将 IMU 坐标系对齐到人体骨骼坐标系
3. **增量跟踪**: 使用四元数增量累积消除 IMU 漂移
4. **逆运动学 (IK)**: 从手臂/前臂方向向量解析求出 5 个关节角度
5. **平滑输出**: 限速 + 低通滤波, 防止电机抖动
6. **PD 控制**: 机器人端以 500Hz 频率执行 PD 位置控制

---

## 版本信息

- 开发日期: 2026 年春
- 目标机器人: Unitree G1
- IMU 传感器: ESP32 无线 IMU (自定义固件)
- Python 版本: 3.8+
