# Force_handcontrol — 灵巧手力反馈双边遥操作 V3

> 文件：`Force_handcontrol.py`  
> 更新：2026-05-08  
> 相对 V2 新增：TCP 桥接模式（树莓派通过 WiFi 经 G1 控制灵巧手）

---

## 目录

1. [系统架构](#系统架构)
2. [两种通信模式对比](#两种通信模式对比)
3. [模式一：原始模式（网线直连）](#模式一原始模式网线直连)
4. [模式二：TCP 桥接模式（通过 G1）](#模式二tcp-桥接模式通过-g1)
5. [配置参数](#配置参数)
6. [启动顺序](#启动顺序)
7. [状态机说明](#状态机说明)
8. [常见问题](#常见问题)

---

## 系统架构

### 模式一：原始模式（V2，网线直连）
```
BLE手套 → ble_broker.py(TCP 9001) → Force_handcontrol.py
                                          ├─ DDS eth0 → 灵巧手（192.168.123.211）
                                          └─ Dynamixel XL330×5（/dev/ttyAMA0）
```

### 模式二：TCP 桥接模式（V3，通过 G1）
```
BLE手套 → ble_broker.py(TCP 9001) → Force_handcontrol.py（树莓派 wlan0）
                                          ├─ TCP:9100 → hand_bridge.py（G1）
                                          │                └─ DDS eth0 → 灵巧手（192.168.123.211）
                                          ├─ TCP:9101 ← hand_bridge.py（触觉反馈）
                                          └─ Dynamixel XL330×5（/dev/ttyAMA0）
```

---

## 两种通信模式对比

| 对比项 | 模式一（原始/网线） | 模式二（TCP桥接/WiFi） |
|--------|-------------------|----------------------|
| 灵巧手连接 | 树莓派 eth0 直连 | G1 eth0 连接 |
| 树莓派网络需求 | 仅需有线网卡 | 仅需 WiFi（wlan0） |
| G1 需要运行 | 不需要 | `hand_bridge.py` |
| 延迟 | 低（直连） | 略高（WiFi+TCP转发） |
| 移动灵活性 | 受网线长度限制 | 无线自由移动 |
| 参数设置 | `USE_TCP_BRIDGE = False` | `USE_TCP_BRIDGE = True` |

---

## 模式一：原始模式（网线直连）

### 硬件连接
```
树莓派
  ├── eth0 ──网线──► 灵巧手（192.168.123.211）
  └── /dev/ttyAMA0 ──► Dynamixel XL330×5
```

### 配置
```python
# Force_handcontrol.py 顶部参数区
USE_TCP_BRIDGE = False
```

### 启动（3 个终端，均在树莓派上）
ssh pi@10.42.0.174
**终端 1 — 灵巧手驱动**
```bash
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/ftp/inspire_hand_ws
python3 inspire_hand_sdk/example/Headless_driver_r.py
```

**终端 2 — BLE Broker**
```bash
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/Five_finger_force_test
python3 ble_broker.py
```

**终端 3 — 主程序**
```bash
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/ftp
python3 Force_handcontrol.py eth0
```

---

## 模式二：TCP 桥接模式（通过 G1）

### 硬件连接
```
树莓派 ──WiFi──► 路由器 ──► G1（192.168.3.85）
                                └──eth0──► 灵巧手（192.168.123.211）
树莓派 /dev/ttyAMA0 ──► Dynamixel XL330×5
```

### 网络要求
- 树莓派和 G1 连接同一 WiFi
- G1 已开启 IP 转发（`sysctl net.ipv4.ip_forward=1`）

### G1 首次部署（安装 SDK）

> 若 G1 上已安装过 `inspire_hand_ws`，跳过此步，直接看「文件部署」。

> ⚠️ **架构说明**：G1 是 aarch64，repo 自带的 `venv_x86.tar.xz` 不可用。

**步骤一：传入 inspire_hand_ws（在开发机执行）**
```bash
scp -r /home/wxc/projects/dexEXO/ftp/inspire_hand_ws unitree@192.168.3.85:~/dexEXO/
```

**步骤二：注释掉 `__init__.py` 中的 GUI 导入（在 G1 上执行）**

`inspire_sdkpy/__init__.py` 会无条件加载 `pyqtgraph` 等 GUI 包，G1 上无法安装，需注释掉：
```bash
sed -i 's/^from .qt_tabs import/#from .qt_tabs import/' \
  ~/dexEXO/inspire_hand_ws/inspire_hand_sdk/inspire_sdkpy/__init__.py
```

**步骤三：安装缺失的纯 Python 依赖（在开发机下载，传到 G1 安装）**
```bash
# 开发机执行：
mkdir -p /tmp/g1_pkgs
pip download pymodbus==3.6.9 pyserial==3.5 --no-deps -d /tmp/g1_pkgs
scp /tmp/g1_pkgs/*.whl unitree@192.168.3.85:/tmp/

# G1 上执行：
pip3 install --user /tmp/pymodbus-3.6.9-py3-none-any.whl /tmp/pyserial-3.5-py2.py3-none-any.whl
```

**步骤四：写入 ~/.bashrc（G1 上执行，永久生效）**
```bash
echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
echo 'export PYTHONPATH=$PYTHONPATH:/home/unitree/workspace/zhw_workspace:/home/unitree/dexEXO/inspire_hand_ws/inspire_hand_sdk' >> ~/.bashrc
source ~/.bashrc
```

**步骤五：验证**
```bash
python3 -c "from unitree_sdk2py.core.channel import ChannelPublisher; print('unitree OK')"
python3 -c "from inspire_sdkpy import inspire_dds; print('inspire OK')"
python3 ~/dexEXO/hand_bridge.py eth0
# 看到"等待树莓派连接…"即表示成功
```

### 文件部署

**将 `hand_bridge.py` 从电脑传到 G1：**
```bash
# 在树莓派上执行
scp /home/pi/dexEXO/ftp/hand_bridge.py unitree@192.168.3.85:~/dexEXO/
```

### 配置
```python
# Force_handcontrol.py 顶部参数区
USE_TCP_BRIDGE   = True
BRIDGE_HOST      = "192.168.3.85"   # G1 的 WiFi IP
BRIDGE_CTRL_PORT = 9100
BRIDGE_TOUCH_PORT = 9101
```

### 启动顺序（4 个终端）

**G1 终端 1 — 灵巧手驱动（G1 上运行）**
```bash
ssh unitree@192.168.6.146
cd ~/dexEXO/inspire_hand_ws
python3 inspire_hand_sdk/example/Headless_driver_r.py
```
等待灵巧手完成自检后继续。

**G1 终端 2 — TCP 桥接（G1 上运行）**
```bash
ssh unitree@192.168.3.85
python3 ~/dexEXO/hand_bridge.py eth0
```
看到 `等待树莓派连接…` 后继续。

**树莓派终端 1 — BLE Broker**
```bash
ssh pi@10.42.0.174
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/Five_finger_force_test
python3 ble_broker.py
```
等待输出 `[BLE] 已连接 F0:FD:45:02:85:B3，等待客户端...` 后继续。

**树莓派终端 2 — 主程序**
```bash
ssh pi@10.42.0.174
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/ftp
python3 Force_handcontrol.py
```

---

## 配置参数

### 通信模式0

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `USE_TCP_BRIDGE` | `True` | `True`=TCP桥接模式，`False`=DDS直连模式 |
| `BRIDGE_HOST` | `192.168.3.85` | G1 的 WiFi IP |
| `BRIDGE_CTRL_PORT` | `9100` | 角度指令端口 |
| `BRIDGE_TOUCH_PORT` | `9101` | 触觉反馈端口 |

### 状态机阈值

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `HAND_CONTACT_ON_N` | `0.30 N` | 灵巧手触觉超过此值 → 进入 FORCE_ENTRY |
| `EXO_ZERO_THR_N` | `0.05 N` | 外骨骼触觉判零阈值 |
| `RELEASE_HOLD_SEC` | `0.30 s` | 触觉归零后保持时间 → 切回 GLOVE |
| `LOCK_ANGLE_RETREAT` | `80 tick` | 进入 LOCKED 时角度向伸直方向退让量 |

### 舵机力控

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `HAND_TO_SERVO_GAIN` | `60.0 mA/N` | FORCE_ENTRY：灵巧手触觉→舵机电流增益 |
| `FORCE_KP` | `35.0 tick/N` | LOCKED 阶段 PID 比例增益 |
| `FORCE_STEP_MAX` | `12.0 tick` | 单周期最大位置修正 |
| `HAND_FORCE_BASE` | `200` | LOCKED 阶段灵巧手基础保持力 |
| `HAND_FORCE_GAIN` | `40.0` | exo_force → force_set 增益 |

---

## 启动顺序（模式二快速参考）

```
[G1]   Headless_driver_r.py   ← 灵巧手与 G1 建立 Modbus 连接
[G1]   hand_bridge.py         ← 开启 TCP 9100/9101 端口等待树莓派
[Pi]   ble_broker.py          ← BLE 手套连接
[Pi]   Force_handcontrol.py   ← 主程序，连接 G1 TCP，开始遥操作
```

启动成功后输入：
```
INIT    ← 记录舵机初始位置（必须）
STATUS  ← 查看各指状态
EXIT    ← 退出
```

---

## 状态机说明

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
| `GLOVE` | 手套弯曲角度控制（150=伸直，850=握拳） | 零电流，自由 |
| `FORCE_ENTRY` | 冻结在接触时角度 | 电流 = 灵巧手触觉 × 60 mA/N |
| `LOCKED` | PID（err=exo_force-hand_force，err>0→角度减小→握紧） | 位置模式锁死 |
| `RELEASE` | 冻结 | 反向 -150 mA 归位 |

---

## 常见问题

**Q: TCP 桥接模式下触觉数据始终为 0**  
A: 检查 G1 上 `hand_bridge.py` 是否正常运行；检查 DDS `HAND_TOUCH_TOPIC` 话题是否有数据（先用 `dds_debug_all_fingers.py` 验证）。

**Q: TCP 控制连接一直失败**  
A: 确认 `hand_bridge.py` 已在 G1 上启动；确认 `BRIDGE_HOST` 填写的是 G1 的 WiFi IP（`192.168.3.85`）；检查防火墙：`sudo ufw allow 9100 && sudo ufw allow 9101`。

**Q: 切回原始模式（网线直连）**  
A: 修改 `Force_handcontrol.py` 中 `USE_TCP_BRIDGE = False`，不启动 `hand_bridge.py`，按 V2 流程操作。

**Q: 舵机 Ping 失败**  
A: 检查 `/dev/ttyAMA0` 权限，确认波特率 1 Mbps，确认 ID 1~5。

**Q: 状态机不进入 FORCE_ENTRY**  
A: 灵巧手触觉未超过 `HAND_CONTACT_ON_N = 0.30 N`；用 `STATUS` 命令查看 `hand_force` 是否有值。

**Q: G1 上启动报 `pyqtgraph` 错误**  
A: `inspire_sdkpy/__init__.py` 会无条件加载 GUI 模块，G1 上 numpy 版本冲突无法安装。
执行一次以下命令注释掉该行即可：
```bash
sed -i 's/^from .qt_tabs import/#from .qt_tabs import/' \
  ~/dexEXO/inspire_hand_ws/inspire_hand_sdk/inspire_sdkpy/__init__.py
```

**Q: G1 上 cyclonedds 版本冲突（0.10.2 vs 0.10.5）**  
A: 不要用 `pip install -e .` 安装 inspire_hand_sdk 或 unitree_sdk2_python。
直接用 PYTHONPATH 加路径 + ROS foxy 系统 cyclonedds 即可正常运行。
