# 灵巧手力反馈双边遥操作 — V4 双手版

> 文件：`both_Force_handcontrol.py`（Pi）+ `both_hand_bridge.py`（G1）  
> 更新：2026-05-18  
> 相对 V3 新增：**左右手同时控制**，单 G1 进程桥接双手  
> 2026-05-18 修复：`ble_broker.py` 新增 `--port` / `--devices` 命令行参数，解决左手 Broker 9002 端口无法绑定问题

---

## 目录

1. [系统架构](#系统架构)
2. [端口分配](#端口分配)
3. [文件清单](#文件清单)
4. [G1 部署（首次）](#g1-部署首次)
5. [启动顺序](#启动顺序)
6. [配置参数](#配置参数)
7. [舵机 ID 分配](#舵机-id-分配)
8. [状态机说明](#状态机说明)
9. [常见问题](#常见问题)
10. [单手模式（V3 兼容）](#单手模式v3-兼容)

---

## 系统架构

```
【树莓派 Pi 10.42.0.174】
  ble_broker.py(TCP 9001) ← BLE 右手手套
  ble_broker.py(TCP 9002) ← BLE 左手手套
  both_Force_handcontrol.py
    ├─ TCP:9100 ──────────────────────────────────────► G1
    ├─ TCP:9101 ◄────────────────── 右手触觉反馈        │
    ├─ TCP:9102 ────────────────────────────────────►   │ both_hand_bridge.py
    ├─ TCP:9103 ◄────────────────── 左手触觉反馈        │
    └─ /dev/ttyAMA0 → Dynamixel XL330×10（双手）       │
                                                  【G1 192.168.3.85】
                                                    eth0 → 右手（192.168.123.211）
                                                    eth0 → 左手（192.168.123.210）
```

---

## 端口分配

| 端口 | 方向 | 说明 |
|------|------|------|
| `9100` | Pi → G1 | **右手**角度指令（TCP ctrl） |
| `9101` | G1 → Pi | **右手**触觉反馈（TCP touch） |
| `9102` | Pi → G1 | **左手**角度指令（TCP ctrl） |
| `9103` | G1 → Pi | **左手**触觉反馈（TCP touch） |
| `9001` | Pi 本地 | BLE Broker **右手**手套数据 |
| `9002` | Pi 本地 | BLE Broker **左手**手套数据 |

---

## 文件清单

| 文件 | 运行位置 | 说明 |
|------|----------|------|
| `both_hand_bridge.py` | **G1** | 双手 DDS↔TCP 桥接，单进程 |
| `both_Force_handcontrol.py` | **Pi** | 双手力反馈主程序 |
| `hand_bridge.py` | G1（可选） | 单右手桥接（V3，保留不变） |
| `Force_handcontrol.py` | Pi（可选） | 单右手主程序（V3，保留不变） |
| `ble_broker.py` | Pi × 2 | BLE → TCP 转发，需启动两个实例 |

---

## G1 部署（首次）

> 若已按 V3 `README_V3.md` 配置过 G1（PYTHONPATH、inspire_hand_ws），只需额外传输 `both_hand_bridge.py`，跳到「文件部署」。

### 1. 传输 inspire_hand_ws（开发机执行）

```bash
scp -r /home/wxc/projects/dexEXO/ftp/inspire_hand_ws unitree@192.168.3.85:~/dexEXO/
```

### 2. 注释 GUI 导入（G1 上执行）

```bash
sed -i 's/^from .qt_tabs import/#from .qt_tabs import/' \
  ~/dexEXO/inspire_hand_ws/inspire_hand_sdk/inspire_sdkpy/__init__.py
```

### 3. 写入环境变量（G1 上执行，只需一次）

```bash
cat >> ~/.bashrc << 'EOF'
source /opt/ros/foxy/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/unitree/workspace/zhw_workspace:/home/unitree/dexEXO/inspire_hand_ws/inspire_hand_sdk
EOF
source ~/.bashrc
```

### 4. 验证

```bash
python3 -c "from unitree_sdk2py.core.channel import ChannelPublisher; print('unitree OK')"
python3 -c "from inspire_sdkpy import inspire_dds; print('inspire OK')"
```

### 5. 文件部署

```bash
# 在开发机执行——传输桥接脚本到 G1
scp /home/wxc/projects/dexEXO/ftp/both_hand_bridge.py unitree@192.168.3.85:~/dexEXO/

# 在开发机执行——传输 BLE Broker（已支持 --port 参数）到 Pi
scp /home/wxc/projects/dexEXO/ftp/ble_broker.py \
    pi@10.42.0.174:/home/pi/dexEXO/ftp/

# 在开发机执行——传输双手主程序到 Pi
scp /home/wxc/projects/dexEXO/ftp/both_Force_handcontrol.py \
    pi@10.42.0.174:/home/pi/dexEXO/ftp/
```

---

## 启动顺序

> **每次启动前**先确认 G1 与 Pi 网络互通：
> ```bash
> # 开发机上测试
> ping 192.168.3.85   # G1
> ping 10.42.0.174    # Pi
> ```

---

### 【宇树 G1】终端 1 — 右手灵巧手驱动

```bash
ssh unitree@192.168.3.85
cd ~/dexEXO/inspire_hand_ws
python3 inspire_hand_sdk/example/Headless_driver_r.py
```

等待右手自检完成（约 5 s），看到频率稳定输出后继续。

---

### 【宇树 G1】终端 2 — 左手灵巧手驱动

```bash
ssh unitree@192.168.3.85
cd ~/dexEXO/inspire_hand_ws
python3 inspire_hand_sdk/example/Headless_driver_l.py
```

等待左手自检完成后继续。

---

### 【宇树 G1】终端 3 — 双手 TCP 桥接

```bash
ssh unitree@192.168.3.85
python3 ~/dexEXO/both_hand_bridge.py eth0
```

> ⚠️ 注意：文件在 `~/dexEXO/both_hand_bridge.py`，不是 `~/dexEXO/ftp/`

看到如下四行输出后继续：

```
[右手][TCP控制] 监听 0.0.0.0:9100
[右手][TCP触觉] 监听 0.0.0.0:9101
[左手][TCP控制] 监听 0.0.0.0:9102
[左手][TCP触觉] 监听 0.0.0.0:9103
```

---

### 【树莓派 Pi】终端 1 — 右手 BLE Broker

```bash
ssh pi@10.42.0.174
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/ftp
# --devices 指定右手手套 BLE 地址，--port 指定本地监听端口
python3 ble_broker.py --port 9001 --devices F0:FD:45:02:85:B3
```

看到 `BLE 已连接: F0:FD:45:02:85:B3` 后继续。

---

### 【树莓派 Pi】终端 2 — 左手 BLE Broker

```bash
ssh pi@10.42.0.174
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/ftp
# --devices 指定左手手套 BLE 地址，--port 使用独立端口 9002
python3 ble_broker.py --port 9002 --devices F0:FD:45:02:67:3B
```

看到 `BLE 已连接: F0:FD:45:02:67:3B` 后继续。

> ⚠️ **两个 Broker 必须用不同的 `--port` 和 `--devices`**，否则第二个实例会因端口冲突（`Address already in use`）启动失败，导致左手持续 `Connection refused`。

---

### 【树莓派 Pi】终端 3 — 双手主程序

```bash
ssh pi@10.42.0.174
source /home/pi/dexEXO/dexEXO/bin/activate
cd /home/pi/dexEXO/ftp
python3 both_Force_handcontrol.py
```

启动后按提示操作：

```
INIT      ← 记录双手舵机初始位置（必须，戴上手套后执行）
STATUS    ← 查看双手状态
EXIT      ← 退出
```

---

## 配置参数

在 `both_Force_handcontrol.py` 顶部的 `HAND_CONFIGS` 字典中修改：

```python
HAND_CONFIGS = {
    "r": {
        "label"            : "右手",
        "bridge_ctrl_port" : 9100,
        "bridge_touch_port": 9101,
        "ble_broker_port"  : 9001,
        "dxl_ids"          : [1, 2, 3, 4, 5],
        "dxl_device"       : "/dev/ttyAMA0",
    },
    "l": {
        "label"            : "左手",
        "bridge_ctrl_port" : 9102,
        "bridge_touch_port": 9103,
        "ble_broker_port"  : 9002,
        "dxl_ids"          : [6, 7, 8, 9, 10],
        "dxl_device"       : "/dev/ttyAMA0",
    },
}

BRIDGE_HOST = "192.168.3.85"   # G1 的 WiFi IP
```

### 状态机阈值

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `HAND_CONTACT_ON_N` | `0.50 N` | 灵巧手触觉超过此值 → 进入 FORCE_ENTRY |
| `EXO_ZERO_THR_N` | `0.10 N` | 外骨骼触觉判零阈值 |
| `RELEASE_HOLD_SEC` | `2.00 s` | 触觉归零后持续时间 → 切回 GLOVE |
| `LOCK_ANGLE_RETREAT` | `80 tick` | 进入 LOCKED 时角度向伸直方向退让量 |

### 力控参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `HAND_TO_SERVO_GAIN` | `60.0 mA/N` | FORCE_ENTRY：灵巧手触觉→舵机电流增益 |
| `FORCE_KP` | `35.0 tick/N` | LOCKED 阶段 PID 比例增益 |
| `FORCE_STEP_MAX` | `3.0 tick` | 单周期最大位置修正 |
| `HAND_FORCE_BASE` | `200` | LOCKED 阶段灵巧手基础保持力 |
| `HAND_FORCE_GAIN` | `40.0` | exo_force → force_set 增益 |
| `TOTAL_CURRENT_LIMIT_MA` | `800 mA` | 单手五指总电流上限 |

---

## 舵机 ID 分配

| 手指 | 右手 ID | 左手 ID |
|------|---------|---------|
| 拇指 | 1 | 6 |
| 食指 | 2 | 7 |
| 中指 | 3 | 8 |
| 无名指 | 4 | 9 |
| 小指 | 5 | 10 |

左右手舵机挂在同一 RS485 总线上（`/dev/ttyAMA0`），通过 ID 区分。

> 若左右手使用**不同串口**，修改 `HAND_CONFIGS["l"]["dxl_device"]` 为第二个串口路径，  
> 并在 `setup_servos()` 中添加第二个 `PortHandler` 实例。

---

## 状态机说明

每手 5 指独立运行，状态机逻辑与 V3 单手版完全相同：

```
         灵巧手触觉 ≥ 0.50 N
GLOVE ──────────────────────► FORCE_ENTRY
  ▲                                │
  │                    舵机位置稳定(0.4s)
  │                                ▼
  │                            LOCKED
  │                                │
  │         外骨骼触觉≈0 持续 2.0s │
  │                                ▼
  └─────────────────────────── RELEASE
           舵机归位完成
```

| 状态 | 灵巧手 | 舵机 |
|------|--------|------|
| `GLOVE` | 手套弯曲角度控制（150=伸直，850=握拳） | 零电流，自由 |
| `FORCE_ENTRY` | 冻结在接触时角度 | 电流 = 灵巧手触觉 × 60 mA/N |
| `LOCKED` | PID（err=exo_force−hand_force，err>0→减小角度→握紧） | 位置模式锁死 |
| `RELEASE` | 跟随手套 | 反向 −150 mA 归位 |

---

## 常见问题

**Q: 左手 BLE Broker 报 `OSError: [Errno 98] Address already in use`**  
A: 两个 Broker 实例必须用 `--port 9001` 和 `--port 9002` 分开启动。先 `pkill -f ble_broker.py` 杀掉残留进程，再按启动顺序重新启动。

**Q: 某手触觉数据始终为 0**  
A: 检查 G1 上 `both_hand_bridge.py` 是否输出该手的 DDS 订阅成功信息；检查对应 Headless_driver 是否正常运行。

**Q: 左手控制连接一直失败（9102/9103 端口）**  
A: 确认 `both_hand_bridge.py`（非旧版 `hand_bridge.py`）在 G1 上运行；检查 `BRIDGE_HOST` 填写 G1 WiFi IP（`192.168.3.85`）。

**Q: 两手手套数据串混**  
A: 每个 Broker 用 `--devices` 指定自己负责的 BLE 地址：  
- 右手：`--devices F0:FD:45:02:85:B3`  
- 左手：`--devices F0:FD:45:02:67:3B`

**Q: 左手舵机 Ping 全部失败**  
A: 确认左手舵机 ID 已编程为 6~10（默认出厂 ID=1，需用 Dynamixel Wizard 逐个修改）。

**Q: 想只用右手（回退 V3 模式）**  
A: 直接启动 `hand_bridge.py`（G1）+ `Force_handcontrol.py`（Pi），与 V4 文件完全独立，互不影响。

**Q: G1 报 cyclonedds 版本冲突**  
A: 不要 `pip install -e .`，只通过 `PYTHONPATH` 加路径 + ROS foxy 系统 cyclonedds 即可。参见 V3 README 的详细说明。

**Q: `video_id` 找不到（摄像头问题）**  
A: G1 摄像头的 video_id 重启后可能变化（video4→video5）。  
用 `teleimager-server --cf` 查看当前可用设备，修改 `cam_config_server.yaml` 中的 `video_id` 字段。

---

## 单手模式（V3 兼容）

V3 文件保持不变，可随时切回：

```bash
# G1
python3 ~/dexEXO/hand_bridge.py eth0        # 仅开 9100/9101

# Pi
python3 Force_handcontrol.py                 # 仅连右手
```

V3 与 V4 使用**不同端口**，可同时运行（不冲突），但不建议同时启动同一灵巧手的驱动。

---

## 快速部署（开发机一键发送文件）

> 在**开发机**（`/home/wxc/projects/dexEXO`）执行以下命令，将所有最新文件推送到 G1 和 Pi。

### 发送到宇树 G1（192.168.3.85）

```bash
# 桥接脚本（核心）
scp /home/wxc/projects/dexEXO/ftp/both_hand_bridge.py \
    unitree@192.168.3.85:~/dexEXO/both_hand_bridge.py

# 若需同步整个 inspire_hand_ws SDK（首次或更新 SDK 时）
scp -r /home/wxc/projects/dexEXO/ftp/inspire_hand_ws \
    unitree@192.168.3.85:~/dexEXO/
```

### 发送到树莓派 Pi（10.42.0.174）

```bash
# BLE Broker（已支持 --port / --devices 参数）
scp /home/wxc/projects/dexEXO/Five_finger_force_test/ble_broker.py \
    pi@10.42.0.174:/home/pi/dexEXO/Five_finger_force_test/ble_broker.py

# 双手主程序
scp /home/wxc/projects/dexEXO/ftp/both_Force_handcontrol.py \
    pi@10.42.0.174:/home/pi/dexEXO/ftp/both_Force_handcontrol.py
```

### 一键全部推送（复制执行）

```bash
PI=pi@10.42.0.174
G1=unitree@192.168.3.85
BASE=/home/wxc/projects/dexEXO

scp $BASE/ftp/both_hand_bridge.py         $G1:~/dexEXO/
scp $BASE/Five_finger_force_test/ble_broker.py  $PI:/home/pi/dexEXO/Five_finger_force_test/
scp $BASE/ftp/both_Force_handcontrol.py   $PI:/home/pi/dexEXO/ftp/
echo "✅ 文件推送完成"
```
