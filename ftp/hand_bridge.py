#!/usr/bin/env python3
"""
hand_bridge.py — 灵巧手 TCP ↔ DDS 桥接程序
在宇树 G1 上运行，负责：
  1. 监听树莓派的 TCP 连接，接收角度/力控指令 → 发布 DDS rt/inspire_hand/ctrl/r
  2. 订阅 DDS rt/inspire_hand/touch/r 触觉数据 → 发送给树莓派

通信协议（JSON，换行符分隔）：
  树莓派 → G1（角度指令）：
    {"type":"ctrl","angle_set":[a0,a1,a2,a3,a4,a5],"force_set":[f0,f1,f2,f3,f4,f5],"speed_set":[s0,s1,s2,s3,s4,s5],"mode":1}

  G1 → 树莓派（触觉反馈）：
    {"type":"touch","forces":[f0,f1,f2,f3,f4,f5]}

启动：
  python3 hand_bridge.py [网卡名，默认eth0]
  例如：python3 hand_bridge.py eth0
"""

import sys
import json
import socket
import threading
import time

# ========== DDS 路径 ==========
# G1 是 aarch64，必须用系统 Python + ROS foxy 的 cyclonedds，不可用 venv_x86
# 启动前需执行：
#   source /opt/ros/foxy/setup.bash
#   export PYTHONPATH=$PYTHONPATH:/home/unitree/workspace/zhw_workspace:/home/unitree/dexEXO/inspire_hand_ws/inspire_hand_sdk/inspire_sdkpy
sys.path.insert(0, '/home/unitree/workspace/zhw_workspace')
sys.path.insert(0, '/home/unitree/dexEXO/inspire_hand_ws/inspire_hand_sdk')

# ========== DDS ==========
try:
    from unitree_sdk2py.core.channel import (
        ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
    )
    # 直接导入模块文件，绕过 inspire_sdkpy/__init__.py（避免加载 pyqtgraph 等 GUI 依赖）
    import importlib.util, os as _os
    _dds_dir = '/home/unitree/dexEXO/inspire_hand_ws/inspire_hand_sdk/inspire_sdkpy/inspire_dds'
    def _load(name, path):
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    _ctrl_mod  = _load('_inspire_hand_ctrl',  _os.path.join(_dds_dir, '_inspire_hand_ctrl.py'))
    _touch_mod = _load('_inspire_hand_touch', _os.path.join(_dds_dir, '_inspire_hand_touch.py'))
    inspire_hand_ctrl  = _ctrl_mod.inspire_hand_ctrl
    inspire_hand_touch = _touch_mod.inspire_hand_touch
    DDS_AVAILABLE = True
    print("[DDS] 库加载成功")
except ImportError as e:
    DDS_AVAILABLE = False
    print(f"[警告] DDS 库未找到({e})，以模拟模式运行")

# =====================================================================
# ======================== 参数配置 ===================================
# =====================================================================

BRIDGE_HOST     = "0.0.0.0"    # 监听所有网卡
BRIDGE_CTRL_PORT = 9100         # 接收树莓派角度指令的端口
BRIDGE_TOUCH_PORT = 9101        # 向树莓派发送触觉反馈的端口

HAND_CTRL_TOPIC  = "rt/inspire_hand/ctrl/r"
HAND_TOUCH_TOPIC = "rt/inspire_hand/touch/r"

# 灵巧手触觉字段（与 Force_handcontrol.py 保持完全一致）
TOUCH_FIELDS = [
    "fingerfive_top_touch",   # 拇指
    "fingerfour_top_touch",   # 食指
    "fingerthree_top_touch",  # 中指
    "fingertwo_top_touch",    # 无名指
    "fingerone_top_touch",    # 小指
]

# =====================================================================
# ======================== 全局状态 ===================================
# =====================================================================

running = True
latest_touch_forces = [0.0] * 6    # 最新触觉数据（6槽，与灵巧手 angle_set 对齐）

# 已连接的触觉推送客户端列表
touch_clients: list = []
touch_clients_lock = threading.Lock()

# DDS 发布器
dds_pub = None

# =====================================================================
# ======================== 触觉原始值 → 牛顿 ==========================
# =====================================================================

CALIBRATION_K       = 0.00292650244415058227
CALIBRATION_B       = -0.6037947156125716
THUMB_CALIBRATION_K = 0.004420145759358057
THUMB_CALIBRATION_B = -1.0701492398616255
FORCE_MAX_N         = 10.0

def raw_to_force_n(raw: float, is_thumb: bool = False) -> float:
    if is_thumb:
        f = THUMB_CALIBRATION_K * raw + THUMB_CALIBRATION_B
    else:
        f = CALIBRATION_K * raw + CALIBRATION_B
    return float(max(0.0, min(f, FORCE_MAX_N)))

# =====================================================================
# ======================== DDS 触觉订阅 ===============================
# =====================================================================

def on_touch_msg(msg):
    """DDS 触觉回调：解析触觉数据，更新全局变量，推送给所有已连接客户端"""
    global latest_touch_forces
    try:
        forces = []
        for i, field in enumerate(TOUCH_FIELDS):
            raw_seq = getattr(msg, field, None)
            if raw_seq is None or len(raw_seq) == 0:
                raw = 0.0
            else:
                raw = float(max(raw_seq))
            is_thumb = (i == 0)
            forces.append(raw_to_force_n(raw, is_thumb))
        # 补齐第6槽（拇旋，无触觉）
        forces_6 = forces + [0.0]
        latest_touch_forces = forces_6

        # 推送给所有树莓派客户端
        payload = json.dumps({"type": "touch", "forces": forces_6}) + "\n"
        data = payload.encode("utf-8")
        with touch_clients_lock:
            dead = []
            for conn in touch_clients:
                try:
                    conn.sendall(data)
                except Exception:
                    dead.append(conn)
            for conn in dead:
                touch_clients.remove(conn)
                try:
                    conn.close()
                except Exception:
                    pass

    except Exception as e:
        print(f"[DDS触觉] 解析错误: {e}")


# =====================================================================
# ======================== TCP 控制指令服务端 =========================
# =====================================================================

def handle_ctrl_client(conn: socket.socket, addr) -> None:
    """接收树莓派发来的角度/力控指令，写入DDS"""
    print(f"[TCP控制] 客户端连接: {addr}")
    buf = ""
    try:
        while running:
            chunk = conn.recv(4096)
            if not chunk:
                break
            buf += chunk.decode("utf-8", errors="ignore")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    pkt = json.loads(line)
                    if pkt.get("type") == "ctrl" and dds_pub is not None:
                        msg = inspire_hand_ctrl(
                            pos_set   = [0] * 6,
                            angle_set = tuple(int(a) for a in pkt.get("angle_set", [500]*6)),
                            force_set = list(pkt.get("force_set", [200]*6)),
                            speed_set = list(pkt.get("speed_set", [500]*6)),
                            mode      = pkt.get("mode", 1),
                        )
                        dds_pub.Write(msg)
                    elif pkt.get("type") == "ctrl" and not DDS_AVAILABLE:
                        # 模拟模式：打印收到的指令
                        print(f"[模拟] 收到指令: angle={pkt.get('angle_set')}")
                except json.JSONDecodeError:
                    pass
                except Exception as e:
                    print(f"[TCP控制] 处理指令错误: {e}")
    except Exception as e:
        print(f"[TCP控制] 连接错误: {e}")
    finally:
        conn.close()
        print(f"[TCP控制] 客户端断开: {addr}")


def ctrl_server() -> None:
    """控制指令服务端：持续监听树莓派连接"""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((BRIDGE_HOST, BRIDGE_CTRL_PORT))
    srv.listen(5)
    srv.settimeout(1.0)
    print(f"[TCP控制] 监听 {BRIDGE_HOST}:{BRIDGE_CTRL_PORT}")
    while running:
        try:
            conn, addr = srv.accept()
            t = threading.Thread(target=handle_ctrl_client, args=(conn, addr), daemon=True)
            t.start()
        except socket.timeout:
            continue
        except Exception as e:
            if running:
                print(f"[TCP控制] 接受连接错误: {e}")
    srv.close()


# =====================================================================
# ======================== TCP 触觉推送服务端 =========================
# =====================================================================

def handle_touch_client(conn: socket.socket, addr) -> None:
    """将此客户端加入推送列表，连接保持直到断开"""
    print(f"[TCP触觉] 客户端连接: {addr}")
    with touch_clients_lock:
        touch_clients.append(conn)
    # 保持线程存活（推送由 on_touch_msg 完成）
    try:
        while running:
            # 保持连接，检测对方是否断开（recv超时）
            conn.settimeout(5.0)
            try:
                data = conn.recv(16)
                if not data:
                    break
            except socket.timeout:
                continue
    except Exception:
        pass
    finally:
        with touch_clients_lock:
            if conn in touch_clients:
                touch_clients.remove(conn)
        conn.close()
        print(f"[TCP触觉] 客户端断开: {addr}")


def touch_server() -> None:
    """触觉推送服务端：持续监听树莓派连接"""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((BRIDGE_HOST, BRIDGE_TOUCH_PORT))
    srv.listen(5)
    srv.settimeout(1.0)
    print(f"[TCP触觉] 监听 {BRIDGE_HOST}:{BRIDGE_TOUCH_PORT}")
    while running:
        try:
            conn, addr = srv.accept()
            t = threading.Thread(target=handle_touch_client, args=(conn, addr), daemon=True)
            t.start()
        except socket.timeout:
            continue
        except Exception as e:
            if running:
                print(f"[TCP触觉] 接受连接错误: {e}")
    srv.close()


# =====================================================================
# ======================== 程序入口 ====================================
# =====================================================================

import signal

def signal_handler(sig, frame):
    global running
    print("\n接收到 Ctrl+C，正在停止…")
    running = False

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    print("=" * 60)
    print("   hand_bridge.py  灵巧手 TCP ↔ DDS 桥接")
    print("   运行在宇树 G1 上")
    print("=" * 60)

    # DDS 初始化
    if DDS_AVAILABLE:
        nic = sys.argv[1] if len(sys.argv) > 1 else "eth0"
        print(f"[DDS] 初始化，网卡={nic}")
        ChannelFactoryInitialize(0, nic)

        # 发布控制指令
        dds_pub = ChannelPublisher(HAND_CTRL_TOPIC, inspire_hand_ctrl)
        dds_pub.Init()
        print(f"[DDS] 发布器已启动: {HAND_CTRL_TOPIC}")

        # 订阅触觉反馈
        touch_sub = ChannelSubscriber(HAND_TOUCH_TOPIC, inspire_hand_touch)
        touch_sub.Init(on_touch_msg, 10)
        print(f"[DDS] 订阅器已启动: {HAND_TOUCH_TOPIC}")
    else:
        dds_pub = None
        print("[警告] DDS 不可用，以模拟模式运行")

    # 启动 TCP 服务端
    threading.Thread(target=ctrl_server, daemon=True).start()
    threading.Thread(target=touch_server, daemon=True).start()

    print("\n" + "=" * 60)
    print(f"  ✓ 控制指令端口 : {BRIDGE_CTRL_PORT}  (树莓派 → G1)")
    print(f"  ✓ 触觉反馈端口 : {BRIDGE_TOUCH_PORT}  (G1 → 树莓派)")
    print(f"  ✓ DDS 状态     : {'可用' if DDS_AVAILABLE else '模拟模式'}")
    print("=" * 60)
    print("\n等待树莓派连接…\n")

    try:
        while running:
            time.sleep(0.5)
    except KeyboardInterrupt:
        running = False

    print("hand_bridge.py 已退出")
