#!/usr/bin/env python3
"""
both_hand_bridge.py — G1 上运行，单进程同时桥接左右两只灵巧手
  右手: DDS topic rt/inspire_hand/ctrl/r  touch/r   TCP ctrl:9100  touch:9101
  左手: DDS topic rt/inspire_hand/ctrl/l  touch/l   TCP ctrl:9102  touch:9103

启动:
  python3 both_hand_bridge.py [网卡名，默认eth0]

依赖（G1 aarch64 系统 Python，不使用 venv_x86）:
  source /opt/ros/foxy/setup.bash
  export PYTHONPATH=$PYTHONPATH:/home/unitree/workspace/zhw_workspace:\
/home/unitree/dexEXO/inspire_hand_ws/inspire_hand_sdk/inspire_sdkpy
"""

import sys
import json
import socket
import threading
import time
import signal

# ========== DDS 路径 ==========
sys.path.insert(0, '/home/unitree/workspace/zhw_workspace')
sys.path.insert(0, '/home/unitree/dexEXO/inspire_hand_ws/inspire_hand_sdk')

# ========== DDS 加载（复用 hand_bridge.py 的绕过方式）==========
try:
    from unitree_sdk2py.core.channel import (
        ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
    )
    import importlib.util, os as _os
    _dds_dir = '/home/unitree/dexEXO/inspire_hand_ws/inspire_hand_sdk/inspire_sdkpy/inspire_dds'
    def _load(name, path):
        spec = importlib.util.spec_from_file_location(name, path)
        mod  = importlib.util.module_from_spec(spec)
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
# ======================== 左右手配置 ==================================
# =====================================================================

BRIDGE_HOST = "0.0.0.0"

HAND_CONFIGS = {
    "r": {
        "label":       "右手",
        "ctrl_topic":  "rt/inspire_hand/ctrl/r",
        "touch_topic": "rt/inspire_hand/touch/r",
        "tcp_ctrl":    9100,
        "tcp_touch":   9101,
    },
    "l": {
        "label":       "左手",
        "ctrl_topic":  "rt/inspire_hand/ctrl/l",
        "touch_topic": "rt/inspire_hand/touch/l",
        "tcp_ctrl":    9102,
        "tcp_touch":   9103,
    },
}

# 触觉字段顺序（与 Force_handcontrol.py / hand_bridge.py 保持一致）
TOUCH_FIELDS = [
    "fingerfive_top_touch",   # 拇指
    "fingerfour_top_touch",   # 食指
    "fingerthree_top_touch",  # 中指
    "fingertwo_top_touch",    # 无名指
    "fingerone_top_touch",    # 小指
]

# =====================================================================
# ======================== 触觉标定（复用 hand_bridge.py 参数）========
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
# ======================== 单只手桥接类 ================================
# =====================================================================

class HandBridge:
    """单只手的 DDS ↔ TCP 桥接，左右手各实例化一个"""

    def __init__(self, side: str, cfg: dict):
        self.side  = side
        self.cfg   = cfg
        self.label = cfg["label"]

        self.latest_touch_forces = [0.0] * 6
        self.touch_clients: list = []
        self.touch_clients_lock  = threading.Lock()
        self.dds_pub = None

    # ── DDS 初始化（由主程序在 ChannelFactoryInitialize 后调用）──────
    def init_dds(self):
        if not DDS_AVAILABLE:
            return
        self.dds_pub = ChannelPublisher(self.cfg["ctrl_topic"], inspire_hand_ctrl)
        self.dds_pub.Init()
        print(f"[{self.label}][DDS] 发布器: {self.cfg['ctrl_topic']}")

        touch_sub = ChannelSubscriber(self.cfg["touch_topic"], inspire_hand_touch)
        touch_sub.Init(self._on_touch_msg, 10)
        print(f"[{self.label}][DDS] 订阅器: {self.cfg['touch_topic']}")

    # ── DDS 触觉回调 ─────────────────────────────────────────────────
    def _on_touch_msg(self, msg):
        try:
            forces = []
            for i, field in enumerate(TOUCH_FIELDS):
                raw_seq = getattr(msg, field, None)
                raw = float(max(raw_seq)) if raw_seq and len(raw_seq) > 0 else 0.0
                forces.append(raw_to_force_n(raw, is_thumb=(i == 0)))
            forces_6 = forces + [0.0]
            self.latest_touch_forces = forces_6

            payload = json.dumps({"type": "touch", "forces": forces_6}) + "\n"
            data = payload.encode("utf-8")
            with self.touch_clients_lock:
                dead = []
                for conn in self.touch_clients:
                    try:
                        conn.sendall(data)
                    except Exception:
                        dead.append(conn)
                for conn in dead:
                    self.touch_clients.remove(conn)
                    try:
                        conn.close()
                    except Exception:
                        pass
        except Exception as e:
            print(f"[{self.label}][DDS触觉] 解析错误: {e}")

    # ── TCP 控制服务：接收角度指令 ────────────────────────────────────
    def _handle_ctrl_client(self, conn: socket.socket, addr):
        print(f"[{self.label}][TCP控制] 客户端连接: {addr}")
        buf = ""
        try:
            while True:
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
                        if pkt.get("type") == "ctrl" and self.dds_pub is not None:
                            msg = inspire_hand_ctrl(
                                pos_set   = [0] * 6,
                                angle_set = tuple(int(a) for a in pkt.get("angle_set", [500]*6)),
                                force_set = list(pkt.get("force_set", [200]*6)),
                                speed_set = list(pkt.get("speed_set", [500]*6)),
                                mode      = pkt.get("mode", 1),
                            )
                            self.dds_pub.Write(msg)
                        elif not DDS_AVAILABLE:
                            print(f"[{self.label}][模拟] 收到指令: {pkt.get('angle_set')}")
                    except json.JSONDecodeError:
                        pass
                    except Exception as e:
                        print(f"[{self.label}][TCP控制] 处理错误: {e}")
        except Exception as e:
            print(f"[{self.label}][TCP控制] 连接错误: {e}")
        finally:
            conn.close()
            print(f"[{self.label}][TCP控制] 客户端断开: {addr}")

    def _ctrl_server(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((BRIDGE_HOST, self.cfg["tcp_ctrl"]))
        srv.listen(5)
        srv.settimeout(1.0)
        print(f"[{self.label}][TCP控制] 监听 {BRIDGE_HOST}:{self.cfg['tcp_ctrl']}")
        while running:
            try:
                conn, addr = srv.accept()
                threading.Thread(target=self._handle_ctrl_client,
                                 args=(conn, addr), daemon=True).start()
            except socket.timeout:
                continue
            except Exception as e:
                if running:
                    print(f"[{self.label}][TCP控制] 接受连接错误: {e}")
        srv.close()

    # ── TCP 触觉服务：向树莓派推送触觉 ───────────────────────────────
    def _handle_touch_client(self, conn: socket.socket, addr):
        print(f"[{self.label}][TCP触觉] 客户端连接: {addr}")
        with self.touch_clients_lock:
            self.touch_clients.append(conn)
        try:
            while running:
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
            with self.touch_clients_lock:
                if conn in self.touch_clients:
                    self.touch_clients.remove(conn)
            conn.close()
            print(f"[{self.label}][TCP触觉] 客户端断开: {addr}")

    def _touch_server(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((BRIDGE_HOST, self.cfg["tcp_touch"]))
        srv.listen(5)
        srv.settimeout(1.0)
        print(f"[{self.label}][TCP触觉] 监听 {BRIDGE_HOST}:{self.cfg['tcp_touch']}")
        while running:
            try:
                conn, addr = srv.accept()
                threading.Thread(target=self._handle_touch_client,
                                 args=(conn, addr), daemon=True).start()
            except socket.timeout:
                continue
            except Exception as e:
                if running:
                    print(f"[{self.label}][TCP触觉] 接受连接错误: {e}")
        srv.close()

    # ── 启动所有线程 ─────────────────────────────────────────────────
    def start(self):
        threading.Thread(target=self._ctrl_server,  daemon=True).start()
        threading.Thread(target=self._touch_server, daemon=True).start()
        print(f"[{self.label}] TCP 服务已启动  "
              f"ctrl:{self.cfg['tcp_ctrl']}  touch:{self.cfg['tcp_touch']}")


# =====================================================================
# ======================== 程序入口 ====================================
# =====================================================================

running = True

def signal_handler(sig, frame):
    global running
    print("\n接收到 Ctrl+C，正在停止…")
    running = False

signal.signal(signal.SIGINT, signal_handler)


if __name__ == "__main__":
    print("=" * 60)
    print("   both_hand_bridge.py  左右双手 TCP ↔ DDS 桥接")
    print("   运行在宇树 G1 上")
    print("=" * 60)

    nic = sys.argv[1] if len(sys.argv) > 1 else "eth0"

    # DDS 全局初始化（只能调一次）
    if DDS_AVAILABLE:
        print(f"[DDS] 初始化，网卡={nic}")
        ChannelFactoryInitialize(0, nic)

    # 实例化左右手桥接
    right = HandBridge("r", HAND_CONFIGS["r"])
    left  = HandBridge("l", HAND_CONFIGS["l"])

    # DDS 发布/订阅
    right.init_dds()
    left.init_dds()

    # 启动 TCP 服务
    right.start()
    left.start()

    print("\n" + "=" * 60)
    print(f"  ✓ 右手控制端口 : {HAND_CONFIGS['r']['tcp_ctrl']}   (树莓派 → G1)")
    print(f"  ✓ 右手触觉端口 : {HAND_CONFIGS['r']['tcp_touch']}   (G1 → 树莓派)")
    print(f"  ✓ 左手控制端口 : {HAND_CONFIGS['l']['tcp_ctrl']}   (树莓派 → G1)")
    print(f"  ✓ 左手触觉端口 : {HAND_CONFIGS['l']['tcp_touch']}   (G1 → 树莓派)")
    print(f"  ✓ DDS 状态     : {'可用' if DDS_AVAILABLE else '模拟模式'}")
    print("=" * 60)
    print("\n等待树莓派连接…\n")

    try:
        while running:
            time.sleep(0.5)
    except KeyboardInterrupt:
        running = False

    print("both_hand_bridge.py 已退出")
