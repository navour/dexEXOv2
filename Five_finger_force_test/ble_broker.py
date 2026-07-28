#!/usr/bin/env python3
"""
BLE 数据代理 (Broker)
- 唯一连接蓝牙设备的进程
- 将 BLE 原始字节流广播给所有本地 TCP 客户端
- finger_force.py 和 ble_hand_control.py 均连接此 Broker，不再直连 BLE

启动方式:
    python3 ble_broker.py

客户端连接方式 (TCP localhost:9001)，接收到的数据与直连 BLE 完全相同（原始字节）
"""

import argparse
import asyncio
import socket
import threading
import time
import signal
import sys
from bleak import BleakClient

# ===== BLE 配置（默认值，可被命令行参数覆盖）=====
DEFAULT_DEVICE_ADDRESSES = [
    "F0:FD:45:02:85:B3",  # 设备1（右手）
    "F0:FD:45:02:67:3B",  # 设备2（左手）
]
TX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
RX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
BLE_KEEPALIVE_INTERVAL = 3.0
BLE_KEEPALIVE_DATA = b"PING"
BLE_RECONNECT_DELAY = 2.0

# ===== 命令行参数解析 =====
_parser = argparse.ArgumentParser(description="BLE Broker")
_parser.add_argument("--port", type=int, default=9001,
                     help="TCP 广播端口（默认 9001，左手实例用 9002）")
_parser.add_argument("--devices", nargs="+", metavar="MAC",
                     help="指定要连接的 BLE 设备地址列表（可多个），"
                          "不填则尝试所有默认地址")
_args = _parser.parse_args()

DEVICE_ADDRESSES: list[str] = _args.devices if _args.devices else DEFAULT_DEVICE_ADDRESSES

# ===== TCP Broker 配置 =====
BROKER_HOST = "127.0.0.1"
BROKER_PORT = _args.port

# ===== 全局状态 =====
clients: list = []          # 已连接的 TCP 客户端 socket
clients_lock = threading.Lock()
running = True


def broadcast(data: bytes) -> None:
    """向所有已连接的 TCP 客户端广播数据，自动清理断开的连接"""
    with clients_lock:
        dead = []
        for c in clients:
            try:
                c.sendall(data)
            except Exception:
                dead.append(c)
        for c in dead:
            clients.remove(c)
            try:
                c.close()
            except Exception:
                pass
            print(f"[Broker] 客户端断开，剩余 {len(clients)} 个")


def tcp_server_thread() -> None:
    """TCP 服务器线程：接受本地客户端连接"""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((BROKER_HOST, BROKER_PORT))
    srv.listen(10)
    srv.settimeout(1.0)
    while running:
        try:
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with clients_lock:
                clients.append(conn)
            print(f"[Broker] 新客户端: {addr}，当前 {len(clients)} 个")
        except socket.timeout:
            continue
        except Exception as e:
            if running:
                print(f"[Broker] TCP 服务器错误: {e}")
    srv.close()


async def ble_session(address: str) -> None:
    """连接一个 BLE 地址并持续转发数据"""
    print(f"[Broker] 尝试连接 BLE: {address}")
    disconnected_event = asyncio.Event()

    def on_disconnect(client) -> None:
        print(f"[Broker] BLE 断开: {address}")
        disconnected_event.set()

    async with BleakClient(address, disconnected_callback=on_disconnect, timeout=15.0) as client:
        if not client.is_connected:
            raise RuntimeError(f"无法连接: {address}")
        print(f"[Broker] BLE 已连接: {address}，正在转发数据...")

        def notification_handler(_, data: bytearray) -> None:
            # 直接广播原始字节，保持与直连 BLE 完全一致
            broadcast(bytes(data))

        await client.start_notify(RX_CHAR_UUID, notification_handler)

        try:
            last_keepalive = time.time()
            while running and not disconnected_event.is_set():
                now = time.time()
                if now - last_keepalive >= BLE_KEEPALIVE_INTERVAL:
                    try:
                        await client.write_gatt_char(TX_CHAR_UUID, BLE_KEEPALIVE_DATA, response=False)
                        last_keepalive = now
                    except Exception as e:
                        print(f"[Broker] 心跳失败: {e}")
                        break
                await asyncio.sleep(0.3)
        finally:
            try:
                if client.is_connected:
                    await client.stop_notify(RX_CHAR_UUID)
            except Exception:
                pass


async def ble_main() -> None:
    """BLE 主循环：轮流尝试多个地址，断开自动重连"""
    last_good_addr = None
    retry_count = 0

    while running:
        if last_good_addr:
            addr_order = [last_good_addr] + [a for a in DEVICE_ADDRESSES if a != last_good_addr]
        else:
            addr_order = list(DEVICE_ADDRESSES)

        for addr in addr_order:
            if not running:
                break
            try:
                await ble_session(addr)
                last_good_addr = addr
                retry_count = 0
                print(f"[Broker] BLE 会话结束，{BLE_RECONNECT_DELAY}s 后重连...")
                await asyncio.sleep(BLE_RECONNECT_DELAY)
                break
            except Exception as e:
                retry_count += 1
                delay = min(1.0 * retry_count, 10.0)
                print(f"[Broker] 连接失败 {addr}: {e}，{delay:.1f}s 后重试...")
                await asyncio.sleep(delay)


def signal_handler(signum, frame):
    global running
    print("\n[Broker] 正在停止...")
    running = False
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 启动 TCP 服务器线程
    t = threading.Thread(target=tcp_server_thread, daemon=True)
    t.start()

    print("=" * 50)
    print(" BLE Broker 启动")
    print(f" BLE 设备: {DEVICE_ADDRESSES}")
    print(f" TCP 广播端口: {BROKER_HOST}:{BROKER_PORT}")
    print("=" * 50)
    print(f"[Broker] TCP 监听 {BROKER_HOST}:{BROKER_PORT}，等待客户端连接...")

    asyncio.run(ble_main())
