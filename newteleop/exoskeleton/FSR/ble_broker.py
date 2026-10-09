#!/usr/bin/env python3
"""独占一个 STM32 BLE 模块，并把原始通知广播到本机 TCP。"""

from __future__ import annotations

import argparse
import asyncio
import errno
import signal
import socket
import sys
import threading
import time

HAND_PROFILES = {
    "right": ("F0:FD:45:02:85:B3", 9001),
    "left": ("F0:FD:45:02:67:3B", 9002),
}
HAND_CHOICES = (*HAND_PROFILES, "both")
TX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
RX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
PORT_IN_USE_EXIT_CODE = 3


class BrokerPortInUseError(RuntimeError):
    """The local TCP endpoint is already owned by another process."""

    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        super().__init__(
            f"TCP端口 {host}:{port} 已被占用；已有 BLE broker 正在运行，"
            "或该端口被其他程序占用。请勿重复启动。"
        )


class Broker:
    def __init__(self, address: str, host: str, port: int) -> None:
        self.address = address
        self.host = host
        self.port = port
        self.running = True
        self.clients: list[socket.socket] = []
        self.lock = threading.Lock()
        self.server: socket.socket | None = None

    def broadcast(self, data: bytes) -> None:
        with self.lock:
            dead = []
            for client in self.clients:
                try:
                    client.sendall(data)
                except OSError:
                    dead.append(client)
            for client in dead:
                self.clients.remove(client)
                client.close()
            if dead:
                print(f"[TCP] 清理{len(dead)}个客户端，剩余{len(self.clients)}个")

    def prepare_tcp(self) -> None:
        """在启动BLE前占用监听端口，让端口冲突成为启动失败。"""
        if self.server is not None:
            return
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.host, self.port))
            server.listen(10)
            server.settimeout(1.0)
        except OSError as exc:
            server.close()
            if exc.errno == errno.EADDRINUSE:
                raise BrokerPortInUseError(self.host, self.port) from exc
            raise
        self.server = server

    def serve_tcp(self) -> None:
        server = self.server
        if server is None:
            raise RuntimeError("TCP监听端口尚未准备")
        try:
            print(f"[TCP] 监听 {self.host}:{self.port}")
            while self.running:
                try:
                    client, address = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if not self.running:
                        break
                    raise
                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self.lock:
                    self.clients.append(client)
                print(f"[TCP] 客户端 {address} 已连接，共{len(self.clients)}个")
        finally:
            server.close()
            if self.server is server:
                self.server = None

    async def session(self, connect_lock: asyncio.Lock) -> None:
        disconnected = asyncio.Event()

        from bleak import BleakClient

        def on_disconnect(_client: object) -> None:
            print(f"[BLE] 已断开 {self.address}")
            disconnected.set()

        client = BleakClient(
            self.address, disconnected_callback=on_disconnect, timeout=15.0)
        try:
            # BlueZ 在同一个适配器上并发执行两个 Device1.Connect 容易留下
            # InProgress；只串行化连接阶段，连接成功后的通知仍然双手并行。
            async with connect_lock:
                if not self.running:
                    return
                print(f"[BLE] 正在连接 {self.address}")
                await client.connect()
            if not client.is_connected:
                raise RuntimeError("连接建立后状态仍为断开")
            print(f"[BLE] 已连接 {self.address}，开始转发通知")
            await client.start_notify(RX_UUID, lambda _sender, data: self.broadcast(bytes(data)))
            last_ping = time.monotonic()
            while self.running and not disconnected.is_set():
                if time.monotonic() - last_ping >= 3.0:
                    await client.write_gatt_char(TX_UUID, b"PING", response=False)
                    last_ping = time.monotonic()
                await asyncio.sleep(0.2)
        finally:
            if client.is_connected:
                await client.disconnect()

    async def run(self, connect_lock: asyncio.Lock) -> None:
        if self.server is None:
            raise RuntimeError("必须在连接BLE前准备TCP监听端口")
        threading.Thread(target=self.serve_tcp, daemon=True).start()
        while self.running:
            retry_delay = 2.0
            try:
                await self.session(connect_lock)
            except Exception as exc:
                if self.running:
                    if "InProgress" in str(exc):
                        retry_delay = 10.0
                    print(f"[BLE] {exc}；{retry_delay:g}秒后重连")
            if self.running:
                await asyncio.sleep(retry_delay)

    def stop(self) -> None:
        self.running = False
        server = self.server
        self.server = None
        if server is not None:
            server.close()
        with self.lock:
            for client in self.clients:
                client.close()
            self.clients.clear()


async def run_brokers(brokers: list[Broker]) -> None:
    """共用一个 asyncio 事件循环连接左右两块BLE。"""
    try:
        # 必须先成功占用全部端口；任一失败时不能连接任何一只BLE。
        for broker in brokers:
            broker.prepare_tcp()
    except Exception:
        for broker in brokers:
            broker.stop()
        raise
    connect_lock = asyncio.Lock()
    await asyncio.gather(*(broker.run(connect_lock) for broker in brokers))


def main() -> int:
    parser = argparse.ArgumentParser(description="左右手 STM32/FSR BLE -> 本机TCP代理")
    parser.add_argument("--hand", choices=HAND_CHOICES, required=True,
                        help="right/left连单手，both在一个进程中连双手")
    parser.add_argument("--address", help="单手模式覆盖默认MAC")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, help="单手模式覆盖默认TCP端口")
    parser.add_argument("--right-address", help="both模式覆盖右手MAC")
    parser.add_argument("--left-address", help="both模式覆盖左手MAC")
    parser.add_argument("--right-port", type=int, help="both模式覆盖右手端口，默认9001")
    parser.add_argument("--left-port", type=int, help="both模式覆盖左手端口，默认9002")
    args = parser.parse_args()

    if args.hand == "both":
        if args.address is not None or args.port is not None:
            parser.error("--hand both时请用--right/left-address或--right/left-port")
        brokers = []
        for side in ("right", "left"):
            default_address, default_port = HAND_PROFILES[side]
            address = getattr(args, f"{side}_address") or default_address
            port = getattr(args, f"{side}_port") or default_port
            brokers.append(Broker(address, args.host, port))
    else:
        default_address, default_port = HAND_PROFILES[args.hand]
        side_address = getattr(args, f"{args.hand}_address")
        side_port = getattr(args, f"{args.hand}_port")
        brokers = [Broker(
            args.address or side_address or default_address,
            args.host,
            args.port or side_port or default_port,
        )]

    for side, broker in zip(
            ("right", "left") if args.hand == "both" else (args.hand,), brokers):
        print(f"手={side} BLE={broker.address} TCP={broker.host}:{broker.port}")

    def stop(_signum: int, _frame: object) -> None:
        print("\n正在停止...")
        for broker in brokers:
            broker.stop()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        asyncio.run(run_brokers(brokers))
    except BrokerPortInUseError as exc:
        print(f"[启动失败] {exc}", file=sys.stderr)
        return PORT_IN_USE_EXIT_CODE
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
