#!/usr/bin/env python3
"""电脑直连因时灵巧手时使用的 JSON/TCP -> Modbus TCP 安全桥接。

默认为只读模式；必须同时传入 --enable-write 和 --safe-open 才允许写角度寄存器。
"""

import argparse
import json
import signal
import socket
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Optional

from pymodbus.client import ModbusTcpClient


ANGLE_SET_REGISTER = 1486
ANGLE_ACT_REGISTER = 1546
FORCE_ACT_REGISTER = 1582
CHANNEL_NAMES = ["小指", "无名指", "中指", "食指", "拇指弯曲", "拇指对掌"]
TOP_TOUCH_CHANNELS = [
    ("拇指", 4498, True),
    ("食指", 4128, False),
    ("中指", 3758, False),
    ("无名指", 3388, False),
    ("小指", 3018, False),
]
TOP_TOUCH_COUNT = 96
FORCE_K = 0.00292650244415058227
FORCE_B = -0.6037947156125716
THUMB_FORCE_K = 0.004420145759358057
THUMB_FORCE_B = -1.0701492398616255


def six_ints(text: str) -> list[int]:
    try:
        values = [int(part.strip()) for part in text.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("必须是逗号分隔的6个整数") from exc
    if len(values) != 6:
        raise argparse.ArgumentTypeError("必须恰好有6个整数")
    return values


def signed_words(registers: list[int]) -> list[int]:
    packed = struct.pack(">" + "H" * len(registers), *registers)
    return list(struct.unpack(">" + "h" * len(registers), packed))


def touch_raw_to_force_n(raw: int, is_thumb: bool) -> float:
    force = ((THUMB_FORCE_K * raw + THUMB_FORCE_B) if is_thumb
             else (FORCE_K * raw + FORCE_B))
    return max(0.0, min(force, 10.0))


class Bridge:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.running = True
        self.client = ModbusTcpClient(args.hand_ip, port=args.hand_port, timeout=2.0)
        self.server: Optional[socket.socket] = None
        self.last_written: Optional[list[int]] = None
        self.safe_sent = False
        self.force_publisher = ForcePublisher(args.force_listen_host, args.force_listen_port)
        self.force_seq = 0
        self.next_force_read = 0.0
        self.next_touch_read = 0.0
        self.touch_raw_max = [0] * 5
        self.touch_top5_mean = [0.0] * 5
        self.touch_force_n = [0.0] * 5
        self.touch_valid = False
        self.touch_mono_ms = 0

    def read_angles(self) -> list[int]:
        response = self.client.read_holding_registers(
            address=ANGLE_ACT_REGISTER, count=6, slave=self.args.device_id)
        if response.isError() or not hasattr(response, "registers"):
            raise RuntimeError(f"读取 ANGLE_ACT 失败: {response}")
        return signed_words(list(response.registers))

    def read_force(self) -> list[int]:
        response = self.client.read_holding_registers(
            address=FORCE_ACT_REGISTER, count=6, slave=self.args.device_id)
        if response.isError() or not hasattr(response, "registers"):
            raise RuntimeError(f"读取 FORCE_ACT 失败: {response}")
        return signed_words(list(response.registers))

    def read_top_touch(self) -> tuple[list[int], list[float], list[float]]:
        maxima = []
        top5_means = []
        forces = []
        for name, address, is_thumb in TOP_TOUCH_CHANNELS:
            response = self.client.read_holding_registers(
                address=address, count=TOP_TOUCH_COUNT, slave=self.args.device_id)
            if response.isError() or not hasattr(response, "registers"):
                raise RuntimeError(f"读取{name} top_touch失败: {response}")
            values = signed_words(list(response.registers))
            if len(values) != TOP_TOUCH_COUNT:
                raise RuntimeError(f"读取{name} top_touch长度={len(values)}")
            maximum = max(values)
            maxima.append(maximum)
            top5_means.append(sum(sorted(values, reverse=True)[:5]) / 5.0)
            forces.append(touch_raw_to_force_n(maximum, is_thumb))
        return maxima, top5_means, forces

    def publish_force_if_due(self) -> None:
        now = time.monotonic()
        if now < self.next_force_read:
            return
        self.next_force_read = now + 1.0 / self.args.force_hz
        self.force_seq += 1
        if now >= self.next_touch_read:
            self.next_touch_read = now + 1.0 / self.args.touch_hz
            try:
                (self.touch_raw_max, self.touch_top5_mean,
                 self.touch_force_n) = self.read_top_touch()
                self.touch_valid = True
                self.touch_mono_ms = int(time.monotonic() * 1000)
            except Exception as exc:
                self.touch_valid = False
                print(f"[top_touch读取失败] {exc}", file=sys.stderr)
        packet = {
            "type": "inspire_feedback",
            "seq": self.force_seq,
            "mono_ms": int(now * 1000),
            "force_act": [0] * 6,
            "top_touch_order": [item[0] for item in TOP_TOUCH_CHANNELS],
            "top_touch_raw_max": self.touch_raw_max,
            "top_touch_top5_mean": self.touch_top5_mean,
            "top_touch_force_n": self.touch_force_n,
            "touch_valid": self.touch_valid,
            "touch_mono_ms": self.touch_mono_ms,
            "valid": False,
        }
        try:
            packet["force_act"] = self.read_force()
            packet["valid"] = True
        except Exception as exc:
            packet["error"] = str(exc)
        self.force_publisher.broadcast(packet)

    def write_angles(self, angles: list[int], *, safety_return: bool = False) -> None:
        if not self.args.enable_write:
            print(f"[只读模式] 收到但未写入: {angles}")
            return
        if len(angles) != 6 or any(value < 0 or value > 1000 for value in angles):
            raise ValueError("六路角度必须都在0~1000内")
        if not safety_return and self.last_written is not None:
            changes = [abs(a-b) for a, b in zip(angles, self.last_written)]
            if max(changes) > self.args.max_step:
                raise ValueError(f"拒绝单次跳变 {max(changes)} tick，上限为 {self.args.max_step}")
        response = self.client.write_registers(
            address=ANGLE_SET_REGISTER, values=angles, slave=self.args.device_id)
        if response.isError():
            raise RuntimeError(f"写入 ANGLE_SET 失败: {response}")
        self.last_written = list(angles)
        self.safe_sent = safety_return

    def return_safe(self, reason: str) -> None:
        if not self.args.enable_write or self.safe_sent:
            return
        try:
            self.write_angles(self.args.safe_open, safety_return=True)
            print(f"[安全返回] {reason}: {self.args.safe_open}")
        except Exception as exc:
            print(f"[严重] 安全返回写入失败: {exc}", file=sys.stderr)

    def commit_safe_open(self, angles: list[int], token: str) -> None:
        if not self.args.enable_write or not self.args.allow_safe_update:
            raise ValueError("桥接未启用安全基准更新")
        if token != "COMMIT_SAFE_OPEN":
            raise ValueError("安全基准提交令牌错误")
        if len(angles) != 6 or any(value < 0 or value > 1000 for value in angles):
            raise ValueError("提交值必须是0~1000内的6个整数")
        if self.last_written is None or angles != self.last_written:
            raise ValueError("提交值必须与最后一条已写入命令完全相同")
        self.args.safe_open = list(angles)
        self.safe_sent = False
        path = Path(self.args.safe_file)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(",".join(str(value) for value in angles) + "\n", encoding="utf-8")
        print(f"[安全基准已更新] {angles}")
        print(f"[已保存] {path}")

    def handle_connection(self, conn: socket.socket, address) -> None:
        print(f"JSON客户端已连接: {address}")
        conn.settimeout(min(0.05, 1.0 / self.args.force_hz))
        buffer = ""
        last_packet = time.monotonic()
        self.safe_sent = False
        try:
            while self.running:
                self.publish_force_if_due()
                try:
                    chunk = conn.recv(4096)
                    if not chunk:
                        self.return_safe("客户端断开")
                        break
                    buffer += chunk.decode("utf-8", errors="strict")
                    last_packet = time.monotonic()
                except socket.timeout:
                    if time.monotonic() - last_packet > self.args.watchdog_ms / 1000.0:
                        self.return_safe(f"命令超时>{self.args.watchdog_ms}ms")
                        break
                    continue

                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    if not line.strip():
                        continue
                    packet = json.loads(line)
                    if packet.get("type") == "set_safe":
                        angles = [int(value) for value in packet.get("angles", [])]
                        self.commit_safe_open(angles, str(packet.get("token", "")))
                        conn.sendall((json.dumps({"type": "safe_committed", "angles": angles}) + "\n").encode())
                        continue
                    if packet.get("type") != "ctrl":
                        continue
                    # 初期联调只允许模式1的角度写入，不写力和速度寄存器。
                    if int(packet.get("mode", 0)) & 0b0001 == 0:
                        raise ValueError("初期桥接只接受角度模式")
                    angles = [int(value) for value in packet.get("angle_set", [])]
                    self.write_angles(angles)
        except Exception as exc:
            print(f"[客户端处理错误] {exc}", file=sys.stderr)
            self.return_safe("数据或写入异常")
        finally:
            conn.close()
            print("JSON客户端已关闭。")

    def run(self) -> None:
        print("========== 独立 Inspire 安全桥接 ==========")
        print(f"灵巧手: {self.args.hand_ip}:{self.args.hand_port}, Device ID={self.args.device_id}")
        print(f"JSON监听: {self.args.listen_host}:{self.args.listen_port}")
        print(f"FORCE_ACT发布: {self.args.force_listen_host}:{self.args.force_listen_port}, "
              f"{self.args.force_hz:.1f} Hz")
        print(f"五指top_touch采集: {self.args.touch_hz:.1f} Hz, "
              "顺序=[拇指,食指,中指,无名指,小指]")
        print(f"写入状态: {'已启用' if self.args.enable_write else '只读（不会运动）'}")
        if not self.client.connect():
            raise RuntimeError("Modbus TCP连接失败")
        self.force_publisher.start()
        actual = self.read_angles()
        self.last_written = list(self.args.safe_open) if self.args.enable_write else None
        print(f"当前实际角度: {actual}")
        print("通道顺序: " + ", ".join(f"{i}={name}" for i, name in enumerate(CHANNEL_NAMES)))
        if self.args.enable_write:
            print(f"安全返回值: {self.args.safe_open}")
            print(f"单次最大变化: {self.args.max_step} tick，看门狗: {self.args.watchdog_ms} ms")
            print(f"安全基准在线更新: {'允许' if self.args.allow_safe_update else '禁止'}")

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.args.listen_host, self.args.listen_port))
        self.server.listen(1)
        self.server.settimeout(min(0.05, 1.0 / self.args.force_hz))
        print("桥接已启动，按 Ctrl+C 退出。")
        while self.running:
            self.publish_force_if_due()
            try:
                conn, address = self.server.accept()
                self.handle_connection(conn, address)
            except socket.timeout:
                continue

    def close(self) -> None:
        self.running = False
        self.return_safe("桥接退出")
        if self.server:
            self.server.close()
        self.force_publisher.close()
        self.client.close()


class ForcePublisher:
    """将桥接器读到的 FORCE_ACT 和五指top_touch广播给本机客户端。"""

    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.running = True
        self.server: Optional[socket.socket] = None
        self.clients: list[socket.socket] = []
        self.lock = threading.Lock()

    def start(self) -> None:
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port))
        self.server.listen(4)
        self.server.settimeout(0.5)
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _accept_loop(self) -> None:
        while self.running and self.server is not None:
            try:
                conn, address = self.server.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self.lock:
                    self.clients.append(conn)
                print(f"Inspire反馈客户端已连接: {address}")
            except socket.timeout:
                continue
            except OSError:
                break

    def broadcast(self, packet: dict) -> None:
        data = (json.dumps(packet, ensure_ascii=False, separators=(",", ":")) + "\n").encode()
        with self.lock:
            dead = []
            for conn in self.clients:
                try:
                    conn.sendall(data)
                except OSError:
                    dead.append(conn)
            for conn in dead:
                self.clients.remove(conn)
                conn.close()

    def close(self) -> None:
        self.running = False
        if self.server:
            self.server.close()
        with self.lock:
            for conn in self.clients:
                conn.close()
            self.clients.clear()


def main() -> int:
    parser = argparse.ArgumentParser(description="独立电脑 JSON/TCP -> Inspire Modbus TCP 桥接")
    parser.add_argument("--hand-ip", default="192.168.123.210")
    parser.add_argument("--hand-port", type=int, default=6000)
    parser.add_argument("--device-id", type=int, default=1)
    parser.add_argument("--listen-host", default="127.0.0.1")
    parser.add_argument("--listen-port", type=int, default=9102)
    parser.add_argument("--force-listen-host", default="127.0.0.1")
    parser.add_argument("--force-listen-port", type=int, default=9202)
    parser.add_argument("--force-hz", type=float, default=20.0)
    parser.add_argument("--touch-hz", type=float, default=5.0,
                        help="五指top_touch阵列读取频率，1..10Hz")
    parser.add_argument("--enable-write", action="store_true", help="明确允许写角度寄存器")
    parser.add_argument("--safe-open", type=six_ints,
                        help="客户端断开/超时时写入的6路安全值")
    parser.add_argument("--max-step", type=int, default=50, help="相邻JSON指令单通道最大变化")
    parser.add_argument("--watchdog-ms", type=int, default=500)
    parser.add_argument("--allow-safe-update", action="store_true",
                        help="允许点动工具显式提交新的安全张手基准")
    parser.add_argument("--safe-file", default="config/committed_left_safe_open.txt",
                        help="成功提交后保存六路值的文件")
    args = parser.parse_args()
    if args.enable_write and args.safe_open is None:
        parser.error("--enable-write 必须同时指定 --safe-open")
    if not args.enable_write and args.safe_open is not None:
        parser.error("只读模式不需要 --safe-open")
    if args.max_step < 1 or args.max_step > 50:
        parser.error("--max-step 必须在1~50之间")
    if args.force_hz < 1 or args.force_hz > 50:
        parser.error("--force-hz 必须在1~50之间")
    if args.touch_hz < 1 or args.touch_hz > 10:
        parser.error("--touch-hz 必须在1~10之间")
    bridge = Bridge(args)

    def stop_handler(_signum, _frame):
        bridge.running = False

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    try:
        bridge.run()
    except Exception as exc:
        print(f"桥接启动失败: {exc}", file=sys.stderr)
        return 1
    finally:
        bridge.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
