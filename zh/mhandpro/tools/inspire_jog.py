#!/usr/bin/env python3
"""因时灵巧手安全点动工具：单步小变化、持续心跳、未提交时自动回原基准。"""

import argparse
import json
import socket
import threading
import time


NAMES = ["小指", "无名指", "中指", "食指", "拇指弯曲", "拇指对掌"]


def six_ints(text: str) -> list[int]:
    try:
        values = [int(part.strip()) for part in text.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("必须是逗号分隔的6个整数") from exc
    if len(values) != 6 or any(value < 0 or value > 1000 for value in values):
        raise argparse.ArgumentTypeError("必须是0~1000内的6个整数")
    return values


def ctrl_packet(angles: list[int]) -> bytes:
    return (json.dumps({
        "type": "ctrl", "angle_set": angles, "mode": 1,
        "force_set": [100] * 6, "speed_set": [100] * 6,
    }) + "\n").encode()


class Jogger:
    def __init__(self, host: str, port: int, base: list[int], step: int):
        self.sock = socket.create_connection((host, port), timeout=3.0)
        self.sock.settimeout(None)
        self.original = list(base)
        self.current = list(base)
        self.step = step
        self.selected = 0
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.committed = False
        self.error: Exception | None = None
        self.heartbeat = threading.Thread(target=self._heartbeat, daemon=True)

    def _send_current(self) -> None:
        with self.lock:
            self.sock.sendall(ctrl_packet(self.current))

    def _heartbeat(self) -> None:
        try:
            while not self.stop.wait(0.1):
                self._send_current()
        except Exception as exc:
            self.error = exc
            self.stop.set()

    def start(self) -> None:
        self._send_current()
        self.heartbeat.start()

    def adjust(self, direction: int) -> None:
        if self.stop.is_set():
            raise RuntimeError(f"心跳线程已停止: {self.error}")
        with self.lock:
            target = self.current[self.selected] + direction * self.step
            if not 0 <= target <= 1000:
                raise ValueError("目标超出0~1000，已拒绝")
            self.current[self.selected] = target
            self.sock.sendall(ctrl_packet(self.current))

    def reset_gradually(self) -> None:
        print("正在分步返回原始安全基准...")
        while self.current != self.original:
            with self.lock:
                for i in range(6):
                    delta = self.original[i] - self.current[i]
                    if delta:
                        self.current[i] += max(-self.step, min(self.step, delta))
                self.sock.sendall(ctrl_packet(self.current))
            time.sleep(0.1)
        print("已返回原始安全基准。")

    def commit(self) -> None:
        with self.lock:
            packet = {"type": "set_safe", "angles": self.current, "token": "COMMIT_SAFE_OPEN"}
            self.sock.sendall((json.dumps(packet) + "\n").encode())
        # 桥接会返回确认。暂停心跳可避免接收与发送输出混淆。
        self.sock.settimeout(2.0)
        response = self.sock.recv(4096).decode()
        self.sock.settimeout(None)
        if '"safe_committed"' not in response:
            raise RuntimeError(f"桥接未确认提交: {response!r}")
        self.original = list(self.current)
        self.committed = True
        print(f"新安全张手基准已提交: {self.current}")

    def close(self) -> None:
        try:
            if not self.committed:
                self.reset_gradually()
        finally:
            self.stop.set()
            self.heartbeat.join(timeout=1.0)
            self.sock.close()


def print_help() -> None:
    print("""
命令：
  select 0..5   选择通道（0小指 1无名指 2中指 3食指 4拇指弯曲 5拇指对掌）
  +             当前通道增加一步（四指/拇弯伸直，拇对掌外展）
  -             当前通道减少一步（四指/拇弯弯曲，拇对掌对掌）
  show          显示当前六路命令
  reset         分步返回启动时的原基准
  commit        输入二次确认后，将当前姿态保存为新安全张手基准
  quit          未提交时先返回原基准，再退出
""")


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspire左手安全点动与张手基准建立")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9102)
    parser.add_argument("--base", type=six_ints, required=True)
    parser.add_argument("--step", type=int, default=10)
    args = parser.parse_args()
    if not 1 <= args.step <= 20:
        parser.error("--step必须在1~20内")

    jogger = Jogger(args.host, args.port, args.base, args.step)
    try:
        jogger.start()
        print_help()
        print(f"原始安全基准: {jogger.original}")
        while True:
            try:
                command = input(f"点动[通道{jogger.selected} {NAMES[jogger.selected]}]> ").strip().lower()
                if command.startswith("select "):
                    parts = command.split()
                    if len(parts) != 2:
                        raise ValueError("用法：select 0..5")
                    selected = int(parts[1])
                    if selected not in range(6):
                        raise ValueError("通道必须在0~5内")
                    jogger.selected = selected
                elif command == "+":
                    jogger.adjust(+1); print(jogger.current)
                elif command == "-":
                    jogger.adjust(-1); print(jogger.current)
                elif command == "show":
                    print(jogger.current)
                elif command == "reset":
                    jogger.reset_gradually()
                elif command == "commit":
                    print(f"当前候选张手值: {jogger.current}")
                    confirmation = input("确认姿态自然、未达机械极限，输入 COMMIT 保存: ")
                    if confirmation == "COMMIT":
                        jogger.commit()
                    else:
                        print("已取消提交。")
                elif command in ("quit", "q"):
                    break
                elif command in ("help", "h"):
                    print_help()
                elif command:
                    print("未知命令，输入 help 查看帮助。")
            except (ValueError, IndexError) as exc:
                # 用户输入或0~1000边界错误不应关闭连接；保持当前安全命令并继续点动。
                print(f"点动被拒绝：{exc}。灵巧手保持当前姿态，可输入 +、reset 或 show。")
    except (KeyboardInterrupt, EOFError):
        print("\n收到退出信号。")
    except Exception as exc:
        print(f"点动错误: {exc}")
    finally:
        jogger.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
