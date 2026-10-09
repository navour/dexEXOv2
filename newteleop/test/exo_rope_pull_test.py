#!/usr/bin/env python3
"""XL330-M288 单舵机绳端拉力测试；默认只读，--enable-write 才输出。

运行后保留电流模式0、目标电流0、扭矩关闭，不自动回卷绳索。
使用说明见 exo_rope_pull_test.md。
"""

import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import signal
import sys
import time


def signed(value, bits):
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--device', default='/dev/serial0')
    p.add_argument('--baudrate', type=int, default=1_000_000)
    p.add_argument('--id', type=int, required=True, choices=range(1, 11))
    p.add_argument('--current', required=True, help='有符号电流raw；max使用当前硬件上限，反向用--current=-max')
    p.add_argument('--current-limit', type=int, help='本轮临时Current Limit(1~1750raw)，卸力后恢复原值；写EEPROM')
    p.add_argument('--min-voltage', type=float, default=3.7, help='电压低于此值停机，默认3.7V')
    p.add_argument('--enable-write', action='store_true', help='允许切到模式0并输出电流')
    p.add_argument('--duration', type=float, default=5, help='保持秒数0.1~120；0表示不限时，Ctrl+C停止；保护仍有效')
    p.add_argument('--ramp', type=float, default=2, help='从0升到目标的秒数，1~10')
    p.add_argument('--max-travel', type=int, default=200, help='相对起点最大位移tick，1~6000')
    p.add_argument('--max-temperature', type=int, default=55, help='停机温度°C，30~65')
    p.add_argument('--actual-current-limit', type=int, help='回读电流停机阈值，默认目标绝对值+30')
    p.add_argument('--csv', type=Path, help='遥测CSV路径，必须不存在；默认脚本所在目录newteleop/test下时间戳文件')
    p.add_argument('--record-force', action='store_true', help='卸力后手工输入本次稳定拉力N')
    p.add_argument('--radius-mm', type=float, help='有效卷线半径mm，用实测拉力估算卷线轴扭矩')
    a = p.parse_args(argv)
    # 软件只限制器件寄存器范围，实际可用值还受舵机现有Current Limit限制。
    if a.current not in ('max', '-max'):
        try:
            a.current = int(a.current)
        except ValueError:
            p.error('--current必须是整数、max或-max')
        if not 1 <= abs(a.current) <= 1750:
            p.error('--current绝对值必须为1~1750；从30开始逐级测试')
    if a.duration != 0 and not 0.1 <= a.duration <= 120:
        p.error('--duration必须为0（不限时）或0.1~120秒')
    for name, low, high in [('ramp', 1, 10),
                            ('max_travel', 1, 6000), ('max_temperature', 30, 65)]:
        if not low <= getattr(a, name) <= high:
            p.error(f'--{name.replace("_", "-")}必须为{low}~{high}')
    if a.baudrate <= 0:
        p.error('--baudrate必须为正数')
    if a.current_limit is not None and not 1 <= a.current_limit <= 1750:
        p.error('--current-limit必须为1~1750')
    if not 3.7 <= a.min_voltage <= 6.0:
        p.error('--min-voltage必须为3.7~6.0V')
    if a.current_limit is not None and isinstance(a.current, int) and abs(a.current) > a.current_limit:
        p.error('--current不能超过--current-limit')
    if a.actual_current_limit is None and isinstance(a.current, int):
        a.actual_current_limit = abs(a.current) + 30
    minimum = abs(a.current) if isinstance(a.current, int) else 0
    if a.actual_current_limit is not None and not minimum < a.actual_current_limit <= 2000:
        p.error('--actual-current-limit必须大于目标电流绝对值且不超过2000')
    if a.radius_mm is not None and (not math.isfinite(a.radius_mm) or a.radius_mm <= 0):
        p.error('--radius-mm必须是有限正数')
    return a


class Servo:
    def __init__(self, packet, port, sid):
        self.packet, self.port, self.sid = packet, port, sid

    def check(self, result, error):
        if result != 0:
            raise RuntimeError(self.packet.getTxRxResult(result))
        if error:
            raise RuntimeError(self.packet.getRxPacketError(error))

    def read(self, address, size=1):
        value, result, error = getattr(self.packet, f'read{size}ByteTxRx')(
            self.port, self.sid, address)
        self.check(result, error)
        return value

    def write(self, address, value, size=1):
        result, error = getattr(self.packet, f'write{size}ByteTxRx')(
            self.port, self.sid, address, value & ((1 << (8 * size)) - 1))
        self.check(result, error)

    def sample(self):
        return (signed(self.read(132, 4), 32), signed(self.read(126, 2), 16),
                self.read(146), self.read(144, 2) / 10, self.read(70))


def check_sample(a, sample, start):
    position, current, temperature, voltage, error = sample
    if error:
        raise RuntimeError(f'硬件错误0x{error:02X}')
    if voltage < a.min_voltage:
        raise RuntimeError(f'电压过低：{voltage:.1f}V < {a.min_voltage:.1f}V')
    if temperature >= a.max_temperature:
        raise RuntimeError(f'温度达到停机阈值：{temperature}°C')
    if abs(current) >= a.actual_current_limit:
        raise RuntimeError(f'实际电流达到停机阈值：{current}raw')
    if abs(position - start) >= a.max_travel:
        raise RuntimeError(f'位移达到停机阈值：{position - start:+d}tick；'
                           '这是行程保护，不是保持时间到期。检查固定端、松绳和剩余机械行程')


def release(servo):
    """独立尝试清零和关扭矩；一次写失败不能跳过关扭矩。"""
    errors = []
    for address, value, size in [(102, 0, 2), (64, 0, 1)]:
        try:
            servo.write(address, value, size)
        except Exception as exc:
            errors.append(f'写{address}: {exc}')
    try:
        if servo.read(64) != 0:
            raise RuntimeError('扭矩仍开启')
        # 只有确认已关扭矩，才清除看门狗错误并重试清零。
        servo.write(98, 0)
        servo.write(102, 0, 2)
        if servo.read(102, 2) != 0:
            raise RuntimeError('目标电流未清零')
    except Exception as exc:
        errors.append(f'卸力确认失败: {exc}')
        raise RuntimeError('；'.join(errors)) from exc


def run(a, servo, confirm=input):
    model, result, error = servo.packet.ping(servo.port, servo.sid)
    servo.check(result, error)
    if model != 1200:
        raise RuntimeError(f'仅支持项目XL330-M288(model 1200)，当前model={model}')
    mode, torque, limit, watchdog = servo.read(11), servo.read(64), servo.read(38, 2), servo.read(98)
    effective_limit = a.current_limit if a.current_limit is not None else limit
    if a.current in ('max', '-max'):
        if not 1 <= effective_limit <= 1750:
            raise RuntimeError(f'无法使用硬件电流上限：{effective_limit}')
        a.current = -effective_limit if a.current == '-max' else effective_limit
        print(f'使用本轮最大电流：{a.current:+d}raw')
    if a.actual_current_limit is None:
        a.actual_current_limit = abs(a.current) + 30
    if not abs(a.current) < a.actual_current_limit <= 2000:
        raise RuntimeError('--actual-current-limit必须大于解析后的目标电流绝对值且不超过2000')
    sample = servo.sample()
    print(f'ID={a.id} mode={mode} torque={torque} Current Limit={limit}raw '
          f'position={sample[0]} current={sample[1]}raw temperature={sample[2]}°C voltage={sample[3]:.1f}V')
    hold = '不限时，按Ctrl+C停止' if a.duration == 0 else f'{a.duration}s'
    print(f'计划：0 → {a.current}raw（{a.ramp}s），保持{hold}；最大位移{a.max_travel}tick')
    if effective_limit != limit:
        print(f'本轮将临时修改EEPROM Current Limit：{limit} → {effective_limit}raw；卸力后恢复{limit}raw')
    if not a.enable_write:
        print('只读检查完成；加 --enable-write 才会输出。')
        return None
    if torque or watchdog or mode not in (0, 3, 5):
        raise RuntimeError('拒绝接管：需要torque=0、watchdog=0、mode为0/3/5')
    if abs(a.current) > effective_limit:
        raise RuntimeError(f'目标超过电流上限{effective_limit}；需要显式指定--current-limit才能修改')
    check_sample(a, sample, sample[0])
    if confirm('固定舵机和拉力计，外骨骼未穿戴，停止其他串口程序并备好断电。输入 PULL 开始: ').strip() != 'PULL':
        print('已取消，未写寄存器。')
        return None
    path = a.csv or Path(__file__).resolve().parent / f'rope_pull_id{a.id}_{datetime.now():%Y%m%d_%H%M%S_%f}.csv'
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', newline='', encoding='utf-8') as log:
        writer = csv.writer(log)
        writer.writerow(['elapsed_s', 'phase', 'id', 'target_raw', 'command_raw',
                         'current_raw', 'position_tick', 'travel_tick', 'temperature_C', 'voltage_V', 'hardware_error'])
        touched = False
        limit_touched = False
        try:
            # 等待确认期间硬件状态可能改变，写入前再次检查。
            if servo.read(64) or servo.read(98) or servo.read(11) != mode or servo.read(38, 2) != limit:
                raise RuntimeError('等待期间舵机状态改变，拒绝输出')
            touched = True  # 写入可能已执行但应答丢失，也必须进入卸力流程。
            if mode != 0:
                servo.write(11, 0)
            if servo.read(11) != 0:
                raise RuntimeError('电流模式回读失败')
            servo.write(102, 0, 2)
            if effective_limit != limit:
                limit_touched = True  # 即使应答丢失，也尝试恢复。
                servo.write(38, effective_limit, 2)
                time.sleep(0.05)
                if servo.read(38, 2) != effective_limit:
                    raise RuntimeError('Current Limit回读不符，拒绝开启扭矩')
            servo.write(98, 25)  # 500ms；独占总线时通信中断自动停止。
            if servo.read(98) != 25:
                raise RuntimeError('看门狗设置失败')
            initial = servo.sample()
            check_sample(a, initial, initial[0])
            start = initial[0]
            servo.write(64, 1)
            began = time.monotonic()
            last_print = -1.0
            while True:
                elapsed = time.monotonic() - began
                if a.duration != 0 and elapsed >= a.ramp + a.duration:
                    break
                sample = servo.sample()
                check_sample(a, sample, start)
                if servo.read(98) != 25 or servo.read(64) != 1:
                    raise RuntimeError('看门狗触发或扭矩意外关闭')
                # 通信耗时不能让已超时的一轮继续加载。
                elapsed = time.monotonic() - began
                if a.duration != 0 and elapsed >= a.ramp + a.duration:
                    break
                command = round(a.current * min(elapsed / a.ramp, 1))
                servo.write(102, command, 2)
                phase = 'hold' if elapsed >= a.ramp else 'ramp'
                pos, current, temp, voltage, hw = sample
                writer.writerow([round(elapsed, 3), phase, a.id, a.current, command,
                                 current, pos, pos - start, temp, voltage, hw])
                log.flush()
                if elapsed - last_print >= 0.5:
                    print(f'{phase:4s} {elapsed:5.1f}s 目标={command:+d} 实际={current:+d}raw '
                          f'位移={pos-start:+d}tick 温度={temp}°C 电压={voltage:.1f}V', flush=True)
                    last_print = elapsed
                time.sleep(0.05)
        except KeyboardInterrupt:
            if a.duration != 0:
                raise
            print('\n已手动停止不限时测试，正在卸力。')
        finally:
            if touched:
                try:
                    release(servo)
                    print('已确认目标电流0、扭矩关闭。保留模式0，不自动回位。')
                except Exception as exc:
                    print(f'立即物理断电！卸力失败：{exc}', file=sys.stderr)
                    if limit_touched:
                        print(f'Current Limit可能仍为{effective_limit}，下次运行前须检查并恢复{limit}raw。', file=sys.stderr)
                    raise
                if limit_touched:
                    try:
                        servo.write(38, limit, 2)
                        time.sleep(0.05)
                        if servo.read(38, 2) != limit:
                            raise RuntimeError('恢复值回读不符')
                        print(f'Current Limit已恢复为{limit}raw。')
                    except Exception as exc:
                        raise RuntimeError(f'已卸力，但Current Limit恢复失败；下次运行前须恢复{limit}raw：{exc}') from exc
    print(f'遥测已保存：{path}')
    return path


def stop_signal(signum, frame):
    # SIGTERM也执行finally卸力，但不进入手工拉力输入。
    raise SystemExit(128 + signum)


def main(argv=None):
    a = parse_args(argv)
    try:
        from dynamixel_sdk import PacketHandler, PortHandler
    except ImportError:
        print('缺少依赖 dynamixel-sdk，请使用已安装该依赖的Python环境。', file=sys.stderr)
        return 2
    port = PortHandler(a.device)
    previous = signal.signal(signal.SIGTERM, stop_signal)
    try:
        if not port.openPort() or not port.setBaudRate(a.baudrate):
            raise RuntimeError(f'无法打开串口/设置波特率：{a.device}')
        # pyserial独占锁可阻止其他同样使用exclusive的程序打开端口。
        if hasattr(port, 'ser') and hasattr(port.ser, 'exclusive'):
            port.ser.exclusive = True
        path = run(a, Servo(PacketHandler(2.0), port, a.id))
    except KeyboardInterrupt:
        print('\n测试中止。')
        return 130
    except Exception as exc:
        print(f'测试失败：{exc}', file=sys.stderr)
        return 1
    finally:
        port.closePort()
        signal.signal(signal.SIGTERM, previous)
    if path and a.record_force:
        try:
            value = input('输入刚才保持阶段的稳定拉力(N)，回车跳过：').strip()
            if value:
                force = float(value)
                if not math.isfinite(force) or force < 0:
                    raise ValueError('拉力必须为有限非负数')
                torque = force * a.radius_mm / 1000 if a.radius_mm else ''
                with path.with_suffix('.force.csv').open('x', newline='', encoding='utf-8') as f:
                    w = csv.writer(f)
                    w.writerow(['id', 'target_current_raw', 'force_N', 'radius_mm', 'estimated_spool_torque_Nm'])
                    w.writerow([a.id, a.current, force, a.radius_mm or '', torque])
                print(f'已记录拉力 {force:g}N' + (f'，估算卷线轴扭矩 {torque:.4f}N·m' if torque != '' else ''))
        except (KeyboardInterrupt, EOFError):
            print('\n未记录拉力；遥测已保留。')
        except (ValueError, OSError) as exc:
            print(f'拉力记录失败，遥测已保留：{exc}', file=sys.stderr)
            return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
