#!/usr/bin/env python3
"""pymodbus 新旧版本的从站参数名兼容层。

pymodbus 3.9 起把 ``slave=`` 改名成 ``device_id=``，旧名在新版上直接抛
TypeError。本目录里的工具是 2026-07 在树莓派（当时的 pymodbus 3.6）上写的，
开发 PC 现在装的是 3.14 —— 同一份代码要在两边都能跑，所以调用点统一走这里。

不做版本号判断而是试了再退回：版本号判断会在下一次改名时再坏一遍，
按实际接受的关键字回退则不会。
"""


def read_holding(client, address, count, device_id):
    """读保持寄存器，自动适配 device_id / slave 两种关键字。"""
    try:
        return client.read_holding_registers(
            address, count=count, device_id=device_id)
    except TypeError:
        return client.read_holding_registers(
            address=address, count=count, slave=device_id)


def write_registers(client, address, values, device_id):
    """写保持寄存器，自动适配 device_id / slave 两种关键字。"""
    try:
        return client.write_registers(
            address, list(values), device_id=device_id)
    except TypeError:
        return client.write_registers(
            address=address, values=list(values), slave=device_id)
