# Ubuntu x64 厂商 SDK 调试资料

> 更新：2026-10-09
>
> 范围：厂商原始示例与前期调试参考。

---

## 目录

1. [系统架构](#系统架构)
2. [文件清单](#文件清单)
3. [使用方法](#使用方法)
4. [常见问题](#常见问题)

---

## 系统架构

厂商示例用于核对 SDK 接口。正式双手遥操使用 [zh/mhandpro](../../zh/mhandpro/README.md)，由一个采集进程管理一个 USB 接收器。

---

## 文件清单

| 路径 | 内容 |
|---|---|
| `mhandpro_x64_sdk/include/` | SDK 头文件 |
| `mhandpro_x64_sdk/official_sample/` | 厂商 C++ 示例及版本记录 |
| `../../zh/mhandpro/sdk/lib/x64/` | zh 现有 x64 运行库 |
| `../../zh/mhandpro/sdk/lib/arm64/` | zh 现有 ARM64 运行库 |

---

## 使用方法

正式采集编译与启动见 [zh/mhandpro README](../../zh/mhandpro/README.md)。示例源码不作为 G1 力控入口，也不自动启动 INSPIRE。
SDK 动态库必须与 Ubuntu 版本、CPU 架构匹配。原树莓派台架 C++ 的 x64 查找路径与新采集器不同，不要混用。

---

## 常见问题

串口权限不足时检查 `dialout` 组与接收器设备权限；动态库加载失败时检查路径、架构及依赖。
厂商更新记录见 [官方示例说明](mhandpro_x64_sdk/official_sample/readme.txt)。
