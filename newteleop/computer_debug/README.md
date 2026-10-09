# Ubuntu x64 电脑端调试资料

> 运行位置：Ubuntu 20.04/22.04 x86-64 开发电脑  
> 用途：mHandPro厂商SDK复现与电脑端诊断，不部署到树莓派最小运行包  
> 更新：2026-07-30

---

## 目录

1. [用途](#用途)
2. [文件清单](#文件清单)
3. [与ARM64运行包的关系](#与arm64运行包的关系)
4. [常见问题](#常见问题)

---

## 用途

本目录与树莓派 ARM64 运行文件分开，用来保留前期在 Ubuntu 22.04 x86-64 电脑上调试 mHandPro 的官方 SDK 和过程文件。

## 文件清单

当前官方库位置：

```text
computer_debug/
└── mhandpro_x64_sdk/
    ├── lib/x64/libVDMocapSDK_mHandPro.so
    ├── include/                       # 如原包包含，原样保留
    └── official_sample/               # 官方 x64 示例源码/程序
```

## 与ARM64运行包的关系

当在 x86-64 电脑上编译 `../mhandpro/mhandpro_diagnostic.cpp` 时，程序会默认加载上述 x64 动态库。ARM64 树莓派则会加载 `../mhandpro/sdk/lib/arm64/` 中的库。`official_sample/` 是厂商原始参考实现，不作为新遥操主入口。

## 常见问题

- 串口权限不足：确认用户属于 `dialout`，重新登录后再测试。
- 动态库加载失败：检查 `lib/x64/libVDMocapSDK_mHandPro.so` 存在且架构为x86-64。
- `official_sample/readme.txt` 保持厂商原文，便于核对SDK版本，不按项目README格式改写。
