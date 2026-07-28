# Ubuntu x64 电脑端调试资料

本目录与树莓派 ARM64 运行文件分开，用来保留前期在 Ubuntu 22.04 x86-64 电脑上调试 mHandPro 的官方 SDK 和过程文件。

期望的官方库位置：

```text
computer_debug/
└── mhandpro_x64_sdk/
    ├── lib/x64/libVDMocapSDK_mHandPro.so
    ├── include/                       # 如原包包含，原样保留
    └── official_sample/               # 官方 x64 示例源码/程序
```

当在 x86-64 电脑上编译 `../mhandpro/mhandpro_diagnostic.cpp` 时，程序会默认加载上述 x64 动态库。ARM64 树莓派则会加载 `../mhandpro/sdk/lib/arm64/` 中的库。

## 待恢复文件

上一次精简时原 x64 SDK 未被 Git 跟踪且没有其他副本，因此需从 mHandPro 厂商原始 Linux SDK 包重新复制到本目录。在动态库恢复前，不影响树莓派 ARM64 运行。
