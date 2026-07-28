# newteleop

`newteleop` 是 dexEXO 重构期间的最小、可独立部署的新遥操包。当手套、Inspire 灵巧手和外骨骼力反馈全部调试完成后，可将整个目录移出旧 `dexEXO` 仓库。

## 目录职责

```text
newteleop/
├── README.md
├── requirements.txt
├── computer_debug/                 # Ubuntu x64 电脑调试资料
├── mhandpro/
│   ├── mhandpro_diagnostic.cpp       # 手套解算、标定和遥操主程序
│   ├── standalone_inspire_bridge.py # Inspire Modbus TCP 安全桥
│   ├── config/                      # 左手标定和行程配置
│   ├── sdk/                         # 官方 ARM64 头文件和动态库
│   └── tools/                       # Inspire 只读/点动检查工具
└── exoskeleton/
    ├── README.md
    └── tools/                       # Dynamixel 只读和安全点动工具
```

`mhandpro` 与 `exoskeleton` 不再各放一个名叫 `diagnostic` 的目录：两边都统一使用 `tools`，但由上级目录明确区分设备。`computer_debug` 保留电脑端 x64 SDK 和调试过程，不是树莓派必需部分。

## 树莓派最小部署

在开发电脑上执行：

```bash
scp -r ~/桌面/dexexo/dexEXO/newteleop pi@192.168.3.76:/home/pi/cnn/
```

在树莓派上执行：

```bash
cd ~/cnn/newteleop
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
cd mhandpro
chmod +x build.sh
./build.sh
```

手套程序在 `mhandpro` 目录中运行，以保证 `config/` 相对路径正确：

```bash
cd ~/cnn/newteleop/mhandpro
../.venv/bin/python standalone_inspire_bridge.py
./bin/mhandpro_diagnostic
```

外骨骼只读检查：

```bash
cd ~/cnn/newteleop
.venv/bin/python exoskeleton/tools/exo_dynamixel_probe.py
```

## 当前边界

- 已包含：mHandPro 读取/标定、Inspire 左手遥操、Dynamixel 硬件检查。
- 尚未迁入：旧 BLE 手套逻辑、旧五指 PID 主程序、G1/ROS/仿真代码。
- 后续只把经实物验证的外骨骼力控部分迁入，避免重新带入旧工程的耦合。
