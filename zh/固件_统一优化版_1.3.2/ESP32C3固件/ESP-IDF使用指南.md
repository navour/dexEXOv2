# ESP-IDF 新手使用指南

## 1. 本指南适用范围

本文面向第一次使用 ESP-IDF 的开发者，分为两部分：先解释通用概念，再介绍本仓库 ESP32-C3 无线 IMU 固件的实际操作。

本文使用以下环境：

- Ubuntu Linux
- ESP-IDF v5.4.2
- ESP32-C3
- 固件源码：`/home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件`
- 示例串口：`/dev/ttyACM0`（实际端口可能变化）

## 2. 先认识几个概念

| 名称 | 作用 |
|---|---|
| ESP-IDF | 乐鑫官方的 ESP32 开发框架，包含编译器、库和烧录工具。 |
| `idf.py` | ESP-IDF 的统一命令入口，用于配置、编译、烧录和查看日志。 |
| 项目 | 含顶层 `CMakeLists.txt` 的固件源码目录。 |
| 组件 | 功能相对独立的代码模块，通常包含源码、头文件和自己的 `CMakeLists.txt`。 |
| `sdkconfig` | 当前项目的 ESP-IDF 配置结果，例如芯片、Flash 和系统功能配置。 |
| 构建目录 | 保存 CMake/Ninja 缓存、目标文件和最终固件，不是源代码。 |
| 烧录（flash） | 把编译产生的固件写入 ESP32 的 Flash。 |
| 监视器（monitor） | 通过串口查看 ESP32 输出的运行日志。 |

ESP-IDF 的常见工作顺序是：

```text
激活环境 → 配置项目 → 编译 → 连接开发板 → 烧录 → 查看日志
```

## 3. 本仓库固件结构

```text
ESP32C3固件/
├── CMakeLists.txt          ESP-IDF 项目入口
├── sdkconfig               当前项目配置
├── partitions.csv          Flash 分区表
├── main/
│   ├── main.c              程序入口和主任务
│   └── imu_config.h        传感器、输出和传输模式配置
└── components/             驱动和功能组件
    ├── icm20948/
    ├── wifi_transport/
    ├── power_manager/
    ├── ota_update/
    └── ...
```

通常只修改 `main/`、相关 `components/` 和配置文件，不要编辑构建目录中的生成文件。

## 4. 每次开发前：激活并检查环境

每次打开新终端，都要先激活 ESP-IDF 5.4.2：

```bash
source /home/hong/esp-idf-v5.4.2/export.sh
```

检查环境：

```bash
idf.py --version
which python3
echo "$IDF_PATH"
```

预期包含：

```text
ESP-IDF v5.4.2
/home/hong/.espressif/python_env/idf5.4_py3.10_env/bin/python3
/home/hong/esp-idf-v5.4.2
```

如果 `idf.py` 不存在，或版本显示为 6.0，说明当前终端没有激活正确环境。关闭终端重新打开，再执行上面的 `source` 命令。

## 5. 配置固件

### 5.1 传感器和传输模式

主要配置位于 `main/imu_config.h`。本项目通常使用：

```c
#define IMU_SENSOR_TYPE      IMU_SENSOR_ICM20948
#define OUTPUT_MODE          OUTPUT_MODE_VISUALIZER
#define TRANSPORT_MODE       TRANSPORT_MODE_WIFI
#define ICM_READ_TRIGGER_MODE ICM_READ_TRIGGER_INT
#define USE_MAGNETOMETER     1
```

### 5.2 配置 Wi-Fi

当前版本实际从 `components/wifi_transport/wifi_transport.c` 的 `s_networks[]` 读取网络信息：

```c
static const wifi_network_t s_networks[] = {
    { "你的WiFi名称", "你的WiFi密码" },
};
```

可以保存多个网络，设备会扫描并连接列表中优先级最高的可见网络。建议使用 2.4 GHz Wi-Fi，且不要把真实密码提交到公开仓库。

> 注意：组件虽然提供了 `menuconfig` 的 Wi-Fi 选项，但当前 `wifi_transport.c` 没有读取对应的 `CONFIG_WIFI_SSID` 和 `CONFIG_WIFI_PASSWORD`。因此，仅在 `menuconfig` 中填写 Wi-Fi 信息不会改变当前固件实际使用的网络。

### 5.3 打开配置界面

```bash
cd /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件
idf.py -B /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件/build_unified_1_3_2 menuconfig
```

方向键移动，Enter 进入，Space 选择，`S` 保存，`Q` 退出。

## 6. 编译固件

由于 ESP-IDF 工具链曾在中文构建路径中生成错误参数，本项目把构建输出放在纯英文目录：

```bash
cd /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件

idf.py -B /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件/build_unified_1_3_2 build
```

`-B` 指定构建目录。以后执行编译、烧录和监视命令时，应始终使用同一个 `-B` 路径。

编译成功时，末尾通常会出现：

```text
Project build complete
```

主要产物包括：

```text
/home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件/build_unified_1_3_2/vqf_esp32.bin
/home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件/build_unified_1_3_2/bootloader/bootloader.bin
/home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件/build_unified_1_3_2/partition_table/partition-table.bin
```

编译不会修改开发板，只会在电脑上生成固件。

## 7. 连接开发板与确认串口

连接 USB 数据线后执行：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

本设备通常显示为：

```text
/dev/ttyACM0
```

其中一种通配符没有匹配结果时，`ls` 可能同时显示一条“没有那个文件或目录”，只要另一种端口存在即可。

检查串口权限：

```bash
groups
ls -l /dev/ttyACM0
test -r /dev/ttyACM0 && test -w /dev/ttyACM0 && echo "串口权限正常"
```

用户组中应包含 `dialout`。如果没有：

```bash
sudo usermod -aG dialout "$USER"
```

随后注销并重新登录。也可用 `newgrp dialout` 临时刷新当前终端。

## 8. 烧录固件

先确认连接的是目标 IMU 开发板，再执行：

```bash
idf.py \
  -B /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件/build_unified_1_3_2 \
  -p /dev/ttyACM0 \
  flash
```

烧录成功时通常会看到写入进度、校验成功和硬件复位信息。

如果自动进入下载模式失败，可尝试：

1. 按住开发板 `BOOT` 键。
2. 短按一下 `RESET` 键。
3. 松开 `RESET`，再松开 `BOOT`。
4. 重新执行烧录命令。

不同开发板的按键名称和自动下载电路可能不同。

## 9. 查看串口日志

只查看日志：

```bash
idf.py \
  -B /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件/build_unified_1_3_2 \
  -p /dev/ttyACM0 \
  monitor
```

编译、烧录并立即查看日志：

```bash
idf.py \
  -B /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件/build_unified_1_3_2 \
  -p /dev/ttyACM0 \
  flash monitor
```

退出监视器按 `Ctrl+]`。

无线 IMU 正常联网时，应看到类似日志：

```text
扫描到若干 AP
匹配到已知网络
正在连接 WiFi
获取 IP: 192.168.x.x
WiFi 已连接
```

## 10. 常用 ESP-IDF 命令

以下命令均在项目目录中运行，并使用同一个构建目录：

```bash
# 编译
idf.py -B /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件/build_unified_1_3_2 build

# 打开配置界面
idf.py -B /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件/build_unified_1_3_2 menuconfig

# 重新运行 CMake 配置
idf.py -B /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件/build_unified_1_3_2 reconfigure

# 查看固件各部分占用空间
idf.py -B /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件/build_unified_1_3_2 size

# 清理该构建目录中的生成文件和缓存
idf.py -B /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件/build_unified_1_3_2 fullclean

# 擦除开发板上的整个 Flash（会删除固件和 NVS 数据，谨慎使用）
idf.py -B /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件/build_unified_1_3_2 -p /dev/ttyACM0 erase-flash
```

普通代码修改后通常只需要再次运行 `build`。更换 ESP-IDF 版本、芯片目标或遇到异常 CMake 缓存时，再考虑 `fullclean` 或换一个全新的构建目录。

## 11. 常见问题排查

### 11.1 `idf.py` 不存在或版本错误

现象：终端提示 `idf.py: command not found`，或者显示 ESP-IDF 6.0。

处理：

```bash
source /home/hong/esp-idf-v5.4.2/export.sh
idf.py --version
```

确认结果为 `ESP-IDF v5.4.2`。

### 11.2 Python 环境属于另一个 ESP-IDF 版本

现象：路径包含 `idf6.0_py3.13_env`，或者安装脚本提示 Python 环境由 IDF 6.0 创建。

处理：打开一个干净终端，清除旧变量后重新激活 5.4.2：

```bash
unset IDF_PATH IDF_PYTHON_ENV_PATH ESP_IDF_VERSION
source /home/hong/esp-idf-v5.4.2/export.sh
which python3
```

### 11.3 编译器提示无法执行 `cc1`

现象：

```text
riscv32-esp-elf-gcc: fatal error: cannot execute 'cc1'
```

原因通常是 RISC-V 工具链解压不完整。先验证：

```bash
find /home/hong/.espressif/tools/riscv32-esp-elf -type f -name cc1
```

如果没有结果，应备份损坏目录后重新运行 ESP-IDF 5.4.2 的 `install.sh esp32c3`。不要直接复制 `cc1plus` 冒充 `cc1`。

### 11.4 `picolibc.specs` 路径中出现 `\x0a`

原因是构建目录路径中的中文被工具链错误解析。不要继续复用原构建目录，改用纯英文目录：

```bash
idf.py -B /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件/build_unified_1_3_2 build
```

### 11.5 `/dev/ttyACM0` 不存在

依次检查：

- USB 线是否支持数据传输，而不是仅充电。
- 开发板是否正常供电。
- 换用另一个 USB 接口或数据线。
- 重新插拔后再次执行 `ls -l /dev/ttyACM* /dev/ttyUSB*`。
- 尝试 BOOT/RESET 下载模式。

串口编号可能变成 `/dev/ttyACM1`，应使用本次实际出现的端口。

### 11.6 串口存在但不可读

检查：

```bash
ls -l /dev/ttyACM0
groups
```

如果设备属于 `dialout` 组，而当前用户不在该组，按照第 7 节添加权限并重新登录。不要使用 `sudo idf.py` 绕过权限。

### 11.7 串口被其他程序占用

关闭其他串口终端、Arduino IDE、另一个 `idf.py monitor` 或可能占用端口的软件。可以检查：

```bash
fuser -v /dev/ttyACM0
```

### 11.8 固件找不到 Wi-Fi

检查：

- `s_networks[]` 中的名称和密码是否完全正确。
- 路由器或热点是否开启 2.4 GHz。
- Wi-Fi 信号是否足够。
- 路由器是否开启客户端隔离。
- PC 与 IMU 是否连接到同一个局域网。

修改网络信息后必须重新编译并烧录。

## 12. 日常操作速查

正常开发时，可以按下面的顺序执行：

```bash
# 1. 激活 ESP-IDF 5.4.2
source /home/hong/esp-idf-v5.4.2/export.sh

# 2. 检查版本
idf.py --version

# 3. 进入项目
cd /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件

# 4. 编译
idf.py -B /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件/build_unified_1_3_2 build

# 5. 确认串口
ls -l /dev/ttyACM* /dev/ttyUSB*

# 6. 烧录并查看日志（按实际端口修改）
idf.py \
  -B /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件/build_unified_1_3_2 \
  -p /dev/ttyACM0 \
  flash monitor
```

看到 `Project build complete`、烧录校验成功以及设备获得 IP，说明编译、烧录和联网三个阶段均已完成。
