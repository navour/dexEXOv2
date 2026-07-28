# mHandPro 与 Inspire 左手遥操

本目录只包含 mHandPro 手套、六维映射和 Inspire 左手遥操所需的最小代码。外骨骼代码位于同级 `../exoskeleton/`。

## 工作原理

`mhandpro_diagnostic.cpp` 直接动态加载 mHandPro 官方 ARM64 SDK，读取 20 个骨骼节点四元数，计算父子骨段相对旋转，再输出：

```text
[小指, 无名指, 中指, 食指, 拇指弯曲, 拇指对掌]
```

`standalone_inspire_bridge.py` 连接 `192.168.123.210:6000`，接收本机 `127.0.0.1:9102` 的六通道命令，执行行程限制后写入 Inspire。同一进程以默认 20 Hz 读取 `FORCE_ACT`，以5 Hz读取拇、食、中、无名、小指的五组 `top_touch` 阵列，并将换行 JSON 只读广播到 `127.0.0.1:9202`，供外骨骼力反馈使用。

## 编译

树莓派需为 64 位 ARM (`uname -m` 输出 `aarch64`)：

```bash
cd ~/cnn/newteleop/mhandpro
chmod +x build.sh
./build.sh
```

编译结果为 `bin/mhandpro_diagnostic`。程序默认加载：

```text
sdk/lib/arm64/libVDMocapSDK_mHandProArm64.so
```

## 运行顺序

终端 1：

```bash
cd ~/cnn/newteleop/mhandpro
../.venv/bin/python standalone_inspire_bridge.py
```

终端 2：

```bash
cd ~/cnn/newteleop/mhandpro
./bin/mhandpro_diagnostic
```

常用命令：

```text
status                         查看手套连接和传感器状态
load                           加载 config/left_hand.calib
zero                           按当前自然张手更新零点
show                           检查六维输出
teleop config/inspire_left_pi_smoke_10.cfg
teleop config/inspire_left_video_100.cfg
quit
```

100% 配置只用于已验证的空载映射；加入外骨骼后应重新从低行程限制开始。

## 树莓派当前设备

```text
Wi-Fi SSH：192.168.3.76
eth0：192.168.123.100/24
Inspire：192.168.123.210:6000
mHandPro：/dev/ttyUSB0
```

`pi` 用户必须属于 `dialout`，并对 `/dev/ttyUSB0` 可读可写。

## tools

`tools/` 中是 Inspire 的低风险硬件检查程序，不是日常遥操必启进程。

不带数据手套时，可直接只读监视 Inspire 左食指 `fingerfour_top_touch`：

```bash
cd ~/cnn/newteleop
./.venv/bin/python mhandpro/tools/inspire_index_top_touch_monitor.py
```

该工具读取地址4128的96个16位寄存器，按官方SDK重排为12x8阵列；不写入
角度、力、速度或标定寄存器。右手轻压Inspire食指指尖时，重点观察
`max_raw`、`top5_mean`和最大值坐标是否变化。显示完整阵列：

```bash
./.venv/bin/python mhandpro/tools/inspire_index_top_touch_monitor.py \
  --matrix --seconds 20
```
