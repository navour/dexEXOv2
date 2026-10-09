# mHandPro 与 Inspire 左手遥操

> 运行位置：树莓派 ARM64  
> 六维顺序：`[小指, 无名指, 中指, 食指, 拇指弯曲, 拇指对掌]`  
> 更新：2026-07-30  
> 状态：左手已实测；右手mHandPro与右INSPIRE尚未接入

---

## 目录

1. [工作原理](#工作原理)
2. [文件与端口](#文件与端口)
3. [编译](#编译)
4. [运行顺序](#运行顺序)
5. [与外骨骼联合测试](#与mhandpro外骨骼联合测试)
6. [穿戴外骨骼的独立标定](#穿戴外骨骼的独立mhandpro标定)
7. [设备与工具](#树莓派当前设备)

---

本目录只包含 mHandPro 手套、六维映射和 Inspire 左手遥操所需的最小代码。外骨骼代码位于同级 `../exoskeleton/`。

## 工作原理

`mhandpro_diagnostic.cpp` 直接动态加载 mHandPro 官方 ARM64 SDK，读取 20 个骨骼节点四元数，计算父子骨段相对旋转，再输出：

```text
[小指, 无名指, 中指, 食指, 拇指弯曲, 拇指对掌]
```

`standalone_inspire_bridge.py` 连接 `192.168.123.210:6000`，接收本机 `127.0.0.1:9102` 的六通道命令，执行行程限制后写入 Inspire。同一进程以默认 20 Hz 读取 `FORCE_ACT`，以5 Hz读取拇、食、中、无名、小指的五组 `top_touch` 阵列，并将换行 JSON 只读广播到 `127.0.0.1:9202`。力控程序通过 `127.0.0.1:9302` 使整手进入抓取保持，当前只对食指叠加力差PID。

## 文件与端口

| 文件/端口 | 作用 |
|---|---|
| `mhandpro_diagnostic.cpp` | 左手姿态、标定、六维映射与遥操命令发送 |
| `standalone_inspire_bridge.py` | TCP命令到INSPIRE Modbus的唯一安全桥 |
| `9102` | mHandPro六维命令输入 |
| `9202` | `FORCE_ACT`与五指`top_touch`只读广播 |
| `9302` | 外骨骼状态与INSPIRE保持/力差修正 |

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
../.venv/bin/python standalone_inspire_bridge.py \
  --enable-write \
  --safe-open 980,965,957,946,949,922 \
  --force-hz 20 --touch-hz 5
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
teleop config/inspire_left.cfg
teleop config/inspire_left_haptic_100.cfg
teleop config/inspire_left_video_100.cfg
quit
```

`inspire_left_haptic_100.cfg` 是力反馈专用100%配置，禁用额外捏取协同；
`inspire_left_video_100.cfg` 含视频用捏取协同，不用于力反馈对比。

## 与mHandPro、外骨骼联合测试

桥接器额外监听9302，状态含义为：

- `GLOVE`：Inspire六通道跟随mHandPro。
- `FORCE_ENTRY`：Inspire六通道继续跟随mHandPro，允许完成整手包络；外骨骼正电流收绳。
- `LOCKED`：整手继续保持；食指先向张开方向退让80 tick，再复用旧程序PID，每周期最多调整3 tick。
- `RELEASE`：六通道按每次最多50 tick平滑恢复跟随mHandPro，外骨骼回 `init_pos`。
- `STOP`或9302超时0.5秒：取消覆盖，恢复mHandPro。

先用10%配置验证方向。本轮已确认低行程效果不明显后，切换力反馈专用
`config/inspire_left_haptic_100.cfg`进行抓取。FREE和FORCE_ENTRY都跟随手套；
只有舵机位置稳定进入LOCKED后，整手才保持当前抓取姿态。
完整四终端命令参见 `../exoskeleton/README.md`。

### 两个手套终端已连接后的具体操作

终端1是 `standalone_inspire_bridge.py`。它显示下列信息后保持运行，
不需要再输入命令：

```text
JSON监听: 127.0.0.1:9102
FORCE_ACT发布: 127.0.0.1:9202
力反馈覆盖: 127.0.0.1:9302
写入状态: 已启用
桥接已启动
```

终端2是 `mhandpro_diagnostic`，在 `诊断>` 提示符后按顺序操作。

1. 检查手套连接：

   ```text
   status
   ```

   应看到左手连接、有效数据帧，且20个节点没有
   `BadMag` / `NoData` / `UnReady`。

2. 加载已有左手标定：

   ```text
   load
   ```

   等待出现“标定已加载”。这一步不需要重做完整动作标定。

3. 手掌与手指自然张开、保持不动，更新本次佩戴零点：

   ```text
   zero
   ```

   `zero` 只更新张手零点，不会覆盖已加载的屈伸与对掌标定。

4. 检查六维解算：

   ```text
   show
   ```

   张手时六维应接近0。再分别缓慢弯曲小指、无名指、中指、食指、
   拇指屈曲和拇指对掌，每次输入 `show`，确认对应维度主要增大。

5. 先做10%空载方向测试：

   ```text
   teleop config/inspire_left_pi_smoke_10.cfg
   ```

   程序会再次要求确认。保持mHandPro张手，确认Inspire周围无障碍后输入：

   ```text
   ARM
   ```

   逐指缓慢屈伸，只检查方向、通道顺序和连续性。结束时在同一终端输入：

   ```text
   STOP
   ```

   `STOP` 会退出teleop并返回 `诊断>`；不要用 `Ctrl+C` 代替正常STOP。

6. 10%的六通道全部正确后，启动力反馈专用100%抓取配置：

   ```text
   teleop config/inspire_left_haptic_100.cfg
   ```

   保持张手并再次输入：

   ```text
   ARM
   ```

   此时可用手套缓慢让Inspire整手抓住软物体。在外骨骼程序进入
   FORCE_ENTRY期间六通道仍跟随手套，可继续调整包络；进入LOCKED后，
   桥接器才自动保持当前六通道抓取角度。

7. 整个联合测试结束时，先在外骨骼力控终端输入 `STOP`，再在
   mHandPro teleop终端输入 `STOP`。回到 `诊断>` 后可输入：

   ```text
   quit
   ```

若teleop启动后显示无法连接 `127.0.0.1:9102`，先检查终端1是否仍显示
“桥接已启动”。若桥接显示安全返回或JSON客户端已关闭，先 `STOP`，
确认端口9102/9202/9302只有一个桥接进程占用后再重试。

## 穿戴外骨骼的独立mHandPro标定

外骨骼会改变手套节点姿态、可达行程和拇指轨迹，因此应另存
`config/left_hand_exoskeleton.calib`，不覆盖裸手的 `left_hand.calib`。

### 重要：标定命令会立即采样

`calib index` 等命令不会在命令之后等待用户再弯手指。按下回车后
程序立即开始平均采样。每一项都必须按下列顺序操作：

1. 先将目标手指移到最大舒适标定姿态。
2. 保持姿态不动。
3. 用另一只手输入 `calib ...` 并回车。
4. 看到“已完成…标定”后才恢复张手。

如果先输入命令、后弯手指，程序会把张手或过渡姿态记为满量程轴，
该通道可能始终为0或非常不灵敏。

新版程序会在四指屈曲标定后显示“有效幅度”。幅度必须至少5度；
低于5度时本次标定会被拒绝。四指单屈曲输出使用标定轴投影的绝对幅度，
避免食指第二关节接近180度时四元数等价符号翻转将真实屈曲夹成0。

### 新建外骨骼专用标定

停止teleop和外骨骼力控，保持舵机不拉绳。穿好手套与外骨骼，
自然张手并保持1.5秒，输入：

```text
open
```

`open` 会建立新张手零点并清除内存中之前的动作轴。对于一套全新的
外骨骼标定，不要先 `load config/left_hand.calib`；否则裸手标定中保留的
四指侧摆轴可能与外骨骼下的新屈曲轨迹串扰。

然后依次完成五个屈曲轴和拇指对掌轴。每行都是“先摆好并保持姿态，
再输入命令”：

```text
食指单独最大舒适弯曲  -> calib index
中指单独最大舒适弯曲  -> calib middle
无名指单独最大舒适弯曲 -> calib ring
小指单独最大舒适弯曲  -> calib pinky
拇指只屈曲、不对掌        -> calib thumb-flex
拇指根部横向移向小指根部 -> calib thumb-opp
```

当前Inspire六个主动自由度不包含四指侧摆。外骨骼限制侧摆时不要执行
`calib spread`，否则过小或错向的侧摆轴可能干扰屈曲计算。

标定后自然张手，输入 `show`；再逐指单独弯曲并反复 `show`。张手时
六维建议小于0.05～0.10，目标通道最大舒适动作建议达到0.8～1.0，
非目标通道尽量低于0.2。可用 `monitor` 连续观察10秒。

验证正常后另存：

```text
save config/left_hand_exoskeleton.calib
```

以后穿戴外骨骼时：

```text
load config/left_hand_exoskeleton.calib
zero
show
```

### 只重新标定一个自由度

可以单独重新标定一个自由度，例如 `calib thumb-flex`。该命令
只替换内存中的拇指弯曲轴，不会清除其他四指或拇指对掌标定。
必须先加载要保留的标定文件：

```text
load config/left_hand_exoskeleton.calib
zero
show
```

先将拇指保持在“最大舒适弯曲、不对掌”的姿态，再输入：

```text
calib thumb-flex
```

恢复张手后用 `show` 验证，正常后保存回同一文件：

```text
save config/left_hand_exoskeleton.calib
```

单项修正时不要执行 `open`；`open` 会建立新张手零点并清除旧动作轴。
当次佩戴的零点变化只用 `zero`。如果在新进程中未先 `load` 就直接
单项标定，内存中可能没有需要保留的其他标定。

`zero` 只用于补偿每次穿戴的零点差异，不会清除已保存的动作轴。

### 磁状态与动作标定的区别

`status` 会显示20个节点状态。只有出现 `BAD_MAG`、`UNREADY` 或 `NO_DATA`
时，才先处理磁干扰、连接或SDK P-pose标定。如果节点均为 `OK`，原始三段
转角会随动作明显变化，但某一新六维始终为0，通常是该动作轴采样姿态或
方向错误，而不是磁标定问题。

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
