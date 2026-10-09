# mHandPro 与 Inspire 左手遥操

本目录只包含 mHandPro 手套、六维映射和 Inspire 左手遥操所需的最小代码。外骨骼代码位于同级 `../exoskeleton/`。

## 工作原理

`mhandpro_diagnostic.cpp` 直接动态加载 mHandPro 官方 ARM64 SDK，读取 20 个骨骼节点四元数，计算父子骨段相对旋转，再输出：

```text
[小指, 无名指, 中指, 食指, 拇指弯曲, 拇指对掌]
```

`standalone_inspire_bridge.py` 连接 `192.168.123.210:6000`，接收本机 `127.0.0.1:9102` 的六通道命令，执行行程限制后写入 Inspire。

## 编译

x86_64 和 aarch64 都能编，`build.sh` 用本机 `g++`，按编译平台自动选库：

```bash
bash build.sh
```

编译结果为 `bin/mhandpro_diagnostic`。默认加载的库按平台分：

| 平台 | 默认库路径 |
|---|---|
| x86_64 | `sdk/lib/x64/libVDMocapSDK_mHandPro.so` |
| aarch64 | `sdk/lib/arm64/libVDMocapSDK_mHandProArm64.so` |

第一个命令行参数可以覆盖库路径。

**`.so` 是按发行版分别编译的，不能混用。** 厂商交付在
`mHandPro_LinuxSDK/so/`，下面有 `ubuntu20.04_x64`、`ubuntu22.04_x64`、
`ubuntu20.04_arm64`、`ubuntu22.04_arm64`、`kylin_v10_arm64` 五份。
`sdk/lib/x64/` 里这份来自 `ubuntu22.04_x64`（开发机是 22.04）；换机器前先
`lsb_release -d` 确认，拷错版本不会立刻报错，只会在运行时出奇怪问题。

SDK 版本 3.0.20，依赖全是标准库（libstdc++/libm/libmvec/libgomp/libgcc/libc），
不需要装厂商的 XR 串口驱动 —— 接收器（Exar XR21V1410，USB ID `04e2:1410`）
在内核 6.8 上直接枚举成 `/dev/ttyUSB0`，SDK 自己会扫 `/dev` 找口，
`ttyUSB` 和 `ttyXRUSB` 两种命名都认。用户需在 `dialout` 组。

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
calibrate                      官方标准 P-pose 手掌标定（推荐）
quickpose                      官方快速 P-pose（熟练后使用，不检查姿势质量）
mapcal                         五姿势标定六维Inspire映射并自动保存
show                           检查六维输出
teleop config/inspire_left_pi_smoke_10.cfg
teleop config/inspire_left_video_100.cfg
quit
```

## 图形上位机（推荐）

`mhandpro_studio.py` 把上述 CLI 功能集中到一个桌面界面，并实时显示左右
Inspire FTP 手部 URDF。标定、映射和安全判据仍由同一个
`mhandpro_diagnostic` 执行，界面没有复制一套算法。URDF 直接复用仓库中的
`仿真/models/g1_29dof/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf`，六通道到
12 个活动关节的展开直接复用 `PC端/hand_mapping.py`。

首次使用可建立独立环境：

```bash
cd /home/hong/unitree/mhandpro
python3 -m venv .venv
.venv/bin/pip install -r requirements-gui.txt
bash build.sh
./studio.sh
```

如果 `mhandpro/.venv` 不存在，`studio.sh` 会优先复用已经配置好的
`PC端/.venv`。无硬件查看界面和双手动画：

```bash
./studio.sh --demo
```

界面用法：

- 启动后先点击右上角“连接数据手套”。界面不再自动占用 USB 接收器；需要时可
  用同一位置的“断开数据手套”安全退出诊断进程。
- 左侧选择右手、左手或双手，再点击 P-pose、五姿势映射、张手零点等操作。
- 标定会进入类似手臂标定界面的全屏工作流卡片，显示步骤点、当前手势、说明、
  倒计时/采集阶段和质量日志；“取消”不会覆盖旧映射。
- 鼠标左键拖动手部模型，滚轮缩放；底部同时显示六通道闭合度。
- “状态检查”和“20 秒链路检测”会打开可保持的大型诊断面板，不再只显示三行。
- 数据输出：先启动原来的 `dual_arm_viz.py`/9103、9104 下游，再选右手、左手或
  双手，点击“启动双手真机映射（100%）”。双手时上位机执行的就是原 CLI：
  `teleop both config/inspire_right_sim.cfg config/inspire_left_sim.cfg`。按大面板中的
  ARM 后开始输出，运行时按钮变为红色 STOP；不再尝试从 PC 直连机器人内网的
  `192.168.123.210/.211:6000`。
- `*_sim.cfg` 的 `range_scale=1.0`，输出完整 0–100% 闭合度。底部“高级命令”仍可
  执行 CLI 命令；同一接收端不要再启动第二个 `teleop` 写入者。

当前真机 Wi-Fi 地址为 `192.168.3.78`，吊装 lowcmd 全链路使用：

```bash
# 终端 A：机器人 ~/zh，两次安全确认都输入 yes
# --hand-haptic 仅供机器人端力反馈手套；数据手套上位机不再使用它
~/zh/start.sh --hand both --hand-haptic --waist --mode lowcmd --hand-touch-hz 10

# 终端 B：PC 双臂/双手/腰部发送端
/home/hong/unitree/最新直接能用的代码/PC端/.venv/bin/python \
  /home/hong/unitree/最新直接能用的代码/PC端/dual_arm_viz.py \
  --hand --hand-left --waist --udp-target 192.168.3.78:9527

# 终端 C：新数据手套上位机，代替原 CLI
cd /home/hong/unitree/mhandpro && ./studio.sh
```

新上位机优先使用原真机验证路径下的
`最新直接能用的代码/mhandpro/config/inspire_{right,left}_sim.cfg`。
- 机器人端部署 `hand_feedback.py` 后，上位机会显示五级链路、六路目标/实际角度、
  相对抓握力、错误、状态和温度。3D 中半透明青色是映射目标，实体绿色是真手
  实际角度；橙色通道表示检测到接触。
- 顶部链路状态已放到左手卡片右侧，不再遮挡下方 3D 手模型。
- 数据手套震动反馈已完全停用：上位机不连接 9201/9202，不监听
  本机 UDP 9536，不向 mHandPro SDK 下发非零震动等级。启动和退出时仅会
  发送一次 `TREMOR_NONE`清理可能的遗留震动。机器人端力反馈手套不受影响。
- 顶部“一键记录闭环数据”把手套、虚拟指尖、映射目标和真手反馈异步写到
  `PC端/hand_records/hand_episode_*.jsonl`。磁盘慢时只丢日志，不阻塞控制。
- 安全回放不会建立任何真机 socket：`./studio.sh --replay <episode.jsonl>`。
- 导出训练/仿真中间格式：
  `../PC端/.venv/bin/python ../PC端/export_hand_episode.py <episode> --output <目录> --target lerobot`。

当前 `*_sim.cfg` 默认使用 One Euro 自适应滤波和 0.08/帧的上游限步；旧 EMA
仍可通过 `filter=ema` 一行切回。机器人端 60 counts/帧的最终安全限速没有取消。

状态卡把“在线”和“节点质量”分开判断：有持续新帧就显示在线；`SS_NONE` 是
没有安装物理传感器的骨架节点，不算故障；真正的 `NO_DATA`、`UNREADY` 或
`BAD_MAG` 会显示“在线 · 节点需检查”，详细节点在状态检查面板中查看。

OK 手势使用 `thumb_uv` 指腹锚点。旧 V4 标定继续使用节点3位置；重新执行一次
`mapcal` 会保存 V5，并改用 SDK 五指虚拟指尖，绝不会拿新坐标解释旧锚点。
接近 OK 锚点时，上位机会将拇指映射到
对掌 1.0 / 弯曲 0.5，并把食指平滑引导到 URDF 接触闭合度 0.58；左右手模型的
指腹间隙约 3.2 mm。远离 OK 手势时食指保持原六维映射。

启动前的纯软件自检：

```bash
./studio.sh --check
../PC端/.venv/bin/python -m unittest -v test_mhandpro_studio.py
../PC端/.venv/bin/python -m unittest -v ../PC端/test_hand_telemetry.py \
  ../PC端/test_hand_closed_loop.py
```

程序会按当前连接的左/右手自动加载对应六维动作映射。`calibrate` 成功后会继续
用官方 P-pose 的并指伸直手型采集本次张手零点，因此日常不再需要手动
`load`、`zero`。标准 P-pose 要求：

- 身体站直，手臂向正前方伸直并平举到胸口高度。
- 手掌朝下，手掌、小臂、大臂尽量在同一直线上。
- 食指到小指并拢伸直，拇指与食指张开约 45~60°。
- 标定期间身体、手臂、手腕和手指全部保持静止。

`quickpose` 直接调用厂商 `FastCalibration`，没有进度和成功返回值，只适合已经
熟悉标准姿势后的日常快速回正。姿态不确定或快速标定后仍有偏差时使用
`calibrate`。

逐指弯曲、展开和拇指对掌等命令不属于厂商官方手掌标定，只用于维护六维
Inspire 映射；输入 `mapping-help` 查看。更换操作者、手套尺寸或映射明显串扰
时才需要重做。

推荐用一个 `mapcal` 代替逐项命令。先完成 `calibrate`，再输入 `mapcal`，
程序会依次采集五个独立姿势：

1. 比赞：四指弯到日常抓握深度，拇指竖起。
2. 四指重新伸直，然后自然展开。
3. 只做拇指弯曲。
4. 四指伸直并拢，拇指横贴掌心。
5. OK 手势：拇指指腹与食指指腹相碰。

每个姿势都有单独提示、3秒倒计时和1.5秒静止采集。幅度不足或拇指两轴过于
相似时只重做当前姿势；五步全部合格后才自动覆盖保存当前手的
`config/right_hand.calib` 或 `config/left_hand.calib`，中途取消不会修改旧文件。

100% 配置只用于已验证的空载映射；加入外骨骼后应重新从低行程限制开始。

## 树莓派当前设备

```text
Wi-Fi SSH：192.168.3.76
eth0：192.168.123.100/24
Inspire：192.168.123.210:6000
mHandPro：/dev/ttyUSB0
```

`pi` 用户必须属于 `dialout`，并对 `/dev/ttyUSB0` 可读可写。

## 官方磁力校准（`magcal`）

`teleop` 的安全看门狗把 `BAD_MAG` 判为致命故障并中止控制。磁场是环境属性，
换个位置往往还在（实测报错节点会从手背跳到中指，说明不是单颗传感器的问题），
所以要用官方磁校准 API 重新校准磁力计。

```
诊断> magcal          # 官方深度校准，75 秒；持续 BAD_MAG 时使用
诊断> magcal normal   # 官方普通校准，30 秒
```

摘下手套，握住腕口，远离显示器/机箱/音箱/无线充电板。让整只手套缓慢覆盖
前后、左右、上下各个朝向，同时绕三个轴连续翻转，不能只在一个平面画圈。
**完成后必须把手套关机再开机才生效。**

实现严格遵循官方 API 生命周期：

1. `StartMagCorrect()` 开始采集。
2. 按普通 30 秒或深度 75 秒覆盖全部朝向。
3. `EndMagCorrect()` 结束采集并开始计算。
4. `GetDGMagCorrectResult()` 读取进度和失败节点。

磁校准改变了融合输出。重启手套后先执行 `calibrate`，它会自动更新本次张手
零点；再用 `show` 检查单指动作。只有仍有明显串扰或满量程不足时，才通过
`mapping-help` 重做对应六维动作映射。

## tools

`tools/` 中是 Inspire 的低风险硬件检查程序，不是日常遥操必启进程。
