# 单舵机绳端最大阻力测量

脚本 `exo_rope_pull_test.py` 直接控制树莓派 `/dev/serial0` 上的一个
XL330-M288(model 1200)，不需要 G1、INSPIRE 或 FSR 数据。
右手拇/食/中/无/小指对应 ID1~5；左手对应 ID6~10。

舵机通过恒定 Goal Current 控制输出，单位约 1mA/raw，不能未经标定直接
指定精确 N·m。这里测的是给定电流下机构绳端实际拉力。
[ROBOTIS 控制表](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/)
给出的电流寄存器范围不是机构允许电流或连续工作额定值。

## 操作

1. 外骨骼不要穿戴。停止上位机、力反馈和其他读写该串口的程序。
   固定舵机，将拉力计串接在绳子和固定支架之间，对准实际拉绳方向，消除松绳。
   拉力计先归零。测量时缓慢施力，避免用猛拉峰值作为稳定阻力。
2. 在树莓派 `newteleop` 目录，用安装了 `dynamixel-sdk` 的 Python 运行。
   例如左食指，先只读检查（不会写寄存器）：

   ```bash
   ./.venv/bin/python test/exo_rope_pull_test.py --id 7 --current 30
   ```

3. 确认正电流是该指的收绳方向后开始，反向收绳则用 `--current -30`：

   ```bash
   ./.venv/bin/python test/exo_rope_pull_test.py \
     --id 7 --current 30 --duration 5 --enable-write --record-force
   ```

   输入 `PULL` 后，用2秒升到目标，保持5秒，再卸力。观察 `hold` 阶段稳定读数，
   卸力后输入拉力（N）。如果拉力计显示 kgf，乘 9.80665 换算为 N。
   `Ctrl+C` 可提前卸力退出。断绳、松脱时可能空转收绳，保持可立即断电。
4. 每轮检查机构和温度，逐级改为50、100、150、200、250、300等电流重测，
   不要直接从低电流跳到寄存器最大值。充分冷却后再重复高负载试验。
   达到机构/拉力计额定载荷、温升明显或力不再增加时停止加流。

当前力反馈软件单指最大请求为300raw；测试 `--current 300` 可以评估这个
配置下的最大绳端阻力。它不代表舵机物理极限，也不代表可以持续输出。
脚本默认不提高舵机 EEPROM 中已有的 Current Limit。只有显式指定
`--current-limit` 才会临时修改，并在卸力后恢复原值；详见下方600raw测试。

## 参数和记录

- `--max-travel 200`：相对本轮起点位移达到200tick立即卸力。默认不允许禁用，
  可设置1~6000tick，例如 `--max-travel 6000`。默认仍为200tick。
  应按机构剩余行程设置，不能把触发保护误认为最大力。
- `--actual-current-limit`：回读电流停机阈值，默认目标绝对值+30raw。
- `--max-temperature 55`：默认55°C停机；这是软件阈值，不是连续加载保证。
- `--duration 5`：保持时间0.1~120秒，0表示不限时；`--ramp 2`：升流时间1~10秒。
- `--csv 路径.csv`：指定遥测文件，不覆盖已有文件，默认保存在脚本所在的 `newteleop/test/`，
  不受启动时工作目录影响。手工拉力记录的 `.force.csv` 也保存在同一目录。
  遥测是每轮写新命令前的读数，包含升流和保持阶段，不会读取拉力计。
- `--record-force`：成功完成并卸力后手工记录稳定拉力，保存到同名 `.force.csv`。
- `--radius-mm 8`：配合记录拉力，用 `F × r` 估算卷线轴扭矩，单位 N·m。
  半径是绳层中心的有效半径；有滑轮倍率或明显摩擦时不能直接视为舵机扭矩。

通信错误、硬件错误、超温、过流或超行程会终止测试。总线看门狗设为500ms；
它监视总线指令间隔，其他程序的通信可能使其无法触发，所以必须独占总线。
正常结束、Ctrl+C、SIGTERM 都尝试清电流并关闭扭矩；通信已断时软件无法保证
卸力成功，应物理断电。脚本结束保留模式0，不自动回位、不自动拉紧。
模式切换可能重置控制器部分RAM参数，重新运行正式力控时应让其重新初始化。

比较多轮保持阶段的稳定力，记录电流、供电电压、温度与绳路。若保持阶段位置持续
移动，测到的是该运动条件下的阻力；需要静态最大保持力时应固定拉力计端点，
观察位置稳定后再读数。真正的持续可用最大阻力还需要额定工况下的温升测试。

## 从开发电脑同步到树莓派

按项目记录，树莓派为 `pi@192.168.3.76`，项目目录为 `/home/pi/cnn/newteleop`。
以下命令在 Ubuntu 开发电脑执行；若树莓派 IP 已变化，请替换地址：

```bash
cd /home/cnn/桌面/dexexo/dexEXO
ssh pi@192.168.3.76 'mkdir -p /home/pi/cnn/newteleop/test'
scp newteleop/test/exo_rope_pull_test.py newteleop/test/exo_rope_pull_test.md pi@192.168.3.76:/home/pi/cnn/newteleop/test/
```

同步后，在树莓派终端运行（左食指 ID7 示例）：

```bash
cd /home/pi/cnn/newteleop
./.venv/bin/python test/exo_rope_pull_test.py --id 7 --current 30 --duration 5 --enable-write --record-force
```

本地无硬件测试：在仓库根目录运行
`python3 -m unittest discover -s newteleop/test -p test_exo_rope_pull_test.py`。

## 更长保持时间和当前最大电流

`--current max` 读取当前 Current Limit 作为正向目标，反向使用
`--current=-max`。例如回读 Current Limit=300raw，max就等于300raw。
这不是舵机的物理最大电流。单独使用max不会改写EEPROM上限；
若同时传入 `--current-limit`，max会使用本轮指定的新上限。

在确认拉力计固定端承力、绳子没有大量松弛、机构剩余行程足够后，可以设置：

```bash
./.venv/bin/python test/exo_rope_pull_test.py --id 7 --current max --duration 30 --max-travel 1000 --enable-write --record-force
```

定时保持最长可设120秒，另可用 `--duration 0` 不限时，但这仅是软件允许范围，不表示高电流可以连续加载
这么久。优先用满足读数需要的短时间，留意温升；温度保护不能代替额定负载判断。
若只看到 `ramp` 就报位移超限，说明尚未进入保持阶段，延长duration不会解决。
应先检查拉力计另一端是否固定、绳子是否松弛或打滑。行程保护保持开启；
只有根据实际剩余行程确认后才增大 `--max-travel`。

## 不限时输出

同步新版脚本后，`--duration 0` 表示升流结束后持续输出，直到按 `Ctrl+C`
或触发保护。默认仍为5秒；不限时不会关闭位移、温度、电流和通信保护。

```bash
./.venv/bin/python test/exo_rope_pull_test.py --id 7 --current max --duration 0 --max-travel 1000 --enable-write --record-force
```

按 `Ctrl+C` 后先清电流、关闭扭矩，再保存遥测；如果指定了 `--record-force`，
卸力成功后会提示输入刚才观察到的拉力。不稳定或未进入保持阶段的读数可直接回车跳过。
SIGTERM或保护故障也会尝试卸力，但不会提示输入拉力。不限时加载需有人看守，
读数完成就停止；软件允许不限时不代表舵机能够连续承受最大电流。
此前的“位移达到停机阈值”仍会触发，不会因解除时间限制而消失。

## 600raw短时拉力测试

已测得300raw下约12N后，若要测试600raw，必须同时指定目标和本轮硬件上限：

```bash
./.venv/bin/python test/exo_rope_pull_test.py --id 7 --current 600 --current-limit 600 --ramp 3 --duration 3 --max-travel 3000 --min-voltage 4.0 --enable-write --record-force
```

先固定拉力计并消除松绳，确认剩余机械行程和拉力计量程足够。
本命令3秒升流、保持3秒，回读电流停机阈值默认630raw。看到hold阶段
实际电流接近600才表示达到输出；设置600不保证供电条件下实际能达到。
3000tick是沿用已测试的行程设置，不保证在更大力下不会触发，不能靠不断放宽来避开机械问题。

`--current-limit 600` 在扭矩关闭时写入EEPROM地址38，回读确认后才允许开扭矩；
正常结束、Ctrl+C、SIGTERM及保护触发时，先卸力，再恢复原来的Current Limit
（例如300raw）并回读。如果通信断开、强制杀进程或断电，恢复可能失败；
EEPROM值会保留，下次运行前必须只读检查，不能假设已恢复。每次修改和恢复
都会写EEPROM，不要在自动循环中反复启动此测试。

`--min-voltage` 默认3.7V；上面的600raw命令使用4.0V作为提前停机阈值。
此前300raw测试时供电已从5.5V跌到约4.5V，应先检查供电线、接头和电源容量。
出现低压停机时应处理供电，不要继续调低阈值强行加载。1750raw是寄存器允许
上限，不是持续额定值，600raw也不保证能长期输出；首次使用短时测试。
