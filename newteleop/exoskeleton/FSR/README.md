# 外骨骼 FSR 读取（当前不再标定）

> 数据来源：STM32紧凑18项帧，经BLE Broker转为本机TCP  
> 更新：2026-07-30  
> 当前状态：左手通道顺序已确认；右手必须逐指识别

---

## 目录

1. [数据接口](#数据接口)
2. [启动BLE Broker](#启动ble-broker)
3. [只读监视](#只读监视)
4. [确认右手模块和手指通道](#确认右手模块和手指通道)
5. [历史标定工具](#历史推拉力计交互标定当前跳过)
6. [与旧代码的复用关系](#与旧代码的复用关系)

---

本目录只负责读取由 STM32 采集、经 BLE 转发的五片薄膜压力传感器（FSR）。
这些程序不控制 Dynamixel 舵机，也不向 STM32 写数据。

当前力反馈首测不读取标定 JSON。STM32查表的最低有效输出约为
`4.903N`：不超过该阈值视为0，超过后保留绝对力原值，不减去4.903。
`exo_pressure_calibrate.py` 仅作为历史工具保留，本轮不要运行。

仓库旧 `sensors/` 目录已检查：其中只有双 BLE 采集、HDF5 写入和 Wi-Fi 转发程序，
没有提交实际的 `.hdf5/.csv/.json` 传感器实验数据。旧程序预期的是 IMU、18路弯曲值和
5组阵列触觉的完整 JSON/HDF5 数据结构；当前实物则发送下面的18项紧凑帧，因此本目录
仅复用其 BLE/HDF5 设计思路，不直接依赖它的解析格式。

## 数据接口

左手 BLE broker 独占 BLE 连接，并把原始字节广播到 `127.0.0.1:9002`。
当前实物固件发送18项紧凑帧：

```text
{前13项旧手套数据, 拇指FSR, 食指FSR, 中指FSR, 无名指FSR, 小指FSR}
```

本目录的程序缓存 TCP 分片，只处理完整的 `{...}` 帧，并取最后五项。它们不使用
旧 `sensors/gatt_blu_251202.py` 所定义的 IMU、18路弯曲和阵列触觉 JSON/HDF5 格式。

| 手 | BLE名称 | MAC | TCP端口 |
|---|---|---|---:|
| 右 | `RFstar_85B3` | `F0:FD:45:02:85:B3` | 9001 |
| 左 | `RFstar_673B` | `F0:FD:45:02:67:3B` | 9002 |

## 启动BLE Broker

每块板只能由一个Broker独占连接：

```bash
python exoskeleton/FSR/ble_broker.py --hand right
python exoskeleton/FSR/ble_broker.py --hand left
```

也可用一个进程同时连接两块BLE，右/左仍分别广播到9001/9002：

```bash
python exoskeleton/FSR/ble_broker.py --hand both
```

`both`模式如需更换MAC或端口，使用 `--right-address`、`--left-address`、
`--right-port`和`--left-port`。不能把单手的`--address/--port`用在`both`模式。

可选择`--hand both`单进程，或在两个终端分别启动right/left；
两种方式不能同时运行。其他程序只连接9001/9002，不直接连接BLE。

## 只读监视

启动左手 broker 后，在 `newteleop` 根目录运行：

```bash
python exoskeleton/FSR/exo_pressure_monitor.py --hand left
```

有限时采集示例：

```bash
python exoskeleton/FSR/exo_pressure_monitor.py --hand right --seconds 20
```

同时监视双手（需要右手9001和左手9002两路Broker数据都已就绪）：

```bash
python exoskeleton/FSR/exo_pressure_monitor.py --hand both
```

双手模式会将右手和左手各打印一行，每个刷新周期用分隔线分组。
自定义端口时使用 `--right-port` 和 `--left-port`；`--port` 只保留给单手模式。

程序显示五路瞬时值、滚动均值、标准差和接收频率。

## 确认右手模块和手指通道

1. 只给待确认的右手板上电，运行 `bluetoothctl scan on`；断电后 `RFstar_85B3` 消失、上电后重现，才能确认MAC身份。
2. 启动右手Broker和监视器，确认末五路持续刷新。
3. 运行只读识别工具，按提示依次只压拇、食、中、无名、小指FSR：

   ```bash
   python exoskeleton/FSR/fsr_channel_identifier.py --hand right
   ```

4. 每次按压的主响应应明显大于第二响应，最终五指对应五个唯一通道。断电重启后重复一次。
5. 若帧顺序不是 `[thumb,index,middle,ring,pinky]`，记录实测排列，后续给右手解析器增加独立的 `channel_order` 重排；不要修改左手已验证的默认顺序。

如果按某一片时多个通道同时显著变化，先排查共地、ADC串扰、FSR机械耦合和线束插错；如果已有明显机械压力但数值不变，逐个拔插连接器或用万用表检查该路，而不是增加外骨骼电流。

## 历史：推拉力计交互标定（当前跳过）

传感器本体标定时，将单片FSR平放在稳定、平整的桌面夹具中，用面积稳定的平头接触头
垂直加载。标定时保持舵机断电，不要把传感器装在外骨骼上使用推拉力计。

```bash
python exoskeleton/FSR/exo_pressure_calibrate.py \
  --hand left \
  --port 9002 \
  --output exoskeleton/FSR/left_pressure_calibration.json
```

交互命令：

```text
ZERO                 五路无载采样10秒
FINGER index         选择食指，也可用 thumb/middle/ring/pinky 或 0～4
POINT 0.5            测力计稳定在0.5 N时采集3秒
STATUS               查看最新数据和已记录点数
FIT                  显示分段点及线性拟合结果
SAVE                 保存原始样本、统计量和拟合结果
QUIT                 退出
```

推荐每片依次采集加载和卸载：

```text
0 → 0.5 → 1.0 → 1.5 → 2.0 → 1.5 → 1.0 → 0.5 → 0 N
```

实际最大力以传感器、夹具和后续人体安全范围为准，不应为了覆盖示例点强行过载。
每片至少重复三轮，以检查迟滞和重复性。

## 安装到外骨骼之后

桌面标定得到长期的“传感器输出 → N”关系。安装到指尖后不再用推拉力计完成整套曲线，
而是：

1. 每次穿戴重新采集五路无载零点；
2. 检查轻压时对应通道单调增加；
3. 必要时用少量已知力点复核安装、软垫和预紧造成的偏差；
4. 闭环始终同时使用舵机位置终点和电流限制，不能只依赖FSR。

## 与旧代码的复用关系

- 复用旧 `Five_finger_force_test/ble_broker.py` 的 BLE 唯一连接和 TCP 广播机制；
- 复用旧五指力控的完整帧缓存、状态机、总电流限制和异常释放思路；
- 不复用旧 BLE 弯曲手套到 Inspire 的运动映射，运动输入已由 mHandPro 替代；
- 当前首测统一使用固件的 `4.903N` 最低有效阈值，不做减法。
