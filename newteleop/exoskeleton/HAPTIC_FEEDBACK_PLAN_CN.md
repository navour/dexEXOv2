# mHandPro + G1 INSPIRE + 外骨骼力控方案

> 版本：2026-09-01
> 当前主路：INSPIRE 安装在 G1，mHandPro 接 Ubuntu，外骨骼与 FSR 接树莓派
> 历史台架：`newteleop/mhandpro/standalone_inspire_bridge.py` 只用于树莓派网线直连单手，不参与 G1 联调

## 1. 最终架构

```text
左/右 mHandPro（共用一个USB接收器）
  -> Ubuntu: zh/mhandpro/mhandpro_diagnostic
  -> TCP 9103/9104: zh/PC端/dual_arm_viz.py
  -> UDP 9527: G1 zh/机器人端/robot_arm_receiver.py
  -> G1 hand_driver.py 唯一占有左/右 INSPIRE Modbus
       -> TCP 9201/9202 发布 INSPIRE top_touch
       <- TCP 9301/9302 接收逐指力控覆盖

左/右 STM32 FSR -> BLE -> 树莓派 9001/9002
树莓派 dual_hand_force_test.py
  -> /dev/serial0 -> XL330 ID 1..10
  -> 9301/9302 -> G1 逐指 LOCKED/PID 覆盖
```

硬件所有权必须保持：

| 硬件 | 唯一拥有者 |
|---|---|
| mHandPro USB 接收器 | Ubuntu 上的单个 `mhandpro_diagnostic` |
| 右/左 INSPIRE Modbus | G1 `hand_driver.py` |
| 双 BLE FSR | 树莓派 `ble_broker.py` |
| XL330 ID 1～10 | 树莓派 `dual_hand_force_test.py` |

不得同时运行两个能写同一硬件的进程。

## 2. 旧代码保留了什么

参考对象：

```text
ftp/Force_handcontrol.py
ftp/both_Force_handcontrol.py
Five_finger_force_test/finger_force.py
Five_finger_force_test/finger_force_v2.py
```

已复用：

- 逐指 `FREE -> FORCE_ENTRY -> LOCKED -> RELEASE` 状态机。
- INSPIRE 触觉驱动外骨骼收绳，舵机稳定后切模式5锁定。
- `FSR操作者侧力 - INSPIRE机器人侧力` 的 LOCKED 位置 P 调节。
- 单指电流上限、每手五指总电流上限、电流爬升率。
- RELEASE 返回当次穿戴 INIT 位置。
- 断流、通信错误、硬件错误、电流、电压、温度联锁。

已替换：

- 旧 BLE 弯曲手套的动作输入已完全替换为 mHandPro。
- 旧 Pi -> G1 DDS/TCP 手桥已替换为 `zh` 的 UDP 遥操尾块和 G1 原生 Modbus 手驱动。
- 旧程序一个进程同时拥有手套、INSPIRE、BLE 和舵机的高耦合结构已拆开。

## 3. 力的定义

软件统一指序：

```text
[0,1,2,3,4] = [拇指,食指,中指,无名指,小指]
```

INSPIRE 六角度槽指序是 `[小,无,中,食,拇弯,拇对掌]`，因此五指对应
角度槽 `[4,3,2,1,0]`。

两类力：

- `F_robot`：G1 上 INSPIRE top_touch 经实验线性式换算的机器人指端力。
- `F_operator`：外骨骼 FSR 数据。INIT 采集绑带静态预载，闭环使用新增力。

现有 STM32 输出存在约 4.903 N 低端，因此代码的反馈量为：

```text
fsr_excess = max(0, fsr_raw - fsr_init_preload - preload_deadband)
F_operator = 0                         , fsr_excess == 0
F_operator = 4.903 + fsr_excess        , fsr_excess > 0
```

这不等于高精度拉绳张力。FSR 对预紧、接触面积和手指曲率敏感，
可以完成当前人手侧力闭环和释放判断，但需要逐指标定和重复性验收。
如需要精确的拉绳力，应在每根拉绳上串联张力传感器。

## 4. 状态机

```text
STOP
  | INIT只读采集返回位和FSR预载，人工ARM
  v
FREE
  | F_robot >= contact_on
  v
FORCE_ENTRY
  | 模式0限流收绳；位置稳定
  v
LOCKED
  | 外骨骼模式5锁位
  | 树莓派向G1发 position_step = clamp(Kp*(F_operator-F_robot))
  | INSPIRE触觉释放，或FSR已加载后持续卸载
  v
RELEASE
  | 模式0反向电流返回init_pos
  v
RETURN_SETTLE
  | 模式5保持，位置与INSPIRE释放连续稳定
  +----> FREE
```

FORCE_ENTRY 外骨骼请求电流：

```text
I_request = min(F_robot * force_to_current_gain, max_goal_current)
```

当前双手联调值为 `60 raw/N`、最大 `300 raw`，每手收绳电流总和上限
`800 raw`。这是已进入实物试验的参数，不是任意新机构的安全默认值。

## 5. 跨机协议

### G1 -> Pi，9201/9202

换行 JSON：

```json
{
  "type": "inspire_feedback",
  "hand": "right",
  "seq": 1,
  "top_touch_order": ["拇指", "食指", "中指", "无名指", "小指"],
  "top_touch_raw_max": [0, 0, 0, 0, 0],
  "top_touch_force_n": [0.0, 0.0, 0.0, 0.0, 0.0],
  "touch_valid": true
}
```

### Pi -> G1，9301/9302

```json
{
  "type": "haptic_override",
  "finger_states": ["FREE", "LOCKED", "FREE", "FREE", "FREE"],
  "finger_steps": [0.0, 2.0, 0.0, 0.0, 0.0]
}
```

G1 的 0.5 s 覆盖看门狗超时后清空所有 LOCKED 状态，恢复手套跟随。

## 6. 安全联锁

- 上电默认 STOP；没有 `--enable-write` 不能 ARM。
- 两手的舵机、FSR、INSPIRE 数据和 INIT 必须全部就绪才能双手 ARM。
- 任一指 Dynamixel 通信、硬件错误、实际电流、电压或温度越界会联锁十指 STOP。
- FSR 和 INSPIRE 任一数据超时会 FAULT。
- RELEASE 不因新接触中断，必须先完成安全返回。
- 9301/9302 断开只会让 G1 恢复 mHandPro 跟随，不允许留下锁定角度。
- SIGINT、SIGTERM、正常退出和异常共用清电流/关扭矩路径。
- 软件 STOP 不代替物理断电。

## 7. 分阶段验收

1. 双 BLE/FSR 只读：逐指确认左右手和五通道。
2. G1 双 INSPIRE 触觉只读：9201/9202 帧率、指序、空载与按压可分。
3. 未穿戴单指：从低电流验证收绳方向、锁定、释放和断电。
4. 未穿戴单手五指：逐指后再多指，验证每手总电流缩放。
5. 未穿戴双手：单进程 ID 1～10，任一故障十指联锁。
6. mHandPro + 双 INSPIRE + 双外骨骼：先逐指，再单手多指，最后双手。
7. 只有手部闭环稳定后才加 G1 手臂，腰部最后加。

具体命令和停机顺序见 [`../README.md`](../README.md)。

## 8. 当前验证状态

代码/无硬件已验证：

- 双 BLE broker 的左右 MAC 与 9001/9002 参数组合。
- 双手 ARM 前置拒绝和明确错误输出。
- FSR 必须先加载再卸载的两阶段滞回。
- G1 Modbus 帧、手部换算、断流张手和 Haptic Override 纯逻辑测试。
- Python 语法检查和只读上位机 HTTP 接口。

仍需实物分阶段验证：

- 十个舵机在 10 Hz 长时运行时的TTL丢包、供电压降和温升。
- 双手 FSR 在实际穿戴预紧下的逐指重复性和交叉耦合。
- `300 raw`、每手 `800 raw` 在实际绳路上的人体舒适边界。
- 双手同时接触、逐指解锁和各类断线注入。
