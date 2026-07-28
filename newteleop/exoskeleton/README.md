# 外骨骼力反馈测试与集成

完整的力反馈架构、开环/真闭环选择、旧代码复用和分阶段实施顺序见
[`HAPTIC_FEEDBACK_PLAN_CN.md`](HAPTIC_FEEDBACK_PLAN_CN.md)。

本目录专门放置操作者侧外骨骼、Dynamixel XL330 舵机、拉绳和力反馈相关代码。

`mHandpro/` 只保留 mHandPro 手套 SDK、姿态解算、标定及 Inspire 灵巧手映射代码。

## 当前硬件

- Dynamixel XL330-M288-T × 5
- Protocol 2.0，1 Mbps
- 树莓派串口：`/dev/serial0` → `/dev/ttyAMA0`
- 右手舵机 ID：1～5（ID 1 拇指、2 食指、3 中指、4 无名指、5 小指）
- 左手舵机 ID：6～10（ID 6 拇指、7 食指、8 中指、9 无名指、10 小指）
- 当前优先调试左手；工具必须显式选择 `--hand left`。

## 诊断工具

- `tools/exo_dynamixel_probe.py`：只读 Ping 和状态检查，不开启扭矩。
- `tools/exo_single_servo_jog.py`：单舵机小幅点动，低 PWM、低速、自动回原位并关闭扭矩。

当旧力控程序已将舵机设为电流模式 0 时，点动必须显式使用
`--temporary-position-mode`。工具只会在扭矩已关闭且硬件无错时临时切换为位置模式 3，
测试结束后自动恢复原模式。

首次使用 `--delta 20 --pwm 50`。若空载下因机械摩擦无法观察，可使用
`--delta 80 --pwm 100`；工具仍会限制在 100 tick 和约 11.4% PWM 以内。

## 与原力控代码的关系

仓库原有五指力控主实现位于 `../Five_finger_force_test/`（即旧 `dexEXO` 仓库），其核心是五指独立 PID 电流闭环。该代码的 BLE 手套输入与现在的 mHandPro 不同，后续仅迁入已验证的执行器、状态机和安全限制。

## 安全顺序

1. 只读检查 5 个舵机。
2. 外骨骼不穿戴，分别确认每个舵机的收绳/放绳方向。
3. 确认位置软限位、电流限制和断线扭矩关闭。
4. 穿戴后从单指、小力度开始闭环。
5. 再启用五指与 Inspire 触觉反馈联动。
