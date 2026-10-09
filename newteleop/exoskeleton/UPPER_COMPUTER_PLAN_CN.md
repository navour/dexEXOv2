# 外骨骼本地上位机方案

> 版本：2026-09-01  
> 对象：双手 FSR、两路 BLE、G1 INSPIRE、Dynamixel XL330 ID 1～10  
> 正式实现：树莓派本地 PyQt5 + pyqtgraph，不使用 Tk 或网页控制

## 1. 结论

上位机使用 [`local_supervisor.py`](supervisor/local_supervisor.py)。GUI 和
`DualHandController` 在同一进程，不会为界面再打开一个串口或再创建
一套力控状态机。

```text
右/左 STM32 FSR --BLE--> ble_broker.py --TCP 9001/9002--+
                                                            |
G1 INSPIRE top_touch ---------------TCP 9201/9202-----------+--> DualHandController
G1 力控覆盖 <----------------------TCP 9301/9302-----------+          |
                                                                       +-- /dev/serial0 --> ID1..10
                                                                       |
                                                                       +-- 内存遥测快照
                                                                                  |
                                                                                  +--> PyQt 界面
```

安全命令直接调用已有的
`status/init_all/arm_all/stop_all/close`，而不是向另一个进程发不可见的远程
命令。GUI 关闭时先 STOP，再关闭 TTL 和 GUI 自己启动的 BLE broker。

## 2. 交互和视觉

视觉参考 `zh/PC端/dual_arm_viz.py` 与 `zh/mhandpro/mhandpro_studio.py`
的深色 HUD：深蓝背景、青色主信息、绿/黄/红表示正常/警告/故障。
界面的信息层级为：

1. 顶栏：全局 `STOP/PARTIAL/ARMED/FAULT`、写入授权、时间。
2. 左轨：`1 舅机 → 2 BLE → 3 G1`，ARM 就绪清单和红色 STOP。
3. 工作区：右/左手各五指 FSR 压力条、INSPIRE 力和状态机。
4. 详情页：十舅机表、30 s 实时曲线、完整日志。

结构按实物保持固定映射：

| 手别 | 拇指 | 食指 | 中指 | 无名指 | 小指 |
|---|---:|---:|---:|---:|---:|
| 右手 | ID 1 | ID 2 | ID 3 | ID 4 | ID 5 |
| 左手 | ID 6 | ID 7 | ID 8 | ID 9 | ID 10 |

建议在舅机箱上贴相同 ID 标签，不按外壳颜色猜测左右手。

## 3. 实时数据

每只手显示：

- BLE/FSR 端到端连接、数据年龄和帧率。
- INSPIRE 触觉连接、数据年龄和帧率。
- 9301/9302 mHandPro 力控覆盖是否启用和连接。
- 五指 FSR 原始力、INIT 预载、相对新增力和 INSPIRE 力。
- `FREE/FORCE_ENTRY/LOCKED/RELEASE/RETURN_SETTLE/FAULT` 状态。

每个舅机显示：

- ID、手别、手指、INIT 位置和实时位置。
- Goal Current、Present Current、电压、温度。
- 运行模式、扭矩开关、硬件错误字、健康数据年龄和通信错误。

“BLE/FSR 正常”表示 broker TCP 存在，且最近 0.5 s 内仍有 BLE
Notification 被解析为有效 FSR 帧。它是端到端健康指示，当前不包含 RSSI。

## 4. 安全边界

1. 界面 5 Hz 刷新只读缓存，不额外读 TTL 寄存器。
2. INIT/ARM 都有二次确认；ARM 还必须通过所有链路、INIT 和 FAULT 门控。
3. 慢速连接与硬件命令在工作线程执行，避免 Qt 事件循环卡死。
4. `Esc` 随时调用原力控 `stop_all()`，但不代替实体急停/断电。
5. G1 按钮只启动树莓派的数据客户端，不远程启动或复位机器人。

## 5. 已实现与后续项

已实现：两手状态卡、十指压力条、十舅机表、FSR/电流曲线、日志、
BLE 子进程管理、G1 数据连接、五个安全命令与 ARM 门控。

后续可选：

- 实验参数与 FAULT 事件 JSONL/CSV 落盘。
- 点击某舅机展开本轮状态转移和故障上下文。
- broker 发布重连次数，在底层稳定支持时增加 RSSI，但 RSSI 不参与 ARM。
- 树莓派 `.desktop` 桌面启动器与开机后手动确认启动。

安装、启动和操作见 [`LOCAL_SUPERVISOR_CN.md`](LOCAL_SUPERVISOR_CN.md)。
