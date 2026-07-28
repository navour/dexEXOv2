# mHandPro + Inspire + 外骨骼力反馈实施方案

## 1. 结论先行

### 1.1 最终接线

```text
mHandPro USB接收器 ──USB──► 树莓派
Inspire左手 ──网线──► 树莓派 eth0 (192.168.123.100/24)
左右外骨骼XL330 ──TTL转接板──► 树莓派 /dev/serial0
开发电脑 ──Wi-Fi/SSH──► 树莓派 wlan0 (192.168.3.76)
```

集成运行时 Inspire 网线接树莓派，电脑只做 SSH 管理、日志查看和代码复制。
只有回到 Ubuntu 电脑单独调试 Inspire 时，才临时把网线接回电脑并配置
`192.168.123.100/24`。

### 1.2 当前能做什么

当前可以实现“有界力提示”：

```text
Inspire五指 top_touch 阵列接触力
  → 基线扣除/滤波/归一化
  → XL330模式5（有位置终点的限流收绳）
  → 操作者感到手指继续弯曲受阻
```

它是受限的阻抗/馈通式力反馈，不是“操作者手指实际受力”的真正闭环。

### 1.3 为什么当前不是真正闭环

系统中存在两个已有内环：

1. Inspire 可以使用自身指端传感器完成机器手接触/力控。
2. XL330 内部可以闭环控制电流、位置或“限流位置”。

但系统没有测量以下量：

```text
外骨骼拉绳对操作者手指的实际力
```

mHandPro 提供姿态/骨骼旋转，不提供指尖力。XL330 的实际电流只能粗略反映电机转矩，
还混有齿轮摩擦、卷线轮半径、拉绳摩擦、惯性和手指位姿影响，不能直接当成准确的手指力。

---

## 2. 旧力控代码如何复用

可参考的旧实现：

```text
../../ftp/Force_handcontrol.py
../../ftp/both_Force_handcontrol.py
../../Five_finger_force_test/finger_force.py
../../Five_finger_force_test/finger_force_v2.py
```

可复用的思路：

- 五指独立状态机：`GLOVE/FREE → FORCE_ENTRY/CONTACT → LOCKED/HOLD → RELEASE`。
- 目标力和反馈力的低通滤波、死区和滞回。
- 单舵机电流上限、五指总电流上限。
- 释放回佩戴初始位置。
- 断线、Ctrl+C 和异常退出时清零/关扭矩。
- Inspire 六通道与五个外骨骼舵机的映射。

不能直接复制的部分：

- 旧 BLE 手套的弯曲和指尖/外骨骼触觉解析。
- 依赖旧手套五路力值的 PID 外环。
- 未经当前机械验证的正负号、初始位置和电流参数。
- 旧代码中 150～450 级别的电流参数；不得直接用于首次佩戴。

---

## 3. 新运行架构

### 3.1 进程与端口

```text
mhandpro_diagnostic
  └──六维角度JSON──TCP 127.0.0.1:9102──► standalone_inspire_bridge
                                                        ├──Modbus写ANGLE_SET──► Inspire
                                                        └──Modbus读五组top_touch
                                                                   │
                                                                   ▼
                                                 本机力数据流 127.0.0.1:9202
                                                                   │
                                                                   ▼
                                                   force_control/left_index_force_test
                                                                   │
                                                          /dev/serial0
                                                                   ▼
                                                        XL330 ID 6～10
```

设计约束：

- `standalone_inspire_bridge` 是 Modbus TCP 的唯一拥有者，同时写角度并读力。
- `force_control/left_index_force_test.py` 是 Dynamixel 串口的唯一拥有者。
- 力数据必须含时间戳和序号，不允许用旧数据持续施力。
- 运行力反馈时不能同时运行点动、电流脉冲或其他 Dynamixel 工具。

### 3.2 映射

```text
Inspire top_touch摘要: [拇指, 食指, 中指, 无名指, 小指]

拇指       channel 0 → 左手ID 6
食指       channel 1 → 左手ID 7
中指       channel 2 → 左手ID 8
无名指   channel 3 → 左手ID 9
小指       channel 4 → 左手ID 10
```

每指对应一组12x8 top_touch阵列，首版复用旧代码取各阵列最大值。

---

## 4. 近期方案：有边界的开环/阻抗式力提示

### 4.1 Inspire 信号处理

对每个通道采集空载基线 `b` 和噪声，然后计算：

```text
delta = raw - baseline
```

接触阈值不写死，由实测得到：

```text
contact_on  > 空载噪声上界
contact_off < contact_on             # 滞回，防止反复开关
force_full  = “明显但安全的指尖接触”实测值
```

归一化强度：

```text
s = clamp((delta - contact_on) / (force_full - contact_on), 0, 1)
s_filtered = s_filtered + alpha * (s - s_filtered)
```

初期 `alpha` 取 0.1～0.2，阈值必须使用
`tools/inspire_force_monitor.py` 的实测数据确定。

### 4.2 外骨骼执行策略

首版使用 XL330 模式 5（Current-based Position Control）：

- 目标位置限制“最多收多少绳”。
- Goal Current 限制“最大允许转矩”。
- 释放时返回当次佩戴的 `neutral_position`。
- 回到基准附近后关闭扭矩。

不使用模式 0 作为首版佩戴控制，因为纯电流模式没有位置终点，空载小电流也会持续卷绳。

### 4.3 每指状态机

```text
FREE
  扭矩关闭，位置在neutral附近
    │ top_touch换算力连续多帧高于contact_on
    ▼
CONTACT_ENTRY
  模式5，先写当前位置，电流从0缓慢爬升
    ▼
HOLD
  目标位置不超过neutral + max_travel
  电流限制随s_filtered缓慢变化
    │ top_touch换算力持续低于contact_off
    ▼
RELEASE
  低电流返回neutral
    │ 进入neutral容差
    ▼
FREE（关扭矩）
```

---

## 5. 真正力闭环方案

### 5.1 推荐传感器位置

优先在每根外骨骼拉绳上串联小型张力/拉力传感器，而不是改造 mHandPro 指尖：

```text
XL330卷线轮 ── 拉绳 ── 张力传感器 ── 外骨骼手指
```

优点：

- 直接测量外骨骼施加的拉力。
- 不占用 mHandPro 指尖空间，不影响姿态测量。
- 不依赖操作者指尖是否正好压在某个触觉片上。
- 可对每根绳独立标定牛顿值。

指尖 FSR/薄膜压力传感器可作为次选，但它对接触位置、预紧、温漂和手指曲率更敏感。

### 5.2 真闭环结构

```text
Inspire指端力 → 舒适缩放/限幅 → 操作者目标力 F_target
                                              │
                                              ▼
                           error = F_target - F_tendon_measured
                                              │
                                   PI/PID + 反饱和 + 速率限制
                                              │
                                              ▼
                                   XL330 Goal Current/目标位置
```

不建议机器端和人手端做 1:1 力复制。应设置舒适缩放、起感阈值、最大人手力和力变化率。

---

## 6. 分阶段实施与验收

### 阶段 A：Inspire 触觉只读标定（当前下一步）

1. 外骨骼扭矩保持关闭。
2. 运行 `mhandpro/tools/inspire_force_monitor.py`。
3. 采集六路空载基线和噪声。
4. 分别轻压小指、无名指、中指、食指和拇指。
5. 记录轻触、中等接触和释放值。

验收：通道映射明确，接触与空载可分，释放后能回到基线附近。

### 阶段 B：桥接力数据输出

1. 扩展 `standalone_inspire_bridge.py`，以5 Hz读取五组 `top_touch` 阵列并发布摘要。
2. 在树莓派本机发布含序号、时间戳和六路 raw 的 JSON。
3. 力读取失败不影响桥接执行安全张手，但必须通知力控程序进入 FAULT。

验收：遥操灵巧手时能同时稳定收到力数据，无数据时不会留下旧力命令。

### 阶段 C：未穿戴的左食指模式5安全执行器

1. 只启用左食指 ID 7。
2. 加载 `left_neutral.json`。
3. 确认 ID 7 正位置方向为收绳。
4. 从非常小的 `max_travel` 和 Goal Current 开始。
5. 用人工注入的 0～1 强度信号验证收绳、保持、释放和 STOP。

验收：位置永远不越过软边界，通信中断和 Ctrl+C 会清零并关扭矩。

### 阶段 D：佩戴左食指低强度实验

1. 重新采集当次佩戴的 neutral。
2. 只打开 ID 7，其他舵机保持扭矩关闭。
3. 先用人工强度，再接 Inspire 食指 `fingerfour_top_touch`。
4. 每次只提高一档电流或行程，不同时改两个参数。
5. 反复验证 STOP、拔网线、停止手套数据和松开物体。

验收：只在 Inspire 食指接触时感到柔和阻力，释放时及时放松，任意断线不会持续收绳。

### 阶段 E：逐指扩展

1. 依次验证 ID 8、9、10、6 的收放方向和位置边界。
2. 每增加一指都重做单指断线/释放验证。
3. 最后才开启五指总电流限制和同时接触。

### 阶段 F：加入张力传感器的真闭环

1. 为每根拉绳安装并标定张力传感器。
2. 设置硬件过力保护，不只依赖 Python。
3. 从 P 控制开始，确认稳定后再加 I；非必要不加 D。
4. 实测不同手指姿态下的绳力与人体舒适度。

---

## 7. 必须实现的安全看门狗

- Inspire top_touch 超时：进入 RELEASE/FAULT。
- mHandPro 姿态帧超时：进入 RELEASE/FAULT。
- Dynamixel 通信失败：目标电流清零，尝试关扭矩。
- 位置超过 `neutral + max_travel + tolerance`：立即 FAULT。
- 实际电流、温度或电压异常：立即 FAULT。
- 力命令爬升率限制：不允许从 0 一帧跳到最大值。
- 单指和五指总电流限制。
- 必须有 `ARM`/`STOP` 状态，默认上电是 STOP。
- SIGINT、SIGTERM、异常和正常退出共用同一个清理路径。
- 调试时保持物理断电手段在手边；软件 STOP 不代替硬件断电。

---

## 8. 当前已验证与待验证

已验证：

- 树莓派可同时连接 Inspire、mHandPro 和 10 个 XL330。
- 左手 ID 6～10 只读 Ping 正常。
- 左食指 ID 7 正位置方向为收绳，模式3点动可返回并恢复模式5。
- 左手佩戴零力基准已稳定采集。

待验证：

- Inspire 五组top_touch阵列的空载噪声、接触位置和有效范围。
- 左手 ID 6、8、9、10 的收放方向。
- 模式5下安全的最小可感电流和行程。
- 力数据断线后的自动释放。
- 加入张力传感器后的人手侧真力闭环。
