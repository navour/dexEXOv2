# 左食指力反馈首测

`left_index_force_test.py` 仅控制左食指 Dynamixel ID 7，用于验证：

```text
Inspire fingerfour_top_touch[96] -> 取最大值 -> 旧系统换算公式 -> 目标绝对力
STM32 FSR -> 4.903N阈值开关 -> 反馈绝对力
目标力-反馈力 -> 受限Goal Current -> ID 7拉绳
```

FSR处理与原 `ftp/Force_handcontrol.py` 一致：

```python
measured = 0.0 if raw <= 4.903 else raw
```

4.903N是STM32查表的最低有效输出，不是需要减去的零偏。因此本程序只能
测试约5N以上的力，不能识别0～4.9N。

详细四终端启动顺序见上级 `README.md`。首次先不带 `--enable-write` 运行并输入
`STATUS`。写入测试必须外骨骼未穿戴，并准备物理断电：

```bash
cd ~/cnn/newteleop
./.venv/bin/python exoskeleton/force_control/left_index_force_test.py --enable-write
```

输入 `ARM` 后，ID 7保持关扭矩待机。Inspire食指换算力达到4.95N才启动；力消失后
自动回neutral并关扭矩。

Dynamixel位置每4096 tick回绕一圈。程序会将读到的位置映射到最接近neutral的
等效圈后再执行启动、限位和回中判断。例如 `position_raw=2692` 与 `neutral=6787`
经归一化后为 `position_norm=6788`，实际只相1 tick。
