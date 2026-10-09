# 无线 IMU 软件统一优化版 1.3.2

此目录是独立版本，原来的 `SOFTWARE/`（优化固件 1.2.5）和
`SOFTWARE_完整椭球校准版/`（完整校准固件 1.3.1）均保持不变。

## 合并内容

- ESP32 固件版本 `1.3.2`。
- ICM-20948 陀螺仪 DLPF_CFG=3，约 51.2 Hz。
- VQF 运动偏置估计默认启用，可通过构建选项关闭做 A/B 测试。
- 姿态 UDP 包保持旧 12 字节前缀，并追加采样时间戳与三轴陀螺仪，
  总长度 22 字节；旧上位机仍可读取前 12 字节。
- 上位机执行鲁棒完整三维椭球拟合，通过 `CAL_SET` 下发完整 3×3
  软铁矩阵。
- ESP32 校验参数后写入 NVS、立即应用，并在 IMU 任务中安全重置 VQF。
- `CAL_STOP` 只取消采集，不再执行旧的对角软铁拟合。
- AK09916 的 `ST2.HOFL` 溢出样本不会进入校准或 VQF。

## 构建固件

```bash
export IDF_PYTHON_ENV_PATH=/home/hong/.espressif/python_env/idf5.4_py3.10_env
source /home/hong/esp-idf-v5.4.2/export.sh
cd /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/ESP32C3固件
idf.py -B build_unified_1_3_2 build
```

固件产物：

```text
ESP32C3固件/build_unified_1_3_2/vqf_esp32.bin
```

## 有线烧录

确认目标设备串口后执行：

```bash
idf.py -B build_unified_1_3_2 -p /dev/ttyACM0 flash monitor
```

按 `Ctrl+]` 退出监视器。

## 完整椭球校准

```bash
cd /home/hong/unitree/无线接收/SOFTWARE_统一优化版_1.3.2/多IMU数据展示
python3 -m pip install -r requirements.txt
python3 multi_imu_dashboard.py
```

1. 完整装配设备并使用电池供电，保持 WiFi 和 LED 为正常工作状态。
2. 选择设备，确认界面显示固件 `1.3.2`。
3. 按 `C` 开始采集。
4. 缓慢连续翻转 15～25 秒，覆盖六个面和斜方向。
5. 等界面显示绿色“数据质量已达标”。
6. 按 `X`，上位机拟合并下发完整参数。
7. 看到“完整校准已保存”后断电重启，确认参数仍然有效。

`Esc` 只取消本次采集，不修改原参数。`E` 会清除 NVS 中已有的校准，
除非准备重新校准，否则不要按。

协议详见 `PROTOCOL_完整椭球校准.md`。
