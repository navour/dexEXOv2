# 完整椭球校准协议扩展

适用固件：`1.3.2` 及更高版本。本统一版同时保留向后兼容的 12 字节
姿态包前缀，并使用 22 字节扩展包发送融合时间戳和三轴陀螺仪。

## 开始采集

```text
$CMD,CAL_START\n
```

设备以约 20 Hz 返回：

```text
$CAL,SAMPLE,<mx>,<my>,<mz>
```

当前 ICM-20948 配置下磁力计单位为 µT。

## 取消采集

```text
$CMD,CAL_STOP\n
```

回复：

```text
$CAL,CANCELLED
```

此命令不会修改 NVS，也不会调用旧的对角拟合。

## 下发完整参数

```text
$CMD,CAL_SET,<hi_x>,<hi_y>,<hi_z>,<m00>,<m01>,<m02>,<m10>,<m11>,<m12>,<m20>,<m21>,<m22>,<field_norm>\n
```

校正公式：

```text
calibrated = soft_iron @ (raw - hard_iron)
```

ESP32 拒绝以下参数：

- 数量不足、存在额外字段或无法解析；
- NaN、无穷大或超出允许范围；
- 矩阵不对称或不是正定矩阵；
- 矩阵行列式或磁场模长超出安全范围。

失败回复：

```text
$CAL,FAIL,PARAM_PARSE_ERROR
$CAL,FAIL,PARAM_VALIDATE_ERROR
$CAL,FAIL,NVS_SAVE_ERROR
```

成功后依次回复：

```text
$CAL,SET_OK,<field_norm>
$CAL,PARAMS,<3个偏置>,<9个矩阵元素>,<field_norm>
```

成功时设备会停止采集、写入 NVS、切换到新参数并重置 VQF 状态。
