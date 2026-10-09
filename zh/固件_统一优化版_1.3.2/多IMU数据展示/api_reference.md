# MultiImuService API Reference

> 本文档为 `multi_imu_core.py` 中 `MultiImuService` 类的完整 API 说明，面向程序/Agent 调用。

## 快速开始

```python
from multi_imu_core import MultiImuService

svc = MultiImuService()
svc.start()          # 启动发现/数据/连接线程
# ... 使用 API ...
svc.stop()           # 停止所有线程
```

---

## 数据模型

### `DeviceState`

每个已发现设备的状态快照 (dataclass):

| 字段 | 类型 | 说明 |
|------|------|------|
| `node_id` | `str` | MAC 派生的节点标识 (6 位十六进制, 如 `"9495B4"`) |
| `ip` | `str` | 设备 IP 地址 |
| `tcp_port` | `int` | TCP 命令端口 (默认 4210) |
| `udp_port` | `int` | UDP 数据端口 (默认 4211) |
| `device_id` | `str` | 用户自定义设备 ID (NVS 持久化, 可为空) |
| `last_seen` | `float` | 最后一次收到数据的 `time.time()` 时间戳 |
| `connected` | `bool` | TCP 连接是否建立 |
| `quat` | `List[float]` | 当前四元数 `[w, x, y, z]`, 归一化 |
| `rest` | `bool` | 设备是否处于静止状态 |
| `seq` | `int` | 最新 UDP 包序号 (-1 表示未收到数据) |
| `battery_voltage` | `Optional[float]` | 电池电压 (V), 无电池信息时为 `None` |
| `battery_percent` | `Optional[int]` | 电池百分比 (0-100), 无电池信息时为 `None` |
| `key_voltage` | `Optional[float]` | 按键检测电压 |
| `key_pressed` | `bool` | 按键是否按下 |
| `power_source` | `str` | 供电来源: `"BAT"` / `"USB"` / `"UNKNOWN"` |
| `charging` | `bool` | 是否正在 USB 充电 |
| `cal_state` | `str` | 校准状态: `"idle"` / `"start"` / `"progress"` / `"computing"` / `"done"` / `"fail"` / `"stopped"` / `"erased"` |
| `cal_message` | `str` | 最新校准/电源相关原始消息文本 |

---

## API 方法

### 生命周期

#### `start() -> None`

启动后台线程 (UDP 发现、UDP 数据接收、TCP 连接维护)。多次调用安全，重复调用不产生新线程。

#### `stop() -> None`

停止所有后台线程，关闭所有 TCP 连接。

---

### 设备发现与选择

#### `list_devices() -> List[DeviceState]`

返回当前所有已发现设备的快照列表，按 `node_id` 排序。  
返回的是深拷贝，修改返回值不影响内部状态。

```python
devices = svc.list_devices()
for d in devices:
    print(f"{d.node_id} ({d.device_id or 'unnamed'}) @ {d.ip} connected={d.connected}")
```

#### `select_device(node_id: str) -> bool`

选择当前操作设备。成功返回 `True`，设备不存在返回 `False`。

#### `get_selected() -> Optional[DeviceState]`

获取当前选中设备的状态快照。未选中或设备已失联返回 `None`。

---

### 磁力计校准

#### `send_cal_start(node_id: str) -> bool`

发送校准开始命令。设备将开始采集磁力计数据。  
进度通过 `DeviceState.cal_state` 和 `cal_message` 更新。

#### `send_cal_stop(node_id: str) -> bool`

取消正在进行的原始样本采集，不计算、不保存参数，原有 NVS 参数保持不变。

#### `send_cal_set(node_id: str, hard_iron, soft_iron, field_norm) -> bool`

把上位机完整三维椭球拟合得到的 `hard_iron[3]`、`soft_iron[3][3]` 和磁场模长可靠发送给设备。设备校验通过后写入 NVS、立即应用并重置 VQF。需要设备固件 `1.3.0` 或更高。

#### `send_cal_erase(node_id: str) -> bool`

擦除设备 NVS 中保存的校准数据。

---

### 电源控制

#### `send_shutdown(node_id: str) -> bool`

发送远程关机命令。设备将执行安全关机流程。

---

### 设备 ID 管理

#### `set_device_id(node_id: str, device_id: str) -> bool`

设置设备自定义 ID。该 ID 保存在设备 NVS 闪存中，**断电后保持**。

- `device_id`: 最长 32 字符的字符串
- 传入空字符串 `""` 可清除 ID
- 设置成功后, 设备发现广播会包含新 ID

```python
svc.set_device_id("9495B4", "left_hand")
```

#### `get_device_id(node_id: str) -> bool`

请求设备返回当前 ID。结果异步通过 `DeviceState.device_id` 字段更新。

```python
svc.get_device_id("9495B4")
time.sleep(0.5)  # 等待响应
d = svc.get_selected()
print(d.device_id)  # "left_hand"
```

---

### OTA 固件更新

#### `ota_update(node_id: str, firmware_path: str, timeout: float = 60.0) -> bool`

通过 WiFi OTA 推送固件到指定设备。

- `firmware_path`: 编译产出的 `.bin` 固件文件路径
- `timeout`: HTTP 传输超时秒数 (默认 60s)
- 返回 `True` 表示上传成功，设备将自动重启到新固件
- 返回 `False` 表示上传失败 (设备离线/网络错误/固件无效)

```python
success = svc.ota_update("9495B4", "build/vqf_esp32.bin")
if success:
    print("OTA 成功, 设备重启中...")
    time.sleep(5)  # 等待设备重启
```

**注意**: OTA 期间设备会暂时停止 IMU 数据发送。更新完成后设备自动重启并恢复正常工作。

#### `get_ota_url(node_id: str) -> Optional[str]`

获取设备 OTA 上传 URL。可用于手动 HTTP POST 上传或浏览器访问。

```python
url = svc.get_ota_url("9495B4")
# 返回: "http://192.168.1.100:8080/update"
```

也可通过浏览器访问 `http://<设备IP>:8080/` 查看上传页面。

---

## 通信协议概要

| 通道 | 端口 | 方向 | 用途 |
|------|------|------|------|
| UDP 广播 | 4212 | ESP32 → PC | 设备发现 (`VQF_DISC,...`) |
| UDP 数据 | 4211 | ESP32 → PC | 四元数 (Q15 二进制) + 文本消息 |
| TCP 命令 | 4210 | PC → ESP32 | 命令发送 (`$CMD,...`) |
| HTTP OTA | 8080 | PC → ESP32 | 固件上传 (POST /update) |

### TCP 命令列表

| 命令 | 说明 | 响应 |
|------|------|------|
| `$CMD,CAL_START\n` | 开始磁力计校准 | `$CAL,START,<total>,<sec>` |
| `$CMD,CAL_STOP\n` | 取消采集且不修改参数 | `$CAL,CANCELLED` |
| `$CMD,CAL_SET,<3+9+1个浮点数>\n` | 保存完整 3×3 校准参数 | `$CAL,SET_OK,<field_norm>` 或 `$CAL,FAIL,<reason>` |
| `$CMD,CAL_ERASE\n` | 擦除校准数据 | `$CAL,ERASED` |
| `$CMD,OFF\n` | 远程关机 | (设备关机) |
| `$CMD,SET_ID,<id>\n` | 设置设备 ID | `$ID,OK,<id>` 或 `$ID,FAIL,<reason>` |
| `$CMD,GET_ID\n` | 查询设备 ID | `$ID,<id>` (ID 为空时返回 `$ID,`) |

### 发现广播格式

```
VQF_DISC,node=<MAC6>,id=<device_id>,ip=<IP>,tcp=4210,udp=4211
```

`id=` 字段仅在设备已设置自定义 ID 时出现。
