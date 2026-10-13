# dexEXO 全仓库架构与新包边界

> 更新：2026-10-09。目录、文档与关键链路源码交叉核对；不将二进制、缓存和历史文档中的“已实测”视作本次真机验证。

## 新旧链路

| 版本 | 运动输入 | G1 INSPIRE 接口 | 操作者反馈 |
|---|---|---|---|
| ftp V4 | BLE 紧凑18项帧的前13项弯曲量 | TCP 9100/9102 → DDS → Headless → Modbus | 后5项FSR + XL330 |
| newteleop/mhandpro 历史台架 | mHandPro 单左手 | Pi standalone bridge 直连 Modbus | 左食指/左手实验 |
| zh 当前参考 | 双 mHandPro → PC 闭合度 | UHND UDP → G1 hand_driver 原生 Modbus | 9201/9202触觉、9301/9302覆盖 |
| newteleop 外骨骼 | 使用 zh 手套与上位遥操 | 连接 zh 的 G1 触觉/覆盖服务 | 双 FSR、十舵机、PyQt/CLI 共用控制器 |

原 BLE 板仍采集 FSR，但前13项不再作为 INSPIRE 动作输入。G1 手臂/腰部 DDS 与 INSPIRE Modbus 是两条独立执行链。

## 各目录职责

| 根目录 | 作用与本次处理 |
|---|---|
| `newteleop/` | 当前手部力控包；本次修改范围 |
| `zh/PC端` | 最新 IMU、双臂/腰部可视化、双手闭合度发送；完整身体遥操仍使用它 |
| `zh/机器人端` | 现有 G1 接收器与驱动，保留原文件 |
| `zh/mhandpro` | 新双手 SDK 采集、拇指标定/UV、配置来源 |
| `zh/仿真` | MuJoCo、G1模型与动作验证，不是实时手部硬件依赖 |
| `zh/HumDex_IMU融合实验` | 全身参考/策略接口实验，非本次手部入口 |
| `zh/固件_统一优化版_1.3.2` | ESP32无线IMU固件、协议和多IMU工具 |
| `zh/inspire_hand_ws` | INSPIRE/Unitree SDK参考；原生Modbus手链不启动其Headless |
| `ftp/` | V1～V4旧手套双边遥操；借鉴状态机与README组织方式 |
| `Five_finger_force_test/` | 旧五指控制、BLE broker、参数实验 |
| `Finger_force_test/` | 单指/早期力控与DDS触觉转发 |
| `Finger_force_test1.26/` | 历史快照 |
| `xl330_force_demo/`、`dxl_bridge/` | 舵机电流、位置、串口硬件实验 |
| `sensors/` | BLE采集、记录和网络转发实验；与当前18项紧凑帧格式不同 |
| `STM32G431CBT6-0119/` | 传感器底层固件、HAL/驱动/工程文件 |
| `gazebos/` | Gazebo手部模型、桥接和关节映射 |
| `shiloh/` | sensors/gazebos交付副本与归档 |
| `宇树双臂遥操系统分享包/` | zh双臂系统的早期PC、Pi和G1实现 |
| `VR/` | Pico4U/机器人相机链路说明 |
| `tools/` | 通用舵机监测工具 |
| `mHandPro_LinuxSDK/` | 厂商原始分架构库 |
| `docs/` | 旧架构与交接历史，部分“当前”描述对应ftp时代 |
| `source/` | 厂商触觉灵巧手用户手册PDF |
| `build/` | 编译生成产物，非本次Python运行入口 |
| 根目录网络脚本 | 网卡配置/恢复，未自动运行 |
| `.venv`、`__pycache__`、ZIP | 本地环境、缓存、历史归档，不作为源码权威 |

## 控制所有权

- 一个 mHandPro USB 接收器：一个采集进程管理双手。
- 同一个 G1 UDP 9527：只使用一个 zh 上位遥操发送端。
- 每只 INSPIRE：一个 G1 `HandFollower` 独占 Modbus，触觉与角度在同线程处理。
- Pi十舵机：一个 `DualHandController` 与共享串口/RLock；GUI直接复用此控制器。
- 每块BLE板：一个 broker；GUI管理它自己的双手broker，独立CLI需手动启动broker。

## 当前维护边界

不整合新代码包。PC、mHandPro 与 G1 驱动保留在 zh；newteleop/exoskeleton 的代码和配置保持原状态。本次保留文档整理，上位机说明移至 [supervisor/LOCAL_SUPERVISOR_CN.md](exoskeleton/supervisor/LOCAL_SUPERVISOR_CN.md)。

## 现场验收边界

离线测试不能确认舵机电流方向、绳索行程、真实FSR标定、蓝牙连接稳定性、G1内网或触觉刷新率。按主README分阶段完成只读、未穿戴低电流、逐指、位置遥操、双边闭环，再进行双臂/腰部联合测试。
