/**
 * @file imu_config.h
 * @brief IMU 传感器选择配置
 *
 * 修改 IMU_SENSOR_TYPE 即可切换传感器:
 *   IMU_SENSOR_LSM9DS1  — LSM9DS1 九轴 (陀螺仪+加速度计+磁力计), 使用 VQF 9D
 *   IMU_SENSOR_MPU9250  — MPU9250/MPU6500 六轴 (陀螺仪+加速度计), 使用 VQF 6D
 */
#pragma once

// ============================================================================
// 传感器类型定义
// ============================================================================
#define IMU_SENSOR_LSM9DS1   1
#define IMU_SENSOR_MPU9250   2
#define IMU_SENSOR_ICM20948  3

// ============================================================================
// >>> 修改这里切换传感器 <<<
// ============================================================================
#define IMU_SENSOR_TYPE      IMU_SENSOR_ICM20948

// ============================================================================
// LSM9DS1 调试选项: 设为 1 则只用陀螺仪+加速度计 (6D), 不使用磁力计
// 用于排查磁力计对姿态造成的影响
// ============================================================================
#define LSM9DS1_USE_6D_ONLY  0

// ============================================================================
// 便捷宏
// ============================================================================
#define USE_LSM9DS1   (IMU_SENSOR_TYPE == IMU_SENSOR_LSM9DS1)
#define USE_MPU9250   (IMU_SENSOR_TYPE == IMU_SENSOR_MPU9250)
#define USE_ICM20948  (IMU_SENSOR_TYPE == IMU_SENSOR_ICM20948)

// ============================================================================
// 输出模式选择
//   OUTPUT_MODE_RAW        — 终端打印易读裸数据 (陀螺仪/加速度计/磁力计)
//   OUTPUT_MODE_VISUALIZER — 输出四元数协议 ($Q,...) 供 visualizer.py 使用
//   OUTPUT_MODE_MAG_DIAG   — 磁力计校准诊断: 打印磁场模长|B|、各轴分量、姿态角
//   OUTPUT_MODE_MAG_CAL    — 磁力计椭圆校准模式: 采集数据→拟合椭球→存入NVS
// ============================================================================
#define OUTPUT_MODE_RAW         0
#define OUTPUT_MODE_VISUALIZER  1
#define OUTPUT_MODE_MAG_DIAG    2
#define OUTPUT_MODE_MAG_CAL     3

// >>> 修改这里切换输出模式 <<<
#define OUTPUT_MODE             OUTPUT_MODE_VISUALIZER

// ============================================================================
// 传输通道选择
//   TRANSPORT_MODE_SERIAL — USB 串口输出 (默认, 需有线连接)
//   TRANSPORT_MODE_WIFI   — WiFi TCP 输出 (无线, 需配置 WiFi)
// ============================================================================
#define TRANSPORT_MODE_SERIAL   0
#define TRANSPORT_MODE_WIFI     1

// >>> 修改这里切换传输通道 <<<
#define TRANSPORT_MODE          TRANSPORT_MODE_WIFI

#define USE_WIFI    (TRANSPORT_MODE == TRANSPORT_MODE_WIFI)
#define USE_SERIAL  (TRANSPORT_MODE == TRANSPORT_MODE_SERIAL)

// ============================================================================
// ICM20948 读取触发模式
//   ICM_READ_TRIGGER_TIMER - 使用周期定时器触发读取 (当前轮询式)
//   ICM_READ_TRIGGER_INT   - 使用 ICM INT 引脚数据就绪中断触发读取
// ============================================================================
#define ICM_READ_TRIGGER_TIMER  0
#define ICM_READ_TRIGGER_INT    1

// >>> ICM20948 触发模式切换 <<<
#define ICM_READ_TRIGGER_MODE   ICM_READ_TRIGGER_INT

// ============================================================================
// 磁力计开关 (仅对 LSM9DS1 有效, MPU9250 无磁力计)
//   1 — 使用磁力计 (VQF 9D)
//   0 — 不使用磁力计 (VQF 6D)
// ============================================================================
// >>> 修改这里开启/关闭磁力计 <<<
#define USE_MAGNETOMETER        1

// ============================================================================
// VQF 实际时间积分开关
//   1 — 使用实际测量的采样间隔更新 VQF (更精确)
//   0 — 使用固定 dt = 1/SAMPLE_RATE_HZ (默认)
// ============================================================================
#define VQF_USE_ACTUAL_DT       1
