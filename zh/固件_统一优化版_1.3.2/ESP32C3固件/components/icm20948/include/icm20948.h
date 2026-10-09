/**
 * @file icm20948.h
 * @brief ICM-20948 9轴 IMU + AK09916 磁力计 I2C 驱动 (ESP-IDF 5.x)
 *
 * 使用 I2C Bypass 模式直接访问内置 AK09916 磁力计
 * 加速度计/陀螺仪以 225Hz 输出 (SMPLRT_DIV=4)，磁力计 100Hz 连续测量
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// I2C Pin Configuration
// ============================================================================
#define ICM20948_I2C_SCL_PIN     7
#define ICM20948_I2C_SDA_PIN     6
#define ICM20948_INT_PIN         1
#define ICM20948_I2C_FREQ_HZ    400000  // 400kHz Fast Mode

// 陀螺仪 DLPF_CFG：3 对应约51.2 Hz的3 dB带宽。
// A/B测试旧配置时可在编译参数中定义为0，无需改驱动逻辑。
#ifndef ICM20948_GYRO_DLPF_CFG
#define ICM20948_GYRO_DLPF_CFG  3
#endif

#if ICM20948_GYRO_DLPF_CFG < 0 || ICM20948_GYRO_DLPF_CFG > 7
#error "ICM20948_GYRO_DLPF_CFG 必须在0到7之间"
#endif

// ============================================================================
// I2C Addresses
// ============================================================================
#define ICM20948_I2C_ADDR       0x68    // AD0=LOW (0x69 if AD0=HIGH)
#define AK09916_I2C_ADDR        0x0C    // 内置磁力计

// ============================================================================
// WHO_AM_I 标识值
// ============================================================================
#define ICM20948_WHO_AM_I_VAL   0xEA
#define AK09916_WIA2_VAL        0x09

// ============================================================================
// 九轴传感器数据 (已缩放为物理单位)
// ============================================================================
typedef struct {
    float accel_x, accel_y, accel_z;   // 单位: g
    float gyro_x, gyro_y, gyro_z;      // 单位: deg/s
    float mag_x, mag_y, mag_z;         // 单位: µT
    float temperature;                  // 单位: °C
    bool mag_valid;                     // true = 本次读取获得新磁力计数据
} icm20948_data_t;

/**
 * @brief 初始化 ICM-20948 + AK09916
 *
 * 配置: 陀螺仪 ±2000°/s、DLPF_CFG可配置, 加速度计 ±8g
 *       陀螺仪/加速度计 225Hz, 磁力计 100Hz 连续
 *
 * @return ESP_OK 成功, 其他表示失败
 */
esp_err_t icm20948_init(void);

/**
 * @brief 读取全部九轴数据 (加速度、陀螺仪、磁力计、温度)
 *
 * 加速度计和陀螺仪每次调用都读取 (14字节突发读取)
 * 磁力计通过 DRDY 检测, 有新数据时更新并设置 mag_valid=true
 *
 * @param[out] data 指向数据结构的指针
 * @return ESP_OK 成功
 */
esp_err_t icm20948_read_all(icm20948_data_t *data);

#ifdef __cplusplus
}
#endif
