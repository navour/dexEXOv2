/**
 * @file lsm9ds1.h
 * @brief LSM9DS1 九轴IMU驱动 (ESP-IDF 5.x I2C Master API)
 *
 * 基于 SparkFun LSM9DS1 Arduino Library 移植
 * 支持加速度计、陀螺仪、磁力计三轴数据读取
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// I2C 引脚配置
// ============================================================================
#define LSM9DS1_I2C_SCL_PIN     35
#define LSM9DS1_I2C_SDA_PIN     36
#define LSM9DS1_I2C_FREQ_HZ    400000  // 400kHz Fast Mode

// ============================================================================
// I2C 地址 (通过扫描确认)
// ============================================================================
#define LSM9DS1_AG_ADDR         0x6B   // SDO_AG = HIGH
#define LSM9DS1_M_ADDR          0x1E   // SDO_M  = HIGH

// ============================================================================
// WHO_AM_I 期望值
// ============================================================================
#define LSM9DS1_WHO_AM_I_AG     0x68
#define LSM9DS1_WHO_AM_I_M      0x3D

// ============================================================================
// 量程配置
// ============================================================================
typedef enum {
    LSM9DS1_GYRO_245DPS  = 0,   // ±245 °/s
    LSM9DS1_GYRO_500DPS  = 1,   // ±500 °/s
    LSM9DS1_GYRO_2000DPS = 3,   // ±2000 °/s
} lsm9ds1_gyro_scale_t;

typedef enum {
    LSM9DS1_ACCEL_2G  = 0,      // ±2g
    LSM9DS1_ACCEL_4G  = 2,      // ±4g
    LSM9DS1_ACCEL_8G  = 3,      // ±8g
    LSM9DS1_ACCEL_16G = 1,      // ±16g
} lsm9ds1_accel_scale_t;

typedef enum {
    LSM9DS1_MAG_4GAUSS  = 0,    // ±4 gauss
    LSM9DS1_MAG_8GAUSS  = 1,    // ±8 gauss
    LSM9DS1_MAG_12GAUSS = 2,    // ±12 gauss
    LSM9DS1_MAG_16GAUSS = 3,    // ±16 gauss
} lsm9ds1_mag_scale_t;

// ============================================================================
// 传感器数据结构 (物理单位)
// ============================================================================
typedef struct {
    float accel_x, accel_y, accel_z;   // 单位: g
    float gyro_x, gyro_y, gyro_z;      // 单位: °/s (deg/s)
    float mag_x, mag_y, mag_z;         // 单位: gauss
    float temperature;                  // 单位: °C
} lsm9ds1_data_t;

// ============================================================================
// 配置结构体
// ============================================================================
typedef struct {
    lsm9ds1_gyro_scale_t  gyro_scale;
    lsm9ds1_accel_scale_t accel_scale;
    lsm9ds1_mag_scale_t   mag_scale;
    uint8_t gyro_odr;    // 陀螺仪输出数据率 (1-6, 对应 14.9-952 Hz)
    uint8_t accel_odr;   // 加速度计输出数据率 (1-6, 对应 10-952 Hz)
    uint8_t mag_odr;     // 磁力计输出数据率 (0-7, 对应 0.625-80 Hz)
} lsm9ds1_config_t;

/**
 * @brief 获取默认配置
 *        Gyro: ±245°/s, 238Hz
 *        Accel: ±2g, 238Hz
 *        Mag: ±4gauss, 80Hz
 */
lsm9ds1_config_t lsm9ds1_get_default_config(void);

/**
 * @brief 初始化 LSM9DS1
 * @param config 配置参数 (NULL 使用默认配置)
 * @return ESP_OK 成功
 */
esp_err_t lsm9ds1_init(const lsm9ds1_config_t *config);

/**
 * @brief 读取全部九轴数据
 * @param[out] data 传感器数据
 * @return ESP_OK 成功
 */
esp_err_t lsm9ds1_read_all(lsm9ds1_data_t *data);

/**
 * @brief 读取陀螺仪数据
 * @param[out] gx, gy, gz 陀螺仪数据 (°/s)
 */
esp_err_t lsm9ds1_read_gyro(float *gx, float *gy, float *gz);

/**
 * @brief 读取加速度计数据
 * @param[out] ax, ay, az 加速度计数据 (g)
 */
esp_err_t lsm9ds1_read_accel(float *ax, float *ay, float *az);

/**
 * @brief 读取磁力计数据
 * @param[out] mx, my, mz 磁力计数据 (gauss)
 */
esp_err_t lsm9ds1_read_mag(float *mx, float *my, float *mz);

/**
 * @brief 读取温度
 * @param[out] temp 温度 (°C)
 */
esp_err_t lsm9ds1_read_temp(float *temp);

/**
 * @brief 校准陀螺仪和加速度计 (静止状态下调用)
 * @param samples 采集样本数 (推荐 >= 100)
 */
esp_err_t lsm9ds1_calibrate(int samples);

#ifdef __cplusplus
}
#endif
