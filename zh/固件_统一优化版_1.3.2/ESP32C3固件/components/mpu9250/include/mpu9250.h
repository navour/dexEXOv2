#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// I2C Pin Configuration - 根据硬件修改
// ============================================================================
#define MPU9250_I2C_SCL_PIN     35
#define MPU9250_I2C_SDA_PIN     36
#define MPU9250_I2C_FREQ_HZ    400000  // 400kHz Fast Mode

// ============================================================================
// I2C Addresses
// ============================================================================
#define MPU9250_I2C_ADDR        0x68   // AD0 = LOW (0x69 if AD0 = HIGH)
#define AK8963_I2C_ADDR         0x0C   // 磁力计地址

// ============================================================================
// WHO_AM_I 标识值
// ============================================================================
#define MPU6500_WHO_AM_I_VAL    0x70
#define MPU9250_WHO_AM_I_VAL    0x71
#define MPU9255_WHO_AM_I_VAL    0x73

// ============================================================================
// 陀螺仪量程
// ============================================================================
typedef enum {
    MPU9250_GYRO_FS_250DPS  = 0x00,
    MPU9250_GYRO_FS_500DPS  = 0x08,
    MPU9250_GYRO_FS_1000DPS = 0x10,
    MPU9250_GYRO_FS_2000DPS = 0x18,
} mpu9250_gyro_fs_t;

// ============================================================================
// 加速度计量程
// ============================================================================
typedef enum {
    MPU9250_ACCEL_FS_2G   = 0x00,
    MPU9250_ACCEL_FS_4G   = 0x08,
    MPU9250_ACCEL_FS_8G   = 0x10,
    MPU9250_ACCEL_FS_16G  = 0x18,
} mpu9250_accel_fs_t;

// ============================================================================
// 九轴传感器数据（已缩放为物理单位）
// ============================================================================
typedef struct {
    float accel_x, accel_y, accel_z;   // 单位: g
    float gyro_x, gyro_y, gyro_z;      // 单位: deg/s
    float mag_x, mag_y, mag_z;         // 单位: uT
    float temperature;                  // 单位: °C
} mpu9250_data_t;

/**
 * @brief 初始化 MPU9250/9255 + AK8963 磁力计
 * @return ESP_OK 成功, 其他表示失败
 */
esp_err_t mpu9250_init(void);

/**
 * @brief 读取全部九轴数据（加速度、陀螺仪、磁力计、温度）
 * @param[out] data 指向数据结构的指针
 * @return ESP_OK 成功
 */
esp_err_t mpu9250_read_all(mpu9250_data_t *data);

#ifdef __cplusplus
}
#endif
