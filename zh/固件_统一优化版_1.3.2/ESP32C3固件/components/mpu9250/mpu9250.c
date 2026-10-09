/**
 * @file mpu9250.c
 * @brief MPU9250/9255 + AK8963 I2C 驱动 (ESP-IDF 5.x 新版 I2C Master API)
 *
 * 使用 I2C Bypass 模式直接访问 AK8963 磁力计
 * 加速度计/陀螺仪以设定采样率输出，磁力计 100Hz 连续测量
 */

#include "mpu9250.h"
#include <string.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MPU9250";

// ============================================================================
// MPU9250 寄存器地址
// ============================================================================
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_CONFIG2   0x1D
#define REG_INT_PIN_CFG     0x37
#define REG_INT_ENABLE      0x38
#define REG_ACCEL_XOUT_H    0x3B   // 从此地址连续读 14 字节: accel(6)+temp(2)+gyro(6)
#define REG_USER_CTRL       0x6A
#define REG_PWR_MGMT_1      0x6B
#define REG_PWR_MGMT_2      0x6C
#define REG_WHO_AM_I        0x75

// ============================================================================
// AK8963 寄存器地址
// ============================================================================
#define AK_WIA              0x00   // WHO_AM_I, 应返回 0x48
#define AK_ST1              0x02   // 状态寄存器1 (DRDY bit0)
#define AK_HXL              0x03   // 磁力计数据起始 (小端序, LSB在前)
#define AK_ST2              0x09   // 状态寄存器2 (必须读取以完成测量周期)
#define AK_CNTL1            0x0A   // 控制寄存器1
#define AK_CNTL2            0x0B   // 控制寄存器2 (复位)
#define AK_ASAX             0x10   // 灵敏度校准值

// ============================================================================
// 静态变量
// ============================================================================
static i2c_master_bus_handle_t s_i2c_bus  = NULL;
static i2c_master_dev_handle_t s_mpu_dev  = NULL;
static i2c_master_dev_handle_t s_ak_dev   = NULL;

static float s_accel_scale   = 0.0f;   // 加速度计 LSB → g
static float s_gyro_scale    = 0.0f;   // 陀螺仪  LSB → deg/s
static float s_mag_adj[3]    = {0};    // 磁力计灵敏度校准因子
static float s_last_mag[3]   = {0};    // 保存最近一次有效的磁力计读数

// 默认配置
static const mpu9250_gyro_fs_t  s_gyro_fs  = MPU9250_GYRO_FS_2000DPS;
static const mpu9250_accel_fs_t s_accel_fs = MPU9250_ACCEL_FS_8G;

#define I2C_TIMEOUT_MS   100

// ============================================================================
// I2C 底层读写辅助函数
// ============================================================================
static esp_err_t mpu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_mpu_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t mpu_read_regs(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(s_mpu_dev, &reg, 1, out, len, I2C_TIMEOUT_MS);
}

static esp_err_t ak_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_ak_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t ak_read_regs(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(s_ak_dev, &reg, 1, out, len, I2C_TIMEOUT_MS);
}

// ============================================================================
// 设置缩放因子
// ============================================================================
static void set_scale_factors(void)
{
    switch (s_accel_fs) {
        case MPU9250_ACCEL_FS_2G:  s_accel_scale = 1.0f / 16384.0f; break;
        case MPU9250_ACCEL_FS_4G:  s_accel_scale = 1.0f / 8192.0f;  break;
        case MPU9250_ACCEL_FS_8G:  s_accel_scale = 1.0f / 4096.0f;  break;
        case MPU9250_ACCEL_FS_16G: s_accel_scale = 1.0f / 2048.0f;  break;
    }
    switch (s_gyro_fs) {
        case MPU9250_GYRO_FS_250DPS:  s_gyro_scale = 1.0f / 131.0f; break;
        case MPU9250_GYRO_FS_500DPS:  s_gyro_scale = 1.0f / 65.5f;  break;
        case MPU9250_GYRO_FS_1000DPS: s_gyro_scale = 1.0f / 32.8f;  break;
        case MPU9250_GYRO_FS_2000DPS: s_gyro_scale = 1.0f / 16.4f;  break;
    }
}

// ============================================================================
// I2C 总线初始化 + 添加 MPU9250 设备
// ============================================================================
static esp_err_t i2c_bus_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = I2C_NUM_0,
        .scl_io_num = MPU9250_I2C_SCL_PIN,
        .sda_io_num = MPU9250_I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 添加 MPU9250 设备
    i2c_device_config_t mpu_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU9250_I2C_ADDR,
        .scl_speed_hz    = MPU9250_I2C_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(s_i2c_bus, &mpu_cfg, &s_mpu_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Add MPU9250 device failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

// ============================================================================
// AK8963 磁力计初始化
// ============================================================================
static esp_err_t ak8963_init(void)
{
    // 添加 AK8963 设备到 I2C 总线 (bypass 模式已开启)
    i2c_device_config_t ak_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = AK8963_I2C_ADDR,
        .scl_speed_hz    = MPU9250_I2C_FREQ_HZ,
    };
    esp_err_t ret = i2c_master_bus_add_device(s_i2c_bus, &ak_cfg, &s_ak_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Add AK8963 device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 验证 AK8963 WHO_AM_I
    uint8_t wia = 0;
    ret = ak_read_regs(AK_WIA, &wia, 1);
    if (ret != ESP_OK || wia != 0x48) {
        ESP_LOGE(TAG, "AK8963 not found (WIA=0x%02X, expected 0x48)", wia);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "AK8963 detected (WIA=0x%02X)", wia);

    // 复位
    ret = ak_write_reg(AK_CNTL2, 0x01);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));

    // 进入 Fuse ROM 访问模式，读取灵敏度校准值
    ret = ak_write_reg(AK_CNTL1, 0x0F);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t asa[3];
    ret = ak_read_regs(AK_ASAX, asa, 3);
    if (ret != ESP_OK) return ret;

    // 计算灵敏度校准因子: Hadj = H * ((ASA-128)/256 + 1)
    for (int i = 0; i < 3; i++) {
        s_mag_adj[i] = (float)(asa[i] - 128) / 256.0f + 1.0f;
    }
    ESP_LOGI(TAG, "AK8963 ASA: [%d, %d, %d]  adj: [%.3f, %.3f, %.3f]",
             asa[0], asa[1], asa[2], s_mag_adj[0], s_mag_adj[1], s_mag_adj[2]);

    // 关闭
    ret = ak_write_reg(AK_CNTL1, 0x00);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));

    // 设置为 16-bit 输出, 连续测量模式 2 (100Hz)
    // CNTL1: bit4=1 (16-bit), bits[3:0]=0x06 (continuous mode 2)  → 0x16
    ret = ak_write_reg(AK_CNTL1, 0x16);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    return ESP_OK;
}

// ============================================================================
// MPU9250 整体初始化
// ============================================================================
esp_err_t mpu9250_init(void)
{
    esp_err_t ret;

    // 1. 初始化 I2C 总线
    ret = i2c_bus_init();
    if (ret != ESP_OK) return ret;

    // 2. 复位 MPU9250
    ret = mpu_write_reg(REG_PWR_MGMT_1, 0x80);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU9250 reset failed");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // 3. 唤醒, 选择 PLL 时钟源
    ret = mpu_write_reg(REG_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(50));

    // 4. 验证 WHO_AM_I
    uint8_t who = 0;
    ret = mpu_read_regs(REG_WHO_AM_I, &who, 1);
    if (ret != ESP_OK) return ret;
    if (who != MPU6500_WHO_AM_I_VAL && who != MPU9250_WHO_AM_I_VAL && who != MPU9255_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "MPU WHO_AM_I mismatch: 0x%02X (expected 0x70/0x71/0x73)", who);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "MPU detected (WHO_AM_I=0x%02X)", who);

    // 5. 使能所有传感器
    ret = mpu_write_reg(REG_PWR_MGMT_2, 0x00);
    if (ret != ESP_OK) return ret;

    // 6. 采样率分频器: SampleRate = 1kHz / (1 + DIV)
    //    DIV=4 → 200Hz
    ret = mpu_write_reg(REG_SMPLRT_DIV, 4);
    if (ret != ESP_OK) return ret;

    // 7. 数字低通滤波器 DLPF_CFG=1: 带宽 184Hz (陀螺仪)
    ret = mpu_write_reg(REG_CONFIG, 0x01);
    if (ret != ESP_OK) return ret;

    // 8. 陀螺仪量程
    ret = mpu_write_reg(REG_GYRO_CONFIG, s_gyro_fs);
    if (ret != ESP_OK) return ret;

    // 9. 加速度计量程
    ret = mpu_write_reg(REG_ACCEL_CONFIG, s_accel_fs);
    if (ret != ESP_OK) return ret;

    // 10. 加速度计 DLPF: A_DLPF_CFG=1, 带宽 184Hz
    ret = mpu_write_reg(REG_ACCEL_CONFIG2, 0x01);
    if (ret != ESP_OK) return ret;

    // 11. 计算缩放因子
    set_scale_factors();

    // 12. 关闭 I2C Master, 确保 bypass 可用
    ret = mpu_write_reg(REG_USER_CTRL, 0x00);
    if (ret != ESP_OK) return ret;

    // 13. 开启 I2C Bypass 模式 (BYPASS_EN = bit1)
    //     这样 AK8963 可以直接从 I2C 总线访问
    ret = mpu_write_reg(REG_INT_PIN_CFG, 0x02);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(50));

    // 14. 初始化 AK8963 磁力计
    ret = ak8963_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AK8963 init failed — 磁力计数据不可用，继续运行");
        s_ak_dev = NULL; // 标记为不可用
    }

    ESP_LOGI(TAG, "MPU9250/9255 初始化完成 (Accel: ±8g, Gyro: ±2000dps, Mag: 100Hz)");
    return ESP_OK;
}

// ============================================================================
// 读取全部九轴数据
// ============================================================================
esp_err_t mpu9250_read_all(mpu9250_data_t *data)
{
    if (data == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t buf[14]; // accel(6) + temp(2) + gyro(6) = 14 bytes

    // 一次突发读取加速度计 + 温度 + 陀螺仪
    esp_err_t ret = mpu_read_regs(REG_ACCEL_XOUT_H, buf, 14);
    if (ret != ESP_OK) return ret;

    // 解析原始值 (MPU9250 数据为大端序: MSB 在前)
    int16_t ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t tr = (int16_t)((buf[6]  << 8) | buf[7]);
    int16_t gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

    // 转换为物理单位
    data->accel_x     = (float)ax * s_accel_scale;
    data->accel_y     = (float)ay * s_accel_scale;
    data->accel_z     = (float)az * s_accel_scale;
    data->gyro_x      = (float)gx * s_gyro_scale;
    data->gyro_y      = (float)gy * s_gyro_scale;
    data->gyro_z      = (float)gz * s_gyro_scale;
    data->temperature  = (float)tr / 333.87f + 21.0f;

    // 读取磁力计 (如果可用)
    if (s_ak_dev != NULL) {
        uint8_t st1 = 0;
        ret = ak_read_regs(AK_ST1, &st1, 1);
        if (ret == ESP_OK && (st1 & 0x01)) {
            // DRDY=1, 有新数据
            uint8_t mag_buf[7]; // HXL, HXH, HYL, HYH, HZL, HZH, ST2
            ret = ak_read_regs(AK_HXL, mag_buf, 7);
            if (ret == ESP_OK) {
                // 检查磁力溢出标志 (ST2 bit3 = HOFL)
                if (!(mag_buf[6] & 0x08)) {
                    // AK8963 数据为小端序: LSB 在前
                    int16_t mx = (int16_t)((mag_buf[1] << 8) | mag_buf[0]);
                    int16_t my = (int16_t)((mag_buf[3] << 8) | mag_buf[2]);
                    int16_t mz = (int16_t)((mag_buf[5] << 8) | mag_buf[4]);

                    // 缩放: raw * adj * 0.15 → µT (16-bit模式: 4912µT / 32760 ≈ 0.15)
                    s_last_mag[0] = (float)mx * s_mag_adj[0] * 0.15f;
                    s_last_mag[1] = (float)my * s_mag_adj[1] * 0.15f;
                    s_last_mag[2] = (float)mz * s_mag_adj[2] * 0.15f;
                }
            }
        }
    }

    // 总是返回最近一次有效的磁力计读数
    data->mag_x = s_last_mag[0];
    data->mag_y = s_last_mag[1];
    data->mag_z = s_last_mag[2];

    return ESP_OK;
}
