/**
 * @file icm20948.c
 * @brief ICM-20948 + AK09916 I2C 驱动 (ESP-IDF 5.x 新版 I2C Master API)
 *
 * 使用 I2C Bypass 模式直接访问 AK09916 磁力计:
 *   ICM20948 的 INT_PIN_CFG.BYPASS_EN=1, ESP32 直接与 AK09916 通信
 *   两次独立 I2C 读取分别获取 IMU 数据和磁力计数据
 *
 * 自动检测 AK09916 vs AK09916C (CNTL2 寄存器地址不同):
 *   AK09916:  CNTL2=0x30, CNTL3=0x31
 *   AK09916C: CNTL2=0x31, CNTL3=0x32
 */

#include "icm20948.h"
#include <string.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ICM20948";

// ============================================================================
// ICM-20948 Register Bank 0
// ============================================================================
#define REG_WHO_AM_I             0x00
#define REG_USER_CTRL            0x03
#define REG_LP_CONFIG            0x05
#define REG_PWR_MGMT_1           0x06
#define REG_PWR_MGMT_2           0x07
#define REG_INT_PIN_CFG          0x0F
#define REG_INT_ENABLE_1         0x11
#define REG_ACCEL_XOUT_H         0x2D
#define REG_BANK_SEL             0x7F

// ============================================================================
// ICM-20948 Register Bank 2
// ============================================================================
#define REG_GYRO_SMPLRT_DIV      0x00
#define REG_GYRO_CONFIG_1        0x01
#define REG_ODR_ALIGN_EN         0x09
#define REG_ACCEL_SMPLRT_DIV_1   0x10
#define REG_ACCEL_SMPLRT_DIV_2   0x11
#define REG_ACCEL_CONFIG         0x14

// ============================================================================
// AK09916 磁力计寄存器 (公共)
// ============================================================================
#define AK_WIA1              0x00   // Company ID, 应返回 0x48
#define AK_WIA2              0x01   // Device ID, 应返回 0x09
#define AK_ST1               0x10   // 状态寄存器 (DRDY bit0)
#define AK_HXL               0x11   // 磁力计数据起始
#define AK_ST2               0x18   // 状态寄存器2

// CNTL2/CNTL3 地址取决于变体 (运行时检测)
static uint8_t s_ak_cntl2 = 0x31;  // 默认 AK09916C
static uint8_t s_ak_cntl3 = 0x32;

// ============================================================================
// 静态变量
// ============================================================================
static i2c_master_bus_handle_t  s_i2c_bus  = NULL;
static i2c_master_dev_handle_t  s_icm_dev  = NULL;
static i2c_master_dev_handle_t  s_ak_dev   = NULL;   // AK09916 bypass 模式句柄

static float   s_accel_scale   = 0.0f;
static float   s_gyro_scale    = 0.0f;
static float   s_last_mag[3]   = {0};
static uint8_t s_current_bank  = 0xFF;
static bool    s_mag_available = false;

#define I2C_TIMEOUT_MS  100

// ============================================================================
// Bank 切换
// ============================================================================
static esp_err_t icm_select_bank(uint8_t bank)
{
    if (bank == s_current_bank) return ESP_OK;
    uint8_t buf[2] = { REG_BANK_SEL, (uint8_t)(bank << 4) };
    esp_err_t ret = i2c_master_transmit(s_icm_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
    if (ret == ESP_OK) s_current_bank = bank;
    return ret;
}

// ============================================================================
// ICM20948 底层读写 (带自动 Bank 切换)
// ============================================================================
static esp_err_t icm_write_reg(uint8_t bank, uint8_t reg, uint8_t val)
{
    esp_err_t ret = icm_select_bank(bank);
    if (ret != ESP_OK) return ret;
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_icm_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t icm_read_regs(uint8_t bank, uint8_t reg, uint8_t *out, size_t len)
{
    esp_err_t ret = icm_select_bank(bank);
    if (ret != ESP_OK) return ret;
    return i2c_master_transmit_receive(s_icm_dev, &reg, 1, out, len, I2C_TIMEOUT_MS);
}

// ============================================================================
// AK09916 底层读写 (bypass 模式, 直接 I2C)
// ============================================================================
static esp_err_t ak_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_ak_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t ak_read_regs(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(s_ak_dev, &reg, 1, out, len, I2C_TIMEOUT_MS);
}

static void set_scale_factors(void)
{
    s_accel_scale = 1.0f / 4096.0f;   // ±8g: 4096 LSB/g
    s_gyro_scale  = 1.0f / 16.4f;     // ±2000 dps: 16.4 LSB/(°/s)
}

// ============================================================================
// I2C 总线初始化
// ============================================================================
static esp_err_t i2c_bus_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = I2C_NUM_0,
        .scl_io_num = ICM20948_I2C_SCL_PIN,
        .sda_io_num = ICM20948_I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = ICM20948_I2C_ADDR,
        .scl_speed_hz    = ICM20948_I2C_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_icm_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Add ICM20948 device failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

// ============================================================================
// AK09916 初始化 (bypass 模式, 直接 I2C)
// ============================================================================
static esp_err_t ak09916_init(void)
{
    esp_err_t ret;

    // 添加 AK09916 为独立 I2C 设备
    i2c_device_config_t ak_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = AK09916_I2C_ADDR,
        .scl_speed_hz    = ICM20948_I2C_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(s_i2c_bus, &ak_cfg, &s_ak_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Add AK09916 device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 验证 WIA1 + WIA2
    uint8_t wia1 = 0, wia2 = 0;
    ret = ak_read_regs(AK_WIA1, &wia1, 1);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "AK09916 WIA1 read failed"); return ret; }
    ret = ak_read_regs(AK_WIA2, &wia2, 1);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "AK09916 WIA2 read failed"); return ret; }
    if (wia2 != AK09916_WIA2_VAL) {
        ESP_LOGE(TAG, "AK09916 not found (WIA2=0x%02X)", wia2);
        return ESP_ERR_NOT_FOUND;
    }

    // 自动检测 AK09916 vs AK09916C (CNTL2/CNTL3 寄存器地址不同)
    // 先尝试 AK09916C (CNTL3=0x32, CNTL2=0x31)
    ak_write_reg(0x32, 0x01);  // CNTL3 reset (AK09916C addr)
    vTaskDelay(pdMS_TO_TICKS(100));
    ak_write_reg(0x31, 0x01);  // CNTL2 = single measurement (AK09916C addr)
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t st1 = 0;
    ak_read_regs(AK_ST1, &st1, 1);

    if (st1 & 0x01) {
        s_ak_cntl2 = 0x31;
        s_ak_cntl3 = 0x32;
        uint8_t dummy[8];
        ak_read_regs(AK_HXL, dummy, 8);
    } else {
        // 尝试标准 AK09916 (CNTL3=0x31, CNTL2=0x30)
        ak_write_reg(0x31, 0x01);
        vTaskDelay(pdMS_TO_TICKS(100));
        ak_write_reg(0x30, 0x01);
        vTaskDelay(pdMS_TO_TICKS(20));

        ak_read_regs(AK_ST1, &st1, 1);
        if (st1 & 0x01) {
            s_ak_cntl2 = 0x30;
            s_ak_cntl3 = 0x31;
            uint8_t dummy[8];
            ak_read_regs(AK_HXL, dummy, 8);
        }
    }
    ESP_LOGI(TAG, "AK09916 variant: CNTL2=0x%02X, CNTL3=0x%02X", s_ak_cntl2, s_ak_cntl3);

    // 软复位 (使用检测到的地址)
    ret = ak_write_reg(s_ak_cntl3, 0x01);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));

    // 设置连续测量模式 4 (100Hz): CNTL2 = 0x08
    ret = ak_write_reg(s_ak_cntl2, 0x08);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(50));

    // 等待首次测量完成并清除标志
    vTaskDelay(pdMS_TO_TICKS(20));
    st1 = 0;
    ak_read_regs(AK_ST1, &st1, 1);
    if (st1 & 0x01) {
        uint8_t dummy[8];
        ak_read_regs(AK_HXL, dummy, 8);
    }

    ESP_LOGI(TAG, "AK09916 initialized (100Hz continuous)");
    return ESP_OK;
}

// ============================================================================
// ICM-20948 整体初始化
// ============================================================================
esp_err_t icm20948_init(void)
{
    esp_err_t ret;

    // 1. 初始化 I2C 总线
    ret = i2c_bus_init();
    if (ret != ESP_OK) return ret;

    // 探测 ICM20948 地址 (0x68 或 0x69)
    ret = i2c_master_probe(s_i2c_bus, ICM20948_I2C_ADDR, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ICM20948 未在 0x%02X 响应, 尝试备用地址 0x69", ICM20948_I2C_ADDR);
        i2c_master_bus_rm_device(s_icm_dev);
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address  = 0x69,
            .scl_speed_hz    = ICM20948_I2C_FREQ_HZ,
        };
        ret = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_icm_dev);
        if (ret != ESP_OK) return ret;
        ret = i2c_master_probe(s_i2c_bus, 0x69, I2C_TIMEOUT_MS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ICM20948 在 0x68/0x69 均未找到");
            return ESP_ERR_NOT_FOUND;
        }
    }

    // 2. 复位
    ret = icm_write_reg(0, REG_PWR_MGMT_1, 0x81);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "reset failed"); return ret; }
    s_current_bank = 0xFF;
    vTaskDelay(pdMS_TO_TICKS(100));

    // 3. 唤醒, 自动选最佳时钟
    ret = icm_write_reg(0, REG_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(50));

    // 4. 验证 WHO_AM_I
    uint8_t who = 0;
    ret = icm_read_regs(0, REG_WHO_AM_I, &who, 1);
    if (ret != ESP_OK) return ret;
    if (who != ICM20948_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: 0x%02X (expected 0x%02X)", who, ICM20948_WHO_AM_I_VAL);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "ICM-20948 detected (WHO_AM_I=0x%02X)", who);

    // 5. 使能所有传感器
    ret = icm_write_reg(0, REG_PWR_MGMT_2, 0x00);
    if (ret != ESP_OK) return ret;

    // ================================================================
    // 配置陀螺仪 (Bank 2)
    // ================================================================
    ret = icm_write_reg(2, REG_GYRO_SMPLRT_DIV, 4);    // 225Hz
    if (ret != ESP_OK) return ret;
    const uint8_t gyro_config = (uint8_t)(
        0x07 | (ICM20948_GYRO_DLPF_CFG << 3));
    ret = icm_write_reg(2, REG_GYRO_CONFIG_1, gyro_config);
    if (ret != ESP_OK) return ret;
    ESP_LOGI(TAG, "陀螺仪: 225Hz ±2000dps DLPF_CFG=%d",
             ICM20948_GYRO_DLPF_CFG);

    // ================================================================
    // 配置加速度计 (Bank 2)
    // ================================================================
    ret = icm_write_reg(2, REG_ACCEL_SMPLRT_DIV_1, 0x00);
    if (ret != ESP_OK) return ret;
    ret = icm_write_reg(2, REG_ACCEL_SMPLRT_DIV_2, 0x04);  // 225Hz
    if (ret != ESP_OK) return ret;
    ret = icm_write_reg(2, REG_ACCEL_CONFIG, 0x05);    // ±8g, DLPF
    if (ret != ESP_OK) return ret;
    ret = icm_write_reg(2, REG_ODR_ALIGN_EN, 0x01);
    if (ret != ESP_OK) return ret;

    set_scale_factors();

    // ================================================================
    // 使能 I2C Bypass 模式 (ESP32 直接访问 AK09916)
    // ================================================================
    ret = icm_write_reg(0, REG_USER_CTRL, 0x00);       // 禁用 I2C Master
    if (ret != ESP_OK) return ret;
    ret = icm_write_reg(0, REG_INT_PIN_CFG, 0x02);     // BYPASS_EN=1
    if (ret != ESP_OK) return ret;
    ret = icm_write_reg(0, REG_INT_ENABLE_1, 0x01);    // RAW_DATA_0_RDY_EN
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    // 初始化 AK09916 (通过 bypass 直接 I2C)
    ret = ak09916_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AK09916 init failed — 磁力计不可用, 继续运行");
        s_mag_available = false;
    } else {
        s_mag_available = true;
    }

    // 回到 Bank 0
    ret = icm_select_bank(0);

    ESP_LOGI(TAG, "ICM-20948 初始化完成 (Accel:±8g, Gyro:±2000dps, ODR:225Hz, Mag:%s)",
             s_mag_available ? "100Hz" : "不可用");
    return ESP_OK;
}

// ============================================================================
// 读取全部九轴数据
// ============================================================================
esp_err_t icm20948_read_all(icm20948_data_t *data)
{
    if (data == NULL) return ESP_ERR_INVALID_ARG;

    // 读取 ICM20948: 14字节突发读取 (Accel + Gyro + Temp)
    uint8_t buf[14];
    uint8_t reg = REG_ACCEL_XOUT_H;
    esp_err_t ret = i2c_master_transmit_receive(s_icm_dev, &reg, 1, buf, 14, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) return ret;

    // 解析加速度计 (大端序)
    int16_t ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4]  << 8) | buf[5]);
    data->accel_x = (float)ax * s_accel_scale;
    data->accel_y = (float)ay * s_accel_scale;
    data->accel_z = (float)az * s_accel_scale;

    // 解析陀螺仪 (大端序)
    int16_t gx = (int16_t)((buf[6]  << 8) | buf[7]);
    int16_t gy = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t gz = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyro_x = (float)gx * s_gyro_scale;
    data->gyro_y = (float)gy * s_gyro_scale;
    data->gyro_z = (float)gz * s_gyro_scale;

    // 温度
    int16_t tr = (int16_t)((buf[12] << 8) | buf[13]);
    data->temperature = (float)tr / 333.87f + 21.0f;

    // 读取 AK09916 磁力计 (bypass 直接 I2C)
    data->mag_valid = false;
    if (s_mag_available) {
        uint8_t st1 = 0;
        ret = ak_read_regs(AK_ST1, &st1, 1);
        if (ret == ESP_OK && (st1 & 0x01)) {
            // DRDY=1, 读取 8 字节: HXL, HXH, HYL, HYH, HZL, HZH, DUMMY, ST2
            uint8_t mag_buf[8];
            ret = ak_read_regs(AK_HXL, mag_buf, 8);
            if (ret == ESP_OK && (mag_buf[7] & 0x08) == 0) {
                int16_t mx = (int16_t)((mag_buf[1] << 8) | mag_buf[0]);
                int16_t my = (int16_t)((mag_buf[3] << 8) | mag_buf[2]);
                int16_t mz = (int16_t)((mag_buf[5] << 8) | mag_buf[4]);
                s_last_mag[0] = (float)mx * 0.15f;
                s_last_mag[1] = (float)my * 0.15f;
                s_last_mag[2] = (float)mz * 0.15f;
                data->mag_valid = true;
            } else if (ret == ESP_OK) {
                // ST2.HOFL=1：磁场过强，本帧不得进入拟合或 VQF。
                static uint32_t overflow_count = 0;
                if (++overflow_count % 100 == 1) {
                    ESP_LOGW(TAG, "AK09916 magnetic overflow, sample discarded");
                }
            }
        }
    }

    data->mag_x = s_last_mag[0];
    data->mag_y = s_last_mag[1];
    data->mag_z = s_last_mag[2];

    return ESP_OK;
}
