/**
 * @file lsm9ds1.c
 * @brief LSM9DS1 九轴IMU驱动实现 (ESP-IDF 5.x I2C Master API)
 *
 * 基于 SparkFun LSM9DS1 Arduino Library 移植到 ESP-IDF
 * 参考: https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library
 */

#include "lsm9ds1.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LSM9DS1";

// ============================================================================
// LSM9DS1 寄存器定义 (Accel/Gyro)
// ============================================================================
#define REG_ACT_THS              0x04
#define REG_ACT_DUR              0x05
#define REG_WHO_AM_I_XG          0x0F
#define REG_CTRL_REG1_G          0x10   // Gyro ODR + Scale + BW
#define REG_CTRL_REG2_G          0x11
#define REG_CTRL_REG3_G          0x12
#define REG_OUT_TEMP_L           0x15
#define REG_OUT_TEMP_H           0x16
#define REG_STATUS_REG_0         0x17
#define REG_OUT_X_L_G            0x18   // Gyro data start
#define REG_CTRL_REG4            0x1E
#define REG_CTRL_REG5_XL         0x1F
#define REG_CTRL_REG6_XL         0x20   // Accel ODR + Scale + BW
#define REG_CTRL_REG7_XL         0x21
#define REG_CTRL_REG8            0x22
#define REG_CTRL_REG9            0x23
#define REG_CTRL_REG10           0x24
#define REG_STATUS_REG_1         0x27
#define REG_OUT_X_L_XL           0x28   // Accel data start
#define REG_FIFO_CTRL            0x2E
#define REG_FIFO_SRC             0x2F

// ============================================================================
// LSM9DS1 寄存器定义 (Magnetometer)
// ============================================================================
#define REG_OFFSET_X_REG_L_M     0x05
#define REG_WHO_AM_I_M           0x0F
#define REG_CTRL_REG1_M          0x20
#define REG_CTRL_REG2_M          0x21
#define REG_CTRL_REG3_M          0x22
#define REG_CTRL_REG4_M          0x23
#define REG_CTRL_REG5_M          0x24
#define REG_STATUS_REG_M         0x27
#define REG_OUT_X_L_M            0x28   // Mag data start

// ============================================================================
// 灵敏度常量 (来自 LSM9DS1 数据手册)
// ============================================================================
#define SENSITIVITY_GYRO_245     0.00875f    // dps/LSB
#define SENSITIVITY_GYRO_500     0.0175f
#define SENSITIVITY_GYRO_2000    0.07f

#define SENSITIVITY_ACCEL_2      0.000061f   // g/LSB
#define SENSITIVITY_ACCEL_4      0.000122f
#define SENSITIVITY_ACCEL_8      0.000244f
#define SENSITIVITY_ACCEL_16     0.000732f

#define SENSITIVITY_MAG_4        0.00014f    // gauss/LSB
#define SENSITIVITY_MAG_8        0.00029f
#define SENSITIVITY_MAG_12       0.00043f
#define SENSITIVITY_MAG_16       0.00058f

// ============================================================================
// 静态变量
// ============================================================================
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_ag_dev  = NULL;  // 加速度计/陀螺仪设备
static i2c_master_dev_handle_t s_m_dev   = NULL;  // 磁力计设备

static float s_gyro_res  = 0.0f;   // 陀螺仪分辨率 (dps/LSB)
static float s_accel_res = 0.0f;   // 加速度计分辨率 (g/LSB)
static float s_mag_res   = 0.0f;   // 磁力计分辨率 (gauss/LSB)

// 校准偏置
static float s_gyro_bias[3]  = {0, 0, 0};
static float s_accel_bias[3] = {0, 0, 0};

#define I2C_TIMEOUT_MS   100

// ============================================================================
// I2C 底层读写
// ============================================================================
static esp_err_t ag_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_ag_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t ag_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_ag_dev, &reg, 1, val, 1, I2C_TIMEOUT_MS);
}

static esp_err_t ag_read_regs(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(s_ag_dev, &reg, 1, out, len, I2C_TIMEOUT_MS);
}

static esp_err_t m_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_m_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t m_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_m_dev, &reg, 1, val, 1, I2C_TIMEOUT_MS);
}

static esp_err_t m_read_regs(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(s_m_dev, &reg, 1, out, len, I2C_TIMEOUT_MS);
}

// ============================================================================
// 设置灵敏度
// ============================================================================
static void calc_gyro_res(lsm9ds1_gyro_scale_t scale)
{
    switch (scale) {
        case LSM9DS1_GYRO_245DPS:  s_gyro_res = SENSITIVITY_GYRO_245;  break;
        case LSM9DS1_GYRO_500DPS:  s_gyro_res = SENSITIVITY_GYRO_500;  break;
        case LSM9DS1_GYRO_2000DPS: s_gyro_res = SENSITIVITY_GYRO_2000; break;
        default:                   s_gyro_res = SENSITIVITY_GYRO_245;  break;
    }
}

static void calc_accel_res(lsm9ds1_accel_scale_t scale)
{
    switch (scale) {
        case LSM9DS1_ACCEL_2G:  s_accel_res = SENSITIVITY_ACCEL_2;  break;
        case LSM9DS1_ACCEL_4G:  s_accel_res = SENSITIVITY_ACCEL_4;  break;
        case LSM9DS1_ACCEL_8G:  s_accel_res = SENSITIVITY_ACCEL_8;  break;
        case LSM9DS1_ACCEL_16G: s_accel_res = SENSITIVITY_ACCEL_16; break;
        default:                s_accel_res = SENSITIVITY_ACCEL_2;   break;
    }
}

static void calc_mag_res(lsm9ds1_mag_scale_t scale)
{
    switch (scale) {
        case LSM9DS1_MAG_4GAUSS:  s_mag_res = SENSITIVITY_MAG_4;  break;
        case LSM9DS1_MAG_8GAUSS:  s_mag_res = SENSITIVITY_MAG_8;  break;
        case LSM9DS1_MAG_12GAUSS: s_mag_res = SENSITIVITY_MAG_12; break;
        case LSM9DS1_MAG_16GAUSS: s_mag_res = SENSITIVITY_MAG_16; break;
        default:                  s_mag_res = SENSITIVITY_MAG_4;   break;
    }
}

// ============================================================================
// I2C 总线初始化
// ============================================================================
static esp_err_t i2c_bus_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = I2C_NUM_0,
        .scl_io_num = LSM9DS1_I2C_SCL_PIN,
        .sda_io_num = LSM9DS1_I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 总线初始化失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 添加 AG 设备 (加速度计 + 陀螺仪)
    i2c_device_config_t ag_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = LSM9DS1_AG_ADDR,
        .scl_speed_hz    = LSM9DS1_I2C_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(s_i2c_bus, &ag_cfg, &s_ag_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "添加 AG 设备失败");
        return ret;
    }

    // 添加 M 设备 (磁力计)
    i2c_device_config_t m_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = LSM9DS1_M_ADDR,
        .scl_speed_hz    = LSM9DS1_I2C_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(s_i2c_bus, &m_cfg, &s_m_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "添加 M 设备失败");
        return ret;
    }

    return ESP_OK;
}

// ============================================================================
// 初始化陀螺仪
// ============================================================================
static esp_err_t init_gyro(const lsm9ds1_config_t *cfg)
{
    // CTRL_REG1_G: [ODR_G(3)] [FS_G(2)] [0] [BW_G(2)]
    // ODR: 1=14.9Hz, 2=59.5Hz, 3=119Hz, 4=238Hz, 5=476Hz, 6=952Hz
    // BW_G=11 → 最大带宽 (ODR=238Hz时为78Hz, 减少相位延迟)
    uint8_t reg1_g = (cfg->gyro_odr << 5) | (cfg->gyro_scale << 3) | 0x03;
    esp_err_t ret = ag_write_reg(REG_CTRL_REG1_G, reg1_g);
    if (ret != ESP_OK) return ret;

    // CTRL_REG2_G: 默认 0x00 (无中断输出)
    ret = ag_write_reg(REG_CTRL_REG2_G, 0x00);
    if (ret != ESP_OK) return ret;

    // CTRL_REG3_G: LP_mode=0, HP_EN=0
    ret = ag_write_reg(REG_CTRL_REG3_G, 0x00);
    if (ret != ESP_OK) return ret;

    // CTRL_REG4: 使能 XYZ 陀螺仪轴
    ret = ag_write_reg(REG_CTRL_REG4, 0x38);
    if (ret != ESP_OK) return ret;

    calc_gyro_res(cfg->gyro_scale);
    ESP_LOGI(TAG, "陀螺仪: ODR=%d, Scale=%d, Res=%.5f °/s/LSB",
             cfg->gyro_odr, cfg->gyro_scale, s_gyro_res);

    return ESP_OK;
}

// ============================================================================
// 初始化加速度计
// ============================================================================
static esp_err_t init_accel(const lsm9ds1_config_t *cfg)
{
    // CTRL_REG5_XL: 使能 XYZ 加速度计轴
    esp_err_t ret = ag_write_reg(REG_CTRL_REG5_XL, 0x38);
    if (ret != ESP_OK) return ret;

    // CTRL_REG6_XL: [ODR_XL(3)] [FS_XL(2)] [BW_SCAL_ODR] [BW_XL(2)]
    uint8_t reg6_xl = (cfg->accel_odr << 5) | (cfg->accel_scale << 3);
    ret = ag_write_reg(REG_CTRL_REG6_XL, reg6_xl);
    if (ret != ESP_OK) return ret;

    // CTRL_REG7_XL: 默认 (高通滤波器禁用)
    ret = ag_write_reg(REG_CTRL_REG7_XL, 0x00);
    if (ret != ESP_OK) return ret;

    calc_accel_res(cfg->accel_scale);
    ESP_LOGI(TAG, "加速度计: ODR=%d, Scale=%d, Res=%.6f g/LSB",
             cfg->accel_odr, cfg->accel_scale, s_accel_res);

    return ESP_OK;
}

// ============================================================================
// 初始化磁力计
// ============================================================================
static esp_err_t init_mag(const lsm9ds1_config_t *cfg)
{
    // CTRL_REG1_M: [TEMP_COMP] [OM(2)] [DO(3)] [FAST_ODR] [ST]
    // OM=11 (ultra-high performance), DO=cfg->mag_odr
    // TEMP_COMP=1 (温度补偿)
    uint8_t reg1_m = 0x80 | (3 << 5) | (cfg->mag_odr << 2);
    esp_err_t ret = m_write_reg(REG_CTRL_REG1_M, reg1_m);
    if (ret != ESP_OK) return ret;

    // CTRL_REG2_M: [0] [FS(2)] [0] [REBOOT] [SOFT_RST] [0] [0]
    uint8_t reg2_m = (cfg->mag_scale << 5);
    ret = m_write_reg(REG_CTRL_REG2_M, reg2_m);
    if (ret != ESP_OK) return ret;

    // CTRL_REG3_M: 连续转换模式
    ret = m_write_reg(REG_CTRL_REG3_M, 0x00);
    if (ret != ESP_OK) return ret;

    // CTRL_REG4_M: Z 轴操作模式 = ultra-high performance
    ret = m_write_reg(REG_CTRL_REG4_M, 0x0C);
    if (ret != ESP_OK) return ret;

    // CTRL_REG5_M: BDU=1 (块数据更新)
    ret = m_write_reg(REG_CTRL_REG5_M, 0x40);
    if (ret != ESP_OK) return ret;

    calc_mag_res(cfg->mag_scale);
    ESP_LOGI(TAG, "磁力计: ODR=%d, Scale=%d, Res=%.5f gauss/LSB",
             cfg->mag_odr, cfg->mag_scale, s_mag_res);

    return ESP_OK;
}

// ============================================================================
// 公开 API 实现
// ============================================================================

lsm9ds1_config_t lsm9ds1_get_default_config(void)
{
    lsm9ds1_config_t cfg = {
        .gyro_scale  = LSM9DS1_GYRO_245DPS,
        .accel_scale = LSM9DS1_ACCEL_2G,
        .mag_scale   = LSM9DS1_MAG_4GAUSS,
        .gyro_odr    = 4,   // 238 Hz
        .accel_odr   = 4,   // 238 Hz
        .mag_odr     = 7,   // 80 Hz
    };
    return cfg;
}

esp_err_t lsm9ds1_init(const lsm9ds1_config_t *config)
{
    lsm9ds1_config_t cfg;
    if (config != NULL) {
        cfg = *config;
    } else {
        cfg = lsm9ds1_get_default_config();
    }

    // 1. 初始化 I2C 总线
    esp_err_t ret = i2c_bus_init();
    if (ret != ESP_OK) return ret;

    // 2. 验证 AG WHO_AM_I
    uint8_t who_ag = 0;
    ret = ag_read_reg(REG_WHO_AM_I_XG, &who_ag);
    if (ret != ESP_OK || who_ag != LSM9DS1_WHO_AM_I_AG) {
        ESP_LOGE(TAG, "AG WHO_AM_I 不匹配: 0x%02X (期望 0x%02X)",
                 who_ag, LSM9DS1_WHO_AM_I_AG);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "AG 检测成功 (WHO_AM_I=0x%02X)", who_ag);

    // 3. 验证 M WHO_AM_I
    uint8_t who_m = 0;
    ret = m_read_reg(REG_WHO_AM_I_M, &who_m);
    if (ret != ESP_OK || who_m != LSM9DS1_WHO_AM_I_M) {
        ESP_LOGE(TAG, "M WHO_AM_I 不匹配: 0x%02X (期望 0x%02X)",
                 who_m, LSM9DS1_WHO_AM_I_M);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "M 检测成功 (WHO_AM_I=0x%02X)", who_m);

    // 4. 软件复位 AG
    ret = ag_write_reg(REG_CTRL_REG8, 0x05); // SW_RESET + BDU
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(50));

    // 5. 软件复位 M
    ret = m_write_reg(REG_CTRL_REG2_M, 0x0C); // SOFT_RST + REBOOT
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(50));

    // 6. 初始化各传感器
    ret = init_gyro(&cfg);
    if (ret != ESP_OK) return ret;

    ret = init_accel(&cfg);
    if (ret != ESP_OK) return ret;

    ret = init_mag(&cfg);
    if (ret != ESP_OK) return ret;

    // 7. 使能 FIFO 并设置 BDU
    ret = ag_write_reg(REG_CTRL_REG8, 0x44); // BDU=1, IF_ADD_INC=1
    if (ret != ESP_OK) return ret;

    // 8. 禁用 FIFO (使用连续模式读取)
    ret = ag_write_reg(REG_FIFO_CTRL, 0x00);
    if (ret != ESP_OK) return ret;
    ret = ag_write_reg(REG_CTRL_REG9, 0x00);
    if (ret != ESP_OK) return ret;

    // ---- 调试: 回读关键寄存器验证配置 ----
    uint8_t reg_val;
    ag_read_reg(REG_CTRL_REG1_G, &reg_val);
    ESP_LOGI(TAG, "[REG_DBG] CTRL_REG1_G  = 0x%02X (期望 ODR|Scale)", reg_val);
    ag_read_reg(REG_CTRL_REG5_XL, &reg_val);
    ESP_LOGI(TAG, "[REG_DBG] CTRL_REG5_XL = 0x%02X (期望 0x38, 轴使能)", reg_val);
    ag_read_reg(REG_CTRL_REG6_XL, &reg_val);
    ESP_LOGI(TAG, "[REG_DBG] CTRL_REG6_XL = 0x%02X (期望 ODR|FS|BW)", reg_val);
    ag_read_reg(REG_CTRL_REG7_XL, &reg_val);
    ESP_LOGI(TAG, "[REG_DBG] CTRL_REG7_XL = 0x%02X (期望 0x00)", reg_val);
    ag_read_reg(REG_CTRL_REG8, &reg_val);
    ESP_LOGI(TAG, "[REG_DBG] CTRL_REG8    = 0x%02X (期望 0x44, BDU+IF_ADD_INC)", reg_val);
    ag_read_reg(REG_CTRL_REG4, &reg_val);
    ESP_LOGI(TAG, "[REG_DBG] CTRL_REG4    = 0x%02X (期望 0x38, 陀螺仪轴使能)", reg_val);

    ESP_LOGI(TAG, "LSM9DS1 初始化完成");
    return ESP_OK;
}

esp_err_t lsm9ds1_read_gyro(float *gx, float *gy, float *gz)
{
    uint8_t buf[6];
    esp_err_t ret = ag_read_regs(REG_OUT_X_L_G, buf, 6);
    if (ret != ESP_OK) return ret;

    // LSM9DS1 数据为小端序
    int16_t raw_gx = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t raw_gy = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_gz = (int16_t)((buf[5] << 8) | buf[4]);

    *gx = (float)raw_gx * s_gyro_res - s_gyro_bias[0];
    *gy = (float)raw_gy * s_gyro_res - s_gyro_bias[1];
    *gz = (float)raw_gz * s_gyro_res - s_gyro_bias[2];

    return ESP_OK;
}

// 调试: 每 N 次打印一次原始数据
static uint32_t s_accel_dbg_cnt = 0;
#define ACCEL_DBG_INTERVAL  400   // 200Hz × 2s

esp_err_t lsm9ds1_read_accel(float *ax, float *ay, float *az)
{
    uint8_t buf[6];
    esp_err_t ret = ag_read_regs(REG_OUT_X_L_XL, buf, 6);
    if (ret != ESP_OK) return ret;

    int16_t raw_ax = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t raw_ay = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_az = (int16_t)((buf[5] << 8) | buf[4]);

    // 调试: 打印原始 int16 值 + 原始字节
    if (++s_accel_dbg_cnt >= ACCEL_DBG_INTERVAL) {
        s_accel_dbg_cnt = 0;
        float mag_raw = sqrtf((float)raw_ax*raw_ax + (float)raw_ay*raw_ay + (float)raw_az*raw_az) * s_accel_res;
        ESP_LOGI(TAG, "[ACCEL_DBG] raw=[%+6d, %+6d, %+6d] "
                 "bytes=[%02X %02X, %02X %02X, %02X %02X] "
                 "res=%.6f bias=[%.4f,%.4f,%.4f] |raw|=%.3fg",
                 raw_ax, raw_ay, raw_az,
                 buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
                 s_accel_res,
                 s_accel_bias[0], s_accel_bias[1], s_accel_bias[2],
                 mag_raw);
    }

    *ax = (float)raw_ax * s_accel_res - s_accel_bias[0];
    *ay = (float)raw_ay * s_accel_res - s_accel_bias[1];
    *az = (float)raw_az * s_accel_res - s_accel_bias[2];

    return ESP_OK;
}

esp_err_t lsm9ds1_read_mag(float *mx, float *my, float *mz)
{
    uint8_t buf[6];
    esp_err_t ret = m_read_regs(REG_OUT_X_L_M, buf, 6);
    if (ret != ESP_OK) return ret;

    int16_t raw_mx = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t raw_my = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_mz = (int16_t)((buf[5] << 8) | buf[4]);

    *mx = (float)raw_mx * s_mag_res;
    *my = (float)raw_my * s_mag_res;
    *mz = (float)raw_mz * s_mag_res;

    return ESP_OK;
}

esp_err_t lsm9ds1_read_temp(float *temp)
{
    uint8_t buf[2];
    esp_err_t ret = ag_read_regs(REG_OUT_TEMP_L, buf, 2);
    if (ret != ESP_OK) return ret;

    int16_t raw_temp = (int16_t)((buf[1] << 8) | buf[0]);
    // LSM9DS1 温度: 偏移 25°C, 灵敏度 16 LSB/°C
    *temp = 25.0f + (float)raw_temp / 16.0f;

    return ESP_OK;
}

esp_err_t lsm9ds1_read_all(lsm9ds1_data_t *data)
{
    if (data == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t ret;

    // 读取陀螺仪
    ret = lsm9ds1_read_gyro(&data->gyro_x, &data->gyro_y, &data->gyro_z);
    if (ret != ESP_OK) return ret;

    // 读取加速度计
    ret = lsm9ds1_read_accel(&data->accel_x, &data->accel_y, &data->accel_z);
    if (ret != ESP_OK) return ret;

    // 读取磁力计
    ret = lsm9ds1_read_mag(&data->mag_x, &data->mag_y, &data->mag_z);
    if (ret != ESP_OK) return ret;

    // 读取温度
    ret = lsm9ds1_read_temp(&data->temperature);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t lsm9ds1_calibrate(int samples)
{
    if (samples <= 0) samples = 200;

    ESP_LOGI(TAG, "开始校准 (%d 样本), 请保持传感器静止...", samples);

    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    int valid = 0;

    for (int i = 0; i < samples; i++) {
        uint8_t buf[6];

        // 读取陀螺仪原始数据
        esp_err_t ret = ag_read_regs(REG_OUT_X_L_G, buf, 6);
        if (ret != ESP_OK) continue;

        int16_t raw_gx = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t raw_gy = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t raw_gz = (int16_t)((buf[5] << 8) | buf[4]);

        gx_sum += (float)raw_gx * s_gyro_res;
        gy_sum += (float)raw_gy * s_gyro_res;
        gz_sum += (float)raw_gz * s_gyro_res;

        // 读取加速度计原始数据
        ret = ag_read_regs(REG_OUT_X_L_XL, buf, 6);
        if (ret != ESP_OK) continue;

        int16_t raw_ax = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t raw_ay = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t raw_az = (int16_t)((buf[5] << 8) | buf[4]);

        ax_sum += (float)raw_ax * s_accel_res;
        ay_sum += (float)raw_ay * s_accel_res;
        az_sum += (float)raw_az * s_accel_res;

        valid++;
        vTaskDelay(pdMS_TO_TICKS(5)); // ~200Hz
    }

    if (valid < 10) {
        ESP_LOGE(TAG, "校准失败: 有效样本不足 (%d)", valid);
        return ESP_FAIL;
    }

    // 陀螺仪偏置 = 均值
    s_gyro_bias[0] = gx_sum / valid;
    s_gyro_bias[1] = gy_sum / valid;
    s_gyro_bias[2] = gz_sum / valid;

    // 加速度计偏置 (假设 Z 轴朝上 = 1g)
    s_accel_bias[0] = ax_sum / valid;
    s_accel_bias[1] = ay_sum / valid;
    s_accel_bias[2] = az_sum / valid - 1.0f; // 减去重力

    ESP_LOGI(TAG, "校准完成 (%d 样本)", valid);
    ESP_LOGI(TAG, "陀螺仪偏置: [%.4f, %.4f, %.4f] °/s",
             s_gyro_bias[0], s_gyro_bias[1], s_gyro_bias[2]);
    ESP_LOGI(TAG, "加速度计偏置: [%.4f, %.4f, %.4f] g",
             s_accel_bias[0], s_accel_bias[1], s_accel_bias[2]);

    return ESP_OK;
}
