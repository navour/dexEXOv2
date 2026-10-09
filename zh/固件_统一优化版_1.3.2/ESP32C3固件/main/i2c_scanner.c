/**
 * @file i2c_scanner.c
 * @brief I2C 地址扫描器 - 用于发现 LSM9DS1 的 AG 和 M 地址
 * 
 * LSM9DS1 可能的地址:
 *   AG (加速度计/陀螺仪): 0x6A (SDO_AG=LOW) 或 0x6B (SDO_AG=HIGH)
 *   M  (磁力计):          0x1C (SDO_M=LOW)  或 0x1E (SDO_M=HIGH)
 * 
 * WHO_AM_I 期望值:
 *   AG: 0x68
 *   M:  0x3D
 */

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "I2C_SCAN";

#define I2C_SCL_PIN     4
#define I2C_SDA_PIN     5
#define I2C_FREQ_HZ     100000   // 100kHz for scanning
#define I2C_TIMEOUT_MS  50

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " I2C 地址扫描器");
    ESP_LOGI(TAG, " SCL=GPIO%d  SDA=GPIO%d", I2C_SCL_PIN, I2C_SDA_PIN);
    ESP_LOGI(TAG, "========================================");

    // 初始化 I2C 总线
    i2c_master_bus_handle_t bus = NULL;
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = I2C_NUM_0,
        .scl_io_num = I2C_SCL_PIN,
        .sda_io_num = I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 总线初始化失败: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "开始扫描 I2C 总线 (0x01 ~ 0x7F)...\n");
    
    int found_count = 0;
    
    printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    for (int row = 0; row < 8; row++) {
        printf("%02X: ", row * 16);
        for (int col = 0; col < 16; col++) {
            uint8_t addr = row * 16 + col;
            if (addr < 0x03 || addr > 0x77) {
                printf("   ");
                continue;
            }

            // 尝试对该地址进行探测
            ret = i2c_master_probe(bus, addr, I2C_TIMEOUT_MS);
            if (ret == ESP_OK) {
                printf("%02X ", addr);
                found_count++;
            } else {
                printf("-- ");
            }
        }
        printf("\n");
    }

    printf("\n找到 %d 个设备\n\n", found_count);

    // 针对 LSM9DS1 可能的地址进行详细探测
    ESP_LOGI(TAG, "====== LSM9DS1 地址详细探测 ======");
    
    // 可能的 AG 地址
    uint8_t ag_addrs[] = {0x6A, 0x6B};
    // 可能的 M 地址
    uint8_t m_addrs[] = {0x1C, 0x1E};

    for (int i = 0; i < 2; i++) {
        uint8_t addr = ag_addrs[i];
        ret = i2c_master_probe(bus, addr, I2C_TIMEOUT_MS);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "AG 地址 0x%02X: 已找到!", addr);
            
            // 读取 WHO_AM_I (0x0F)
            i2c_master_dev_handle_t dev = NULL;
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addr,
                .scl_speed_hz = I2C_FREQ_HZ,
            };
            if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) == ESP_OK) {
                uint8_t reg = 0x0F;
                uint8_t who = 0;
                if (i2c_master_transmit_receive(dev, &reg, 1, &who, 1, I2C_TIMEOUT_MS) == ESP_OK) {
                    ESP_LOGI(TAG, "  WHO_AM_I = 0x%02X (期望 0x68 for LSM9DS1 AG)", who);
                }
                i2c_master_bus_rm_device(dev);
            }
        } else {
            ESP_LOGW(TAG, "AG 地址 0x%02X: 未找到", addr);
        }
    }

    for (int i = 0; i < 2; i++) {
        uint8_t addr = m_addrs[i];
        ret = i2c_master_probe(bus, addr, I2C_TIMEOUT_MS);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "M 地址 0x%02X: 已找到!", addr);
            
            // 读取 WHO_AM_I (0x0F)
            i2c_master_dev_handle_t dev = NULL;
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addr,
                .scl_speed_hz = I2C_FREQ_HZ,
            };
            if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) == ESP_OK) {
                uint8_t reg = 0x0F;
                uint8_t who = 0;
                if (i2c_master_transmit_receive(dev, &reg, 1, &who, 1, I2C_TIMEOUT_MS) == ESP_OK) {
                    ESP_LOGI(TAG, "  WHO_AM_I = 0x%02X (期望 0x3D for LSM9DS1 M)", who);
                }
                i2c_master_bus_rm_device(dev);
            }
        } else {
            ESP_LOGW(TAG, "M 地址 0x%02X: 未找到", addr);
        }
    }

    // 额外: 也检查 MPU9250 地址 (0x68/0x69) 以排除干扰
    for (uint8_t addr = 0x68; addr <= 0x69; addr++) {
        ret = i2c_master_probe(bus, addr, I2C_TIMEOUT_MS);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "MPU 地址 0x%02X: 已找到!", addr);
            i2c_master_dev_handle_t dev = NULL;
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addr,
                .scl_speed_hz = I2C_FREQ_HZ,
            };
            if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) == ESP_OK) {
                uint8_t reg = 0x75;
                uint8_t who = 0;
                if (i2c_master_transmit_receive(dev, &reg, 1, &who, 1, I2C_TIMEOUT_MS) == ESP_OK) {
                    ESP_LOGI(TAG, "  WHO_AM_I (0x75) = 0x%02X", who);
                }
                // Also read 0x0F
                reg = 0x0F;
                who = 0;
                if (i2c_master_transmit_receive(dev, &reg, 1, &who, 1, I2C_TIMEOUT_MS) == ESP_OK) {
                    ESP_LOGI(TAG, "  WHO_AM_I (0x0F) = 0x%02X", who);
                }
                i2c_master_bus_rm_device(dev);
            }
        }
    }

    ESP_LOGI(TAG, "====== 扫描完成 ======");
    
    // 清理
    i2c_del_master_bus(bus);
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
